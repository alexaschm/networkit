/*
 * ParallelRIGTest.cpp
 *
 *  Created on: Aug 24, 2026
 *      Author: Mikhail Kirilin
 */

#include <algorithm>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <networkit/GlobalState.hpp>
#include <networkit/Globals.hpp>
#include <networkit/auxiliary/Parallelism.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/io/METISGraphReader.hpp>
#include <networkit/isomorphism/ParallelRI.hpp>
#include <networkit/isomorphism/RI.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

#include "SubgraphIsomorphismTestUtils.hpp"

namespace NetworKit {

using IsomorphismTest::graphOf;
using IsomorphismTest::Match;
using IsomorphismTest::sortMatches;
using Semantics = SubgraphIsomorphism::Semantics;

namespace {

/// Builds a ParallelRI of the given variant in the shape the shared harness wants.
template <typename Variant>
auto parallelFactory(Variant variant) {
    return [variant](const Graph &pattern, const Graph &target, Semantics semantics,
                     count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new ParallelRI(pattern, target, variant, semantics, maxMatches));
    };
}

/// The graph the agreement tests run on. Small enough to read in milliseconds, real enough that
/// the search tree is as lopsided as the work stealing is meant to cope with - which the 4-node
/// corpus in the shared harness is far too regular to be.
Graph karate() {
    METISGraphReader reader;
    return reader.read("input/karate.graph");
}

/// A path of @a n nodes. Lengthening it is the cheapest way to buy a deeper and much wider search
/// tree without changing the target: in karate a 5-path occurs 22 064 times and a 7-path 326 328.
Graph path(count n) {
    Graph G(n);
    for (node u = 0; u + 1 < n; ++u)
        G.addEdge(u, u + 1);
    return G;
}

Graph triangle() {
    return graphOf(3, {{0, 1}, {1, 2}, {2, 0}});
}

/// Sorted matches from the sequential search, which is the answer ParallelRI has to reproduce.
std::vector<Match> sequentialMatches(const Graph &pattern, const Graph &target, Semantics semantics,
                                     RI::Variant variant) {
    RI algo(pattern, target, variant, semantics, 0);
    algo.run();
    std::vector<Match> matches = algo.getMatches();
    sortMatches(matches);
    return matches;
}

/// Sorted matches from the parallel search, at whatever the current thread setting is.
std::vector<Match> parallelMatches(const Graph &pattern, const Graph &target, Semantics semantics,
                                   RI::Variant variant) {
    ParallelRI algo(pattern, target, variant, semantics, 0);
    algo.run();
    std::vector<Match> matches = algo.getMatches();
    sortMatches(matches);
    return matches;
}

/// How many matches a search finds, without keeping any of them.
///
/// Counting rather than storing is what makes an enumeration big enough to provoke stealing also
/// cheap enough for a unit test: a few hundred thousand matches cost nothing to count and tens of
/// megabytes, plus a sort, to compare one by one.
template <typename Algo>
count countOnly(Algo &&algo) {
    algo.setStoreMatches(false);
    algo.run();
    return algo.numberOfMatches();
}

} // namespace

/**
 * Parameterised over the variant, exactly as RIGTest is, so RI-Ds gets the same parallel exercise
 * plain RI does - including the per-worker domain build, which only happens on this path.
 *
 * A hung test here is a termination bug rather than a slow test: the failure mode of the token
 * ring is a worker that never notices the search is over, not a wrong answer.
 */
class ParallelRIGTest : public testing::TestWithParam<RI::Variant> {

protected:
    void SetUp() override { threadsBefore = Aux::getMaxNumberOfThreads(); }

    /// Both of these are process-*global*. A failing ASSERT_* returns from a test early, so
    /// restoring them here rather than at the end of each test is what keeps one failure from
    /// quietly changing how every later test in the binary behaves.
    void TearDown() override {
        GlobalState::setReceivedSIGINT(false);
        Aux::setNumberOfThreads(threadsBefore);
    }

private:
    int threadsBefore = 1;
};

INSTANTIATE_TEST_SUITE_P(Variants, ParallelRIGTest,
                         testing::Values(RI::Variant::RI, RI::Variant::RI_DS));

// -------------------------------------------------------------------------------------------
// The three assertions the shared harness offers
// -------------------------------------------------------------------------------------------

TEST_P(ParallelRIGTest, testMatchesReference) {
    IsomorphismTest::expectMatchesReference(parallelFactory(GetParam()));
}

TEST_P(ParallelRIGTest, testRespectsMatchCap) {
    IsomorphismTest::expectRespectsMatchCap(parallelFactory(GetParam()));
}

TEST_P(ParallelRIGTest, testCallbackFormsAgree) {
    // The first real exercise of SubgraphIsomorphism::invokeCallback()'s mutex: a serial
    // MatchCallback must see every match exactly once even though several workers produce them.
    IsomorphismTest::expectCallbackFormsAgree(parallelFactory(GetParam()));
}

// -------------------------------------------------------------------------------------------
// Agreement with the sequential search, on a graph big enough for the workers to overlap
// -------------------------------------------------------------------------------------------

/**
 * The strongest correctness assertion in the file: the same enumeration, run both ways, compared
 * element by element rather than merely counted.
 *
 * A count alone would let a lost match and a duplicated one cancel each other out, which is
 * exactly the shape a stealing bug takes.
 */
TEST_P(ParallelRIGTest, testAgreesWithSequentialRI) {
    const Graph target = karate();
    const Graph pattern = path(5);

    const std::vector<Match> expected =
        sequentialMatches(pattern, target, Semantics::MONOMORPHISM, GetParam());
    ASSERT_EQ(expected.size(), 22064u) << "a change here would quietly weaken every case below";

    EXPECT_EQ(parallelMatches(pattern, target, Semantics::MONOMORPHISM, GetParam()), expected);
}

/**
 * The property that catches lost and duplicated work: the answer must not depend on how many
 * workers produced it.
 *
 * One worker is not a special case in the implementation - it walks the queues, the coalescing and
 * the token ring like any other count - so this really does compare the same machinery at four
 * different degrees of contention.
 */
TEST_P(ParallelRIGTest, testAnswerDoesNotDependOnWorkerCount) {
    const Graph target = karate();
    const Graph pattern = path(5);

    const std::vector<Match> expected =
        sequentialMatches(pattern, target, Semantics::MONOMORPHISM, GetParam());
    ASSERT_FALSE(expected.empty());

    for (const int workers : {1, 2, 4, 8, 16}) {
        Aux::setNumberOfThreads(workers);

        ParallelRI algo(pattern, target, GetParam(), Semantics::MONOMORPHISM, 0);
        ASSERT_EQ(algo.numberOfWorkers(), static_cast<count>(workers))
            << "numberOfWorkers() has to follow Aux::setNumberOfThreads()";
        algo.run();

        std::vector<Match> actual = algo.getMatches();
        sortMatches(actual);
        EXPECT_EQ(actual, expected) << "workers: " << workers;
        EXPECT_EQ(algo.numberOfMatches(), expected.size()) << "workers: " << workers;
    }
}

/**
 * The same question as above, on a search long enough that the workers genuinely have to steal
 * from each other, and answered by counting so that it stays cheap.
 *
 * A 7-path in karate has 326 328 occurrences over a search tree of several million states, which
 * is far past the point where every worker still has its own root to chew on. This is therefore
 * the case that actually puts load on the published queues and the token ring - and the one whose
 * count drifts if a steal ever loses or duplicates a state. Run it repeatedly
 * (`--gtest_repeat=50`): a race that shows up once in ten runs is exactly the expected failure
 * mode here.
 */
TEST_P(ParallelRIGTest, testLongEnumerationAgreesAtEveryWorkerCount) {
    const Graph target = karate();
    const Graph pattern = path(7);

    RI reference(pattern, target, GetParam(), Semantics::MONOMORPHISM, 0);
    const count expected = countOnly(reference);
    ASSERT_EQ(expected, 326328u);

    for (const int workers : {1, 2, 4, 8, 16}) {
        Aux::setNumberOfThreads(workers);
        ParallelRI algo(pattern, target, GetParam(), Semantics::MONOMORPHISM, 0);
        EXPECT_EQ(countOnly(algo), expected) << "workers: " << workers;
    }
}

/**
 * Duplication and loss told apart, which comparing sets cannot do.
 *
 * The per-worker slots are merged into one sequence and sorted, so the comparison is between
 * multisets: a match reported twice makes the sequence longer, a match lost makes it shorter, and
 * either way the two sequences stop being equal.
 */
TEST_P(ParallelRIGTest, testEveryMatchIsReportedExactlyOnce) {
    const Graph target = karate();
    const Graph pattern = triangle();

    const std::vector<Match> expected =
        sequentialMatches(pattern, target, Semantics::MONOMORPHISM, GetParam());
    ASSERT_EQ(expected.size(), 270u) << "karate's triangle count is what pins this case";

    ParallelRI algo(pattern, target, GetParam(), Semantics::MONOMORPHISM, 0);
    std::vector<std::vector<Match>> perWorker(algo.numberOfWorkers());
    algo.setCallback([&](index tid, const Match &match) {
        ASSERT_LT(tid, perWorker.size());
        perWorker[tid].push_back(match);
    });
    algo.run();

    std::vector<Match> collected;
    for (const std::vector<Match> &slot : perWorker)
        collected.insert(collected.end(), slot.begin(), slot.end());

    sortMatches(collected);
    EXPECT_EQ(collected, expected);
    EXPECT_EQ(algo.numberOfMatches(), expected.size());
}

/**
 * A pattern in two components, so some position has no parent and draws candidates from the whole
 * target rather than from one neighbourhood.
 *
 * That parentless path is also where RI-Ds uses a domain unconditionally, so this is the case in
 * which the two variants do the most different things while having to agree.
 */
TEST_P(ParallelRIGTest, testDisconnectedPatternIsSearchedInParallel) {
    const Graph target = karate();
    const Graph pattern = graphOf(4, {{0, 1}, {2, 3}});

    const std::vector<Match> expected =
        sequentialMatches(pattern, target, Semantics::MONOMORPHISM, GetParam());
    ASSERT_GT(expected.size(), 1000u);

    EXPECT_EQ(parallelMatches(pattern, target, Semantics::MONOMORPHISM, GetParam()), expected);
}

// -------------------------------------------------------------------------------------------
// The inputs where the pool has to stop without ever having had anything to do
// -------------------------------------------------------------------------------------------

TEST_P(ParallelRIGTest, testDegenerateInputs) {
    // Every graph here is a named local, never a temporary: SubgraphIsomorphism holds both graphs
    // by pointer so that a caller can mutate them between runs, which makes `ParallelRI algo(f(),
    // g(), ...)` a dangling read the moment the constructor returns.
    const Graph k4 = graphOf(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});
    const Graph nothing(0);
    const Graph oneEdge = graphOf(2, {{0, 1}});
    const Graph path3 = graphOf(3, {{0, 1}, {1, 2}});
    const Graph tri = triangle();

    // An empty pattern has exactly one match - the empty mapping - and it is reported during
    // seeding, before any worker has a queue to drain. No bail-out may swallow it.
    ParallelRI empty(nothing, k4, GetParam(), Semantics::INDUCED, 0);
    empty.run();
    ASSERT_TRUE(empty.hasFinished());
    EXPECT_EQ(empty.numberOfMatches(), 1u);
    ASSERT_EQ(empty.getMatches().size(), 1u);
    EXPECT_TRUE(empty.getMatches().front().empty());

    // More pattern nodes than target nodes: nothing can match, and run() has to say so through
    // patternCannotFit() rather than by starting a pool with an empty seed set.
    ParallelRI tooBig(k4, oneEdge, GetParam(), Semantics::INDUCED, 0);
    tooBig.run();
    ASSERT_TRUE(tooBig.hasFinished());
    EXPECT_EQ(tooBig.numberOfMatches(), 0u);
    EXPECT_FALSE(tooBig.hasMatch());

    // The other way to find nothing: the shape passes every cheap bail-out, so the pool really
    // runs and every branch dies. This is the one that hangs if termination is wrong.
    ParallelRI noMatch(tri, path3, GetParam(), Semantics::MONOMORPHISM, 0);
    noMatch.run();
    ASSERT_TRUE(noMatch.hasFinished());
    EXPECT_EQ(noMatch.numberOfMatches(), 0u);

    // A cap of zero means "no limit", not "no matches" - the same reading RI gives it.
    ParallelRI uncapped(tri, k4, GetParam(), Semantics::MONOMORPHISM, 0);
    uncapped.run();
    EXPECT_EQ(uncapped.numberOfMatches(), 24u);
}

// -------------------------------------------------------------------------------------------
// Interruption, which has to unwind every worker and then throw exactly once
// -------------------------------------------------------------------------------------------

/**
 * CTRL+C partway through a long enumeration.
 *
 * The workers may only poll the non-throwing isRunning(), because an exception leaving an OpenMP
 * structured block is undefined behaviour; the InterruptException comes from the single
 * assureRunning() after the join. What this test really proves is that the pool *unwinds* - if any
 * worker kept spinning in the token ring, the region would never join and the binary would hang
 * here rather than fail.
 */
TEST_P(ParallelRIGTest, testInterruptStopsEveryWorker) {
    const Graph target = karate();
    const Graph pattern = path(5);

    ParallelRI algo(pattern, target, GetParam(), Semantics::MONOMORPHISM, 0);

    // Several workers hit this at once, so the counter has to be atomic; a user's callback would
    // not need one unless it kept state of its own.
    std::atomic<count> delivered{0};
    algo.setCallback([&](index, const Match &) {
        if (delivered.fetch_add(1) + 1 == 5)
            GlobalState::setReceivedSIGINT(true);
    });

    EXPECT_THROW(algo.run(), Aux::SignalHandler::InterruptException);
    GlobalState::setReceivedSIGINT(false);

    EXPECT_FALSE(algo.hasFinished()) << "an interrupted run must not count as finished";
    EXPECT_THROW(algo.numberOfMatches(), std::runtime_error);
    EXPECT_GE(delivered.load(), 5u) << "matches already handed over cannot be taken back";

    // And nothing is poisoned: a clean second search gives the whole answer.
    const std::vector<Match> expected =
        sequentialMatches(pattern, target, Semantics::MONOMORPHISM, GetParam());
    EXPECT_EQ(parallelMatches(pattern, target, Semantics::MONOMORPHISM, GetParam()), expected);
}

// -------------------------------------------------------------------------------------------
// The contract numberOfWorkers() exists for
// -------------------------------------------------------------------------------------------

/**
 * A worker id handed to a ParallelMatchCallback must be a valid index into a vector sized by
 * numberOfWorkers().
 *
 * That is the entire reason the accessor is public, and the reason the number of workers is asked
 * for once inside run() rather than re-read per match: a caller that sized its accumulator from it
 * would otherwise be writing out of bounds.
 */
TEST_P(ParallelRIGTest, testWorkerIdsStayBelowNumberOfWorkers) {
    Aux::setNumberOfThreads(4);

    const Graph target = karate();
    const Graph pattern = triangle();

    ParallelRI algo(pattern, target, GetParam(), Semantics::MONOMORPHISM, 0);
    const count workers = algo.numberOfWorkers();
    ASSERT_EQ(workers, 4u);

    std::vector<std::vector<Match>> perWorker(workers);
    algo.setCallback([&](index tid, const Match &match) {
        ASSERT_LT(tid, perWorker.size()) << "a worker id outside [0, numberOfWorkers()) would make "
                                            "every documented per-worker accumulator unsafe";
        perWorker[tid].push_back(match);
    });
    algo.run();

    count total = 0;
    for (const std::vector<Match> &slot : perWorker)
        total += slot.size();

    EXPECT_EQ(total, algo.numberOfMatches())
        << "every match has to arrive in exactly one worker's slot";
}

} // namespace NetworKit
