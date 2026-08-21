/*
 * RIGTest.cpp
 *
 *  Created on: Aug 21, 2026
 *      Author: Mikhail Kirilin
 */

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <networkit/Globals.hpp>
#include <networkit/auxiliary/Random.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/edgescores/ChibaNishizekiTriangleEdgeScore.hpp>
#include <networkit/generators/ErdosRenyiGenerator.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/io/METISGraphReader.hpp>
#include <networkit/isomorphism/RI.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

#include "SubgraphIsomorphismTestUtils.hpp"
#include "../RIImpl.hpp"
#include "../SearchGraph.hpp"

namespace NetworKit {

using IsomorphismDetails::RIImpl;
using IsomorphismDetails::SearchGraph;
using IsomorphismTest::Case;
using IsomorphismTest::Match;
using Semantics = SubgraphIsomorphism::Semantics;

namespace {

/// The two snapshots RI::run() builds, so a test can drive RIImpl exactly the way the driver does.
struct Snapshot {
    SearchGraph pattern;
    SearchGraph target;

    explicit Snapshot(const Case &testCase)
        : pattern(testCase.pattern, /* buildMatrix = */ true, testCase.patternEdgeLabels),
          target(testCase.target, /* buildMatrix = */ false, testCase.targetEdgeLabels) {}

    /// Whether RI::run() would refuse this case rather than search it - see
    /// SearchGraph::collapsedLabelledEdges().
    bool refused() const {
        return pattern.collapsedLabelledEdges() || target.collapsedLabelledEdges();
    }
};

/// Where @a pu sits in the matching order, or `none` if it is not in it.
index positionOf(const RIImpl::Ordering &ordering, node pu) {
    const auto found = std::find(ordering.order.begin(), ordering.order.end(), pu);
    return found == ordering.order.end() ? none
                                         : static_cast<index>(found - ordering.order.begin());
}

/// A label per node id, with `none` mixed in so both sides carry wildcards.
std::vector<index> randomLabels(count upperNodeIdBound) {
    std::vector<index> labels;
    labels.reserve(upperNodeIdBound);
    for (count i = 0; i < upperNodeIdBound; ++i) {
        const index drawn = static_cast<index>(Aux::Random::integer(0, 2));
        labels.push_back(drawn == 2 ? none : drawn);
    }
    return labels;
}

} // namespace

/**
 * Everything here is parameterised over the variant, so neither is tested less than the other -
 * including the tests whose subject is variant-independent, where running twice is itself the
 * evidence that RI-Ds really does order exactly as plain RI does.
 */
class RIGTest : public testing::TestWithParam<RI::Variant> {};

INSTANTIATE_TEST_SUITE_P(Variants, RIGTest, testing::Values(RI::Variant::RI, RI::Variant::RI_DS));

// -------------------------------------------------------------------------------------------
// The three assertions the shared harness offers
// -------------------------------------------------------------------------------------------

TEST_P(RIGTest, testMatchesReference) {
    const RI::Variant variant = GetParam();
    const auto make = [variant](const Graph &pattern, const Graph &target, Semantics semantics,
                                count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new RI(pattern, target, variant, semantics, maxMatches));
    };

    IsomorphismTest::expectMatchesReference(make);
}

TEST_P(RIGTest, testRespectsMatchCap) {
    const RI::Variant variant = GetParam();
    const auto make = [variant](const Graph &pattern, const Graph &target, Semantics semantics,
                                count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new RI(pattern, target, variant, semantics, maxMatches));
    };

    IsomorphismTest::expectRespectsMatchCap(make);
}

TEST_P(RIGTest, testCallbackFormsAgree) {
    const RI::Variant variant = GetParam();
    const auto make = [variant](const Graph &pattern, const Graph &target, Semantics semantics,
                                count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new RI(pattern, target, variant, semantics, maxMatches));
    };

    IsomorphismTest::expectCallbackFormsAgree(make);
}

// -------------------------------------------------------------------------------------------
// The ordering, which agreement with the reference judges only by luck on a 4-node corpus
// -------------------------------------------------------------------------------------------

/**
 * Structural invariants of the matching order, checked directly rather than through a search.
 *
 * This is what catches the whole "the ordering is subtly wrong" class: a removed id in the order,
 * a node listed twice, a parent pointing forwards, a parent that is not actually adjacent, or a
 * `none` parent at a position that does have an earlier neighbour.
 */
TEST_P(RIGTest, testOrderingInvariants) {
    for (const Case &testCase : IsomorphismTest::standardCases()) {
        const Snapshot snapshot(testCase);
        const RIImpl::Ordering ordering =
            RIImpl::computeOrdering(snapshot.pattern, snapshot.target, testCase.patternLabels,
                                    testCase.targetLabels, GetParam());

        ASSERT_EQ(ordering.order.size(), snapshot.pattern.numberOfNodes())
            << "case: " << testCase.name;
        ASSERT_EQ(ordering.parent.size(), ordering.order.size()) << "case: " << testCase.name;

        std::vector<bool> seen(snapshot.pattern.upperNodeIdBound(), false);
        for (const node pu : ordering.order) {
            ASSERT_LT(pu, snapshot.pattern.upperNodeIdBound()) << "case: " << testCase.name;
            EXPECT_TRUE(snapshot.pattern.hasNode(pu))
                << "case: " << testCase.name << " - ordered an id that is not a node";
            EXPECT_FALSE(seen[pu]) << "case: " << testCase.name << " - node ordered twice";
            seen[pu] = true;
        }

        if (!ordering.parent.empty())
            EXPECT_EQ(ordering.parent[0], none) << "case: " << testCase.name;

        for (index i = 0; i < ordering.order.size(); ++i) {
            const node pu = ordering.order[i];

            // A parent must be an *earlier* position that really is adjacent, in either direction.
            const index parentPos = ordering.parent[i];
            if (parentPos != none) {
                ASSERT_LT(parentPos, i) << "case: " << testCase.name;
                const node pp = ordering.order[parentPos];
                EXPECT_TRUE(snapshot.pattern.hasEdge(pp, pu) || snapshot.pattern.hasEdge(pu, pp))
                    << "case: " << testCase.name << " - parent is not adjacent";
            }

            // And `none` means exactly "this position starts a component", never "I gave up".
            bool anyEarlierAdjacent = false;
            for (index j = 0; j < i && !anyEarlierAdjacent; ++j) {
                const node earlier = ordering.order[j];
                anyEarlierAdjacent =
                    snapshot.pattern.hasEdge(earlier, pu) || snapshot.pattern.hasEdge(pu, earlier);
            }
            EXPECT_EQ(parentPos == none, !anyEarlierAdjacent)
                << "case: " << testCase.name << " at position " << i;
        }
    }
}

/**
 * Two orders small enough to work out by hand, plus the pair that proves the score is three-level.
 *
 * The eight-node graph is built in the shape of the paper's worked example. At the step that
 * decides between nodes 5 and 0 the two tie on the first term (one arc into the order each) and on
 * the third (one untouched neighbour each), and differ only on the middle one: 5 reaches the
 * already-ordered node 1 through the still-unordered node 2, while 0 reaches nothing. So 5 coming
 * before 0 is the only direct evidence that the middle term is computed at all - a two-level score
 * would order them the other way round, by node id.
 */
TEST_P(RIGTest, testOrderingHandTraced) {
    const RI::Variant variant = GetParam();
    const auto orderingOf = [variant](const Graph &pattern) {
        const SearchGraph patternGraph(pattern, /* buildMatrix = */ true);
        // The target is never consulted, so it can be anything; passing the pattern says so.
        const SearchGraph targetGraph(pattern, /* buildMatrix = */ false);
        return RIImpl::computeOrdering(patternGraph, targetGraph, {}, {}, variant);
    };

    // A path 0-1-2. The middle node is the only one of degree two, so it goes first, and both ends
    // then hang off position 0.
    const RIImpl::Ordering path = orderingOf(IsomorphismTest::graphOf(3, {{0, 1}, {1, 2}}));
    EXPECT_EQ(path.order, (std::vector<node>{1, 0, 2}));
    EXPECT_EQ(path.parent, (std::vector<index>{none, 0, 0}));

    // Two components. Position 2 restarts, which is what the `none` parent is for.
    const RIImpl::Ordering split = orderingOf(IsomorphismTest::graphOf(4, {{0, 1}, {2, 3}}));
    EXPECT_EQ(split.order, (std::vector<node>{0, 1, 2, 3}));
    EXPECT_EQ(split.parent, (std::vector<index>{none, 0, none, 2}));

    const RIImpl::Ordering worked = orderingOf(IsomorphismTest::graphOf(
        8, {{4, 1}, {4, 5}, {4, 0}, {4, 3}, {1, 2}, {1, 6}, {5, 2}, {5, 7}, {0, 7}, {3, 6}}));
    EXPECT_EQ(worked.order[0], 4u) << "the first node must be the unique maximum-degree node";
    EXPECT_LT(positionOf(worked, 5), positionOf(worked, 0))
        << "node 5 beats node 0 only on the two-hop term";
}

// -------------------------------------------------------------------------------------------
// RI-Ds, on a target big enough for a domain to actually remove something
// -------------------------------------------------------------------------------------------

/**
 * The corpus is far too small for a domain to prune, so RI-Ds is effectively untested by it.
 *
 * Domains are pure pruning, so any divergence between the variants is an unsound domain - the
 * refinement sweep removed a target node a real match needs. The absolute count is pinned the way
 * ReferenceMatcherGTest does it, against ChibaNishizeki's per-edge triangle counts, which is an
 * independent check from a part of NetworKit that has nothing to do with this module.
 */
TEST_P(RIGTest, testVariantsAgreeOnKarate) {
    METISGraphReader reader;
    Graph karate = reader.read("input/karate.graph");
    karate.indexEdges();

    ChibaNishizekiTriangleEdgeScore triangleScore(karate);
    triangleScore.run();
    const std::vector<count> perEdge = triangleScore.scores();
    const count edgeSum = std::accumulate(perEdge.begin(), perEdge.end(), count{0});
    ASSERT_GT(edgeSum, 0u) << "karate should contain triangles";

    // Each triangle is counted on three edges and found once per way of labelling its three nodes,
    // so the number of matches is 6 * (edgeSum / 3) = 2 * edgeSum.
    const count expected = 2 * edgeSum;

    const Graph pattern = IsomorphismTest::graphOf(3, {{0, 1}, {1, 2}, {2, 0}});

    // The domains really do remove something here, which is the whole reason this case exists:
    // karate has nodes of degree one, and none of those can host a triangle node of degree two.
    // On the 4-node corpus above no domain ever loses an entry.
    count couldHostATriangleNode = 0;
    karate.forNodes([&](node v) {
        if (karate.degree(v) >= 2)
            ++couldHostATriangleNode;
    });
    ASSERT_LT(couldHostATriangleNode, karate.numberOfNodes());

    RI plain(pattern, karate, RI::Variant::RI, Semantics::MONOMORPHISM, 0);
    plain.run();
    std::vector<Match> plainMatches = plain.getMatches();
    IsomorphismTest::sortMatches(plainMatches);

    RI withDomains(pattern, karate, RI::Variant::RI_DS, Semantics::MONOMORPHISM, 0);
    withDomains.run();
    std::vector<Match> domainMatches = withDomains.getMatches();
    IsomorphismTest::sortMatches(domainMatches);

    EXPECT_EQ(domainMatches, plainMatches) << "RI-Ds is pure pruning and must not change the "
                                              "match set";
    EXPECT_EQ(plainMatches.size(), expected);
    EXPECT_EQ(expected, 270u);
}

// -------------------------------------------------------------------------------------------
// expand(), which nothing else exercises while ParallelRIImpl is stubbed
// -------------------------------------------------------------------------------------------

/**
 * Drive RIImpl a level at a time, in the shape ParallelRIImpl::workerLoop() will use, and assert
 * the match set is the recursion's.
 *
 * Nothing is carried on the C++ call stack here: every state on the pending list has to be
 * self-contained, which is exactly the property a stolen state needs. A disagreement means
 * something the recursion keeps on its own stack was never written into the State.
 */
TEST_P(RIGTest, testExpandAgreesWithRun) {
    const RI::Variant variant = GetParam();

    for (const Case &testCase : IsomorphismTest::standardCases()) {
        const Snapshot snapshot(testCase);
        if (snapshot.refused())
            continue;

        Aux::SignalHandler handler;
        const RIImpl::Ordering ordering =
            RIImpl::computeOrdering(snapshot.pattern, snapshot.target, testCase.patternLabels,
                                    testCase.targetLabels, variant);

        std::vector<Match> viaRun;
        RIImpl(snapshot.pattern, snapshot.target, testCase.patternLabels, testCase.targetLabels,
               ordering, testCase.semantics, variant, handler,
               [&viaRun](const std::vector<node> &match) {
                   viaRun.push_back(match);
                   return true;
               })
            .run();

        std::vector<Match> viaExpand;
        RIImpl expander(snapshot.pattern, snapshot.target, testCase.patternLabels,
                        testCase.targetLabels, ordering, testCase.semantics, variant, handler,
                        [&viaExpand](const std::vector<node> &match) {
                            viaExpand.push_back(match);
                            return true;
                        });

        std::vector<RIImpl::State> pending(1);
        pending.front().mapping.assign(ordering.order.size(), none);
        while (!pending.empty()) {
            RIImpl::State state = std::move(pending.back());
            pending.pop_back();

            std::vector<RIImpl::State> children;
            // No cap is set, so the only reason expand() could ask to stop is a bug.
            EXPECT_TRUE(expander.expand(state, children)) << "case: " << testCase.name;

            for (RIImpl::State &child : children)
                pending.push_back(std::move(child));
        }

        IsomorphismTest::sortMatches(viaRun);
        IsomorphismTest::sortMatches(viaExpand);
        EXPECT_EQ(viaExpand, viaRun) << "case: " << testCase.name;
    }
}

// -------------------------------------------------------------------------------------------
// Randomised cross-check, for the ordering or parent bug the hand-built corpus happens to miss
// -------------------------------------------------------------------------------------------

TEST_P(RIGTest, testMatchesReferenceOnRandomGraphs) {
    const RI::Variant variant = GetParam();

    // The reference enumerates every injective mapping, so the sizes here are what keeps this in
    // the millisecond range: 6 target nodes to the power of 4 pattern nodes.
    constexpr count patternNodes = 4;
    constexpr count targetNodes = 6;
    constexpr int trials = 8;

    Aux::Random::setSeed(1701, false);

    for (const bool directed : {false, true}) {
        for (const Semantics semantics : {Semantics::MONOMORPHISM, Semantics::INDUCED}) {
            for (const bool labelled : {false, true}) {
                for (int trial = 0; trial < trials; ++trial) {
                    const Graph pattern =
                        ErdosRenyiGenerator(patternNodes, 0.5, directed).generate();
                    const Graph target = ErdosRenyiGenerator(targetNodes, 0.4, directed).generate();

                    std::vector<index> patternLabels;
                    std::vector<index> targetLabels;
                    if (labelled) {
                        patternLabels = randomLabels(pattern.upperNodeIdBound());
                        targetLabels = randomLabels(target.upperNodeIdBound());
                    }

                    std::vector<Match> expected = IsomorphismTest::referenceMatches(
                        pattern, target, semantics, patternLabels, targetLabels);
                    IsomorphismTest::sortMatches(expected);

                    RI algo(pattern, target, variant, semantics, 0);
                    if (labelled)
                        algo.setLabels(patternLabels, targetLabels);
                    algo.run();

                    std::vector<Match> actual = algo.getMatches();
                    IsomorphismTest::sortMatches(actual);

                    EXPECT_EQ(actual, expected)
                        << "directed: " << directed
                        << ", induced: " << (semantics == Semantics::INDUCED)
                        << ", labelled: " << labelled << ", trial: " << trial;
                }
            }
        }
    }
}

} // namespace NetworKit
