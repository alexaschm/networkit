#ifndef NETWORKIT_CPP_ISOMORPHISM_TEST_SUBGRAPH_ISOMORPHISM_TEST_UTILS_HPP_
#define NETWORKIT_CPP_ISOMORPHISM_TEST_SUBGRAPH_ISOMORPHISM_TEST_UTILS_HPP_

// Shared test fixture for the isomorphism module. Header-only because networkit_add_test compiles
// exactly one .cpp per test target, so this is the only way four separate test files can share a
// corpus and a reference implementation.

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <networkit/Globals.hpp>
#include <networkit/auxiliary/Parallelism.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {
namespace IsomorphismTest {

using Semantics = SubgraphIsomorphism::Semantics;

/// One match, in the shape the module defines: indexed by pattern node, sized to the pattern's
/// upperNodeIdBound(), holding `none` at ids that are not nodes.
using Match = std::vector<node>;

// ---------------------------------------------------------------------------------------------
// The reference matcher
//
// Everything downstream trusts this, so it is written to be *obviously* right rather than fast.
// Two rules it follows deliberately:
//
//   1. It never touches SearchGraph. A reference built on the code under test inherits that
//      code's bugs and proves nothing. It works straight off Graph.
//   2. Generating candidate mappings and judging them are separate functions. The generator knows
//      only about injectivity; the judge re-derives every rule from scratch on a finished mapping.
//      Neither half is clever enough to be wrong in an interesting way.
//
// Plain Graph::hasEdge is safe here even on inputs with self-loops and parallel edges: a mapping
// is injective over distinct pattern nodes, so every pair it asks about has u != v, and hasEdge
// answers a boolean question that multiplicity cannot change.
// ---------------------------------------------------------------------------------------------

/// Whether pattern node @a pu may sit on target node @a tv as far as labels are concerned.
/// Unlabelled searches accept everything; `none` is a wildcard on either side.
inline bool labelsCompatible(const std::vector<index> &patternLabels,
                             const std::vector<index> &targetLabels, node pu, node tv) {
    if (patternLabels.empty())
        return true;

    const index patternLabel = patternLabels[pu];
    const index targetLabel = targetLabels[tv];
    return patternLabel == none || targetLabel == none || patternLabel == targetLabel;
}

/// Whether one complete mapping is a match. Judged from scratch, without reference to how the
/// mapping was produced, so this doubles as the check that an *algorithm's* output is well-formed.
inline bool isValidMatch(const Graph &pattern, const Graph &target, Semantics semantics,
                         const std::vector<index> &patternLabels,
                         const std::vector<index> &targetLabels, const Match &match) {
    const count pz = pattern.upperNodeIdBound();

    if (match.size() != pz)
        return false;

    // Exactly the pattern nodes that exist are mapped, each to a target node that exists, and
    // each respecting the labels.
    for (node pu = 0; pu < pz; ++pu) {
        if (!pattern.hasNode(pu)) {
            if (match[pu] != none)
                return false;
            continue;
        }
        if (match[pu] == none || match[pu] >= target.upperNodeIdBound()
            || !target.hasNode(match[pu]))
            return false;
        if (!labelsCompatible(patternLabels, targetLabels, pu, match[pu]))
            return false;
    }

    // Injective: no two pattern nodes share an image.
    for (node a = 0; a < pz; ++a) {
        if (!pattern.hasNode(a))
            continue;
        for (node b = a + 1; b < pz; ++b) {
            if (pattern.hasNode(b) && match[a] == match[b])
                return false;
        }
    }

    // The semantics themselves, over every *ordered* pair of distinct pattern nodes - ordered so
    // that directed graphs get both directions checked independently.
    for (node a = 0; a < pz; ++a) {
        if (!pattern.hasNode(a))
            continue;
        for (node b = 0; b < pz; ++b) {
            if (a == b || !pattern.hasNode(b))
                continue;

            const bool patternEdge = pattern.hasEdge(a, b);
            const bool targetEdge = target.hasEdge(match[a], match[b]);

            if (patternEdge && !targetEdge)
                return false;
            if (semantics == Semantics::INDUCED && !patternEdge && targetEdge)
                return false;
        }
    }

    return true;
}

/// Every match, by exhaustive enumeration.
///
/// Generates every injective assignment of pattern nodes to target nodes with no pruning beyond
/// injectivity, then hands each finished assignment to @ref isValidMatch(). That is
/// O(targetNodes ^ patternNodes), so it is only usable on the tiny inputs in @ref standardCases().
///
/// An empty pattern yields exactly one match - the empty mapping - which is the convention the
/// four algorithms have to agree on.
inline std::vector<Match> referenceMatches(const Graph &pattern, const Graph &target,
                                           Semantics semantics,
                                           const std::vector<index> &patternLabels = {},
                                           const std::vector<index> &targetLabels = {}) {
    std::vector<node> patternNodes;
    pattern.forNodes([&](node u) { patternNodes.push_back(u); });

    std::vector<node> targetNodes;
    target.forNodes([&](node v) { targetNodes.push_back(v); });

    std::vector<Match> matches;
    Match current(pattern.upperNodeIdBound(), none);
    std::vector<bool> used(target.upperNodeIdBound(), false);

    std::function<void(index)> assign = [&](index next) {
        if (next == patternNodes.size()) {
            if (isValidMatch(pattern, target, semantics, patternLabels, targetLabels, current))
                matches.push_back(current);
            return;
        }

        const node pu = patternNodes[next];
        for (node tv : targetNodes) {
            if (used[tv])
                continue;

            used[tv] = true;
            current[pu] = tv;
            assign(next + 1);
            current[pu] = none;
            used[tv] = false;
        }
    };
    assign(0);

    return matches;
}

/// Put matches in a canonical order so two result sets can be compared. Necessary rather than
/// cosmetic: ParallelRI explicitly promises no particular order.
inline void sortMatches(std::vector<Match> &matches) {
    std::sort(matches.begin(), matches.end());
}

// ---------------------------------------------------------------------------------------------
// The shared corpus
// ---------------------------------------------------------------------------------------------

/// One entry of the corpus. Label vectors are empty for an unlabelled case.
struct Case {
    std::string name;
    Graph pattern;
    Graph target;
    Semantics semantics;
    std::vector<index> patternLabels;
    std::vector<index> targetLabels;
};

inline Graph graphOf(count n, std::initializer_list<std::pair<node, node>> edges,
                     bool directed = false) {
    Graph G(n, false, directed);
    for (const std::pair<node, node> &e : edges)
        G.addEdge(e.first, e.second);
    return G;
}

/// The cases every algorithm must agree with the reference on.
///
/// Each one is here because it is somewhere four independent implementations would otherwise
/// quietly disagree - not for coverage of the happy path, which any of them will get right.
inline std::vector<Case> standardCases() {
    std::vector<Case> cases;

    const Graph triangle = graphOf(3, {{0, 1}, {1, 2}, {2, 0}});
    const Graph path3 = graphOf(3, {{0, 1}, {1, 2}});
    const Graph k4 = graphOf(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});
    const Graph k5 = graphOf(
        5, {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2}, {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 4}});

    // The worked example in the class documentation: a 3-path does occur in a triangle, but not
    // as an induced occurrence, because the pattern's missing edge must stay missing.
    cases.push_back({"path3-in-triangle-mono", path3, triangle, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back({"path3-in-triangle-induced", path3, triangle, Semantics::INDUCED, {}, {}});

    // Dense: here the two semantics agree, because the pattern has no missing edge to protect.
    cases.push_back({"triangle-in-k4-mono", triangle, k4, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back({"triangle-in-k4-induced", triangle, k4, Semantics::INDUCED, {}, {}});
    cases.push_back({"k4-in-k5-induced", k4, k5, Semantics::INDUCED, {}, {}});

    // A pattern that cannot fit. Catches an early-exit that bails out too eagerly, and one that
    // does not bail out at all.
    cases.push_back({"pattern-larger-than-target",
                     triangle,
                     graphOf(2, {{0, 1}}),
                     Semantics::MONOMORPHISM,
                     {},
                     {}});

    // An isolated pattern node has no already-mapped neighbour to draw candidates from, so it
    // exercises the fallback "every unmapped target node" rule.
    cases.push_back({"pattern-with-isolated-node",
                     graphOf(3, {{0, 1}}),
                     graphOf(4, {{0, 1}, {1, 2}}),
                     Semantics::MONOMORPHISM,
                     {},
                     {}});
    cases.push_back({"disconnected-pattern",
                     graphOf(4, {{0, 1}, {2, 3}}),
                     graphOf(5, {{0, 1}, {1, 2}, {3, 4}}),
                     Semantics::MONOMORPHISM,
                     {},
                     {}});

    // Removed node ids. In the snapshot a removed id looks exactly like an isolated node - both
    // have an empty adjacency slice - so an algorithm that does not consult hasNode() will map
    // pattern nodes onto ids that are not nodes and invent matches that do not exist.
    Graph gappedTarget = graphOf(7, {{0, 1}, {1, 3}, {3, 4}, {4, 6}});
    gappedTarget.removeNode(2);
    gappedTarget.removeNode(5);
    cases.push_back(
        {"target-with-removed-ids", path3, gappedTarget, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back({"isolated-pattern-node-vs-removed-ids",
                     graphOf(3, {{0, 1}}),
                     gappedTarget,
                     Semantics::MONOMORPHISM,
                     {},
                     {}});

    Graph gappedPattern = graphOf(4, {{0, 1}, {1, 3}});
    gappedPattern.removeNode(2);
    cases.push_back(
        {"pattern-with-removed-ids", gappedPattern, k4, Semantics::MONOMORPHISM, {}, {}});

    // Degenerate patterns. Both must produce exactly one match - the empty mapping - rather than
    // zero or a crash, and the match has to be full width with `none` at every gap.
    cases.push_back({"empty-pattern", Graph(0), k4, Semantics::INDUCED, {}, {}});
    Graph allRemoved(3);
    for (node u = 0; u < 3; ++u)
        allRemoved.removeNode(u);
    cases.push_back({"pattern-with-all-nodes-removed", allRemoved, k4, Semantics::INDUCED, {}, {}});
    cases.push_back(
        {"single-node-pattern", Graph(1), gappedTarget, Semantics::MONOMORPHISM, {}, {}});

    // Multi-edges and a self-loop in the target must change nothing. If they leak through, the
    // same match gets reported twice, or degree pruning rejects valid host nodes.
    Graph messyTarget(4, false, false);
    messyTarget.addEdge(0, 1);
    messyTarget.addEdge(0, 1); // parallel
    messyTarget.addEdge(1, 2);
    messyTarget.addEdge(2, 0);
    messyTarget.addEdge(3, 3); // self-loop
    messyTarget.addEdge(2, 3);
    cases.push_back(
        {"target-with-multiedges-and-loop", path3, messyTarget, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back({"target-with-multiedges-and-loop-induced",
                     path3,
                     messyTarget,
                     Semantics::INDUCED,
                     {},
                     {}});

    // Directed. The 2-cycle is the case that catches an implementation checking only one
    // direction: it is a different question from an undirected edge, and both arcs must exist.
    const Graph arc = graphOf(2, {{0, 1}}, true);
    const Graph twoCycle = graphOf(2, {{0, 1}, {1, 0}}, true);
    const Graph directedTriangle = graphOf(3, {{0, 1}, {1, 2}, {2, 0}}, true);
    const Graph directedTarget = graphOf(4, {{0, 1}, {1, 0}, {1, 2}, {2, 3}, {3, 1}}, true);

    cases.push_back({"arc-in-directed", arc, directedTarget, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back(
        {"two-cycle-in-directed", twoCycle, directedTarget, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back(
        {"directed-triangle", directedTriangle, directedTarget, Semantics::MONOMORPHISM, {}, {}});
    cases.push_back({"directed-triangle-induced",
                     directedTriangle,
                     directedTarget,
                     Semantics::INDUCED,
                     {},
                     {}});

    // Labels, including `none` as a wildcard on each side independently.
    cases.push_back({"labelled-path3-in-k4", path3, k4, Semantics::MONOMORPHISM,
                     /* pattern */ {7, 8, 7},
                     /* target  */ {7, 8, 7, 9}});
    cases.push_back({"labelled-wildcard-on-pattern",
                     path3,
                     k4,
                     Semantics::MONOMORPHISM,
                     {none, 8, none},
                     {7, 8, 7, 9}});
    cases.push_back({"labelled-wildcard-on-target",
                     path3,
                     k4,
                     Semantics::MONOMORPHISM,
                     {7, 8, 7},
                     {none, 8, 7, none}});
    cases.push_back(
        {"labelled-no-match", triangle, k4, Semantics::MONOMORPHISM, {1, 1, 1}, {1, 1, 2, 2}});

    return cases;
}

/**
 * The reference matcher wearing the SubgraphIsomorphism interface.
 *
 * Nobody should ever *use* this - it enumerates every injective mapping - but it follows the
 * run() protocol exactly, and that buys two things. It is a worked example of that protocol for
 * whoever implements a real algorithm, short enough to read in one go. And it gives the helpers
 * below something to run against before any real algorithm exists, so a mistake in the harness
 * surfaces now rather than on the first day somebody tries to use it.
 */
class ReferenceSubgraphIsomorphism final : public SubgraphIsomorphism {

public:
    ReferenceSubgraphIsomorphism(const Graph &pattern, const Graph &target,
                                 Semantics semantics = Semantics::INDUCED, count maxMatches = 0)
        : SubgraphIsomorphism(pattern, target, semantics, maxMatches) {}

    void run() override {
        // The signal handler is a plain local, exactly as in MaximalCliques::run(). This search is
        // sequential, so the throwing assureRunning() is safe; a parallel one would have to use
        // isRunning() inside the region instead.
        Aux::SignalHandler handler;

        // 1. Forget any earlier run.
        prepareRun();

        // 2. Search, reporting each complete mapping and stopping the moment reportMatch()
        //    says so.
        for (const Match &match :
             referenceMatches(*pattern, *target, semantics, patternLabels, targetLabels)) {
            handler.assureRunning();
            if (!reportMatch(match))
                break;
        }

        // 3. Mark the run finished.
        finishRun();
    }
};

// ---------------------------------------------------------------------------------------------
// The assertions each algorithm's test file calls
//
// Every helper takes a factory rather than a type, because RI and ParallelRI need a Variant
// argument that VF2 and VF3 do not:
//
//     auto make = [](const Graph &p, const Graph &t, Semantics s, count cap) {
//         return std::unique_ptr<SubgraphIsomorphism>(new VF2(p, t, s, cap));
//     };
// ---------------------------------------------------------------------------------------------

/// Run the algorithm over the whole corpus and assert it reproduces the reference exactly.
template <typename Construct>
void expectMatchesReference(Construct construct) {
    for (const Case &testCase : standardCases()) {
        std::vector<Match> expected =
            referenceMatches(testCase.pattern, testCase.target, testCase.semantics,
                             testCase.patternLabels, testCase.targetLabels);
        sortMatches(expected);

        std::unique_ptr<SubgraphIsomorphism> algo =
            construct(testCase.pattern, testCase.target, testCase.semantics, count{0});
        if (!testCase.patternLabels.empty())
            algo->setLabels(testCase.patternLabels, testCase.targetLabels);
        algo->run();

        std::vector<Match> actual = algo->getMatches();
        sortMatches(actual);

        EXPECT_EQ(actual, expected) << "case: " << testCase.name;
        EXPECT_EQ(algo->numberOfMatches(), expected.size()) << "case: " << testCase.name;
        EXPECT_EQ(algo->hasMatch(), !expected.empty()) << "case: " << testCase.name;

        // Independently of agreeing with the reference, every match must be well-formed on its
        // own terms - full width, `none` at pattern gaps, injective, semantics respected.
        for (const Match &match : actual) {
            EXPECT_TRUE(isValidMatch(testCase.pattern, testCase.target, testCase.semantics,
                                     testCase.patternLabels, testCase.targetLabels, match))
                << "case: " << testCase.name << " produced a malformed match";
        }
    }
}

/// Assert that a cap on the number of matches is honoured exactly.
template <typename Construct>
void expectRespectsMatchCap(Construct construct) {
    // 24 matches: every ordering of 3 of K4's 4 nodes.
    const Graph pattern = graphOf(3, {{0, 1}, {1, 2}, {2, 0}});
    const Graph target = graphOf(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});
    const count total = 24;

    for (count cap : {count{1}, count{5}, count{23}, total, total + 10}) {
        std::unique_ptr<SubgraphIsomorphism> algo =
            construct(pattern, target, Semantics::MONOMORPHISM, cap);
        algo->run();

        const count expected = std::min(cap, total);
        EXPECT_EQ(algo->numberOfMatches(), expected) << "cap: " << cap;
        EXPECT_EQ(algo->getMatches().size(), expected) << "cap: " << cap;
        EXPECT_TRUE(algo->hasMatch()) << "cap: " << cap;

        for (const Match &match : algo->getMatches()) {
            EXPECT_TRUE(isValidMatch(pattern, target, Semantics::MONOMORPHISM, {}, {}, match))
                << "cap: " << cap << " produced a malformed match";
        }
    }
}

/// Assert that the three ways of receiving matches agree with each other.
///
/// This is where the serialization guarantee on MatchCallback gets exercised: the serial form must
/// see every match exactly once even when several workers are producing them, which is what
/// SubgraphIsomorphism::invokeCallback() takes a lock for.
template <typename Construct>
void expectCallbackFormsAgree(Construct construct) {
    for (const Case &testCase : standardCases()) {
        std::vector<Match> expected =
            referenceMatches(testCase.pattern, testCase.target, testCase.semantics,
                             testCase.patternLabels, testCase.targetLabels);
        sortMatches(expected);

        const auto build = [&]() {
            std::unique_ptr<SubgraphIsomorphism> algo =
                construct(testCase.pattern, testCase.target, testCase.semantics, count{0});
            if (!testCase.patternLabels.empty())
                algo->setLabels(testCase.patternLabels, testCase.targetLabels);
            return algo;
        };

        // Serial callback: collected without any locking of our own, which is only safe because
        // the module promises this form is never entered twice at once.
        {
            std::vector<Match> collected;
            std::unique_ptr<SubgraphIsomorphism> algo = build();
            algo->setCallback([&](const std::vector<node> &match) { collected.push_back(match); });
            algo->run();

            sortMatches(collected);
            EXPECT_EQ(collected, expected) << "case: " << testCase.name << " (serial callback)";
            EXPECT_EQ(algo->numberOfMatches(), expected.size())
                << "case: " << testCase.name << " (serial callback)";
        }

        // Parallel callback: every worker gets its own slot, so this needs no locking either.
        // Sizing by numberOfWorkers() is the point of that accessor - and if an algorithm ever
        // reported a tid at or beyond it, this would index out of bounds and the test would say so.
        {
            std::unique_ptr<SubgraphIsomorphism> algo = build();
            std::vector<std::vector<Match>> perWorker(algo->numberOfWorkers());
            algo->setCallback([&](index tid, const std::vector<node> &match) {
                ASSERT_LT(tid, perWorker.size());
                perWorker[tid].push_back(match);
            });
            algo->run();

            std::vector<Match> collected;
            for (const std::vector<Match> &slot : perWorker)
                collected.insert(collected.end(), slot.begin(), slot.end());

            sortMatches(collected);
            EXPECT_EQ(collected, expected) << "case: " << testCase.name << " (parallel callback)";
        }

        // Counting only: nothing is stored, so getMatches() throws but the count still holds.
        {
            std::unique_ptr<SubgraphIsomorphism> algo = build();
            algo->setStoreMatches(false);
            algo->run();

            EXPECT_EQ(algo->numberOfMatches(), expected.size())
                << "case: " << testCase.name << " (counting only)";
            EXPECT_THROW(algo->getMatches(), std::runtime_error) << "case: " << testCase.name;
        }
    }
}

} // namespace IsomorphismTest
} // namespace NetworKit

#endif // NETWORKIT_CPP_ISOMORPHISM_TEST_SUBGRAPH_ISOMORPHISM_TEST_UTILS_HPP_
