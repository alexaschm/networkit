/*
 * ReferenceMatcherGTest.cpp
 *
 * Tests the brute-force matcher in SubgraphIsomorphismTestUtils.hpp.
 *
 * Every algorithm in this module is checked against that matcher, so it is the one thing here
 * with nothing behind it. It is therefore pinned down three ways: against counts worked out by
 * hand, against an unrelated piece of NetworKit that counts triangles a completely different way,
 * and against structural invariants that must hold for any match whatsoever.
 */

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include <networkit/Globals.hpp>
#include <networkit/edgescores/ChibaNishizekiTriangleEdgeScore.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/io/METISGraphReader.hpp>

#include "SubgraphIsomorphismTestUtils.hpp"

namespace NetworKit {

using IsomorphismTest::graphOf;
using IsomorphismTest::isValidMatch;
using IsomorphismTest::Match;
using IsomorphismTest::referenceMatches;
using IsomorphismTest::Semantics;
using IsomorphismTest::standardCases;

class ReferenceMatcherGTest : public testing::Test {};

TEST_F(ReferenceMatcherGTest, testHandComputedCounts) {

    const Graph triangle = graphOf(3, {{0, 1}, {1, 2}, {2, 0}});
    const Graph path3 = graphOf(3, {{0, 1}, {1, 2}});
    const Graph k4 = graphOf(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});

    // K4 is complete, so every injective map of three nodes works: 4 * 3 * 2 = 24. The pattern is
    // complete too, so it has no non-edge to protect and INDUCED gives the same answer.
    EXPECT_EQ(referenceMatches(triangle, k4, Semantics::MONOMORPHISM).size(), 24u);
    EXPECT_EQ(referenceMatches(triangle, k4, Semantics::INDUCED).size(), 24u);

    // The worked example from the class documentation. All 3! = 6 orderings satisfy the two
    // pattern edges, but the pattern's missing 0-2 edge is present in the target, so INDUCED
    // rejects every one of them.
    EXPECT_EQ(referenceMatches(path3, triangle, Semantics::MONOMORPHISM).size(), 6u);
    EXPECT_EQ(referenceMatches(path3, triangle, Semantics::INDUCED).size(), 0u);

    // Three nodes do not fit into two, whatever the edges say.
    EXPECT_EQ(referenceMatches(triangle, graphOf(2, {{0, 1}}), Semantics::MONOMORPHISM).size(), 0u);

    // A single undirected edge maps onto each of K4's 6 edges in both orientations.
    EXPECT_EQ(referenceMatches(graphOf(2, {{0, 1}}), k4, Semantics::MONOMORPHISM).size(), 12u);
}

TEST_F(ReferenceMatcherGTest, testDirectedCounts) {

    // The 2-cycle is the case that separates a real directed check from one that quietly treats
    // arcs as undirected: only the 0<->1 pair has both arcs.
    const Graph target = graphOf(4, {{0, 1}, {1, 0}, {1, 2}, {2, 3}, {3, 1}}, true);

    EXPECT_EQ(referenceMatches(graphOf(2, {{0, 1}}, true), target, Semantics::MONOMORPHISM).size(),
              5u); // one match per arc

    EXPECT_EQ(referenceMatches(graphOf(2, {{0, 1}, {1, 0}}, true), target, Semantics::MONOMORPHISM)
                  .size(),
              2u); // 0->1 and 1->0, in both assignments

    // 1 -> 2 -> 3 -> 1 is the only directed triangle, found once per starting point.
    EXPECT_EQ(
        referenceMatches(graphOf(3, {{0, 1}, {1, 2}, {2, 0}}, true), target, Semantics::INDUCED)
            .size(),
        3u);
}

TEST_F(ReferenceMatcherGTest, testDegeneratePatterns) {

    const Graph k4 = graphOf(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});

    // An empty pattern occurs exactly once - as the empty mapping - not zero times.
    const std::vector<Match> empty = referenceMatches(Graph(0), k4, Semantics::INDUCED);
    ASSERT_EQ(empty.size(), 1u);
    EXPECT_TRUE(empty[0].empty());

    // A pattern whose nodes were all removed is the same statement at a non-zero id bound, and
    // pins the convention: full width, `none` at every gap.
    Graph allRemoved(3);
    for (node u = 0; u < 3; ++u)
        allRemoved.removeNode(u);

    const std::vector<Match> removed = referenceMatches(allRemoved, k4, Semantics::INDUCED);
    ASSERT_EQ(removed.size(), 1u);
    EXPECT_EQ(removed[0], (Match{none, none, none}));

    // A one-node pattern sits on every target node and nowhere else.
    const std::vector<Match> single = referenceMatches(Graph(1), k4, Semantics::MONOMORPHISM);
    EXPECT_EQ(single.size(), 4u);
}

TEST_F(ReferenceMatcherGTest, testRemovedTargetIdsAreNeverUsed) {

    // The phantom-match bug in one test. A removed id and an isolated node are indistinguishable
    // by adjacency, so a matcher that enumerates ids rather than nodes will map pattern nodes
    // onto ids that do not exist - and it shows up first on patterns with an isolated node,
    // because those are the ones whose candidates are not constrained by a neighbour.
    Graph target = graphOf(7, {{0, 1}, {1, 3}, {3, 4}, {4, 6}});
    target.removeNode(2);
    target.removeNode(5);

    ASSERT_EQ(target.numberOfNodes(), 5u);
    ASSERT_EQ(target.upperNodeIdBound(), 7u);

    const std::vector<Match> matches =
        referenceMatches(graphOf(3, {{0, 1}}), target, Semantics::MONOMORPHISM);

    ASSERT_FALSE(matches.empty());
    for (const Match &match : matches) {
        for (node image : match) {
            ASSERT_NE(image, none);
            EXPECT_TRUE(target.hasNode(image)) << "mapped onto removed id " << image;
        }
    }

    // Isolated pattern node 2 may sit on any of the 5 real nodes not already taken by the edge,
    // and the edge maps onto each of the 4 target edges in both orientations: 8 * 3 = 24.
    EXPECT_EQ(matches.size(), 24u);
}

TEST_F(ReferenceMatcherGTest, testMultiEdgesAndLoopsInTargetChangeNothing) {

    // Parallel edges and self-loops carry no information a subgraph search can use, so adding
    // them must not change the answer - in particular must not report a match twice.
    const Graph path3 = graphOf(3, {{0, 1}, {1, 2}});
    const Graph clean = graphOf(4, {{0, 1}, {1, 2}, {2, 0}, {2, 3}});

    Graph messy(4);
    messy.addEdge(0, 1);
    messy.addEdge(0, 1); // parallel
    messy.addEdge(1, 2);
    messy.addEdge(2, 0);
    messy.addEdge(3, 3); // self-loop
    messy.addEdge(2, 3);

    for (Semantics semantics : {Semantics::MONOMORPHISM, Semantics::INDUCED}) {
        std::vector<Match> fromClean = referenceMatches(path3, clean, semantics);
        std::vector<Match> fromMessy = referenceMatches(path3, messy, semantics);
        IsomorphismTest::sortMatches(fromClean);
        IsomorphismTest::sortMatches(fromMessy);
        EXPECT_EQ(fromClean, fromMessy);
    }
}

TEST_F(ReferenceMatcherGTest, testLabelWildcards) {

    const Graph edge = graphOf(2, {{0, 1}});
    const Graph target = graphOf(3, {{0, 1}, {1, 2}});

    // Labels 5, 6, 5 on the target; the pattern asks for a 5 next to a 6.
    const std::vector<index> targetLabels = {5, 6, 5};

    EXPECT_EQ(referenceMatches(edge, target, Semantics::MONOMORPHISM, {5, 6}, targetLabels).size(),
              2u); // 0->0,1->1 and 0->2,1->1

    EXPECT_EQ(referenceMatches(edge, target, Semantics::MONOMORPHISM, {6, 6}, targetLabels).size(),
              0u); // no two adjacent 6s

    // `none` on the pattern side matches any target label...
    EXPECT_EQ(
        referenceMatches(edge, target, Semantics::MONOMORPHISM, {none, 6}, targetLabels).size(),
        2u);

    // ...and on the target side it is the target node that accepts any pattern label.
    EXPECT_EQ(
        referenceMatches(edge, target, Semantics::MONOMORPHISM, {9, 9}, {none, none, none}).size(),
        4u); // unlabelled behaviour: both orientations of both edges
}

TEST_F(ReferenceMatcherGTest, testKarateTrianglesAgainstChibaNishizeki) {

    // The strongest check available: an unrelated part of NetworKit counts the same objects by a
    // completely different method. ChibaNishizeki gives per-edge triangle counts, so their sum is
    // 3 * (number of triangles), while subgraph matching finds each triangle once per way of
    // labelling its corners - 6 - which makes the expected count exactly twice the sum.
    METISGraphReader reader;
    Graph karate = reader.read("input/karate.graph");
    karate.indexEdges();

    ChibaNishizekiTriangleEdgeScore triangleScore(karate);
    triangleScore.run();
    const std::vector<count> perEdge = triangleScore.scores();

    count edgeSum = 0;
    for (count c : perEdge)
        edgeSum += c;

    ASSERT_GT(edgeSum, 0u) << "karate should contain triangles";
    ASSERT_EQ(edgeSum % 3, 0u) << "each triangle must be counted on exactly three edges";

    const Graph triangle = graphOf(3, {{0, 1}, {1, 2}, {2, 0}});
    const std::vector<Match> matches = referenceMatches(triangle, karate, Semantics::MONOMORPHISM);

    EXPECT_EQ(matches.size(), 2 * edgeSum);
    EXPECT_EQ(matches.size(), 6 * (edgeSum / 3));

    // Pinned, so that a change making both sides wrong the same way cannot pass unnoticed:
    // karate has 45 triangles, hence 135 edge-incidences and 270 labelled occurrences.
    EXPECT_EQ(edgeSum / 3, 45u);
    EXPECT_EQ(matches.size(), 270u);

    // Spot-check that they really are triangles, independently of how they were found.
    for (const Match &match : matches) {
        EXPECT_TRUE(karate.hasEdge(match[0], match[1]));
        EXPECT_TRUE(karate.hasEdge(match[1], match[2]));
        EXPECT_TRUE(karate.hasEdge(match[2], match[0]));
    }
}

TEST_F(ReferenceMatcherGTest, testCorpusInvariants) {

    // Properties that must hold for every case in the shared corpus. These are the invariants the
    // four algorithms will be held to, so the reference has to satisfy them first.
    for (const IsomorphismTest::Case &testCase : standardCases()) {
        std::vector<Match> matches =
            referenceMatches(testCase.pattern, testCase.target, testCase.semantics,
                             testCase.patternLabels, testCase.targetLabels);

        for (const Match &match : matches) {
            EXPECT_EQ(match.size(), testCase.pattern.upperNodeIdBound())
                << "case: " << testCase.name << " - matches are indexed by pattern node";

            EXPECT_TRUE(isValidMatch(testCase.pattern, testCase.target, testCase.semantics,
                                     testCase.patternLabels, testCase.targetLabels, match))
                << "case: " << testCase.name;

            // `none` exactly at the pattern's gaps, a real target node everywhere else.
            for (node pu = 0; pu < testCase.pattern.upperNodeIdBound(); ++pu) {
                if (testCase.pattern.hasNode(pu)) {
                    EXPECT_NE(match[pu], none) << "case: " << testCase.name;
                    EXPECT_TRUE(testCase.target.hasNode(match[pu])) << "case: " << testCase.name;
                } else {
                    EXPECT_EQ(match[pu], none) << "case: " << testCase.name;
                }
            }
        }

        // No match may be reported twice.
        std::vector<Match> sorted = matches;
        IsomorphismTest::sortMatches(sorted);
        EXPECT_EQ(std::unique(sorted.begin(), sorted.end()), sorted.end())
            << "case: " << testCase.name << " reported a duplicate match";
    }
}

TEST_F(ReferenceMatcherGTest, testInducedIsSubsetOfMonomorphism) {

    // INDUCED adds a requirement and never relaxes one, so its matches are always a subset. A
    // reference that got the direction of the non-edge rule backwards would fail here.
    for (const IsomorphismTest::Case &testCase : standardCases()) {
        std::vector<Match> induced =
            referenceMatches(testCase.pattern, testCase.target, Semantics::INDUCED,
                             testCase.patternLabels, testCase.targetLabels);
        std::vector<Match> mono =
            referenceMatches(testCase.pattern, testCase.target, Semantics::MONOMORPHISM,
                             testCase.patternLabels, testCase.targetLabels);

        IsomorphismTest::sortMatches(induced);
        IsomorphismTest::sortMatches(mono);

        EXPECT_TRUE(std::includes(mono.begin(), mono.end(), induced.begin(), induced.end()))
            << "case: " << testCase.name;
        EXPECT_LE(induced.size(), mono.size()) << "case: " << testCase.name;
    }
}

TEST_F(ReferenceMatcherGTest, testIsValidMatchRejectsMalformedMappings) {

    // isValidMatch is what judges the algorithms' output, so it has to reject as well as accept.
    const Graph path3 = graphOf(3, {{0, 1}, {1, 2}});
    const Graph target = graphOf(4, {{0, 1}, {1, 2}, {2, 0}, {2, 3}});

    EXPECT_TRUE(isValidMatch(path3, target, Semantics::MONOMORPHISM, {}, {}, {0, 1, 2}));

    EXPECT_FALSE(isValidMatch(path3, target, Semantics::MONOMORPHISM, {}, {}, {0, 1}))
        << "wrong width";
    EXPECT_FALSE(isValidMatch(path3, target, Semantics::MONOMORPHISM, {}, {}, {0, 1, 0}))
        << "not injective";
    EXPECT_FALSE(isValidMatch(path3, target, Semantics::MONOMORPHISM, {}, {}, {0, 3, 1}))
        << "pattern edge 0-1 has no counterpart";
    EXPECT_FALSE(isValidMatch(path3, target, Semantics::MONOMORPHISM, {}, {}, {0, 1, none}))
        << "existing pattern node left unmapped";

    // 0-1-2 in the target closes into a triangle, so the pattern's missing 0-2 edge is violated:
    // fine under MONOMORPHISM, rejected under INDUCED. This single pair is the whole difference
    // between the two semantics.
    EXPECT_TRUE(isValidMatch(path3, target, Semantics::MONOMORPHISM, {}, {}, {0, 1, 2}));
    EXPECT_FALSE(isValidMatch(path3, target, Semantics::INDUCED, {}, {}, {0, 1, 2}));
    EXPECT_TRUE(isValidMatch(path3, target, Semantics::INDUCED, {}, {}, {0, 1, 3}) == false)
        << "0-1 and 1-3 are not both edges";

    // Labels
    EXPECT_TRUE(
        isValidMatch(path3, target, Semantics::MONOMORPHISM, {1, 2, 1}, {1, 2, 1, 9}, {0, 1, 2}));
    EXPECT_FALSE(
        isValidMatch(path3, target, Semantics::MONOMORPHISM, {1, 2, 9}, {1, 2, 1, 9}, {0, 1, 2}))
        << "pattern node 2 wants label 9, target node 2 has label 1";
    EXPECT_TRUE(
        isValidMatch(path3, target, Semantics::MONOMORPHISM, {1, 2, none}, {1, 2, 1, 9}, {0, 1, 2}))
        << "none on the pattern side is a wildcard";

    // A mapping onto a removed target id must be rejected however plausible it looks.
    Graph gapped = graphOf(4, {{0, 1}, {1, 2}});
    gapped.removeNode(3);
    EXPECT_FALSE(isValidMatch(path3, gapped, Semantics::MONOMORPHISM, {}, {}, {0, 1, 3}));
}

} // namespace NetworKit
