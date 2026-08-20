/*
 * VF2GTest.cpp
 *
 *  Created on: Aug 17, 2026
 *      Author: Alexandra
 */

#include <algorithm>
#include <iterator>
#include <vector>

#include <gtest/gtest.h>

#include <networkit/auxiliary/Random.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/graph/GraphTools.hpp>
#include <networkit/io/EdgeListReader.hpp>
#include <networkit/io/METISGraphReader.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>
#include <networkit/isomorphism/VF2.hpp>

#include "SubgraphIsomorphismTestUtils.hpp"
#include "../SearchGraph.hpp"

namespace NetworKit {

class VF2GTest : public testing::Test {};

TEST_F(VF2GTest, dummyTest) {

    Graph target = Graph(6);

    target.addEdge(0, 1);
    target.addEdge(1, 2);
    target.addEdge(2, 0);

    target.addEdge(3, 4);
    target.addEdge(4, 5);
    target.addEdge(5, 3);

    target.addEdge(2, 3);

    Graph pattern = Graph(3);

    pattern.addEdge(0, 1);
    pattern.addEdge(1, 2);
    pattern.addEdge(2, 0);

    VF2 vf = VF2(pattern, target);
    vf.run();
    EXPECT_TRUE(vf.hasMatch());
    EXPECT_EQ(vf.numberOfMatches(), 12);

    METISGraphReader reader;
    Graph karate = reader.read("input/karate.graph");

    VF2 vf1 = VF2(pattern, karate);
    vf1.run();
    EXPECT_TRUE(vf1.hasMatch());
    EXPECT_EQ(vf1.numberOfMatches(), 270);
}

TEST_F(VF2GTest, testInterrupt) {
    Graph pattern = IsomorphismTest::graphOf(3, {{0, 1}, {1, 2}, {2, 0}});
    METISGraphReader reader;
    Graph airfoil = reader.read("input/airfoil1.graph");

    VF2 vf = VF2(pattern, airfoil);

    std::thread interruptThread([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::raise(SIGINT);
    });

    EXPECT_THROW(vf.run(), Aux::SignalHandler::InterruptException);

    interruptThread.join();
}

TEST_F(VF2GTest, testTrivialCases) {

    // Empty pattern, undirected
    Graph pattern1 = Graph(0);
    Graph target1 = IsomorphismTest::graphOf(3, {{0, 1}, {1, 2}, {2, 0}});

    VF2 vf1 = VF2(pattern1, target1);
    vf1.run();

    EXPECT_TRUE(vf1.hasMatch());
    EXPECT_EQ(vf1.numberOfMatches(), 1);
    EXPECT_EQ(vf1.getMatches(), std::vector<std::vector<node>>{{}});

    // Empty pattern, undirected
    Graph pattern2 = Graph(0, false, true);
    Graph target2 = IsomorphismTest::graphOf(3, {{0, 1}, {1, 2}, {2, 0}}, true);

    VF2 vf2 = VF2(pattern2, target2);
    vf2.run();

    EXPECT_TRUE(vf2.hasMatch());
    EXPECT_EQ(vf2.numberOfMatches(), 1);
    EXPECT_EQ(vf2.getMatches(), std::vector<std::vector<node>>{{}});

    // Pattern with more nodes than target, undirected
    Graph pattern3 = Graph(4);
    Graph target3 = Graph(2);

    VF2 vf3 = VF2(pattern3, target3);
    vf3.run();

    EXPECT_FALSE(vf3.hasMatch());
    EXPECT_EQ(vf3.numberOfMatches(), 0);
    EXPECT_EQ(vf3.getMatches(), std::vector<std::vector<node>>{});

    // Pattern with more nodes than target, directed
    Graph pattern4 = Graph(4, false, true);
    Graph target4 = Graph(2, false, true);

    VF2 vf4 = VF2(pattern4, target4);
    vf4.run();

    EXPECT_FALSE(vf4.hasMatch());
    EXPECT_EQ(vf4.numberOfMatches(), 0);
    EXPECT_EQ(vf4.getMatches(), std::vector<std::vector<node>>{});

    // Pattern with higher maximum degree than target, undirected
    Graph pattern5 = IsomorphismTest::graphOf(4, {{0, 1}, {0, 2}, {0, 3}});
    Graph target5 = IsomorphismTest::graphOf(5, {{1, 2}});

    VF2 vf5 = VF2(pattern5, target5);
    vf5.run();

    EXPECT_FALSE(vf5.hasMatch());
    EXPECT_EQ(vf5.numberOfMatches(), 0);
    EXPECT_EQ(vf5.getMatches(), std::vector<std::vector<node>>{});

    // Pattern with higher maximum in-degree than target, directed
    Graph pattern6 = IsomorphismTest::graphOf(4, {{1, 0}, {2, 0}, {3, 0}}, true);
    Graph target6 = IsomorphismTest::graphOf(5, {{1, 2}}, true);

    VF2 vf6 = VF2(pattern6, target6);
    vf6.run();

    EXPECT_FALSE(vf6.hasMatch());
    EXPECT_EQ(vf6.numberOfMatches(), 0);
    EXPECT_EQ(vf6.getMatches(), std::vector<std::vector<node>>{});

    // Pattern with higher maximum out-degree than target, directed
    Graph pattern7 = IsomorphismTest::graphOf(4, {{0, 1}, {0, 2}, {0, 3}}, true);
    Graph target7 = IsomorphismTest::graphOf(5, {{1, 2}}, true);

    VF2 vf7 = VF2(pattern7, target7);
    vf7.run();

    EXPECT_FALSE(vf7.hasMatch());
    EXPECT_EQ(vf7.numberOfMatches(), 0);
    EXPECT_EQ(vf7.getMatches(), std::vector<std::vector<node>>{});
}

TEST_F(VF2GTest, testMatchesReference) {
    auto make = [](const Graph &pattern, const Graph &target,
                   SubgraphIsomorphism::Semantics semantics, count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new VF2(pattern, target, semantics, maxMatches));
    };

    IsomorphismTest::expectMatchesReference(make);
}

TEST_F(VF2GTest, testRespectsMatchCap) {
    auto make = [](const Graph &pattern, const Graph &target,
                   SubgraphIsomorphism::Semantics semantics, count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new VF2(pattern, target, semantics, maxMatches));
    };

    IsomorphismTest::expectRespectsMatchCap(make);
}

TEST_F(VF2GTest, testCallbackFormsAgree) {
    auto make = [](const Graph &pattern, const Graph &target,
                   SubgraphIsomorphism::Semantics semantics, count maxMatches) {
        return std::unique_ptr<SubgraphIsomorphism>(
            new VF2(pattern, target, semantics, maxMatches));
    };

    IsomorphismTest::expectCallbackFormsAgree(make);
}

} // namespace NetworKit
