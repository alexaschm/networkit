/*
 * SearchGraphGTest.cpp
 *
 *  Created on: Aug 12, 2026
 *      Author: Alexandra
 */

#include <vector>

#include <gtest/gtest.h>

#include <networkit/graph/Graph.hpp>
#include <networkit/io/EdgeListReader.hpp>
#include <networkit/io/METISGraphReader.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>
#include <networkit/isomorphism/VF2.hpp>

#include "../SearchGraph.hpp"

namespace NetworKit {

class SearchGraphGTest : public testing::Test {};

TEST_F(SearchGraphGTest, testSearchGraphBasicFuncs) {

    Graph G = Graph(10, false, true);

    G.removeNode(5);
    G.removeNode(7);

    IsomorphismDetails::SearchGraph SG = IsomorphismDetails::SearchGraph(G, false);

    EXPECT_EQ(SG.numberOfNodes(), 8);
    EXPECT_EQ(SG.upperNodeIdBound(), 10);
    EXPECT_EQ(SG.isDirected(), true);
}

TEST_F(SearchGraphGTest, testSearchGraphCSRUndirected) {

    // Graph G is undirected
    Graph G = Graph(10);

    G.addEdge(3, 5);
    G.addEdge(9, 1);
    G.addEdge(5, 7);
    G.addEdge(1, 5);
    G.addEdge(5, 9);
    G.addEdge(5, 5);
    G.addEdge(2, 4);

    G.removeNode(0);
    G.removeNode(6);
    G.removeEdge(2, 4);

    IsomorphismDetails::SearchGraph SG = IsomorphismDetails::SearchGraph(G, false);

    // Check inDegrees = outDegrees, even nodes never have incident edges so none should be found
    G.forNodes([&](node u) {
        EXPECT_EQ(SG.inDegree(u), G.degreeIn(u));
        EXPECT_EQ(SG.outDegree(u), G.degreeOut(u));
        EXPECT_EQ(SG.outDegree(u), SG.inDegree(u));
        if (u % 2 == 0) {
            for (node v = 0; v < G.upperNodeIdBound(); v++) {
                EXPECT_FALSE(SG.hasEdge(u, v));
                EXPECT_FALSE(SG.hasEdge(v, u));
            }
        }
    });

    // All present edges must be found (oriented both ways)
    G.forEdges([&](node u, node v) {
        EXPECT_TRUE(SG.hasEdge(u, v));
        EXPECT_TRUE(SG.hasEdge(v, u));
    });

    // Check in and out slice for node 5
    EXPECT_EQ(std::vector<node>(SG.inBegin(5), SG.inEnd(5)), (std::vector<node>{1, 3, 5, 7, 9}));
    EXPECT_EQ(std::vector<node>(SG.outBegin(5), SG.outEnd(5)), (std::vector<node>{1, 3, 5, 7, 9}));
    EXPECT_EQ(std::vector<node>(SG.inBegin(5), SG.inEnd(5)),
              std::vector<node>(SG.outBegin(5), SG.outEnd(5)));
}

TEST_F(SearchGraphGTest, testSearchGraphCSRDirected) {

    // Graph G is directed
    Graph G = Graph(8, false, true);

    G.addEdge(0, 5);
    G.addEdge(0, 4);
    G.addEdge(0, 3);
    G.addEdge(0, 2);
    G.addEdge(0, 1);
    G.addEdge(0, 0);

    G.removeNode(4);
    G.removeNode(7);
    G.removeEdge(0, 5);

    IsomorphismDetails::SearchGraph SG = IsomorphismDetails::SearchGraph(G, false);

    // Check inDegrees and outDegrees
    G.forNodes([&](node u) {
        EXPECT_EQ(SG.inDegree(u), G.degreeIn(u));
        EXPECT_EQ(SG.outDegree(u), G.degreeOut(u));
    });

    EXPECT_NE(SG.inDegree(0), SG.outDegree(0));

    // All present edges must be found, explicitly check self loop at node 0
    G.forEdges([&](node u, node v) {
        EXPECT_TRUE(SG.hasEdge(u, v));
        if (u != v) {
            EXPECT_FALSE(SG.hasEdge(v, u));
        } else {
            EXPECT_TRUE(SG.hasEdge(v, u));
        }
    });

    // Non-present edges must not be found
    EXPECT_FALSE(SG.hasEdge(1, 2));
    EXPECT_FALSE(SG.hasEdge(0, 4));
    EXPECT_FALSE(SG.hasEdge(0, 5));

    // Check in and out slices
    EXPECT_EQ(std::vector<node>(SG.inBegin(0), SG.inEnd(0)), (std::vector<node>{0}));
    EXPECT_EQ(std::vector<node>(SG.outBegin(0), SG.outEnd(0)), (std::vector<node>{0, 1, 2, 3}));
    EXPECT_EQ(std::vector<node>(SG.inBegin(1), SG.inEnd(1)), (std::vector<node>{0}));
    EXPECT_EQ(std::vector<node>(SG.outBegin(1), SG.outEnd(1)), (std::vector<node>{}));

    // Deleted or isolated nodes have empty slice
    EXPECT_EQ(SG.inBegin(4), SG.inEnd(4));
    EXPECT_EQ(SG.inBegin(6), SG.inEnd(6));
}

TEST_F(SearchGraphGTest, testSearchGraphAdjMatrix) {

    // Graph G is directed and needs two 64-bit words per row
    Graph G = Graph(75, false, true);

    G.addEdge(1, 20);
    G.addEdge(42, 7);
    G.addEdge(50, 74);
    G.addEdge(38, 19);
    G.addEdge(25, 62);

    G.removeNode(38);
    G.removeEdge(25, 62);

    IsomorphismDetails::SearchGraph SG = IsomorphismDetails::SearchGraph(G, true);

    // All present edges must be found
    G.forEdges([&](node u, node v) {
        EXPECT_TRUE(SG.hasEdge(u, v));
        EXPECT_FALSE(SG.hasEdge(v, u));
    });

    // Non-present edges must not be found
    G.forNodes([&](node u) {
        if ((u != 1) && (u != 7) && (u != 20) && (u != 42) && (u != 50) && (u != 74)) {
            for (node v = 0; v < G.upperNodeIdBound(); v++) {
                EXPECT_FALSE(SG.hasEdge(u, v));
                EXPECT_FALSE(SG.hasEdge(v, u));
            }
        }
    });

    // Graph H is directed but has edge (v,u) for every edge (u,v) and needs one 64-bit word per row
    EdgeListReader reader('\t', 0, "#", true, true);
    Graph H = reader.read("input/example.edgelist");

    IsomorphismDetails::SearchGraph SH = IsomorphismDetails::SearchGraph(H, true);

    H.forEdges([&](node u, node v) {
        EXPECT_TRUE(SH.hasEdge(u, v));
        EXPECT_TRUE(SH.hasEdge(v, u));
    });
}

TEST_F(SearchGraphGTest, testCSRAdjEqualBehaviour) {

    METISGraphReader reader;
    Graph G = reader.read("input/karate.graph");

    IsomorphismDetails::SearchGraph S_CSR = IsomorphismDetails::SearchGraph(G, false);
    IsomorphismDetails::SearchGraph S_Adj = IsomorphismDetails::SearchGraph(G, true);

    for (node u = 0; u < G.upperNodeIdBound(); u++) {
        for (node v = 0; v < G.upperNodeIdBound(); v++) {
            EXPECT_EQ(S_CSR.hasEdge(u, v), S_Adj.hasEdge(u, v));
        }
    }
}

} // namespace NetworKit
