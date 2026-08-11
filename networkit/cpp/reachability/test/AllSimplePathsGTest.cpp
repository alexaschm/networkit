/*
 * AllSimplePathsGTest.cpp
 *
 *  Created on: 27.06.2017
 *      Author: Eugenio Angriman
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <networkit/auxiliary/Random.hpp>
#include <networkit/io/EdgeListReader.hpp>
#include <networkit/reachability/AllSimplePaths.hpp>

namespace NetworKit {

class AllSimplePathsGTest : public testing::Test {};

TEST_F(AllSimplePathsGTest, testAllSimplePaths) {
    EdgeListReader reader('\t', 0, "#", true, true);
    Graph G = reader.read("input/example.edgelist");

    AllSimplePaths allSimplePaths(G, 1, 9);
    EXPECT_ANY_THROW(allSimplePaths.run());

    G.addEdge(9, 6);
    G.addEdge(6, 9);
    AllSimplePaths allSimplePaths2(G, 3, 1);
    EXPECT_NO_THROW(allSimplePaths2.run());

    ASSERT_EQ(allSimplePaths2.numberOfSimplePaths(), 4);
    std::vector<node> path1{3, 7, 10, 9, 6, 1};
    std::vector<node> path2{3, 7, 10, 9, 6, 5, 1};
    std::vector<node> path3{3, 10, 9, 6, 1};
    std::vector<node> path4{3, 10, 9, 6, 5, 1};
    std::vector<std::vector<node>> results{path1, path2, path3, path4};

    allSimplePaths2.parallelForAllSimplePaths([&](const std::vector<node> &p) {
        ASSERT_TRUE(std::find(results.begin(), results.end(), p) != results.end());
    });

    std::vector<std::vector<node>> paths = allSimplePaths2.getAllSimplePaths();

    // backwards check
    for (count i = 0; i < results.size(); i++)
        ASSERT_TRUE(std::find(paths.begin(), paths.end(), results[i]) != paths.end());
}

TEST_F(AllSimplePathsGTest, testAllSimplePathsWeighted) {

    // same graph as in unweighted test, now weighted
    Graph G = Graph(11, true, true);
    std::vector<node> startNodes = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 10};
    std::vector<node> endNodes = {5, 6, 4, 8, 7, 10, 2, 8, 1, 6, 1, 5, 3, 10, 2, 4, 10, 3, 7, 9};
    std::vector<edgeweight> weights = {14, 3, 19, 8, 6, 17, 11, 2, 20, 9, 5, 13, 1, 16, 7, 18, 4, 12, 10, 15};

    for (index i = 0; i < startNodes.size(); ++i) {
        G.setWeight(startNodes[i], endNodes[i], weights[i]);
    }

    AllSimplePaths allSimplePaths(G, 1, 9);
    EXPECT_ANY_THROW(allSimplePaths.run());

    G.addEdge(9, 6);
    G.addEdge(6, 9);
    AllSimplePaths allSimplePaths2(G, 3, 1);
    EXPECT_NO_THROW(allSimplePaths2.run());

    ASSERT_EQ(allSimplePaths2.numberOfSimplePaths(), 4);
    std::vector<node> path1{3, 7, 10, 9, 6, 1};
    std::vector<node> path2{3, 7, 10, 9, 6, 5, 1};
    std::vector<node> path3{3, 10, 9, 6, 1};
    std::vector<node> path4{3, 10, 9, 6, 5, 1};
    std::vector<std::vector<node>> results{path1, path2, path3, path4};

    allSimplePaths2.parallelForAllSimplePaths([&](const std::vector<node> &p) {
        ASSERT_TRUE(std::find(results.begin(), results.end(), p) != results.end());
    });

    std::vector<std::vector<node>> paths = allSimplePaths2.getAllSimplePaths();

    // backwards check
    for (count i = 0; i < results.size(); i++)
        ASSERT_TRUE(std::find(paths.begin(), paths.end(), results[i]) != paths.end());
}


TEST_F(AllSimplePathsGTest, testAllSimplePathsWeightedCutoff) {

    Graph G = Graph(8, true, true);

    // disjoint paths of length 3 and 5 between nodes 0 and 7
    G.setWeight(0, 1, 2);
    G.setWeight(1, 2, 7);
    G.setWeight(2, 7, 5);

    G.setWeight(0, 3, 3);
    G.setWeight(3, 4, 3);
    G.setWeight(4, 5, 1);
    G.setWeight(5, 6, 4);
    G.setWeight(6, 7, 9);

    // test cutoff 2, nodes are not connected
    AllSimplePaths allSimplePaths2(G, 0, 7, 2);
    EXPECT_ANY_THROW(allSimplePaths2.run());

    // test cutoff 4 and 6
    AllSimplePaths allSimplePaths4(G, 0, 7, 4);
    EXPECT_NO_THROW(allSimplePaths4.run());

    AllSimplePaths allSimplePaths6(G, 0, 7, 6);
    EXPECT_NO_THROW(allSimplePaths6.run());

    ASSERT_EQ(allSimplePaths4.numberOfSimplePaths(), 1);
    ASSERT_EQ(allSimplePaths6.numberOfSimplePaths(), 2);

}

} /* namespace NetworKit */
