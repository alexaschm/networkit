#include <algorithm>

#include <tlx/unused.hpp>

#include "SearchGraph.hpp"

namespace NetworKit {
namespace IsomorphismDetails {

SearchGraph::SearchGraph(const Graph &G, bool buildMatrix)
    : matrixStride(0), n(G.numberOfNodes()), z(G.upperNodeIdBound()), directed(G.isDirected()),
      hasMatrix(buildMatrix) {
    buildCSR(G);
    if (hasMatrix)
        buildAdjacencyMatrix(G);
}

void SearchGraph::buildCSR(const Graph &G) {
    // Leave the object in a valid, empty state so that every accessor stays in bounds while the
    // real implementation is missing: one offset per node plus the sentinel, all zero, means
    // every node has an empty neighbour slice.
    outFirst.assign(z + 1, 0);
    if (directed)
        inFirst.assign(z + 1, 0);

    // TODO: fill the CSR arrays.
    //  1. Count the out-degree of every node into outFirst[u + 1], then prefix-sum outFirst so
    //     that outFirst[u] becomes the start of u's slice and outFirst[z] the total edge count.
    //     Nodes that do not exist keep a degree of 0 and end up with an empty slice, which is
    //     what lets callers skip hasNode() checks.
    //  2. Size outHead to outFirst[z] and walk the edges again with G.forEdges(), writing each
    //     neighbour into its node's slice.
    //  3. Sort every slice ascending. hasEdge() binary-searches it and the feasibility rules
    //     intersect two slices, so both rely on the order.
    //  4. Repeat steps 1-3 for the in-edges into inFirst/inHead, but only when `directed`. For an
    //     undirected graph inBegin()/inEnd() fall back to the out-arrays, so building a second
    //     copy would just waste memory.
    // See the note in SearchGraph.hpp for which parts of NetworKit to follow here, and for why
    // Graph::sortEdges() must not be used for step 3.
    tlx::unused(G, outHead, inHead);
}

void SearchGraph::buildAdjacencyMatrix(const Graph &G) {
    // As above: a correctly sized, all-zero matrix is a valid "no edges" answer, so hasEdge()
    // cannot read out of bounds while this is unimplemented.
    matrixStride = (z + 63) / 64;
    matrix.assign(static_cast<std::size_t>(z) * matrixStride, 0);

    // TODO: set one bit per edge.
    //  1. For every edge u -> v, set bit v of row u:
    //         matrix[u * matrixStride + v / 64] |= uint64_t{1} << (v % 64);
    //  2. For an undirected graph set both u -> v and v -> u, so that hasEdge() never has to
    //     normalize the order of its arguments.
    tlx::unused(G);
}

bool SearchGraph::hasEdge(node u, node v) const noexcept {
    if (hasMatrix)
        return (matrix[static_cast<std::size_t>(u) * matrixStride + v / 64] >> (v % 64)) & 1u;

    // The slices are sorted, so a binary search is the best we can do without the matrix.
    return std::binary_search(outBegin(u), outEnd(u), v);
}

} // namespace IsomorphismDetails
} // namespace NetworKit
