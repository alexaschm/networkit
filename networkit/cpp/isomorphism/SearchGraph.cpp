#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>

#include "SearchGraph.hpp"

namespace NetworKit {
namespace IsomorphismDetails {

namespace {

/**
 * Drop self-loops and collapse parallel edges in a sorted CSR, in place.
 *
 * Both are meaningless to a subgraph search - the two matching semantics only ever constrain
 * pairs of distinct nodes - and both actively break it if left in: a repeated neighbour makes the
 * search enumerate the same candidate twice and report the same match twice, and either one
 * inflates the degrees that every feasibility rule prunes on.
 *
 * The slices are already sorted, so equal entries are adjacent and one linear scan suffices.
 * Compaction happens in place because the write cursor can never overtake the read cursor.
 */
void compactSlices(std::vector<index> &first, std::vector<node> &head, count z) {
    index write = 0;
    for (node u = 0; u < z; ++u) {
        const index begin = first[u];
        const index end = first[u + 1];
        first[u] = write; // safe: `begin` was read before this overwrites it

        node previous = none;
        for (index i = begin; i < end; ++i) {
            const node v = head[i];
            if (v == u || v == previous)
                continue;
            previous = v;
            head[write++] = v;
        }
    }

    first[z] = write;
    head.resize(write);
}

} // namespace

SearchGraph::SearchGraph(const Graph &G, bool buildMatrix)
    : matrixStride(0), n(G.numberOfNodes()), z(G.upperNodeIdBound()), directed(G.isDirected()),
      hasMatrix(buildMatrix) {
    buildCSR(G);
    if (hasMatrix)
        buildAdjacencyMatrix(G);
}

void SearchGraph::buildCSR(const Graph &G) {
    // Count the out-degree of every node into outFirst[u + 1], then turn the counts into start
    // offsets with a prefix sum. A node that was removed keeps a degree of 0 and so ends up with
    // an empty slice, which is what lets callers skip hasNode() checks.
    outFirst.assign(z + 1, 0);
    for (node u = 0; u < z; ++u) {
        if (G.hasNode(u)) {
            outFirst[u + 1] = G.degreeOut(u);
        }
    }
    std::partial_sum(outFirst.begin(), outFirst.end(), outFirst.begin());

    // Place every neighbour into its node's slice. forEdges() visits an undirected edge once,
    // oriented u >= v, so the reverse orientation has to be added here - except for a self-loop,
    // which Graph stores only once and which degreeOut() therefore also counted only once.
    outHead.resize(outFirst[z]);
    std::vector<index> cursor = outFirst;
    G.forEdges([&](node u, node v) {
        outHead[cursor[u]++] = v;
        if (!directed && u != v) {
            outHead[cursor[v]++] = u;
        }
    });

    // hasEdge() binary-searches a slice and the feasibility rules intersect two of them, so both
    // rely on the order. Sorting over pointers rather than iterators keeps the offsets unsigned.
    for (node u = 0; u < z; ++u) {
        std::sort(outHead.data() + outFirst[u], outHead.data() + outFirst[u + 1]);
    }
    compactSlices(outFirst, outHead, z);

    // For an undirected graph inBegin()/inEnd() fall back to the out-arrays, so a second copy
    // would only waste memory.
    if (directed) {
        inFirst.assign(z + 1, 0);
        for (node u = 0; u < z; ++u) {
            if (G.hasNode(u)) {
                inFirst[u + 1] = G.degreeIn(u);
            }
        }
        std::partial_sum(inFirst.begin(), inFirst.end(), inFirst.begin());

        inHead.resize(inFirst[z]);
        std::vector<index> inCursor = inFirst;
        G.forEdges([&](node u, node v) { inHead[inCursor[v]++] = u; });

        for (node u = 0; u < z; ++u) {
            std::sort(inHead.data() + inFirst[u], inHead.data() + inFirst[u + 1]);
        }
        compactSlices(inFirst, inHead, z);
    }
}

void SearchGraph::buildAdjacencyMatrix(const Graph &G) {
    matrixStride = tlx::div_ceil(z, 64);
    matrix.assign(static_cast<std::size_t>(z) * matrixStride, 0);

    // Setting the same bit twice is harmless, so parallel edges need no handling here. Self-loops
    // do need it: buildCSR() drops them, and the two hasEdge() backends have to agree.
    G.forNodes([&](node u) {
        G.forNeighborsOf(u, [&](node v) {
            if (v == u)
                return;
            matrix[static_cast<std::size_t>(u) * matrixStride + v / 64] |= uint64_t{1} << (v % 64);
        });
    });
}

bool SearchGraph::hasEdge(node u, node v) const noexcept {
    if (hasMatrix)
        return (matrix[static_cast<std::size_t>(u) * matrixStride + v / 64] >> (v % 64)) & 1u;

    // The slices are sorted, so a binary search is the best we can do without the matrix.
    return std::binary_search(outBegin(u), outEnd(u), v);
}

} // namespace IsomorphismDetails
} // namespace NetworKit
