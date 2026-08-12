#ifndef NETWORKIT_CPP_ISOMORPHISM_SEARCH_GRAPH_HPP_
#define NETWORKIT_CPP_ISOMORPHISM_SEARCH_GRAPH_HPP_

// Private header of the isomorphism module. Not installed, not part of the public API.

#include <cstdint>
#include <vector>

#include <networkit/Globals.hpp>
#include <networkit/graph/Graph.hpp>

namespace NetworKit {
namespace IsomorphismDetails {

/**
 * A read-only snapshot of a Graph, laid out for the innermost loop of a subgraph search.
 *
 * ## Why this exists
 *
 * `Graph::hasEdge(u, v)` walks the adjacency list of @a u looking for @a v. That is a linear
 * scan. A subgraph search asks that question millions of times per second, in its innermost
 * loop, so calling `Graph::hasEdge()` there would dominate the running time. Every algorithm in
 * this module therefore builds one of these snapshots first and works only against it.
 *
 * ## What it stores
 *
 * Two things:
 *
 * - **CSR adjacency**, separately for out-edges and in-edges. "CSR" means the neighbours of all
 *   nodes are packed back to back into one flat array (`outHead`), and a second array
 *   (`outFirst`) says where each node's slice starts. The neighbours of node @a u are then
 *   `outHead[outFirst[u] .. outFirst[u + 1])`. This is compact, cache-friendly, and iterating it
 *   costs one pointer bump per neighbour. The slices are kept **sorted**, which makes
 *   @ref hasEdge() a binary search and makes intersecting two neighbourhoods cheap.
 *
 *   For an undirected graph the in-edges are the same as the out-edges, so only one copy is
 *   built and the in-accessors return the out-slices.
 *
 * - Optionally a **bit-packed adjacency matrix**, one bit per ordered node pair. This answers
 *   @ref hasEdge() in constant time, but needs `upperNodeIdBound()^2` bits, so it is only ever
 *   built for the *pattern*, which is small by assumption. Building it for a target with a
 *   million nodes would need 125 GB.
 *
 * ## Ownership
 *
 * The snapshot copies what it needs, so it stays valid even if nobody holds on to the original
 * `Graph`. It is immutable once built, which is what lets @ref ParallelRI share one snapshot
 * across all its workers without any synchronization.
 */
class SearchGraph {

public:
    /**
     * Build the snapshot.
     *
     * @param G The graph to snapshot. Only read, never stored.
     * @param buildMatrix Whether to also build the bit-packed adjacency matrix. Pass true for the
     *        pattern, false for the target. Passing true for a large graph will exhaust memory.
     */
    SearchGraph(const Graph &G, bool buildMatrix);

    /// Number of nodes that actually exist.
    count numberOfNodes() const noexcept { return n; }

    /// One past the largest node id. Node ids are not necessarily contiguous, so this can be
    /// larger than @ref numberOfNodes(); all the arrays here are sized by this.
    count upperNodeIdBound() const noexcept { return z; }

    bool isDirected() const noexcept { return directed; }

    /// First out-neighbour of @a u. Iterate up to @ref outEnd(). The range is sorted ascending.
    const node *outBegin(node u) const noexcept { return outHead.data() + outFirst[u]; }

    /// One past the last out-neighbour of @a u.
    const node *outEnd(node u) const noexcept { return outHead.data() + outFirst[u + 1]; }

    /// First in-neighbour of @a u. For an undirected graph this is the same as @ref outBegin().
    const node *inBegin(node u) const noexcept {
        return directed ? inHead.data() + inFirst[u] : outBegin(u);
    }

    /// One past the last in-neighbour of @a u.
    const node *inEnd(node u) const noexcept {
        return directed ? inHead.data() + inFirst[u + 1] : outEnd(u);
    }

    count outDegree(node u) const noexcept { return outFirst[u + 1] - outFirst[u]; }

    count inDegree(node u) const noexcept {
        return directed ? inFirst[u + 1] - inFirst[u] : outDegree(u);
    }

    /**
     * Whether the edge @a u -> @a v exists.
     *
     * Constant time when the snapshot was built with the adjacency matrix, otherwise a binary
     * search over the sorted out-neighbours of @a u. Either way this is safe to call from an
     * inner loop, unlike `Graph::hasEdge()`.
     */
    bool hasEdge(node u, node v) const noexcept;

private:
    /**
     * Fill outFirst/outHead, and inFirst/inHead when the graph is directed.
     *
     * TODO: implement.
     *  1. Size outFirst to z + 1 and count the out-degree of every node into it, then turn those
     *     counts into start offsets with a prefix sum. Nodes that do not exist get a degree of 0,
     *     which leaves them with an empty slice - that is intentional and means callers do not
     *     have to check hasNode().
     *  2. Walk the edges a second time and place each neighbour into its node's slice.
     *  3. Sort each slice ascending. hasEdge() and the neighbourhood intersections rely on this.
     *  4. Repeat for the in-edges if the graph is directed; skip it otherwise, since inBegin()
     *     and friends fall back to the out-arrays.
     */
    void buildCSR(const Graph &G);

    /**
     * Fill the bit-packed adjacency matrix.
     *
     * TODO: implement.
     *  1. Set matrixStride to the number of 64-bit words per row, i.e. (z + 63) / 64, and size
     *     `matrix` to z * matrixStride words, zero-initialized.
     *  2. For every edge u -> v set bit v of row u; for an undirected graph set both directions
     *     so that hasEdge() does not have to normalize its arguments.
     */
    void buildAdjacencyMatrix(const Graph &G);

    /// CSR out-edges. outHead[outFirst[u] .. outFirst[u + 1]) are the out-neighbours of u.
    std::vector<index> outFirst;
    std::vector<node> outHead;

    /// CSR in-edges. Empty for undirected graphs.
    std::vector<index> inFirst;
    std::vector<node> inHead;

    /// Bit per ordered node pair, row-major, matrixStride words per row. Empty if not requested.
    std::vector<uint64_t> matrix;
    count matrixStride;

    count n;
    count z;
    bool directed;
    bool hasMatrix;
};

} // namespace IsomorphismDetails
} // namespace NetworKit

#endif // NETWORKIT_CPP_ISOMORPHISM_SEARCH_GRAPH_HPP_
