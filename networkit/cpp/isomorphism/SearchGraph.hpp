#ifndef NETWORKIT_CPP_ISOMORPHISM_SEARCH_GRAPH_HPP_
#define NETWORKIT_CPP_ISOMORPHISM_SEARCH_GRAPH_HPP_

// Private header of the isomorphism module. Not installed, not part of the public API.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <tlx/math/div_ceil.hpp>
#include <networkit/Globals.hpp>
#include <networkit/graph/Graph.hpp>

namespace NetworKit {
namespace IsomorphismDetails {

/**
 * How many values occur in both of two sorted, strictly ascending ranges.
 *
 * Every adjacency slice handed out by @ref SearchGraph has that shape, so this is a plain linear
 * merge rather than a lookup structure. It takes raw ranges instead of two nodes so that it also
 * serves the case where only one side is a slice - intersecting a target neighbourhood with a
 * candidate domain, for instance, which is what RI-DS does on every step.
 */
inline count intersectionSize(const node *aBegin, const node *aEnd, const node *bBegin,
                              const node *bEnd) noexcept {
    count common = 0;
    while (aBegin != aEnd && bBegin != bEnd) {
        if (*aBegin < *bBegin) {
            ++aBegin;
        } else if (*bBegin < *aBegin) {
            ++bBegin;
        } else {
            ++common;
            ++aBegin;
            ++bBegin;
        }
    }
    return common;
}

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
 * - Optionally a **label per arc**, at the same offset as the arc's head. Ask for it by handing
 *   the constructor a label vector indexed by edge id; the snapshot re-indexes it into CSR order
 *   once, so the search never has to go back to `Graph::edgeId()`. Because `outLabel[i]` belongs
 *   to `outHead[i]`, filtering a slice by edge label costs one comparison per candidate and no
 *   extra lookup. See @ref edgeLabel() and @ref outLabelBegin().
 *
 * - A bit per id saying **whether it is a node at all**, behind @ref hasNode(). Removed ids keep
 *   an empty slice and so look exactly like isolated nodes; a search enumerating "every unmapped
 *   target node" has to be able to tell the two apart or it maps pattern nodes onto ids that do
 *   not exist. Alongside it, @ref maxOutDegree() / @ref maxInDegree() over the finished slices,
 *   so the cheap "this pattern cannot possibly fit" test does not have to rescan the graph.
 *
 * ## What it normalizes away
 *
 * The snapshot is the **simple graph underlying** @a G: parallel edges are collapsed to one, and
 * self-loops are dropped. So a slice never repeats a neighbour and never contains its own node,
 * @ref outDegree() / @ref inDegree() are simple-graph degrees rather than `Graph::degree()`, and
 * @ref hasEdge(u, u) is always false.
 *
 * This is not a convenience, it is what makes the search correct. Both matching semantics only
 * ever constrain pairs of *distinct* nodes, so neither multiplicity nor a self-loop can change
 * which matches exist - but every algorithm in this module enumerates candidates by walking a
 * slice, so a repeated neighbour would make it try the same candidate twice and report the same
 * match twice. Worse, all of them prune by comparing degrees and neighbourhood cardinalities,
 * which are *set* sizes: an inflated degree on the pattern side makes valid host nodes fail the
 * `deg(target) >= deg(pattern)` test and silently discards real matches. `Graph::addEdge()`
 * permits parallel edges by default, so this is reachable without doing anything unusual.
 *
 * Collapsing is lossless for structure, but not for edge labels: two parallel arcs carrying
 * *different* labels become one arc that can only carry one of them. The snapshot does not try to
 * paper over that. It collapses as usual and sets @ref collapsedLabelledEdges(), which is the flag
 * every algorithm checks before it starts searching, so the caller gets a refusal instead of an
 * answer to a question they did not ask. Parallel arcs whose labels *agree* collapse losslessly
 * and raise nothing.
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
     *        pattern, false for the target. This is a *request*, not a demand: it is honoured
     *        only while @ref upperNodeIdBound() stays small, and quietly dropped otherwise, since
     *        the matrix is an accelerator that correctness never depends on. Ask
     *        @ref hasAdjacencyMatrix() what actually happened.
     * @param edgeLabels One label per edge of @a G, indexed by **edge id**, or empty for an
     *        unlabelled snapshot. @a G must then have `hasEdgeIds()`, and the vector must be at
     *        least `upperEdgeIdBound()` long; both are what
     *        `SubgraphIsomorphism::setEdgeLabels()` already checks, and both throw here too so
     *        that a snapshot built any other way fails loudly rather than reading past its input.
     */
    SearchGraph(const Graph &G, bool buildMatrix, const std::vector<index> &edgeLabels = {});

    /// Number of nodes that actually exist.
    count numberOfNodes() const noexcept { return n; }

    /// One past the largest node id. Node ids are not necessarily contiguous, so this can be
    /// larger than @ref numberOfNodes(); all the arrays here are sized by this.
    count upperNodeIdBound() const noexcept { return z; }

    /**
     * Whether node @a u exists in the snapshotted graph.
     *
     * `Graph::removeNode()` leaves the id reserved and never lowers @ref upperNodeIdBound(), so a
     * search that walks ids from 0 to the bound will meet ids that are not nodes. In the snapshot
     * those are indistinguishable from isolated nodes by adjacency alone - both have an empty
     * slice - so this is the only way to tell them apart, and a search that skips the check will
     * happily map a pattern node onto an id that no longer exists.
     *
     * @a u must be below @ref upperNodeIdBound().
     */
    bool hasNode(node u) const noexcept { return nodeExists[u]; }

    bool isDirected() const noexcept { return directed; }

    /// Largest @ref outDegree() over all nodes, or 0 if there are none. A *distinct*-neighbour
    /// count, so this is not `GraphTools::maxDegree()` when the graph has parallel edges or loops.
    count maxOutDegree() const noexcept { return maxOut; }

    /// Largest @ref inDegree() over all nodes. Same as @ref maxOutDegree() when undirected.
    count maxInDegree() const noexcept { return directed ? maxIn : maxOut; }

    /// Whether the bit-packed adjacency matrix was actually built.
    bool hasAdjacencyMatrix() const noexcept { return hasMatrix; }

    /// First out-neighbour of @a u. Iterate up to @ref outEnd(). The range is sorted **strictly**
    /// ascending: no duplicates, and @a u itself is never in it.
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

    /// Number of *distinct* out-neighbours of @a u, not counting @a u itself. This is not
    /// `Graph::degreeOut(u)` when @a G has parallel edges or a self-loop at @a u.
    count outDegree(node u) const noexcept { return outFirst[u + 1] - outFirst[u]; }

    /// Number of *distinct* in-neighbours of @a u, not counting @a u itself.
    count inDegree(node u) const noexcept {
        return directed ? inFirst[u + 1] - inFirst[u] : outDegree(u);
    }

    /**
     * Whether the edge @a u -> @a v exists.
     *
     * Constant time when the snapshot was built with the adjacency matrix, otherwise a binary
     * search over the sorted out-neighbours of @a u. Either way this is safe to call from an
     * inner loop, unlike `Graph::hasEdge()`.
     *
     * Always false for `u == v`, whether or not @a G had a self-loop there. Both @a u and @a v
     * must be below @ref upperNodeIdBound(); a node that was *removed* from @a G is fine and
     * simply has no edges, but an id beyond the bound is undefined.
     */
    bool hasEdge(node u, node v) const noexcept {
        if (hasMatrix)
            return (matrix[static_cast<std::size_t>(u) * matrixStride + v / 64] >> (v % 64)) & 1u;

        // The slices are sorted, so a binary search is the best we can do without the matrix.
        return std::binary_search(outBegin(u), outEnd(u), v);
    }

    /// How many nodes are out-neighbours of both @a u and @a v. See @ref intersectionSize().
    count commonOutNeighbors(node u, node v) const noexcept {
        return intersectionSize(outBegin(u), outEnd(u), outBegin(v), outEnd(v));
    }

    /// Whether the snapshot was built with a label per arc.
    bool hasEdgeLabels() const noexcept { return !outLabel.empty(); }

    /**
     * Whether collapsing parallel edges threw a label away.
     *
     * True only when a run of repeated neighbours carried labels that were *not* all equal, so
     * that the one arc left over cannot stand for all of them. Collapsing equally-labelled
     * parallel edges really is lossless and leaves this false, as does any graph without parallel
     * edges - and so does an unlabelled snapshot, which has no labels to lose.
     *
     * An algorithm that honours edge labels must check this once, after building its snapshots and
     * before searching, and refuse the input rather than answer a different question. Self-loops
     * are not involved: they are dropped whatever their labels, and no matching rule ever looks at
     * one.
     */
    bool collapsedLabelledEdges() const noexcept { return lostLabels; }

    /**
     * The label of the arc @a u -> @a v, or @ref none if there is no such arc or the snapshot is
     * unlabelled.
     *
     * A binary search over the sorted out-slice, even when the bit matrix is available: the matrix
     * answers *whether* an arc exists but knows no offset, and the offset is what the label sits
     * at. Patterns are small, so the extra `log deg` is affordable; if it ever is not, the fix is a
     * second matrix-indexed offset table rather than a different layout here.
     *
     * A directed mutual pair is two arcs with two ids in two different slices, so `edgeLabel(u, v)`
     * and `edgeLabel(v, u)` are independent. For an undirected graph the one edge id was written
     * into both endpoints' slices, so they agree.
     */
    index edgeLabel(node u, node v) const noexcept {
        if (outLabel.empty())
            return none;

        const node *begin = outBegin(u);
        const node *end = outEnd(u);
        const node *found = std::lower_bound(begin, end, v);
        if (found == end || *found != v)
            return none;

        return outLabel[outFirst[u] + static_cast<index>(found - begin)];
    }

    /// Labels of the out-arcs of @a u, one per entry of the @ref outBegin() slice and in the same
    /// order. Null when the snapshot is unlabelled.
    const index *outLabelBegin(node u) const noexcept {
        return outLabel.empty() ? nullptr : outLabel.data() + outFirst[u];
    }

    /// Labels of the in-arcs of @a u, one per entry of the @ref inBegin() slice. For an undirected
    /// graph this is the same as @ref outLabelBegin().
    const index *inLabelBegin(node u) const noexcept {
        if (!directed)
            return outLabelBegin(u);
        return inLabel.empty() ? nullptr : inLabel.data() + inFirst[u];
    }

private:
    /**
     * Fill outFirst/outHead, and inFirst/inHead when the graph is directed.
     *
     * Counts degrees into the offset array, prefix-sums it into start offsets, scatters the
     * neighbours, sorts each slice, then compacts the slices to drop self-loops and collapse
     * parallel edges. The degrees taken from `Graph` are an upper bound on the final slice sizes;
     * the compaction pass shrinks the arrays to the true size.
     *
     * Also fills `nodeExists` while it is already asking `Graph` which ids are nodes, and records
     * `maxOut`/`maxIn` once the slices are final - taking those maxima before compaction would
     * count parallel edges and self-loops, which is exactly the number every caller must not have.
     *
     * Reuse: NetworKit has no CSR graph class to inherit from, so this has to be written, but the
     * count/prefix-sum/scatter shape is established in the repo - see
     * ParallelPartitionCoarsening.cpp, and MaximalCliques.cpp for a CSR kept under the same
     * firstOut/head names used here.
     *
     * Two things that look like shortcuts and are not: Graph::sortEdges() would sort the input in
     * place, but it mutates the caller's graph and allocates a full parallel copy of the whole
     * adjacency structure, so the sort has to run on this snapshot's own slices. And
     * CSRGeneralMatrix::adjacencyMatrix() really is a sorted CSR, but it stores a double per
     * entry with algebraic semantics, which is far more machinery than a boolean neighbour list
     * needs.
     *
     * With edge labels this pass also fills outLabel/inLabel. The four-argument `forEdges()`
     * overload hands out the edge id per arc, so the label array is scattered by the same cursor
     * that places the head - which is what makes `outLabel[i]` the label of `outHead[i]`. The sort
     * and the compaction then have to move the two arrays *together*; sorting the heads alone
     * would leave every arc holding some other arc's label, with nothing to warn anybody.
     *
     * @param edgeLabels Labels indexed by edge id, or empty for an unlabelled snapshot.
     */
    void buildCSR(const Graph &G, const std::vector<index> &edgeLabels);

    /**
     * Fill the bit-packed adjacency matrix: matrixStride 64-bit words per row, bit v of row u set
     * iff the edge u -> v exists. For an undirected graph both directions are set, so hasEdge()
     * never has to normalize the order of its arguments.
     *
     * Reads the finished CSR rather than @a G, so it must run after @ref buildCSR(). That is what
     * makes the two hasEdge() backends agree by construction: the slices have already had their
     * self-loops dropped and their parallel edges collapsed, so there is nothing left here to
     * normalize away, and no second walk of `Graph`'s adjacency to pay for. An undirected snapshot
     * keeps both orientations of every edge in its out-slices, which is where "both directions are
     * set" comes from.
     *
     * Whether the matrix is affordable at all is decided by the constructor.
     *
     */
    void buildAdjacencyMatrix();

    /// CSR out-edges. outHead[outFirst[u] .. outFirst[u + 1]) are the out-neighbours of u.
    std::vector<index> outFirst;
    std::vector<node> outHead;

    /// CSR in-edges. Empty for undirected graphs.
    std::vector<index> inFirst;
    std::vector<node> inHead;

    /// One label per arc, at the same offset as its head in outHead/inHead. Both empty unless the
    /// constructor was given edge labels; inLabel additionally empty when undirected, where
    /// inLabelBegin() falls back to the out-arrays exactly as inBegin() does.
    std::vector<index> outLabel;
    std::vector<index> inLabel;

    /// Whether collapsing parallel arcs discarded a label that differed from the one kept.
    bool lostLabels;

    /// Whether each id below z is a node. Sized z, so a removed id can be told from an isolated
    /// one; both have an empty slice.
    std::vector<bool> nodeExists;

    /// Maxima over the *compacted* slices, so they are distinct-neighbour counts. maxIn is left
    /// at 0 for undirected graphs, where maxInDegree() returns maxOut instead.
    count maxOut;
    count maxIn;

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
