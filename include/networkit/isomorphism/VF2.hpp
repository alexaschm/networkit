#ifndef NETWORKIT_ISOMORPHISM_VF2_HPP_
#define NETWORKIT_ISOMORPHISM_VF2_HPP_

#include <networkit/Globals.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {

/**
 * @ingroup isomorphism
 * Finds every occurrence of a pattern graph inside a target graph, using the VF2 algorithm.
 *
 * See @ref SubgraphIsomorphism for what "occurrence" means, how to use the class, and how to
 * choose between the four algorithms in this module.
 *
 * ## How VF2 works
 *
 * VF2 grows a **partial mapping** one node at a time, depth first. It starts with nothing mapped
 * and repeatedly picks an unmapped pattern node together with a candidate target node for it. If
 * that pair is consistent with everything mapped so far, it is added to the mapping and the
 * search descends. If not, the pair is discarded and the next one is tried. When every pattern
 * node is mapped, that is a match. When no candidate works, the search backs up and undoes the
 * last pair.
 *
 * The trick that makes this fast enough is the **terminal sets**. At any point during the search,
 * the interesting candidates are not all unmapped nodes, but only those adjacent to something
 * already mapped - because a connected pattern has to grow outward from what you already have.
 * VF2 keeps four such sets: pattern nodes reachable by an out-edge from the mapping, pattern
 * nodes reachable by an in-edge, and the same two for the target. Candidates are drawn from these
 * sets, which is dramatically smaller than the whole graph.
 *
 * Before accepting a pair, VF2 applies several cheap **feasibility rules**. Some check
 * consistency with what is already mapped: if pattern node @a u has an edge to an already-mapped
 * node, the target node it is being paired with must have the corresponding edge too. Others look
 * one step ahead: if @a u has more terminal-set neighbours than its candidate does, the mapping
 * can never be completed, so the pair can be rejected immediately. Rejecting early is what avoids
 * exploring a doomed subtree.
 *
 * VF2 picks the next pattern node **dynamically**, from whatever the current state looks like.
 * That is the main difference to its relatives: @ref VF3 computes one fixed order up front and
 * groups nodes into classes, and @ref RI computes a fixed order too but keeps no candidate sets
 * at all.
 *
 * ## When to use it
 *
 * VF2 is the oldest and simplest of the four and is usually beaten by @ref VF3 and @ref RI on
 * anything large or dense. Its value here is that it is short, deterministic and easy to reason
 * about, which makes it the reference implementation the others are validated against. Reach for
 * it on small inputs, or when you want an answer you can trust without thinking hard.
 *
 * ## What it supports
 *
 * Both semantics, directed and undirected graphs, and optional node labels via
 * @ref setNodeLabels() - with labels, a pair is only feasible if the two labels agree.
 *
 * **Edge** labels are refused, not honoured: @ref run() throws if @ref setEdgeLabels() was used,
 * because VF2's feasibility rules compare node labels only and would otherwise report matches that
 * violate the edge labels asked for. @ref RI is where edge-label support in this module is going.
 *
 * The search polls `Aux::SignalHandler` regularly, so a long enumeration can be stopped with
 * CTRL+C.
 *
 * The implementation is based on
 *
 * Cordella, L. P., Foggia, P., Sansone, C., & Vento, M. (2004).
 * A (Sub)Graph Isomorphism Algorithm for Matching Large Graphs.
 * IEEE Transactions on Pattern Analysis and Machine Intelligence, 26(10), 1367-1372.
 */
class VF2 final : public SubgraphIsomorphism {

public:
    /**
     * @param pattern The graph to look for. Must not contain self-loops.
     * @param target The graph to look in. Must agree with @a pattern on directedness.
     * @param semantics Whether matches must be induced. See @ref SubgraphIsomorphism::Semantics.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     */
    VF2(const Graph &pattern, const Graph &target, Semantics semantics = Semantics::INDUCED,
        count maxMatches = 0);

    /**
     * Run the search. Retrieve the results with @ref getMatches(), @ref numberOfMatches() or
     * @ref hasMatch().
     */
    void run() override;
};

} // namespace NetworKit

#endif // NETWORKIT_ISOMORPHISM_VF2_HPP_
