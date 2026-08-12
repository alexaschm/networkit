#ifndef NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_
#define NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_

// Private header of the isomorphism module. Not installed, not part of the public API.
//
// Unlike VF2Impl and VF3Impl, which are hidden inside their own .cpp files, this one needs a
// header: RI.cpp runs one of these to do the whole search, and ParallelRI.cpp runs one per
// worker. Two translation units, so the class has to be visible to both. The same reason
// networkit/cpp/randomization/CurveballImpl.hpp and
// networkit/cpp/centrality/GroupClosenessGrowShrinkImpl.hpp exist.

#include <vector>

#include <networkit/Globals.hpp>
#include <networkit/isomorphism/RI.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

#include "SearchGraph.hpp"

namespace NetworKit {
namespace IsomorphismDetails {

/**
 * The RI search core, written so that it can be driven sequentially or in parallel.
 *
 * ## How it is split
 *
 * The expensive preparation - working out the order in which pattern nodes get mapped - depends
 * only on the two graphs, not on where the search currently is. It is therefore a @b static
 * function producing an @ref Ordering, which the caller computes once. @ref RI computes it and
 * uses it itself; @ref ParallelRI computes it once and shares the same read-only object with
 * every worker.
 *
 * The search then comes in two flavours:
 *
 * - @ref run() does the whole thing, from the empty mapping to the last match, recursively. This
 *   is what @ref RI calls.
 * - @ref expand() takes one @ref State and produces its children, one level at a time. This is
 *   what a parallel worker calls, because a worker needs to be able to hand a half-finished
 *   piece of the search tree to another worker, and you cannot hand over a C++ call stack.
 *
 * Both share every rule below, so the two algorithms cannot drift apart.
 *
 * ## Why there is no bookkeeping
 *
 * @ref VF2 maintains terminal sets and @ref VF3 maintains per-class counters, both updated on
 * every step and undone on every backtrack. RI maintains nothing: the matching order already
 * encodes which nodes constrain which, so at each depth the candidates follow directly from the
 * order and from what is already mapped. Each step is therefore very cheap, and RI wins by taking
 * many cheap steps rather than few expensive ones.
 *
 * The one exception is @ref RI::Variant::RI_DS, which keeps candidate domains so it can do
 * forward checking. Those live in `domains` and are only touched in that variant.
 */
class RIImpl {

public:
    /**
     * The fixed order in which pattern nodes are mapped. Computed once, then read-only.
     *
     * `order[i]` is the pattern node mapped at depth @a i. `parent[i]` is the *position* in
     * `order` of an already-ordered neighbour of `order[i]`, or @ref none when position @a i
     * starts a new connected component. The parent is what lets the search enumerate candidates
     * from one target neighbourhood instead of from the entire target.
     */
    struct Ordering {
        std::vector<node> order;
        std::vector<index> parent;
    };

    /**
     * One partial mapping: a position in the search tree.
     *
     * This is the unit of work @ref ParallelRI steals between its workers, which is why it is a
     * plain value rather than recursion state. `mapping` is indexed by *position in the order*,
     * not by pattern node, so `mapping[i]` is the target node that `order[i]` was mapped to.
     * `depth` is how many positions are filled, and `nextCandidate` is where to resume in the
     * candidate list, so a partially explored state can be put down and picked up again.
     */
    struct State {
        std::vector<node> mapping;
        count depth = 0;
        index nextCandidate = 0;
    };

    /**
     * Work out the order in which the pattern nodes should be mapped.
     *
     * Static because it depends on nothing but the inputs; ParallelRI computes it once and hands
     * the result to every worker.
     *
     * TODO: implement.
     *  1. Start with an empty order. Repeatedly pick the unmapped pattern node that has the most
     *     edges into the nodes already in the order. Ties go to the node with the higher degree.
     *     Under RI_DS, ties instead go to the node with the smaller candidate domain, which is a
     *     better predictor of how much the node will prune.
     *  2. If the pattern is disconnected, the greedy step will at some point find no node with
     *     any edge into the prefix. Pick the best remaining node by degree alone and carry on;
     *     that position gets `none` as its parent.
     *  3. Fill parent[i] with the position of any already-ordered neighbour of order[i]. Any one
     *     will do for correctness; preferring the earliest keeps the candidate lists small.
     *
     * Reuse: the greedy pattern-side loop is the core of RI and has to be written here. The target
     * is only consulted for the RI_DS domain estimate, and if that estimate ever wants a
     * structural ordering to lean on, CoreDecomposition::getNodeOrder() gives a degeneracy
     * ordering for the cost of three lines - MaximalCliques.cpp shows the call.
     *
     * @param pattern Snapshot of the pattern.
     * @param target Snapshot of the target, used for the RI_DS domain estimate.
     * @param patternLabels Empty when unlabelled.
     * @param targetLabels Empty when unlabelled.
     * @param variant Plain RI or RI-DS; only affects the tie-break.
     */
    static Ordering computeOrdering(const SearchGraph &pattern, const SearchGraph &target,
                                    const std::vector<index> &patternLabels,
                                    const std::vector<index> &targetLabels, RI::Variant variant);

    /**
     * @param pattern Snapshot of the pattern, built with the adjacency matrix.
     * @param target Snapshot of the target, built without it.
     * @param patternLabels Empty when the search is unlabelled.
     * @param targetLabels Empty when the search is unlabelled.
     * @param ordering Shared, read-only; must outlive this object.
     * @param semantics Whether matches must be induced.
     * @param variant Plain RI or RI-DS.
     * @param sink Where complete mappings go. One per worker; never shared between threads.
     */
    RIImpl(const SearchGraph &pattern, const SearchGraph &target,
           const std::vector<index> &patternLabels, const std::vector<index> &targetLabels,
           const Ordering &ordering, SubgraphIsomorphism::Semantics semantics, RI::Variant variant,
           SubgraphIsomorphism::MatchSink sink);

    /**
     * Run the entire search from the empty mapping. Used by @ref RI.
     *
     * TODO: implement.
     *  1. Under RI_DS, call initializeDomains() first.
     *  2. Recurse over the fixed order: at each depth collect the candidates, test consistent()
     *     on each, map it, descend, unmap. At full depth, report the mapping.
     *  3. Stop the moment reportMapping() returns false, unwinding all the way out.
     */
    void run();

    /**
     * Advance one state by a single level, producing its children. Used by @ref ParallelRI.
     *
     * A worker calls this instead of recursing, so that the states it has not looked at yet can
     * be stolen by an idle worker. States that reach full depth are reported here rather than
     * returned.
     *
     * TODO: implement.
     *  1. If `state` is already at full depth, report it and return.
     *  2. Otherwise collect the candidates for `state.depth`, skipping the first
     *     `state.nextCandidate` of them so that a resumed state does not redo work.
     *  3. For each consistent candidate, append a child with depth + 1 to @a children.
     *  4. Update `state.nextCandidate` so the caller may put the state back on its deque and
     *     continue later.
     *
     * @param state In/out: the state to expand; its resume point is updated.
     * @param children Out: the children produced. Appended to, not cleared.
     * @return false if the search must stop because the match cap was reached.
     */
    bool expand(State &state, std::vector<State> &children);

private:
    /**
     * Collect the target nodes worth trying at `state.depth`.
     *
     * TODO: implement. When the position has a parent, the candidates are the neighbours of the
     * target node its parent was mapped to - typically a handful. When it has none, this position
     * starts a new component and every unmapped target node qualifies. Under RI_DS, intersect
     * with the node's domain first, which is usually far smaller.
     */
    void candidatesFor(const State &state, std::vector<node> &out) const;

    /**
     * Whether mapping the pattern node at `state.depth` onto @a tv is consistent so far.
     *
     * TODO: implement by calling ruleLabels(), ruleEdgesToPrefix(), then - only under
     * Semantics::INDUCED - ruleNonEdgesToPrefix(), and finally forwardCheck() under RI_DS.
     * Cheapest test first, return false at the first failure.
     */
    bool consistent(const State &state, node tv) const;

    /**
     * Every pattern edge to an already-mapped node must exist in the target.
     *
     * TODO: implement. For each earlier position @a i whose pattern node is adjacent to the one
     * being mapped, check that the target has the corresponding edge between @a tv and
     * `state.mapping[i]`. Check both directions when the graphs are directed. Also reject @a tv
     * outright if it is already used by an earlier position - the mapping must be injective.
     */
    bool ruleEdgesToPrefix(const State &state, node tv) const;

    /**
     * Under INDUCED semantics, every pattern *non*-edge must stay a non-edge in the target.
     *
     * TODO: implement as the mirror of @ref ruleEdgesToPrefix(): for each earlier position whose
     * pattern node is *not* adjacent to the one being mapped, the target must have no edge
     * between @a tv and that position's image. Never called under MONOMORPHISM, where extra
     * target edges are explicitly allowed.
     */
    bool ruleNonEdgesToPrefix(const State &state, node tv) const;

    /**
     * Label rule.
     *
     * TODO: implement. True immediately when unlabelled. Otherwise the two labels must be equal,
     * with @ref none acting as a wildcard on either side.
     */
    bool ruleLabels(node pu, node tv) const;

    /**
     * RI-DS only: check that no still-unmapped pattern node has been left with nothing to map to.
     *
     * TODO: implement. Tentatively remove @a tv from the domains of the unmapped pattern nodes
     * adjacent to the one being mapped, and reject if any domain becomes empty. Finding that out
     * now is much cheaper than descending several levels and discovering it there. Return true
     * unconditionally under plain RI, which keeps no domains.
     */
    bool forwardCheck(const State &state, node tv) const;

    /**
     * RI-DS only: work out which target nodes each pattern node could possibly map to.
     *
     * TODO: implement. A target node belongs to a pattern node's domain if their labels are
     * compatible and the target node's degree is at least the pattern node's - a node of degree
     * five can never sit on a target node of degree three. Under plain RI this is not called.
     *
     * Reuse: `domains` as declared below allocates a vector per position. Two alternatives worth
     * taking from elsewhere in NetworKit before this grows hot: `Aux::SparseVector<T>` for the
     * scratch marks used while filtering, since its reset() only clears entries that were actually
     * touched; and the pxvector/pxlookup technique in MaximalCliques.cpp, which keeps a whole
     * backtracking search's candidate sets in one buffer with no allocation at all. Do not reach
     * for Aux::SetIntersector here - it returns a std::set and allocates on every call.
     */
    void initializeDomains();

    /**
     * Hand a complete mapping to the sink.
     *
     * TODO: implement. `state.mapping` is indexed by position in the order, but the sink expects
     * a vector indexed by *pattern node*, so permute it through `ordering->order` into the
     * reusable `matchBuffer` before reporting. Getting this backwards is the easiest bug to write
     * here and produces plausible-looking but wrong matches.
     *
     * @return whatever the sink returned: false means stop searching.
     */
    bool reportMapping(const State &state);

    /**
     * Let the user interrupt a long search.
     *
     * TODO: implement. Increment `nodesVisited` and poll `Aux::SignalHandler` only when its low
     * bits are zero, so the check costs nothing in the common case.
     *
     * Reuse: hold an `Aux::SignalHandler` member and call assureRunning() on it - that is the
     * entire body. Note that ParallelRI must *not* let the resulting InterruptException escape its
     * OpenMP region; see the note on ParallelRIImpl::workerLoop().
     */
    void checkSignal();

    const SearchGraph *patternGraph;
    const SearchGraph *targetGraph;

    const std::vector<index> *patternLabels;
    const std::vector<index> *targetLabels;
    bool labelled;

    /// Shared with every other worker. Never modified after construction.
    const Ordering *ordering;

    SubgraphIsomorphism::Semantics semantics;
    RI::Variant variant;
    SubgraphIsomorphism::MatchSink sink;

    /// RI_DS only: domains[i] lists the target nodes position i could map to. Empty under RI.
    std::vector<std::vector<node>> domains;

    /// Reused buffer handed to the sink, indexed by pattern node.
    std::vector<node> matchBuffer;

    /// Scratch for candidatesFor(), reused so that expanding a state costs no allocation.
    mutable std::vector<node> candidateBuffer;

    /// Counts search steps so checkSignal() can poll only every so often.
    count nodesVisited;
};

} // namespace IsomorphismDetails
} // namespace NetworKit

#endif // NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_
