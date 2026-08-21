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
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/isomorphism/RI.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

#include "MatchReporter.hpp"
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
 * Both share every rule below, so the two algorithms cannot drift apart. @ref candidatesFor(),
 * @ref consistent() with the rules it calls, and @ref reportMapping() are the **only** places that
 * know what a match is; @ref run(), @ref recurse() and @ref expand() are pure control flow. A
 * matching rule that appears anywhere else is how the two searches start disagreeing.
 *
 * ## What the constructor does, and why
 *
 * Sizing @ref matchBuffer and building the RI-Ds domains both happen in the **constructor**, not
 * in @ref run(). @ref ParallelRI builds one RIImpl per worker and those workers only ever call
 * @ref expand(), so anything set up in run() would simply be missing on that path: RI-Ds would
 * quietly degrade to RI, `domains[j]` would index out of bounds, and a zero-length matchBuffer
 * would report empty matches. The constructor is the only placement that covers both entry points,
 * and it puts the domain scan inside the OpenMP region, where it runs in parallel.
 *
 * `domains` is immutable after construction. That is what makes a @ref State evaluable by any
 * worker: nothing about the search depends on which worker got there first.
 *
 * ## Why there is no bookkeeping
 *
 * @ref VF2 maintains terminal sets and @ref VF3 maintains per-class counters, both updated on
 * every step and undone on every backtrack. RI maintains nothing: the matching order already
 * encodes which nodes constrain which, so at each depth the candidates follow directly from the
 * order and from what is already mapped. Each step is therefore very cheap, and RI wins by taking
 * many cheap steps rather than few expensive ones.
 *
 * The one exception is @ref RI::Variant::RI_DS, which computes a candidate domain per position
 * once, before the search, and then intersects candidate lists with it. Those live in `domains`
 * and are only touched in that variant. There is no forward checking and no domain reduction
 * during backtracking: the paper measured that heavier per-step pruning does not pay off.
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
     *
     * `mapping` must be **full width** - one entry per position in the order, `none` past `depth`
     * - so that whoever picks the state up has room to write at its own position. @ref expand()
     * repairs a short one defensively rather than trusting its caller.
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
     * Starting from the maximum-degree node, this repeatedly appends the unordered node maximising
     * a lexicographic triple over the already-ordered prefix `mu`: the number of **arcs** from the
     * candidate into `mu`; then the number of `mu`-members reached from the candidate in two hops
     * **through an unordered intermediate**, which is a two-step look-ahead at constraints not yet
     * in force; then the number of the candidate's neighbours that lie outside `mu` and touch
     * nothing in it. The three are not a partition of the candidate's neighbourhood - the
     * intermediate of the second term is counted by none of them - so all three are computed
     * rather than derived from the degree. A full tie goes to the smallest node id.
     *
     * The paper's Figure 2 does not survive checking on six points, all of them noted next to the
     * implementation, one of which produces a wrong ordering rather than a compile error.
     *
     * A disconnected pattern needs no special case: once the order exhausts a component every
     * remaining candidate scores zero on the first two terms, so the triple degenerates to degree
     * alone and the next component restarts at its largest node, with @ref none as its parent.
     *
     * @param pattern Snapshot of the pattern.
     * @param target Unused. Kept because both drivers pass it and because a target-aware variant
     *        would plug in here; RI's ordering is target-independent, which is the paper's claim.
     * @param patternLabels Unused, as above.
     * @param targetLabels Unused, as above.
     * @param variant Unused: RI-Ds orders exactly as plain RI does. There is no domain-size
     *        tie-break, because that is what the paper's measurements advise against.
     */
    static Ordering computeOrdering(const SearchGraph &pattern, const SearchGraph &target,
                                    const std::vector<index> &patternLabels,
                                    const std::vector<index> &targetLabels, RI::Variant variant);

    /**
     * Inputs no match can survive, judged before the search starts.
     *
     * The pattern cannot have more nodes than the target, because the mapping is injective, and it
     * cannot have a higher maximum degree, because a node's distinct neighbours have to fit in its
     * image's neighbourhood. All three numbers are precomputed by @ref SearchGraph.
     *
     * Public and static because ParallelRI's driver never calls @ref run(), and the two must not
     * disagree about which inputs are hopeless.
     */
    static bool patternCannotFit(const SearchGraph &pattern, const SearchGraph &target);

    /**
     * @param pattern Snapshot of the pattern, built with the adjacency matrix.
     * @param target Snapshot of the target, built without it.
     * @param patternLabels Empty when the search is unlabelled.
     * @param targetLabels Empty when the search is unlabelled.
     * @param ordering Shared, read-only; must outlive this object.
     * @param semantics Whether matches must be induced.
     * @param variant Plain RI or RI-DS.
     * @param handler Polled so a long search can be stopped with CTRL+C. Shared between
     *        workers; only the non-throwing isRunning() may be used, since ParallelRI runs this
     *        inside an OpenMP region.
     * @param report Where complete mappings go. One per worker; never shared between threads.
     */
    RIImpl(const SearchGraph &pattern, const SearchGraph &target,
           const std::vector<index> &patternLabels, const std::vector<index> &targetLabels,
           const Ordering &ordering, SubgraphIsomorphism::Semantics semantics, RI::Variant variant,
           Aux::SignalHandler &handler, MatchReporter report);

    /**
     * Run the entire search from the empty mapping. Used by @ref RI.
     *
     * Bails out at once on an input no match can survive - see @ref patternCannotFit(), and, under
     * RI-Ds, an empty domain - and otherwise hands an empty mapping to @ref recurse(). Both
     * bail-outs are guarded on a non-empty order: an empty pattern has exactly one match, the
     * empty mapping, and must never be caught by either.
     */
    void run();

    /**
     * Advance one state by a single level, producing its children. Used by @ref ParallelRI.
     *
     * A worker calls this instead of recursing, so that the states it has not looked at yet can
     * be stolen by an idle worker. States that reach full depth are reported here rather than
     * returned.
     *
     * The candidate list is recomputed on re-entry rather than cached, which is sound because
     * @ref candidatesFor() is deterministic, and is why `state.nextCandidate` is an index into
     * that list rather than a copy of what is left of it. Expanding copies a mapping per child
     * where @ref recurse() mutates one in place; that cost is exactly what buys ParallelRI
     * stealable work, and it is why run() recurses instead of being written on top of this.
     *
     * @param state In/out: the state to expand; its resume point is updated. Repaired to full
     *        width first if it arrives short.
     * @param children Out: the children produced. Appended to, not cleared.
     * @return false if the search must stop because the match cap was reached. The interrupt poll
     *         is not done here - that belongs to the worker loop.
     */
    bool expand(State &state, std::vector<State> &children);

private:
    /**
     * Depth-first walk of the fixed order.
     *
     * A private member rather than a lambda: run() returns void, so the recursion needs its own
     * way to propagate "stop the whole search", and a recursive std::function would put an
     * allocation and an indirect call in the hot loop.
     *
     * @return false to stop the whole search - interrupted, or the match cap reached - true to
     *         carry on.
     */
    bool recurse(State &state);

    /**
     * Collect the target nodes worth trying at `state.depth`, **appending** them to @a out.
     *
     * Appending rather than clearing is what makes `candidateBuffer` a flat stack with one
     * half-open window per depth, so a whole backtracking search costs no allocation - the
     * pxvector technique from MaximalCliques.cpp.
     *
     * When the position has a parent, exactly one target slice holds every valid candidate: the
     * out-slice of the parent's image for an undirected or forward pattern edge, the in-slice for
     * a backward one, and the shorter of the two when both arcs exist, since @ref
     * ruleEdgesToPrefix() rechecks both directions anyway. Walking the wrong one loses real
     * matches. With edge labels the slice is filtered by the parent edge's label as it is walked,
     * which costs one comparison per candidate because the label sits at the same offset as the
     * head.
     *
     * When it has none, this position starts a new component and every existing target node
     * qualifies - `hasNode()` is what separates a removed id from an isolated node. Under RI_DS
     * the position's domain replaces or intersects the list, both of them strictly ascending, so
     * that is a merge and not a lookup.
     *
     * Target nodes already used are deliberately not filtered out here; injectivity lives in
     * @ref ruleEdgesToPrefix() and nowhere else.
     */
    void candidatesFor(const State &state, std::vector<node> &out) const;

    /**
     * Whether mapping the pattern node at `state.depth` onto @a tv is consistent so far.
     *
     * Four gates in increasing cost: degree domination, which on a sparse target rejects most
     * candidates before an edge is touched; @ref ruleLabels(); @ref ruleEdgesToPrefix(); and,
     * under Semantics::INDUCED only, @ref ruleNonEdgesToPrefix(). The order is the paper's, whose
     * cheap condition 3 exists precisely to obviate the expensive condition 4.
     */
    bool consistent(const State &state, node tv) const;

    /**
     * Every pattern edge to an already-mapped node must exist in the target, with a compatible
     * edge label.
     *
     * One pass over the earlier positions. This is also where **injectivity** is enforced, and the
     * only place: a candidate equal to an earlier position's image is rejected here. A directed
     * mutual pair is checked in both directions independently, because its two arcs are two edge
     * ids and can carry different labels.
     */
    bool ruleEdgesToPrefix(const State &state, node tv) const;

    /**
     * Under INDUCED semantics, every pattern *non*-edge must stay a non-edge in the target.
     *
     * The mirror of @ref ruleEdgesToPrefix(), and it has to run over *all* earlier positions
     * rather than only the neighbours - without that, Semantics::INDUCED silently computes a
     * monomorphism. Edge labels play no part here; they constrain only edges that exist. Never
     * called under MONOMORPHISM, where extra target edges are explicitly allowed.
     *
     * Could be fused with @ref ruleEdgesToPrefix() into a single pass under INDUCED. Kept separate
     * to match the declared API; the fusion is a micro-optimisation.
     */
    bool ruleNonEdgesToPrefix(const State &state, node tv) const;

    /**
     * Node label rule. True immediately when unlabelled; otherwise the two labels must be equal,
     * with @ref none acting as a wildcard on either side.
     */
    bool ruleLabels(node pu, node tv) const;

    /**
     * RI-DS only: work out which target nodes each position could possibly map to.
     *
     * Returns immediately under plain RI. Otherwise it builds `domains` in two halves. First, one
     * pass per position over the target ids in ascending order - which leaves each domain sorted,
     * as every consumer requires - keeping the ids that could host the position's pattern node at
     * all: existing, degree-dominating, label-compatible. That judgement never consults the partial
     * mapping, which is what makes a domain a *superset* of the true image set and intersecting
     * with it sound.
     *
     * Then a single refinement sweep: a target node survives only if, for every other position its
     * pattern node is adjacent to, that position's domain is reachable across an arc of the right
     * direction and a compatible label. Not run to convergence, and separable on purpose - on a
     * large unlabelled target a domain approaches the whole node set, which makes the sweep the one
     * costly part of RI-Ds. It is pure pruning, so gating it behind a size threshold would always
     * be safe.
     *
     * How much that sweep removed is then recorded per position in @ref domainEarnsItsKeep, which
     * is what decides whether the domain is worth applying to a target slice later. The build's own
     * removals do not count towards that: @ref consistent() rejects exactly those candidates
     * already, and more cheaply.
     *
     * Called from the constructor, so `domains` exists on the expand() path too.
     */
    void initializeDomains();

    /**
     * Hand a complete mapping over.
     *
     * `state.mapping` is indexed by position in the order, but the reporter expects a vector
     * indexed by *pattern node*, so it is permuted through `ordering->order` into the reusable
     * `matchBuffer`. Getting this backwards is the easiest bug to write here and produces
     * plausible-looking but wrong matches.
     *
     * @return whatever the reporter returned: false means stop searching.
     */
    bool reportMapping(const State &state);

    const SearchGraph *patternGraph;
    const SearchGraph *targetGraph;

    const std::vector<index> *patternLabels;
    const std::vector<index> *targetLabels;
    bool labelled;

    /// Shared with every other worker. Never modified after construction.
    const Ordering *ordering;

    SubgraphIsomorphism::Semantics semantics;
    RI::Variant variant;

    /// Shared with every other worker. Poll it as `handler->isRunning()` and stop on false -
    /// never assureRunning(), because ParallelRI runs this inside an OpenMP region and an
    /// exception escaping one of those is undefined behaviour.
    Aux::SignalHandler *handler;

    MatchReporter report;

    /// RI_DS only: domains[i] lists the target nodes position i could map to. Empty under RI.
    /// Built by the constructor and never touched again, which is what lets any worker evaluate
    /// any State.
    std::vector<std::vector<node>> domains;

    /// RI_DS only: whether domains[i] is worth intersecting a *target slice* with, decided once
    /// from how much the refinement sweep removed. False leaves the domain in place for the
    /// parentless path, where it replaces a full scan and always wins, while letting the slice
    /// path skip a binary search that would only re-reject what consistent() rejects for two
    /// integer compares. Pure economics: clearing it cannot change which matches are found.
    std::vector<bool> domainEarnsItsKeep;

    /// Reused buffer handed to the reporter, indexed by pattern node. Sized by the constructor.
    std::vector<node> matchBuffer;

    /// Scratch for candidatesFor(), reused so that expanding a state costs no allocation. A flat
    /// stack: recurse() gives each depth the half-open window it appended and pops it on the way
    /// out, so it must be iterated by index - a deeper level appends behind you and can
    /// reallocate. expand() never recurses and owns the whole buffer.
    mutable std::vector<node> candidateBuffer;
};

} // namespace IsomorphismDetails
} // namespace NetworKit

#endif // NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_
