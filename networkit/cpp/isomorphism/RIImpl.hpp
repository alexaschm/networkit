#ifndef NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_
#define NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_

// Private header of the isomorphism module. Not installed, not part of the public API.
//
// Unlike VF2Impl and VF3Impl, which are hidden inside their own .cpp files, this one needs a
// header: RI.cpp runs one of these to do the whole search, and ParallelRI.cpp runs one per
// worker. Two translation units, so the class has to be visible to both. The same reason
// networkit/cpp/randomization/CurveballImpl.hpp and
// networkit/cpp/centrality/GroupClosenessGrowShrinkImpl.hpp exist.

#include <string>
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
 * The expensive preparation - working out which target nodes each pattern node could take, and
 * then the order in which pattern nodes get mapped - depends only on the two graphs, not on where
 * the search currently is. It is therefore two @b static functions producing a @ref Domains and an
 * @ref Ordering, which the caller computes once, in that order: under RI-Ds the order is read off
 * the domain sizes. @ref RI computes both and uses them itself; @ref ParallelRI computes them once
 * and shares the same two read-only objects with every worker.
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
 * Sizing @ref matchBuffer happens in the **constructor**, not in @ref run(). @ref ParallelRI builds
 * one RIImpl per worker and those workers only ever call @ref expand(), so anything set up in run()
 * would simply be missing on that path and a zero-length matchBuffer would report empty matches.
 * The constructor is the only placement that covers both entry points.
 *
 * The domains are *not* built here. They are a @ref Domains the caller computed before the order -
 * it has to be, because under RI-Ds the order is computed from the domain sizes - and this object
 * only holds a pointer to it. Being shared and immutable is what makes a @ref State evaluable by
 * any worker: nothing about the search depends on which worker got there first.
 *
 * ## Why there is no bookkeeping
 *
 * @ref VF2 maintains terminal sets and @ref VF3 maintains per-class counters, both updated on
 * every step and undone on every backtrack. RI maintains nothing: the matching order already
 * encodes which nodes constrain which, so at each depth the candidates follow directly from the
 * order and from what is already mapped. Each step is therefore very cheap, and RI wins by taking
 * many cheap steps rather than few expensive ones.
 *
 * The one exception is @ref RI::Variant::RI_DS, which computes a candidate domain per pattern node
 * once, before the search, and then intersects candidate lists with it. Those live in @ref Domains
 * and are only populated in that variant. Forward checking happens there too, as one preprocessing
 * step; nothing prunes a domain once the search is running and nothing has to be restored on
 * backtracking, which is what keeps a step cheap.
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
     * Per-pattern-node candidate sets, computed once and shared read-only by every worker.
     *
     * Indexed by *pattern node id*, never by position in the order: under RI-Ds the order is
     * computed from these, so it does not exist yet when they are built.
     */
    struct Domains {
        /// `ofPatternNode[pu]` lists the target nodes @a pu could map to, ascending. Empty under
        /// plain RI, and then nothing here is consulted at all.
        std::vector<std::vector<node>> ofPatternNode;

        /// Whether intersecting a target *slice* with the domain repays the binary search it
        /// costs. Pure economics: clearing an entry cannot change which matches are found. See
        /// MinSweepYieldForSliceIntersection in the .cpp.
        std::vector<bool> earnsItsKeep;

        /// Some pattern node has nowhere to go, so the instance has no matches at all. Precomputed
        /// here so that both drivers get the same early exit from the same place.
        bool anyEmpty = false;
    };

    /**
     * RI-DS only: work out which target nodes each pattern node could possibly map to.
     *
     * Returns an empty @ref Domains under plain RI, which every consumer reads as "no domains".
     * Otherwise it runs three steps, each exactly once.
     *
     * **Build.** One pass per pattern node over the target ids in ascending order - which leaves
     * each domain sorted, as every consumer requires - keeping the ids that could host that node at
     * all: existing, degree-dominating, label-compatible. That judgement never consults a partial
     * mapping, which is what makes a domain a *superset* of the true image set and intersecting
     * with it sound.
     *
     * **Refinement sweep.** A target node survives only if, for every pattern node its owner is
     * adjacent to, that node's domain is reachable across an arc of the right direction and a
     * compatible label. One pass over the domains, deliberately not run to convergence: on a large
     * unlabelled target a domain approaches the whole node set, which makes the sweep the one
     * costly part of RI-Ds.
     *
     * **Forward checking.** The paper's Section 4.2.2, one call to the file-local
     * forwardCheckSingletons(). See its comment for why its inner worklist is not a second pass.
     *
     * How much the sweep and the forward check removed between them is then recorded per node in
     * @ref Domains::earnsItsKeep, which is what decides whether the domain is worth applying to a
     * target slice later. The build's own removals do not count towards that: @ref consistent()
     * rejects exactly those candidates already, and more cheaply.
     *
     * Static, and called by the driver before @ref computeOrdering(), because the order depends on
     * the domain sizes and because one shared copy replaces one per worker.
     */
    static Domains computeDomains(const SearchGraph &pattern, const SearchGraph &target,
                                  const std::vector<index> &patternNodeLabels,
                                  const std::vector<index> &targetNodeLabels, RI::Variant variant);

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
     * Under RI-Ds the domains add two things on top, both from Kimmig, Meyerhenke and Strash
     * (2017). Every pattern node whose domain holds exactly one target node is appended **before**
     * any node whose domain is larger - their Section 4.1 - because such a node's image is already
     * decided and mapping it first constrains everything else for free. And a candidate that ties
     * on the whole triple is settled by the smaller domain, their Section 4.2.1, which slots in one
     * level above the id tie-break rather than replacing the triple's third term: the paper calls
     * that term the degree, but Bonnici's - and this file's - is `V_unv`, so domain size is a
     * *fourth* key. Under plain RI both are inert, so the emitted order is unchanged.
     *
     * @param pattern Snapshot of the pattern.
     * @param domains Computed by @ref computeDomains() beforehand. Empty under plain RI, in which
     *        case the order depends on the pattern alone, exactly as the original paper claims.
     */
    static Ordering computeOrdering(const SearchGraph &pattern, const Domains &domains);

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
     * @param patternNodeLabels Empty when the search is unlabelled.
     * @param targetNodeLabels Empty when the search is unlabelled.
     * @param ordering Shared, read-only; must outlive this object.
     * @param domains Shared, read-only; must outlive this object. Empty under plain RI, which is
     *        how the variant reaches the search - there is no separate flag.
     * @param semantics Whether matches must be induced.
     * @param handler Polled so a long search can be stopped with CTRL+C. Shared between
     *        workers; only the non-throwing isRunning() may be used, since ParallelRI runs this
     *        inside an OpenMP region.
     * @param report Where complete mappings go. One per worker; never shared between threads.
     */
    RIImpl(const SearchGraph &pattern, const SearchGraph &target,
           const std::vector<index> &patternNodeLabels, const std::vector<index> &targetNodeLabels,
           const Ordering &ordering, const Domains &domains,
           SubgraphIsomorphism::Semantics semantics, Aux::SignalHandler &handler,
           MatchReporter report);

    /**
     * The empty mapping the search starts from, sized to the order.
     *
     * The width is the contract every other @ref State inherits: a state has one entry per position
     * whether or not that position is filled yet, so whoever picks it up has room to write at its
     * own position. Children are copies of their parent, so getting it right once here is what
     * makes it right everywhere - and it is why both drivers start from this rather than sizing a
     * State of their own.
     */
    State rootState() const;

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
     *        width first if it arrives short - defensively, since every state either comes from
     *        @ref rootState() or is a copy of one that did.
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
     * the pattern node's domain replaces or intersects the list, both of them strictly ascending,
     * so that is a merge and not a lookup.
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

    const std::vector<index> *patternNodeLabels;
    const std::vector<index> *targetNodeLabels;
    bool nodeLabelled;

    /// Shared with every other worker. Never modified after construction.
    const Ordering *ordering;

    /// Shared with every other worker, and the only thing that tells the search which variant it
    /// is running: `ofPatternNode` is empty under plain RI. Never modified after construction,
    /// which is what lets any worker evaluate any State.
    const Domains *domains;

    SubgraphIsomorphism::Semantics semantics;

    /// Shared with every other worker. Poll it as `handler->isRunning()` and stop on false -
    /// never assureRunning(), because ParallelRI runs this inside an OpenMP region and an
    /// exception escaping one of those is undefined behaviour.
    Aux::SignalHandler *handler;

    MatchReporter report;

    /// Reused buffer handed to the reporter, indexed by pattern node. Sized by the constructor.
    SubgraphIsomorphism::Match matchBuffer;

    /// Scratch for candidatesFor(), reused so that expanding a state costs no allocation. A flat
    /// stack: recurse() gives each depth the half-open window it appended and pops it on the way
    /// out, so it must be iterated by index - a deeper level appends behind you and can
    /// reallocate. expand() never recurses and owns the whole buffer.
    mutable std::vector<node> candidateBuffer;
};

/**
 * Everything an RI search needs built before it can start.
 *
 * @ref RI::run() and @ref ParallelRI::run() need exactly the same four things from the same inputs,
 * and the sequence is not free: the domains have to exist before the ordering, because under RI-DS
 * the ordering is read off the domain sizes. Building them here rather than once in each driver is
 * what keeps that dependency in a single place, and what stops the sequential and the parallel
 * search from quietly answering two different questions.
 */
struct RISearchSetup {
    /// Built **with** the adjacency matrix: the pattern is small, so it can afford the memory that
    /// makes hasEdge() constant time.
    SearchGraph patternGraph;

    /// Built **without** it - a bit per ordered pair of target ids does not fit - so hasEdge()
    /// falls back to a binary search over the sorted neighbours.
    SearchGraph targetGraph;

    /// Empty, and free to compute, under plain RI.
    RIImpl::Domains domains;

    RIImpl::Ordering ordering;
};

/**
 * Build the snapshots, the RI-DS domains and the matching order for one RI search.
 *
 * @param algorithmName Named in the refusal below, so a caller sees which of the two drivers turned
 *        the input down.
 * @throws std::runtime_error when collapsing parallel edges had to throw an edge label away, which
 *         is the one input the snapshot cannot represent. Raised before the domains are computed
 *         and, for ParallelRI, before any worker exists, so nothing is left half-built.
 */
RISearchSetup prepareRISearch(const Graph &pattern, const Graph &target,
                              const std::vector<index> &patternNodeLabels,
                              const std::vector<index> &targetNodeLabels,
                              const std::vector<index> &patternEdgeLabels,
                              const std::vector<index> &targetEdgeLabels, RI::Variant variant,
                              const std::string &algorithmName);

} // namespace IsomorphismDetails
} // namespace NetworKit

#endif // NETWORKIT_CPP_ISOMORPHISM_RI_IMPL_HPP_
