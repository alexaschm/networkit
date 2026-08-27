#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <networkit/auxiliary/SparseVector.hpp>

#include "RIImpl.hpp"

namespace NetworKit {
namespace IsomorphismDetails {

namespace {

/**
 * Whether pattern node @a pu may sit on target node @a tv as far as node labels are concerned.
 *
 * Deliberately the same three lines as IsomorphismTest::nodeLabelsCompatible(), which is what the
 * test corpus judges every algorithm against. @ref RIImpl::ruleLabels() is a one-line delegate to
 * this, and @ref couldMap() calls it too, so the search and the RI-Ds domains cannot disagree about
 * what a label permits.
 */
bool nodeLabelsCompatible(const std::vector<index> &patternNodeLabels,
                          const std::vector<index> &targetNodeLabels, node pu, node tv) {
    if (patternNodeLabels.empty())
        return true;

    const index patternLabel = patternNodeLabels[pu];
    const index targetLabel = targetNodeLabels[tv];
    return patternLabel == none || targetLabel == none || patternLabel == targetLabel;
}

/// Whether a pattern arc carrying @a patternLabel may sit on a target arc carrying @a targetLabel.
/// @ref none is a wildcard on either side, exactly as it is for node labels.
bool edgeLabelsCompatible(index patternLabel, index targetLabel) noexcept {
    return patternLabel == none || targetLabel == none || patternLabel == targetLabel;
}

/**
 * Call @a fn once per arc incident to @a u, ignoring direction.
 *
 * A directed mutual pair is therefore visited *twice*, once per arc. That is what the ordering's
 * first score term wants: the paper counts edges, not neighbours, and two arcs between the same
 * pair really are two constraints.
 */
template <typename Callback>
void forEachIncidentArc(const SearchGraph &g, node u, Callback fn) {
    for (const node *it = g.outBegin(u); it != g.outEnd(u); ++it)
        fn(*it);

    // For an undirected snapshot the in-accessors return the out-slices, so walking them again
    // would visit every neighbour twice.
    if (!g.isDirected())
        return;

    for (const node *it = g.inBegin(u); it != g.inEnd(u); ++it)
        fn(*it);
}

/**
 * Call @a fn once per *distinct* neighbour of @a u, in either direction.
 *
 * The mirror image of @ref forEachIncidentArc() for the two score terms that are vertex sets
 * rather than edge sets. An in-neighbour that is also an out-neighbour is skipped, which the
 * pattern's adjacency matrix answers in constant time.
 */
template <typename Callback>
void forEachDistinctNeighbor(const SearchGraph &g, node u, Callback fn) {
    for (const node *it = g.outBegin(u); it != g.outEnd(u); ++it)
        fn(*it);

    if (!g.isDirected())
        return;

    for (const node *it = g.inBegin(u); it != g.inEnd(u); ++it)
        if (!g.hasEdge(u, *it))
            fn(*it);
}

/**
 * Whether @a tv could host @a pu at all, judged without any partial mapping.
 *
 * This is what an RI-Ds domain is filtered by, so it must never look at where the search currently
 * is: a domain has to be a *superset* of the images @a pu can take in any match, or intersecting a
 * candidate list with it would throw real matches away.
 */
bool couldMap(const SearchGraph &pattern, const SearchGraph &target,
              const std::vector<index> &patternNodeLabels,
              const std::vector<index> &targetNodeLabels, node pu, node tv) {
    if (!target.hasNode(tv))
        return false;

    if (target.outDegree(tv) < pattern.outDegree(pu))
        return false;

    if (pattern.isDirected() && target.inDegree(tv) < pattern.inDegree(pu))
        return false;

    return nodeLabelsCompatible(patternNodeLabels, targetNodeLabels, pu, tv);
}

/**
 * Whether the sorted arc slice `[begin, end)` meets @a domain in at least one node, counting only
 * arcs whose label is compatible with @a patternLabel.
 *
 * Like `intersectionSize(...) != 0`, but it stops at the first hit, which is all the RI-Ds
 * refinement sweep ever needs. @ref SearchGraph does not offer that, and the counting form would
 * walk both ranges to the end on every one of the sweep's many calls.
 *
 * @param labels One label per slice entry, at the same offset, or nullptr to ignore labels.
 */
bool intersectsDomain(const node *begin, const node *end, const index *labels, index patternLabel,
                      const std::vector<node> &domain) {
    auto candidate = domain.begin();
    for (const node *it = begin; it != end; ++it) {
        // Searched from where the last entry left off rather than from the front, so progress
        // stays monotone as in a merge, but a slice far shorter than the domain - which is the
        // usual shape on a large target - costs log(domain) per entry instead of a walk over the
        // whole domain.
        candidate = std::lower_bound(candidate, domain.end(), *it);

        if (candidate == domain.end())
            return false;

        if (*candidate != *it)
            continue;

        if (labels == nullptr || edgeLabelsCompatible(patternLabel, labels[it - begin]))
            return true;
    }

    return false;
}

/**
 * One "the image of this pattern node has to reach that pattern node's domain" requirement.
 *
 * The RI-Ds refinement sweep asks the same question of every candidate in a domain, and the answer
 * depends on the pattern node that owns the domain, never on the candidate. So the questions are
 * collected once per pattern node and then replayed per candidate; see @ref
 * collectArcConstraints().
 */
struct ArcConstraint {
    /// Whose domain the candidate has to reach.
    node pj;
    /// The pattern arc's label, or @ref none when the pattern is unlabelled or the label is a
    /// wildcard - in which case the target arcs are not filtered at all.
    index label;
    /// Whether to walk the candidate's out-slice (a pattern arc pu -> pj) or its in-slice (pj ->
    /// pu).
    bool outgoing;
};

/**
 * Collect what the refinement sweep must check for every candidate in @a pu's domain.
 *
 * Read straight off the pattern's own slices, one entry per incident arc, which is what keeps the
 * sweep proportional to `deg(pu)` rather than to the size of the pattern. A directed mutual pair
 * contributes two entries, one per arc, because the two arcs are two edge ids and can carry
 * different labels - the same reason @ref RIImpl::ruleEdgesToPrefix() checks both directions
 * separately. An undirected snapshot has no in-slices of its own, so its neighbours are all
 * reported as outgoing, exactly as `hasEdge()` sees them.
 *
 * Labels come from the slice rather than from `edgeLabel()`: a label sits at the same offset as its
 * head, so this costs a pointer bump where the lookup would cost a binary search per arc.
 *
 * @param out Appended to, not cleared, so one buffer serves the whole sweep.
 */
void collectArcConstraints(const SearchGraph &pattern, node pu, std::vector<ArcConstraint> &out) {
    const node *outBegin = pattern.outBegin(pu);
    const index *outLabels = pattern.outLabelBegin(pu);
    for (const node *it = outBegin; it != pattern.outEnd(pu); ++it)
        out.push_back({*it, outLabels == nullptr ? none : outLabels[it - outBegin], true});

    if (!pattern.isDirected())
        return;

    const node *inBegin = pattern.inBegin(pu);
    const index *inLabels = pattern.inLabelBegin(pu);
    for (const node *it = inBegin; it != pattern.inEnd(pu); ++it)
        out.push_back({*it, inLabels == nullptr ? none : inLabels[it - inBegin], false});
}

/**
 * How much of a domain the refinement sweep has to remove before intersecting a target slice with
 * it is worth doing.
 *
 * The two halves of a domain are not worth the same. What the *build* half removes - ids that are
 * not nodes, degrees too small, labels that clash - is exactly what the first two gates of
 * @ref RIImpl::consistent() reject anyway, for two integer compares and an array read. Paying a
 * binary search over the domain to reject the same candidate is strictly worse. Only what the
 * *sweep* removes is information the per-candidate rules do not have.
 *
 * So the sweep's yield is the whole question. The figure below is measured, not derived: a
 * labelled triangle over caidaRouterLevel (n = 192k), with the number of label classes varied to
 * drive the yield, intersecting versus not -
 *
 *     sweep yield   0.56   0.66   0.72   0.79   0.84   0.97
 *     intersect/not 1.21   1.08   1.06   1.01   1.02   0.99
 *
 * - so intersecting costs over a fifth at a yield of one half, and stops costing anything around
 * four fifths. It never clearly wins at any yield reachable on these inputs, which is why the
 * threshold sits at the point where the cost disappears rather than anywhere lower: past it the
 * pruning is free, and free pruning is worth keeping for patterns deeper than the ones measured,
 * where one rejected candidate saves a whole subtree.
 *
 * Below the threshold RI-Ds simply does not intersect, which costs nothing but pruning power: a
 * domain is a superset of the images a position can take, so declining to apply it lets more
 * candidates through to rules that reject them anyway. The match set cannot change either way.
 */
constexpr double MinSweepYieldForSliceIntersection = 0.8;

/**
 * Forward checking over the finished domains - Section 4.2.2 of Kimmig, Meyerhenke and Strash.
 *
 * A pattern node whose domain holds exactly one target node will be mapped onto that node in every
 * match there is, so injectivity forbids that node to every other pattern node. Removing it can
 * shrink another domain from two entries to one, creating a singleton that did not exist when the
 * pass started, so the worklist below keeps going until no new singleton appears - the paper's "We
 * repeat this procedure for any newly introduced singleton domains". That loop is internal:
 * forward checking itself is one preprocessing step, run once, from @ref RIImpl::computeDomains().
 *
 * Sound for the same reason the refinement sweep is: it only ever removes a target node that no
 * match could have used, so a domain stays a superset of the images its pattern node can take.
 *
 * Nearly always free. With no singleton anywhere in the pattern - the usual case on an unlabelled
 * target - it costs one scan over the pattern nodes and stops.
 *
 * @return false when some domain ran empty, which means the instance has no matches at all.
 */
bool forwardCheckSingletons(const SearchGraph &pattern, std::vector<std::vector<node>> &domains) {
    const count z = pattern.upperNodeIdBound();

    // A node enters the worklist at most once. Its domain can still shrink afterwards, but only to
    // empty, and that is caught at the erase below rather than by re-queueing.
    std::vector<bool> queued(z, false);
    std::vector<node> pending;

    for (node pu = 0; pu < z; ++pu) {
        if (!pattern.hasNode(pu))
            continue;
        if (domains[pu].empty())
            return false;
        if (domains[pu].size() == 1) {
            queued[pu] = true;
            pending.push_back(pu);
        }
    }

    while (!pending.empty()) {
        const node pu = pending.back();
        pending.pop_back();

        // Still a singleton: anything that could have emptied this domain returns false at the
        // erase below, before its owner is ever popped.
        const node fixed = domains[pu].front();

        for (node pw = 0; pw < z; ++pw) {
            if (pw == pu || !pattern.hasNode(pw))
                continue;

            std::vector<node> &other = domains[pw];
            const auto found = std::lower_bound(other.begin(), other.end(), fixed);
            if (found == other.end() || *found != fixed)
                continue;

            // One id at a time, because these are the ascending vectors candidatesFor() merges
            // against. The paper clears a whole bitmask in one word operation, but a bitmask
            // cannot be walked as the sorted range the merge needs. One memmove per hit, and there
            // is at most one hit per (singleton, other node) pair.
            other.erase(found);

            // Two pattern nodes forced onto the same target node land here, and the instance has
            // no matches at all.
            if (other.empty())
                return false;

            // The singleton this removal just created. Picking it up here is what "repeat this
            // procedure for any newly introduced singleton domains" means; it is this loop
            // continuing, not forward checking being applied a second time.
            if (other.size() == 1 && !queued[pw]) {
                queued[pw] = true;
                pending.push_back(pw);
            }
        }
    }

    return true;
}

} // namespace

RIImpl::Ordering RIImpl::computeOrdering(const SearchGraph &pattern, const Domains &domains) {
    // Plain RI's ordering looks at nothing but the pattern; RI-Ds reads the domain sizes on top,
    // and nothing else about the target. Either way it is computed once and shared, because
    // nothing it reads changes while the search runs.

    // ## The score, and the paper's errata
    //
    // Starting from the maximum-degree node, repeatedly append the unordered node maximising the
    // lexicographic triple (V_vis, V_neig, V_unv), where mu is the already-ordered prefix:
    //
    //   V_vis(u)  - arcs from u into mu. Arcs, not neighbours: a directed mutual pair counts two.
    //   V_neig(u) - members of mu reached from u in two hops through an *unordered* intermediate.
    //               The intermediate itself is counted by nothing.
    //   V_unv(u)  - neighbours of u outside mu that are adjacent to nothing in mu.
    //
    // The three are not a partition of N(u), so V_unv cannot be had by subtracting the other two
    // from the degree. Six things in Bonnici et al. (2013) do not survive checking, and each is a
    // trap for the next reader comparing this against Figure 2:
    //
    //   1. Figure 2's rank reads (|V_vis|, |V_neig|, |V_neig|). The third term is |V_unv|; the
    //      body text is unambiguous.
    //   2. Figure 2 initialises the running best *before* the while loop rather than inside it. As
    //      printed, a previous winner's score survives and can block every remaining candidate.
    //      This is the one erratum that yields a wrong ordering rather than a compile error.
    //   3. The output is written (u_0, ..., u_n) with n = |V|, which lists n+1 entries. It ends at
    //      u_{k-1} for k = |V_P|.
    //   4. B_i is printed with 0 < j <= i, which drops j = 0 and admits a self-loop at j = i. It
    //      is 0 <= j < i.
    //   5. Figure 2's V_unv drops the "v is unvisited" restriction the body text states. Without
    //      it an already-ordered vertex qualifies whenever mu's members are mutually non-adjacent.
    //   6. Ties compare with <=, so the last candidate examined wins. Harmless once erratum 2 is
    //      fixed - the paper breaks ties arbitrarily - but this picks the smallest node id, because
    //      reproducibility matters more than which arbitrary choice is made.
    //
    // Figure 3's worked example is consistent once the roles are kept straight: the intermediates
    // it names are not the vertices V_neig counts, which are the mu-members reached through them.

    const count z = pattern.upperNodeIdBound();
    const count total = pattern.numberOfNodes();

    Ordering result;
    result.order.reserve(total);
    result.parent.reserve(total);

    std::vector<bool> inOrder(z, false);

    // visCount[u] = arcs from u into the order. Maintained incrementally - bumped once per
    // incident arc when a node is appended - so the whole run pays O(m) for the first score term.
    std::vector<count> visCount(z, 0);

    // Scratch marks for V_neig, which is a union over mu and so has to de-duplicate. SparseVector
    // is the right shape here because its reset() clears only the entries actually touched, and
    // the touched set is tiny next to z.
    SparseVector<bool> reached(z, false);

    // The full triple for one candidate. V_neig is the expensive term - O(sum of deg(v) over
    // v in N(u)) - and cannot be maintained incrementally for less than a quadratic
    // shared-neighbour table, which is unaffordable at the sizes the matrix cutoff allows.
    const auto tripleFor = [&](node u) {
        std::array<count, 3> score{visCount[u], 0, 0};

        forEachDistinctNeighbor(pattern, u, [&](node v) {
            if (inOrder[v])
                return;

            forEachDistinctNeighbor(pattern, v, [&](node w) {
                if (!inOrder[w] || reached.indexIsUsed(w))
                    return;
                reached.insert(w, true);
                ++score[1];
            });
        });
        reached.reset();

        forEachDistinctNeighbor(pattern, u, [&](node v) {
            if (!inOrder[v] && visCount[v] == 0)
                ++score[2];
        });

        return score;
    };

    // ## What RI-Ds adds on top: SI, and singletons first
    //
    // Under plain RI the domains are empty, so every node reports size 0 here: the singleton phase
    // never engages and the tie-break clause below is never true. The order is then bit-for-bit
    // what it was before this variant existed.
    const auto domainSize = [&](node u) -> count {
        return domains.ofPatternNode.empty() ? 0 : domains.ofPatternNode[u].size();
    };

    // Section 4.1 puts every singleton-domain node at the front of the order: its image is already
    // decided, so mapping it first constrains every later position for free. This is a hard filter
    // rather than another score term - nothing else may be appended while such a node is left
    // unordered. Forward checking is what manufactures most of these, which is why the two land
    // together.
    count remainingSingletons = 0;
    for (node u = 0; u < z; ++u)
        if (pattern.hasNode(u) && domainSize(u) == 1)
            ++remainingSingletons;

    const auto eligible = [&](node u) {
        return pattern.hasNode(u) && !inOrder[u]
               && (remainingSingletons == 0 || domainSize(u) == 1);
    };

    while (result.order.size() < total) {
        // Two passes, so the expensive V_neig is computed only for the candidates that are still
        // in the running after the cheap first term.
        count bestVis = 0;
        for (node u = 0; u < z; ++u)
            if (eligible(u))
                bestVis = std::max(bestVis, visCount[u]);

        // Reset per iteration - erratum 2. Ascending ids with a strict comparison means a full tie
        // goes to the smallest id, which is erratum 6.
        node best = none;
        std::array<count, 3> bestScore{};
        for (node u = 0; u < z; ++u) {
            if (!eligible(u) || visCount[u] != bestVis)
                continue;

            const std::array<count, 3> score = tripleFor(u);

            // Section 4.2.1's domain-size tie-break (SI) slips in one level above the id tie-break,
            // not into the triple: the paper calls RI's third term the degree, but Bonnici's - and
            // this file's - is V_unv, so the domain is a *fourth* key. A tie on all three counts
            // goes to the more constrained node; only a tie on the domain too goes by id.
            if (best == none || score > bestScore
                || (score == bestScore && domainSize(u) < domainSize(best))) {
                best = u;
                bestScore = score;
            }
        }

        // Unreachable, and here so that it stays that way. Everything below indexes by `best`, so
        // a `none` would read and then write far past the end of every array in this function. The
        // reason it cannot happen is a two-part invariant that is easy to break from a distance:
        // `bestVis` is a maximum over exactly the nodes `eligible` admits, so some eligible node
        // always matches it; and `remainingSingletons` counts the *unordered* singletons, which
        // holds because the singleton phase admits nothing else and so consumes exactly one per
        // iteration. Break either half - a new filter in `eligible`, a domain that changes under
        // us - and this stops silently corrupting memory and says so instead.
        if (best == none)
            break;

        // Every iteration of the singleton phase consumes exactly one singleton, because `eligible`
        // admitted nothing else.
        if (remainingSingletons != 0)
            --remainingSingletons;

        // visCount[best] == 0 is precisely the new-component case: no arc reaches back into the
        // order, so there is no parent to find and no point scanning for one. Two degenerate cases
        // need no code of their own because of this. With an empty prefix every candidate scores
        // (0, 0, deg), so step 0 picks the maximum-degree node - the paper's u_0. And once the
        // order exhausts a component, every remaining node scores (0, 0, deg) again, so a
        // disconnected pattern restarts at its next-largest node automatically.
        index parentPos = none;
        if (visCount[best] != 0) {
            for (index j = 0; j < result.order.size(); ++j) {
                const node earlier = result.order[j];
                if (pattern.hasEdge(earlier, best) || pattern.hasEdge(best, earlier)) {
                    parentPos = j;
                    break;
                }
            }
        }

        inOrder[best] = true;
        result.order.push_back(best);
        result.parent.push_back(parentPos);
        forEachIncidentArc(pattern, best, [&](node v) { ++visCount[v]; });
    }

    return result;
}

bool RIImpl::patternCannotFit(const SearchGraph &pattern, const SearchGraph &target) {
    // The mapping is injective, so the pattern can never have more nodes than the target, and no
    // pattern node's distinct neighbours can be squeezed into a smaller target neighbourhood. All
    // three numbers are precomputed by SearchGraph, so this costs nothing.
    if (pattern.numberOfNodes() > target.numberOfNodes())
        return true;

    if (pattern.maxOutDegree() > target.maxOutDegree())
        return true;

    return pattern.isDirected() && pattern.maxInDegree() > target.maxInDegree();
}

RIImpl::RIImpl(const SearchGraph &pattern, const SearchGraph &target,
               const std::vector<index> &patternNodeLabels,
               const std::vector<index> &targetNodeLabels, const Ordering &ordering,
               const Domains &domains, SubgraphIsomorphism::Semantics semantics,
               Aux::SignalHandler &handler, MatchReporter report)
    : patternGraph(&pattern), targetGraph(&target), patternNodeLabels(&patternNodeLabels),
      targetNodeLabels(&targetNodeLabels), nodeLabelled(!patternNodeLabels.empty()),
      ordering(&ordering), domains(&domains), semantics(semantics), handler(&handler),
      report(std::move(report)) {

    // Here rather than in run(), because ParallelRI builds one RIImpl per worker and those workers
    // only ever call expand(). See the class documentation. The domains are not built here at all:
    // the driver computed them before the order and every worker shares that one copy.
    matchBuffer.assign(pattern.upperNodeIdBound(), none);
}

RIImpl::State RIImpl::rootState() const {
    State state;
    state.mapping.assign(ordering->order.size(), none);
    return state;
}

void RIImpl::run() {
    // Guarded on a non-empty order throughout: an empty pattern has exactly one match - the empty
    // mapping - and must never be caught by a bail-out.
    if (!ordering->order.empty()) {
        if (patternCannotFit(*patternGraph, *targetGraph))
            return;

        // Under RI-Ds an empty domain means some pattern node has nowhere to go at all. Decided
        // once by computeDomains(), so ParallelRI gets the same bail-out from the same place.
        if (domains->anyEmpty)
            return;
    }

    State state = rootState();
    recurse(state);
}

bool RIImpl::recurse(State &state) {
    // isRunning() and never assureRunning(): one atomic read, cheap enough to poll at every level,
    // and safe inside the OpenMP region ParallelRI runs the same body in.
    if (!handler->isRunning())
        return false;

    if (state.depth == ordering->order.size())
        return reportMapping(state);

    // This depth owns [base, end) of one shared flat buffer; candidatesFor() appends.
    const index base = candidateBuffer.size();
    candidatesFor(state, candidateBuffer);
    const index end = candidateBuffer.size();

    bool keepGoing = true;
    for (index k = base; k < end && keepGoing; ++k) {
        // By index, never by pointer or iterator: depth + 1 appends behind us and can reallocate.
        const node tv = candidateBuffer[k];
        if (!consistent(state, tv))
            continue;

        state.mapping[state.depth] = tv;
        ++state.depth;
        keepGoing = recurse(state);
        --state.depth;
        state.mapping[state.depth] = none;
    }

    // Pop this depth's window. Nothing is ever cleared globally, which is why no per-depth buffer
    // member is needed.
    candidateBuffer.resize(base);
    return keepGoing;
}

bool RIImpl::expand(State &state, std::vector<State> &children) {
    const count full = ordering->order.size();

    // A stolen state must be full width so that a child has room at its own position. Purely
    // defensive: every state either came from rootState() or is a copy of one that did, so nothing
    // reachable from the two drivers arrives short. It stays because the repair is a no-op when the
    // width is already right, and because a State assembled by hand would otherwise write past its
    // own end.
    if (state.mapping.size() < full)
        state.mapping.resize(full, none);

    if (state.depth == full)
        return reportMapping(state);

    // expand() never recurses, so unlike recurse() it owns the whole buffer.
    candidateBuffer.clear();
    candidatesFor(state, candidateBuffer);

    for (index k = state.nextCandidate; k < candidateBuffer.size(); ++k) {
        // Advanced before the candidate is judged, so a caller that stops consuming right after
        // being handed a child resumes in the right place.
        state.nextCandidate = k + 1;

        const node tv = candidateBuffer[k];
        if (!consistent(state, tv))
            continue;

        State child = state;
        child.mapping[state.depth] = tv;
        child.depth = state.depth + 1;
        child.nextCandidate = 0;
        children.push_back(std::move(child));
    }

    // Exhausted. Recomputing the list on re-entry is correct because candidatesFor() is
    // deterministic, which is why the resume point is an index rather than a cached list.
    state.nextCandidate = candidateBuffer.size();

    // Only the match cap stops the search from here; the interrupt poll belongs to the worker
    // loop, and folding it in would blur the two.
    return true;
}

void RIImpl::candidatesFor(const State &state, std::vector<node> &out) const {
    const index pos = state.depth;
    const node pu = ordering->order[pos];
    const index parentPos = ordering->parent[pos];

    // Indexed by pattern node, and `pu` is already in a register, so RI-Ds costs nothing extra
    // here that plain RI does not pay.
    const std::vector<node> *builtDomain =
        domains->ofPatternNode.empty() ? nullptr : &domains->ofPatternNode[pu];

    if (parentPos == none) {
        // This position starts a new connected component, so nothing structural constrains it.
        // Here the domain is used whatever its selectivity, because it is not filtering a slice -
        // it *replaces* a scan of every id in the target, and it can only be shorter. That is why
        // RI-Ds pays off on a disconnected pattern.
        if (builtDomain != nullptr) {
            out.insert(out.end(), builtDomain->begin(), builtDomain->end());
            return;
        }

        // The hasNode() filter is not optional: a removed id and an isolated node have identical
        // empty slices.
        for (node tv = 0; tv < targetGraph->upperNodeIdBound(); ++tv)
            if (targetGraph->hasNode(tv))
                out.push_back(tv);
        return;
    }

    // On a slice, though, the domain has to earn its place: everything it rejects that the sweep
    // did not find is something consistent() rejects more cheaply than a binary search can.
    const std::vector<node> *domain =
        builtDomain != nullptr && domains->earnsItsKeep[pu] ? builtDomain : nullptr;

    const node pp = ordering->order[parentPos];
    const node parentImage = state.mapping[parentPos];

    // A parent is adjacent in at least one direction, so exactly one of these branches applies.
    // Walking the wrong slice loses real matches.
    const bool forward = patternGraph->hasEdge(pp, pu);
    const bool backward = patternGraph->isDirected() && patternGraph->hasEdge(pu, pp);
    bool useOut = forward;
    if (forward && backward) {
        // A superset suffices, because ruleEdgesToPrefix() rechecks both directions anyway.
        useOut = targetGraph->outDegree(parentImage) <= targetGraph->inDegree(parentImage);
    }

    const node *begin =
        useOut ? targetGraph->outBegin(parentImage) : targetGraph->inBegin(parentImage);
    const node *end = useOut ? targetGraph->outEnd(parentImage) : targetGraph->inEnd(parentImage);
    const index *sliceLabels =
        useOut ? targetGraph->outLabelBegin(parentImage) : targetGraph->inLabelBegin(parentImage);
    const index patternLabel =
        useOut ? patternGraph->edgeLabel(pp, pu) : patternGraph->edgeLabel(pu, pp);

    // The label sits at the same offset as the head, so filtering the slice by the parent edge's
    // label costs one comparison per candidate and no extra lookup - the cheapest place in the
    // whole search to spend an edge label. A `none` on the pattern side is a wildcard and needs no
    // filtering at all.
    const bool filterLabels = sliceLabels != nullptr && patternLabel != none;

    auto inDomain = domain == nullptr ? std::vector<node>::const_iterator{} : domain->begin();
    for (const node *it = begin; it != end; ++it) {
        const node tv = *it;

        if (filterLabels && !edgeLabelsCompatible(patternLabel, sliceLabels[it - begin]))
            continue;

        // Both ranges are strictly ascending, so RI-Ds walks them together and never looks a
        // candidate up from the front. The search is from the previous entry's position, which
        // keeps the merge's monotone progress while costing log(domain) rather than a walk across
        // the whole domain when the slice is much the shorter of the two - the usual shape here,
        // since a target neighbourhood is small and an unlabelled domain is nearly every node.
        if (domain != nullptr) {
            inDomain = std::lower_bound(inDomain, domain->end(), tv);
            if (inDomain == domain->end())
                return;
            if (*inDomain != tv)
                continue;
        }

        // Target nodes already used are deliberately *not* filtered here; ruleEdgesToPrefix()
        // rejects them, and keeping injectivity in one place avoids a used[] array that expand()
        // could not reconstruct from a stolen State without an O(depth) rebuild.
        out.push_back(tv);
    }
}

bool RIImpl::consistent(const State &state, node tv) const {
    const node pu = ordering->order[state.depth];

    // Cheapest gate first. On a sparse target degree domination rejects most candidates before an
    // edge is touched, and the paper's condition 3 exists precisely because it "often obviates the
    // need for the substantial work needed to verify condition 4".
    //
    // hasNode(tv) is not rechecked: candidatesFor() guarantees it on every path.
    if (targetGraph->outDegree(tv) < patternGraph->outDegree(pu))
        return false;

    if (patternGraph->isDirected() && targetGraph->inDegree(tv) < patternGraph->inDegree(pu))
        return false;

    if (!ruleLabels(pu, tv))
        return false;

    if (!ruleEdgesToPrefix(state, tv))
        return false;

    return semantics != SubgraphIsomorphism::Semantics::INDUCED || ruleNonEdgesToPrefix(state, tv);
}

bool RIImpl::ruleEdgesToPrefix(const State &state, node tv) const {
    const node pu = ordering->order[state.depth];
    const bool edgeLabelled = patternGraph->hasEdgeLabels();

    for (index i = 0; i < state.depth; ++i) {
        const node ti = state.mapping[i];

        // Injectivity lives here, and only here.
        if (ti == tv)
            return false;

        const node pi = ordering->order[i];

        if (patternGraph->hasEdge(pi, pu)) {
            if (!targetGraph->hasEdge(ti, tv))
                return false;
            if (edgeLabelled
                && !edgeLabelsCompatible(patternGraph->edgeLabel(pi, pu),
                                         targetGraph->edgeLabel(ti, tv)))
                return false;
        }

        // Undirected needs no second test because hasEdge() is symmetric on both snapshots. A
        // directed mutual pair does, and independently in each direction, because its two arcs are
        // two edge ids and can carry different labels.
        if (patternGraph->isDirected() && patternGraph->hasEdge(pu, pi)) {
            if (!targetGraph->hasEdge(tv, ti))
                return false;
            if (edgeLabelled
                && !edgeLabelsCompatible(patternGraph->edgeLabel(pu, pi),
                                         targetGraph->edgeLabel(tv, ti)))
                return false;
        }
    }

    return true;
}

bool RIImpl::ruleNonEdgesToPrefix(const State &state, node tv) const {
    const node pu = ordering->order[state.depth];

    // Over *all* earlier positions, not just neighbours. Getting that wrong is the classic
    // failure: without it, Semantics::INDUCED silently computes a monomorphism. Edge labels play
    // no part - they constrain only edges that exist.
    for (index i = 0; i < state.depth; ++i) {
        const node pi = ordering->order[i];
        const node ti = state.mapping[i];

        if (!patternGraph->hasEdge(pi, pu) && targetGraph->hasEdge(ti, tv))
            return false;

        if (patternGraph->isDirected() && !patternGraph->hasEdge(pu, pi)
            && targetGraph->hasEdge(tv, ti))
            return false;
    }

    return true;
}

bool RIImpl::ruleLabels(node pu, node tv) const {
    return nodeLabelsCompatible(*patternNodeLabels, *targetNodeLabels, pu, tv);
}

RIImpl::Domains RIImpl::computeDomains(const SearchGraph &pattern, const SearchGraph &target,
                                       const std::vector<index> &patternNodeLabels,
                                       const std::vector<index> &targetNodeLabels,
                                       RI::Variant variant) {
    Domains result;
    if (variant != RI::Variant::RI_DS)
        return result;

    const count z = pattern.upperNodeIdBound();
    if (z == 0)
        return result;

    // Indexed by pattern node id, not by position: the order is computed from these, so it does
    // not exist yet. Ids that are not nodes keep an empty domain that nothing ever reads.
    result.ofPatternNode.assign(z, {});
    result.earnsItsKeep.assign(z, false);

    // ## Built once
    //
    // Walking target ids ascending leaves every domain sorted for free, which every consumer
    // relies on.
    for (node pu = 0; pu < z; ++pu) {
        if (!pattern.hasNode(pu))
            continue;

        std::vector<node> &domain = result.ofPatternNode[pu];
        for (node tv = 0; tv < target.upperNodeIdBound(); ++tv)
            if (couldMap(pattern, target, patternNodeLabels, targetNodeLabels, pu, tv))
                domain.push_back(tv);
    }

    // ## Refinement sweep, a single pass
    //
    // Deliberately not run to convergence, and deliberately separable from the half above: on a
    // large unlabelled target a domain approaches the whole node set, which makes this the one
    // costly part of RI-Ds. It is pure pruning, so skipping it is always safe - which is what
    // makes gating it behind a size threshold an option if it ever fails to pay.
    //
    // Refining in place is sound: a partly refined domain is still a superset of the images that
    // pattern node can take, since every removal took away a node that cannot be one. Because the
    // pass is single and refines in place, which domains it manages to shrink depends on the order
    // it walks them in - and that order is now ascending node id rather than position in the
    // matching order, which is what makes the result independent of an order that does not exist
    // yet. Both walks are sound; they simply leave slightly different supersets.
    std::vector<count> builtSize(z, 0);

    // Reused across pattern nodes, so the sweep allocates once for the whole pass rather than once
    // per node.
    std::vector<ArcConstraint> constraints;

    for (node pu = 0; pu < z; ++pu) {
        if (!pattern.hasNode(pu))
            continue;

        std::vector<node> &domain = result.ofPatternNode[pu];
        builtSize[pu] = domain.size();

        // Which domains a candidate has to reach, across which kind of arc, and under which edge
        // label depends on `pu` alone - not on the candidate. So it is collected once here, before
        // the candidates are walked. Reading it off the pattern's own slices is also what makes the
        // cost proportional to pu's degree: asking hasEdge(pu, pj) for every pattern id instead
        // would rediscover a handful of neighbours by rejecting all the rest, once per entry of a
        // domain that on a large unlabelled target is nearly the whole node set.
        constraints.clear();
        collectArcConstraints(pattern, pu, constraints);

        const auto keep = [&](node tv) {
            // Direction follows the pattern arc exactly as in candidatesFor(); with edge labels the
            // required neighbourhood narrows to target arcs carrying a compatible label.
            for (const ArcConstraint &arc : constraints) {
                const node *begin = arc.outgoing ? target.outBegin(tv) : target.inBegin(tv);
                const node *end = arc.outgoing ? target.outEnd(tv) : target.inEnd(tv);
                const index *labels = arc.label == none ? nullptr
                                                        : (arc.outgoing ? target.outLabelBegin(tv)
                                                                        : target.inLabelBegin(tv));

                if (!intersectsDomain(begin, end, labels, arc.label, result.ofPatternNode[arc.pj]))
                    return false;
            }

            return true;
        };

        domain.erase(
            std::remove_if(domain.begin(), domain.end(), [&](node tv) { return !keep(tv); }),
            domain.end());
    }

    // ## Forward checking, one step
    //
    // After the sweep, because it can only gain from the smaller domains the sweep leaves, and
    // before the order, because a singleton it manufactures is what the singletons-first rule in
    // computeOrdering() acts on. Called exactly once; its inner worklist is what handles the
    // singletons its own removals create, and it does so before this one call returns.
    result.anyEmpty = !forwardCheckSingletons(pattern, result.ofPatternNode);

    // ## What the pruning was worth
    //
    // Measured against what the *build* left, not against the target: the build's own removals are
    // duplicated for free by consistent()'s first two gates, so only what the sweep and the forward
    // check removed can pay for a binary search per candidate. Forward checking counts because
    // ruleEdgesToPrefix() enforces injectivity only against *already-mapped* positions, and so can
    // never reproduce a removal justified by a node that will be mapped later. See
    // MinSweepYieldForSliceIntersection.
    for (node pu = 0; pu < z; ++pu) {
        if (!pattern.hasNode(pu))
            continue;

        const count built = builtSize[pu];
        const count pruned = built - result.ofPatternNode[pu].size();
        result.earnsItsKeep[pu] =
            built != 0
            && static_cast<double>(pruned)
                   >= MinSweepYieldForSliceIntersection * static_cast<double>(built);
    }

    return result;
}

bool RIImpl::reportMapping(const State &state) {
    // The single highest-risk line in this file, so it is written in exactly one place.
    // state.mapping is indexed by POSITION in the order; matchBuffer is indexed by PATTERN NODE
    // id. Backwards, this still produces plausible-looking but wrong matches.
    for (index i = 0; i < ordering->order.size(); ++i)
        matchBuffer[ordering->order[i]] = state.mapping[i];

    // No reset afterwards: a reported state is always at full depth, so every complete mapping
    // writes the same set of positions and nothing stale survives. Ids that are not nodes keep the
    // `none` the constructor put there.
    return report(matchBuffer);
}

RISearchSetup prepareRISearch(const Graph &pattern, const Graph &target,
                              const std::vector<index> &patternNodeLabels,
                              const std::vector<index> &targetNodeLabels,
                              const std::vector<index> &patternEdgeLabels,
                              const std::vector<index> &targetEdgeLabels, RI::Variant variant,
                              const std::string &algorithmName) {
    // The pattern is small, so it can afford the adjacency matrix that makes hasEdge() constant
    // time; the target cannot, and falls back to a binary search over its sorted neighbours.
    SearchGraph patternGraph(pattern, /* buildMatrix = */ true, patternEdgeLabels);
    SearchGraph targetGraph(target, /* buildMatrix = */ false, targetEdgeLabels);

    // RI matches edge labels, so it refuses only what no snapshot can represent: parallel edges
    // whose labels disagree, which the compaction has to collapse into one arc that cannot carry
    // both. The compaction is what notices, and it runs here rather than in setEdgeLabels(), which
    // is why the refusal is late. Parallel edges carrying the same label collapse losslessly and
    // are not affected. Raised here, at the front of the preparation, so ParallelRI has no worker
    // left to unwind.
    if (patternGraph.collapsedLabelledEdges() || targetGraph.collapsedLabelledEdges())
        throw std::runtime_error(algorithmName
                                 + " does not support parallel edges whose edge labels disagree - "
                                   "see SubgraphIsomorphism::setEdgeLabels()");

    // The domains come first, because under RI-DS the order is computed from their sizes. Empty and
    // free under plain RI. ParallelRI shares this one copy with every worker, which is what
    // replaced one scan of the whole target per worker.
    RIImpl::Domains domains = RIImpl::computeDomains(patternGraph, targetGraph, patternNodeLabels,
                                                     targetNodeLabels, variant);

    // Deciding the matching order is the expensive part of RI.
    RIImpl::Ordering ordering = RIImpl::computeOrdering(patternGraph, domains);

    return RISearchSetup{std::move(patternGraph), std::move(targetGraph), std::move(domains),
                         std::move(ordering)};
}

} // namespace IsomorphismDetails
} // namespace NetworKit
