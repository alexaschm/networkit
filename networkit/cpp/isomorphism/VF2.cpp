#include <stdexcept>
#include <vector>

#include <tlx/unused.hpp>

#include <networkit/Globals.hpp>
#include <networkit/isomorphism/VF2.hpp>

#include "SearchGraph.hpp"

namespace NetworKit {

namespace {

using IsomorphismDetails::SearchGraph;

/**
 * The actual VF2 search.
 *
 * Lives here rather than in VF2.hpp so that the public header never has to mention the search
 * state. Everything below is created when VF2::run() starts and thrown away when it returns,
 * which is the same shape MaximalCliquesImpl uses in networkit/cpp/clique/MaximalCliques.cpp.
 *
 * ## The state, in words
 *
 * The partial mapping is stored twice, once in each direction: `core1[u]` is the target node
 * that pattern node @a u is mapped to, and `core2[v]` is the pattern node that target node @a v
 * is mapped to. Both hold @ref none where nothing is mapped yet. Keeping both directions means
 * "is this target node already taken" is a single array read.
 *
 * The four terminal sets are *not* stored as sets. Instead, `out1[u]` holds the search depth at
 * which pattern node @a u entered the out-terminal set, or 0 if it is not in it. Membership is
 * then one comparison, and undoing a step on backtrack is "reset every entry whose depth equals
 * the depth we are leaving" rather than an expensive set removal. `in1`, `in2` and `out2` work
 * the same way for the other three sets. `t1out` and friends are just the sizes, kept up to date
 * incrementally because the look-ahead rules compare them.
 */
class VF2Impl {

public:
    /**
     * @param pattern Snapshot of the pattern, built with the adjacency matrix.
     * @param target Snapshot of the target, built without it.
     * @param patternLabels Empty when the search is unlabelled.
     * @param targetLabels Empty when the search is unlabelled.
     * @param semantics Whether matches must be induced.
     * @param sink Where complete mappings are reported. VF2 is sequential, so this is sink(0).
     */
    VF2Impl(const Graph &pattern, const Graph &target, const std::vector<index> &patternLabels,
            const std::vector<index> &targetLabels, SubgraphIsomorphism::Semantics semantics,
            SubgraphIsomorphism::MatchSink sink)
        : patternGraph(pattern, /* buildMatrix = */ true),
          targetGraph(target, /* buildMatrix = */ false), patternLabels(&patternLabels),
          targetLabels(&targetLabels), labelled(!patternLabels.empty()), semantics(semantics),
          sink(sink), t1in(0), t1out(0), t2in(0), t2out(0), nodesVisited(0) {}

    /**
     * Search for every match and report each one to the sink.
     *
     * TODO: implement.
     *  1. Size core1/core2 to the two upper node id bounds and fill them with `none`; size
     *     in1/out1/in2/out2 to the same and fill them with 0; size `mapping` to the pattern's
     *     upper node id bound. Reserve `mapping` once and reuse it - a match is reported by
     *     reference, so it must not be reallocated per match.
     *  2. Handle the trivial cases up front: an empty pattern matches once, and a pattern with
     *     more nodes or a higher maximum degree than the target can never match at all.
     *  3. Call match(0) and let the recursion do the rest.
     */
    void run() {
        // TODO: remove once implemented.
        tlx::unused(patternGraph, targetGraph, patternLabels, targetLabels, labelled, semantics,
                    sink, core1, core2, in1, out1, in2, out2, t1in, t1out, t2in, t2out, mapping,
                    nodesVisited);
        throw std::logic_error("VF2Impl::run() is not implemented yet");
    }

private:
    /**
     * One level of the depth-first search: extend a mapping of @a depth pairs by one more.
     *
     * TODO: implement.
     *  1. If @a depth equals the number of pattern nodes, the mapping is complete: report it and
     *     return what reportMapping() returned, so that a false travels all the way back up and
     *     stops the search.
     *  2. Otherwise loop over the candidate pairs from nextCandidatePair(). For each, test
     *     feasible(); if it passes, addPair(), recurse into match(depth + 1), then removePair()
     *     regardless of the outcome. Propagate a false result upward immediately.
     *  3. Return true when the loop runs out - that means "no match down here, but keep going".
     *
     * @return false if the whole search must stop, true otherwise.
     */
    bool match(count depth) {
        tlx::unused(depth);
        throw std::logic_error("VF2Impl::match() is not implemented yet");
    }

    /**
     * Produce the next candidate pair to try at this depth.
     *
     * TODO: implement.
     *  1. If both out-terminal sets are non-empty, pair the smallest unmapped pattern node in
     *     the pattern's out-terminal set with each target node in the target's out-terminal set.
     *  2. Otherwise, if both in-terminal sets are non-empty, do the same with those.
     *  3. Otherwise the mapping is not connected to anything, so fall back to the smallest
     *     unmapped pattern node paired with every unmapped target node.
     *  Fixing the pattern node and only varying the target node is what keeps the search tree
     *  from exploding: every candidate at a given depth extends the same pattern node.
     *
     * @param depth Current search depth.
     * @param cursor In/out: where the previous call stopped, so iteration can resume.
     * @param pu Out: the pattern node to map.
     * @param tv Out: the target node to try for it.
     * @return false when the candidates at this depth are exhausted.
     */
    bool nextCandidatePair(count depth, index &cursor, node &pu, node &tv) const {
        tlx::unused(depth, cursor, pu, tv);
        throw std::logic_error("VF2Impl::nextCandidatePair() is not implemented yet");
    }

    /**
     * Whether the pair (@a pu, @a tv) may be added to the mapping.
     *
     * TODO: implement by calling the five rules below in increasing order of cost, returning
     * false at the first that fails. Cheap rejections first is the whole point.
     */
    bool feasible(node pu, node tv, count depth) const {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF2Impl::feasible() is not implemented yet");
    }

    /**
     * Consistency rule for out-edges.
     *
     * TODO: implement. For every out-neighbour of @a pu that is already mapped, the target must
     * contain the corresponding edge out of @a tv. Under Semantics::INDUCED the converse is also
     * required: every mapped out-neighbour of @a tv must correspond to an out-neighbour of
     * @a pu, which is what forbids extra target edges. Under MONOMORPHISM that half is skipped.
     */
    bool ruleSuccessors(node pu, node tv) const {
        tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleSuccessors() is not implemented yet");
    }

    /**
     * Consistency rule for in-edges. The mirror image of @ref ruleSuccessors().
     *
     * TODO: implement. For an undirected graph this is the same test as ruleSuccessors(), so it
     * can return true immediately and let that one do the work.
     */
    bool rulePredecessors(node pu, node tv) const {
        tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::rulePredecessors() is not implemented yet");
    }

    /**
     * One-step look-ahead on the terminal sets.
     *
     * TODO: implement. Count how many neighbours of @a pu lie in each pattern terminal set and
     * how many neighbours of @a tv lie in the corresponding target terminal set. If the pattern
     * count ever exceeds the target count, there will not be enough target nodes left to map
     * them onto, so reject now rather than discovering it several levels deeper.
     */
    bool ruleTerminalCounts(node pu, node tv) const {
        tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleTerminalCounts() is not implemented yet");
    }

    /**
     * Two-step look-ahead on the nodes outside both the mapping and the terminal sets.
     *
     * TODO: implement. Same idea as @ref ruleTerminalCounts(), but counting the neighbours that
     * are in neither the mapping nor any terminal set. Only meaningful under
     * Semantics::INDUCED; return true immediately for MONOMORPHISM, where extra target edges are
     * allowed and the counting argument does not hold.
     */
    bool ruleNewCounts(node pu, node tv) const {
        tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleNewCounts() is not implemented yet");
    }

    /**
     * Label rule.
     *
     * TODO: implement. Return true immediately when the search is unlabelled. Otherwise the two
     * labels must be equal, except that @ref none on either side is a wildcard that matches
     * anything.
     */
    bool ruleLabels(node pu, node tv) const {
        tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleLabels() is not implemented yet");
    }

    /**
     * Add (@a pu, @a tv) to the mapping and update the four terminal sets.
     *
     * TODO: implement.
     *  1. Set core1[pu] = tv and core2[tv] = pu.
     *  2. Remove both from the terminal sets they were in, adjusting the sizes.
     *  3. For every neighbour of pu and of tv that is not yet mapped and not yet in the relevant
     *     terminal set, record @a depth as its entry depth and bump the size. Storing the depth
     *     rather than a flag is what makes removePair() able to undo exactly this step.
     */
    void addPair(node pu, node tv, count depth) {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF2Impl::addPair() is not implemented yet");
    }

    /**
     * Undo @ref addPair() exactly.
     *
     * TODO: implement. Reset core1[pu] and core2[tv] to `none`, and clear every terminal-set
     * entry whose recorded depth equals @a depth, restoring the sizes. Because entries are
     * stamped with the depth that created them, this touches only what this level added.
     */
    void removePair(node pu, node tv, count depth) {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF2Impl::removePair() is not implemented yet");
    }

    /**
     * Hand a complete mapping to the sink.
     *
     * TODO: implement. Copy core1 into `mapping` for the pattern nodes that exist, then return
     * sink.report(mapping). Reuse the same buffer every time; the sink copies it if it needs to.
     */
    bool reportMapping() { throw std::logic_error("VF2Impl::reportMapping() is not implemented"); }

    /**
     * Let the user interrupt a long search.
     *
     * TODO: implement. Increment `nodesVisited` and, when the low bits are zero, ask
     * `Aux::SignalHandler` whether to keep running. Checking every single node would cost more
     * than the search itself, hence the mask.
     */
    void checkSignal() { throw std::logic_error("VF2Impl::checkSignal() is not implemented"); }

    SearchGraph patternGraph;
    SearchGraph targetGraph;

    const std::vector<index> *patternLabels;
    const std::vector<index> *targetLabels;
    bool labelled;

    SubgraphIsomorphism::Semantics semantics;
    SubgraphIsomorphism::MatchSink sink;

    /// core1[patternNode] = target node it is mapped to, or `none`.
    std::vector<node> core1;
    /// core2[targetNode] = pattern node mapped onto it, or `none`.
    std::vector<node> core2;

    /// Depth at which each node entered the corresponding terminal set; 0 means "not in it".
    std::vector<count> in1, out1, in2, out2;
    /// Current sizes of the four terminal sets.
    count t1in, t1out, t2in, t2out;

    /// Reused buffer handed to the sink, so a match costs no allocation.
    std::vector<node> mapping;

    /// Counts recursion steps so checkSignal() can poll only every so often.
    count nodesVisited;
};

} // namespace

VF2::VF2(const Graph &pattern, const Graph &target, Semantics semantics, count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches) {}

void VF2::run() {
    prepareRun();
    VF2Impl(*pattern, *target, patternLabels, targetLabels, semantics, sink()).run();
    finishRun();
}

} // namespace NetworKit
