#include <stdexcept>
#include <vector>

#include <tlx/unused.hpp>

#include <networkit/Globals.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/isomorphism/VF3.hpp>

#include "MatchReporter.hpp"
#include "SearchGraph.hpp"

namespace NetworKit {

namespace {

using IsomorphismDetails::MatchReporter;
using IsomorphismDetails::SearchGraph;

/**
 * The actual VF3 search.
 *
 * Same arrangement as VF2Impl in VF2.cpp: it lives here so the public header stays free of search
 * state, and it is created and destroyed inside VF3::run().
 *
 * ## The state, in words
 *
 * The mapping is kept in both directions, `core1` and `core2`, exactly as in VF2.
 *
 * What VF3 adds is all precomputed before the search starts and never changes afterwards:
 *
 * - `patternClass` / `targetClass` put every node into a class derived from its label, and
 *   `targetClassSize` counts how many target nodes each class has.
 * - `order` is the fixed sequence in which pattern nodes get mapped, and `orderParent[i]` is the
 *   position earlier in `order` that position @a i is connected to, or `none` if it starts a new
 *   component. Candidates for position @a i are then drawn from the neighbours of whatever the
 *   parent was mapped to, instead of from the whole target.
 * - `feasibilitySets` holds, per depth and per class, how many nodes of that class the remaining
 *   pattern still needs. The pruning rule is then a lookup and a comparison rather than a walk
 *   over neighbourhoods.
 */
class VF3Impl {

public:
    /**
     * @param pattern Snapshot of the pattern, built with the adjacency matrix.
     * @param target Snapshot of the target, built without it.
     * @param patternNodeLabels Empty when the search is unlabelled; then there is one class.
     * @param targetNodeLabels Empty when the search is unlabelled.
     * @param semantics Whether matches must be induced.
     * @param handler Polled so a long search can be stopped with CTRL+C.
     * @param report Where complete mappings are reported.
     */
    VF3Impl(const Graph &pattern, const Graph &target, const std::vector<index> &patternNodeLabels,
            const std::vector<index> &targetNodeLabels, SubgraphIsomorphism::Semantics semantics,
            Aux::SignalHandler &handler, MatchReporter report)
        : patternGraph(pattern, /* buildMatrix = */ true),
          targetGraph(target, /* buildMatrix = */ false), patternNodeLabels(&patternNodeLabels),
          targetNodeLabels(&targetNodeLabels), nodeLabelled(!patternNodeLabels.empty()),
          semantics(semantics), handler(&handler), report(std::move(report)), numberOfClasses(0) {}

    /**
     * Search for every match and report each one.
     *
     * TODO: implement.
     *  1. classifyNodes(), then computeNodeOrder(), then precomputeFeasibilitySets(). In that
     *     order - each step needs the previous one.
     *  2. Size core1/core2 and the reusable `mapping` buffer, filling the cores with `none`.
     *  3. Bail out early if any class needs more pattern nodes than the target has: that is a
     *     guaranteed no-match and costs one pass over the class counts to spot.
     *  4. Call match(0).
     */
    void run() {
        // TODO: remove once implemented.
        tlx::unused(patternGraph, targetGraph, patternNodeLabels, targetNodeLabels, nodeLabelled,
                    semantics, handler, report, patternClass, targetClass, targetClassSize,
                    numberOfClasses, order, orderParent, feasibilitySets, core1, core2, mapping);
        throw std::logic_error("VF3Impl::run() is not implemented yet");
    }

private:
    /**
     * Group the nodes of both graphs into classes.
     *
     * TODO: implement. Map each distinct label to a small dense class id and fill
     * patternClass/targetClass with it, counting the target's classes into targetClassSize. When
     * the search is unlabelled every node goes into class 0, which makes the rest of the
     * algorithm degrade gracefully into a statically ordered VF2.
     */
    void classifyNodes() {
        throw std::logic_error("VF3Impl::classifyNodes() is not implemented yet");
    }

    /**
     * Compute the fixed order in which pattern nodes will be mapped.
     *
     * TODO: implement. Greedily append the pattern node that scores best on, in order of
     * importance: most edges into the nodes already placed in the order, then lowest
     * classProbability(), then highest degree. Record in orderParent[i] the position of an
     * already-ordered neighbour, or `none` if there is none, so candidate enumeration can start
     * from the parent's image rather than from the whole target. Section 4 of the paper.
     *
     * Reuse: the greedy loop itself is specific to VF3 and has to be written. If the target side
     * ever needs a cheap structural ordering to break ties with, CoreDecomposition::getNodeOrder()
     * already provides a degeneracy ordering; MaximalCliques.cpp shows the three lines it takes.
     */
    void computeNodeOrder() {
        throw std::logic_error("VF3Impl::computeNodeOrder() is not implemented yet");
    }

    /**
     * How likely a random target node is to belong to class @a cls.
     *
     * TODO: implement as targetClassSize[cls] divided by the number of target nodes. Rare classes
     * come out small, and computeNodeOrder() prefers them because a rare class yields few
     * candidates and therefore prunes hard.
     */
    double classProbability(index cls) const {
        tlx::unused(cls);
        throw std::logic_error("VF3Impl::classProbability() is not implemented yet");
    }

    /**
     * Precompute, per depth and per class, how many nodes the rest of the pattern still needs.
     *
     * TODO: implement. Walk `order` backwards accumulating the class counts, and store the
     * running totals into feasibilitySets so that feasibilitySets[depth][cls] answers "from this
     * depth on, how many more nodes of class cls does the pattern require". ruleClassCounts()
     * then compares that against what the target still has available.
     */
    void precomputeFeasibilitySets() {
        throw std::logic_error("VF3Impl::precomputeFeasibilitySets() is not implemented yet");
    }

    /**
     * One level of the depth-first search: map the pattern node at position @a depth in `order`.
     *
     * TODO: implement.
     *  1. If @a depth equals the number of pattern nodes, report the mapping and return the
     *     reporter's answer so a false stops the whole search.
     *  2. Otherwise get the candidates for this depth, and for each one test feasible(), then
     *     addPair(), recurse, removePair(). Propagate a false upward at once.
     *
     * @return false if the whole search must stop, true otherwise.
     */
    bool match(count depth) {
        tlx::unused(depth);
        throw std::logic_error("VF3Impl::match() is not implemented yet");
    }

    /**
     * Collect the target nodes worth trying at position @a depth.
     *
     * TODO: implement. If orderParent[depth] is set, the pattern node has an already-mapped
     * neighbour, so the candidates are exactly the neighbours of that neighbour's image - a
     * handful of nodes rather than the whole graph. If it is `none`, this position starts a new
     * component and every unmapped target node of the right class is a candidate.
     *
     * @param depth Current position in `order`.
     * @param out Filled with the candidates; cleared first.
     */
    void candidatesFor(count depth, std::vector<node> &out) const {
        tlx::unused(depth, out);
        throw std::logic_error("VF3Impl::candidatesFor() is not implemented yet");
    }

    /**
     * Whether the pair (@a pu, @a tv) may be added to the mapping at @a depth.
     *
     * TODO: implement by calling ruleLabels(), then ruleEdges(), then ruleClassCounts(), cheapest
     * first, returning false at the first failure.
     */
    bool feasible(node pu, node tv, count depth) const {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF3Impl::feasible() is not implemented yet");
    }

    /**
     * Consistency rule against everything already mapped.
     *
     * TODO: implement. Every edge between @a pu and an already-mapped pattern node must have a
     * counterpart between @a tv and that node's image, in both directions for a directed graph.
     * Under Semantics::INDUCED the converse is required as well - a target edge between @a tv and
     * a mapped node with no matching pattern edge disqualifies the pair. Under MONOMORPHISM extra
     * target edges are fine, so that half is skipped.
     */
    bool ruleEdges(node pu, node tv, count depth) const {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF3Impl::ruleEdges() is not implemented yet");
    }

    /**
     * The look-ahead that makes VF3 different from VF2.
     *
     * TODO: implement. For each class, compare what the remaining pattern still needs -
     * feasibilitySets[depth] - against how many unmapped target nodes of that class are still
     * reachable. If the pattern needs more of some class than remains available, no extension can
     * possibly succeed, so reject here instead of finding out several levels deeper.
     */
    bool ruleClassCounts(node pu, node tv, count depth) const {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF3Impl::ruleClassCounts() is not implemented yet");
    }

    /**
     * Label rule.
     *
     * TODO: implement. True immediately when unlabelled. Otherwise the classes must agree, with
     * @ref none acting as a wildcard on either side.
     */
    bool ruleLabels(node pu, node tv) const {
        tlx::unused(pu, tv);
        throw std::logic_error("VF3Impl::ruleLabels() is not implemented yet");
    }

    /**
     * Add (@a pu, @a tv) to the mapping.
     *
     * TODO: implement. Set core1[pu] = tv and core2[tv] = pu, and update whatever running
     * per-class availability counters ruleClassCounts() reads. Unlike VF2 there are no terminal
     * sets to maintain, because the matching order is fixed in advance.
     */
    void addPair(node pu, node tv, count depth) {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF3Impl::addPair() is not implemented yet");
    }

    /**
     * Undo @ref addPair() exactly.
     *
     * TODO: implement. Reset core1[pu] and core2[tv] to `none` and restore the counters addPair()
     * changed.
     */
    void removePair(node pu, node tv, count depth) {
        tlx::unused(pu, tv, depth);
        throw std::logic_error("VF3Impl::removePair() is not implemented yet");
    }

    /**
     * Hand a complete mapping over.
     *
     * TODO: implement. Copy core1 into `mapping` and return report(mapping). Note that
     * `mapping` must be indexed by pattern node, not by position in `order` - the caller of this
     * class knows nothing about the internal ordering.
     */
    bool reportMapping() { throw std::logic_error("VF3Impl::reportMapping() is not implemented"); }

    SearchGraph patternGraph;
    SearchGraph targetGraph;

    const std::vector<index> *patternNodeLabels;
    const std::vector<index> *targetNodeLabels;
    bool nodeLabelled;

    SubgraphIsomorphism::Semantics semantics;

    /// Call `handler->assureRunning()` in the recursion; it throws to abort a long search.
    /// VF3 is sequential, so the throwing form is safe here - see Betweenness.cpp for what a
    /// parallel search has to do instead.
    Aux::SignalHandler *handler;

    MatchReporter report;

    /// Class of every node, derived from its label. All zero when the search is unlabelled.
    std::vector<index> patternClass, targetClass;
    /// How many target nodes each class contains.
    std::vector<count> targetClassSize;
    count numberOfClasses;

    /// Fixed matching order: order[i] is the pattern node mapped at depth i.
    std::vector<node> order;
    /// orderParent[i] is the position in `order` that position i attaches to, or `none`.
    std::vector<index> orderParent;

    /// feasibilitySets[depth][cls] = nodes of class cls the pattern still needs from `depth` on.
    std::vector<std::vector<count>> feasibilitySets;

    /// core1[patternNode] = target node it is mapped to, or `none`.
    std::vector<node> core1;
    /// core2[targetNode] = pattern node mapped onto it, or `none`.
    std::vector<node> core2;

    /// Reused buffer handed to the reporter, so a match costs no allocation.
    std::vector<node> mapping;
};

} // namespace

VF3::VF3(const Graph &pattern, const Graph &target, Semantics semantics, count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches) {}

void VF3::run() {
    // Ahead of need, and deliberately so: VF3Impl is unwritten, so nothing can be silently wrong
    // today, but having the refusal in place first means whoever writes the search inherits a
    // defined answer rather than a gap. Same reasoning as VF2's - see there.
    if (isEdgeLabelled())
        throw std::runtime_error("VF3 does not support edge labels - see "
                                 "SubgraphIsomorphism::setEdgeLabels()");

    Aux::SignalHandler handler;
    prepareRun();
    VF3Impl(*pattern, *target, patternNodeLabels, targetNodeLabels, semantics, handler,
            [this](const Match &match) { return reportMatch(match); })
        .run();
    finishRun();
}

} // namespace NetworKit
