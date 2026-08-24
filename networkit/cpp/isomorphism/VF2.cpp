#include <stdexcept>
#include <vector>

#include <tlx/unused.hpp>

#include <networkit/Globals.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/isomorphism/VF2.hpp>

#include "MatchReporter.hpp"
#include "SearchGraph.hpp"

namespace NetworKit {

namespace {

using IsomorphismDetails::MatchReporter;
using IsomorphismDetails::SearchGraph;

/**
 * Issues:
 *
 * We start with match(0). If nodes as added to terminal sets at depth 0, they get the depth stamp 0
 * so it just reads as if they are not present. Current fix: Start with match(1)
 *
 * Is removePair supposed to exactly reverse addPair? Upon addition we remove pu, tv from all
 * terminal sets but dont store their depth so we are unable to restore this in removePair. Current
 * fix: Use a vector to store depth stamps of pair pu, tv before removing them from all terminal
 * sets in addPair(pu,tv)
 *
 * depth parameter in nextCandidatePair() is never used. If both in1 and in2 or both out1 and out2
 * are nonempty, in nextCandidatePair() we pair the smallest node in the first set with every node
 * in the second set. Could/should we use the depth parameter anywhere?
 *
 * TODO: More testcases?
 *
 * TODO: ruleLabels must check edge labels
 */

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
     * @param patternNodeLabels Empty when the search is unlabelled.
     * @param targetNodeLabels Empty when the search is unlabelled.
     * @param semantics Whether matches must be induced.
     * @param handler Polled so a long search can be stopped with CTRL+C.
     * @param report Where complete mappings are reported.
     */
    VF2Impl(const Graph &pattern, const Graph &target, const std::vector<index> &patternNodeLabels,
            const std::vector<index> &targetNodeLabels, SubgraphIsomorphism::Semantics semantics,
            Aux::SignalHandler &handler, MatchReporter report)
        : patternGraph(pattern, /* buildMatrix = */ true),
          targetGraph(target, /* buildMatrix = */ false), patternNodeLabels(&patternNodeLabels),
          targetNodeLabels(&targetNodeLabels), nodeLabelled(!patternNodeLabels.empty()),
          semantics(semantics), handler(&handler), report(std::move(report)), t1in(0), t1out(0),
          t2in(0), t2out(0) {}

    /**
     * Search for every match and report each one. Initialize core1/core2 and in1/out1/in2/out2.
     * Handle trivial cases immediately and call match(0) to start the recursion.
     */
    void run() {

        core1.assign(patternGraph.upperNodeIdBound(), none);
        core2.assign(targetGraph.upperNodeIdBound(), none);
        in1.assign(patternGraph.upperNodeIdBound(), 0);
        out1.assign(patternGraph.upperNodeIdBound(), 0);
        in2.assign(targetGraph.upperNodeIdBound(), 0);
        out2.assign(targetGraph.upperNodeIdBound(), 0);
        mapping.resize(patternGraph.upperNodeIdBound(), none);

        if (patternGraph.numberOfNodes() == 0) {
            reportMapping();
            return;
        }

        if (patternGraph.numberOfNodes() > targetGraph.numberOfNodes()
            || patternGraph.maxInDegree() > targetGraph.maxInDegree()
            || patternGraph.maxOutDegree() > targetGraph.maxOutDegree()) {
            return;
        }

        // TODO: This is supposed to be match(0)
        match(1);
    }

private:
    /**
     * One level of the depth-first search: extend a mapping of @a depth pairs by one more.
     *
     * @return false if the whole search must stop, true otherwise.
     */
    bool match(count depth) {

        std::vector<count> oldNodePairDepthStamps(4, 0);

        // If all pattern nodes are mapped, return mapping
        if (depth - 1 == patternGraph.numberOfNodes()) {
            return reportMapping();
        }

        index cursor = 0;
        node pu = none;
        node tv = none;
        bool continueSearch;

        // Iterate over all candidate pairs and if candidate pair is feasible, add pair and call
        // match(depth + 1)
        while (nextCandidatePair(depth, cursor, pu, tv)) {
            handler->assureRunning();
            if (feasible(pu, tv)) {
                oldNodePairDepthStamps = addPair(pu, tv, depth);
                continueSearch = match(depth + 1);
                // Remove pair independent of outcome and abort search if it must be stopped
                removePair(pu, tv, depth, oldNodePairDepthStamps);
                if (!continueSearch) {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Produce the next candidate pair to try at this depth.
     *
     * @param depth Current search depth.
     * @param cursor In/out: where the previous call stopped, so iteration can resume.
     * @param pu Out: the pattern node to map.
     * @param tv Out: the target node to try for it.
     * @return false when the candidates at this depth are exhausted.
     */
    bool nextCandidatePair(count depth, index &cursor, node &pu, node &tv) const {

        if (t1out != 0 && t2out != 0) {

            // Find smallest unmapped pattern node in out1
            for (index u = 0; u < core1.size(); ++u) {
                if (patternGraph.hasNode(u) && core1[u] == none && out1[u] != 0) {
                    pu = u;
                    break;
                }
            }
            // Pair with every unmapped target node in out2
            for (index v = cursor; v < core2.size(); ++v) {
                if (targetGraph.hasNode(v) && core2[v] == none && out2[v] != 0) {
                    tv = v;
                    cursor = v + 1;
                    return true;
                }
            }

            return false;

        } else if (t1in != 0 && t2in != 0) {

            // Find smallest unmapped pattern node in in1
            for (index u = 0; u < core1.size(); ++u) {
                if (patternGraph.hasNode(u) && core1[u] == none && in1[u] != 0) {
                    pu = u;
                    break;
                }
            }
            // Pair with every unmapped target node in in2
            for (index v = cursor; v < core2.size(); ++v) {
                if (targetGraph.hasNode(v) && core2[v] == none && in2[v] != 0) {
                    tv = v;
                    cursor = v + 1;
                    return true;
                }
            }

            return false;

        } else {

            // Find smallest unmapped pattern node
            for (index u = 0; u < core1.size(); ++u) {
                if (patternGraph.hasNode(u) && core1[u] == none) {
                    pu = u;
                    break;
                }
            }
            // Pair with every unmapped target node
            for (index v = cursor; v < core2.size(); ++v) {
                if (targetGraph.hasNode(v) && core2[v] == none) {
                    tv = v;
                    cursor = v + 1;
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Whether the pair (@a pu, @a tv) may be added to the mapping.
     */
    bool feasible(node pu, node tv) const {

        return ruleSuccessors(pu, tv) && rulePredecessors(pu, tv) && ruleTerminalCounts(pu, tv)
               && ruleNewCounts(pu, tv) && ruleLabels(pu, tv);
    }

    /**
     * Consistency rule for out-edges. For every out-neighbour of @a pu that is already mapped, the
     * target must contain the corresponding edge out of @a tv.
     */
    bool ruleSuccessors(node pu, node tv) const {

        // Check for every mapped out-neighbor of pu if the target has the corresponding edge out of
        // tv
        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] != none) {
                if (!targetGraph.hasEdge(tv, core1[u])) {
                    return false;
                }
            }
        }

        // Under Semantics::INDUCED: Check for every mapped out-neighbor of tv if the pattern has
        // the corresponding edge out of pu
        if (semantics == SubgraphIsomorphism::Semantics::INDUCED) {
            for (auto it = targetGraph.outBegin(tv); it != targetGraph.outEnd(tv); ++it) {
                node v = *it;
                if (core2[v] != none) {
                    if (!patternGraph.hasEdge(pu, core2[v])) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * Consistency rule for in-edges. The mirror image of @ref ruleSuccessors(). For every
     * in-neighbour of @a pu that is already mapped, the target must contain the corresponding edge
     * into @a tv.
     */
    bool rulePredecessors(node pu, node tv) const {

        // If undirected, in-neighbors=out-neighbors, return true immediately
        if (!patternGraph.isDirected()) {
            return true;
        }

        // Check for every mapped in-neighbor of pu if the target has the corresponding edge into tv
        for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
            node u = *it;
            if (core1[u] != none) {
                if (!targetGraph.hasEdge(core1[u], tv)) {
                    return false;
                }
            }
        }

        // Under Semantics::INDUCED: Check for every mapped in-neighbor of tv if the pattern has the
        // corresponding edge into pu
        if (semantics == SubgraphIsomorphism::Semantics::INDUCED) {
            for (auto it = targetGraph.inBegin(tv); it != targetGraph.inEnd(tv); ++it) {
                node v = *it;
                if (core2[v] != none) {
                    if (!patternGraph.hasEdge(core2[v], pu)) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * One-step look-ahead on the terminal sets. Count unmapped, out- and in-neighbors of pu and tv
     * that are part of a terminal set and return false if the pattern count exceeds the target
     * count.
     */
    bool ruleTerminalCounts(node pu, node tv) const {

        count in1Neighbors = 0;
        count out1Neighbors = 0;
        count in2Neighbors = 0;
        count out2Neighbors = 0;

        // Count unmapped, out-terminal out-neighbors of tv and pu
        for (auto it = targetGraph.outBegin(tv); it != targetGraph.outEnd(tv); ++it) {
            node v = *it;
            if (core2[v] == none && out2[v] != 0) {
                out2Neighbors++;
            }
        }
        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] == none && out1[u] != 0) {
                // Return false if pu has more such neighbors than tv
                if (++out1Neighbors > out2Neighbors) {
                    return false;
                }
            }
        }

        // If directed, do the same for in-neighbors
        if (patternGraph.isDirected()) {
            for (auto it = targetGraph.inBegin(tv); it != targetGraph.inEnd(tv); ++it) {
                node v = *it;
                if (core2[v] == none && in2[v] != 0) {
                    in2Neighbors++;
                }
            }
            for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
                node u = *it;
                if (core1[u] == none && in1[u] != 0) {
                    if (++in1Neighbors > in2Neighbors) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * Two-step look-ahead on the nodes outside both the mapping and the terminal sets. Count
     * unmapped out- and in-neighbors of pu and tv that are not part of any terminal set and return
     * false if the pattern count exceeds the target count.
     */
    bool ruleNewCounts(node pu, node tv) const {

        // Semantics::MONOMORPHISM allows extra target edges, return true immediately
        if (semantics == SubgraphIsomorphism::Semantics::MONOMORPHISM) {
            return true;
        }

        count in1Neighbors = 0;
        count out1Neighbors = 0;
        count in2Neighbors = 0;
        count out2Neighbors = 0;

        // Count unmapped, non-terminal out-neighbors of tv and pu
        for (auto it = targetGraph.outBegin(tv); it != targetGraph.outEnd(tv); ++it) {
            node v = *it;
            if (core2[v] == none && in2[v] == 0 && out2[v] == 0) {
                out2Neighbors++;
            }
        }
        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] == none && in1[u] == 0 && out1[u] == 0) {
                // Return false if pu has more such neighbors than tv
                if (++out1Neighbors > out2Neighbors) {
                    return false;
                }
            }
        }

        // If directed, do the same for in-neighbors
        if (patternGraph.isDirected()) {
            for (auto it = targetGraph.inBegin(tv); it != targetGraph.inEnd(tv); ++it) {
                node v = *it;
                if (core2[v] == none && in2[v] == 0 && out2[v] == 0) {
                    in2Neighbors++;
                }
            }
            for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
                node u = *it;
                if (core1[u] == none && in1[u] == 0 && out1[u] == 0) {
                    if (++in1Neighbors > in2Neighbors) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * Label rule. Return true immediately if the search is unlabelled. Otherwise the two
     * labels must be equal, except that @ref none on either side is a wildcard that matches
     * anything.
     */
    bool ruleLabels(node pu, node tv) const {

        if (!nodeLabelled) {
            return true;
        }

        if ((*patternNodeLabels)[pu] == (*targetNodeLabels)[tv] || (*patternNodeLabels)[pu] == none
            || (*targetNodeLabels)[tv] == none) {
            return true;
        }

        return false;
    }

    /**
     * Add (@a pu, @a tv) to the mapping and update the four terminal sets.
     *
     * @return oldNodePairDepthStamps depth stamps @a pu and @a tv had before
     * addPair(pu, tv) removed them from any terminal sets that they were part of
     */
    std::vector<count> addPair(node pu, node tv, count depth) {

        // Save at which depth pu and tv were added to their respective terminal sets
        std::vector<count> oldNodePairDepthStamps(4, 0);
        oldNodePairDepthStamps[0] = in1[pu];
        oldNodePairDepthStamps[1] = out1[pu];
        oldNodePairDepthStamps[2] = in2[tv];
        oldNodePairDepthStamps[3] = out2[tv];

        // Map pu and tv onto each other
        core1[pu] = tv;
        core2[tv] = pu;

        // Remove pu and tv from any terminal sets that they are part of by resetting their depth
        // stamps to zero
        if (in1[pu] != 0) {
            in1[pu] = 0;
            t1in--;
        }
        if (out1[pu] != 0) {
            out1[pu] = 0;
            t1out--;
        }
        if (in2[tv] != 0) {
            in2[tv] = 0;
            t2in--;
        }
        if (out2[tv] != 0) {
            out2[tv] = 0;
            t2out--;
        }

        // Unmapped neighbors of pu and tv are added to the respective terminal sets
        // If undirected, iterating over out-neighbors is sufficient, because in1=out1 and in2=out2
        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] == none && out1[u] == 0) {
                out1[u] = depth;
                t1out++;

                if (!patternGraph.isDirected()) {
                    in1[u] = depth;
                    t1in++;
                }
            }
        }

        for (auto it = targetGraph.outBegin(tv); it != targetGraph.outEnd(tv); ++it) {
            node v = *it;
            if (core2[v] == none && out2[v] == 0) {
                out2[v] = depth;
                t2out++;

                if (!patternGraph.isDirected()) {
                    in2[v] = depth;
                    t2in++;
                }
            }
        }

        // If directed, iterate over inNeighbors separately
        if (patternGraph.isDirected()) {
            for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
                node u = *it;
                if (core1[u] == none && in1[u] == 0) {
                    in1[u] = depth;
                    t1in++;
                }
            }

            for (auto it = targetGraph.inBegin(tv); it != targetGraph.inEnd(tv); ++it) {
                node v = *it;
                if (core2[v] == none && in2[v] == 0) {
                    in2[v] = depth;
                    t2in++;
                }
            }
        }

        return oldNodePairDepthStamps;
    }

    /**
     * Undo @ref addPair() exactly.
     */
    void removePair(node pu, node tv, count depth, std::vector<count> oldNodePairDepthStamps) {

        // Unmap pu and tv
        core1[pu] = none;
        core2[tv] = none;

        // Remove all nodes from the terminal sets that were added as a result of addPair(pu, tv)
        for (index u = 0; u < in1.size(); ++u) {
            if (in1[u] == depth) {
                in1[u] = 0;
                t1in--;
            }
            if (out1[u] == depth) {
                out1[u] = 0;
                t1out--;
            }
        }

        for (index v = 0; v < in2.size(); ++v) {
            if (in2[v] == depth) {
                in2[v] = 0;
                t2in--;
            }
            if (out2[v] == depth) {
                out2[v] = 0;
                t2out--;
            }
        }

        // If pu or tv were part of any terminal sets before addPair(pu, tv), restore their old
        // depth stamps
        if (oldNodePairDepthStamps[0] != 0) {
            in1[pu] = oldNodePairDepthStamps[0];
            t1in++;
        }
        if (oldNodePairDepthStamps[1] != 0) {
            out1[pu] = oldNodePairDepthStamps[1];
            t1out++;
        }
        if (oldNodePairDepthStamps[2] != 0) {
            in2[tv] = oldNodePairDepthStamps[2];
            t2in++;
        }
        if (oldNodePairDepthStamps[3] != 0) {
            out2[tv] = oldNodePairDepthStamps[3];
            t2out++;
        }
    }

    /**
     * Hand a complete mapping over. Copy core1 into 'mapping' for the pattern nodes that exist,
     * then return report(mapping)
     */
    bool reportMapping() {

        for (index u = 0; u < mapping.size(); ++u) {
            if (patternGraph.hasNode(u)) {
                mapping[u] = core1[u];
            }
        }

        return report(mapping);
    }

    SearchGraph patternGraph;
    SearchGraph targetGraph;

    const std::vector<index> *patternNodeLabels;
    const std::vector<index> *targetNodeLabels;
    bool nodeLabelled;

    SubgraphIsomorphism::Semantics semantics;

    /// Call `handler->assureRunning()` in the recursion; it throws to abort a long search.
    /// VF2 is sequential, so the throwing form is safe here - see Betweenness.cpp for what a
    /// parallel search has to do instead.
    Aux::SignalHandler *handler;

    MatchReporter report;

    /// core1[patternNode] = target node it is mapped to, or `none`.
    std::vector<node> core1;
    /// core2[targetNode] = pattern node mapped onto it, or `none`.
    std::vector<node> core2;

    /// Depth at which each node entered the corresponding terminal set; 0 means "not in it".
    std::vector<count> in1, out1, in2, out2;
    /// Current sizes of the four terminal sets.
    count t1in, t1out, t2in, t2out;

    /// Reused buffer handed to the reporter, so a match costs no allocation.
    std::vector<node> mapping;
};

} // namespace

VF2::VF2(const Graph &pattern, const Graph &target, Semantics semantics, count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches) {}

void VF2::run() {
    // VF2's feasibility rules compare *node* labels and nothing else, so an edge label set here
    // would simply be ignored and the matches reported would violate it, with nothing to say so.
    // Refusing is the honest answer. Teaching the search to honour edge labels is later work and
    // belongs in ruleSuccessors/rulePredecessors, where the mapped neighbour's edge is already in
    // hand - see the TODO at the top of this file.
    if (isEdgeLabelled())
        throw std::runtime_error("VF2 does not support edge labels - see "
                                 "SubgraphIsomorphism::setEdgeLabels()");

    Aux::SignalHandler handler;
    prepareRun();
    VF2Impl(*pattern, *target, patternNodeLabels, targetNodeLabels, semantics, handler,
            [this](const Match &match) { return reportMatch(match); })
        .run();
    finishRun();
}

} // namespace NetworKit
