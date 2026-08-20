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

auto printVector = [](const auto &v) {
    std::cout << "[";
    for (index i = 0; i < v.size(); ++i) {
        if (i > 0)
            std::cout << ", ";

        if (v[i] == none)
            std::cout << "none";
        else
            std::cout << v[i];
    }
    std::cout << "]\n";
};

/**
 * Issues:
 *
 * We start with match(0). If nodes as added to terminal sets at depth 0, it just reads as if they
 * are not present. Current fix: Start with match(1)
 *
 * Is removePair supposed to exactly reverse addPair? Upon addition we remove pu, tv from all
 * terminal sets but dont store their depth so we are unable to restore this in removePair. Current
 * fix: Use vector to store depth stamps of pair pu, tv
 *
 * Depth parameter is not used in nextCandidatePair and feasible
 *
 * TODO: More testcases, reference check with brute force algo
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
     * @param patternLabels Empty when the search is unlabelled.
     * @param targetLabels Empty when the search is unlabelled.
     * @param semantics Whether matches must be induced.
     * @param handler Polled so a long search can be stopped with CTRL+C.
     * @param report Where complete mappings are reported.
     */
    VF2Impl(const Graph &pattern, const Graph &target, const std::vector<index> &patternLabels,
            const std::vector<index> &targetLabels, SubgraphIsomorphism::Semantics semantics,
            Aux::SignalHandler &handler, MatchReporter report)
        : patternGraph(pattern, /* buildMatrix = */ true),
          targetGraph(target, /* buildMatrix = */ false), patternLabels(&patternLabels),
          targetLabels(&targetLabels), labelled(!patternLabels.empty()), semantics(semantics),
          handler(&handler), report(std::move(report)), t1in(0), t1out(0), t2in(0), t2out(0) {}

    /**
     * Search for every match and report each one.
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
        /*tlx::unused(patternGraph, targetGraph, patternLabels, targetLabels, labelled, semantics,
                    handler, report, core1, core2, in1, out1, in2, out2, t1in, t1out, t2in, t2out,
                    mapping);
        throw std::logic_error("VF2Impl::run() is not implemented yet");*/

        std::cout << "Entering run()" << std::endl;

        if ((patternGraph.numberOfNodes() > targetGraph.numberOfNodes())
            || (patternGraph.maxInDegree() > targetGraph.maxInDegree())
            || (patternGraph.maxOutDegree() > targetGraph.maxOutDegree())) {
            return;
        }

        core1.assign(patternGraph.upperNodeIdBound(), none);
        core2.assign(targetGraph.upperNodeIdBound(), none);
        in1.assign(patternGraph.upperNodeIdBound(), 0);
        out1.assign(patternGraph.upperNodeIdBound(), 0);
        in2.assign(targetGraph.upperNodeIdBound(), 0);
        out2.assign(targetGraph.upperNodeIdBound(), 0);
        mapping.resize(patternGraph.upperNodeIdBound(), none);

        std::cout << "core1: ";
        printVector(core1);

        std::cout << "core2: ";
        printVector(core2);

        std::cout << "in1: ";
        printVector(in1);

        std::cout << "out1: ";
        printVector(out1);

        std::cout << "in2: ";
        printVector(in2);

        std::cout << "out2: ";
        printVector(out2);

        std::cout << "mapping: ";
        printVector(mapping);

        if (patternGraph.numberOfNodes() == 0) {
            reportMapping();
            return;
        }

        match(1);
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
        /*tlx::unused(depth);
        throw std::logic_error("VF2Impl::match() is not implemented yet");*/

        bool helper;
        std::vector<count> helperVector(4, 0);

        std::cout << "Entering match()" << std::endl;

        if (depth - 1 == patternGraph.numberOfNodes()) {
            std::cout << "Depth reached" << std::endl;
            helper = reportMapping();
            std::cout << "Report mapping returns " << helper << std::endl;
            return helper;
        }

        index cursor = 0;
        node pu = none;
        node tv = none;
        bool continueSearch;

        while (nextCandidatePair(depth, cursor, pu, tv)) {
            std::cout << "Candidate pair: (" << pu << ", " << tv << ")" << std::endl;
            handler->assureRunning();
            if (feasible(pu, tv, depth)) {
                std::cout << "(" << pu << "," << tv << ") is feasible" << std::endl;
                helperVector = addPair(pu, tv, depth);
                std::cout << "Recurse into match with depth " << depth + 1 << std::endl;
                continueSearch = match(depth + 1);
                removePair(pu, tv, depth, helperVector);
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
     * TODO: implement.
     *  1. If both out-terminal sets are non-empty, pair the smallest unmapped pattern node in
     *     the pattern's out-terminal set with each target node in the target's out-terminal set.
     *  2. Otherwise, if both in-terminal sets are non-empty, do the same with those.
     *  3. Otherwise the mapping is not connected to anything, so fall back to the smallest
     *     unmapped pattern node paired with every unmapped target node.
     *  Fixing the pattern node and only varying the target node is what keeps the search tree
     *  from exploding: every candidate at a given depth extends the same pattern node.
     *
     *  Reuse: the terminal sets are stored as depth stamps here rather than as real sets, so
     *  enumerating them means scanning. If that scan shows up in profiles, MaximalCliques.cpp
     *  solves the same problem with pxvector/pxlookup and swapNodeToPos() - one buffer partitioned
     *  by index boundaries, swapped in place, no allocation anywhere in the recursion.
     *
     * @param depth Current search depth.
     * @param cursor In/out: where the previous call stopped, so iteration can resume.
     * @param pu Out: the pattern node to map.
     * @param tv Out: the target node to try for it.
     * @return false when the candidates at this depth are exhausted.
     */
    bool nextCandidatePair(count depth, index &cursor, node &pu, node &tv) const {
        /*tlx::unused(depth, cursor, pu, tv);
        throw std::logic_error("VF2Impl::nextCandidatePair() is not implemented yet");*/

        // FIX depth parameter is never used
        // FIX repetitive structure

        std::cout << "Search for candidates at depth " << depth << std::endl;
        std::cout << "Cursor is " << cursor << std::endl;

        if ((t1out != 0) && (t2out != 0)) {
            std::cout << "Both out-terminal sets are non-empty" << std::endl;
            // Find smallest unmapped pattern node in out1
            // TODO Is it right to search pu repeatedly?
            for (index u = 0; u < core1.size(); ++u) {
                if (patternGraph.hasNode(u) && core1[u] == none && out1[u] != 0) {
                    pu = u;
                    break;
                }
            }

            // Pair with each unmapped target node in out2
            for (index v = cursor; v < core2.size(); ++v) {
                if (targetGraph.hasNode(v) && core2[v] == none && out2[v] != 0) {
                    tv = v;
                    cursor = v + 1;
                    return true;
                }
            }
            std::cout << "Cursor out of bounds." << std::endl;
            return false;
        } else if ((t1in != 0) && (t2in != 0)) {
            std::cout << "Both in-terminal sets are non-empty" << std::endl;
            // Find smallest unmapped pattern node in in1
            // TODO Is it right to search pu repeatedly?
            for (index u = 0; u < core1.size(); ++u) {
                if (patternGraph.hasNode(u) && core1[u] == none && in1[u] != 0) {
                    pu = u;
                    break;
                }
            }

            // Pair with each unmapped target node in in2
            for (index v = cursor; v < core2.size(); ++v) {
                if (targetGraph.hasNode(v) && core2[v] == none && in2[v] != 0) {
                    tv = v;
                    cursor = v + 1;
                    return true;
                }
            }
            std::cout << "Cursor out of bounds." << std::endl;

            return false;
        } else {
            std::cout << "At least one out- and in-terminal set are empty" << std::endl;
            // Find smallest unmapped pattern node in in1
            // TODO Is it right to search pu repeatedly?
            for (index u = 0; u < core1.size(); ++u) {
                if (patternGraph.hasNode(u) && core1[u] == none) {
                    pu = u;
                    break;
                }
            }

            // Pair with each unmapped target node in in2
            for (index v = cursor; v < core2.size(); ++v) {
                if (targetGraph.hasNode(v) && core2[v] == none) {
                    tv = v;
                    cursor = v + 1;
                    return true;
                }
            }
            std::cout << "Cursor out of bounds." << std::endl;

            return false;
        }

        return false;
    }

    /**
     * Whether the pair (@a pu, @a tv) may be added to the mapping.
     *
     * TODO: implement by calling the five rules below in increasing order of cost, returning
     * false at the first that fails. Cheap rejections first is the whole point.
     */
    bool feasible(node pu, node tv, count depth) const {
        /*tlx::unused(pu, tv, depth);
        throw std::logic_error("VF2Impl::feasible() is not implemented yet");*/

        // TODO remove depth parameter is never used but also the five rules dont use it?

        return ruleSuccessors(pu, tv) && rulePredecessors(pu, tv) && ruleTerminalCounts(pu, tv)
               && ruleNewCounts(pu, tv) && ruleLabels(pu, tv);
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
        /*tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleSuccessors() is not implemented yet");*/

        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] != none) {
                if (!targetGraph.hasEdge(tv, core1[u])) {
                    return false;
                }
            }
        }

        // FIX Optional: Could do degree equality check for induced case and reject if not equal
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
     * Consistency rule for in-edges. The mirror image of @ref ruleSuccessors().
     *
     * TODO: implement. For an undirected graph this is the same test as ruleSuccessors(), so it
     * can return true immediately and let that one do the work.
     */
    bool rulePredecessors(node pu, node tv) const {
        /*tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::rulePredecessors() is not implemented yet");*/

        if (!patternGraph.isDirected()) {
            return true;
        }

        for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
            node u = *it;
            if (core1[u] != none) {
                if (!targetGraph.hasEdge(core1[u], tv)) {
                    return false;
                }
            }
        }

        // FIX Optional: Could do degree equality check for induced case and reject if not equal
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
     * One-step look-ahead on the terminal sets.
     *
     * TODO: implement. Count how many neighbours of @a pu lie in each pattern terminal set and
     * how many neighbours of @a tv lie in the corresponding target terminal set. If the pattern
     * count ever exceeds the target count, there will not be enough target nodes left to map
     * them onto, so reject now rather than discovering it several levels deeper.
     */
    bool ruleTerminalCounts(node pu, node tv) const {
        /*tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleTerminalCounts() is not implemented yet");*/

        count in1Neighbors = 0;
        count out1Neighbors = 0;
        count in2Neighbors = 0;
        count out2Neighbors = 0;

        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] == none && out1[u] != 0) {
                out1Neighbors++;
            }
        }

        for (auto it = targetGraph.outBegin(tv); it != targetGraph.outEnd(tv); ++it) {
            node v = *it;
            if (core2[v] == none && out2[v] != 0) {
                out2Neighbors++;
            }
        }

        if (out1Neighbors > out2Neighbors) {
            return false;
        }

        if (patternGraph.isDirected()) {
            for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
                node u = *it;
                if (core1[u] == none && in1[u] != 0) {
                    in1Neighbors++;
                }
            }

            for (auto it = targetGraph.inBegin(tv); it != targetGraph.inEnd(tv); ++it) {
                node v = *it;
                if (core2[v] == none && in2[v] != 0) {
                    in2Neighbors++;
                }
            }

            if (in1Neighbors > in2Neighbors) {
                return false;
            }
        }

        return true;
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
        /*tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleNewCounts() is not implemented yet");*/

        if (semantics == SubgraphIsomorphism::Semantics::MONOMORPHISM) {
            return true;
        }

        count in1Neighbors = 0;
        count out1Neighbors = 0;
        count in2Neighbors = 0;
        count out2Neighbors = 0;

        for (auto it = patternGraph.outBegin(pu); it != patternGraph.outEnd(pu); ++it) {
            node u = *it;
            if (core1[u] == none && in1[u] == 0 && out1[u] == 0) {
                out1Neighbors++;
            }
        }

        for (auto it = targetGraph.outBegin(tv); it != targetGraph.outEnd(tv); ++it) {
            node v = *it;
            if (core2[v] == none && in2[v] == 0 && out2[v] == 0) {
                out2Neighbors++;
            }
        }

        if (out1Neighbors > out2Neighbors) {
            return false;
        }

        if (patternGraph.isDirected()) {
            for (auto it = patternGraph.inBegin(pu); it != patternGraph.inEnd(pu); ++it) {
                node u = *it;
                if (core1[u] == none && in1[u] == 0 && out1[u] == 0) {
                    in1Neighbors++;
                }
            }

            for (auto it = targetGraph.inBegin(tv); it != targetGraph.inEnd(tv); ++it) {
                node v = *it;
                if (core2[v] == none && in2[v] == 0 && out2[v] == 0) {
                    in2Neighbors++;
                }
            }

            if (in1Neighbors > in2Neighbors) {
                return false;
            }
        }

        return true;
    }

    /**
     * Label rule.
     *
     * TODO: implement. Return true immediately when the search is unlabelled. Otherwise the two
     * labels must be equal, except that @ref none on either side is a wildcard that matches
     * anything.
     */
    bool ruleLabels(node pu, node tv) const {
        /*tlx::unused(pu, tv);
        throw std::logic_error("VF2Impl::ruleLabels() is not implemented yet");*/

        if (!labelled) {
            return true;
        }

        if ((*patternLabels)[pu] == (*targetLabels)[tv] || (*patternLabels)[pu] == none
            || (*targetLabels)[tv] == none) {
            return true;
        }

        return false;

        // FIX Do we want to do neighbor label checks here?
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
    std::vector<count> addPair(node pu, node tv, count depth) {
        /*tlx::unused(pu, tv, depth);
        throw std::logic_error("VF2Impl::addPair() is not implemented yet");*/

        std::vector<count> returnVector(4, 0);

        returnVector[0] = in1[pu];
        returnVector[1] = out1[pu];
        returnVector[2] = in2[tv];
        returnVector[3] = out2[tv];

        std::cout << "Add pair (" << pu << "," << tv << ") at depth " << depth << std::endl;

        core1[pu] = tv;
        core2[tv] = pu;

        // Remove pu and tv from any terminal sets that they are part of
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

        // Now that pu and tv are mapped, their unmapped neighbors become part of the terminal sets
        // (if not already)
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

        std::cout << "core1: ";
        printVector(core1);

        std::cout << "core2: ";
        printVector(core2);

        std::cout << "in1: ";
        printVector(in1);

        std::cout << "out1: ";
        printVector(out1);

        std::cout << "in2: ";
        printVector(in2);

        std::cout << "out2: ";
        printVector(out2);

        return returnVector;
    }

    /**
     * Undo @ref addPair() exactly.
     *
     * TODO: implement. Reset core1[pu] and core2[tv] to `none`, and clear every terminal-set
     * entry whose recorded depth equals @a depth, restoring the sizes. Because entries are
     * stamped with the depth that created them, this touches only what this level added.
     */
    void removePair(node pu, node tv, count depth, std::vector<count> helperVector) {
        /*tlx::unused(pu, tv, depth);
        throw std::logic_error("VF2Impl::removePair() is not implemented yet");*/

        // FIX What if depth == 0? Think about wether this is a problem...
        // When we added added (pu,tv) to the mapping, we removed them from the terminal sets and
        // set their stamps to zero, how can we restore the correct depth stamps

        std::cout << "Remove pair (" << pu << "," << tv << ") at depth " << depth << std::endl;

        core1[pu] = none;
        core2[tv] = none;

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

        if (helperVector[0] != 0) {
            in1[pu] = helperVector[0];
            t1in++;
        }
        if (helperVector[1] != 0) {
            out1[pu] = helperVector[1];
            t1out++;
        }
        if (helperVector[2] != 0) {
            in2[tv] = helperVector[2];
            t2in++;
        }
        if (helperVector[3] != 0) {
            out2[tv] = helperVector[3];
            t2out++;
        }

        std::cout << "core1: ";
        printVector(core1);

        std::cout << "core2: ";
        printVector(core2);

        std::cout << "in1: ";
        printVector(in1);

        std::cout << "out1: ";
        printVector(out1);

        std::cout << "in2: ";
        printVector(in2);

        std::cout << "out2: ";
        printVector(out2);
    }

    /**
     * Hand a complete mapping over.
     *
     * TODO: implement. Copy core1 into `mapping` for the pattern nodes that exist, then return
     * report(mapping). Reuse the same buffer every time; the reporter copies it if it needs to.
     */
    bool reportMapping() {

        // FIX Hier kann es sein, dass das leere Mapping reported wird

        // throw std::logic_error("VF2Impl::reportMapping() is not implemented");

        for (index u = 0; u < mapping.size(); ++u) {
            if (patternGraph.hasNode(u)) {
                mapping[u] = core1[u];
            }
        }

        return report(mapping);
    }

    SearchGraph patternGraph;
    SearchGraph targetGraph;

    const std::vector<index> *patternLabels;
    const std::vector<index> *targetLabels;
    bool labelled;

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
    Aux::SignalHandler handler;
    prepareRun();
    VF2Impl(*pattern, *target, patternLabels, targetLabels, semantics, handler,
            [this](const std::vector<node> &match) { return reportMatch(match); })
        .run();
    finishRun();
}

} // namespace NetworKit
