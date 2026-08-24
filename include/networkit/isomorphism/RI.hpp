#ifndef NETWORKIT_ISOMORPHISM_RI_HPP_
#define NETWORKIT_ISOMORPHISM_RI_HPP_

#include <cstdint>

#include <networkit/Globals.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {

/**
 * @ingroup isomorphism
 * Finds every occurrence of a pattern graph inside a target graph, using the RI algorithm.
 *
 * See @ref SubgraphIsomorphism for what "occurrence" means, how to use the class, and how to
 * choose between the four algorithms in this module.
 *
 * ## How RI works
 *
 * RI is built on one observation: almost all of the benefit comes from the **order** in which you
 * map the pattern nodes, and very little from clever bookkeeping during the search. So it spends
 * its effort computing a good order once, up front, and then runs a deliberately plain
 * backtracking search.
 *
 * The order is built greedily, starting from the highest-degree node, and each further node is
 * chosen by three numbers compared in turn. The first is how many **edges** the candidate has into
 * the nodes already ordered; every such edge is a constraint, because when the candidate's turn
 * comes its images must be adjacent to already-mapped target nodes, so the more of them there are
 * the fewer candidates survive. The second is a look-ahead at constraints that are not in force
 * yet: how many already-ordered nodes the candidate can reach in two hops **through a node that is
 * not ordered yet**. The third is how many of the candidate's neighbours lie outside the order and
 * touch nothing in it, which is a rough measure of how much new territory choosing it opens up.
 * Ties on all three go to the smallest node id, so the order is reproducible. Putting the
 * most-constrained node next keeps the search tree narrow from the very top.
 *
 * After that the search is straightforward. Walk the fixed order; at each position take the
 * candidates implied by the already-mapped neighbours, check each one against the pattern edges,
 * and recurse. There are no terminal sets and no candidate bookkeeping to maintain, which makes
 * each individual step very cheap - that is the trade RI makes against @ref VF2 and @ref VF3.
 *
 * @ref Variant::RI_DS adds **domains**: before the search starts, it works out for each pattern
 * node which target nodes could host it at all - judged from labels and degrees, then narrowed
 * once by checking that each of the node's pattern neighbours still has somewhere to go across a
 * real target edge - and then uses those sets to shrink candidate lists during the search. The
 * order is the same as plain RI's; there is no domain-size tie-break. Nor is there any forward
 * checking or other per-step inference, because the paper measured that the extra pruning does not
 * pay for what it costs.
 *
 * Only part of what a domain knows is new, though. Ruling a target node out on its degree or its
 * label is something the search already does for each candidate it looks at, at the cost of a
 * couple of integer compares; only the narrowing step finds anything those per-candidate rules
 * miss. So a domain is consulted for a candidate list drawn from a neighbourhood only when the
 * narrowing removed most of that domain, and is otherwise left aside as the more expensive way to
 * reach the same answer. What a domain always does is replace the scan over every node in the
 * target that a pattern node with no already-mapped neighbour would otherwise need - which is why
 * it helps most on a disconnected pattern.
 *
 * Even so, building the domains costs a pass over the target per pattern node, and on the graphs
 * this has been measured against that pass is rarely repaid: on a long enumeration the two
 * variants land within about a percent of each other, and plain RI is ahead whenever the search is
 * short enough for the setup to show. @ref Variant::RI is therefore the default. Reach for RI_DS
 * when the pattern is disconnected, or when the labels are selective enough that the narrowing
 * step really bites.
 *
 * ## When to use it
 *
 * RI is usually the fastest of the three sequential algorithms here, particularly on sparse
 * graphs and on biological networks, which is what it was designed for. If the search is long
 * enough to be worth spreading over several cores, use @ref ParallelRI, which runs this same
 * search in parallel and returns the same matches.
 *
 * ## What it supports
 *
 * Both semantics, directed and undirected graphs, optional node labels via @ref setNodeLabels(),
 * and optional edge labels via @ref setEdgeLabels(). Edge labels are cheap here: the parent edge's
 * label filters the candidate slice as it is walked, which costs one comparison per candidate.
 *
 * The one input it refuses is **parallel edges whose labels disagree**. The search works off a
 * snapshot in which parallel edges are collapsed to one, and a single arc cannot stand for two
 * different labels, so `run()` throws rather than answer a question that was not asked. Parallel
 * edges carrying the same label collapse losslessly and are fine.
 *
 * The search polls `Aux::SignalHandler` regularly, so a long enumeration can be stopped with
 * CTRL+C.
 *
 * The implementation is based on
 *
 * Bonnici, V., Giugno, R., Pulvirenti, A., Shasha, D., & Ferro, A. (2013).
 * A subgraph isomorphism algorithm and its application to biochemical data.
 * BMC Bioinformatics, 14(Suppl 7), S13.
 *
 * and, for the RI-DS variant and the sequential baseline of the parallel version,
 *
 * Kimmig, R., Meyerhenke, H., & Strash, D. (2017).
 * Shared Memory Parallel Subgraph Enumeration.
 * IEEE International Parallel and Distributed Processing Symposium Workshops (IPDPSW).
 */
class RI final : public SubgraphIsomorphism {

public:
    /**
     * Which flavour of the RI search to run.
     */
    enum class Variant : uint8_t {
        /// Plain RI: order by the three-level score described above, and search with no
        /// bookkeeping at all. The default.
        RI,
        /// RI-DS: the same order, plus per-node candidate domains computed once before the search.
        /// Costs a pass over the target per pattern node, which pays for itself mainly on
        /// disconnected patterns and on selectively labelled inputs.
        RI_DS
    };

    /**
     * @param pattern The graph to look for. Must not contain self-loops.
     * @param target The graph to look in. Must agree with @a pattern on directedness.
     * @param variant Plain RI or RI-DS. See @ref Variant.
     * @param semantics Whether matches must be induced. See @ref SubgraphIsomorphism::Semantics.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     */
    RI(const Graph &pattern, const Graph &target, Variant variant = Variant::RI,
       Semantics semantics = Semantics::INDUCED, count maxMatches = 0);

    /**
     * Run the search. Retrieve the results with @ref getMatches(), @ref numberOfMatches() or
     * @ref hasMatch().
     */
    void run() override;

private:
    Variant variant;
};

} // namespace NetworKit

#endif // NETWORKIT_ISOMORPHISM_RI_HPP_
