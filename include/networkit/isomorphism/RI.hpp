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
 * The order is built greedily. Repeatedly append whichever unmapped pattern node has the most
 * edges into the nodes already in the order, breaking ties by degree. The reason this works is
 * that every such edge is a constraint: when it is that node's turn, its candidates must be
 * adjacent to already-mapped target nodes, and the more such constraints there are, the fewer
 * candidates survive. Putting the most-constrained node next keeps the search tree narrow from
 * the very top.
 *
 * After that the search is straightforward. Walk the fixed order; at each position take the
 * candidates implied by the already-mapped neighbours, check each one against the pattern edges,
 * and recurse. There are no terminal sets and no candidate bookkeeping to maintain, which makes
 * each individual step very cheap - that is the trade RI makes against @ref VF2 and @ref VF3.
 *
 * @ref Variant::RI_DS adds two things on top: it breaks ordering ties by **candidate domain
 * size** (how many target nodes could possibly host each pattern node, judged from labels and
 * degrees), and it applies **forward checking** - after mapping a node, it verifies that every
 * still-unmapped pattern node retains at least one possible candidate, and backtracks at once if
 * one is left with none. Both cost a little per step and pay off on labelled and sparse inputs,
 * which is why RI_DS is the default.
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
 * Both semantics, directed and undirected graphs, and optional node labels via
 * @ref setLabels().
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
        /// Plain RI: order by how many edges each node has into the already-ordered prefix.
        RI,
        /// RI-DS: additionally break ordering ties by candidate domain size, and prune with
        /// forward checking. Costs a little more per step; usually worth it.
        RI_DS
    };

    /**
     * @param pattern The graph to look for. Must not contain self-loops.
     * @param target The graph to look in. Must agree with @a pattern on directedness.
     * @param variant Plain RI or RI-DS. See @ref Variant.
     * @param semantics Whether matches must be induced. See @ref SubgraphIsomorphism::Semantics.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     */
    RI(const Graph &pattern, const Graph &target, Variant variant = Variant::RI_DS,
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
