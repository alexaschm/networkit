#ifndef NETWORKIT_ISOMORPHISM_VF3_HPP_
#define NETWORKIT_ISOMORPHISM_VF3_HPP_

#include <networkit/Globals.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {

/**
 * @ingroup isomorphism
 * Finds every occurrence of a pattern graph inside a target graph, using the VF3 algorithm.
 *
 * @warning **The search is not implemented yet.** @ref run() throws a `std::logic_error` on every
 * input it does not reject earlier. Everything around the search is in place - the constructor,
 * the input validation and the edge-label refusal all behave exactly as the other algorithms' do -
 * so this class is usable as a specification and not as an algorithm. Use @ref RI or @ref VF2
 * until the search lands. The rest of this documentation describes what VF3 *will* do, and is
 * written down now because the shape of the search is what the unwritten pieces have to fit.
 *
 * See @ref SubgraphIsomorphism for what "occurrence" means, how to use the class, and how to
 * choose between the four algorithms in this module.
 *
 * ## How VF3 works
 *
 * VF3 is VF2 with three additions, all aimed at doing less work per step of the search.
 *
 * **Node classes.** Nodes that carry the same label are put into one class. Since a pattern node
 * can only ever map onto a target node of the same class, the search can reason about whole
 * classes instead of individual nodes - for instance, if the pattern needs four nodes of a class
 * the target only has three of, there is no point searching at all. Without labels there is a
 * single class and this machinery does nothing, which is why VF3 gains the most on labelled
 * inputs.
 *
 * **A fixed matching order.** Where VF2 decides which pattern node to map next from the current
 * state, VF3 computes the whole order once, before the search starts. It puts rare classes and
 * well-connected nodes first, because a node that is rare in the target or that has many edges
 * into the already-mapped part gives the fewest candidates and so prunes the most. Each position
 * in the order also records its "parent", the earlier position it is connected to, so candidates
 * can be enumerated from the parent's neighbourhood rather than from the whole graph.
 *
 * **Precomputed feasibility sets.** VF2 recomputes its terminal-set counts at every state. VF3
 * precomputes the equivalent information per class up front, so the pruning rules become table
 * lookups instead of loops over neighbourhoods.
 *
 * The search itself is still the same shape as VF2: grow a partial mapping depth first, test each
 * candidate pair against the pruning rules, backtrack when stuck.
 *
 * ## When to use it
 *
 * VF3 is the one to reach for on large or dense targets, and especially when you have labels. On
 * small unlabelled inputs its precomputation is not repaid and @ref VF2 or @ref RI will be
 * quicker.
 *
 * The search is single-threaded and deterministic: the same input always produces the same
 * matches in the same order. If you want to use several cores, see @ref ParallelRI.
 *
 * ## What it supports
 *
 * Both semantics, directed and undirected graphs, and optional node labels via
 * @ref setNodeLabels().
 *
 * The search polls `Aux::SignalHandler` regularly, so a long enumeration can be stopped with
 * CTRL+C.
 *
 * The implementation is based on
 *
 * Carletti, V., Foggia, P., Saggese, A., & Vento, M. (2018).
 * Challenging the Time Complexity of Exact Subgraph Isomorphism
 * for Huge and Dense Graphs with VF3.
 * IEEE Transactions on Pattern Analysis and Machine Intelligence, 40(4), 804-818.
 */
class VF3 final : public SubgraphIsomorphism {

public:
    /**
     * @param pattern The graph to look for. Must not contain self-loops.
     * @param target The graph to look in. Must agree with @a pattern on directedness.
     * @param semantics Whether matches must be induced. See @ref SubgraphIsomorphism::Semantics.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     */
    VF3(const Graph &pattern, const Graph &target, Semantics semantics = Semantics::INDUCED,
        count maxMatches = 0);

    /**
     * Run the search. Retrieve the results with @ref getMatches(), @ref numberOfMatches() or
     * @ref hasMatch().
     *
     * @warning Not implemented. Throws `std::runtime_error` if @ref setEdgeLabels() was used, and
     * `std::logic_error` otherwise. See the class documentation.
     */
    void run() override;
};

} // namespace NetworKit

#endif // NETWORKIT_ISOMORPHISM_VF3_HPP_
