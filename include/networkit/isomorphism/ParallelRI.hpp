#ifndef NETWORKIT_ISOMORPHISM_PARALLEL_RI_HPP_
#define NETWORKIT_ISOMORPHISM_PARALLEL_RI_HPP_

#include <networkit/Globals.hpp>
#include <networkit/graph/Graph.hpp>
#include <networkit/isomorphism/RI.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {

/**
 * @ingroup isomorphism
 * The @ref RI search spread across several CPU cores.
 *
 * See @ref SubgraphIsomorphism for what "occurrence" means and how to use the class, and
 * @ref RI for how the underlying algorithm works. This class runs exactly that search and finds
 * exactly the same matches; only the speed and the order differ.
 *
 * ## How the work is split
 *
 * The search explores a tree: each node of the tree is a partial mapping, and its children are
 * the ways of extending that mapping by one more pattern node. That tree is wildly unbalanced -
 * one branch may die immediately while its sibling contains millions of matches - so handing each
 * worker a fixed slice of it up front would leave most of them idle almost at once.
 *
 * Instead each worker keeps a **private double-ended queue** of partial mappings. It pushes and
 * pops at one end, which is its own and needs no synchronization whatsoever. When a worker runs
 * out of work it **steals** from the *other* end of some randomly chosen victim's queue. Taking
 * from the far end is deliberate: that is where the oldest, shallowest, and therefore largest
 * pieces of the search tree sit, so one steal buys a lot of work and steals stay rare.
 *
 * Two refinements make this pay off:
 *
 * - Expanding a single partial mapping is so cheap that publishing every one of them for stealing
 *   would cost more than doing the work. States are therefore **coalesced into larger tasks**
 *   before they become visible to thieves.
 * - Working out that *everybody* has finished is itself a synchronization problem. Rather than a
 *   shared counter that every worker hammers, a **token is passed around a ring** of workers; a
 *   full lap with nobody having found new work means the search is over.
 *
 * Recording matches costs nothing extra: each worker writes into its own padded slot, and the
 * slots are merged once at the end. There is no lock anywhere on the hot path.
 *
 * ## Number of workers
 *
 * Taken from the global NetworKit setting. Change it with `Aux::setNumberOfThreads()`, exactly as
 * for every other parallel algorithm in the library.
 *
 * ## Two things that differ from the sequential algorithms
 *
 * @note **The order of the matches is not reproducible.** The set of matches is always the same,
 * but which worker finds what depends on timing, so @ref getMatches() may come back in a
 * different order from one run to the next. If you need a stable order, sort the result or use
 * @ref RI.
 *
 * @note **The choice of callback decides whether this class can actually use its cores.** A
 * @ref SubgraphIsomorphism::MatchCallback is never invoked concurrently, so every worker has to
 * queue up behind it and the speedup is capped by how long the callback takes. A
 * @ref SubgraphIsomorphism::ParallelMatchCallback is invoked directly by each worker, along with
 * that worker's id, and needs no queueing - but it must be thread-safe. If you are streaming
 * matches out of a parallel search, you almost certainly want the second one.
 *
 * @note With a callback *and* a match limit, a few matches beyond the limit may be delivered
 * before every worker notices that the limit was reached. Stored results and
 * @ref numberOfMatches() are unaffected; they are always trimmed to the limit exactly.
 *
 * The search polls `Aux::SignalHandler` regularly, so a long enumeration can be stopped with
 * CTRL+C.
 *
 * The implementation is based on
 *
 * Kimmig, R., Meyerhenke, H., & Strash, D. (2017).
 * Shared Memory Parallel Subgraph Enumeration.
 * IEEE International Parallel and Distributed Processing Symposium Workshops (IPDPSW).
 */
class ParallelRI final : public SubgraphIsomorphism {

public:
    /**
     * @param pattern The graph to look for. Must not contain self-loops.
     * @param target The graph to look in. Must agree with @a pattern on directedness.
     * @param variant Plain RI or RI-DS. See @ref RI::Variant.
     * @param semantics Whether matches must be induced. See @ref SubgraphIsomorphism::Semantics.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     */
    ParallelRI(const Graph &pattern, const Graph &target, RI::Variant variant = RI::Variant::RI_DS,
               Semantics semantics = Semantics::INDUCED, count maxMatches = 0);

    /**
     * Run the search. Retrieve the results with @ref getMatches(), @ref numberOfMatches() or
     * @ref hasMatch().
     */
    void run() override;

private:
    RI::Variant variant;
};

} // namespace NetworKit

#endif // NETWORKIT_ISOMORPHISM_PARALLEL_RI_HPP_
