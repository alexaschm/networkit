#include <atomic>
#include <deque>
#include <stdexcept>
#include <vector>

#include <tlx/unused.hpp>

#include <networkit/Globals.hpp>
#include <networkit/auxiliary/Parallelism.hpp>
#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/isomorphism/ParallelRI.hpp>

#include "RIImpl.hpp"
#include "SearchGraph.hpp"

namespace NetworKit {

namespace {

using IsomorphismDetails::RIImpl;
using IsomorphismDetails::SearchGraph;

/**
 * The worker pool that drives one RIImpl per thread.
 *
 * ## What each worker owns
 *
 * A private deque of partial mappings, a private RIImpl to expand them with, and a private
 * MatchSink to report through. None of those are shared, so the common path - take a state off
 * my own deque, expand it, push the children back - touches no memory any other thread writes to.
 *
 * The only shared things are: the graph snapshots and the matching order, which are read-only
 * after construction; the victims' deques, touched only when stealing; and two atomics used for
 * stopping. That is the whole synchronization surface.
 *
 * ## The two ends of the deque
 *
 * The owner works at one end, thieves take from the other. The owner's end holds the newest and
 * therefore deepest states, which are small pieces of work with good cache locality. The thief's
 * end holds the oldest and shallowest, which are the biggest pieces. So the owner gets cheap
 * fast steps and a thief gets one big chunk per steal, which is exactly the split that keeps
 * stealing rare.
 */
class ParallelRIImpl {

public:
    /**
     * @param patternGraph Shared read-only snapshot of the pattern.
     * @param targetGraph Shared read-only snapshot of the target.
     * @param patternLabels Empty when the search is unlabelled.
     * @param targetLabels Empty when the search is unlabelled.
     * @param ordering Shared read-only matching order, computed once by the caller.
     * @param semantics Whether matches must be induced.
     * @param variant Plain RI or RI-DS.
     * @param handler Shared by every worker. Only its non-throwing isRunning() may be called
     *        from inside the parallel region.
     * @param serializeCallback True when the user's callback must not run concurrently, in which
     *        case reporting has to happen inside a critical section.
     * @param sinks One per worker, in worker-id order. Never shared between threads.
     */
    ParallelRIImpl(const SearchGraph &patternGraph, const SearchGraph &targetGraph,
                   const std::vector<index> &patternLabels, const std::vector<index> &targetLabels,
                   const RIImpl::Ordering &ordering, SubgraphIsomorphism::Semantics semantics,
                   RI::Variant variant, Aux::SignalHandler &handler, bool serializeCallback,
                   std::vector<SubgraphIsomorphism::MatchSink> sinks)
        : patternGraph(&patternGraph), targetGraph(&targetGraph), patternLabels(&patternLabels),
          targetLabels(&targetLabels), ordering(&ordering), semantics(semantics), variant(variant),
          handler(&handler), serializeCallback(serializeCallback), sinks(std::move(sinks)),
          numWorkers(this->sinks.size()), stopped(false), tokenHolder(0) {}

    /**
     * Run the parallel search.
     *
     * TODO: implement.
     *  1. Size `workers` to numWorkers and seed the roots with seedRoots().
     *  2. Open an `omp parallel num_threads(numWorkers)` region. Inside it, give each thread its
     *     own RIImpl built from the shared snapshots, the shared ordering and sinks[tid], then
     *     call workerLoop(tid).
     *  3. That is all - the loop below handles work, stealing and termination. Do not touch the
     *     algorithm object from inside the region; sinks[tid] is the only channel out.
     *
     * Reuse: `Aux::getMaxNumberOfThreads()` from networkit/auxiliary/Parallelism.hpp is already
     * used by the caller to size `sinks`. Note that there is no Aux::getThreadId() to go with it -
     * the convention throughout NetworKit is a bare omp_get_thread_num() - and that loop indices
     * in an `omp for` have to be NetworKit::omp_index rather than count.
     */
    void run() {
        // TODO: remove once implemented.
        tlx::unused(patternGraph, targetGraph, patternLabels, targetLabels, ordering, semantics,
                    variant, handler, serializeCallback, sinks, numWorkers, workers, stopped,
                    tokenHolder);
        throw std::logic_error("ParallelRIImpl::run() is not implemented yet");
    }

private:
    /// One worker's private state. Padded so two workers never share a cache line.
    struct alignas(64) Worker {
        /// Partial mappings still to explore. Owner works the back, thieves take the front.
        std::deque<RIImpl::State> states;
        /// Set while the owner is touching its deque, so a thief knows to try somebody else.
        std::atomic<bool> busy{false};
        /// Seed for pickVictim(). Per worker, so choosing a victim needs no shared RNG.
        uint64_t rngState = 0;
        /// How many states have been expanded since the last batch was published for stealing.
        count sinceLastPublish = 0;
    };

    /**
     * Create the initial states and spread them over the workers.
     *
     * TODO: implement. The root of the search tree is the empty mapping, and its children are the
     * ways of mapping the first pattern node in the order onto a target node. Expanding the root
     * once here and dealing the resulting states round-robin over the deques gives every worker
     * something to do immediately, instead of having them all steal from worker 0 at startup.
     */
    void seedRoots() { throw std::logic_error("ParallelRIImpl::seedRoots() is not implemented"); }

    /**
     * What one worker does for the whole search.
     *
     * TODO: implement.
     *  1. Loop: pop a state from my own deque with popLocal(). If there is none, try trySteal().
     *     If that fails too, join the termination protocol via passToken() and check quiescent().
     *  2. Expand the state with my own RIImpl and push the children back with pushLocal().
     *  3. If expand() returns false the match limit was reached: set `stopped` and return, which
     *     unwinds every other worker as well.
     *  4. Call `handler->isRunning()` and on false set `stopped` and return, so every worker
     *     winds down.
     *
     * Step 4 must use isRunning() and never assureRunning(): an exception escaping an OpenMP
     * structured block is undefined behaviour. ParallelRI::run() calls assureRunning() once,
     * after the region has joined, which is where the InterruptException actually comes from.
     * This is the pattern Betweenness.cpp uses.
     */
    void workerLoop(index tid) {
        tlx::unused(tid);
        throw std::logic_error("ParallelRIImpl::workerLoop() is not implemented yet");
    }

    /**
     * Take a state off worker @a tid's own end of its deque.
     *
     * TODO: implement. Pop from the back. This is the hot path, so it must stay free of atomics
     * in the common case; only the handful of states near the stealing end need any care, and
     * that is what the `busy` flag is for.
     *
     * @return false if the deque is empty.
     */
    bool popLocal(index tid, RIImpl::State &out) {
        tlx::unused(tid, out);
        throw std::logic_error("ParallelRIImpl::popLocal() is not implemented yet");
    }

    /**
     * Push a freshly produced state onto worker @a tid's own end.
     *
     * TODO: implement. Push to the back, then bump sinceLastPublish and call coalesceIntoTask()
     * when it crosses the threshold.
     */
    void pushLocal(index tid, RIImpl::State &&state) {
        tlx::unused(tid, state);
        throw std::logic_error("ParallelRIImpl::pushLocal() is not implemented yet");
    }

    /**
     * Try to take work from somebody else.
     *
     * TODO: implement. Pick a victim with pickVictim(), take from the *front* of its deque -
     * the opposite end from where the owner works, so the two rarely collide - and give up after
     * a few unsuccessful attempts rather than spinning. Returning false is normal and simply
     * means "nothing to steal right now"; the caller then goes into the termination protocol.
     *
     * @return false if nothing could be stolen.
     */
    bool trySteal(index thief, RIImpl::State &out) {
        tlx::unused(thief, out);
        throw std::logic_error("ParallelRIImpl::trySteal() is not implemented yet");
    }

    /**
     * Choose whom to steal from.
     *
     * TODO: implement. Draw a uniformly random worker other than @a thief, using that worker's
     * own rngState so no two threads share an RNG. Random choice is what keeps the load balanced
     * without anybody having to track who is busy.
     *
     * Reuse: `Aux::Random::getURNG()` is already thread-local, so it gives a private generator per
     * worker with no seeding work and no extra field. Using it here would make Worker::rngState
     * redundant. The documented idiom is to bind the reference once outside the loop and drive a
     * std::uniform_int_distribution with it, rather than calling Aux::Random::integer() per draw.
     */
    index pickVictim(index thief) {
        tlx::unused(thief);
        throw std::logic_error("ParallelRIImpl::pickVictim() is not implemented yet");
    }

    /**
     * Make a batch of this worker's states available for stealing.
     *
     * TODO: implement. Expanding one state is far too cheap to justify the synchronization a
     * steal costs, so states become visible in batches rather than one at a time. Publish the
     * oldest batch, reset sinceLastPublish, and leave the newest states private to the owner.
     */
    void coalesceIntoTask(index tid) {
        tlx::unused(tid);
        throw std::logic_error("ParallelRIImpl::coalesceIntoTask() is not implemented yet");
    }

    /**
     * Move the termination token one step around the ring.
     *
     * TODO: implement. An idle worker holding the token passes it to (tid + 1) % numWorkers. Any
     * worker that finds new work in the meantime marks the token dirty, which cancels the lap.
     * The ring exists so that detecting "everyone is done" does not need a counter that every
     * worker writes to - that counter would be contended precisely when the search is winding
     * down and workers are idling fastest.
     */
    void passToken(index tid) {
        tlx::unused(tid);
        throw std::logic_error("ParallelRIImpl::passToken() is not implemented yet");
    }

    /**
     * Whether the search is over.
     *
     * TODO: implement. True once the token has made a full lap of the ring without any worker
     * having found new work, or once `stopped` is set because the match limit was hit.
     */
    bool quiescent() const {
        throw std::logic_error("ParallelRIImpl::quiescent() is not implemented yet");
    }

    const SearchGraph *patternGraph;
    const SearchGraph *targetGraph;

    const std::vector<index> *patternLabels;
    const std::vector<index> *targetLabels;

    /// Computed once by the caller and read by every worker. Never modified here.
    const RIImpl::Ordering *ordering;

    SubgraphIsomorphism::Semantics semantics;
    RI::Variant variant;

    /// Shared by every worker. Inside the region only the non-throwing isRunning() may be
    /// called on it; ParallelRI::run() does the throwing assureRunning() once, after the join.
    Aux::SignalHandler *handler;

    /// True when the user's callback must not be invoked concurrently.
    bool serializeCallback;

    /// One per worker, indexed by worker id.
    std::vector<SubgraphIsomorphism::MatchSink> sinks;
    count numWorkers;

    std::vector<Worker> workers;

    /// Set when the match limit is reached or the search is interrupted, so every worker unwinds.
    std::atomic<bool> stopped;
    /// Who currently holds the termination token.
    std::atomic<index> tokenHolder;
};

} // namespace

ParallelRI::ParallelRI(const Graph &pattern, const Graph &target, RI::Variant variant,
                       Semantics semantics, count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches), variant(variant) {}

void ParallelRI::run() {
    Aux::SignalHandler handler;

    const count numWorkers = static_cast<count>(Aux::getMaxNumberOfThreads());

    prepareRun(numWorkers);

    // Built once and shared read-only by every worker.
    const SearchGraph patternGraph(*pattern, /* buildMatrix = */ true);
    const SearchGraph targetGraph(*target, /* buildMatrix = */ false);
    const RIImpl::Ordering ordering =
        RIImpl::computeOrdering(patternGraph, targetGraph, patternLabels, targetLabels, variant);

    // Every handle is taken here, on one thread, before any worker starts.
    std::vector<MatchSink> sinks;
    sinks.reserve(numWorkers);
    for (index tid = 0; tid < numWorkers; ++tid)
        sinks.push_back(sink(tid));

    ParallelRIImpl(patternGraph, targetGraph, patternLabels, targetLabels, ordering, semantics,
                   variant, handler, hasSerialCallback(), std::move(sinks))
        .run();

    // Only now, with every worker joined, is it safe to let an exception out.
    handler.assureRunning();

    finishRun();
}

} // namespace NetworKit
