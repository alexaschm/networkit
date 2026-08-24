#include <algorithm>
#include <atomic>
#include <deque>
#include <functional>
#include <stdexcept>
#include <utility>
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

using Match = SubgraphIsomorphism::Match;

using IsomorphismDetails::RIImpl;
using IsomorphismDetails::SearchGraph;

/// Upper bound on how many matches a worker records between two publications of its local count.
constexpr count MaxPublishInterval = 64;
/// How many publication rounds per worker to aim for, bounding the overshoot past maxMatches.
constexpr count PublishRounds = 8;

/**
 * The worker pool that drives one RIImpl per thread.
 *
 * ## What each worker owns
 *
 * A private deque of partial mappings, a private RIImpl to expand them with, and a private buffer
 * and counter for the matches it finds. None of those are shared, so the common path - take a
 * state off my own deque, expand it, push the children back, record a match - touches no memory
 * any other thread writes to.
 *
 * The only shared things are: the graph snapshots and the matching order, which are read-only
 * after construction; the victims' deques, touched only when stealing; and three atomics used for
 * stopping. That is the whole synchronization surface.
 *
 * ## Where the results go
 *
 * Reporting a match is the one place a worker has something to say to the outside world, and the
 * naive design - one shared result vector behind a lock - would put that lock in the middle of the
 * search. So the accumulation lives here instead, per worker, and ParallelRI::run() merges the
 * buffers once after the join. This class is the only one in the module that needs any of it:
 * VF2, VF3 and RI are sequential and just call SubgraphIsomorphism::reportMatch().
 *
 * The one part that cannot be private is the user's callback, which the caller passes in as
 * @a deliver. It hands a match to whichever callback form was set and handles the serial form's
 * "never entered twice at once" promise itself, so a worker just calls it and moves on.
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
    /// Hands one match to the user's callback. Thread-safe; returns true if a callback took it,
    /// in which case this class must not store it.
    using Deliver = std::function<bool(index, const Match &)>;

    /**
     * @param patternGraph Shared read-only snapshot of the pattern.
     * @param targetGraph Shared read-only snapshot of the target.
     * @param patternNodeLabels Empty when the search is unlabelled.
     * @param targetNodeLabels Empty when the search is unlabelled.
     * @param ordering Shared read-only matching order, computed once by the caller.
     * @param semantics Whether matches must be induced.
     * @param variant Plain RI or RI-DS.
     * @param handler Shared by every worker. Only its non-throwing isRunning() may be called
     *        from inside the parallel region.
     * @param deliver Where the user's callback is invoked. Called from several threads at once.
     * @param storeMatches Whether the matches have to be kept, rather than only counted.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     * @param numWorkers How many workers to run, which is what the caller told the user to
     *        expect from SubgraphIsomorphism::numberOfWorkers().
     */
    ParallelRIImpl(const SearchGraph &patternGraph, const SearchGraph &targetGraph,
                   const std::vector<index> &patternNodeLabels,
                   const std::vector<index> &targetNodeLabels, const RIImpl::Ordering &ordering,
                   SubgraphIsomorphism::Semantics semantics, RI::Variant variant,
                   Aux::SignalHandler &handler, Deliver deliver, bool storeMatches,
                   count maxMatches, count numWorkers)
        : patternGraph(&patternGraph), targetGraph(&targetGraph),
          patternNodeLabels(&patternNodeLabels), targetNodeLabels(&targetNodeLabels),
          ordering(&ordering), semantics(semantics), variant(variant), handler(&handler),
          deliver(std::move(deliver)), storeMatches(storeMatches), maxMatches(maxMatches),
          numWorkers(numWorkers == 0 ? 1 : numWorkers),
          // Worker holds an atomic, so it is neither copyable nor movable and vector::resize()
          // would not compile. vector(size_type) only needs default construction, hence here.
          workers(this->numWorkers), stopped(false), tokenHolder(0), published(0),
          // Publish often enough that the workers overshoot maxMatches only slightly, but rarely
          // enough that a large enumeration performs no atomic operations worth speaking of.
          publishInterval(
              maxMatches == 0
                  ? 0
                  : std::max<count>(
                        1, std::min<count>(MaxPublishInterval,
                                           maxMatches / (this->numWorkers * PublishRounds)))) {
        for (Worker &worker : workers)
            worker.untilPublish = publishInterval;
    }

    /**
     * Run the parallel search.
     *
     * TODO: implement.
     *  1. Seed the roots with seedRoots(). `workers` is already sized by the constructor.
     *  2. Open an `omp parallel num_threads(numWorkers)` region. Inside it, give each thread its
     *     own RIImpl built from the shared snapshots, the shared ordering and a reporter bound to
     *     that thread - `[this, tid](const Match &m) { return recordMatch(tid, m); }` -
     *     then call workerLoop(tid).
     *  3. That is all - the loop below handles work, stealing and termination. Do not touch the
     *     algorithm object from inside the region; recordMatch() is the only channel out.
     *
     * Reuse: `Aux::getMaxNumberOfThreads()` from networkit/auxiliary/Parallelism.hpp is what the
     * caller sizes `numWorkers` with. Note that there is no Aux::getThreadId() to go with it -
     * the convention throughout NetworKit is a bare omp_get_thread_num() - and that loop indices
     * in an `omp for` have to be NetworKit::omp_index rather than count.
     */
    void run() {
        // TODO: remove once implemented.
        tlx::unused(patternGraph, targetGraph, patternNodeLabels, targetNodeLabels, ordering,
                    semantics, variant, handler, numWorkers, workers, stopped, tokenHolder);
        throw std::logic_error("ParallelRIImpl::run() is not implemented yet");
    }

    /**
     * The workers' buffers concatenated. Empty when nothing was stored.
     *
     * Call once, after @ref run() has returned and every worker has joined.
     */
    std::vector<Match> takeMatches() {
        std::vector<Match> merged;
        if (!storeMatches)
            return merged;

        count total = 0;
        for (const Worker &worker : workers)
            total += worker.buffer.size();

        merged.reserve(total);
        for (Worker &worker : workers) {
            for (Match &match : worker.buffer)
                merged.push_back(std::move(match));
            worker.buffer.clear();
            worker.buffer.shrink_to_fit();
        }
        return merged;
    }

    /// How many matches were reported in total, stored or not. Call after @ref run().
    count matchesFound() const {
        count total = 0;
        for (const Worker &worker : workers)
            total += worker.found;
        return total;
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
        /// Matches this worker found and kept. Merged by takeMatches() after the join.
        std::vector<Match> buffer;
        /// How many matches this worker reported, whether or not they were kept.
        count found = 0;
        /// Counts down to the next publication of `found`; only used when maxMatches != 0.
        count untilPublish = 0;
    };

    /**
     * Record one match found by worker @a tid.
     *
     * This is what each worker's RIImpl reports through. Everything it touches is either that
     * worker's own slot or, when a cap was requested, one atomic that is written only every
     * `publishInterval` matches - so the common case performs no synchronization at all.
     *
     * @return true to keep searching, false once the cap has been reached.
     */
    bool recordMatch(index tid, const Match &match) {
        Worker &worker = workers[tid];
        ++worker.found;

        if (!deliver(tid, match) && storeMatches)
            worker.buffer.push_back(match);

        if (maxMatches == 0)
            return true;

        // A lone worker knows the global count exactly, so it needs no coordination at all.
        if (numWorkers == 1)
            return worker.found < maxMatches;

        // Several workers: tell the others how far we have got, but only every so often, and
        // otherwise just read the flag they set when the cap is reached. The read is a plain load
        // from a cache line nobody writes to, so it is effectively free.
        if (--worker.untilPublish == 0) {
            worker.untilPublish = publishInterval;
            const count total =
                published.fetch_add(publishInterval, std::memory_order_relaxed) + publishInterval;
            if (total >= maxMatches) {
                stopped.store(true, std::memory_order_relaxed);
                return false;
            }
        }

        return !stopped.load(std::memory_order_relaxed);
    }

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

    const std::vector<index> *patternNodeLabels;
    const std::vector<index> *targetNodeLabels;

    /// Computed once by the caller and read by every worker. Never modified here.
    const RIImpl::Ordering *ordering;

    SubgraphIsomorphism::Semantics semantics;
    RI::Variant variant;

    /// Shared by every worker. Inside the region only the non-throwing isRunning() may be
    /// called on it; ParallelRI::run() does the throwing assureRunning() once, after the join.
    Aux::SignalHandler *handler;

    /// Where the user's callback is invoked. Entered by several workers at once.
    Deliver deliver;
    bool storeMatches;
    /// 0 means no limit.
    count maxMatches;
    count numWorkers;

    std::vector<Worker> workers;

    /// Set when the match limit is reached or the search is interrupted, so every worker unwinds.
    std::atomic<bool> stopped;
    /// Who currently holds the termination token.
    std::atomic<index> tokenHolder;

    /// Running total of the matches the workers have published. Only touched when maxMatches != 0
    /// and more than one worker is running.
    std::atomic<count> published;
    /// How many matches a worker records between two publications of its local count.
    count publishInterval;
};

} // namespace

ParallelRI::ParallelRI(const Graph &pattern, const Graph &target, RI::Variant variant,
                       Semantics semantics, count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches), variant(variant) {}

count ParallelRI::numberOfWorkers() const {
    return static_cast<count>(Aux::getMaxNumberOfThreads());
}

void ParallelRI::run() {
    Aux::SignalHandler handler;

    prepareRun();

    // Asked once, here, so that the number of workers the search actually runs and the number
    // numberOfWorkers() promised its caller cannot drift apart.
    const count numWorkers = numberOfWorkers();

    // Built once and shared read-only by every worker.
    const SearchGraph patternGraph(*pattern, /* buildMatrix = */ true, patternEdgeLabels);
    const SearchGraph targetGraph(*target, /* buildMatrix = */ false, targetEdgeLabels);

    // The same narrow refusal RI carries, and for the same reason: edge-label support arrives here
    // through the shared RIImpl, but a collapsed run of parallel arcs with disagreeing labels has
    // no single label left to match against. Raised before any worker starts, so no thread is left
    // to unwind.
    if (patternGraph.collapsedLabelledEdges() || targetGraph.collapsedLabelledEdges())
        throw std::runtime_error("ParallelRI does not support parallel edges whose edge labels "
                                 "disagree - see SubgraphIsomorphism::setEdgeLabels()");
    const RIImpl::Ordering ordering = RIImpl::computeOrdering(
        patternGraph, targetGraph, patternNodeLabels, targetNodeLabels, variant);

    // The lambda is what gets the workers at the user's callback: they are not subclasses and so
    // cannot reach the protected invokeCallback() themselves, but run() can.
    ParallelRIImpl impl(
        patternGraph, targetGraph, patternNodeLabels, targetNodeLabels, ordering, semantics,
        variant, handler,
        [this](index tid, const Match &match) { return invokeCallback(tid, match); },
        storesMatches(), maxMatches, numWorkers);
    impl.run();

    // Only now, with every worker joined, is it safe to let an exception out.
    handler.assureRunning();

    finishRun(impl.takeMatches(), impl.matchesFound());
}

} // namespace NetworKit
