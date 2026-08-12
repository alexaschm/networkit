#ifndef NETWORKIT_ISOMORPHISM_SUBGRAPH_ISOMORPHISM_HPP_
#define NETWORKIT_ISOMORPHISM_SUBGRAPH_ISOMORPHISM_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

#include <networkit/Globals.hpp>
#include <networkit/base/Algorithm.hpp>
#include <networkit/graph/Graph.hpp>

namespace NetworKit {

/**
 * @ingroup isomorphism
 * Common base class of the subgraph isomorphism algorithms. Start reading here.
 *
 * ## What these algorithms do
 *
 * You have a small graph, called the **pattern**, and a big graph, called the **target**. You
 * want to find every place in the target where the pattern occurs. Each such place is called a
 * **match**.
 *
 * The classic example is counting triangles. The pattern is a triangle: three nodes 0, 1, 2 with
 * the edges 0-1, 1-2, 2-0. The target is your network. Every match tells you three target nodes
 * that form a triangle.
 *
 * A match is a `std::vector<node>` that is **indexed by pattern node**: `match[u]` is the target
 * node that pattern node @a u was mapped to. So for the triangle pattern above, a match of
 * `{17, 42, 8}` means pattern node 0 sits on target node 17, pattern node 1 on target node 42 and
 * pattern node 2 on target node 8 - and the target really does contain the edges 17-42, 42-8 and
 * 8-17.
 *
 * Two pattern nodes are never mapped to the same target node. In mathematical terms the mapping
 * is injective.
 *
 * ## Induced or not: the one decision you must get right
 *
 * Consider a pattern that is a path of three nodes, `a - b - c`, and a target that is a triangle
 * `x - y - z` (so all three target edges exist).
 *
 * - Under @ref Semantics::MONOMORPHISM the answer is "yes, the path occurs": every pattern edge
 *   (a-b and b-c) has a corresponding target edge. That the target additionally has the edge x-z
 *   does not matter.
 * - Under @ref Semantics::INDUCED the answer is "no": the pattern has **no** edge between a and
 *   c, so the target must have **no** edge between the nodes they map to. But x-z exists, so this
 *   is not an induced occurrence.
 *
 * Rule of thumb: if your question is "does this shape occur somewhere", you want MONOMORPHISM.
 * If your question is "does exactly this shape occur, with nothing extra", you want INDUCED.
 * Path and tree queries are almost always MONOMORPHISM questions. The default is INDUCED because
 * that is the stricter, less surprising answer.
 *
 * ## How to use it
 *
 * Four steps, always in this order:
 *
 * 1. **Construct** with the pattern, the target, the semantics and an optional cap on how many
 *    matches you want.
 * 2. **Configure**, if you need to: @ref setLabels() to restrict matches to nodes that carry the
 *    same label, @ref setCallback() to be handed each match as it is found instead of collecting
 *    them, @ref setStoreMatches() to only count matches without keeping them.
 * 3. **Run** with @ref run().
 * 4. **Query** with @ref getMatches(), @ref numberOfMatches() or @ref hasMatch().
 *
 * @code
 * // How many triangles does G contain?
 * Graph triangle(3);
 * triangle.addEdge(0, 1);
 * triangle.addEdge(1, 2);
 * triangle.addEdge(2, 0);
 *
 * VF2 algo(triangle, G, SubgraphIsomorphism::Semantics::MONOMORPHISM);
 * algo.setStoreMatches(false);   // we only want the count
 * algo.run();
 * count occurrences = algo.numberOfMatches();
 * @endcode
 *
 * Note that this counts every triangle six times, once per way of labelling its corners. That is
 * inherent to subgraph matching, not a quirk of this implementation: divide by the number of
 * automorphisms of your pattern if you want unordered occurrences.
 *
 * ## Which algorithm should I use?
 *
 * | Class          | Use it when                                                              |
 * | -------------- | ------------------------------------------------------------------------ |
 * | @ref VF2       | Small inputs, or as the reference to check the others against.            |
 * | @ref VF3       | Large or dense targets, especially with labels.                           |
 * | @ref RI        | Sparse targets and biological networks; usually the fastest sequentially. |
 * | @ref ParallelRI| Same as RI, but you have cores to spare and the search is long.           |
 *
 * All four return exactly the same set of matches. They differ only in how fast they get there,
 * and @ref ParallelRI additionally does not promise any particular order.
 *
 * ## Things that quietly do not apply
 *
 * - **Edge weights are ignored.** Only the structure of the two graphs is matched.
 * - **Self-loops in the pattern are rejected** by the constructor. Self-loops in the *target* are
 *   fine and are simply never used, because both semantics only constrain pairs of distinct
 *   nodes.
 * - **The argument order is (pattern, target)**, which is the opposite of igraph's
 *   `igraph_subisomorphic_vf2`. Swapping the two compiles fine and answers a different question.
 * - Pattern and target must agree on directedness; mixing them throws from the constructor.
 *
 * ## What this class contributes
 *
 * Everything that does not depend on *how* the search is done: the two graphs, the optional
 * labels, the semantics, the result store, the callbacks, and the cap on the number of matches.
 * A concrete algorithm therefore only has to implement @ref run(). See the protected section for
 * the protocol that @ref run() has to follow.
 */
class SubgraphIsomorphism : public Algorithm {

public:
    /**
     * What counts as a match. See the class documentation for a worked example of the
     * difference; the short version is below.
     */
    enum class Semantics : uint8_t {
        /// Pattern edges must map to target edges **and** pattern non-edges to target non-edges.
        INDUCED,
        /// Only pattern edges must map to target edges. Extra target edges are allowed.
        MONOMORPHISM,
    };

    /**
     * Callback handed one match at a time, instead of collecting them all in memory.
     *
     * The argument is the mapping indexed by pattern node, exactly what @ref getMatches() would
     * have stored.
     *
     * This form is **never called from two threads at once**, not even by @ref ParallelRI, so it
     * needs no locking of its own. The price is that it becomes a bottleneck: a parallel search
     * cannot go faster than this callback runs. If that matters, use @ref ParallelMatchCallback
     * instead.
     *
     * The reference points at an internal buffer that is reused for the next match. Copy the
     * vector if you need to keep it.
     */
    using MatchCallback = std::function<void(const std::vector<node> &)>;

    /**
     * Same as @ref MatchCallback, but it also receives the id of the worker that found the match.
     *
     * This form **may be called from several threads at once** and must therefore be thread-safe.
     * The worker id is in `[0, numberOfWorkers)`, which lets you give every worker its own slot
     * and avoid locking entirely:
     *
     * @code
     * std::vector<Acc> perThread(Aux::getMaxNumberOfThreads());
     * algo.setCallback([&](index tid, const std::vector<node> &match) {
     *     perThread[tid].add(match);   // no lock needed, each tid owns its slot
     * });
     * @endcode
     *
     * The sequential algorithms always pass `tid == 0`, so code written against this form works
     * unchanged with @ref VF2, @ref VF3 and @ref RI.
     *
     * The reference points at an internal buffer that is reused for the next match.
     */
    using ParallelMatchCallback = std::function<void(index, const std::vector<node> &)>;

    /**
     * Handle through which a running search records the matches it finds.
     *
     * You cannot create one - only the algorithm can, via the protected `sink()`. It is public
     * only so that the search implementation classes, which live in the `.cpp` files and are not
     * subclasses, can hold one. If you are just *using* these algorithms, ignore this type.
     *
     * ### Why it exists
     *
     * The obvious design is one shared result vector that every worker appends to under a lock.
     * That would put a lock in the innermost loop of three algorithms that are not even parallel.
     * Instead the algorithm keeps one padded slot per worker, and this handle is bound to exactly
     * one of those slots. Recording a match therefore writes only to memory that no other thread
     * touches: no lock, no shared cache line, and no atomic operation at all unless a cap on the
     * number of matches was requested. The slots are merged once, after the workers have joined.
     *
     * A handle is only valid between `prepareRun()` and `finishRun()`.
     */
    class MatchSink {

    public:
        /**
         * Record one match.
         *
         * @param match The mapping, indexed by pattern node. Copied if it is being stored.
         * @return true to keep searching, false once the requested number of matches is reached.
         *         A search must stop as soon as this returns false.
         */
        bool report(const std::vector<node> &match) {
            WorkerSlot &slot = owner->slots[tid];
            ++slot.found;

            if (owner->parallelCallback)
                owner->parallelCallback(tid, match);
            else if (owner->callback)
                owner->callback(match);
            else if (owner->storeMatches)
                slot.buffer.push_back(match);

            if (owner->maxMatches == 0)
                return true;

            // A lone worker knows the global count exactly, so it needs no coordination at all.
            if (owner->slots.size() == 1)
                return slot.found < owner->maxMatches;

            // Several workers: tell the others how far we have got, but only every so often, and
            // otherwise just read the flag they set when the cap is reached. The read is a plain
            // load from a cache line nobody writes to, so it is effectively free.
            if (--slot.untilPublish == 0) {
                slot.untilPublish = owner->publishInterval;
                const count total =
                    owner->published.fetch_add(owner->publishInterval, std::memory_order_relaxed)
                    + owner->publishInterval;
                if (total >= owner->maxMatches) {
                    owner->capReached.store(true, std::memory_order_relaxed);
                    return false;
                }
            }

            return !owner->capReached.load(std::memory_order_relaxed);
        }

        /**
         * How many more matches this worker may record, or @ref none if there is no cap.
         *
         * Exact for a single worker, an over-estimate when several are running. Use it to size a
         * `reserve()`; do **not** use it to decide when to stop, that is what the return value of
         * @ref report() is for.
         */
        count remaining() const noexcept {
            if (owner->maxMatches == 0)
                return none;

            const count found = owner->slots.size() == 1
                                    ? owner->slots[tid].found
                                    : owner->published.load(std::memory_order_relaxed);

            return found >= owner->maxMatches ? 0 : owner->maxMatches - found;
        }

    private:
        friend class SubgraphIsomorphism;

        MatchSink(SubgraphIsomorphism &owner, index tid) noexcept : owner(&owner), tid(tid) {}

        SubgraphIsomorphism *owner;
        index tid;
    };

    ~SubgraphIsomorphism() override = default;

    /**
     * Run the search. Implemented by each concrete algorithm.
     */
    void run() override = 0;

    /**
     * Only accept matches that map like-labelled nodes onto each other.
     *
     * Both vectors are indexed by node id, so `patternLabels[u]` is the label of pattern node
     * @a u. A pattern node may only be mapped to a target node carrying the same label. The
     * special value @ref none acts as a wildcard and matches any label.
     *
     * Both vectors must have at least `upperNodeIdBound()` entries for their graph, otherwise
     * `std::runtime_error` is thrown. Passing two empty vectors clears the labels and puts the
     * algorithm back into the unlabelled search.
     *
     * Labels usually make the search much faster, because most candidate pairs can be rejected
     * without looking at the graph structure at all. @ref VF3 gets the most out of them.
     *
     * If you already have a `Partition` - from community detection, say - pass its
     * `getVector()`.
     *
     * Call this before @ref run().
     *
     * @param patternLabels Labels of the pattern nodes, indexed by node id.
     * @param targetLabels Labels of the target nodes, indexed by node id.
     */
    void setLabels(const std::vector<index> &patternLabels, const std::vector<index> &targetLabels);

    /**
     * Hand each match to @a callback as it is found, rather than collecting them.
     *
     * Use this when there may be too many matches to hold in memory. @ref getMatches() throws
     * afterwards, but @ref numberOfMatches() and @ref hasMatch() keep working. Replaces any
     * callback set earlier, of either form.
     *
     * This callback is never invoked concurrently. See @ref MatchCallback for what that costs.
     *
     * Call this before @ref run().
     *
     * @param callback Called once per match.
     */
    void setCallback(MatchCallback callback);

    /**
     * Like the other @ref setCallback(), but for a thread-safe callback that also receives the
     * worker id and may be called from several threads at once.
     *
     * This is the form that lets @ref ParallelRI use all its workers. Replaces any callback set
     * earlier, of either form.
     *
     * Call this before @ref run().
     *
     * @param callback Called once per match; must be thread-safe.
     */
    void setCallback(ParallelMatchCallback callback);

    /**
     * Choose whether the matches are kept at all.
     *
     * Pass false when you only want to know *how many* matches there are, as in motif or graphlet
     * counting. Nothing is allocated per match and there is no merge pass at the end, which makes
     * it the cheapest way to ask the question. @ref getMatches() then throws, while
     * @ref numberOfMatches() and @ref hasMatch() keep working.
     *
     * Has no effect if a callback is set, since a callback already means nothing is stored.
     *
     * Call this before @ref run().
     *
     * @param storeMatches Whether to keep matches for @ref getMatches(). Default: true.
     */
    void setStoreMatches(bool storeMatches);

    /**
     * All matches found, each one a vector indexed by pattern node.
     *
     * Throws `std::runtime_error` if the matches were never stored, which happens when a callback
     * was set or @ref setStoreMatches(false) was called, and if @ref run() has not been called
     * yet.
     *
     * @return the matches, each a vector of target node ids.
     */
    const std::vector<std::vector<node>> &getMatches() const;

    /**
     * How many matches were found. Works regardless of whether they were stored.
     *
     * If a cap was requested, this is capped too and is therefore not the total number of matches
     * in the graph. The cap is exact, with one exception: a parallel search that streams to a
     * callback may have handed over a small bounded number of extra matches before every worker
     * noticed that the cap had been reached.
     *
     * @return the number of matches reported.
     */
    count numberOfMatches() const;

    /**
     * Whether at least one match was found.
     *
     * If all you want is a yes/no answer, construct the algorithm with `maxMatches = 1` so the
     * search stops at the first match instead of enumerating all of them.
     *
     * @return true if there was at least one match.
     */
    bool hasMatch() const;

protected:
    // ---------------------------------------------------------------------------------------
    // Everything below is for people implementing a new algorithm in this module.
    //
    // The protocol a run() implementation must follow:
    //
    //   1. Call prepareRun(numWorkers). This throws away the results of any earlier run and
    //      allocates one slot per worker. Sequential algorithms pass nothing and get one slot.
    //   2. Get a MatchSink for each worker with sink(tid). Do this *before* starting any
    //      threads, and give each worker exactly one handle.
    //   3. Search. Whenever a complete mapping is found, call MatchSink::report() on that
    //      worker's own handle. Stop that worker as soon as report() returns false.
    //      Never touch the algorithm object itself from a worker thread.
    //   4. After the workers have joined, call finishRun(). It merges the slots, applies the
    //      cap, and marks the algorithm as finished.
    //
    // Use isLabelled() to find out whether labels are in play, and hasSerialCallback() to find
    // out whether the user's callback must not be called concurrently - if it must not, wrap the
    // report() call in a critical section.
    //
    // The search itself belongs in a separate implementation class in the .cpp file, so that the
    // public header never grows a member. See VF2.cpp for the pattern.
    // ---------------------------------------------------------------------------------------

    /**
     * @param pattern The pattern graph to look for.
     * @param target The target graph to look in.
     * @param semantics Whether matches must be induced.
     * @param maxMatches Stop after this many matches; 0 means no limit.
     */
    SubgraphIsomorphism(const Graph &pattern, const Graph &target, Semantics semantics,
                        count maxMatches);

    /**
     * @return true if @ref setLabels() was used, so the search has to compare labels.
     */
    bool isLabelled() const noexcept { return !patternLabels.empty(); }

    /**
     * @return true if a callback of either form was set.
     */
    bool hasCallback() const noexcept {
        return static_cast<bool>(callback) || static_cast<bool>(parallelCallback);
    }

    /**
     * @return true if the user's callback must not be called from two threads at once, in which
     * case a parallel search has to serialize its @ref MatchSink::report() calls.
     */
    bool hasSerialCallback() const noexcept { return static_cast<bool>(callback); }

    /**
     * Step 1 of the protocol: forget the previous run and allocate the per-worker slots.
     *
     * @param numWorkers How many workers will report concurrently. 0 is treated as 1.
     */
    void prepareRun(count numWorkers = 1);

    /**
     * Step 2 of the protocol: the handle worker @a tid records its matches through.
     *
     * @param tid Worker id in `[0, numWorkers)`, matching what was passed to @ref prepareRun().
     */
    MatchSink sink(index tid = 0) noexcept { return MatchSink(*this, tid); }

    /**
     * Step 4 of the protocol: merge the worker slots, apply the cap, mark the run as finished.
     */
    void finishRun();

    /// The graph we are looking for. Never null, never modified.
    const Graph *pattern;
    /// The graph we are looking in. Never null, never modified.
    const Graph *target;

    /// Empty unless setLabels() was used. Indexed by node id.
    std::vector<index> patternLabels;
    /// Empty unless setLabels() was used. Indexed by node id.
    std::vector<index> targetLabels;

    Semantics semantics;
    /// 0 means no limit.
    count maxMatches;

private:
    /**
     * Reject inputs no search can handle, throwing `std::runtime_error`.
     *
     * Called from the constructor, so bad input fails immediately rather than deep inside
     * @ref run(). Rejects a directedness mismatch and self-loops in the pattern.
     *
     * Deliberately not virtual: it runs while the derived part of the object is still
     * uninitialized, so an override would never be reached.
     */
    void validateInput() const;

    /// One per worker. Padded so two workers never write to the same cache line.
    struct alignas(64) WorkerSlot {
        std::vector<std::vector<node>> buffer;
        count found = 0;
        /// Counts down to the next publication of `found`; only used when maxMatches != 0.
        count untilPublish = 0;
    };

    std::vector<WorkerSlot> slots;
    std::vector<std::vector<node>> result;

    MatchCallback callback;
    ParallelMatchCallback parallelCallback;

    /// Only touched when maxMatches != 0 and more than one worker is reporting.
    std::atomic<count> published{0};
    std::atomic<bool> capReached{false};
    count publishInterval;

    count matchCount;
    bool storeMatches;
};

} // namespace NetworKit

#endif // NETWORKIT_ISOMORPHISM_SUBGRAPH_ISOMORPHISM_HPP_
