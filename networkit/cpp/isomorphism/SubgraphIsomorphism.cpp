#include <mutex>
#include <stdexcept>
#include <utility>

#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

/*
 * ## What this module reuses from the rest of NetworKit
 *
 * Nothing in NetworKit does subgraph matching, so the search algorithms themselves are written
 * from scratch. The pieces around them are not. Before filling in any of the TODOs in this module,
 * check this list - each entry either drops in unchanged or saves writing a known-fiddly helper.
 *
 * Used as-is, no adaptation:
 *
 * - `Aux::SignalHandler` (networkit/auxiliary/SignalHandling.hpp) is what every checkSignal() stub
 *   in this module should be. Hold one as a member and call assureRunning() on it; there is no
 *   need to write any interruption machinery. Construction is cheap and nesting is a no-op, which
 *   is why MaximalCliques declares one in run() and another inside its recursive tomita().
 * - `Aux::Random::getURNG()` (networkit/auxiliary/Random.hpp) is already thread-local, so it is
 *   safe inside a parallel region without any seeding work. ParallelRI can use it for victim
 *   selection instead of carrying a per-worker seed.
 * - `Aux::SparseVector<T>` (networkit/auxiliary/SparseVector.hpp) clears only the entries that
 *   were actually touched, not the whole array. That is the right structure for scratch marks that
 *   get dirtied and then wiped in bulk, such as RI-DS domain bookkeeping. It is *not* the right
 *   structure for core1/core2, which are undone one entry at a time on backtrack and are already
 *   O(1) as plain vectors.
 * - `CoreDecomposition::getNodeOrder()` (networkit/centrality/CoreDecomposition.hpp) hands back a
 *   degeneracy ordering in three lines. Optional input to the target-side tie-break in
 *   RIImpl::computeOrdering; it does not replace the pattern ordering, which is the algorithm.
 * - `tlx::div_ceil` (tlx/math/div_ceil.hpp) says what (z + 63) / 64 means in SearchGraph.
 *
 * Same shape exists, but the code has to be written here:
 *
 * - There is no CSR graph class anywhere in NetworKit, so SearchGraph::buildCSR has to be written.
 *   The count/prefix-sum/scatter idiom to follow is in ParallelPartitionCoarsening.cpp, and
 *   MaximalCliques.cpp keeps its own CSR under the same firstOut/head names this module uses.
 * - Candidate sets: MaximalCliques.cpp partitions one buffer with pxvector/pxlookup and
 *   swapNodeToPos(), which keeps a backtracking search free of allocation. Worth copying the
 *   technique for the terminal sets in VF2 and the domains in RI-DS.
 *
 */

namespace NetworKit {

SubgraphIsomorphism::SubgraphIsomorphism(const Graph &pattern, const Graph &target,
                                         Semantics semantics, count maxMatches)
    : Algorithm(), pattern(&pattern), target(&target), semantics(semantics), maxMatches(maxMatches),
      matchCount(0), storeMatches(true) {
    validateInput();
}

void SubgraphIsomorphism::validateInput() const {
    if (pattern->isDirected() != target->isDirected())
        throw std::runtime_error(
            "Pattern and target graph must either both be directed or both be undirected");

    // Self-loops of the target are harmless: both matching semantics only constrain pairs of
    // distinct nodes, so a target self-loop is never used by a loop-free pattern.
    if (pattern->numberOfSelfLoops() > 0)
        throw std::runtime_error("Subgraph isomorphism is undefined for patterns with self-loops");
}

void SubgraphIsomorphism::setLabels(const std::vector<index> &patternLabels,
                                    const std::vector<index> &targetLabels) {
    if (patternLabels.empty() && targetLabels.empty()) {
        this->patternLabels.clear();
        this->targetLabels.clear();
        return;
    }

    if (patternLabels.size() < pattern->upperNodeIdBound())
        throw std::runtime_error("Pattern label vector is shorter than the pattern's "
                                 "upperNodeIdBound()");

    if (targetLabels.size() < target->upperNodeIdBound())
        throw std::runtime_error("Target label vector is shorter than the target's "
                                 "upperNodeIdBound()");

    this->patternLabels = patternLabels;
    this->targetLabels = targetLabels;
}

void SubgraphIsomorphism::setCallback(MatchCallback callback) {
    this->callback = std::move(callback);
    parallelCallback = nullptr;
}

void SubgraphIsomorphism::setCallback(ParallelMatchCallback callback) {
    parallelCallback = std::move(callback);
    this->callback = nullptr;
}

void SubgraphIsomorphism::setStoreMatches(bool storeMatches) {
    this->storeMatches = storeMatches;
}

void SubgraphIsomorphism::prepareRun() {
    hasRun = false;

    result.clear();
    result.shrink_to_fit();

    matchCount = 0;
}

bool SubgraphIsomorphism::reportMatch(const std::vector<node> &match) {
    ++matchCount;

    // Single-threaded, so the serial callback needs no lock and the parallel one is simply told
    // it is worker 0. Both forms therefore work unchanged with a sequential algorithm.
    if (parallelCallback)
        parallelCallback(0, match);
    else if (callback)
        callback(match);
    else if (storeMatches)
        result.push_back(match);

    return maxMatches == 0 || matchCount < maxMatches;
}

bool SubgraphIsomorphism::invokeCallback(index tid, const std::vector<node> &match) {
    if (parallelCallback) {
        parallelCallback(tid, match);
        return true;
    }

    if (callback) {
        // A MatchCallback promises never to be entered twice at once. Honouring it here rather
        // than in each parallel algorithm is what makes the promise true: the reporting call sits
        // deep inside a search's recursion, far from the code that knows how many threads are
        // running. Only a parallel search reaches this, so no sequential one pays for the lock.
        const std::lock_guard<std::mutex> guard(reportMutex);
        callback(match);
        return true;
    }

    return false;
}

void SubgraphIsomorphism::finishRun() {
    // A sequential search stops the moment reportMatch() returns false, so the cap already holds
    // exactly. Nothing to trim.
    hasRun = true;
}

void SubgraphIsomorphism::finishRun(std::vector<std::vector<node>> &&matches, count found) {
    matchCount = found;
    if (!hasCallback() && storeMatches)
        result = std::move(matches);

    // Workers may overshoot the cap slightly before they all observe it. Nothing was handed to
    // the caller yet unless a callback is in use, so everything but the callback stays exact.
    if (!hasCallback() && maxMatches != 0 && matchCount > maxMatches) {
        matchCount = maxMatches;
        if (result.size() > maxMatches)
            result.resize(maxMatches);
    }

    hasRun = true;
}

const std::vector<std::vector<node>> &SubgraphIsomorphism::getMatches() const {
    if (hasCallback())
        throw std::runtime_error(
            "SubgraphIsomorphism used with a callback does not store the matches");
    if (!storeMatches)
        throw std::runtime_error(
            "SubgraphIsomorphism used with setStoreMatches(false) does not store the matches");
    assureFinished();
    return result;
}

count SubgraphIsomorphism::numberOfMatches() const {
    assureFinished();
    return matchCount;
}

bool SubgraphIsomorphism::hasMatch() const {
    assureFinished();
    return matchCount > 0;
}

} // namespace NetworKit
