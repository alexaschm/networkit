#include <algorithm>
#include <stdexcept>
#include <utility>

#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {

namespace {
/// Upper bound on how many matches a worker records between two publications of its local count.
constexpr count MaxPublishInterval = 64;
/// How many publication rounds per worker to aim for, bounding the overshoot past maxMatches.
constexpr count PublishRounds = 8;
} // namespace

SubgraphIsomorphism::SubgraphIsomorphism(const Graph &pattern, const Graph &target,
                                         Semantics semantics, count maxMatches)
    : Algorithm(), pattern(&pattern), target(&target), semantics(semantics), maxMatches(maxMatches),
      publishInterval(0), matchCount(0), storeMatches(true) {
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

void SubgraphIsomorphism::prepareRun(count numWorkers) {
    hasRun = false;

    if (numWorkers == 0)
        numWorkers = 1;

    result.clear();
    result.shrink_to_fit();
    slots.assign(numWorkers, WorkerSlot{});

    matchCount = 0;
    published.store(0, std::memory_order_relaxed);
    capReached.store(false, std::memory_order_relaxed);

    // Publish often enough that the workers overshoot maxMatches only slightly, but rarely
    // enough that a large enumeration performs no atomic operations worth speaking of. A single
    // worker never publishes at all - it knows the global count itself.
    publishInterval =
        maxMatches == 0
            ? 0
            : std::max<count>(1, std::min<count>(MaxPublishInterval,
                                                 maxMatches / (numWorkers * PublishRounds)));

    for (WorkerSlot &slot : slots)
        slot.untilPublish = publishInterval;
}

void SubgraphIsomorphism::finishRun() {
    matchCount = 0;
    for (const WorkerSlot &slot : slots)
        matchCount += slot.found;

    if (!hasCallback() && storeMatches) {
        count stored = 0;
        for (const WorkerSlot &slot : slots)
            stored += slot.buffer.size();

        result.reserve(stored);
        for (WorkerSlot &slot : slots) {
            for (std::vector<node> &match : slot.buffer)
                result.push_back(std::move(match));
            slot.buffer.clear();
            slot.buffer.shrink_to_fit();
        }
    }

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
