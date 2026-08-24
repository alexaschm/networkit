#include <stdexcept>

#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/isomorphism/RI.hpp>

#include "RIImpl.hpp"
#include "SearchGraph.hpp"

namespace NetworKit {

RI::RI(const Graph &pattern, const Graph &target, Variant variant, Semantics semantics,
       count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches), variant(variant) {}

void RI::run() {
    using IsomorphismDetails::RIImpl;
    using IsomorphismDetails::SearchGraph;

    Aux::SignalHandler handler;
    prepareRun();

    // The pattern is small, so it can afford the adjacency matrix that makes hasEdge() constant
    // time; the target cannot, and falls back to a binary search over its sorted neighbours.
    const SearchGraph patternGraph(*pattern, /* buildMatrix = */ true, patternEdgeLabels);
    const SearchGraph targetGraph(*target, /* buildMatrix = */ false, targetEdgeLabels);

    // RI matches edge labels, so it refuses only what no snapshot can represent: parallel edges
    // whose labels disagree, which the compaction has to collapse into one arc that cannot carry
    // both. The compaction is what notices, and it runs here rather than in setEdgeLabels(), which
    // is why the refusal is late. Parallel edges carrying the same label collapse losslessly and
    // are not affected.
    if (patternGraph.collapsedLabelledEdges() || targetGraph.collapsedLabelledEdges())
        throw std::runtime_error("RI does not support parallel edges whose edge labels disagree - "
                                 "see SubgraphIsomorphism::setEdgeLabels()");

    // Deciding the matching order is the expensive part of RI, and it depends only on the two
    // graphs. ParallelRI computes exactly the same thing once and shares it across its workers.
    const RIImpl::Ordering ordering = RIImpl::computeOrdering(
        patternGraph, targetGraph, patternNodeLabels, targetNodeLabels, variant);

    RIImpl(patternGraph, targetGraph, patternNodeLabels, targetNodeLabels, ordering, semantics,
           variant, handler, [this](const Match &match) { return reportMatch(match); })
        .run();

    // RIImpl may only poll isRunning(), so an interrupted search just returns. Without this, that
    // lands straight in finishRun() and the caller silently gets a truncated match set instead of
    // the documented InterruptException. ParallelRI.cpp does exactly this after its workers join.
    handler.assureRunning();
    finishRun();
}

} // namespace NetworKit
