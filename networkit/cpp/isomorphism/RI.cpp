#include <networkit/auxiliary/SignalHandling.hpp>
#include <networkit/isomorphism/RI.hpp>

#include "RIImpl.hpp"

namespace NetworKit {

RI::RI(const Graph &pattern, const Graph &target, Variant variant, Semantics semantics,
       count maxMatches)
    : SubgraphIsomorphism(pattern, target, semantics, maxMatches), variant(variant) {}

void RI::run() {
    using IsomorphismDetails::prepareRISearch;
    using IsomorphismDetails::RIImpl;
    using IsomorphismDetails::RISearchSetup;

    Aux::SignalHandler handler;
    prepareRun();

    // The snapshots, the RI-DS domains and the matching order, built in the one sequence that
    // works. Shared with ParallelRI::run() rather than repeated here, so the sequential and the
    // parallel search cannot end up answering different questions.
    const RISearchSetup setup =
        prepareRISearch(*pattern, *target, patternNodeLabels, targetNodeLabels, patternEdgeLabels,
                        targetEdgeLabels, variant, "RI");

    RIImpl(setup.patternGraph, setup.targetGraph, patternNodeLabels, targetNodeLabels,
           setup.ordering, setup.domains, semantics, handler,
           [this](const Match &match) { return reportMatch(match); })
        .run();

    // RIImpl may only poll isRunning(), so an interrupted search just returns. Without this, that
    // lands straight in finishRun() and the caller silently gets a truncated match set instead of
    // the documented InterruptException. ParallelRI.cpp does exactly this after its workers join.
    handler.assureRunning();
    finishRun();
}

} // namespace NetworKit
