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

    prepareRun();

    // The pattern is small, so it can afford the adjacency matrix that makes hasEdge() constant
    // time; the target cannot, and falls back to a binary search over its sorted neighbours.
    const SearchGraph patternGraph(*pattern, /* buildMatrix = */ true);
    const SearchGraph targetGraph(*target, /* buildMatrix = */ false);

    // Deciding the matching order is the expensive part of RI, and it depends only on the two
    // graphs. ParallelRI computes exactly the same thing once and shares it across its workers.
    const RIImpl::Ordering ordering =
        RIImpl::computeOrdering(patternGraph, targetGraph, patternLabels, targetLabels, variant);

    RIImpl(patternGraph, targetGraph, patternLabels, targetLabels, ordering, semantics, variant,
           sink())
        .run();

    finishRun();
}

} // namespace NetworKit
