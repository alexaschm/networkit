#include <stdexcept>

#include <tlx/unused.hpp>

#include "RIImpl.hpp"

namespace NetworKit {
namespace IsomorphismDetails {

RIImpl::Ordering RIImpl::computeOrdering(const SearchGraph &pattern, const SearchGraph &target,
                                         const std::vector<index> &patternLabels,
                                         const std::vector<index> &targetLabels,
                                         RI::Variant variant) {
    tlx::unused(pattern, target, patternLabels, targetLabels, variant);
    throw std::logic_error("RIImpl::computeOrdering() is not implemented yet");
}

RIImpl::RIImpl(const SearchGraph &pattern, const SearchGraph &target,
               const std::vector<index> &patternLabels, const std::vector<index> &targetLabels,
               const Ordering &ordering, SubgraphIsomorphism::Semantics semantics,
               RI::Variant variant, Aux::SignalHandler &handler,
               SubgraphIsomorphism::MatchSink sink)
    : patternGraph(&pattern), targetGraph(&target), patternLabels(&patternLabels),
      targetLabels(&targetLabels), labelled(!patternLabels.empty()), ordering(&ordering),
      semantics(semantics), variant(variant), handler(&handler), sink(sink) {}

void RIImpl::run() {
    // TODO: remove once implemented.
    tlx::unused(patternGraph, targetGraph, patternLabels, targetLabels, labelled, ordering,
                semantics, variant, handler, sink, domains, matchBuffer, candidateBuffer);
    throw std::logic_error("RIImpl::run() is not implemented yet");
}

bool RIImpl::expand(State &state, std::vector<State> &children) {
    tlx::unused(state, children);
    throw std::logic_error("RIImpl::expand() is not implemented yet");
}

void RIImpl::candidatesFor(const State &state, std::vector<node> &out) const {
    tlx::unused(state, out);
    throw std::logic_error("RIImpl::candidatesFor() is not implemented yet");
}

bool RIImpl::consistent(const State &state, node tv) const {
    tlx::unused(state, tv);
    throw std::logic_error("RIImpl::consistent() is not implemented yet");
}

bool RIImpl::ruleEdgesToPrefix(const State &state, node tv) const {
    tlx::unused(state, tv);
    throw std::logic_error("RIImpl::ruleEdgesToPrefix() is not implemented yet");
}

bool RIImpl::ruleNonEdgesToPrefix(const State &state, node tv) const {
    tlx::unused(state, tv);
    throw std::logic_error("RIImpl::ruleNonEdgesToPrefix() is not implemented yet");
}

bool RIImpl::ruleLabels(node pu, node tv) const {
    tlx::unused(pu, tv);
    throw std::logic_error("RIImpl::ruleLabels() is not implemented yet");
}

bool RIImpl::forwardCheck(const State &state, node tv) const {
    tlx::unused(state, tv);
    throw std::logic_error("RIImpl::forwardCheck() is not implemented yet");
}

void RIImpl::initializeDomains() {
    throw std::logic_error("RIImpl::initializeDomains() is not implemented yet");
}

bool RIImpl::reportMapping(const State &state) {
    tlx::unused(state);
    throw std::logic_error("RIImpl::reportMapping() is not implemented yet");
}

} // namespace IsomorphismDetails
} // namespace NetworKit
