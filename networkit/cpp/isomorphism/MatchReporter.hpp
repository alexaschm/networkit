#ifndef NETWORKIT_CPP_ISOMORPHISM_MATCH_REPORTER_HPP_
#define NETWORKIT_CPP_ISOMORPHISM_MATCH_REPORTER_HPP_

// Private header of the isomorphism module. Not installed, not part of the public API.

#include <functional>
#include <vector>

#include <networkit/Globals.hpp>
#include <networkit/isomorphism/SubgraphIsomorphism.hpp>

namespace NetworKit {
namespace IsomorphismDetails {

/**
 * How a search core hands a complete mapping back to the algorithm that owns it.
 *
 * The cores (VF2Impl, VF3Impl, RIImpl) are not subclasses of SubgraphIsomorphism - they live in
 * the .cpp files so that the public header never grows a member - so they cannot call its
 * protected reportMatch() themselves. The algorithm's run(), which can, wraps it in a lambda and
 * passes that down:
 *
 * @code
 * VF2Impl(..., [this](const SubgraphIsomorphism::Match &m) { return reportMatch(m); }).run();
 * @endcode
 *
 * @param match The mapping, indexed by pattern node.
 * @return false once the search must stop, because the requested number of matches is reached.
 */
using MatchReporter = std::function<bool(const SubgraphIsomorphism::Match &)>;

} // namespace IsomorphismDetails
} // namespace NetworKit

#endif // NETWORKIT_CPP_ISOMORPHISM_MATCH_REPORTER_HPP_
