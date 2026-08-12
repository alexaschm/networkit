networkit_add_module(isomorphism
    ParallelRI.cpp
    RI.cpp
    RIImpl.cpp
    SearchGraph.cpp
    SubgraphIsomorphism.cpp
    VF2.cpp
    VF3.cpp
    )

networkit_module_link_modules(isomorphism
    auxiliary graph)
