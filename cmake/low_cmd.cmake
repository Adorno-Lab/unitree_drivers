# ---------------------------------------------------------------------------
# Library: low_cmd (wraps DriverUnitreeLowCmd)
# ---------------------------------------------------------------------------
# No background control loop (see the class's own docs) -- publish() is called
# explicitly by the owning control loop, and the ShutdownSignaler it takes at
# construction is only consulted from publish() itself, not polled by a loop of
# its own here. Its constructor still takes a std::shared_ptr<sas::ShutdownSignaler>
# for interface consistency with loco_client/g1_arm_sdk/low_state, so
# <marinholab/sas/core/sas_shutdown_signaler.hpp> is pulled into this library's
# installed public header too -- marinholab::sas::core is therefore a PUBLIC
# usage-requirement dependency, same as the other three libraries in this project
# (see cmake/loco_client.cmake for the history of why that is a plain PUBLIC target
# now rather than a $<BUILD_INTERFACE:...>-only one).
# unitree_sdk2 is only used in the .cpp (IDL message types, ChannelPublisher), so
# it's PRIVATE here -- this library's public header (DriverUnitreeLowCmd.h) has no
# unitree_sdk2 types in its interface, unlike loco_client/g1_arm_sdk.
add_library(low_cmd SHARED
    src/DriverUnitreeLowCmd.cpp
)
add_library(unitree_drivers::low_cmd ALIAS low_cmd)

target_compile_features(low_cmd PUBLIC cxx_std_17)

target_include_directories(low_cmd PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(low_cmd
    PUBLIC
        marinholab::sas::core
    PRIVATE
        unitree_sdk2
)

set_target_properties(low_cmd PROPERTIES
    OUTPUT_NAME unitree_drivers_low_cmd
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
