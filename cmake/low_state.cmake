# ---------------------------------------------------------------------------
# Library: low_state (wraps DriverUnitreeLowState)
# ---------------------------------------------------------------------------
# No background control loop (see the class's own docs) -- the ShutdownSignaler
# it takes at construction drives no behavior here -- but its constructor still
# takes a std::shared_ptr<sas::ShutdownSignaler> for interface consistency with
# loco_client/g1_arm_sdk, so <sas_core/sas_shutdown_signaler.hpp> is now pulled
# into this library's installed public header too (previously a non-issue for
# this particular library, since it had no sas_core dependency at all). Same
# $<BUILD_INTERFACE:...> treatment as loco_client/g1_arm_sdk applies here, and
# for the same reason -- see the long comment in cmake/loco_client.cmake.
# Eigen3 is PUBLIC (Eigen/Dense is in the public header); unitree_sdk2 is only
# used in the .cpp, so it's PRIVATE here.
add_library(low_state SHARED
    src/DriverUnitreeLowState.cpp
)
add_library(unitree_drivers::low_state ALIAS low_state)

target_compile_features(low_state PUBLIC cxx_std_17)

target_include_directories(low_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(low_state
    PUBLIC
        Eigen3::Eigen
        $<BUILD_INTERFACE:sas_core_pure>
    PRIVATE
        unitree_sdk2
)

set_target_properties(low_state PROPERTIES
    OUTPUT_NAME unitree_drivers_low_state
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
