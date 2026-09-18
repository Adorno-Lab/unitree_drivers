# ---------------------------------------------------------------------------
# Library: g1_arm_sdk (wraps DriverUnitreeG1ArmSDK)
# ---------------------------------------------------------------------------
# No dqrobotics/Eigen dependency: DriverUnitreeG1ArmSDK.h only pulls in <array>,
# <memory>, <sas_core/sas_shutdown_signaler.hpp>. unitree_sdk2 is used only in
# the .cpp, but is kept PUBLIC here anyway since it's the shared vocabulary type
# of this whole package family (consistent with loco_client) -- feel free to
# make it PRIVATE if that's not a concern for your consumers.
#
# sas_core_pure is now a *usage-requirement* PUBLIC dependency too -- see the
# long comment in cmake/loco_client.cmake for why it's wrapped in
# $<BUILD_INTERFACE:...> rather than being a plain PUBLIC target (a plain
# PUBLIC sas_core_pure here would make install(EXPORT unitree_driversTargets
# ...) in CMakeLists.txt fail outright, since sas_core_pure is never itself
# installed/exported). Consumers of the *installed* package need sas_core's
# headers on their own include path to compile against the
# ShutdownSignaler-typed constructor, and need to link sas_core themselves to
# call shutdown_signaler->shutdown()/should_shutdown() -- neither is provided
# transitively.
add_library(g1_arm_sdk SHARED
    src/DriverUnitreeG1ArmSDK.cpp
)
add_library(unitree_drivers::g1_arm_sdk ALIAS g1_arm_sdk)

target_compile_features(g1_arm_sdk PUBLIC cxx_std_17)

target_include_directories(g1_arm_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(g1_arm_sdk
    PUBLIC
        unitree_sdk2
        $<BUILD_INTERFACE:sas_core_pure>
)

set_target_properties(g1_arm_sdk PROPERTIES
    OUTPUT_NAME unitree_drivers_g1_arm_sdk
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
