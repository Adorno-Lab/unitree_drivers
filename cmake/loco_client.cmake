# ---------------------------------------------------------------------------
# Library: loco_client (wraps DriverUnitreeLocoClient)
# ---------------------------------------------------------------------------
# PUBLIC deps: unitree_sdk2 and dqrobotics/Eigen appear in the installed public
# header (DriverUnitreeLocoClient.h includes <dqrobotics/DQ.h>), so consumers
# need them too.
#
# sas_core_pure is now a *usage-requirement* PUBLIC dependency as well -- the
# constructor takes a std::shared_ptr<sas::ShutdownSignaler>, so
# <sas_core/sas_shutdown_signaler.hpp> is pulled into the installed public
# header and any caller of the constructor needs that type fully defined. It's
# wrapped in $<BUILD_INTERFACE:...> rather than being a plain PUBLIC target,
# though: sas_core_pure is never installed/exported by sas_core itself (see the
# note in CMakeLists.txt), and install(EXPORT unitree_driversTargets ...) below
# refuses to export a target whose PUBLIC/INTERFACE link libraries include
# another target that isn't part of some export set -- a plain PUBLIC
# sas_core_pure here would make `cmake --install` fail outright, not just leak
# an unresolvable dependency at consumption time. $<BUILD_INTERFACE:...> keeps
# sas_core_pure's include dirs and library visible to anything consuming this
# target from the same build (add_subdirectory/FetchContent), while dropping it
# entirely from what gets exported to install(EXPORT). A consumer of the
# *installed* unitree_drivers package therefore still needs sas_core's headers
# on its own include path to compile against the ShutdownSignaler-typed
# constructor, and needs to link sas_core itself if it calls
# shutdown_signaler->shutdown()/should_shutdown() -- get that the same way this
# file's own FetchContent block does, it isn't provided transitively.
add_library(loco_client SHARED
    src/DriverUnitreeLocoClient.cpp
)
add_library(unitree_drivers::loco_client ALIAS loco_client)

target_compile_features(loco_client PUBLIC cxx_std_17)

target_include_directories(loco_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(loco_client
    PUBLIC
        unitree_sdk2
        Eigen3::Eigen
        ${DQROBOTICS_LIBRARY}
        $<BUILD_INTERFACE:sas_core_pure>
)

set_target_properties(loco_client PROPERTIES
    OUTPUT_NAME unitree_drivers_loco_client
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
