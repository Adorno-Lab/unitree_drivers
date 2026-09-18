cmake_minimum_required(VERSION 3.16)

project(unitree_drivers
    VERSION 0.1.0
    DESCRIPTION "Robot-agnostic pImpl wrappers around the Unitree SDK: locomotion, arm/waist control, and low-level joint/IMU telemetry"
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# unitree_sdk2 ships a proper CMake package config (unitree_sdk2Config.cmake +
# unitree_sdk2Targets.cmake) once installed, exposing a plain (non-namespaced)
# `unitree_sdk2` imported target -- link against that name directly.
find_package(unitree_sdk2 REQUIRED)

# Eigen3 is header-only; Eigen3::Eigen is the standard imported target its own
# Config file provides.
find_package(Eigen3 REQUIRED)

# DQ Robotics (https://dqrobotics.github.io) does not ship a CMake package config
# -- it's conventionally installed system-wide (e.g. via its apt PPA) as a plain
# shared library with no discoverable Config.cmake. find_library() locates it and
# gives us an absolute path we can embed in our exported target, which is more
# portable than passing a bare "-ldqrobotics" link flag through to consumers.

# Not required at the moment.
#find_library(DQROBOTICS_LIBRARY dqrobotics)
#if(NOT DQROBOTICS_LIBRARY)
#    message(FATAL_ERROR
#        "libdqrobotics not found. Install DQ Robotics system-wide "
#        "(see https://dqrobotics.github.io) before configuring unitree_drivers."
#    )
#endif()

# sas_core, when built outside a ROS2/ament workspace (ROS2_BUILD=OFF), does not
# install or export ANYTHING -- its install()/ament_export_targets() calls are
# gated behind `if(ROS2_BUILD)` in its own CMakeLists.txt. That means the
# sas_core_pure target it produces can only ever be a build-tree target, never
# something a downstream find_package(unitree_drivers) could locate on its own.
#
# This is precisely why loco_client/arm_sdk below are built SHARED rather than
# STATIC: a shared library fully resolves and embeds its PRIVATE dependencies at
# its own build/link time, so sas_core_pure never needs to be visible to whatever
# links against the installed unitree_drivers libraries later. If these were
# static libraries instead, sas_core_pure would leak through as an unresolvable
# transitive dependency the moment someone else's find_package(unitree_drivers)
# tried to use them.
#
# Guarded with `if(NOT TARGET sas_core_pure)` so this stays safe to use both
# standalone and when unitree_drivers itself is add_subdirectory()'d/FetchContent's
# by a parent project that also happens to need sas_core.
if(NOT TARGET sas_core_pure)
    include(FetchContent)
    FetchContent_Declare(
        sas_core
        GIT_REPOSITORY https://github.com/SmartArmStack/sas_core.git
        GIT_TAG        jazzy
    )
    set(ROS2_BUILD OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(sas_core)
endif()

# ---------------------------------------------------------------------------
# Library: loco_client (wraps DriverUnitreeLocoClient)
# ---------------------------------------------------------------------------
# PUBLIC deps: unitree_sdk2 and dqrobotics/Eigen appear in the installed public
# header (DriverUnitreeLocoClient.h includes <dqrobotics/DQ.h>), so consumers
# need them too. sas_core_pure is PRIVATE -- see the note above.
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
        #Eigen3::Eigen
        #${DQROBOTICS_LIBRARY}  //
    PRIVATE
        sas_core_pure
)

set_target_properties(loco_client PROPERTIES
    OUTPUT_NAME unitree_drivers_loco_client
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ---------------------------------------------------------------------------
# Library: g1_arm_sdk (wraps DriverUnitreeG1ArmSDK)
# ---------------------------------------------------------------------------
# No dqrobotics/Eigen dependency: DriverUnitreeG1ArmSDK.h only pulls in <array>,
# <atomic>, <memory>. unitree_sdk2 is used only in the .cpp, but is kept PUBLIC
# here anyway since it's the shared vocabulary type of this whole package family
# (consistent with loco_client) -- feel free to make it PRIVATE if that's not a
# concern for your consumers. sas_core_pure is PRIVATE -- see the note above.
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
    PRIVATE
        sas_core_pure
)

set_target_properties(g1_arm_sdk PROPERTIES
    OUTPUT_NAME unitree_drivers_arm_sdk
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ---------------------------------------------------------------------------
# Library: low_state (wraps DriverUnitreeLowState)
# ---------------------------------------------------------------------------
# No sas_core dependency at all (no background control loop -- see the class's
# own docs), so this target has no shared/static constraint forcing SHARED the
# way loco_client/arm_sdk do. Built SHARED anyway for consistency across the
# package. Eigen3 is PUBLIC (Eigen/Dense is in the public header); unitree_sdk2
# is only used in the .cpp, so it's PRIVATE here.
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
    PRIVATE
        unitree_sdk2
)

set_target_properties(low_state PROPERTIES
    OUTPUT_NAME unitree_drivers_low_state
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ---------------------------------------------------------------------------
# Install: headers, libraries, and the exported CMake package
# ---------------------------------------------------------------------------
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS loco_client g1_arm_sdk low_state
    EXPORT unitree_driversTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT unitree_driversTargets
    FILE unitree_driversTargets.cmake
    NAMESPACE unitree_drivers::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_drivers
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/unitree_driversConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_driversConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_drivers
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_driversConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_driversConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/unitree_driversConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unitree_drivers
)

# How to use
# find_package(unitree_drivers REQUIRED)
# target_link_libraries(driver-g1 PRIVATE
#     unitree_drivers::loco_client
#     unitree_drivers::g1_arm_sdk
#     unitree_drivers::low_state
#)
