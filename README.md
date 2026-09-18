![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/capybara_toolkit?color=orange)![Static Badge](https://img.shields.io/badge/status-experimental-red)

# unitree_drivers
Robot-agnostic, pImpl-based C++ classes that wrap the Unitree SDK for locomotion, arm/waist control, low-level joint control, and state/IMU telemetry across Unitree's robot lineup. Reusable building blocks for higher-level drivers (per-robot facades or sas::RobotDriver implementations), not a complete driver itself.


# Install

> [!NOTE]
> Non-sudo privileges? Create a custom prefix folder (e.g. `~/opt`) to hold `lib/` and `include/` without needing root. See [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html) for background.

## Prerequisites

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) — build and install it first, following its own instructions.
- Eigen3 — `sudo apt install libeigen3-dev`
- [DQ Robotics](https://dqrobotics.github.io) — installed system-wide (e.g. via their apt PPA).

If you're installing any of the above without sudo, install them to the same custom prefix `~/opt`, and see the non-sudo instructions for `unitree_drivers` itself.


## Sudo users

```shell
# 1. Configure: choose Release, and (optionally) where to install it.
#    Omit -DCMAKE_INSTALL_PREFIX to use the system default (/usr/local on Linux).
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 2. Build the three shared libraries.
cmake --build build -j$(nproc)

# 3. Install headers, libraries, and the exported CMake package.
sudo cmake --install build
```

## Non-sudo users

```shell
# 1. Configure: choose Release, and install to your own prefix instead of a system path.
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/opt

# 2. Build the three shared libraries.
cmake --build build -j$(nproc)

# 3. Install headers, libraries, and the exported CMake package. No sudo needed.
cmake --install build
```

Any project that later does `find_package(unitree_drivers)` needs to know where to look, since `$HOME/opt` isn't a default search path:

```shell
cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/opt
```


# Usage

```cmake
find_package(unitree_drivers REQUIRED)
target_link_libraries(${YOUR_LIBRARY} PRIVATE
     unitree_drivers::loco_client
     unitree_drivers::arm_sdk
     unitree_drivers::low_state)
```


```
include <unitree_drivers/DriverUnitreeLocoClient.h>
```

