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

> [!TIP]
> If you skipped exporting `CMAKE_PREFIX_PATH` (along with `LD_LIBRARY_PATH`,
> `LIBRARY_PATH`, and `CPATH`) in `~/.bashrc` (see [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html)),
> any project that later does `find_package(unitree_drivers)` needs to be told
> where to look, since `$HOME/opt` isn't a default search path:
>
> ```shell
> cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/opt
> ```


# Usage

```cmake
find_package(unitree_drivers REQUIRED)
target_link_libraries(${YOUR_LIBRARY} PRIVATE
     unitree_drivers::loco_client
     unitree_drivers::g1_arm_sdk
     unitree_drivers::low_state)
```


```cpp
#include <unitree_drivers/DriverUnitreeLocoClient.h>
#include <unitree_drivers/DriverUnitreeG1ArmSDK.h>
#include <unitree_drivers/DriverUnitreeLowState.h>
```

## Shutdown signaling

Every driver constructor takes a shared [`sas::ShutdownSignaler`](https://github.com/SmartArmStack/sas_core)
instead of a raw `std::atomic_bool*`. Construct one, wire it up to your signal
handler, and hand the same `std::shared_ptr` to every sub-driver you build:

```cpp
#include <signal.h>
#include <memory>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <unitree_drivers/DriverUnitreeLocoClient.h>
#include <unitree_drivers/DriverUnitreeG1ArmSDK.h>
#include <unitree_drivers/DriverUnitreeLowState.h>

static std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler =
    std::make_shared<sas::ShutdownSignaler>();

void sig_int_handler(int)
{
    shutdown_signaler->shutdown();
}

int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_int_handler) == SIG_ERR) {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    DriverUnitreeLocoClient loco_client(shutdown_signaler, DriverUnitreeLocoClient::ROBOT::G1);
    DriverUnitreeG1ArmSDK arm_sdk(shutdown_signaler);
    DriverUnitreeLowState low_state(shutdown_signaler, DriverUnitreeLowState::ROBOT::G1);

    // ... connect()/initialize() each driver, run your application loop ...
}
```

Each background control loop (`DriverUnitreeLocoClient`, `DriverUnitreeG1ArmSDK`)
polls `shutdown_signaler->should_shutdown()` every tick and ramps down/zeros its
output the moment it returns `true`, without waiting for the owning application
to notice and call `deinitialize()` itself. `DriverUnitreeLowState` accepts and
validates the same signaler for interface consistency, but has no background
loop of its own to drive.

Consumers of the *installed* `unitree_drivers` package (via `find_package`)
need `sas_core`'s headers on their own include path to compile against the
`ShutdownSignaler`-typed constructors, and need to link `sas_core` themselves
if they call `shutdown_signaler->shutdown()`/`should_shutdown()` directly --
neither is provided transitively. Depend on
[`sas_core`](https://github.com/SmartArmStack/sas_core) the same way this
project's own `CMakeLists.txt` does (e.g. via `FetchContent`).

