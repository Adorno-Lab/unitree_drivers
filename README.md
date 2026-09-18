![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/juanjqo/capybara_toolkit?color=orange)![Static Badge](https://img.shields.io/badge/status-experimental-red)

# unitree_drivers
Robot-agnostic, pImpl-based C++ classes that wrap the Unitree SDK for locomotion, arm/waist control, low-level joint control, and state/IMU telemetry across Unitree's robot lineup. Reusable building blocks for higher-level drivers (per-robot facades or sas::RobotDriver implementations), not a complete driver itself.


# Install


```shell
# 1. Configure: choose Release, and (optionally) where to install it.
#    Omit -DCMAKE_INSTALL_PREFIX to use the system default (/usr/local on Linux).
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 2. Build the three shared libraries.
cmake --build build -j$(nproc)

# 3. Install headers, libraries, and the exported CMake package.
#    sudo is only needed if installing to a system path like /usr/local.
sudo cmake --install build
```
