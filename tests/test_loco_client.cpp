/*
#    Copyright (c) 2024-2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 2.1 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software. If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: - Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

// The locomotion client calls the services of the robot, so only what does not need them is tested here.

#include <stdexcept>

#include <gtest/gtest.h>
#include <unitree_drivers/DriverUnitreeLocoClient.h>

#include "test_utils.hpp"

using unitree_drivers_test::Signaler;
using ROBOT = DriverUnitreeLocoClient::ROBOT;

TEST(LocoClient, NullShutdownSignalerThrows)
{
    EXPECT_THROW(DriverUnitreeLocoClient(nullptr, ROBOT::G1), std::invalid_argument);
}

TEST(LocoClient, H1HasNoSpeedMode)
{
    Signaler signaler;
    DriverUnitreeLocoClient driver(signaler.ptr, ROBOT::H1);
    EXPECT_THROW(driver.set_speed_mode(1), std::runtime_error);
}

TEST(LocoClient, InitializeWithoutConnectionThrows)
{
    Signaler signaler;
    DriverUnitreeLocoClient driver(signaler.ptr, ROBOT::G1);
    EXPECT_THROW(driver.initialize(), std::runtime_error);
}
