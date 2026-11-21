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

#include <map>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree_drivers/DriverUnitreeLowCmd.h>

#include "test_utils.hpp"

using unitree_drivers_test::Listener;
using unitree_drivers_test::Signaler;
using unitree_drivers_test::wait_until;
using ROBOT = DriverUnitreeLowCmd::ROBOT;
using LIMB = DriverUnitreeLowCmd::LIMB;
using G1Cmd = unitree_hg::msg::dds_::LowCmd_;
using H1Cmd = unitree_go::msg::dds_::LowCmd_;

namespace
{

const std::string kTopic = "rt/lowcmd";
const std::vector<LIMB> kLimbs{LIMB::LEFT_ARM, LIMB::RIGHT_ARM, LIMB::LEFT_LEG, LIMB::RIGHT_LEG, LIMB::WAIST};

/// Motor slots of each limb.
const std::map<LIMB, std::vector<int>> kG1Indices{
    {LIMB::LEFT_ARM, {15, 16, 17, 18, 19, 20, 21}},
    {LIMB::RIGHT_ARM, {22, 23, 24, 25, 26, 27, 28}},
    {LIMB::LEFT_LEG, {0, 1, 2, 3, 4, 5}},
    {LIMB::RIGHT_LEG, {6, 7, 8, 9, 10, 11}},
    {LIMB::WAIST, {12, 13, 14}},
};
const std::map<LIMB, std::vector<int>> kH1Indices{
    {LIMB::LEFT_ARM, {16, 17, 18, 19}},
    {LIMB::RIGHT_ARM, {12, 13, 14, 15}},
    {LIMB::LEFT_LEG, {7, 3, 4, 5, 10}},
    {LIMB::RIGHT_LEG, {8, 0, 1, 2, 11}},
    {LIMB::WAIST, {6}},
};

/// Distinct positions for a limb, e.g. 1.1, 1.2, ... for the first one.
Eigen::VectorXd positions_of(const LIMB& limb, const std::size_t& n)
{
    Eigen::VectorXd q(static_cast<Eigen::Index>(n));
    for (std::size_t i = 0; i < n; ++i) {
        q(static_cast<Eigen::Index>(i)) = 1.0 + static_cast<double>(limb) + 0.1*static_cast<double>(i + 1);
    }
    return q;
}

class LowCmdWithDds : public ::testing::Test
{
protected:
    void SetUp() override { unitree_drivers_test::init_dds(); }
};

/// Sets the positions of every limb, publishes, and checks the motor slots of the message received.
template <typename Msg>
void expect_limb_indices_in_message(const ROBOT& robot, const std::map<LIMB, std::vector<int>>& indices)
{
    Listener<Msg> listener(kTopic);
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, robot);
    driver.connect();
    driver.initialize();
    for (const auto& limb : kLimbs) {
        driver.set_joint_positions(limb, positions_of(limb, indices.at(limb).size()));
    }
    ASSERT_TRUE(wait_until([&] { return listener.received(); }, [&] { driver.publish(); }));
    const Msg msg = listener.latest();
    for (const auto& limb : kLimbs) {
        const Eigen::VectorXd q = positions_of(limb, indices.at(limb).size());
        for (std::size_t i = 0; i < indices.at(limb).size(); ++i) {
            const int index = indices.at(limb).at(i);
            EXPECT_FLOAT_EQ(msg.motor_cmd().at(index).q(), static_cast<float>(q(static_cast<Eigen::Index>(i))))
                << "limb " << static_cast<int>(limb) << ", joint " << i;
            EXPECT_FLOAT_EQ(driver.get_joint_command(static_cast<std::size_t>(index)).q,
                            static_cast<float>(q(static_cast<Eigen::Index>(i))));
        }
    }
    EXPECT_EQ(msg.crc(), unitree_drivers_test::expected_crc(msg));
}

} // namespace

TEST(LowCmd, NullShutdownSignalerThrows)
{
    EXPECT_THROW(DriverUnitreeLowCmd(nullptr, ROBOT::G1), std::invalid_argument);
}

TEST(LowCmd, NumberOfJoints)
{
    Signaler signaler;
    EXPECT_EQ(DriverUnitreeLowCmd(signaler.ptr, ROBOT::G1).num_joints(), 35u);
    EXPECT_EQ(DriverUnitreeLowCmd(signaler.ptr, ROBOT::H1).num_joints(), 20u);
}

TEST(LowCmd, NumberOfJointsPerLimb)
{
    Signaler signaler;
    DriverUnitreeLowCmd g1(signaler.ptr, ROBOT::G1);
    DriverUnitreeLowCmd h1(signaler.ptr, ROBOT::H1);
    for (const auto& limb : kLimbs) {
        EXPECT_EQ(g1.num_joints(limb), kG1Indices.at(limb).size()) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(h1.num_joints(limb), kH1Indices.at(limb).size()) << "limb " << static_cast<int>(limb);
    }
}

TEST(LowCmd, PublishWithoutConnectionThrows)
{
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, ROBOT::G1);
    EXPECT_THROW(driver.publish(), std::runtime_error);
}

TEST_F(LowCmdWithDds, WrongSizesThrow)
{
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    EXPECT_THROW(driver.set_joint_positions(LIMB::LEFT_ARM, Eigen::VectorXd::Zero(6)), std::invalid_argument);
    EXPECT_THROW(driver.set_limb_command(LIMB::WAIST, std::vector<DriverUnitreeLowCmd::MotorCommand>(2)),
                 std::invalid_argument);
}

TEST_F(LowCmdWithDds, G1LimbIndices)
{
    expect_limb_indices_in_message<G1Cmd>(ROBOT::G1, kG1Indices);
}

TEST_F(LowCmdWithDds, H1LimbIndices)
{
    expect_limb_indices_in_message<H1Cmd>(ROBOT::H1, kH1Indices);
}

TEST_F(LowCmdWithDds, MotorCommandFields)
{
    Listener<G1Cmd> listener(kTopic);
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    driver.initialize();
    DriverUnitreeLowCmd::MotorCommand command;
    command.q = 0.5;
    command.dq = -0.25;
    command.tau = 1.5;
    command.kp = 60.0;
    command.kd = 2.0;
    command.enable = true;
    std::vector<DriverUnitreeLowCmd::MotorCommand> commands(3, command);
    commands.at(1).enable = false;
    driver.set_limb_command(LIMB::WAIST, commands);

    ASSERT_TRUE(wait_until([&] { return listener.received(); }, [&] { driver.publish(); }));
    const G1Cmd msg = listener.latest();
    for (const int index : {12, 14}) {
        const auto& motor = msg.motor_cmd().at(index);
        EXPECT_EQ(motor.mode(), 1);
        EXPECT_FLOAT_EQ(motor.q(), 0.5f);
        EXPECT_FLOAT_EQ(motor.dq(), -0.25f);
        EXPECT_FLOAT_EQ(motor.tau(), 1.5f);
        EXPECT_FLOAT_EQ(motor.kp(), 60.0f);
        EXPECT_FLOAT_EQ(motor.kd(), 2.0f);
    }
    EXPECT_EQ(msg.motor_cmd().at(13).mode(), 0);

    const DriverUnitreeLowCmd::MotorCommand read = driver.get_joint_command(12);
    EXPECT_TRUE(read.enable);
    EXPECT_DOUBLE_EQ(read.q, 0.5);
    EXPECT_DOUBLE_EQ(read.kd, 2.0);
}

TEST_F(LowCmdWithDds, G1AnkleModeAndModeMachine)
{
    Listener<G1Cmd> listener(kTopic);
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    driver.initialize();

    driver.set_ankle_mode(DriverUnitreeLowCmd::ANKLE_MODE::AB);
    driver.set_mode_machine(5);
    ASSERT_TRUE(wait_until([&] { return listener.received(); }, [&] { driver.publish(); }));
    EXPECT_EQ(listener.latest().mode_pr(), 1);
    EXPECT_EQ(listener.latest().mode_machine(), 5);

    driver.set_ankle_mode(DriverUnitreeLowCmd::ANKLE_MODE::PR);
    ASSERT_TRUE(wait_until([&] { return listener.latest().mode_pr() == 0; }, [&] { driver.publish(); }));
    EXPECT_EQ(listener.latest().crc(), unitree_drivers_test::expected_crc(listener.latest()));
}

TEST_F(LowCmdWithDds, H1MessageFraming)
{
    Listener<H1Cmd> listener(kTopic);
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, ROBOT::H1);
    driver.connect();
    driver.initialize();
    // No mode_pr nor mode_machine in the H1 message: these are no-ops.
    driver.set_ankle_mode(DriverUnitreeLowCmd::ANKLE_MODE::AB);
    driver.set_mode_machine(5);

    ASSERT_TRUE(wait_until([&] { return listener.received(); }, [&] { driver.publish(); }));
    const H1Cmd msg = listener.latest();
    EXPECT_EQ(msg.head().at(0), 0xFE);
    EXPECT_EQ(msg.head().at(1), 0xEF);
    EXPECT_EQ(msg.level_flag(), 0xFF);
    EXPECT_EQ(msg.gpio(), 0);
    EXPECT_EQ(msg.crc(), unitree_drivers_test::expected_crc(msg));
}

TEST_F(LowCmdWithDds, PublishAfterShutdownReturnsFalse)
{
    Signaler signaler;
    DriverUnitreeLowCmd driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    driver.initialize();
    EXPECT_TRUE(driver.publish());
    signaler.stop = true;
    EXPECT_FALSE(driver.publish());
}
