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

#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree_drivers/DriverUnitreeArmSDK.h>

#include "test_utils.hpp"

using unitree_drivers_test::Listener;
using unitree_drivers_test::Signaler;
using unitree_drivers_test::Talker;
using unitree_drivers_test::wait_until;
using ROBOT = DriverUnitreeArmSDK::ROBOT;
using LIMB = DriverUnitreeArmSDK::LIMB;

namespace
{

const std::vector<LIMB> kLimbs{LIMB::LEFT_ARM, LIMB::RIGHT_ARM, LIMB::WAIST};

/// Motor slots of each limb, and the slot that carries the blend weight.
const std::map<LIMB, std::vector<int>> kG1Indices{
    {LIMB::LEFT_ARM, {15, 16, 17, 18, 19, 20, 21}},
    {LIMB::RIGHT_ARM, {22, 23, 24, 25, 26, 27, 28}},
    {LIMB::WAIST, {12, 13, 14}},
};
const std::map<LIMB, std::vector<int>> kH1Indices{
    {LIMB::LEFT_ARM, {16, 17, 18, 19}},
    {LIMB::RIGHT_ARM, {12, 13, 14, 15}},
    {LIMB::WAIST, {6}},
};
constexpr int kG1WeightIndex = 29;
constexpr int kH1WeightIndex = 9;

/// Default gains.
const std::map<LIMB, std::vector<double>> kG1Kp{
    {LIMB::LEFT_ARM, {80, 80, 80, 80, 40, 40, 40}},
    {LIMB::RIGHT_ARM, {80, 80, 80, 80, 40, 40, 40}},
    {LIMB::WAIST, {300, 300, 300}},
};
const std::map<LIMB, std::vector<double>> kG1Kd{
    {LIMB::LEFT_ARM, {3, 3, 3, 3, 1.5, 1.5, 1.5}},
    {LIMB::RIGHT_ARM, {3, 3, 3, 3, 1.5, 1.5, 1.5}},
    {LIMB::WAIST, {3, 3, 3}},
};
constexpr double kH1Kp = 60.0;
constexpr double kH1Kd = 1.5;

/// The measured position of motor slot i in the state messages below. Within the limits of the H1, otherwise its
/// targets would be moved into them.
float q_of(const int& i) { return -0.05f + 0.01f*static_cast<float>(i); }

template <typename State>
State state_message()
{
    State msg{};
    for (std::size_t i = 0; i < msg.motor_state().size(); ++i) {
        msg.motor_state().at(i).q(q_of(static_cast<int>(i)));
    }
    return msg;
}

bool near(const std::vector<double>& a, const std::vector<double>& b, const double& tolerance = 1e-6)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a.at(i) - b.at(i)) > tolerance) {
            return false;
        }
    }
    return true;
}

std::vector<double> measured(const std::vector<int>& indices)
{
    std::vector<double> q;
    for (const int index : indices) {
        q.push_back(q_of(index));
    }
    return q;
}

class ArmSdkWithDds : public ::testing::Test
{
protected:
    void SetUp() override { unitree_drivers_test::init_dds(); }
};

/// Checks the measured positions of every limb, and the command published once the arm control is enabled.
template <typename Cmd, typename State>
void expect_layout_in_messages(const ROBOT& robot, const std::string& state_topic,
                               const std::map<LIMB, std::vector<int>>& indices, const int& weight_index)
{
    Talker<State> talker(state_topic);
    Listener<Cmd> listener("rt/arm_sdk");
    Signaler signaler;
    DriverUnitreeArmSDK driver(signaler.ptr, robot);
    driver.connect();
    driver.initialize();
    const State state = state_message<State>();
    for (const auto& limb : kLimbs) {
        ASSERT_TRUE(wait_until([&] { return near(driver.get_positions(limb), measured(indices.at(limb))); },
                               [&] { talker.write(state); }))
            << "limb " << static_cast<int>(limb);
    }

    // Not enabled: the weight is zero, and no joint is commanded.
    ASSERT_TRUE(wait_until([&] { return listener.received(); }, [&] { talker.write(state); }));
    EXPECT_FLOAT_EQ(listener.latest().motor_cmd().at(weight_index).q(), 0.0f);
    EXPECT_FLOAT_EQ(listener.latest().motor_cmd().at(indices.at(LIMB::LEFT_ARM).at(0)).kp(), 0.0f);

    // Enabled: the weight ramps up, and every joint holds its measured position with the default gains.
    driver.enable_arm_control();
    ASSERT_TRUE(wait_until([&] { return listener.latest().motor_cmd().at(weight_index).q() > 0.0f; },
                           [&] { talker.write(state); }));
    const Cmd msg = listener.latest();
    for (const auto& limb : kLimbs) {
        const std::vector<double> kp = driver.get_desired_kp(limb);
        for (std::size_t i = 0; i < indices.at(limb).size(); ++i) {
            const auto& motor = msg.motor_cmd().at(indices.at(limb).at(i));
            EXPECT_FLOAT_EQ(motor.q(), q_of(indices.at(limb).at(i))) << "limb " << static_cast<int>(limb) << ", joint " << i;
            EXPECT_FLOAT_EQ(motor.kp(), static_cast<float>(kp.at(i)));
        }
    }
}

} // namespace

TEST(ArmSdk, NullShutdownSignalerThrows)
{
    EXPECT_THROW(DriverUnitreeArmSDK(nullptr, ROBOT::G1), std::invalid_argument);
}

TEST(ArmSdk, RobotTypeAndNumberOfJoints)
{
    Signaler signaler;
    DriverUnitreeArmSDK g1(signaler.ptr, ROBOT::G1);
    DriverUnitreeArmSDK h1(signaler.ptr, ROBOT::H1);
    EXPECT_TRUE(g1.get_robot_type() == ROBOT::G1);
    EXPECT_TRUE(h1.get_robot_type() == ROBOT::H1);
    for (const auto& limb : kLimbs) {
        EXPECT_EQ(g1.get_num_joints(limb), kG1Indices.at(limb).size()) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(h1.get_num_joints(limb), kH1Indices.at(limb).size()) << "limb " << static_cast<int>(limb);
    }
}

TEST(ArmSdk, DefaultGains)
{
    Signaler signaler;
    DriverUnitreeArmSDK g1(signaler.ptr, ROBOT::G1);
    DriverUnitreeArmSDK h1(signaler.ptr, ROBOT::H1);
    for (const auto& limb : kLimbs) {
        EXPECT_EQ(g1.get_target_kp(limb), kG1Kp.at(limb)) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(g1.get_target_kd(limb), kG1Kd.at(limb)) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(g1.get_desired_kp(limb), kG1Kp.at(limb));
        EXPECT_EQ(g1.get_desired_kd(limb), kG1Kd.at(limb));
        const std::size_t n = kH1Indices.at(limb).size();
        EXPECT_EQ(h1.get_target_kp(limb), std::vector<double>(n, kH1Kp)) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(h1.get_target_kd(limb), std::vector<double>(n, kH1Kd)) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(h1.get_desired_kp(limb), std::vector<double>(n, kH1Kp));
        EXPECT_EQ(h1.get_desired_kd(limb), std::vector<double>(n, kH1Kd));
    }
}

TEST(ArmSdk, SetGains)
{
    Signaler signaler;
    DriverUnitreeArmSDK driver(signaler.ptr, ROBOT::H1);
    const std::vector<double> kp{10, 20, 30, 40};
    const std::vector<double> kd{1, 2, 3, 4};
    driver.set_gains(LIMB::LEFT_ARM, kp, kd);
    EXPECT_EQ(driver.get_target_kp(LIMB::LEFT_ARM), kp);
    EXPECT_EQ(driver.get_target_kd(LIMB::LEFT_ARM), kd);
    // The desired gains only move towards the targets in the control loop.
    EXPECT_EQ(driver.get_desired_kp(LIMB::LEFT_ARM), std::vector<double>(4, kH1Kp));

    EXPECT_THROW(driver.set_gains(LIMB::LEFT_ARM, {1, 2, 3}, {1, 2, 3}), std::invalid_argument);
    EXPECT_THROW(driver.set_gains(LIMB::WAIST, {500.1}, {1}), std::invalid_argument);
    EXPECT_THROW(driver.set_gains(LIMB::WAIST, {-1}, {1}), std::invalid_argument);
    EXPECT_THROW(driver.set_gains(LIMB::WAIST, {1}, {20.1}), std::invalid_argument);
    EXPECT_THROW(driver.set_gains(LIMB::WAIST, {1}, {std::nan("")}), std::invalid_argument);
    EXPECT_NO_THROW(driver.set_gains(LIMB::WAIST, {500}, {20}));
}

TEST(ArmSdk, SetTargetPositionsWithWrongSizeThrows)
{
    Signaler signaler;
    DriverUnitreeArmSDK driver(signaler.ptr, ROBOT::G1);
    EXPECT_THROW(driver.set_target_positions(LIMB::LEFT_ARM, std::vector<double>(4, 0.0)), std::invalid_argument);
    EXPECT_NO_THROW(driver.set_target_positions(LIMB::LEFT_ARM, std::vector<double>(7, 0.0)));
}

TEST_F(ArmSdkWithDds, G1Layout)
{
    expect_layout_in_messages<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>(
        ROBOT::G1, "rt/lowstate", kG1Indices, kG1WeightIndex);
}

TEST_F(ArmSdkWithDds, H1Layout)
{
    expect_layout_in_messages<unitree_go::msg::dds_::LowCmd_, unitree_go::msg::dds_::LowState_>(
        ROBOT::H1, "rt/lf/lowstate", kH1Indices, kH1WeightIndex);
}

TEST_F(ArmSdkWithDds, H1TargetsAreLimited)
{
    // Left shoulder pitch (slot 16) in [-2.87, 2.87] and roll (slot 17) in [-0.34, 3.11], with a margin of 0.15.
    Talker<unitree_go::msg::dds_::LowState_> talker("rt/lf/lowstate");
    Signaler signaler;
    DriverUnitreeArmSDK driver(signaler.ptr, ROBOT::H1);
    driver.connect();
    driver.initialize();
    auto state = state_message<unitree_go::msg::dds_::LowState_>();
    state.motor_state().at(16).q(2.70f);
    state.motor_state().at(17).q(-0.17f);
    ASSERT_TRUE(wait_until([&] { return std::abs(driver.get_positions(LIMB::LEFT_ARM).at(0) - 2.70) < 1e-6; },
                           [&] { talker.write(state); }));

    driver.set_target_positions(LIMB::LEFT_ARM, {3.5, -1.0, q_of(18), q_of(19)});
    driver.enable_arm_control();
    const std::vector<double> limited{2.72, -0.19, q_of(18), q_of(19)};
    EXPECT_TRUE(wait_until([&] { return near(driver.get_desired_positions(LIMB::LEFT_ARM), limited, 1e-5); },
                           [&] { talker.write(state); }))
        << "desired: " << ::testing::PrintToString(driver.get_desired_positions(LIMB::LEFT_ARM));
}

TEST_F(ArmSdkWithDds, G1TargetsAreNotLimited)
{
    Talker<unitree_hg::msg::dds_::LowState_> talker("rt/lowstate");
    Signaler signaler;
    DriverUnitreeArmSDK driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    driver.initialize();
    auto state = state_message<unitree_hg::msg::dds_::LowState_>();
    state.motor_state().at(15).q(2.70f);
    ASSERT_TRUE(wait_until([&] { return std::abs(driver.get_positions(LIMB::LEFT_ARM).at(0) - 2.70) < 1e-6; },
                           [&] { talker.write(state); }));

    std::vector<double> target = measured(kG1Indices.at(LIMB::LEFT_ARM));
    target.at(0) = 2.80;
    driver.set_target_positions(LIMB::LEFT_ARM, target);
    driver.enable_arm_control();
    EXPECT_TRUE(wait_until([&] { return near(driver.get_desired_positions(LIMB::LEFT_ARM), target, 1e-5); },
                           [&] { talker.write(state); }));
}
