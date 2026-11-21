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

#include <algorithm>
#include <map>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree_drivers/DriverUnitreeLowState.h>

#include "test_utils.hpp"

using unitree_drivers_test::Signaler;
using unitree_drivers_test::Talker;
using unitree_drivers_test::wait_until;
using ROBOT = DriverUnitreeLowState::ROBOT;
using LIMB = DriverUnitreeLowState::LIMB;
using G1State = unitree_hg::msg::dds_::LowState_;
using H1State = unitree_go::msg::dds_::LowState_;

namespace
{

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

// The state of motor slot i, in the messages below.
float q_of(const int& i) { return 1.0f + 0.01f*static_cast<float>(i); }
float dq_of(const int& i) { return -0.1f*static_cast<float>(i); }
float tau_of(const int& i) { return 0.5f*static_cast<float>(i); }

/// A G1 message: the temperature of slot i is max(20 + i, 30).
G1State g1_message()
{
    G1State msg{};
    for (std::size_t i = 0; i < msg.motor_state().size(); ++i) {
        auto& motor = msg.motor_state().at(i);
        const int index = static_cast<int>(i);
        motor.q(q_of(index));
        motor.dq(dq_of(index));
        motor.tau_est(tau_of(index));
        motor.temperature({static_cast<int16_t>(20 + index), 30});
    }
    msg.imu_state().quaternion({0.5f, 0.5f, -0.5f, 0.5f});
    msg.imu_state().gyroscope({0.1f, 0.2f, 0.3f});
    msg.imu_state().accelerometer({0.0f, 0.0f, 9.81f});
    msg.imu_state().rpy({0.01f, 0.02f, 0.03f});
    return msg;
}

/// An H1 message: the temperature of slot i is 40 + i, and the state of charge is 87.
H1State h1_message()
{
    H1State msg{};
    for (std::size_t i = 0; i < msg.motor_state().size(); ++i) {
        auto& motor = msg.motor_state().at(i);
        const int index = static_cast<int>(i);
        motor.q(q_of(index));
        motor.dq(dq_of(index));
        motor.tau_est(tau_of(index));
        motor.temperature(static_cast<uint8_t>(40 + index));
    }
    msg.imu_state().quaternion({1.0f, 0.0f, 0.0f, 0.0f});
    msg.imu_state().rpy({0.04f, 0.05f, 0.06f});
    msg.bms_state().soc(87);
    return msg;
}

class LowStateWithDds : public ::testing::Test
{
protected:
    void SetUp() override { unitree_drivers_test::init_dds(); }
};

/// Publishes msg until the driver receives it, then checks the getters of every limb.
template <typename Msg>
void expect_limb_values(const ROBOT& robot, const std::string& topic, const Msg& msg,
                        const std::map<LIMB, std::vector<int>>& indices, double (*temperature_of)(const int&))
{
    Talker<Msg> talker(topic);
    Signaler signaler;
    DriverUnitreeLowState driver(signaler.ptr, robot);
    driver.connect();
    driver.initialize();
    ASSERT_TRUE(wait_until([&] { return driver.num_joints() > 0; }, [&] { talker.write(msg); }));

    EXPECT_EQ(driver.num_joints(), msg.motor_state().size());
    EXPECT_EQ(driver.get_joint_positions().size(), msg.motor_state().size());
    for (const auto& limb : kLimbs) {
        const Eigen::VectorXd q = driver.get_joint_positions(limb);
        const Eigen::VectorXd dq = driver.get_joint_velocities(limb);
        const Eigen::VectorXd tau = driver.get_joint_torques(limb);
        const Eigen::VectorXd temperature = driver.get_joint_temperatures(limb);
        ASSERT_EQ(static_cast<std::size_t>(q.size()), indices.at(limb).size()) << "limb " << static_cast<int>(limb);
        for (std::size_t i = 0; i < indices.at(limb).size(); ++i) {
            const int index = indices.at(limb).at(i);
            const auto k = static_cast<Eigen::Index>(i);
            EXPECT_FLOAT_EQ(static_cast<float>(q(k)), q_of(index)) << "limb " << static_cast<int>(limb) << ", joint " << i;
            EXPECT_FLOAT_EQ(static_cast<float>(dq(k)), dq_of(index));
            EXPECT_FLOAT_EQ(static_cast<float>(tau(k)), tau_of(index));
            EXPECT_DOUBLE_EQ(temperature(k), temperature_of(index));
        }
    }
}

double g1_temperature(const int& i) { return std::max(20.0 + i, 30.0); }
double h1_temperature(const int& i) { return 40.0 + i; }

} // namespace

TEST(LowState, NullShutdownSignalerThrows)
{
    EXPECT_THROW(DriverUnitreeLowState(nullptr, ROBOT::G1), std::invalid_argument);
}

TEST(LowState, NumberOfJointsPerLimb)
{
    Signaler signaler;
    DriverUnitreeLowState g1(signaler.ptr, ROBOT::G1);
    DriverUnitreeLowState h1(signaler.ptr, ROBOT::H1);
    for (const auto& limb : kLimbs) {
        EXPECT_EQ(g1.num_joints(limb), kG1Indices.at(limb).size()) << "limb " << static_cast<int>(limb);
        EXPECT_EQ(h1.num_joints(limb), kH1Indices.at(limb).size()) << "limb " << static_cast<int>(limb);
    }
}

TEST(LowState, BeforeTheFirstMessage)
{
    Signaler signaler;
    for (const auto& robot : {ROBOT::G1, ROBOT::H1}) {
        DriverUnitreeLowState driver(signaler.ptr, robot);
        EXPECT_EQ(driver.num_joints(), 0u);
        EXPECT_TRUE(driver.get_joint_positions().empty());
        EXPECT_EQ(driver.get_joint_positions(LIMB::LEFT_ARM).size(), 0);
        EXPECT_FALSE(driver.get_imu_data().valid);
        EXPECT_THROW(driver.get_state_of_charge(), std::runtime_error);
    }
}

TEST_F(LowStateWithDds, G1LimbValues)
{
    expect_limb_values(ROBOT::G1, "rt/lowstate", g1_message(), kG1Indices, &g1_temperature);
}

TEST_F(LowStateWithDds, H1LimbValues)
{
    expect_limb_values(ROBOT::H1, "rt/lf/lowstate", h1_message(), kH1Indices, &h1_temperature);
}

TEST_F(LowStateWithDds, G1Imu)
{
    Talker<G1State> talker("rt/lowstate");
    Signaler signaler;
    DriverUnitreeLowState driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    driver.initialize();
    ASSERT_TRUE(wait_until([&] { return driver.get_imu_data().valid; }, [&] { talker.write(g1_message()); }));
    const auto imu = driver.get_imu_data();
    EXPECT_DOUBLE_EQ(imu.quaternion.at(0), 0.5);
    EXPECT_DOUBLE_EQ(imu.quaternion.at(2), -0.5);
    EXPECT_FLOAT_EQ(static_cast<float>(imu.gyroscope.at(1)), 0.2f);
    EXPECT_FLOAT_EQ(static_cast<float>(imu.accelerometer.at(2)), 9.81f);
    EXPECT_FLOAT_EQ(static_cast<float>(imu.rpy.at(2)), 0.03f);
}

TEST_F(LowStateWithDds, G1StateOfChargeFromBmsTopic)
{
    Talker<G1State> state_talker("rt/lowstate");
    Talker<unitree_hg::msg::dds_::BmsState_> bms_talker("rt/lf/bmsstate");
    Signaler signaler;
    DriverUnitreeLowState driver(signaler.ptr, ROBOT::G1);
    driver.connect();
    driver.initialize();
    ASSERT_TRUE(wait_until([&] { return driver.num_joints() > 0; }, [&] { state_talker.write(g1_message()); }));
    // rt/lowstate alone has no battery on the G1.
    EXPECT_THROW(driver.get_state_of_charge(), std::runtime_error);

    unitree_hg::msg::dds_::BmsState_ bms{};
    bms.soc(64);
    double soc = -1.0;
    ASSERT_TRUE(wait_until([&] {
        try {
            soc = driver.get_state_of_charge();
            return true;
        } catch (const std::runtime_error&) {
            return false;
        }
    }, [&] { bms_talker.write(bms); }));
    EXPECT_DOUBLE_EQ(soc, 64.0);
}

TEST_F(LowStateWithDds, H1StateOfChargeFromLowState)
{
    Talker<H1State> talker("rt/lf/lowstate");
    Signaler signaler;
    DriverUnitreeLowState driver(signaler.ptr, ROBOT::H1);
    driver.connect();
    driver.initialize();
    ASSERT_TRUE(wait_until([&] { return driver.num_joints() > 0; }, [&] { talker.write(h1_message()); }));
    EXPECT_DOUBLE_EQ(driver.get_state_of_charge(), 87.0);
    EXPECT_FLOAT_EQ(static_cast<float>(driver.get_imu_data().rpy.at(1)), 0.05f);
}
