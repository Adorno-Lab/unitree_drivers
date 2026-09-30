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
#           - Developed with the assistance of Claude (Anthropic).
#
#   Acknowledgement: The H1 support in this class is based on the developments by
#                    Daniel S. J. Derwent (email: daniel.derwent@manchester.ac.uk)
#                    in https://github.com/Adorno-Lab/sas_robot_driver_unitree_h1
#
# ################################################################
*/

#include "unitree_drivers/DriverUnitreeArmSDK.h"
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <variant>

static const std::string kTopicArmSDK = "rt/arm_sdk";
// G1 (unitree_hg message set) publishes its state on rt/lowstate. The H1 driver in
// Adorno-Lab's sas_robot_driver_unitree_h1 subscribes to unitree_go::msg::dds_::LowState_
// on rt/lf/lowstate instead (same choice as DriverUnitreeLowState::ROBOT::H1), and that
// is what has been run against the physical H1. unitree_sdk2's own H1 arm example reads
// hg::LowState_ from rt/lowstate; if a given H1 firmware turns out to need that, this is
// the one place (plus H1Channels below) to change.
static const std::string kTopicLowStateG1 = "rt/lowstate";
static const std::string kTopicLowStateH1 = "rt/lf/lowstate";

namespace
{

using ROBOT = DriverUnitreeArmSDK::ROBOT;
using LIMB = DriverUnitreeArmSDK::LIMB;

/**
 * @brief Everything about the arm/waist joint layout that differs between robots.
 *
 * @details joints holds the motor slot of every controlled joint, ordered left arm,
 *          then right arm, then waist -- so the per-joint std::vector<float> buffers in
 *          Impl (target/current/desired) share this ordering, and a limb is a
 *          contiguous [offset, offset + count) slice of them.
 */
struct Layout
{
    std::array<std::size_t,3> counts{};      ///< Joints in the left arm, right arm and waist.
    std::vector<int> joints;                 ///< Motor slots, left arm + right arm + waist.
    std::vector<float> lower_limits;         ///< Per-joint target lower bound (rad), same order as joints; empty = no clamping.
    std::vector<float> upper_limits;         ///< Per-joint target upper bound (rad), same order as joints; empty = no clamping.
    int weight_index{};                      ///< Motor slot whose q carries the blend weight.
    std::string state_topic;

    std::size_t count(const LIMB& limb) const { return counts.at(static_cast<std::size_t>(limb)); }

    std::size_t offset(const LIMB& limb) const
    {
        std::size_t off = 0;
        for (std::size_t i = 0; i < static_cast<std::size_t>(limb); ++i) off += counts.at(i);
        return off;
    }
};

Layout make_layout(const ROBOT& robot)
{
    Layout layout;
    switch (robot) {
    case ROBOT::G1:
        // unitree_hg, 35 motor slots. See example/g1/high_level/g1_arm7_sdk_dds_example.cpp.
        layout.counts = {7, 7, 3};
        layout.joints = {
            15, 16, 17, 18, 19, 20, 21,   // left arm: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw
            22, 23, 24, 25, 26, 27, 28,   // right arm: same 7 joints
            12, 13, 14                    // waist: Yaw, Roll, Pitch
        };
        layout.weight_index = 29;         // kNotUsedJoint
        layout.state_topic = kTopicLowStateG1;
        break;
    case ROBOT::H1: {
        // unitree_go, 20 motor slots. Indices match example/h1/high_level/h1_arm_sdk_dds_example.cpp
        // and DriverUnitreeH1 (sas_robot_driver_unitree_h1).
        layout.counts = {4, 4, 1};
        layout.joints = {
            16, 17, 18, 19,               // left arm: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow
            12, 13, 14, 15,               // right arm: same 4 joints
            6                             // waist: Yaw
        };
        layout.weight_index = 9;          // kNotUsedJoint
        layout.state_topic = kTopicLowStateH1;

        // Position limits from DriverUnitreeH1, shrunk by the same 0.15 rad margin it applies.
        constexpr float kMargin = 0.15f;
        const std::vector<std::pair<float,float>> limits = {
            {-2.87f, 2.87f}, {-0.34f, 3.11f}, {-1.30f, 4.45f}, {-1.25f, 2.61f},  // left arm
            {-2.87f, 2.87f}, {-3.11f, 0.34f}, {-4.45f, 1.30f}, {-1.25f, 2.61f},  // right arm
            {-2.35f, 2.35f}                                                     // waist yaw
        };
        for (const auto& [lo, hi] : limits) {
            layout.lower_limits.push_back(lo + kMargin);
            layout.upper_limits.push_back(hi - kMargin);
        }
        break;
    }
    default:
        throw std::invalid_argument("DriverUnitreeArmSDK: unknown ROBOT");
    }
    return layout;
}

/**
 * @brief The DDS objects for one robot's message set. G1 and H1 use different IDL
 *        types (unitree_hg vs unitree_go) that share no base class, but expose the same
 *        motor_cmd()/motor_state() accessors, so Impl handles them through std::visit
 *        with generic lambdas and one instantiation per set.
 */
template <typename CmdT, typename StateT>
struct Channels
{
    using Cmd = CmdT;
    using State = StateT;

    unitree::robot::ChannelPublisherPtr<CmdT> publisher;
    unitree::robot::ChannelSubscriberPtr<StateT> subscriber;
    CmdT cmd_msg{};
    StateT latest_state{};
};

using G1Channels = Channels<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>;
using H1Channels = Channels<unitree_go::msg::dds_::LowCmd_, unitree_go::msg::dds_::LowState_>;

} // namespace

class DriverUnitreeArmSDK::Impl
{
public:
    const ROBOT robot_type_;
    const Layout layout_;

    // target_ is the user-facing goal (set via the public setters), desired_ is the
    // rate-limited trajectory point actually published each control tick, current_ is
    // the last measured joint state read back from the state topic. All are ordered as
    // Layout::joints.
    std::vector<float> target_;
    std::vector<float> current_;
    std::vector<float> desired_;
    // Per limb (LIMB order): whether set_target_positions() has been called for it. A limb
    // with no target yet holds its measured pose on engage instead of tracking toward the
    // all-zero initial target_. Guarded by data_mutex_.
    std::array<bool,3> target_set_{};

    // rt/arm_sdk blend/ramp parameters. See official arm_sdk examples (G1 and H1).
    float arm_weight_{0.0f};
    float weight_rate_{0.2f};          // weight units/sec ramp rate (both engage & disengage)
    float max_joint_velocity_{0.5f};   // rad/s cap on the position tracker
    double arm_control_period_{0.02};  // control loop period, seconds
    std::atomic<bool> arms_enabled_{false};

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_initialized_{false};

    std::unique_ptr<marinholab::sas::core::ThreadManager> arm_control_thread_;
    mutable std::mutex data_mutex_;

    // Declared before channels_ so it outlives the DDS subscriber callback that locks it
    // (members are destroyed in reverse order of declaration).
    std::mutex low_state_mutex_;
    bool has_state_{false};            ///< Guarded by low_state_mutex_. True once one state message arrived.

    std::variant<G1Channels, H1Channels> channels_;
    bool arm_control_seeded_{false};

    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler_; ///< Shared shutdown coordinator, polled every tick in arm_control_loop_callback().

    explicit Impl(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                  const ROBOT& robot_type,
                  const double& control_period)
        : robot_type_{robot_type},
        layout_{make_layout(robot_type)},
        target_(layout_.joints.size(), 0.0f),
        current_(layout_.joints.size(), 0.0f),
        desired_(layout_.joints.size(), 0.0f),
        arm_control_period_{control_period},
        shutdown_signaler_{shutdown_signaler}
    {
        if (robot_type == ROBOT::H1) {
            channels_.emplace<H1Channels>();
        }
    }

    void connect_channels()
    {
        std::visit([this](auto& ch) {
            using Ch = std::decay_t<decltype(ch)>;
            ch.publisher.reset(new unitree::robot::ChannelPublisher<typename Ch::Cmd>(kTopicArmSDK));
            ch.publisher->InitChannel();

            ch.subscriber.reset(new unitree::robot::ChannelSubscriber<typename Ch::State>(layout_.state_topic));
            ch.subscriber->InitChannel([this, &ch](const void* msg) {
                std::scoped_lock lock(low_state_mutex_);
                ch.latest_state = *static_cast<const typename Ch::State*>(msg);
                has_state_ = true;
            }, 1);
        }, channels_);
    }

    void write_cmd()
    {
        std::visit([](auto& ch) { ch.publisher->Write(ch.cmd_msg); }, channels_);
    }

    float get_cmd_weight()
    {
        return std::visit([this](auto& ch) { return static_cast<float>(ch.cmd_msg.motor_cmd().at(layout_.weight_index).q()); }, channels_);
    }

    void set_cmd_weight(const float weight)
    {
        std::visit([this, weight](auto& ch) { ch.cmd_msg.motor_cmd().at(layout_.weight_index).q(weight); }, channels_);
    }

    /// Copies the latest measured joint positions into current_. Returns false if no state message has arrived yet.
    bool update_measured_positions()
    {
        std::scoped_lock state_lock(low_state_mutex_);
        if (!has_state_) {
            return false;
        }
        std::visit([this](auto& ch) {
            for (std::size_t i = 0; i < layout_.joints.size(); ++i)
                current_.at(i) = ch.latest_state.motor_state().at(layout_.joints.at(i)).q();
        }, channels_);
        return true;
    }

    /// Target for joint @p j after applying this robot's joint limits (none for G1).
    float limited_target(const std::size_t j) const
    {
        if (layout_.lower_limits.empty()) {
            return target_.at(j);
        }
        return std::clamp(target_.at(j), layout_.lower_limits.at(j), layout_.upper_limits.at(j));
    }

    void arm_control_loop_callback()
    {
        if (!is_connected_ || !is_initialized_) {
            return;
        }

        std::scoped_lock lock(data_mutex_);
        try {
            // Interruption requested (e.g. SIGINT): disengage arm control so the
            // weight-ramp logic below decays arm_weight_ to 0 over the next few
            // ticks, holding the last commanded pose -- the same path
            // disable_arm_control() takes. Deliberately does NOT call deinitialize()
            // (and therefore not sas::ThreadManager::stop()) from here: this callback
            // runs ON the arm control thread itself, and stop() joins that thread --
            // a thread joining itself is a self-join deadlock. Actual thread teardown
            // happens later, from the main thread, via this object's destructor (or
            // an explicit deinitialize()/disconnect() call) -- see the class-level
            // @note.
            if (shutdown_signaler_->should_shutdown()) {
                arms_enabled_ = false;
            }

            // Nothing is published until the robot's state has been seen at least once,
            // so the tracker can never be seeded from the all-zero initial buffer.
            if (!update_measured_positions()) {
                return;
            }

            const bool enabled = arms_enabled_;
            const float delta_weight = weight_rate_ * static_cast<float>(arm_control_period_);
            const float max_joint_delta = max_joint_velocity_ * static_cast<float>(arm_control_period_);

            if (enabled && !arm_control_seeded_) {
                // Seed the tracker from measured state so engaging never snaps the arm.
                desired_ = current_;
                // Limbs that were never given a target hold where they are.
                for (const LIMB limb : {LIMB::LEFT_ARM, LIMB::RIGHT_ARM, LIMB::WAIST}) {
                    if (!target_set_.at(static_cast<std::size_t>(limb))) {
                        const std::size_t off = layout_.offset(limb);
                        for (std::size_t i = off; i < off + layout_.count(limb); ++i)
                            target_.at(i) = current_.at(i);
                    }
                }
                arm_control_seeded_ = true;
            }
            if (!enabled) {
                arm_control_seeded_ = false; // force a fresh seed on next engage
            }

            // Ramp the blend weight toward its target (0 or 1). Ramping on engage too
            // (rather than snapping to 1.0 as the reference examples do) is an extra
            // safety margin on top of position-matching at the moment of engagement.
            arm_weight_ = enabled
                              ? std::min(1.0f, arm_weight_ + delta_weight)
                              : std::max(0.0f, arm_weight_ - delta_weight);

            std::visit([&](auto& ch) {
                ch.cmd_msg.motor_cmd().at(layout_.weight_index).q(arm_weight_);

                if (enabled) {
                    for (std::size_t j = 0; j < desired_.size(); ++j) {
                        desired_.at(j) += std::clamp(
                            limited_target(j) - desired_.at(j),
                            -max_joint_delta, max_joint_delta);

                        auto& mc = ch.cmd_msg.motor_cmd().at(layout_.joints.at(j));
                        mc.q(desired_.at(j));
                        mc.dq(0.f);
                        mc.kp(60.f);
                        mc.kd(1.5f);
                        mc.tau(0.f);
                    }
                }
                // When disabling: q/kp/kd of the arm joints are deliberately left untouched
                // here, so cmd_msg keeps holding its last commanded pose while only the
                // weight decays -- matching the official arm_sdk examples' shutdown behavior.

                ch.publisher->Write(ch.cmd_msg);
            }, channels_);
        } catch (const std::exception& e) {
            std::cerr << "DriverUnitreeArmSDK arm control loop error: " << e.what() << std::endl;
        }
    }

    void start_arm_control_thread(const double& period,
                                  const marinholab::sas::core::ThreadManager::PRIORITY& priority)
    {
        if (arm_control_thread_ && arm_control_thread_->is_running()) {
            return; // Already running
        }
        arm_control_period_ = period;
        arm_control_thread_ = std::make_unique<marinholab::sas::core::ThreadManager>(
            robot_type_ == ROBOT::G1 ? "g1_arm_sdk_control" : "h1_arm_sdk_control",
            period,
            std::bind(&Impl::arm_control_loop_callback, this),
            priority,
            -1
            );
        arm_control_thread_->start();
    }

    void stop_arm_control_thread()
    {
        if (arm_control_thread_) {
            arm_control_thread_->stop();
            arm_control_thread_.reset();
        }
    }

    // Synchronously ramps weight 1 -> 0 (holding the last commanded pose), so shutdown
    // doesn't depend on the background thread getting more ticks in before it's stopped.
    void disable_arm_control_blocking()
    {
        std::scoped_lock lock(data_mutex_);
        arms_enabled_ = false;
        arm_control_seeded_ = false;

        if (get_cmd_weight() <= 0.0f && arm_weight_ <= 0.0f) {
            return;
        }

        const float ramp_period = 0.02f;
        const float delta_weight = weight_rate_ * ramp_period;
        const auto sleep_time = std::chrono::milliseconds(static_cast<int>(ramp_period * 1000));

        while (arm_weight_ > 0.0f) {
            arm_weight_ = std::max(0.0f, arm_weight_ - delta_weight);
            set_cmd_weight(arm_weight_);
            try {
                write_cmd();
            } catch (const std::exception& e) {
                std::cerr << "DriverUnitreeArmSDK::disable_arm_control_blocking: " << e.what() << std::endl;
                break;
            }
            std::this_thread::sleep_for(sleep_time);
        }
        set_cmd_weight(0.0f);
        try {
            write_cmd();
        } catch (const std::exception& e) {
            std::cerr << "DriverUnitreeArmSDK::disable_arm_control_blocking: " << e.what() << std::endl;
        }
    }

    std::vector<double> limb_values(const std::vector<float>& v, const LIMB& limb) const
    {
        const auto begin = v.begin() + static_cast<std::ptrdiff_t>(layout_.offset(limb));
        return std::vector<double>(begin, begin + static_cast<std::ptrdiff_t>(layout_.count(limb)));
    }
};

/**
 * @brief Constructs the driver for @p robot_type.
 * @param shutdown_signaler Shared sas::ShutdownSignaler (typically the same one a
 *        SIGINT handler calls shutdown() on), forwarded to Impl so the background arm
 *        control loop callback can poll should_shutdown() every tick.
 * @param robot_type Which robot's message set, state topic and joint layout to use.
 * @param control_period Period, in seconds, of the background arm control loop. Also
 *        used inside the loop to scale weight_rate_/max_joint_velocity_ into per-tick
 *        deltas, so it must match the period the thread actually ticks at.
 * @note No hardware/DDS I/O happens here; see connect().
 * @throws std::invalid_argument if shutdown_signaler is nullptr or robot_type is not a known ROBOT.
 */
DriverUnitreeArmSDK::DriverUnitreeArmSDK(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                                             const ROBOT& robot_type,
                                             const double &control_period)
    : robot_type_{robot_type}
{
    if (shutdown_signaler == nullptr) {
        throw std::invalid_argument("DriverUnitreeArmSDK: shutdown_signaler must not be nullptr");
    }
    impl_ = std::make_shared<Impl>(shutdown_signaler, robot_type, control_period);
}

DriverUnitreeArmSDK::~DriverUnitreeArmSDK()
{
    deinitialize();
    disconnect();
}



void DriverUnitreeArmSDK::connect()
{
    // Precondition: unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
    // must already have been called by the owning driver (e.g. DriverUnitreeG1 / DriverUnitreeH1),
    // same as for the aggregated DriverUnitreeLocoClient.
    impl_->connect_channels();
    impl_->is_connected_ = true;
}

void DriverUnitreeArmSDK::initialize()
{
    if (!impl_->is_connected_) {
        throw std::runtime_error("DriverUnitreeArmSDK::initialize: Cannot initialize: not connected");
    }
    impl_->is_initialized_ = true;
    if (!impl_->arm_control_thread_ || !impl_->arm_control_thread_->is_running()) {
        impl_->start_arm_control_thread(impl_->arm_control_period_, marinholab::sas::core::ThreadManager::PRIORITY::NORMAL);
    }
}

void DriverUnitreeArmSDK::deinitialize()
{
    if (impl_->is_initialized_) {
        impl_->disable_arm_control_blocking();
        impl_->stop_arm_control_thread();
        impl_->is_initialized_ = false;
    }
}

void DriverUnitreeArmSDK::disconnect()
{
    if (impl_->is_connected_) {
        if (impl_->is_initialized_) {
            deinitialize();
        } else {
            impl_->stop_arm_control_thread();
        }
        impl_->is_connected_ = false;
    }
}

void DriverUnitreeArmSDK::enable_arm_control()
{
    // Flips a flag only -- the control loop performs the actual weight ramp-up and
    // seeds the trajectory tracker from the measured pose on the next tick.
    std::scoped_lock lock(impl_->data_mutex_);
    impl_->arms_enabled_ = true;
}

void DriverUnitreeArmSDK::disable_arm_control()
{
    // For a graceful, non-blocking disable during normal operation. deinitialize()
    // uses disable_arm_control_blocking() instead to guarantee the ramp completes.
    std::scoped_lock lock(impl_->data_mutex_);
    impl_->arms_enabled_ = false;
}

bool DriverUnitreeArmSDK::is_arm_control_enabled() const
{
    return impl_->arms_enabled_;
}

DriverUnitreeArmSDK::ROBOT DriverUnitreeArmSDK::get_robot_type() const
{
    return robot_type_;
}

std::size_t DriverUnitreeArmSDK::get_num_joints(const LIMB& limb) const
{
    return impl_->layout_.count(limb);
}

void DriverUnitreeArmSDK::set_target_positions(const LIMB& limb, const std::vector<double>& target_positions)
{
    const std::size_t n = impl_->layout_.count(limb);
    if (target_positions.size() != n) {
        throw std::invalid_argument("DriverUnitreeArmSDK::set_target_positions: expected " + std::to_string(n) +
                                    " positions for this limb on this robot, got " + std::to_string(target_positions.size()));
    }
    std::scoped_lock lock(impl_->data_mutex_);
    const std::size_t off = impl_->layout_.offset(limb);
    for (std::size_t i = 0; i < n; ++i)
        impl_->target_.at(off + i) = static_cast<float>(target_positions.at(i));
    impl_->target_set_.at(static_cast<std::size_t>(limb)) = true;
}

std::vector<double> DriverUnitreeArmSDK::get_positions(const LIMB& limb)
{
    std::scoped_lock lock(impl_->data_mutex_);
    return impl_->limb_values(impl_->current_, limb);
}

std::vector<double> DriverUnitreeArmSDK::get_desired_positions(const LIMB& limb)
{
    std::scoped_lock lock(impl_->data_mutex_);
    return impl_->limb_values(impl_->desired_, limb);
}
