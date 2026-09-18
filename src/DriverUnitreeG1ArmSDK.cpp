#include "unitree_drivers/DriverUnitreeG1ArmSDK.h"
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <sas_core/sas_thread_manager.hpp>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicLowState = "rt/lowstate";

class DriverUnitreeG1ArmSDK::Impl
{
public:
    enum JointIndex {
        // Left leg
        kLeftHipPitch,
        kLeftHipRoll,
        kLeftHipYaw,
        kLeftKnee,
        kLeftAnkle,
        kLeftAnkleRoll,

        // Right leg
        kRightHipPitch,
        kRightHipRoll,
        kRightHipYaw,
        kRightKnee,
        kRightAnkle,
        kRightAnkleRoll,

        // Waist (3)
        kWaistYaw,
        kWaistRoll,
        kWaistPitch,

        // Left arm
        kLeftShoulderPitch,
        kLeftShoulderRoll,
        kLeftShoulderYaw,
        kLeftElbow,
        kLeftWristRoll,
        kLeftWristPitch,
        kLeftWristYaw,
        // Right arm
        kRightShoulderPitch,
        kRightShoulderRoll,
        kRightShoulderYaw,
        kRightElbow,
        kRightWristRoll,
        kRightWristPitch,
        kRightWristYaw,

        kNotUsedJoint,
        kNotUsedJoint1,
        kNotUsedJoint2,
        kNotUsedJoint3,
        kNotUsedJoint4,
        kNotUsedJoint5
    };

    // target_positions_ is the user-facing goal (set via the public setters),
    // desired_positions_ is the rate-limited trajectory point actually published this
    // control tick, current_positions_ is the last measured joint state read back from
    // rt/lowstate.
    struct LeftArmData {
        std::array<float, 7> target_positions_{};
        std::array<float, 7> current_positions_{};
        std::array<float, 7> desired_positions_{};
    };

    struct RightArmData {
        std::array<float, 7> target_positions_{};
        std::array<float, 7> current_positions_{};
        std::array<float, 7> desired_positions_{};
    };

    struct WaistData {
        std::array<float, 3> target_positions_{};
        std::array<float, 3> current_positions_{};
        std::array<float, 3> desired_positions_{};
    };

    LeftArmData left_arm_data_;
    RightArmData right_arm_data_;
    WaistData waist_data_;

    // rt/arm_sdk blend/ramp parameters. See official g1 arm_sdk example.
    float arm_weight_{0.0f};
    float weight_rate_{0.2f};          // weight units/sec ramp rate (both engage & disengage)
    float max_joint_velocity_{0.5f};   // rad/s cap on the position tracker
    double arm_control_period_{0.02};  // control loop period, seconds
    std::atomic<bool> arms_enabled_{false};

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_initialized_{false};

    std::unique_ptr<sas::ThreadManager> arm_control_thread_;
    mutable std::mutex data_mutex_;

    // Arm control (DDS)
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_sdk_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
    unitree_hg::msg::dds_::LowCmd_ arm_cmd_msg_{};
    unitree_hg::msg::dds_::LowState_ latest_low_state_{};
    std::mutex low_state_mutex_;

    std::array<float,17> current_jpos_des_{};
    bool arm_control_seeded_{false};

    // Ordering matches LeftArmData (7) + RightArmData (7) + WaistData (3) concatenation.
    static constexpr std::array<int,17> kArmJoints = {
        kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
        kLeftWristRoll, kLeftWristPitch, kLeftWristYaw,
        kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
        kRightWristRoll, kRightWristPitch, kRightWristYaw,
        kWaistYaw, kWaistRoll, kWaistPitch
    };
    static constexpr int kWeightIndex = kNotUsedJoint;

    std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler_; ///< Shared shutdown coordinator, polled every tick in arm_control_loop_callback().

    explicit Impl(const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler) : shutdown_signaler_{shutdown_signaler} {}

    void low_state_callback(const void* msg)
    {
        std::scoped_lock lock(low_state_mutex_);
        latest_low_state_ = *static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
    }

    std::array<float,17> pack_arm_targets() const
    {
        std::array<float,17> out{};
        for (size_t i = 0; i < 7; ++i) out.at(i)      = left_arm_data_.target_positions_.at(i);
        for (size_t i = 0; i < 7; ++i) out.at(7 + i)  = right_arm_data_.target_positions_.at(i);
        for (size_t i = 0; i < 3; ++i) out.at(14 + i) = waist_data_.target_positions_.at(i);
        return out;
    }

    void unpack_arm_desired(const std::array<float,17>& desired)
    {
        for (size_t i = 0; i < 7; ++i) left_arm_data_.desired_positions_.at(i)  = desired.at(i);
        for (size_t i = 0; i < 7; ++i) right_arm_data_.desired_positions_.at(i) = desired.at(7 + i);
        for (size_t i = 0; i < 3; ++i) waist_data_.desired_positions_.at(i)     = desired.at(14 + i);
    }

    void update_measured_arm_positions()
    {
        std::scoped_lock state_lock(low_state_mutex_);
        for (size_t i = 0; i < 7; ++i)
            left_arm_data_.current_positions_.at(i)  = latest_low_state_.motor_state().at(kArmJoints.at(i)).q();
        for (size_t i = 0; i < 7; ++i)
            right_arm_data_.current_positions_.at(i) = latest_low_state_.motor_state().at(kArmJoints.at(7 + i)).q();
        for (size_t i = 0; i < 3; ++i)
            waist_data_.current_positions_.at(i)     = latest_low_state_.motor_state().at(kArmJoints.at(14 + i)).q();
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

            update_measured_arm_positions();

            const bool enabled = arms_enabled_;
            const float delta_weight = weight_rate_ * static_cast<float>(arm_control_period_);
            const float max_joint_delta = max_joint_velocity_ * static_cast<float>(arm_control_period_);

            if (enabled && !arm_control_seeded_) {
                // Seed the tracker from measured state so engaging never snaps the arm.
                std::scoped_lock state_lock(low_state_mutex_);
                for (size_t j = 0; j < current_jpos_des_.size(); ++j) {
                    current_jpos_des_.at(j) = latest_low_state_.motor_state().at(kArmJoints.at(j)).q();
                }
                arm_control_seeded_ = true;
            }
            if (!enabled) {
                arm_control_seeded_ = false; // force a fresh seed on next engage
            }

            // Ramp the blend weight toward its target (0 or 1). Ramping on engage too
            // (rather than snapping to 1.0 as the reference example does) is an extra
            // safety margin on top of position-matching at the moment of engagement.
            arm_weight_ = enabled
                              ? std::min(1.0f, arm_weight_ + delta_weight)
                              : std::max(0.0f, arm_weight_ - delta_weight);
            arm_cmd_msg_.motor_cmd().at(kWeightIndex).q(arm_weight_);

            if (enabled) {
                const std::array<float,17> target_all = pack_arm_targets();
                for (size_t j = 0; j < target_all.size(); ++j) {
                    current_jpos_des_.at(j) += std::clamp(
                        target_all.at(j) - current_jpos_des_.at(j),
                        -max_joint_delta, max_joint_delta);

                    auto& mc = arm_cmd_msg_.motor_cmd().at(kArmJoints.at(j));
                    mc.q(current_jpos_des_.at(j));
                    mc.dq(0.f);
                    mc.kp(60.f);
                    mc.kd(1.5f);
                    mc.tau(0.f);
                }
                unpack_arm_desired(current_jpos_des_);
            }
            // When disabling: q/kp/kd of the arm joints are deliberately left untouched
            // here, so arm_cmd_msg_ keeps holding its last commanded pose while only the
            // weight decays -- matching the official arm_sdk example's shutdown behavior.

            arm_sdk_publisher_->Write(arm_cmd_msg_);
        } catch (const std::exception& e) {
            std::cerr << "DriverUnitreeG1ArmSDK arm control loop error: " << e.what() << std::endl;
        }
    }

    void start_arm_control_thread(const double& period,
                                  const sas::ThreadManager::PRIORITY& priority)
    {
        if (arm_control_thread_ && arm_control_thread_->is_running()) {
            return; // Already running
        }
        arm_control_period_ = period;
        arm_control_thread_ = std::make_unique<sas::ThreadManager>(
            "g1_arm_sdk_control",
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

        if (arm_cmd_msg_.motor_cmd().at(kWeightIndex).q() <= 0.0f && arm_weight_ <= 0.0f) {
            return;
        }

        const float ramp_period = 0.02f;
        const float delta_weight = weight_rate_ * ramp_period;
        const auto sleep_time = std::chrono::milliseconds(static_cast<int>(ramp_period * 1000));

        while (arm_weight_ > 0.0f) {
            arm_weight_ = std::max(0.0f, arm_weight_ - delta_weight);
            arm_cmd_msg_.motor_cmd().at(kWeightIndex).q(arm_weight_);
            try {
                arm_sdk_publisher_->Write(arm_cmd_msg_);
            } catch (const std::exception& e) {
                std::cerr << "DriverUnitreeG1ArmSDK::disable_arm_control_blocking: " << e.what() << std::endl;
                break;
            }
            std::this_thread::sleep_for(sleep_time);
        }
        arm_cmd_msg_.motor_cmd().at(kWeightIndex).q(0.0f);
        try {
            arm_sdk_publisher_->Write(arm_cmd_msg_);
        } catch (const std::exception& e) {
            std::cerr << "DriverUnitreeG1ArmSDK::disable_arm_control_blocking: " << e.what() << std::endl;
        }
    }
};

/**
 * @brief Constructs the driver.
 * @param shutdown_signaler Shared sas::ShutdownSignaler (typically the same one a
 *        SIGINT handler calls shutdown() on), forwarded to Impl so the background arm
 *        control loop callback can poll should_shutdown() every tick.
 * @throws std::invalid_argument if shutdown_signaler is nullptr.
 * @note No hardware/DDS I/O happens here; see connect().
 */
DriverUnitreeG1ArmSDK::DriverUnitreeG1ArmSDK(const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler)
{
    if (shutdown_signaler == nullptr) {
        throw std::invalid_argument("DriverUnitreeG1ArmSDK: shutdown_signaler must not be nullptr");
    }
    impl_ = std::make_shared<Impl>(shutdown_signaler);
}

DriverUnitreeG1ArmSDK::~DriverUnitreeG1ArmSDK()
{
    deinitialize();
    disconnect();
}



void DriverUnitreeG1ArmSDK::connect()
{
    // Precondition: unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
    // must already have been called by the owning driver (DriverUnitreeG1), same as for
    // the aggregated DriverUnitreeLocoClient.
    impl_->arm_sdk_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
    impl_->arm_sdk_publisher_->InitChannel();

    impl_->lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicLowState));
    impl_->lowstate_subscriber_->InitChannel(
        std::bind(&DriverUnitreeG1ArmSDK::Impl::low_state_callback, impl_.get(), std::placeholders::_1), 1);

    impl_->is_connected_ = true;
}

void DriverUnitreeG1ArmSDK::initialize()
{
    if (!impl_->is_connected_) {
        throw std::runtime_error("DriverUnitreeG1ArmSDK::initialize: Cannot initialize: not connected");
    }
    impl_->is_initialized_ = true;
    if (!impl_->arm_control_thread_ || !impl_->arm_control_thread_->is_running()) {
        impl_->start_arm_control_thread(impl_->arm_control_period_, sas::ThreadManager::PRIORITY::NORMAL);
    }
}

void DriverUnitreeG1ArmSDK::deinitialize()
{
    if (impl_->is_initialized_) {
        impl_->disable_arm_control_blocking();
        impl_->stop_arm_control_thread();
        impl_->is_initialized_ = false;
    }
}

void DriverUnitreeG1ArmSDK::disconnect()
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

void DriverUnitreeG1ArmSDK::enable_arm_control()
{
    // Flips a flag only -- the control loop performs the actual weight ramp-up and
    // seeds the trajectory tracker from the measured pose on the next tick.
    std::scoped_lock lock(impl_->data_mutex_);
    impl_->arms_enabled_ = true;
}

void DriverUnitreeG1ArmSDK::disable_arm_control()
{
    // For a graceful, non-blocking disable during normal operation. deinitialize()
    // uses disable_arm_control_blocking() instead to guarantee the ramp completes.
    std::scoped_lock lock(impl_->data_mutex_);
    impl_->arms_enabled_ = false;
}

bool DriverUnitreeG1ArmSDK::is_arm_control_enabled() const
{
    return impl_->arms_enabled_;
}

void DriverUnitreeG1ArmSDK::set_left_arm_target_positions(const std::array<double,7>& target_positions)
{
    std::scoped_lock lock(impl_->data_mutex_);
    for (size_t i = 0; i < 7; ++i)
        impl_->left_arm_data_.target_positions_.at(i) = static_cast<float>(target_positions.at(i));
}

void DriverUnitreeG1ArmSDK::set_right_arm_target_positions(const std::array<double,7>& target_positions)
{
    std::scoped_lock lock(impl_->data_mutex_);
    for (size_t i = 0; i < 7; ++i)
        impl_->right_arm_data_.target_positions_.at(i) = static_cast<float>(target_positions.at(i));
}

void DriverUnitreeG1ArmSDK::set_waist_target_positions(const std::array<double,3>& target_positions)
{
    std::scoped_lock lock(impl_->data_mutex_);
    for (size_t i = 0; i < 3; ++i)
        impl_->waist_data_.target_positions_.at(i) = static_cast<float>(target_positions.at(i));
}

std::array<double,7> DriverUnitreeG1ArmSDK::get_left_arm_positions()
{
    std::scoped_lock lock(impl_->data_mutex_);
    std::array<double,7> out{};
    for (size_t i = 0; i < 7; ++i) out.at(i) = impl_->left_arm_data_.current_positions_.at(i);
    return out;
}

std::array<double,7> DriverUnitreeG1ArmSDK::get_right_arm_positions()
{
    std::scoped_lock lock(impl_->data_mutex_);
    std::array<double,7> out{};
    for (size_t i = 0; i < 7; ++i) out.at(i) = impl_->right_arm_data_.current_positions_.at(i);
    return out;
}

std::array<double,3> DriverUnitreeG1ArmSDK::get_waist_positions()
{
    std::scoped_lock lock(impl_->data_mutex_);
    std::array<double,3> out{};
    for (size_t i = 0; i < 3; ++i) out.at(i) = impl_->waist_data_.current_positions_.at(i);
    return out;
}

std::array<double,7> DriverUnitreeG1ArmSDK::get_left_arm_desired_positions()
{
    std::scoped_lock lock(impl_->data_mutex_);
    std::array<double,7> out{};
    for (size_t i = 0; i < 7; ++i) out.at(i) = impl_->left_arm_data_.desired_positions_.at(i);
    return out;
}

std::array<double,7> DriverUnitreeG1ArmSDK::get_right_arm_desired_positions()
{
    std::scoped_lock lock(impl_->data_mutex_);
    std::array<double,7> out{};
    for (size_t i = 0; i < 7; ++i) out.at(i) = impl_->right_arm_data_.desired_positions_.at(i);
    return out;
}

std::array<double,3> DriverUnitreeG1ArmSDK::get_waist_desired_positions()
{
    std::scoped_lock lock(impl_->data_mutex_);
    std::array<double,3> out{};
    for (size_t i = 0; i < 3; ++i) out.at(i) = impl_->waist_data_.desired_positions_.at(i);
    return out;
}
