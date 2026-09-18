/*
# (C) Copyright 2024-2025 Adorno-Lab software developments
#
#    This file is part of Adorno-lab.
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
*/

#include <unitree_drivers/DriverUnitreeLowState.h>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <stdexcept>
#include <variant>
#include <mutex>
#include <atomic>

// G1 (unitree_hg message set) publishes rt/lowstate under this name, same topic
// DriverUnitreeG1ArmSDK already subscribes to.
static const std::string kTopicLowStateG1 = "rt/lowstate";
// H1 (unitree_go message set, per this class's current ROBOT::H1 mapping): the
// Adorno-Lab team's own internal testing recalled using unitree_go::msg::dds_::LowState_
// on "rt/lf/lowstate" rather than "rt/lowstate" -- plausibly a legacy topic name tied
// to that generation's low-level state bridge -- though that testing was not done
// against real H1 hardware, so treat this as the best available lead rather than a
// confirmed topic name. Verify against your actual H1 hardware/firmware before
// relying on it; if it turns out wrong, or if the message-set ambiguity in the
// class-level @warning resolves the other way (i.e. H1 actually publishes
// unitree_hg::msg::dds_::LowState_), it would likely need to change to
// kTopicLowStateG1 instead.
static const std::string kTopicLowStateH1 = "rt/lf/lowstate";

namespace
{
/**
 * @brief Looks up the motor_state() indices for one limb (or the torso) of one
 *        robot type.
 *
 * @details Builds and returns a fresh std::vector<int> per call rather than
 *          returning a reference to a precomputed file-scope table, specifically to
 *          avoid non-local (namespace-scope) objects with non-trivial
 *          constructors/destructors -- flagged by static analysis (e.g. clazy's
 *          non-pod-global-static) because their initialization order relative to
 *          other translation units' globals isn't guaranteed, and they add
 *          dynamic-allocation cost to program startup/teardown regardless of
 *          whether they're ever used. This function is only ever called from the
 *          getters below (not a per-tick hot path), so the small per-call
 *          allocation (at most 7 ints) is negligible next to the mutex lock and
 *          std::visit already happening in the same call.
 *
 * @param robot Which robot's layout to use.
 * @param limb Which limb (or TORSO) to return indices for.
 * @return The motor_state() indices belonging to @p limb under @p robot 's layout,
 *         in physical joint order (see the class-level @note on limb layouts in the
 *         header for the exact indices, joint names, and their provenance).
 * @throws std::invalid_argument if @p limb doesn't match any known enumerator. This
 *         branch is currently unreachable given LIMB's five enumerators are all
 *         handled below, but it guards against a future LIMB enumerator being added
 *         without a corresponding case here.
 */
std::vector<int> limb_indices(const DriverUnitreeLowState::ROBOT& robot,
                              const DriverUnitreeLowState::LIMB& limb)
{
    using ROBOT = DriverUnitreeLowState::ROBOT;
    using LIMB = DriverUnitreeLowState::LIMB;

    if (robot == ROBOT::G1) {
        // G1 (unitree_hg, 35-slot motor_state array). Verified against
        // unitreerobotics/unitree_sdk2's example/g1/low_level/g1_ankle_swing_example.cpp
        // G1JointIndex enum, and cross-checked against this same index set already
        // baked into DriverUnitreeG1ArmSDK's own kArmJoints array.
        switch (limb) {
        case LIMB::LEFT_ARM:  return {15, 16, 17, 18, 19, 20, 21}; // ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw
        case LIMB::RIGHT_ARM: return {22, 23, 24, 25, 26, 27, 28}; // same 7 joints, right side
        case LIMB::LEFT_LEG:  return {0, 1, 2, 3, 4, 5};           // HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll
        case LIMB::RIGHT_LEG: return {6, 7, 8, 9, 10, 11};         // same 6 joints, right side
        case LIMB::TORSO:     return {12, 13, 14};                 // WaistYaw, WaistRoll, WaistPitch
        }
    } else { // ROBOT::H1
        // H1 (legacy generation, unitree_go/go2, 20-slot motor_state array --
        // matching this class's current ROBOT::H1 -> unitree_go::msg::dds_::LowState_
        // mapping; see the class-level @warning on the H1 message-set ambiguity).
        // Verified against unitreerobotics/unitree_sdk2's
        // example/h1/low_level/motors.hpp JointIndex enum (the header used by
        // example/h1/low_level/humanoid.hpp, which subscribes to the same message
        // type). Note the different joint counts vs G1: a single combined Ankle
        // joint per leg (5 joints, not 6), a single-joint torso/waist (1 joint, not
        // 3 -- no waist roll/pitch at all in this generation), and no wrist joints
        // at all (4 joints per arm, not 7).
        switch (limb) {
        case LIMB::LEFT_ARM:  return {16, 17, 18, 19};  // ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow
        case LIMB::RIGHT_ARM: return {12, 13, 14, 15};  // ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow
        case LIMB::LEFT_LEG:  return {7, 3, 4, 5, 10};  // HipYaw, HipRoll, HipPitch, Knee, Ankle
        case LIMB::RIGHT_LEG: return {8, 0, 1, 2, 11};  // HipYaw, HipRoll, HipPitch, Knee, Ankle
        case LIMB::TORSO:     return {6};               // WaistYaw
        }
    }
    throw std::invalid_argument("DriverUnitreeLowState: unknown LIMB");
}

} // namespace

/**
 * @brief Private implementation (pImpl) for DriverUnitreeLowState.
 *
 * @details Owns the active ChannelSubscriber (one of two possible concrete message
 *          types, selected at connect() time by ROBOT) and the most recently received
 *          rt/lowstate message, cached by value exactly as received -- mirroring
 *          DriverUnitreeG1ArmSDK's own latest_low_state_ caching style -- rather than
 *          eagerly parsed into separate fields on every callback. Every public getter
 *          parses the cached message on demand, under low_state_mutex_, via
 *          std::visit so the same parsing code compiles against both concrete message
 *          types (they share the same accessor names: motor_state(), imu_state(),
 *          q(), dq(), tau_est(), quaternion(), gyroscope(), accelerometer(), rpy()).
 */
class DriverUnitreeLowState::Impl
{
public:
    // Only one of these is ever populated, selected at connect() time by ROBOT. A
    // variant keeps the two concrete subscriber types distinct (ChannelSubscriber<T>
    // for two different message types T) while the rest of Impl's code stays
    // robot-agnostic via std::visit.
    std::variant<unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>,
                 unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>> lowstate_subscriber_;

    // The most recently received rt/lowstate message, cached by value exactly as
    // received (same style as DriverUnitreeG1ArmSDK::Impl::latest_low_state_), parsed
    // on demand by the public getters rather than eagerly on every callback.
    std::variant<unitree_hg::msg::dds_::LowState_, unitree_go::msg::dds_::LowState_> latest_low_state_;
    bool has_received_state_ = false;
    mutable std::mutex low_state_mutex_;

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_initialized_{false};

    // Not used to drive any active loop (see the class-level @note in the header);
    // kept only for constructor-signature consistency with the other sub-drivers
    // DriverUnitreeG1 aggregates.
    std::shared_ptr<sas::ShutdownSignaler> shutdown_signaler_;

    explicit Impl(const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler) : shutdown_signaler_{shutdown_signaler} {}

    /**
     * @brief DDS subscription callback: copies the received message into
     *        latest_low_state_ by value.
     * @tparam LowStateT Concrete message type for the active ROBOT (either
     *         unitree_hg::msg::dds_::LowState_ or unitree_go::msg::dds_::LowState_),
     *         fixed at connect() time via which specialization is bound to
     *         ChannelSubscriber::InitChannel().
     * @param msg Raw message pointer handed to us by the Unitree SDK, actually
     *        pointing to a LowStateT.
     */
    template <typename LowStateT>
    void low_state_callback(const void* msg)
    {
        std::scoped_lock lock(low_state_mutex_);
        latest_low_state_ = *static_cast<const LowStateT*>(msg);
        has_received_state_ = true;
    }

    /**
     * @brief Extracts one field from every joint of one limb into an Eigen::VectorXd.
     * @details A member of Impl (rather than a free function) because Impl is a
     *          private nested class: a free function cannot take a reference to it
     *          as a parameter type, even from within this same file.
     * @tparam FieldFn Callable of signature `float(const MotorState&)`, e.g. a
     *         lambda calling .q(), .dq(), or .tau_est().
     * @param robot Which robot's layout to use (forwarded to limb_indices()).
     * @param limb Which limb to extract.
     * @param field The accessor to apply to each of the limb's motors.
     * @return An Eigen::VectorXd sized to the limb's joint count, or empty (size 0)
     *         if no rt/lowstate message has been received yet.
     */
    template <typename FieldFn>
    Eigen::VectorXd extract_limb_field(const DriverUnitreeLowState::ROBOT& robot,
                                       const DriverUnitreeLowState::LIMB& limb,
                                       FieldFn&& field) const
    {
        const std::vector<int>& indices = limb_indices(robot, limb);
        std::scoped_lock lock(low_state_mutex_);
        if (!has_received_state_) {
            return Eigen::VectorXd(0);
        }
        Eigen::VectorXd out(static_cast<Eigen::Index>(indices.size()));
        std::visit([&](const auto& state) {
            const auto& motors = state.motor_state();
            for (std::size_t i = 0; i < indices.size(); ++i) {
                out(static_cast<Eigen::Index>(i)) =
                    static_cast<double>(field(motors.at(static_cast<std::size_t>(indices[i]))));
            }
        }, latest_low_state_);
        return out;
    }
};

/**
 * @brief Constructs the wrapper. No network I/O happens here; see connect().
 * @param shutdown_signaler Shared sas::ShutdownSignaler. Validated for consistency
 *        with the other sub-drivers but not used to drive any behavior in this class
 *        (see the class-level @note in the header).
 * @param robot_type Which rt/lowstate message type to subscribe to. Fixed for the
 *        lifetime of this object (robot_type_ is const) -- construct a new instance
 *        if you need to talk to a different robot type.
 * @throws std::invalid_argument if shutdown_signaler is nullptr.
 */
DriverUnitreeLowState::DriverUnitreeLowState(const std::shared_ptr<sas::ShutdownSignaler>& shutdown_signaler, const ROBOT& robot_type)
    : robot_type_{robot_type}
{
    if (shutdown_signaler == nullptr) {
        throw std::invalid_argument("DriverUnitreeLowState: shutdown_signaler must not be nullptr");
    }
    impl_ = std::make_shared<Impl>(shutdown_signaler);
}

/**
 * @brief Calls deinitialize() then disconnect() so destruction leaves the object in
 *        a clean, unsubscribed state regardless of what lifecycle stage it was in.
 */
DriverUnitreeLowState::~DriverUnitreeLowState()
{
    deinitialize();
    disconnect();
}

/**
 * @brief Subscribes to the rt/lowstate-equivalent topic with the message type
 *        matching robot_type_.
 * @throws std::invalid_argument if robot_type_ doesn't match any known enumerator.
 *         This branch is currently unreachable given ROBOT's two enumerators (G1,
 *         H1) are both handled below, but it guards against a future ROBOT
 *         enumerator being added without a corresponding case here.
 */
void DriverUnitreeLowState::connect()
{
    // Precondition: unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
    // must already have been called by the owning driver, same as for the aggregated
    // DriverUnitreeLocoClient and DriverUnitreeG1ArmSDK.
    //
    // Note the topic name itself is robot-specific, not just the message type: G1
    // publishes on "rt/lowstate", but H1 (per this class's current ROBOT::H1 mapping
    // to unitree_go::msg::dds_::LowState_) uses "rt/lf/lowstate" instead, per the
    // Adorno-Lab team's recollection from internal testing -- see kTopicLowStateH1's
    // comment above for the caveat that this wasn't verified against real H1
    // hardware.
    switch (robot_type_) {
    case ROBOT::G1:
    {
        unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> subscriber(
            new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicLowStateG1));
        subscriber->InitChannel(
            std::bind(&Impl::low_state_callback<unitree_hg::msg::dds_::LowState_>, impl_.get(),
                      std::placeholders::_1),
            1);
        impl_->lowstate_subscriber_ = subscriber;
        break;
    }
    case ROBOT::H1:
    {
        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> subscriber(
            new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(kTopicLowStateH1));
        subscriber->InitChannel(
            std::bind(&Impl::low_state_callback<unitree_go::msg::dds_::LowState_>, impl_.get(),
                      std::placeholders::_1),
            1);
        impl_->lowstate_subscriber_ = subscriber;
        break;
    }
    default:
        throw std::invalid_argument("DriverUnitreeLowState::connect: unknown RobotType");
    }
    impl_->is_connected_ = true;
}

/**
 * @brief Marks this object ready for use.
 * @details There is no background loop to start (see the class-level @note); this
 *          only flips a flag.
 * @throws std::runtime_error if connect() has not been called yet.
 */
void DriverUnitreeLowState::initialize()
{
    if (!impl_->is_connected_) {
        throw std::runtime_error("DriverUnitreeLowState::initialize: Cannot initialize: not connected");
    }
    impl_->is_initialized_ = true;
}

/**
 * @brief Marks this object as no longer initialized.
 * @details No blocking behavior: there is no control loop or publisher to ramp down
 *          (see the class-level @note).
 */
void DriverUnitreeLowState::deinitialize()
{
    impl_->is_initialized_ = false;
}

/**
 * @brief Unsubscribes from rt/lowstate.
 * @details No-op if not currently connected. Calls deinitialize() first if still
 *          initialized, then releases the active subscriber (whichever alternative
 *          of the variant is populated) via std::visit.
 */
void DriverUnitreeLowState::disconnect()
{
    if (impl_->is_connected_) {
        if (impl_->is_initialized_) {
            deinitialize();
        }
        std::visit([](auto& subscriber) { subscriber.reset(); }, impl_->lowstate_subscriber_);
        impl_->is_connected_ = false;
    }
}

/**
 * @brief Returns the number of joint slots in the underlying rt/lowstate message.
 * @return 35 for G1 (or, per the H1 message-set caveat above, whichever the active
 *         message reports), or 0 if no rt/lowstate message has been received yet.
 */
std::size_t DriverUnitreeLowState::num_joints() const
{
    std::scoped_lock lock(impl_->low_state_mutex_);
    if (!impl_->has_received_state_) {
        return 0;
    }
    return std::visit([](const auto& state) { return state.motor_state().size(); },
                      impl_->latest_low_state_);
}

/**
 * @brief Returns every joint's last measured position.
 * @return A std::vector<double> of joint positions, in radians, indexed as in the
 *         underlying LowState_::motor_state() array. Empty if no rt/lowstate
 *         message has been received yet.
 */
std::vector<double> DriverUnitreeLowState::get_joint_positions() const
{
    std::scoped_lock lock(impl_->low_state_mutex_);
    std::vector<double> out;
    if (!impl_->has_received_state_) {
        return out;
    }
    std::visit([&](const auto& state) {
        const auto& motors = state.motor_state();
        out.resize(motors.size());
        for (std::size_t i = 0; i < motors.size(); ++i) {
            out.at(i) = static_cast<double>(motors.at(i).q());
        }
    }, impl_->latest_low_state_);
    return out;
}

/**
 * @brief Returns every joint's last measured velocity.
 * @return A std::vector<double> of joint velocities, in rad/s, indexed as in the
 *         underlying LowState_::motor_state() array. Empty if no rt/lowstate
 *         message has been received yet.
 */
std::vector<double> DriverUnitreeLowState::get_joint_velocities() const
{
    std::scoped_lock lock(impl_->low_state_mutex_);
    std::vector<double> out;
    if (!impl_->has_received_state_) {
        return out;
    }
    std::visit([&](const auto& state) {
        const auto& motors = state.motor_state();
        out.resize(motors.size());
        for (std::size_t i = 0; i < motors.size(); ++i) {
            out.at(i) = static_cast<double>(motors.at(i).dq());
        }
    }, impl_->latest_low_state_);
    return out;
}

/**
 * @brief Returns every joint's last estimated torque.
 * @return A std::vector<double> of estimated joint torques, in Nm, indexed as in
 *         the underlying LowState_::motor_state() array. Empty if no rt/lowstate
 *         message has been received yet.
 */
std::vector<double> DriverUnitreeLowState::get_joint_torques() const
{
    std::scoped_lock lock(impl_->low_state_mutex_);
    std::vector<double> out;
    if (!impl_->has_received_state_) {
        return out;
    }
    std::visit([&](const auto& state) {
        const auto& motors = state.motor_state();
        out.resize(motors.size());
        for (std::size_t i = 0; i < motors.size(); ++i) {
            out.at(i) = static_cast<double>(motors.at(i).tau_est());
        }
    }, impl_->latest_low_state_);
    return out;
}

/**
 * @brief Returns the last received IMU reading.
 * @return An IMUData; its `valid` field is false if no rt/lowstate message has been
 *         received yet, in which case the rest of the fields are default-initialized
 *         (zeros).
 */
DriverUnitreeLowState::IMUData DriverUnitreeLowState::get_imu_data() const
{
    std::scoped_lock lock(impl_->low_state_mutex_);
    IMUData out;
    out.valid = impl_->has_received_state_;
    if (!impl_->has_received_state_) {
        return out;
    }
    std::visit([&](const auto& state) {
        const auto& imu = state.imu_state();
        for (int i = 0; i < 4; ++i) {
            out.quaternion.at(static_cast<std::size_t>(i)) = static_cast<double>(imu.quaternion().at(i));
        }
        for (int i = 0; i < 3; ++i) {
            out.gyroscope.at(static_cast<std::size_t>(i))     = static_cast<double>(imu.gyroscope().at(i));
            out.accelerometer.at(static_cast<std::size_t>(i)) = static_cast<double>(imu.accelerometer().at(i));
            out.rpy.at(static_cast<std::size_t>(i))           = static_cast<double>(imu.rpy().at(i));
        }
    }, impl_->latest_low_state_);
    return out;
}

// --- Per-limb (and torso) convenience getters ---

/**
 * @brief Returns the number of joints in the given limb (or TORSO) for this
 *        instance's robot type.
 * @param limb Which limb (or TORSO) to query.
 * @return The joint count for @p limb under robot_type_'s layout (e.g. 7 for G1's
 *         LEFT_ARM, 4 for H1's LEFT_ARM; 3 for G1's TORSO, 1 for H1's TORSO). This is
 *         a static property of (robot_type_, limb), looked up via limb_indices(), so
 *         unlike the getters below it doesn't require a message to have been
 *         received yet.
 */
std::size_t DriverUnitreeLowState::num_joints(const LIMB& limb) const
{
    return limb_indices(robot_type_, limb).size();
}

/**
 * @brief Returns the given limb's (or the torso's) last measured joint positions.
 * @param limb Which limb (or TORSO) to read.
 * @return An Eigen::VectorXd of joint positions, in radians, sized and ordered per
 *         the class-level @note on limb layouts for robot_type_. Empty (size 0) if
 *         no rt/lowstate message has been received yet.
 */
Eigen::VectorXd DriverUnitreeLowState::get_joint_positions(const LIMB& limb) const
{
    return impl_->extract_limb_field(robot_type_, limb,
                                     [](const auto& motor) { return motor.q(); });
}

/**
 * @brief Returns the given limb's (or the torso's) last measured joint velocities.
 * @param limb Which limb (or TORSO) to read.
 * @return An Eigen::VectorXd of joint velocities, in rad/s, sized and ordered per
 *         the class-level @note on limb layouts for robot_type_. Empty (size 0) if
 *         no rt/lowstate message has been received yet.
 */
Eigen::VectorXd DriverUnitreeLowState::get_joint_velocities(const LIMB& limb) const
{
    return impl_->extract_limb_field(robot_type_, limb,
                                     [](const auto& motor) { return motor.dq(); });
}

/**
 * @brief Returns the given limb's (or the torso's) last estimated joint torques.
 * @param limb Which limb (or TORSO) to read.
 * @return An Eigen::VectorXd of estimated joint torques, in Nm, sized and ordered
 *         per the class-level @note on limb layouts for robot_type_. Empty (size 0)
 *         if no rt/lowstate message has been received yet.
 */
Eigen::VectorXd DriverUnitreeLowState::get_joint_torques(const LIMB& limb) const
{
    return impl_->extract_limb_field(robot_type_, limb,
                                     [](const auto& motor) { return motor.tau_est(); });
}
