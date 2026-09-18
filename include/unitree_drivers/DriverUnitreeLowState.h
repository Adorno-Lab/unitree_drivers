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

#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Dense>

/**
 * @brief Subscribes to the Unitree SDK's low-level state topic (rt/lowstate) and
 *        exposes every joint's measured state and the onboard IMU reading behind a
 *        single, robot-agnostic interface.
 *
 * @details Selecting a ROBOT at construction time picks which concrete DDS message
 *          type rt/lowstate is subscribed to: G1 publishes it as
 *          unitree_hg::msg::dds_::LowState_ (confirmed against
 *          unitreerobotics/unitree_sdk2's own example/g1/low_level/*.cpp, all of
 *          which include <unitree/idl/hg/LowState_.hpp>). The two message types have
 *          differently-sized motor_state arrays (35 slots for the HG series vs 20 for
 *          the GO2 series, per include/unitree/idl/{hg,go2}/LowState_.hpp in that
 *          same repo) and are otherwise field-compatible
 *          (motor_state()[i].q()/dq()/tau_est(),
 *          imu_state().quaternion()/gyroscope()/accelerometer()/rpy() -- verified
 *          identical across both headers), so joint data is returned as
 *          std::vector<double> sized to whichever message is active, rather than a
 *          fixed-size std::array -- mirroring DriverUnitreeLocoClient::get_phase().
 *
 * @warning H1's rt/lowstate message type is genuinely ambiguous in
 *          unitreerobotics/unitree_sdk2 itself, not just in this class:
 *          example/h1/low_level/humanoid.hpp subscribes to rt/lowstate as
 *          unitree_go::msg::dds_::LowState_ (matching this class's current
 *          ROBOT::H1 mapping, and matching DriverUnitreeG1ArmSDK's own documented
 *          split for rt/lowcmd), while example/h1/high_level/h1_arm_sdk_dds_example.cpp,
 *          h1_27dof_example.cpp, and h1_2_ankle_track.cpp -- despite none but the
 *          last being named "h1_2" -- all subscribe to rt/lowstate as
 *          unitree_hg::msg::dds_::LowState_ instead, identically to G1. This almost
 *          certainly reflects the same firmware/hardware-generation split
 *          DriverUnitreeLocoClient's class docs warn about for FSM IDs: which
 *          message set a given physical H1 unit actually publishes on rt/lowstate
 *          depends on its firmware, not just its ROBOT enumerator. Confirm against
 *          the actual H1 hardware and firmware you're targeting before trusting
 *          ROBOT::H1 readings from this class; if it turns out wrong for your unit,
 *          swap the ROBOT::H1 case in the .cpp to unitree_hg::msg::dds_::LowState_
 *          instead.
 *
 * @warning The topic name for ROBOT::H1 is a second, independent source of
 *          uncertainty on top of the message-type ambiguity above: this class
 *          subscribes on "rt/lf/lowstate" for H1 rather than "rt/lowstate" (see
 *          kTopicLowStateH1 in the .cpp), based on the Adorno-Lab team's
 *          recollection from internal testing that was not done against real H1
 *          hardware. Treat it as the best available lead, not a confirmed topic
 *          name, and verify it alongside the message-type question above.
 *
 * @note Unlike DriverUnitreeLocoClient and DriverUnitreeG1ArmSDK, this class has no
 *       background control loop: it only ever reads rt/lowstate, so there is nothing
 *       for it to publish or ramp down. The latest received message is cached and
 *       parsed on demand by the getters below, entirely on whichever thread the
 *       Unitree SDK invokes the DDS subscription callback on. st_break_loops is
 *       still accepted and validated at construction for interface consistency with
 *       the other sub-drivers DriverUnitreeG1 aggregates (so it can be constructed
 *       and forwarded the same way), but it currently drives no behavior here.
 *
 * @note This class assumes unitree::robot::ChannelFactory::Instance()->Init(domain_id,
 *       network_interface) has already been called by the owning driver before connect()
 *       is invoked, exactly like DriverUnitreeLocoClient and DriverUnitreeG1ArmSDK.
 *
 * @note Limb layouts (get_joint_positions(LIMB)/get_joint_velocities(LIMB)/
 *       get_joint_torques(LIMB)): which motor_state() indices belong to each LIMB
 *       (arms, legs, or TORSO -- the waist), and how many joints each maps to,
 *       differs by ROBOT and is looked up internally at call time from robot_type_
 *       -- callers just pass a LIMB and get back a correctly-sized Eigen::VectorXd
 *       for whichever robot this instance was constructed for.
 *         - ROBOT::G1: LEFT_LEG = indices [0..5] (6 joints: HipPitch, HipRoll,
 *           HipYaw, Knee, AnklePitch, AnkleRoll), RIGHT_LEG = indices [6..11] (same 6
 *           joints, right side), TORSO = indices [12..14] (3 joints: WaistYaw,
 *           WaistRoll, WaistPitch -- WaistRoll/WaistPitch are reported invalid on
 *           G1 hardware variants with the waist locked, per the SDK's own
 *           G1JointIndex comments; this class still returns all 3 slots as reported
 *           by the message), LEFT_ARM = indices [15..21] (7 joints: ShoulderPitch,
 *           ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw),
 *           RIGHT_ARM = indices [22..28] (same 7, right side). Verified against
 *           unitreerobotics/unitree_sdk2's own
 *           example/g1/low_level/g1_ankle_swing_example.cpp G1JointIndex enum, and
 *           cross-checked against this same index set already baked into
 *           DriverUnitreeG1ArmSDK's own kArmJoints array (whose ordering is
 *           left-arm(7) + right-arm(7) + waist(3), i.e. this same TORSO index set).
 *         - ROBOT::H1: this class's ROBOT::H1 subscribes to rt/lowstate as
 *           unitree_go::msg::dds_::LowState_ (see the @warning above), so the limb
 *           indices used here match that same generation's layout, verified against
 *           unitreerobotics/unitree_sdk2's example/h1/low_level/motors.hpp JointIndex
 *           enum (the header used by example/h1/low_level/humanoid.hpp, which
 *           subscribes to the same message type). This generation has a different
 *           joint count per limb than G1: LEFT_LEG = {HipYaw, HipRoll, HipPitch,
 *           Knee, Ankle} (5 joints -- a single combined Ankle joint, not separate
 *           pitch/roll), RIGHT_LEG = same 5 joints on the right, TORSO = {WaistYaw}
 *           (a single joint -- this generation has no waist roll/pitch at all),
 *           LEFT_ARM = {ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow} (4 joints
 *           -- no wrist joints at all in this generation), RIGHT_ARM = same 4 joints
 *           on the right. If the message-set ambiguity in the @warning above turns
 *           out to resolve the other way for your specific H1 hardware/firmware
 *           (i.e. it actually publishes unitree_hg::msg::dds_::LowState_), these
 *           indices would need to change too -- to the H1-2/27-DoF layout in
 *           example/h1/low_level/h1_27dof_example.cpp's H1JointIndex enum instead
 *           (6-joint legs, 1-joint TORSO/waist at a different index, 7-joint arms
 *           starting at index 13), which is a different joint count per limb again.
 */
class DriverUnitreeLowState
{
public:
    /**
     * @brief Identifies which concrete rt/lowstate message type this instance
     *        subscribes to.
     * @note Other robots can be added in future versions.
     */
    enum class ROBOT{G1,H1}; // Other robots can be added in future versions

    /**
     * @brief Identifies one limb's (or the torso's) group of joints within the
     *        underlying rt/lowstate motor_state() array.
     *
     * @note Which motor_state() indices -- and how many joints -- each entry maps to
     *       is robot-specific and is looked up internally from robot_type_; see the
     *       class-level @note on limb layouts for the exact indices and their
     *       provenance. TORSO refers to the waist joint(s) (there is no separate
     *       "head" or other body segment addressed by this enum).
     */
    enum class LIMB{LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, TORSO};

    /**
     * @brief One IMU reading, as published in LowState_::imu_state().
     */
    struct IMUData
    {
        std::array<double,4> quaternion{};     ///< Orientation quaternion (w, x, y, z).
        std::array<double,3> gyroscope{};      ///< Angular velocity (x, y, z), rad/s.
        std::array<double,3> accelerometer{};  ///< Linear acceleration (x, y, z), m/s^2.
        std::array<double,3> rpy{};            ///< Roll, pitch, yaw, rad.
        bool valid = false;                    ///< false until the first rt/lowstate message has been received.
    };

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    const ROBOT robot_type_;

public:
    // Rule of five
    DriverUnitreeLowState()=delete;
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeLowState(const DriverUnitreeLowState&) = delete;
    DriverUnitreeLowState& operator=(const DriverUnitreeLowState&) = delete;
    DriverUnitreeLowState(DriverUnitreeLowState&&) = delete;
    DriverUnitreeLowState& operator=(DriverUnitreeLowState&&) = delete;

    /// @throws std::invalid_argument if st_break_loops is nullptr.
    explicit DriverUnitreeLowState(std::atomic_bool* st_break_loops, const ROBOT& robot_type);
    ~DriverUnitreeLowState();

    /**
     * @brief Subscribes to rt/lowstate.
     * @pre unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
     *      must already have been called by the owning driver.
     */
    void connect();

    /**
     * @brief Marks this object ready for use.
     * @details There is no background loop to start (see the class-level @note); this
     *          only flips a flag, mainly so this class's lifecycle shape matches the
     *          other sub-drivers DriverUnitreeG1 aggregates.
     * @throws std::runtime_error if connect() has not been called yet.
     */
    void initialize();

    /// Marks this object as no longer initialized. No blocking behavior: there is no
    /// control loop or publisher to ramp down (see the class-level @note).
    void deinitialize();

    /// Unsubscribes from rt/lowstate. Calls deinitialize() first if still initialized.
    void disconnect();

    /// Number of joint slots in the underlying rt/lowstate message for this robot
    /// (35 for G1/H1-2, 20 for the original H1). 0 if no message has been received yet.
    std::size_t num_joints() const;

    /// Returns every joint's last measured position, in radians, indexed as in the
    /// underlying LowState_::motor_state() array. Empty if no message has been
    /// received yet.
    std::vector<double> get_joint_positions() const;
    /// Returns every joint's last measured velocity, in rad/s. Empty if no message
    /// has been received yet.
    std::vector<double> get_joint_velocities() const;
    /// Returns every joint's last estimated torque, in Nm. Empty if no message has
    /// been received yet.
    std::vector<double> get_joint_torques() const;

    /// Returns the last received IMU reading. IMUData::valid is false if no message
    /// has been received yet.
    IMUData get_imu_data() const;

    // --- Per-limb (and torso) convenience getters ---
    // Joint count and indexing within motor_state() are robot-specific; see the
    // class-level @note on limb layouts. All three return an empty (size-0)
    // Eigen::VectorXd if no rt/lowstate message has been received yet, for
    // consistency with the full-body getters above.

    /// Number of joints in the given limb (or TORSO) for this instance's robot type
    /// (e.g. 7 for G1's LEFT_ARM, 4 for H1's LEFT_ARM; 3 for G1's TORSO, 1 for H1's
    /// TORSO). Does not require a message to have been received yet -- this is a
    /// static property of (robot_type_, limb).
    std::size_t num_joints(const LIMB& limb) const;
    /// Returns the given limb's (or the torso's) last measured joint positions, in
    /// radians, ordered as documented in the class-level @note on limb layouts.
    Eigen::VectorXd get_joint_positions(const LIMB& limb) const;
    /// Returns the given limb's (or the torso's) last measured joint velocities, in rad/s.
    Eigen::VectorXd get_joint_velocities(const LIMB& limb) const;
    /// Returns the given limb's (or the torso's) last estimated joint torques, in Nm.
    Eigen::VectorXd get_joint_torques(const LIMB& limb) const;
};
