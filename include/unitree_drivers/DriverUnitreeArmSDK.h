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
#           - Documented with the assistance of Claude (Anthropic).
#
#   Acknowledgement: The H1 support in this class is based on the developments by
#                    Daniel S. J. Derwent (email: daniel.derwent@manchester.ac.uk)
#                    in https://github.com/Adorno-Lab/sas_robot_driver_unitree_h1
#
# ################################################################
*/

#pragma once
#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include  <marinholab/sas/core/sas_shutdown_signaler.hpp>

/**
 * @brief Controls the arms and waist of a Unitree G1 or H1 over the rt/arm_sdk DDS topic,
 *        blending with the robot's autonomous locomotion controller via a ramped weight
 *        parameter.
 *
 * @details Selecting a ROBOT at construction time picks the DDS message set, the
 *          state topic name, and the joint layout used internally; every public
 *          method then behaves identically for both robots. Both robots use the same
 *          mechanism: rt/arm_sdk carries per-joint PD targets for the arm/waist joints,
 *          plus a blend weight (0 = the onboard controller owns the arms, 1 = these
 *          commands own them) written into the q field of an otherwise unused motor slot.
 *          What differs between the two is listed below.
 *
 *          | | G1 | H1 |
 *          |---|---|---|
 *          | rt/arm_sdk message | unitree_hg::msg::dds_::LowCmd_ | unitree_go::msg::dds_::LowCmd_ |
 *          | state topic / message | rt/lowstate, unitree_hg::msg::dds_::LowState_ | rt/lf/lowstate, unitree_go::msg::dds_::LowState_ |
 *          | blend-weight motor slot | 29 | 9 |
 *          | joints per arm | 7 | 4 (ShoulderPitch/Roll/Yaw, Elbow; no wrist) |
 *          | waist joints | 3 (Yaw, Roll, Pitch) | 1 (Yaw) |
 *          | joint-limit clamping of targets | none | yes, per-joint limits with a 0.15 rad safety margin |
 *
 *          The H1 layout, topic names, message types and joint limits are taken from
 *          the Adorno-Lab sas_robot_driver_unitree_h1 driver (DriverUnitreeH1), which
 *          drives the H1's arms through rt/arm_sdk with the same weight mechanism, and
 *          from unitree_sdk2's example/h1/high_level/h1_arm_sdk_dds_example.cpp (which
 *          agrees on the motor indices and the publishing message type).
 *
 *          Because the joint counts differ, the number of positions accepted/returned
 *          per limb depends on the robot: use get_num_joints() to query it. The
 *          setters throw std::invalid_argument on a size mismatch.
 *
 *          Default PD gains (kp / kd), unless changed per joint with set_gains(); gain
 *          changes are ramped, so they can be tuned while engaged:
 *
 *          | | G1 | H1 |
 *          |---|---|---|
 *          | shoulders, elbow | 80 / 3 | 60 / 1.5 |
 *          | wrists (roll, pitch, yaw) | 40 / 1.5 | -- |
 *          | waist | 300 / 3 | 60 / 1.5 |
 *
 *          The G1 values are those of Unitree's G1 teleoperation (xr_teleoperate,
 *          teleop/robot_control/robot_arm.py), the H1 values those of unitree_sdk2's
 *          h1_arm_sdk_dds_example.cpp.
 *
 * @warning enable_arm_control() ramps the blend weight up gradually rather than
 *          snapping to 1.0, and seeds the internal trajectory tracker from the
 *          currently measured joint positions before ramping, so engaging never
 *          commands a sudden jump. disable_arm_control() ramps the weight back down
 *          to 0 while holding the last commanded pose. Even so, always keep the robot
 *          clear of obstacles/people while engaging or disengaging arm control.
 *          Engaging is refused (nothing is published) until at least one state message
 *          has been received, so the tracker is never seeded from an all-zero pose.
 *          A limb that has not been given a target via set_target_positions() holds
 *          its measured pose on engage, rather than tracking toward the all-zero
 *          initial target; it keeps doing so on every later engage until a target is
 *          set for it. On H1 the held pose is clamped to the joint limits like any
 *          other target, so a joint resting inside the 0.15 rad safety margin is moved
 *          to the edge of that margin (at the tracker's 0.5 rad/s rate).
 *
 * @note Both robots expose rt/arm_sdk and the weight-blend mechanism (see the table
 *       above). The H1 arm controller in Unitree's xr_teleoperate publishes to rt/lowcmd
 *       instead, but that is a different (and unblended) path; this class does not use
 *       it. For that path see DriverUnitreeLowCmd.
 *
 * @note Per the G1 SDK reference documentation: arm control (weight > 0) and
 *       high-level velocity commands (LocoClient::Move / DriverUnitreeLocoClient's
 *       target-velocity control loop) cannot both actively drive the robot at the same
 *       time while in Running/walking mode -- engaging arm control at weight 1.0 will
 *       make the robot stop responding to velocity commands, and it resumes responding
 *       to them once the weight is ramped back down to 0. Coordinate enabling/disabling
 *       this controller with whatever is driving DriverUnitreeLocoClient in the owning
 *       driver (DriverUnitreeG1 or DriverUnitreeH1). This has not been re-verified for
 *       the H1.
 *
 * @note Signal-driven shutdown: the caller passes a shared sas::ShutdownSignaler
 *       (typically the same one a SIGINT handler calls shutdown() on) at construction
 *       time. The background arm control loop callback polls
 *       shutdown_signaler->should_shutdown() every tick and, the moment it becomes
 *       true, sets arms_enabled_ to false -- reusing the same weight-ramp-to-zero
 *       logic disable_arm_control() triggers -- so the arm starts disengaging within
 *       one control_period (20 ms by default; see the constructor's control_period
 *       argument) of the signal, without waiting for the owning application to notice
 *       and call disable_arm_control()/deinitialize() itself. The callback
 *       deliberately does NOT call deinitialize() (which stops the control thread the
 *       callback itself runs on -- a self-join deadlock); thread teardown happens later
 *       from the main thread via the destructor or an explicit
 *       deinitialize()/disconnect().
 */
class DriverUnitreeArmSDK
{
public:
    /// Identifies which robot's message set, topics and joint layout this instance uses.
    enum class ROBOT{G1,H1};

    /// The joint groups this class controls.
    enum class LIMB{LEFT_ARM, RIGHT_ARM, WAIST};

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    const ROBOT robot_type_;

public:
    // Rule of five
    /// Constructs a controller for the given robot.
    /// @throws std::invalid_argument if shutdown_signaler is nullptr or robot_type is unknown.
    DriverUnitreeArmSDK(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                          const ROBOT& robot_type,
                          const double& control_period = 0.02);
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeArmSDK(const DriverUnitreeArmSDK&) = delete;
    DriverUnitreeArmSDK& operator=(const DriverUnitreeArmSDK&) = delete;
    DriverUnitreeArmSDK(DriverUnitreeArmSDK&&) = delete;
    DriverUnitreeArmSDK& operator=(DriverUnitreeArmSDK&&) = delete;

    ~DriverUnitreeArmSDK();

    /**
     * @brief Sets up the rt/arm_sdk publisher and the robot's state subscriber
     *        (rt/lowstate on G1, rt/lf/lowstate on H1).
     * @pre unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
     *      must already have been called by the owning driver (see
     *      DriverUnitreeLocoClient::connect() for the equivalent precondition there;
     *      both sub-controllers share the same process-wide DDS channel factory).
     */
    void connect();

    /**
     * @brief Starts the background control thread that runs the weight ramp and
     *        joint-tracking loop.
     * @throws std::runtime_error if connect() has not been called yet.
     */
    void initialize();

    /**
     * @brief Synchronously ramps the blend weight from its current value down to 0,
     *        holding the last commanded pose throughout, then stops the control thread.
     * @note Blocking by design: unlike disable_arm_control(), this guarantees the ramp
     *       fully completes before returning, so it's safe to call during shutdown.
     */
    void deinitialize();

    /**
     * @brief Tears down the rt/arm_sdk publisher and state subscriber.
     */
    void disconnect();

    /**
     * @brief Requests the background control loop to begin ramping the blend weight
     *        up toward 1.0 and start tracking target positions.
     * @note Non-blocking: only flips a flag. The actual ramp and trajectory seeding
     *       happen on the background thread. Use deinitialize() instead if you need a
     *       blocking guarantee that the ramp has completed (e.g. during shutdown).
     */
    void enable_arm_control();

    /**
     * @brief Requests the background control loop to begin ramping the blend weight
     *        back down toward 0, holding the last commanded pose.
     * @note Non-blocking; see enable_arm_control().
     */
    void disable_arm_control();

    /**
     * @brief Whether arm control is currently requested to be engaged.
     * @return True if enable_arm_control() was called more recently than
     *         disable_arm_control()/deinitialize(). Does not reflect the current blend
     *         weight itself, only which direction it's being ramped toward.
     */
    bool is_arm_control_enabled() const;

    /// The robot this instance was constructed for.
    ROBOT get_robot_type() const;

    /// Number of joints in @p limb for this robot (G1: 7/7/3, H1: 4/4/1 for left arm/right arm/waist).
    std::size_t get_num_joints(const LIMB& limb) const;

    /**
     * @brief Sets the target joint positions of @p limb, in radians.
     * @param limb Which joint group to set.
     * @param target_positions Exactly get_num_joints(limb) positions, in physical joint
     *        order (arms: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow[, WristRoll,
     *        WristPitch, WristYaw on G1]; waist: Yaw[, Roll, Pitch on G1]).
     * @throws std::invalid_argument if the size does not match get_num_joints(limb).
     * @note Until this is called for a limb, that limb holds its measured pose while
     *       arm control is engaged (see the class-level warning).
     * @note On H1, targets are clamped to the joint limits (with a safety margin)
     *       before being tracked, so get_desired_positions() converges to the clamped
     *       value, not the requested one.
     */
    void set_target_positions(const LIMB& limb, const std::vector<double>& target_positions);
    /// Returns @p limb's measured joint positions, in radians (get_num_joints(limb) values).
    std::vector<double> get_positions(const LIMB& limb);
    /// Returns @p limb's commanded trajectory-point positions, in radians (get_num_joints(limb) values).
    std::vector<double> get_desired_positions(const LIMB& limb);

    /**
     * @brief Sets the PD gains of every joint of @p limb.
     * @param limb Which joint group to set.
     * @param kp Exactly get_num_joints(limb) position gains, in Nm/rad, in the same joint
     *        order as set_target_positions().
     * @param kd Exactly get_num_joints(limb) velocity gains, in Nm·s/rad, same order.
     * @throws std::invalid_argument if a size does not match get_num_joints(limb), or a
     *         gain is not finite, negative, or above the sanity bound (kp <= 500,
     *         kd <= 20). The bounds only guard against typos; they are not the motors'
     *         torque limits.
     * @note The defaults depend on the robot; see the class-level table.
     * @note Safe to call while arm control is engaged: the gains actually published
     *       (see get_desired_kp()/get_desired_kd()) ramp toward the new values at
     *       250 (Nm/rad)/s for kp and 2.5 (Nm·s/rad)/s for kd, so a change never steps the
     *       joint torque. The ramp only advances while arm control is engaged.
     */
    void set_gains(const LIMB& limb, const std::vector<double>& kp, const std::vector<double>& kd);
    /// Returns @p limb's requested position gains (the last set_gains() values), in Nm/rad.
    std::vector<double> get_target_kp(const LIMB& limb);
    /// Returns @p limb's requested velocity gains (the last set_gains() values), in Nm·s/rad.
    std::vector<double> get_target_kd(const LIMB& limb);
    /// Returns @p limb's position gains currently being published (ramping toward get_target_kp()), in Nm/rad.
    std::vector<double> get_desired_kp(const LIMB& limb);
    /// Returns @p limb's velocity gains currently being published (ramping toward get_target_kd()), in Nm·s/rad.
    std::vector<double> get_desired_kd(const LIMB& limb);
};
