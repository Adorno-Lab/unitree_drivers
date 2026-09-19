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
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

/**
 * @brief Publishes to the Unitree SDK's low-level command topic (rt/lowcmd), letting a
 *        caller drive every joint's PD target (q, dq, kp, kd) and feedforward torque
 *        directly -- bypassing the robot's onboard locomotion controller and
 *        DriverUnitreeG1ArmSDK's weight-blend mechanism entirely -- behind a single,
 *        robot-agnostic interface.
 *
 * @details Selecting a ROBOT at construction time picks which concrete DDS message
 *          type rt/lowcmd is published as, mirroring DriverUnitreeLowState's own
 *          ROBOT-driven message selection for rt/lowstate (see that class's docs for
 *          the underlying message-type evidence, which this class reuses without
 *          re-verifying independently): G1 publishes it as
 *          unitree_hg::msg::dds_::LowCmd_ (35 motor_cmd() slots, matching
 *          DriverUnitreeLowState::ROBOT::G1's LowState_ slot count), and, per this
 *          class's current ROBOT::H1 mapping -- same caveat as
 *          DriverUnitreeLowState::ROBOT::H1: genuinely ambiguous in
 *          unitreerobotics/unitree_sdk2 itself, not just here; see that class's
 *          @warning on the H1 message-set split -- as unitree_go::msg::dds_::LowCmd_
 *          (20 motor_cmd() slots). Both message types are published on the same topic
 *          name, "rt/lowcmd" (per unitreerobotics/xr_teleoperate's own
 *          kTopicLowCommand_Debug, used identically for G1, H1-2, and H1), unlike
 *          rt/lowstate's read side, where DriverUnitreeLowState's ROBOT::H1 mapping
 *          subscribes to a different topic name ("rt/lf/lowstate") than G1's
 *          ("rt/lowstate"). If that H1 message-set ambiguity resolves the other way
 *          for your specific hardware/firmware, the ROBOT::H1 case in the .cpp would
 *          need to move to unitree_hg::msg::dds_::LowCmd_ instead -- exactly the same
 *          swap DriverUnitreeLowState's own docs describe for its ROBOT::H1 case.
 *
 * @warning Direct rt/lowcmd control bypasses every safety net the onboard
 *          locomotion controller and DriverUnitreeG1ArmSDK's weight-blend mechanism
 *          provide (balance, joint limits, gradual engagement). There is no ramp, no
 *          seeding from the measured pose, and no blend weight here: publish() sends
 *          exactly the joint commands most recently buffered, verbatim, the instant
 *          it is called. The caller is entirely responsible for commanding safe
 *          trajectories (starting from the robot's actual measured pose -- see
 *          DriverUnitreeLowState -- and rate-limiting any change), for driving
 *          publish() at an appropriate, steady control-loop rate, and for keeping the
 *          robot clear of people/obstacles. This class performs no safety checking of
 *          any kind beyond CRC computation and the shutdown check documented on
 *          publish().
 *
 * @warning Per the Unitree ROS2/SDK2 documentation and reference deployment scripts
 *          (e.g. unitree_rl_gym's real-robot deployment, which reads
 *          LowState_::mode_machine() once at startup and forwards it into every
 *          published LowCmd_ via an init_cmd_hg()-style helper), real G1 hardware
 *          additionally requires every published LowCmd_ to echo back the
 *          mode_machine value the robot itself reports on rt/lowstate -- commands
 *          published with a mismatched or default mode_machine are ignored by the
 *          firmware. set_mode_machine() exists for this purpose, but
 *          DriverUnitreeLowState does not currently expose a mode_machine() getter to
 *          source that value from, so the caller must currently obtain it some other
 *          way (e.g. reading LowState_::mode_machine() directly via the Unitree SDK,
 *          or a future revision of DriverUnitreeLowState adding a getter for it)
 *          before relying on this class against real G1 hardware. This requirement is
 *          specific to ROBOT::G1's unitree_hg message set; ROBOT::H1's
 *          unitree_go::msg::dds_::LowCmd_ has no mode_machine field at all, and
 *          set_mode_machine()/set_ankle_mode() are both no-ops for it -- see their
 *          own docs.
 *
 * @warning ROBOT::H1's unitree_go::msg::dds_::LowCmd_ has its own, different
 *          message-acceptance requirement in place of mode_machine: per
 *          unitreerobotics/unitree_sdk2's own example/go2/go2_stand_example.cpp
 *          InitLowCmd(), the message's head/level_flag/gpio framing fields must be
 *          set (head = {0xFE, 0xEF}, level_flag = 0xFF, gpio = 0) or the command is
 *          rejected/ignored. connect() already does this automatically as part of
 *          resetting the command buffer -- unlike mode_machine, there is nothing the
 *          caller needs to source from rt/lowstate or set explicitly for H1. This
 *          note exists only so the asymmetry between the two ROBOT values' hardware
 *          requirements is documented somewhere a reader of this header will see it.
 *
 * @note Unlike DriverUnitreeG1ArmSDK, this class has no background control loop and
 *       does not itself call ChannelPublisher::Write() on a timer: it only buffers
 *       whatever per-joint commands were most recently set via set_joint_command() /
 *       set_limb_command(), and publish() must be called explicitly, at whatever rate
 *       and from whatever thread the owning driver's own control loop runs on. This
 *       is entirely analogous to how DriverUnitreeLowState leaves reading the cached
 *       state to the caller rather than running its own loop, and is deliberate: this
 *       class is the low-level command counterpart intended to be paired with
 *       DriverUnitreeLowState by a future higher-level driver (e.g. for driving the
 *       Unitree MuJoCo simulator in low-level mode, which requires commanding the
 *       robot at the joint level rather than through the locomotion controller) --
 *       that future driver will own the control loop this class does not have.
 *
 * @note shutdown_signaler is accepted and validated at construction, and consulted by
 *       publish() (see its own docs) as a minimal safety measure, but this class has
 *       no background loop of its own to poll it from every tick the way
 *       DriverUnitreeG1ArmSDK's arm_control_loop_callback() does -- it is only ever
 *       checked at the moment publish() is called.
 *
 * @note This class assumes unitree::robot::ChannelFactory::Instance()->Init(domain_id,
 *       network_interface) has already been called by the owning driver before
 *       connect() is invoked, exactly like DriverUnitreeLocoClient,
 *       DriverUnitreeG1ArmSDK, and DriverUnitreeLowState.
 *
 * @note Limb layouts (set_limb_command(LIMB, ...) / num_joints(LIMB)): the
 *       motor_cmd() indices belonging to each LIMB, and how many joints each maps to,
 *       are identical to -- and must be kept in sync with -- DriverUnitreeLowState's
 *       own class-level @note on limb layouts; see that class for the exact indices,
 *       joint names, and their provenance. The index table is duplicated in this
 *       class's .cpp (rather than shared) purely so this class remains a standalone
 *       translation unit with no compile-time dependency on DriverUnitreeLowState;
 *       DriverUnitreeLowState.cpp's copy is the source of truth if the two ever need
 *       reconciling.
 */
class DriverUnitreeLowCmd
{
public:
    /**
     * @brief Identifies which concrete rt/lowcmd message type this instance
     *        publishes.
     * @note Mirrors DriverUnitreeLowState::ROBOT exactly. Other robots can be added
     *       in future versions.
     */
    enum class ROBOT{G1,H1}; // Other robots can be added in future versions

    /**
     * @brief Identifies one limb's (or the torso's) group of joints within the
     *        underlying rt/lowcmd motor_cmd() array.
     * @note Mirrors DriverUnitreeLowState::LIMB exactly; see the class-level @note on
     *       limb layouts for the exact indices and their provenance. TORSO refers to
     *       the waist joint(s) (there is no separate "head" or other body segment
     *       addressed by this enum).
     */
    enum class LIMB{LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, TORSO};

    /**
     * @brief Ankle control coordinate mode, i.e. unitree_hg::msg::dds_::LowCmd_'s
     *        mode_pr field.
     * @details PR: the ankle's pitch and roll axes are commanded independently. AB:
     *          the two physical actuators driving a parallel-linkage ankle are
     *          commanded directly (their combined motion produces pitch/roll, but the
     *          commanded quantities themselves are the linkage-space A/B values, not
     *          pitch/roll).
     * @note Only meaningful for ROBOT::G1; a no-op for ROBOT::H1, whose
     *       unitree_go::msg::dds_::LowCmd_ has no equivalent field -- see
     *       set_ankle_mode().
     */
    enum class ANKLE_MODE{PR, AB};

    /**
     * @brief One joint's full low-level command: PD target plus feedforward torque.
     * @note enable maps to the underlying motor_cmd().mode() field. Per the Unitree
     *       ROS2 documentation ("mode: 0x01 (Foc/Working mode), 0x00 (Stop/Standby
     *       mode)"), both message sets use the same 0/1 encoding, so this single
     *       bool covers both ROBOT values identically.
     */
    struct MotorCommand
    {
        double q = 0.0;       ///< Target position, rad.
        double dq = 0.0;      ///< Target velocity, rad/s.
        double tau = 0.0;     ///< Feedforward torque, Nm.
        double kp = 0.0;      ///< Position (proportional) gain.
        double kd = 0.0;      ///< Velocity (derivative) gain.
        bool enable = false;  ///< motor_cmd().mode(): true = 1 (enable/servo), false = 0 (disable). Defaults to false (disabled).
    };

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    const ROBOT robot_type_;

public:
    // Rule of five
    DriverUnitreeLowCmd()=delete;
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeLowCmd(const DriverUnitreeLowCmd&) = delete;
    DriverUnitreeLowCmd& operator=(const DriverUnitreeLowCmd&) = delete;
    DriverUnitreeLowCmd(DriverUnitreeLowCmd&&) = delete;
    DriverUnitreeLowCmd& operator=(DriverUnitreeLowCmd&&) = delete;

    /// @throws std::invalid_argument if shutdown_signaler is nullptr.
    explicit DriverUnitreeLowCmd(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler, const ROBOT& robot_type);
    ~DriverUnitreeLowCmd();

    /**
     * @brief Sets up the rt/lowcmd publisher with the message type matching
     *        robot_type_, and buffers an all-disabled, zeroed command (every joint's
     *        enable = false, q = dq = tau = kp = kd = 0) ready to publish() safely.
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

    /// Marks this object as no longer initialized. No blocking behavior and nothing
    /// is published: there is no control loop to ramp down (see the class-level
    /// @note) -- unlike DriverUnitreeG1ArmSDK::deinitialize(), this does not publish a
    /// disabling command. Callers that need the robot left in a known-safe state
    /// should explicitly buffer and publish() a disabled command before calling this.
    void deinitialize();

    /// Tears down the rt/lowcmd publisher. Calls deinitialize() first if still
    /// initialized.
    void disconnect();

    /// Number of motor_cmd() slots in the underlying rt/lowcmd message for this robot
    /// (35 for G1, 20 for H1 -- see the class-level @details).
    std::size_t num_joints() const;

    /// Number of joints in the given limb (or TORSO) for this instance's robot type
    /// (e.g. 7 for G1's LEFT_ARM, 4 for H1's LEFT_ARM; 3 for G1's TORSO, 1 for H1's
    /// TORSO). Mirrors DriverUnitreeLowState::num_joints(LIMB) const exactly. Does
    /// not require connect() to have been called yet -- this is a static property of
    /// (robot_type_, limb).
    std::size_t num_joints(const LIMB& limb) const;

    /**
     * @brief Buffers one joint's command by its absolute motor_cmd() index.
     * @param index Absolute index into the underlying motor_cmd() array (0-based; see
     *        DriverUnitreeLowState's class-level @note on limb layouts for how
     *        indices map to physical joints for this instance's robot type).
     * @param command The PD target, feedforward torque, and enable flag to buffer.
     * @throws std::out_of_range if index >= num_joints().
     * @note Only buffers the command in memory; publish() must be called afterward to
     *       actually send it. Thread-safe with respect to concurrent publish() and
     *       get_joint_command() calls.
     */
    void set_joint_command(std::size_t index, const MotorCommand& command);

    /**
     * @brief Buffers commands for every joint in one limb (or the torso) at once.
     * @param limb Which limb (or TORSO) to command.
     * @param commands One MotorCommand per joint in @p limb, in the same physical
     *        joint order as DriverUnitreeLowState's per-limb getters -- see that
     *        class's class-level @note on limb layouts.
     * @throws std::invalid_argument if commands.size() != num_joints(limb).
     */
    void set_limb_command(const LIMB& limb, const std::vector<MotorCommand>& commands);

    /**
     * @brief Returns the most recently buffered command for one joint.
     * @param index Absolute index into the underlying motor_cmd() array.
     * @return The MotorCommand last set via set_joint_command() / set_limb_command()
     *         for @p index (or the zeroed, disabled default if connect() has been
     *         called but that index has not been set since). Not necessarily what was
     *         last actually published -- see publish().
     * @throws std::out_of_range if index >= num_joints().
     */
    MotorCommand get_joint_command(std::size_t index) const;

    /**
     * @brief Sets the ankle control coordinate mode (unitree_hg's mode_pr field).
     * @param mode PR (independent Pitch/Roll) or AB (coupled linkage A/B). Buffered
     *        the same way as MotorCommand fields: takes effect on the next publish().
     * @note No-op for ROBOT::H1 -- see the class-level @details and ANKLE_MODE's own
     *       docs.
     */
    /**
     * @brief Buffers target positions for every joint in one limb (or the torso),
     *        touching only their q fields.
     * @param limb Which limb (or TORSO) to command.
     * @param positions Target position, in radians, for each joint in @p limb, in
     *        the same physical joint order as DriverUnitreeLowState's per-limb
     *        getters -- see that class's class-level @note on limb layouts.
     * @details Unlike set_limb_command(), this leaves each joint's dq/tau/kp/kd/
     *          enable fields untouched at whatever was most recently buffered for
     *          them (or their MotorCommand defaults, if never set) -- only q is
     *          overwritten. Use set_limb_command() instead when the other fields
     *          need setting too.
     * @throws std::invalid_argument if positions.size() != num_joints(limb).
     */
    void set_joint_positions(const LIMB& limb, const Eigen::VectorXd& positions);

    void set_ankle_mode(const ANKLE_MODE& mode);

    /**
     * @brief Sets the machine-identifier field real G1 hardware requires every
     *        published LowCmd_ to echo back (unitree_hg's mode_machine field).
     * @param mode_machine The value to echo, obtained from the robot's own
     *        LowState_::mode_machine() -- see the class-level @warning: this must be
     *        sourced from the live rt/lowstate stream, not guessed or left at its
     *        default of 0, or real G1 hardware will ignore published commands.
     * @note No-op for ROBOT::H1 -- see the class-level @details and @warning.
     */
    void set_mode_machine(std::uint8_t mode_machine);

    /**
     * @brief Computes the CRC over the buffered command and publishes it to rt/lowcmd
     *        verbatim, exactly as currently buffered.
     * @return false, without publishing anything, if
     *         shutdown_signaler->should_shutdown() is true -- a minimal safety
     *         measure, since this class has no background loop of its own to react to
     *         the signal otherwise (see the class-level @note). true if the publish
     *         call was made.
     * @throws std::runtime_error if connect()/initialize() have not been called yet.
     */
    bool publish();
};
