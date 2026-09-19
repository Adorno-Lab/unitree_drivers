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

#include <unitree_drivers/DriverUnitreeLowCmd.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <stdexcept>
#include <variant>
#include <mutex>
#include <atomic>
#include <cstring>

// Same topic name for both message sets -- per unitreerobotics/xr_teleoperate's own
// kTopicLowCommand_Debug ("rt/lowcmd"), used identically for G1, H1-2, and H1 -- unlike
// rt/lowstate's read side, where DriverUnitreeLowState's H1 mapping uses a different
// topic name than G1's. See the class-level @details for the message-type mapping this
// mirrors from DriverUnitreeLowState.
static const std::string kTopicLowCmd = "rt/lowcmd";

namespace
{
/**
 * @brief CRC32 over a LowCmd_-shaped message, bit-for-bit identical to the algorithm
 *        published in Unitree's own examples (e.g. go2_stand_example.cpp's
 *        crc32_core(), and the equivalent Crc32Core() in the g1 low-level examples).
 * @details Both unitree_hg::msg::dds_::LowCmd_ and unitree_go::msg::dds_::LowCmd_ are
 *          POD-layout DDS messages whose last four bytes are the crc() field itself;
 *          the checksum covers every preceding 32-bit word, exactly as in the
 *          reference examples (`crc32_core((uint32_t*)&low_cmd,
 *          (sizeof(LowCmd_)>>2)-1)`).
 * @param ptr Pointer to the message, reinterpreted as an array of 32-bit words.
 * @param len Number of 32-bit words to checksum (message size in bytes, divided by 4,
 *        minus 1 to exclude the trailing crc() word itself).
 * @return The computed CRC32.
 */
std::uint32_t crc32_core(const std::uint32_t* ptr, std::uint32_t len)
{
    std::uint32_t xbit = 0;
    std::uint32_t data = 0;
    std::uint32_t CRC32 = 0xFFFFFFFF;
    const std::uint32_t dwPolynomial = 0x04c11db7;
    for (std::uint32_t i = 0; i < len; ++i) {
        xbit = 1u << 31;
        data = ptr[i];
        for (std::uint32_t bits = 0; bits < 32; ++bits) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit) {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }
    return CRC32;
}

/**
 * @brief Looks up the motor_cmd() indices for one limb (or the torso) of one robot
 *        type.
 * @details Identical index tables to DriverUnitreeLowState.cpp's own (private,
 *          anonymous-namespace) limb_indices() -- duplicated rather than shared so
 *          this class has no compile-time dependency on DriverUnitreeLowState; see the
 *          header's class-level @note. Built fresh per call rather than a precomputed
 *          file-scope table, for the same non-local-static reasoning given in
 *          DriverUnitreeLowState.cpp's own copy of this function.
 * @param robot Which robot's layout to use.
 * @param limb Which limb (or TORSO) to return indices for.
 * @return The motor_cmd() indices belonging to @p limb under @p robot 's layout, in
 *         physical joint order (see DriverUnitreeLowState's class-level @note on limb
 *         layouts for the exact indices, joint names, and their provenance).
 * @throws std::invalid_argument if @p limb doesn't match any known enumerator. This
 *         branch is currently unreachable given LIMB's five enumerators are all
 *         handled below, but it guards against a future LIMB enumerator being added
 *         without a corresponding case here.
 */
std::vector<int> limb_indices(const DriverUnitreeLowCmd::ROBOT& robot,
                              const DriverUnitreeLowCmd::LIMB& limb)
{
    using ROBOT = DriverUnitreeLowCmd::ROBOT;
    using LIMB = DriverUnitreeLowCmd::LIMB;

    if (robot == ROBOT::G1) {
        // G1 (unitree_hg, 35-slot motor_cmd array). Same indices as
        // DriverUnitreeLowState.cpp's G1 table; see that file for provenance.
        switch (limb) {
        case LIMB::LEFT_ARM:  return {15, 16, 17, 18, 19, 20, 21}; // ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw
        case LIMB::RIGHT_ARM: return {22, 23, 24, 25, 26, 27, 28}; // same 7 joints, right side
        case LIMB::LEFT_LEG:  return {0, 1, 2, 3, 4, 5};           // HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll
        case LIMB::RIGHT_LEG: return {6, 7, 8, 9, 10, 11};         // same 6 joints, right side
        case LIMB::TORSO:     return {12, 13, 14};                 // WaistYaw, WaistRoll, WaistPitch
        }
    } else { // ROBOT::H1
        // H1 (legacy generation, unitree_go/go2, 20-slot motor_cmd array). Same
        // indices as DriverUnitreeLowState.cpp's H1 table; see that file for
        // provenance and the class-level @warning on the H1 message-set ambiguity.
        switch (limb) {
        case LIMB::LEFT_ARM:  return {16, 17, 18, 19};  // ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow
        case LIMB::RIGHT_ARM: return {12, 13, 14, 15};  // ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow
        case LIMB::LEFT_LEG:  return {7, 3, 4, 5, 10};  // HipYaw, HipRoll, HipPitch, Knee, Ankle
        case LIMB::RIGHT_LEG: return {8, 0, 1, 2, 11};  // HipYaw, HipRoll, HipPitch, Knee, Ankle
        case LIMB::TORSO:     return {6};               // WaistYaw
        }
    }
    throw std::invalid_argument("DriverUnitreeLowCmd: unknown LIMB");
}

} // namespace

/**
 * @brief Private implementation (pImpl) for DriverUnitreeLowCmd.
 *
 * @details Owns the active ChannelPublisher (one of two possible concrete message
 *          types, selected at connect() time by ROBOT) and the buffered command
 *          message, mirroring DriverUnitreeLowState::Impl's variant-based approach to
 *          keep a single robot-agnostic code path over the two concrete message types
 *          via std::visit. Unlike DriverUnitreeLowState, there is no background
 *          thread and no callback: buffered_cmd_ is only ever written by
 *          set_joint_command()/set_limb_command()/set_ankle_mode()/set_mode_machine()
 *          and read by publish()/get_joint_command(), all under command_mutex_.
 */
class DriverUnitreeLowCmd::Impl
{
public:
    std::variant<unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>,
                 unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>> lowcmd_publisher_;

    // The buffered command, cached by value and mutated in place by the setters below,
    // published verbatim by publish(). Only one alternative is ever populated, chosen
    // at connect() time by ROBOT (mirrors DriverUnitreeLowState::Impl's
    // latest_low_state_ variant).
    std::variant<unitree_hg::msg::dds_::LowCmd_, unitree_go::msg::dds_::LowCmd_> buffered_cmd_;
    mutable std::mutex command_mutex_;

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_initialized_{false};

    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler_; ///< Checked once per publish() call; see the class-level @note.

    explicit Impl(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler)
        : shutdown_signaler_{shutdown_signaler} {}

    /**
     * @brief Resets buffered_cmd_ to the message type matching @p robot, with every
     *        motor_cmd().mode() left at 0 (disabled) and every PD/torque field at 0.
     * @details std::variant value-initializes each alternative's underlying struct to
     *          all-zero on construction (both LowCmd_ types are POD-layout DDS
     *          messages), so a fresh default-constructed instance already gives an
     *          all-disabled, zeroed command -- matching MotorCommand's own defaults
     *          (q = dq = tau = kp = kd = 0, enable = false).
     *
     *          ROBOT::H1's unitree_go::msg::dds_::LowCmd_ additionally requires three
     *          non-zero framing fields the zero-initialized default does NOT supply,
     *          without which real hardware/simulators reject (or ignore) the message
     *          entirely -- confirmed against unitreerobotics/unitree_sdk2's own
     *          example/go2/go2_stand_example.cpp InitLowCmd(), which sets exactly
     *          these three fields before touching any motor_cmd() entry:
     *            - head()[0]/head()[1] = 0xFE, 0xEF -- a fixed message-framing marker.
     *            - level_flag() = 0xFF -- selects low-level control; this is the
     *              unitree_go-family counterpart to unitree_hg's mode_machine
     *              requirement (see the class-level @warning), though unlike
     *              mode_machine this value is fixed and not read back from
     *              rt/lowstate.
     *            - gpio() = 0 -- set explicitly by the reference example even though
     *              it's already the zero-initialized default; kept here to mirror
     *              that example exactly rather than relying on the implicit zero.
     *          ROBOT::G1's unitree_hg::msg::dds_::LowCmd_ has no equivalent framing
     *          fields (see the class-level @details) -- only mode_pr/mode_machine,
     *          which set_ankle_mode()/set_mode_machine() cover separately.
     */
    void reset_buffer(const ROBOT& robot)
    {
        std::scoped_lock lock(command_mutex_);
        if (robot == ROBOT::G1) {
            buffered_cmd_ = unitree_hg::msg::dds_::LowCmd_{};
        } else {
            unitree_go::msg::dds_::LowCmd_ go_cmd{};
            go_cmd.head()[0] = 0xFE;
            go_cmd.head()[1] = 0xEF;
            go_cmd.level_flag() = 0xFF;
            go_cmd.gpio() = 0;
            buffered_cmd_ = go_cmd;
        }
    }
};

/**
 * @brief Constructs the wrapper. No network I/O happens here; see connect().
 * @param shutdown_signaler Shared sas::ShutdownSignaler, checked once per publish()
 *        call (see the class-level @note -- this class has no background loop to poll
 *        it from otherwise).
 * @param robot_type Which rt/lowcmd message type to publish. Fixed for the lifetime of
 *        this object (robot_type_ is const) -- construct a new instance if you need to
 *        talk to a different robot type.
 * @throws std::invalid_argument if shutdown_signaler is nullptr.
 */
DriverUnitreeLowCmd::DriverUnitreeLowCmd(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler, const ROBOT& robot_type)
    : robot_type_{robot_type}
{
    if (shutdown_signaler == nullptr) {
        throw std::invalid_argument("DriverUnitreeLowCmd: shutdown_signaler must not be nullptr");
    }
    impl_ = std::make_shared<Impl>(shutdown_signaler);
}

/**
 * @brief Calls deinitialize() then disconnect() so destruction leaves the object in a
 *        clean, unpublished state regardless of what lifecycle stage it was in.
 * @note Does not publish a disabling command first -- see deinitialize()'s own docs.
 */
DriverUnitreeLowCmd::~DriverUnitreeLowCmd()
{
    deinitialize();
    disconnect();
}

/**
 * @brief Sets up the rt/lowcmd publisher with the message type matching robot_type_,
 *        and resets the command buffer to all-disabled and zeroed.
 * @throws std::invalid_argument if robot_type_ doesn't match any known enumerator.
 *         This branch is currently unreachable given ROBOT's two enumerators (G1, H1)
 *         are both handled below, but it guards against a future ROBOT enumerator
 *         being added without a corresponding case here.
 */
void DriverUnitreeLowCmd::connect()
{
    // Precondition: unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
    // must already have been called by the owning driver, same as for
    // DriverUnitreeLocoClient, DriverUnitreeG1ArmSDK, and DriverUnitreeLowState.
    switch (robot_type_) {
    case ROBOT::G1:
    {
        unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> publisher(
            new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicLowCmd));
        publisher->InitChannel();
        impl_->lowcmd_publisher_ = publisher;
        break;
    }
    case ROBOT::H1:
    {
        unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> publisher(
            new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(kTopicLowCmd));
        publisher->InitChannel();
        impl_->lowcmd_publisher_ = publisher;
        break;
    }
    default:
        throw std::invalid_argument("DriverUnitreeLowCmd::connect: unknown RobotType");
    }
    impl_->reset_buffer(robot_type_);
    impl_->is_connected_ = true;
}

/**
 * @brief Marks this object ready for use.
 * @details There is no background loop to start (see the class-level @note); this
 *          only flips a flag.
 * @throws std::runtime_error if connect() has not been called yet.
 */
void DriverUnitreeLowCmd::initialize()
{
    if (!impl_->is_connected_) {
        throw std::runtime_error("DriverUnitreeLowCmd::initialize: Cannot initialize: not connected");
    }
    impl_->is_initialized_ = true;
}

/**
 * @brief Marks this object as no longer initialized.
 * @details No blocking behavior and nothing is published: there is no control loop to
 *          ramp down (see the class-level @note). Callers that need the robot left in
 *          a known-safe state should explicitly buffer and publish() a disabled
 *          command before calling this.
 */
void DriverUnitreeLowCmd::deinitialize()
{
    impl_->is_initialized_ = false;
}

/**
 * @brief Tears down the rt/lowcmd publisher.
 * @details No-op if not currently connected. Calls deinitialize() first if still
 *          initialized, then releases the active publisher (whichever alternative of
 *          the variant is populated) via std::visit.
 */
void DriverUnitreeLowCmd::disconnect()
{
    if (impl_->is_connected_) {
        if (impl_->is_initialized_) {
            deinitialize();
        }
        std::visit([](auto& publisher) { publisher.reset(); }, impl_->lowcmd_publisher_);
        impl_->is_connected_ = false;
    }
}

/**
 * @brief Returns the number of motor_cmd() slots in the underlying rt/lowcmd message.
 * @return 35 for G1, 20 for H1 (see the class-level @details) -- a static property of
 *         robot_type_, available even before connect() is called.
 */
std::size_t DriverUnitreeLowCmd::num_joints() const
{
    return (robot_type_ == ROBOT::G1)
    ? unitree_hg::msg::dds_::LowCmd_{}.motor_cmd().size()
    : unitree_go::msg::dds_::LowCmd_{}.motor_cmd().size();
}

/**
 * @brief Returns the number of joints in the given limb (or TORSO) for this instance's
 *        robot type.
 * @param limb Which limb (or TORSO) to query.
 * @return The joint count for @p limb under robot_type_'s layout, looked up via
 *         limb_indices(). A static property of (robot_type_, limb); does not require
 *         connect() to have been called yet.
 */
std::size_t DriverUnitreeLowCmd::num_joints(const LIMB& limb) const
{
    return limb_indices(robot_type_, limb).size();
}

/**
 * @brief Buffers one joint's command by its absolute motor_cmd() index.
 * @throws std::out_of_range if index >= num_joints() (raised by motor_cmd().at()).
 */
void DriverUnitreeLowCmd::set_joint_command(std::size_t index, const MotorCommand& command)
{
    std::scoped_lock lock(impl_->command_mutex_);
    std::visit([&](auto& cmd) {
        auto& mc = cmd.motor_cmd().at(index);
        mc.mode(command.enable ? 1 : 0);
        mc.q(static_cast<float>(command.q));
        mc.dq(static_cast<float>(command.dq));
        mc.tau(static_cast<float>(command.tau));
        mc.kp(static_cast<float>(command.kp));
        mc.kd(static_cast<float>(command.kd));
    }, impl_->buffered_cmd_);
}

/**
 * @brief Buffers commands for every joint in one limb (or the torso) at once.
 * @throws std::invalid_argument if commands.size() != num_joints(limb).
 */
void DriverUnitreeLowCmd::set_limb_command(const LIMB& limb, const std::vector<MotorCommand>& commands)
{
    const std::vector<int> indices = limb_indices(robot_type_, limb);
    if (commands.size() != indices.size()) {
        throw std::invalid_argument(
            "DriverUnitreeLowCmd::set_limb_command: expected " + std::to_string(indices.size()) +
            " commands for this limb, got " + std::to_string(commands.size()));
    }
    for (std::size_t i = 0; i < indices.size(); ++i) {
        set_joint_command(static_cast<std::size_t>(indices.at(i)), commands.at(i));
    }
}

/**
 * @brief Returns the most recently buffered command for one joint.
 * @throws std::out_of_range if index >= num_joints() (raised by motor_cmd().at()).
 */
DriverUnitreeLowCmd::MotorCommand DriverUnitreeLowCmd::get_joint_command(std::size_t index) const
{
    std::scoped_lock lock(impl_->command_mutex_);
    MotorCommand out;
    std::visit([&](const auto& cmd) {
        const auto& mc = cmd.motor_cmd().at(index);
        out.enable = (mc.mode() != 0);
        out.q      = static_cast<double>(mc.q());
        out.dq     = static_cast<double>(mc.dq());
        out.tau    = static_cast<double>(mc.tau());
        out.kp     = static_cast<double>(mc.kp());
        out.kd     = static_cast<double>(mc.kd());
    }, impl_->buffered_cmd_);
    return out;
}

/**
 * @brief Buffers target positions for every joint in one limb (or the torso),
 *        touching only their q fields.
 * @details Writes only the q field of each joint in @p limb into buffered_cmd_ --
 *          unlike set_limb_command(), which overwrites every field of each joint
 *          from a full MotorCommand, this touches one field across a whole limb,
 *          leaving dq/tau/kp/kd/enable exactly as they were.
 * @throws std::invalid_argument if positions.size() != num_joints(limb).
 */
void DriverUnitreeLowCmd::set_joint_positions(const LIMB& limb, const Eigen::VectorXd& positions)
{
    const std::vector<int> indices = limb_indices(robot_type_, limb);
    if (static_cast<std::size_t>(positions.size()) != indices.size()) {
        throw std::invalid_argument(
            "DriverUnitreeLowCmd::set_joint_positions: expected " + std::to_string(indices.size()) +
            " positions for this limb, got " + std::to_string(positions.size()));
    }
    std::scoped_lock lock(impl_->command_mutex_);
    std::visit([&](auto& cmd) {
        auto& motors = cmd.motor_cmd();
        for (std::size_t i = 0; i < indices.size(); ++i) {
            motors.at(static_cast<std::size_t>(indices.at(i))).q(
                static_cast<float>(positions(static_cast<Eigen::Index>(i))));
        }
    }, impl_->buffered_cmd_);
}

/**
 * @brief Sets the ankle control coordinate mode (unitree_hg's mode_pr field).
 * @details No-op for ROBOT::H1: unitree_go::msg::dds_::LowCmd_ has no mode_pr field,
 *          so std::visit's H1 branch below simply does nothing (checked via
 *          if constexpr, since the two branches would otherwise not compile
 *          identically over both variant alternatives).
 */
void DriverUnitreeLowCmd::set_ankle_mode(const ANKLE_MODE& mode)
{
    std::scoped_lock lock(impl_->command_mutex_);
    std::visit([&](auto& cmd) {
        using T = std::decay_t<decltype(cmd)>;
        if constexpr (std::is_same_v<T, unitree_hg::msg::dds_::LowCmd_>) {
            cmd.mode_pr(mode == ANKLE_MODE::PR ? 0 : 1);
        }
        // ROBOT::H1 (unitree_go::msg::dds_::LowCmd_): no mode_pr field -- no-op, per
        // the class-level @details and this method's own docs.
    }, impl_->buffered_cmd_);
}

/**
 * @brief Sets the machine-identifier field real G1 hardware requires every published
 *        LowCmd_ to echo back (unitree_hg's mode_machine field).
 * @details No-op for ROBOT::H1, for the same reason as set_ankle_mode(): the
 *          unitree_go::msg::dds_::LowCmd_ alternative has no mode_machine field.
 */
void DriverUnitreeLowCmd::set_mode_machine(std::uint8_t mode_machine)
{
    std::scoped_lock lock(impl_->command_mutex_);
    std::visit([&](auto& cmd) {
        using T = std::decay_t<decltype(cmd)>;
        if constexpr (std::is_same_v<T, unitree_hg::msg::dds_::LowCmd_>) {
            cmd.mode_machine(mode_machine);
        }
        // ROBOT::H1 (unitree_go::msg::dds_::LowCmd_): no mode_machine field -- no-op,
        // per the class-level @details and @warning and this method's own docs.
    }, impl_->buffered_cmd_);
}

/**
 * @brief Computes the CRC over the buffered command and publishes it to rt/lowcmd
 *        verbatim.
 * @details Dispatches on robot_type_ directly (rather than std::visit-ing both
 *          variants generically) since the buffer and publisher variants' active
 *          alternative is always determined by robot_type_ alone -- connect() and
 *          reset_buffer() both switch on that same value, so the two variants are
 *          always in lockstep for the lifetime of this object. std::get is therefore
 *          always the correct, already-populated alternative here; it would only
 *          throw std::bad_variant_access if connect() had not been called, which is
 *          already excluded by the is_connected_ check above.
 * @throws std::runtime_error if connect()/initialize() have not been called yet.
 */
bool DriverUnitreeLowCmd::publish()
{
    if (!impl_->is_connected_ || !impl_->is_initialized_) {
        throw std::runtime_error("DriverUnitreeLowCmd::publish: not connected/initialized");
    }
    // Minimal safety measure: this class has no background loop of its own to react
    // to the shutdown signal otherwise (see the class-level @note), so publish() is
    // the one place left to check it.
    if (impl_->shutdown_signaler_->should_shutdown()) {
        return false;
    }

    std::scoped_lock lock(impl_->command_mutex_);
    if (robot_type_ == ROBOT::G1) {
        auto& cmd = std::get<unitree_hg::msg::dds_::LowCmd_>(impl_->buffered_cmd_);
        cmd.crc(crc32_core(reinterpret_cast<const std::uint32_t*>(&cmd), (sizeof(cmd) >> 2) - 1));
        std::get<unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>>(impl_->lowcmd_publisher_)->Write(cmd);
    } else { // ROBOT::H1
        auto& cmd = std::get<unitree_go::msg::dds_::LowCmd_>(impl_->buffered_cmd_);
        cmd.crc(crc32_core(reinterpret_cast<const std::uint32_t*>(&cmd), (sizeof(cmd) >> 2) - 1));
        std::get<unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>>(impl_->lowcmd_publisher_)->Write(cmd);
    }
    return true;
}
