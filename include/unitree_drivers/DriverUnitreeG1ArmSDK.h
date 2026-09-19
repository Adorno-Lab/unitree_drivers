#pragma once
#include <array>
#include <memory>

#include  <marinholab/sas/core/sas_shutdown_signaler.hpp>

/**
 * @brief Controls the G1's arms and waist over the rt/arm_sdk DDS topic, blending with
 *        the robot's autonomous locomotion controller via a ramped weight parameter.
 *
 * @details This class is intentionally G1-specific (not shared with H1 via a ROBOT
 *          parameter, unlike DriverUnitreeLocoClient). Investigation of Unitree's own
 *          reference implementations (xr_teleoperate's robot_arm.py) showed that:
 *          - G1 and H1-2 both expose rt/arm_sdk over unitree_hg::msg::dds_::LowCmd_,
 *            with a weight-blend joint (index 29 for G1, 27 for H1-2) that lets custom
 *            arm commands coexist safely with the autonomous locomotion controller.
 *          - The original H1 exposes no rt/arm_sdk topic and no weight-blend mechanism
 *            at all. Its arm control (H1_ArmController in xr_teleoperate) publishes
 *            directly to rt/lowcmd over unitree_go::msg::dds_::LowCmd_, setting a fixed
 *            per-motor mode flag instead of ramping a blend weight, with no documented
 *            safe-coexistence guarantee with autonomous walking.
 *          Because the underlying safety mechanism (not just the message type) differs
 *          between G1 and H1, this class does not attempt to abstract over both. A
 *          future H1 arm-control class should have its own name and its own documented
 *          safety semantics rather than sharing this interface.
 *
 * @warning enable_arm_control() ramps the blend weight up gradually rather than
 *          snapping to 1.0, and seeds the internal trajectory tracker from the
 *          currently measured joint positions before ramping, so engaging never
 *          commands a sudden jump. disable_arm_control() ramps the weight back down
 *          to 0 while holding the last commanded pose. Even so, always keep the robot
 *          clear of obstacles/people while engaging or disengaging arm control.
 *
 * @note Per the G1 SDK reference documentation: arm control (weight > 0) and
 *       high-level velocity commands (LocoClient::Move / DriverUnitreeLocoClient's
 *       target-velocity control loop) cannot both actively drive the robot at the same
 *       time while in Running/walking mode -- engaging arm control at weight 1.0 will
 *       make the robot stop responding to velocity commands, and it resumes responding
 *       to them once the weight is ramped back down to 0. Coordinate enabling/disabling
 *       this controller with whatever is driving DriverUnitreeLocoClient in the owning
 *       DriverUnitreeG1.
 *
 * @note Signal-driven shutdown: the caller passes a shared sas::ShutdownSignaler
 *       (typically the same one a SIGINT handler calls shutdown() on) at construction
 *       time. The background arm control loop callback polls
 *       shutdown_signaler->should_shutdown() every tick and, the moment it becomes
 *       true, sets arms_enabled_ to false -- reusing the same weight-ramp-to-zero
 *       logic disable_arm_control() triggers -- so the arm starts disengaging within
 *       one control period (20 ms) of the signal, without waiting for the owning
 *       application to notice and call disable_arm_control()/deinitialize() itself.
 *       The callback deliberately does NOT call deinitialize() (which stops the
 *       control thread via sas::ThreadManager::stop()): that call blocks on
 *       std::thread::join(), and the callback runs ON the control thread itself, so
 *       joining it from there is a self-join deadlock. Actual thread teardown (and the
 *       final zero-weight publish) still happens via deinitialize()/disconnect(),
 *       called from the destructor (or explicitly) on the owning application's thread
 *       once its own loops observe the signal and unwind.
 */
class DriverUnitreeG1ArmSDK
{
private:
    class Impl;
    std::shared_ptr<Impl> impl_;

public:
    // Rule of five
    /// @throws std::invalid_argument if shutdown_signaler is nullptr.
    DriverUnitreeG1ArmSDK(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler);
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeG1ArmSDK(const DriverUnitreeG1ArmSDK&) = delete;
    DriverUnitreeG1ArmSDK& operator=(const DriverUnitreeG1ArmSDK&) = delete;
    DriverUnitreeG1ArmSDK(DriverUnitreeG1ArmSDK&&) = delete;
    DriverUnitreeG1ArmSDK& operator=(DriverUnitreeG1ArmSDK&&) = delete;

    ~DriverUnitreeG1ArmSDK();

    /**
     * @brief Sets up the rt/arm_sdk publisher and rt/lowstate subscriber.
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
     * @brief Tears down the rt/arm_sdk publisher and rt/lowstate subscriber.
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

    /// Sets the left arm's 7 target joint positions, in radians.
    void set_left_arm_target_positions(const std::array<double,7>& target_positions);
    /// Sets the right arm's 7 target joint positions, in radians.
    void set_right_arm_target_positions(const std::array<double,7>& target_positions);
    /// Sets the waist's 3 target joint positions, in radians.
    void set_waist_target_positions(const std::array<double,3>& target_positions);

    /// Returns the left arm's 7 measured joint positions, in radians.
    std::array<double,7> get_left_arm_positions();
    /// Returns the right arm's 7 measured joint positions, in radians.
    std::array<double,7> get_right_arm_positions();
    /// Returns the waist's 3 measured joint positions, in radians.
    std::array<double,3> get_waist_positions();

    /// Returns the left arm's 7 commanded trajectory-point positions, in radians.
    std::array<double,7> get_left_arm_desired_positions();
    /// Returns the right arm's 7 commanded trajectory-point positions, in radians.
    std::array<double,7> get_right_arm_desired_positions();
    /// Returns the waist's 3 commanded trajectory-point positions, in radians.
    std::array<double,3> get_waist_desired_positions();
};
