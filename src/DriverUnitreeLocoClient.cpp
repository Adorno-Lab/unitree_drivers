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
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#
# ################################################################
*/

#include <unitree_drivers/DriverUnitreeLocoClient.h>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/h1/loco/h1_loco_api.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <iostream>
#include <stdexcept>
#include <variant>
#include <mutex>
#include <atomic>
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <functional>

class DriverUnitreeLocoClient::Impl
{
public:
    // Only one of these is ever populated, selected at construction time by RobotType.
    // A variant keeps the two concrete SDK client types distinct (no shared base class
    // is assumed between g1::LocoClient and h1::LocoClient) while still letting the
    // rest of Impl's code stay robot-agnostic via std::visit.
    std::variant<std::unique_ptr<unitree::robot::g1::LocoClient>,
                 std::unique_ptr<unitree::robot::h1::LocoClient>> loco_client_;

    std::unique_ptr<marinholab::sas::core::ThreadManager> control_thread_;
    mutable std::mutex data_mutex_;

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_initialized_{false};

    std::array<double,3> target_high_level_velocities_{0,0,0};

    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler_; ///< Shared shutdown coordinator, polled every tick in control_loop_callback().

    void create_client(const DriverUnitreeLocoClient::ROBOT& robot_type)
    {
        switch (robot_type) {
        case DriverUnitreeLocoClient::ROBOT::G1:
            loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
            break;
        case DriverUnitreeLocoClient::ROBOT::H1:
            loco_client_ = std::make_unique<unitree::robot::h1::LocoClient>();
            break;
        default:
            throw std::invalid_argument("DriverUnitreeLocoClient::Impl: unknown RobotType");
        }
    }

    /**
    * @brief Constructs the concrete SDK LocoClient matching @p robot_type.
    *
    * @details Only one alternative of the loco_client_ variant is ever populated,
    *          chosen here based on @p robot_type. This is the single point in the
    *          class where the concrete unitree::robot::g1::LocoClient or
    *          unitree::robot::h1::LocoClient type is selected; every other method
    *          on Impl stays robot-agnostic by going through with_client().
    *
    * @param robot_type Which robot's LocoClient implementation to instantiate.
    *
    * @throws std::invalid_argument if @p robot_type doesn't match any known
    *         enumerator. This branch is currently unreachable given ROBOT's two
    *         enumerators (G1, H1) are both handled above, but it guards against a
    *         future ROBOT enumerator being added here without a corresponding
    *         case -- e.g. after adding ROBOT::H1_2, forgetting to add its case
    *         here would fall through to this exception at construction time
    *         instead of silently leaving loco_client_ in a valueless state.
    *
    * @note Only allocates the SDK client object; it does not call LocoClient::Init()
    *       or touch the network in any way. That happens later in connect(), which
    *       requires unitree::robot::ChannelFactory::Instance()->Init() to have
    *       already run (see DriverUnitreeLocoClient::connect()).
    */
    explicit Impl(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler)
        : shutdown_signaler_{shutdown_signaler}
    {
        /*
        switch (robot_type) {
        case DriverUnitreeLocoClient::ROBOT::G1:
            loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
            break;
        case DriverUnitreeLocoClient::ROBOT::H1:
            loco_client_ = std::make_unique<unitree::robot::h1::LocoClient>();
            break;
        default:
            throw std::invalid_argument("DriverUnitreeLocoClient::Impl: unknown RobotType");
        }
        */
    }

    /**
     * @brief Invokes @p fn on whichever concrete LocoClient is currently active.
     *
     * @details Wraps std::visit over loco_client_ so call sites read like a normal
     *          virtual dispatch (`impl_->with_client([](auto& client){ client.Move(...); })`)
     *          without requiring g1::LocoClient and h1::LocoClient to share a common
     *          base class. @p fn is instantiated as a generic lambda and must therefore
     *          compile against *both* concrete client types -- i.e. it may only call
     *          methods present on both g1::LocoClient and h1::LocoClient with
     *          compatible signatures (e.g. GetFsmId, SetFsmId, Move, SetVelocity,
     *          SetSwingHeight, SetStandHeight, GetPhase). Methods that exist on only
     *          one robot's client (e.g. SetSpeedMode, which G1 exposes but H1 does
     *          not) cannot be called through this helper and must be dispatched
     *          manually via std::get<...>(loco_client_) instead, guarded by a check
     *          on the active robot type (see DriverUnitreeLocoClient::set_speed_mode()).
     *
     * @tparam Fn Callable of signature compatible with `auto(ConcreteClient&)`, where
     *         ConcreteClient is either unitree::robot::g1::LocoClient or
     *         unitree::robot::h1::LocoClient.
     * @param fn The operation to run against the active client.
     * @return Whatever @p fn returns.
     */
    template <typename Fn>
    auto with_client(Fn&& fn)
    {
        return std::visit([&](auto& client) { return fn(*client); }, loco_client_);
    }

    /**
     * @brief Background control-loop tick: republishes the last-set target high-level
     *        velocity via the active LocoClient's Move() call.
     *
     * @details Invoked periodically by control_thread_ (see start_control_thread()).
     *          No-ops if the driver isn't both connected and initialized, so stray
     *          ticks that fire during shutdown or before connect() don't attempt to
     *          talk to the SDK. Takes data_mutex_ for the duration of the call to
     *          keep target_high_level_velocities_ consistent with concurrent writes
     *          from DriverUnitreeLocoClient::set_target_high_level_velocities().
     *
     * @note Exceptions from the SDK call are caught and logged rather than propagated,
     *       since this runs on the background thread with no caller able to observe
     *       or handle an exception thrown from here -- letting it propagate would
     *       otherwise terminate the thread (or the process, depending on how
     *       sas::ThreadManager invokes the callback).
     */
    void control_loop_callback()
    {
        if (!is_connected_ || !is_initialized_) {
            return;
        }
        std::scoped_lock lock(data_mutex_);
        // Interruption requested (e.g. SIGINT): zero the target velocity so
        // locomotion stops this tick. Deliberately does NOT call deinitialize() (and
        // therefore not sas::ThreadManager::stop()) from here: this callback runs ON
        // the control thread itself, and stop() joins that thread -- a thread joining
        // itself is a self-join deadlock. Actual thread teardown happens later, from
        // the main thread, via this object's destructor (or an explicit
        // deinitialize()/disconnect() call) -- see the class-level @note.
        if (shutdown_signaler_->should_shutdown()) {
            target_high_level_velocities_ = {0.0, 0.0, 0.0};
        }
        try {
            with_client([&](auto& client) {
                client.Move(static_cast<float>(target_high_level_velocities_.at(0)),
                            static_cast<float>(target_high_level_velocities_.at(1)),
                            static_cast<float>(target_high_level_velocities_.at(2)));
            });
        } catch (const std::exception& e) {
            std::cerr << "DriverUnitreeLocoClient control loop error: " << e.what() << std::endl;
        }
    }

    /**
     * @brief Starts the periodic background thread that drives control_loop_callback().
     *
     * @details Idempotent: if control_thread_ already exists and is running, this is
     *          a no-op, so callers (e.g. DriverUnitreeLocoClient::initialize()) don't
     *          need to guard against calling it more than once.
     *
     * @param period Control loop period, in seconds.
     * @param priority Thread scheduling priority to request from sas::ThreadManager.
     */
    void start_control_thread(const double& period,
                              const marinholab::sas::core::ThreadManager::PRIORITY& priority)
    {
        if (control_thread_ && control_thread_->is_running()) {
            return;
        }
        control_thread_ = std::make_unique<marinholab::sas::core::ThreadManager>(
            "unitree_loco_client_control",
            period,
            std::bind(&Impl::control_loop_callback, this),
            priority,
            -1
            );
        control_thread_->start();
    }

    /**
     * @brief Stops and releases the background control thread, if one is running.
     *
     * @details Safe to call when no thread has been started (control_thread_ is
     *          null): this is then a no-op. After returning, control_thread_ is
     *          reset to null, so a subsequent start_control_thread() call will
     *          create and start a fresh thread rather than being treated as
     *          already-running.
     */
    void stop_control_thread()
    {
        if (control_thread_) {
            control_thread_->stop();
            control_thread_.reset();
        }
    }
};


/**
 * @brief Constructs the wrapper and the underlying concrete LocoClient for @p robot_type.
 * @param shutdown_signaler Shared sas::ShutdownSignaler (typically the same one a
 *        SIGINT handler calls shutdown() on), forwarded to Impl so the background
 *        control loop callback can poll should_shutdown() every tick.
 * @param robot_type Which robot's LocoClient to instantiate. Fixed for the lifetime
 *        of this object (robot_type_ is const) -- construct a new instance if you
 *        need to talk to a different robot type.
 * @param control_period Period, in seconds, of the background control loop. Stored
 *        directly on this wrapper (control_period_ is a member of
 *        DriverUnitreeLocoClient, not forwarded into Impl at construction time here)
 *        and used later, once initialize() starts the control thread, to drive that
 *        loop's actual tick rate.
 *
 * @throws std::invalid_argument if shutdown_signaler is nullptr.
 * @note No network I/O happens here. The underlying LocoClient::Init() call, which
 *       requires unitree::robot::ChannelFactory::Instance()->Init() to have already
 *       run, is deferred to connect().
 */
DriverUnitreeLocoClient::DriverUnitreeLocoClient(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler> &shutdown_signaler,
                                                 const ROBOT& robot_type,
                                                 const double &control_period)
    : robot_type_{robot_type}, control_period_{control_period}
{
    if (shutdown_signaler == nullptr) {
        throw std::invalid_argument("DriverUnitreeLocoClient: shutdown_signaler must not be nullptr");
    }
    impl_ = std::make_shared<Impl>(shutdown_signaler);
}

/**
 * @brief Stops the control thread and (if still connected) sends a final stop command.
 *
 * @details Calls deinitialize() then disconnect() so destruction leaves the robot in
 *          a safe, stopped state regardless of what lifecycle stage the object was in.
 */
DriverUnitreeLocoClient::~DriverUnitreeLocoClient()
{
    deinitialize();
    disconnect();
}

/**
 * @brief Initializes the underlying SDK LocoClient and marks this object as connected.
 *
 * @pre unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
 *      must already have been called by the owning driver. This method does not call
 *      it, since the DDS channel factory is a process-wide singleton that may also be
 *      shared with other sub-controllers (e.g. an arm-sdk controller) owned by the
 *      same top-level driver.
 *
 * @post is-connected state is true; get_fsm_id()/get_fsm_mode()/etc. become valid to call.
 */
void DriverUnitreeLocoClient::connect()
{
    // Precondition: unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
    // must already have been called by the owning driver (DriverUnitreeG1/H1) before this runs.
    if (impl_->is_connected_) {
        return; // already connected — don't re-Init or rebuild the client
    }
    {
        std::scoped_lock lock(impl_->data_mutex_);
        impl_->create_client(robot_type_);
    }
    impl_->with_client([](auto& client) {
        client.Init();
        client.SetTimeout(10.0f);
    });
    impl_->is_connected_ = true;
}

/**
 * @brief Starts the background control thread that periodically republishes the
 *        target high-level velocity via the SDK's Move() call.
 *
 * @throws std::runtime_error if connect() has not been called yet.
 */
void DriverUnitreeLocoClient::initialize()
{
    if (!impl_->is_connected_) {
        throw std::runtime_error("DriverUnitreeLocoClient::initialize: Cannot initialize: not connected");
    }
    impl_->is_initialized_ = true;
    if (!impl_->control_thread_ || !impl_->control_thread_->is_running()) {
        impl_->start_control_thread(control_period_, marinholab::sas::core::ThreadManager::PRIORITY::NORMAL);
    }
}


/**
 * @brief Stops the background control thread and commands a zero velocity (0,0,0)
 *        before returning, so locomotion doesn't continue on the last-sent command.
 *
 * @note Safe to call multiple times; a no-op if not currently initialized.
 */
void DriverUnitreeLocoClient::deinitialize()
{
    if (impl_->is_initialized_) {
        impl_->stop_control_thread();
        try {
            impl_->with_client([](auto& client) { client.Move(0.0f, 0.0f, 0.0f); });
        } catch (const std::exception& e) {
            std::cerr << "DriverUnitreeLocoClient::deinitialize: failed to send stop command: "
                      << e.what() << std::endl;
        }
        impl_->is_initialized_ = false;
    }
}


/**
 * @brief Tears down the connection. Calls deinitialize() first if still initialized,
 *        otherwise just stops the control thread if it happens to be running.
 *
 * @note Safe to call multiple times; a no-op if not currently connected.
 */
void DriverUnitreeLocoClient::disconnect()
{
    if (impl_->is_connected_) {
        if (impl_->is_initialized_) {
            deinitialize();
        } else {
            impl_->stop_control_thread();
        }
        impl_->is_connected_ = false;
    }
}

/**
 * @brief Queries the robot's current locomotion FSM state ID.
 * @return The FSM ID as reported by the SDK.
 * @warning The integer's meaning is robot- and firmware-specific. See the class-level
 *          @warning above before interpreting this value.
 */
int DriverUnitreeLocoClient::get_fsm_id() const
{
    int fsm_id{};
    impl_->with_client([&](auto& client) { client.GetFsmId(fsm_id); });
    return fsm_id;
}


/**
 * @brief Queries the robot's current locomotion FSM mode.
 * @return The FSM mode as reported by the SDK.
 */
int DriverUnitreeLocoClient::get_fsm_mode() const
{
    int fsm_mode{};
    impl_->with_client([&](auto& client) { client.GetFsmMode(fsm_mode); });
    return fsm_mode;
}

/**
 * @brief Queries the current balance mode.
 * @return Balance mode reported by the SDK (commonly 0 = static balance stand,
 *         1 = continuous gait, but confirm against the installed SDK/firmware).
 */
int DriverUnitreeLocoClient::get_balance_mode() const
{
    int balance_mode{};
    impl_->with_client([&](auto& client) { client.GetBalanceMode(balance_mode); });
    return balance_mode;
}

/**
 * @brief Queries the current commanded foot swing height.
 * @return Swing height in meters.
 */
double DriverUnitreeLocoClient::get_swing_height() const
{
    float swing_height{};
    impl_->with_client([&](auto& client) { client.GetSwingHeight(swing_height); });
    return static_cast<double>(swing_height);
}

/**
 * @brief Queries the current commanded standing/body height.
 * @return Stand height in meters.
 */
double DriverUnitreeLocoClient::get_stand_height() const
{
    float stand_height{};
    impl_->with_client([&](auto& client) { client.GetStandHeight(stand_height); });
    return static_cast<double>(stand_height);
}

/**
 * @brief Queries the current gait phase.
 * @return Gait phase vector as reported by the SDK.
 * @note Marked deprecated in the underlying SDK for both G1 and H1 as of the versions
 *       reviewed; prefer other state if your firmware exposes it.
 */
std::vector<double> DriverUnitreeLocoClient::get_phase() const
{
    std::vector<float> phase_float;
    impl_->with_client([&](auto& client) { client.GetPhase(phase_float); });

    std::vector<double> phase_double;
    phase_double.reserve(phase_float.size());
    phase_double.assign(phase_float.begin(), phase_float.end());
    return phase_double;
}

/**
 * @brief Requests a transition to the given locomotion FSM state.
 * @param fsm_id Robot- and firmware-specific FSM state identifier.
 *
 * @warning DO NOT reuse an FSM ID observed on one robot/firmware on another without
 *          re-verifying it. The same integer has been observed to map to different
 *          (sometimes opposite, e.g. "stand up" vs "squat") behaviors between G1 and
 *          H1, and to shift meaning across G1 firmware revisions. Always check the
 *          FSM ID against the SDK headers and firmware actually running on the target
 *          robot before calling this. See the class-level @warning for details.
 */
void DriverUnitreeLocoClient::set_fsm_id(const int& fsm_id)
{
    impl_->with_client([&](auto& client) { client.SetFsmId(fsm_id); });
}

/**
 * @brief Sets the balance mode.
 * @param balance_mode Robot-specific balance mode identifier.
 * @warning Verify against the installed SDK/firmware; see the class-level @warning.
 */
void DriverUnitreeLocoClient::set_balance_mode(const int& balance_mode)
{
    impl_->with_client([&](auto& client) { client.SetBalanceMode(balance_mode); });
}

/**
 * @brief Sets the target foot swing height.
 * @param swing_height Swing height in meters.
 */
void DriverUnitreeLocoClient::set_swing_height(const double& swing_height)
{
    impl_->with_client([&](auto& client) { client.SetSwingHeight(static_cast<float>(swing_height)); });
}

/**
 * @brief Sets the target standing/body height.
 * @param stand_height Stand height in meters.
 */
void DriverUnitreeLocoClient::set_stand_height(const double& stand_height)
{
    impl_->with_client([&](auto& client) { client.SetStandHeight(static_cast<float>(stand_height)); });
}

/**
 * @brief Sets the locomotion speed mode.
 * @param mode Speed mode identifier.
 * @throws std::runtime_error if the currently wrapped robot type is H1, whose
 *         LocoClient does not expose this call in the reviewed SDK version -- if a
 *         future firmware/SDK release adds it, update this class accordingly rather
 *         than assuming it is universally available.
 */
void DriverUnitreeLocoClient::set_speed_mode(const int& mode)
{
    switch (robot_type_) {
    case ROBOT::G1:
        std::get<std::unique_ptr<unitree::robot::g1::LocoClient>>(
            impl_->loco_client_)->SetSpeedMode(mode);
        break;
    case ROBOT::H1:
        throw std::runtime_error("set_speed_mode is not supported on H1");
    default:
        throw std::invalid_argument("DriverUnitreeLocoClient::set_speed_mode: unknown RobotType");
    }
}


/**
 * @brief Sets the target high-level body velocity that the background control loop
 *        will continuously republish once initialize() has started that loop.
 * @param target_high_level_velocities {vx, vy, omega} in m/s, m/s, rad/s respectively.
 */
void DriverUnitreeLocoClient::set_target_high_level_velocities(const std::array<double,3>& target_high_level_velocities)
{
    std::scoped_lock lock(impl_->data_mutex_);
    impl_->target_high_level_velocities_ = target_high_level_velocities;
}
