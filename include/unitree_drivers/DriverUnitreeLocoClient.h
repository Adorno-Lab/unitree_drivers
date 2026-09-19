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

#pragma once
#include <array>
#include <memory>
#include <vector>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

/**
 * @brief Wraps the Unitree SDK's high-level LocoClient (unitree::robot::g1::LocoClient
 *        or unitree::robot::h1::LocoClient) behind a single, robot-agnostic interface.
 *
 * @details Selecting a ROBOT_TYPE at construction time picks which concrete SDK client
 *          is instantiated internally; every public method then dispatches to whichever
 *          client is active. This lets DriverUnitreeG1 and DriverUnitreeH1 share the same
 *          locomotion-control code by aggregation instead of duplicating it.
 *
 * @warning FSM IDs, balance-mode values, and other integer codes accepted by
 *          set_fsm_id()/set_balance_mode() are **not portable across robots or firmware
 *          versions**. The same integer can mean different things on G1 vs H1, and can
 *          even change meaning between firmware revisions of the *same* robot (community
 *          reports show G1's "Start" state alone has been observed at FSM ID 200, 500,
 *          501, and 801 depending on firmware version and waist DoF configuration, with
 *          some IDs becoming non-functional after certain firmware updates). This class
 *          intentionally does NOT hardcode a translation table for these codes, because
 *          doing so would silently go stale as firmware changes. Always confirm the
 *          correct FSM/balance-mode values against the actual SDK headers
 *          (g1_loco_client.hpp / h1_loco_client.hpp) and firmware installed on the
 *          specific robot you're commanding, and keep any such mapping in your
 *          application-level code (e.g. DriverUnitreeG1/DriverUnitreeH1), not here.
 *
 * @note This class assumes unitree::robot::ChannelFactory::Instance()->Init(domain_id,
 *       network_interface) has already been called by the owning driver before connect()
 *       is invoked. It does not manage the DDS domain participant itself, since that is a
 *       process-wide singleton potentially shared with other sub-controllers (e.g. an
 *       arm-sdk controller) aggregated by the same owning driver.
 *
 * @note Signal-driven shutdown: the caller passes a shared sas::ShutdownSignaler
 *       (typically the same one a SIGINT handler calls shutdown() on) at construction
 *       time. The background control loop callback polls
 *       shutdown_signaler->should_shutdown() every tick and, the moment it becomes
 *       true, zeroes the target velocity so locomotion stops within one control period
 *       (10 ms) of the signal, without waiting for the owning application to notice
 *       and call deinitialize() itself. The callback deliberately does NOT call
 *       deinitialize() (which stops the control thread via sas::ThreadManager::stop()):
 *       that call blocks on std::thread::join(), and the callback runs ON the control
 *       thread itself, so joining it from there is a self-join deadlock. Actual thread
 *       teardown still happens via deinitialize()/disconnect(), called from the
 *       destructor (or explicitly) on the owning application's thread once its own
 *       loops observe the signal and unwind.
 */
class DriverUnitreeLocoClient
{
public:
    /**
     * @brief Identifies which concrete Unitree SDK LocoClient this instance wraps.
     * @note Other robots can be added in future versions.
     */
    enum class ROBOT{G1,H1}; // Other robots can be added in future versions
private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    const ROBOT robot_type_;

public:

    // Rule of five
    DriverUnitreeLocoClient()=delete;
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeLocoClient(const DriverUnitreeLocoClient&) = delete;
    DriverUnitreeLocoClient& operator=(const DriverUnitreeLocoClient&) = delete;
    DriverUnitreeLocoClient(DriverUnitreeLocoClient&&) = delete;
    DriverUnitreeLocoClient& operator=(DriverUnitreeLocoClient&&) = delete;

    /// @throws std::invalid_argument if shutdown_signaler is nullptr.
    explicit DriverUnitreeLocoClient(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler, const ROBOT& robot_type);
    ~DriverUnitreeLocoClient();

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    // Getters
    int get_fsm_id()                  const;
    int get_fsm_mode()                const;
    int get_balance_mode()            const;
    double get_swing_height()         const;
    double get_stand_height()         const;
    std::vector<double> get_phase()   const;

    // Setters
    void set_fsm_id(const int& fsm_id);
    void set_balance_mode(const int& balance_mode);
    void set_swing_height(const double& swing_height);
    void set_stand_height(const double& stand_height);
    void set_speed_mode(const int& mode);
    void set_target_high_level_velocities(const std::array<double,3>& target_high_level_velocities);
};
