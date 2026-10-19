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
# ################################################################
*/

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

/**
 * @brief Reads the head camera of a Unitree robot through Unitree's videohub RPC
 *        service, which answers each request with its latest camera frame as a JPEG.
 *
 * @details videohub is the service the Unitree app uses for the camera. It owns the
 *          camera device (on the G1, videohub_pc4 keeps /dev/video4 open), so this is
 *          the way to read the camera without stopping Unitree's services. The
 *          protocol is the one of unitree::robot::go2::VideoClient: service
 *          "videohub", API version 1.0.0.0, API id 1001 (GetImageSample), an empty
 *          binary request, and the JPEG bytes as the binary reply. unitree_sdk2 only
 *          ships the Go2 client, whose service name is compiled into the library;
 *          this class sends the same requests with a configurable service name.
 *          Tested on:
 *          - G1: service "videohub" on eth0, DDS domain 0, 1920x1080 JPEG frames of
 *            ~145 KB, ~16 ms per request, ~18 new frames per second (measured with
 *            unitree-g1's software/utils/videohub_test).
 *          - H1: service "videohub" on eth0, DDS domain 0, with the stock
 *            unitree::robot::go2::VideoClient (sas_robot_driver_unitree_h1's camera
 *            node). The H1 head camera is mounted upside down: its frames must be
 *            rotated by 180 degrees, which is left to the caller (this class only
 *            returns the JPEG bytes, it does not decode them).
 *
 *          The protocol is the same on every robot tested, so unlike the other classes
 *          of unitree_drivers there is no ROBOT enum.
 *
 * @note Background capture thread: initialize() starts a thread that requests a frame
 *       every request_period seconds. A request blocks until videohub answers or
 *       rpc_timeout expires, so the thread is needed even for a single consumer.
 *       videohub answers with its latest frame, so requests faster than the camera
 *       return the same frame again: with skip_duplicates (the default), a frame
 *       identical to the previous one is counted in Statistics::duplicate_frames and
 *       otherwise ignored. Each new frame is stored (get_latest_frame()) and passed
 *       to the frame callback, if one is set (set_frame_callback()).
 *
 * @note Signal-driven shutdown: once shutdown_signaler->should_shutdown() is true, the
 *       capture thread stops sending requests. The thread itself is joined by
 *       deinitialize(), disconnect(), or the destructor, which take up to rpc_timeout
 *       if a request is pending.
 *
 * @note This class assumes unitree::robot::ChannelFactory::Instance()->Init(domain_id,
 *       network_interface) has already been called by the owner before connect() is
 *       invoked, exactly like the other classes of unitree_drivers. On the G1 and the
 *       H1: domain 0, and the interface on the 192.168.123.x network (eth0).
 *
 * @code
 * unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
 * DriverUnitreeVideohub camera(shutdown_signaler);
 * camera.set_frame_callback([](const std::shared_ptr<const DriverUnitreeVideohub::Frame>& frame)
 * {
 *     // frame->jpeg: the JPEG bytes, frame->width x frame->height
 * });
 * camera.connect();
 * camera.initialize();
 * @endcode
 */
class DriverUnitreeVideohub
{
public:
    /**
     * @brief One frame of videohub.
     */
    struct Frame
    {
        std::vector<uint8_t> jpeg;      ///< The JPEG bytes, as videohub sent them.
        int width = 0;                  ///< Width in pixels, read from the JPEG header without decoding it. 0 if not found.
        int height = 0;                 ///< Height in pixels, read from the JPEG header without decoding it. 0 if not found.
        uint64_t sequence = 0;          ///< 1 for the first new frame, incremented on every new frame.
        std::chrono::system_clock::time_point reception_time; ///< When the reply arrived (system clock of this computer).
        double request_duration_ms = 0; ///< Duration of the request that returned this frame.
    };

    /**
     * @brief Counters of the capture thread, cumulative since construction. Compare two
     *        readings to obtain rates.
     */
    struct Statistics
    {
        uint64_t requests = 0;                ///< Requests sent, successful or not.
        uint64_t failed_requests = 0;         ///< Requests that returned an error code or an empty image.
        uint64_t duplicate_frames = 0;        ///< Frames identical to the previous one (only with skip_duplicates).
        uint64_t new_frames = 0;              ///< Frames stored and passed to the frame callback.
        uint64_t callback_exceptions = 0;     ///< Exceptions thrown by the frame callback (caught and logged).
        double total_request_duration_ms = 0; ///< Sum of the durations of all requests.
        int32_t last_error_code = 0;          ///< unitree_sdk2 code of the last failed request (see get_error_description()).
    };

    /**
     * @brief Called by the capture thread on every new frame.
     * @warning It runs on the capture thread: it must not call deinitialize(),
     *          disconnect(), or destroy this object (the thread would join itself), and
     *          it delays the next request while it runs.
     */
    using FrameCallback = std::function<void(const std::shared_ptr<const Frame>&)>;

private:
    class Impl;
    std::shared_ptr<Impl> impl_;

public:
    // Rule of five
    DriverUnitreeVideohub()=delete;
    // Delete copy constructor and assignment (prevents double initialization)
    DriverUnitreeVideohub(const DriverUnitreeVideohub&) = delete;
    DriverUnitreeVideohub& operator=(const DriverUnitreeVideohub&) = delete;
    DriverUnitreeVideohub(DriverUnitreeVideohub&&) = delete;
    DriverUnitreeVideohub& operator=(DriverUnitreeVideohub&&) = delete;

    /**
     * @param shutdown_signaler Shared shutdown coordinator, polled by the capture thread.
     * @param service_name Name of the RPC service (requests go to rt/api/<service_name>/request).
     * @param request_period Period of the requests, in seconds. The default, 1/30 s,
     *        polls faster than the ~18 new frames per second of the G1 camera, so no
     *        frame is missed; the extra requests return duplicates.
     * @param rpc_timeout Timeout of each request, in seconds.
     * @param skip_duplicates If true, a frame identical to the previous one is not
     *        stored nor passed to the frame callback.
     * @throws std::invalid_argument if shutdown_signaler is nullptr, service_name is
     *         empty, or request_period or rpc_timeout is not positive.
     */
    explicit DriverUnitreeVideohub(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                                   const std::string& service_name = "videohub",
                                   const double& request_period = 1.0/30.0,
                                   const double& rpc_timeout = 1.0,
                                   const bool& skip_duplicates = true);
    ~DriverUnitreeVideohub();

    /**
     * @brief Creates the RPC client of videohub. No request is sent yet.
     * @pre unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
     *      must already have been called by the owner.
     */
    void connect();

    /**
     * @brief Starts the capture thread.
     * @throws std::runtime_error if connect() has not been called yet.
     */
    void initialize();

    /// Stops and joins the capture thread (up to rpc_timeout, if a request is pending).
    void deinitialize();

    /// Calls deinitialize() first if still initialized, and releases the RPC client.
    void disconnect();

    /**
     * @brief Sets the function called by the capture thread on every new frame. Can be
     *        called at any time; an empty function removes the callback.
     */
    void set_frame_callback(const FrameCallback& frame_callback);

    /// The last new frame, or nullptr if none has been received yet.
    std::shared_ptr<const Frame> get_latest_frame() const;

    /// The counters of the capture thread.
    Statistics get_statistics() const;

    /// The name of the RPC service given at construction.
    std::string get_service_name() const;

    /**
     * @brief Describes the unitree_sdk2 RPC error codes (unitree/robot/internal/internal_error.hpp).
     * @param code The code, e.g. Statistics::last_error_code.
     * @return A short description of the code.
     */
    static std::string get_error_description(const int32_t& code);
};
