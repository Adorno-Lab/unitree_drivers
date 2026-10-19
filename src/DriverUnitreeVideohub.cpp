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

#include <unitree_drivers/DriverUnitreeVideohub.h>
#include <unitree/robot/client/client.hpp>
#include <iostream>
#include <stdexcept>
#include <mutex>
#include <atomic>
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <functional>

namespace
{

/**
 * @brief Clone of unitree::robot::go2::VideoClient with a configurable service name.
 *
 * @details Same requests as the Go2 client (unitree/robot/go2/video/video_api.hpp):
 *          API version 1.0.0.0, API id 1001 (GetImageSample), an empty binary request,
 *          and a binary JPEG reply. unitree::robot::ChannelFactory::Instance()->Init()
 *          must have been called before Init().
 */
class VideohubClient : public unitree::robot::Client
{
public:
    static constexpr const char* API_VERSION = "1.0.0.0";
    static constexpr int32_t API_ID_GET_IMAGE_SAMPLE = 1001;

    /// @param service_name Name of the RPC service (requests go to `rt/api/<service_name>/request`).
    explicit VideohubClient(const std::string& service_name)
        : unitree::robot::Client(service_name, false)
    {
    }

    /// Sets the API version and registers GetImageSample.
    void Init() override
    {
        SetApiVersion(API_VERSION);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(API_ID_GET_IMAGE_SAMPLE);
    }

    /**
     * @brief Requests one frame. Blocks until the reply arrives or the timeout (SetTimeout()) expires.
     * @param image Output: the JPEG bytes returned by the service.
     * @return 0 on success, an unitree_sdk2 error code otherwise.
     */
    int32_t GetImageSample(std::vector<uint8_t>& image)
    {
        const std::vector<uint8_t> parameter;
        return Call(API_ID_GET_IMAGE_SAMPLE, parameter, image);
    }
};

/**
 * @brief Reads the resolution of a JPEG image from its header, without decoding it.
 *
 * @details Fills Frame::width and Frame::height. Reading the header takes microseconds,
 *          whereas decoding a 1920x1080 frame takes milliseconds, and this class never
 *          decodes the frames. This function is not part of unitree_sdk2 (videohub only
 *          returns the bytes); it follows the JPEG standard, ITU-T T.81 (ISO/IEC 10918-1),
 *          Annex B.
 *
 *          A JPEG file is a sequence of segments. Each one starts with a marker: the byte
 *          0xFF followed by a byte that identifies the segment. Most segments then have a
 *          2-byte big-endian length, which counts itself and the data but not the marker.
 *          The markers used here:
 *          - SOI (Start Of Image), 0xFF 0xD8: the first two bytes of every JPEG file. It
 *            has no length. If the data does not start with it, it is not a JPEG image.
 *          - SOFn (Start Of Frame, n = 0..15), 0xFF 0xC0 to 0xFF 0xCF: the frame header,
 *            which holds the resolution. n is the coding process: SOF0 baseline (the usual
 *            one), SOF1 extended, SOF2 progressive, SOF3 lossless, and so on. Three codes of
 *            that range are not SOFn: 0xC4 (DHT, Define Huffman Table), 0xC8 (JPG, reserved),
 *            and 0xCC (DAC, Define Arithmetic Coding conditioning).
 *
 *          The function starts after SOI and jumps from segment to segment with their
 *          lengths (skipping e.g. APP0/JFIF, DQT, and DHT) until it finds a SOFn. The SOFn
 *          always comes before the compressed image data (SOS), so the search never reaches
 *          it. The layout of a SOFn segment, from the 0xFF of its marker at offset i:
 *
 *          | Offset   | Bytes | Content                           |
 *          |----------|-------|-----------------------------------|
 *          | i, i+1   | 2     | marker (0xFF 0xCn)                |
 *          | i+2, i+3 | 2     | length of the segment             |
 *          | i+4      | 1     | sample precision (bits, usually 8)|
 *          | i+5, i+6 | 2     | height (number of lines)          |
 *          | i+7, i+8 | 2     | width (samples per line)          |
 *
 * @note Fill bytes (extra 0xFF bytes before a marker, which the standard allows but
 *       encoders rarely write) are not handled: the function then returns false, and the
 *       frame is still delivered with width and height 0.
 *
 * @param jpeg JPEG bytes.
 * @param width Output: width in pixels.
 * @param height Output: height in pixels.
 * @return True if the data starts with the SOI marker and a SOFn marker was found; width
 *         and height are left unchanged otherwise.
 */
bool jpeg_resolution(const std::vector<uint8_t>& jpeg, int& width, int& height)
{
    if (jpeg.size() < 4 || jpeg.at(0) != 0xFF || jpeg.at(1) != 0xD8) {
        return false;
    }
    std::size_t i = 2;
    while (i + 9 < jpeg.size()) {
        if (jpeg.at(i) != 0xFF) {
            return false;
        }
        const uint8_t marker = jpeg.at(i + 1);
        const std::size_t length = (static_cast<std::size_t>(jpeg.at(i + 2)) << 8) | jpeg.at(i + 3);
        // SOF0..SOF15, except DHT (C4), JPG (C8) and DAC (CC).
        if (marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC) {
            height = (jpeg.at(i + 5) << 8) | jpeg.at(i + 6);
            width = (jpeg.at(i + 7) << 8) | jpeg.at(i + 8);
            return true;
        }
        i += 2 + length;
    }
    return false;
}

} // namespace

class DriverUnitreeVideohub::Impl
{
public:
    const std::string service_name_;
    const double request_period_;
    const double rpc_timeout_;
    const bool skip_duplicates_;

    std::unique_ptr<VideohubClient> client_;
    std::unique_ptr<marinholab::sas::core::ThreadManager> capture_thread_;

    /// Protects latest_frame_, statistics_, and frame_callback_.
    mutable std::mutex data_mutex_;
    std::shared_ptr<const Frame> latest_frame_;
    Statistics statistics_;
    FrameCallback frame_callback_;

    /// Reply buffer of the capture thread, reused between requests.
    std::vector<uint8_t> image_;

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> is_initialized_{false};

    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler_; ///< Shared shutdown coordinator, polled every tick in capture_loop_callback().

    Impl(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
         const std::string& service_name,
         const double& request_period,
         const double& rpc_timeout,
         const bool& skip_duplicates)
        : service_name_{service_name},
          request_period_{request_period},
          rpc_timeout_{rpc_timeout},
          skip_duplicates_{skip_duplicates},
          shutdown_signaler_{shutdown_signaler}
    {
    }

    /**
     * @brief Capture-thread tick: requests one frame and, if it is new, stores it and
     *        passes it to the frame callback.
     *
     * @details Invoked periodically by capture_thread_ (see start_capture_thread()).
     *          No-ops if the driver isn't both connected and initialized, or once a
     *          shutdown has been requested. The request itself runs without holding
     *          data_mutex_ (it blocks for up to rpc_timeout_), so the getters never
     *          wait for videohub. After a slow request (e.g. a timeout),
     *          sas::ThreadManager skips the missed periods instead of building a backlog.
     *
     * @note Exceptions from the frame callback are caught and logged rather than
     *       propagated, since this runs on the background thread with no caller able
     *       to observe or handle them.
     */
    void capture_loop_callback()
    {
        if (!is_connected_ || !is_initialized_ || shutdown_signaler_->should_shutdown()) {
            return;
        }

        image_.clear();
        const auto start = std::chrono::steady_clock::now();
        const int32_t ret = client_->GetImageSample(image_);
        const auto end = std::chrono::steady_clock::now();
        const double request_duration_ms = std::chrono::duration<double, std::milli>(end - start).count();

        std::shared_ptr<Frame> frame;
        FrameCallback frame_callback;
        {
            std::scoped_lock lock(data_mutex_);
            statistics_.requests++;
            statistics_.total_request_duration_ms += request_duration_ms;
            if (ret != 0 || image_.empty()) {
                statistics_.failed_requests++;
                statistics_.last_error_code = ret;
                return;
            }
            // latest_frame_ is only replaced by this thread, so comparing against it here is safe.
            if (skip_duplicates_ && latest_frame_ && image_ == latest_frame_->jpeg) {
                statistics_.duplicate_frames++;
                return;
            }
            frame = std::make_shared<Frame>();
            frame->jpeg.swap(image_);
            jpeg_resolution(frame->jpeg, frame->width, frame->height);
            frame->sequence = ++statistics_.new_frames;
            frame->reception_time = std::chrono::system_clock::now();
            frame->request_duration_ms = request_duration_ms;
            latest_frame_ = frame;
            frame_callback = frame_callback_;
        }

        // Outside the lock, so the callback can use the getters.
        if (frame_callback) {
            try {
                frame_callback(frame);
            } catch (const std::exception& e) {
                {
                    std::scoped_lock lock(data_mutex_);
                    statistics_.callback_exceptions++;
                }
                std::cerr << "DriverUnitreeVideohub frame callback error: " << e.what() << std::endl;
            }
        }
    }

    /**
     * @brief Starts the periodic background thread that drives capture_loop_callback().
     * @details Idempotent: a no-op if the thread already exists and is running.
     */
    void start_capture_thread()
    {
        if (capture_thread_ && capture_thread_->is_running()) {
            return;
        }
        capture_thread_ = std::make_unique<marinholab::sas::core::ThreadManager>(
            "unitree_videohub",
            request_period_,
            std::bind(&Impl::capture_loop_callback, this),
            marinholab::sas::core::ThreadManager::PRIORITY::NORMAL,
            -1
            );
        capture_thread_->start();
    }

    /// Stops and releases the capture thread, if one is running. A no-op otherwise.
    void stop_capture_thread()
    {
        if (capture_thread_) {
            capture_thread_->stop();
            capture_thread_.reset();
        }
    }
};


/**
 * @brief Constructs the wrapper. No network I/O happens here: the RPC client is
 *        created in connect().
 * @throws std::invalid_argument if shutdown_signaler is nullptr, service_name is
 *         empty, or request_period or rpc_timeout is not positive.
 */
DriverUnitreeVideohub::DriverUnitreeVideohub(const std::shared_ptr<marinholab::sas::core::ShutdownSignaler>& shutdown_signaler,
                                             const std::string& service_name,
                                             const double& request_period,
                                             const double& rpc_timeout,
                                             const bool& skip_duplicates)
{
    if (shutdown_signaler == nullptr) {
        throw std::invalid_argument("DriverUnitreeVideohub: shutdown_signaler must not be nullptr");
    }
    if (service_name.empty()) {
        throw std::invalid_argument("DriverUnitreeVideohub: service_name must not be empty");
    }
    if (!(request_period > 0.0)) {
        throw std::invalid_argument("DriverUnitreeVideohub: request_period must be positive, but it is "
                                    + std::to_string(request_period));
    }
    if (!(rpc_timeout > 0.0)) {
        throw std::invalid_argument("DriverUnitreeVideohub: rpc_timeout must be positive, but it is "
                                    + std::to_string(rpc_timeout));
    }
    impl_ = std::make_shared<Impl>(shutdown_signaler, service_name, request_period, rpc_timeout, skip_duplicates);
}

/**
 * @brief Stops the capture thread and releases the RPC client.
 */
DriverUnitreeVideohub::~DriverUnitreeVideohub()
{
    deinitialize();
    disconnect();
}

/**
 * @brief Creates and initializes the RPC client of videohub, and marks this object as
 *        connected. No request is sent: nothing checks here that videohub answers (a
 *        missing service shows up as failed requests, error 3104, once initialized).
 * @pre unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface)
 *      must already have been called by the owner.
 */
void DriverUnitreeVideohub::connect()
{
    if (impl_->is_connected_) {
        return; // already connected — don't re-Init or rebuild the client
    }
    impl_->client_ = std::make_unique<VideohubClient>(impl_->service_name_);
    impl_->client_->SetTimeout(static_cast<float>(impl_->rpc_timeout_));
    impl_->client_->Init();
    impl_->is_connected_ = true;
}

/**
 * @brief Starts the capture thread.
 * @throws std::runtime_error if connect() has not been called yet.
 */
void DriverUnitreeVideohub::initialize()
{
    if (!impl_->is_connected_) {
        throw std::runtime_error("DriverUnitreeVideohub::initialize: Cannot initialize: not connected");
    }
    impl_->is_initialized_ = true;
    impl_->start_capture_thread();
}

/**
 * @brief Stops and joins the capture thread. Takes up to rpc_timeout if a request is
 *        pending.
 * @note Safe to call multiple times; a no-op if not currently initialized.
 */
void DriverUnitreeVideohub::deinitialize()
{
    if (impl_->is_initialized_) {
        impl_->is_initialized_ = false;
        impl_->stop_capture_thread();
    }
}

/**
 * @brief Tears down the connection: calls deinitialize() first if still initialized,
 *        and releases the RPC client.
 * @note Safe to call multiple times; a no-op if not currently connected.
 */
void DriverUnitreeVideohub::disconnect()
{
    if (impl_->is_connected_) {
        deinitialize();
        impl_->client_.reset();
        impl_->is_connected_ = false;
    }
}

void DriverUnitreeVideohub::set_frame_callback(const FrameCallback& frame_callback)
{
    std::scoped_lock lock(impl_->data_mutex_);
    impl_->frame_callback_ = frame_callback;
}

std::shared_ptr<const DriverUnitreeVideohub::Frame> DriverUnitreeVideohub::get_latest_frame() const
{
    std::scoped_lock lock(impl_->data_mutex_);
    return impl_->latest_frame_;
}

DriverUnitreeVideohub::Statistics DriverUnitreeVideohub::get_statistics() const
{
    std::scoped_lock lock(impl_->data_mutex_);
    return impl_->statistics_;
}

std::string DriverUnitreeVideohub::get_service_name() const
{
    return impl_->service_name_;
}

std::string DriverUnitreeVideohub::get_error_description(const int32_t& code)
{
    switch (code) {
    case 0:    return "success";
    case 3102: return "send request error";
    case 3103: return "api is not registered";
    case 3104: return "call api timeout (no server answered: wrong service name, interface, or domain?)";
    case 3105: return "response api does not match";
    case 3106: return "response data error";
    case 3202: return "server internal error";
    case 3203: return "api not implemented by the server";
    case 3204: return "api parameter error";
    default:   return "unknown code";
    }
}
