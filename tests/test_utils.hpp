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
#
# ################################################################
*/

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace unitree_drivers_test
{

// A domain not used by the robots (0) nor by unitree_mujoco (1).
constexpr int kDomainId = 42;

/// Initializes DDS on the loopback interface, once per process.
inline void init_dds()
{
    static std::once_flag flag;
    std::call_once(flag, [] { unitree::robot::ChannelFactory::Instance()->Init(kDomainId, "lo"); });
}

/// A ShutdownSignaler with its own flag.
struct Signaler
{
    std::atomic_bool stop{false};
    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> ptr{
        std::make_shared<marinholab::sas::core::ShutdownSignaler>(&stop)};
};

/// Calls step() and checks condition() until it holds or the timeout expires.
inline bool wait_until(const std::function<bool()>& condition,
                       const std::function<void()>& step = {},
                       const std::chrono::milliseconds& timeout = std::chrono::milliseconds(5000))
{
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        if (step) {
            step();
        }
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

/// Keeps the last message received on a topic.
template <typename Msg>
class Listener
{
    unitree::robot::ChannelSubscriberPtr<Msg> subscriber_;
    mutable std::mutex mutex_;
    Msg latest_{};
    bool received_{false};

public:
    explicit Listener(const std::string& topic)
        : subscriber_{new unitree::robot::ChannelSubscriber<Msg>(topic)}
    {
        subscriber_->InitChannel([this](const void* msg) {
            std::scoped_lock lock(mutex_);
            latest_ = *static_cast<const Msg*>(msg);
            received_ = true;
        }, 1);
    }

    bool received() const
    {
        std::scoped_lock lock(mutex_);
        return received_;
    }

    void clear()
    {
        std::scoped_lock lock(mutex_);
        received_ = false;
    }

    Msg latest() const
    {
        std::scoped_lock lock(mutex_);
        return latest_;
    }
};

/// Publishes messages on a topic.
template <typename Msg>
class Talker
{
    unitree::robot::ChannelPublisherPtr<Msg> publisher_;

public:
    explicit Talker(const std::string& topic)
        : publisher_{new unitree::robot::ChannelPublisher<Msg>(topic)}
    {
        publisher_->InitChannel();
    }

    void write(const Msg& msg)
    {
        Msg copy = msg;
        publisher_->Write(copy);
    }
};

/// The CRC32 of the low-level commands: every 32-bit word of the message except the last one (the crc field).
template <typename Msg>
std::uint32_t expected_crc(const Msg& msg)
{
    const auto* words = reinterpret_cast<const std::uint32_t*>(&msg);
    const std::uint32_t length = (sizeof(Msg) >> 2) - 1;
    std::uint32_t crc = 0xFFFFFFFF;
    for (std::uint32_t i = 0; i < length; ++i) {
        for (std::uint32_t bit = 0; bit < 32; ++bit) {
            const bool overflow = (crc & 0x80000000u) != 0;
            crc <<= 1;
            if (overflow) {
                crc ^= 0x04c11db7u;
            }
            if (words[i] & (1u << (31 - bit))) {
                crc ^= 0x04c11db7u;
            }
        }
    }
    return crc;
}

} // namespace unitree_drivers_test
