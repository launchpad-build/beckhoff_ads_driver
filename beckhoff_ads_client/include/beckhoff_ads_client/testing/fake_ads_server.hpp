// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * @file fake_ads_server.hpp
 * @brief In-process stand-in for a TwinCAT runtime, for testing ADS clients without a PLC.
 */
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace beckhoff_ads_client
{
namespace testing
{

/** @brief TCP port every ADS router listens on. */
inline constexpr std::uint16_t ADS_TCP_PORT = 48898;
/** @brief ADS error returned for an unknown symbol. */
inline constexpr std::uint32_t SYMBOL_NOT_FOUND = 0x710;
/** @brief ADS error returned for a symbol that may not be accessed. */
inline constexpr std::uint32_t DEVICE_INVALID_ACCESS = 0x704;

/**
 * @brief Answers ADS requests on 127.0.0.1:48898 from an in-memory symbol table.
 *
 * Serves read state, symbol handle by name, handle release, sum read and sum write, which is what
 * beckhoff_ads_client uses. Only one instance can listen at a time on a host; tests that use it must
 * not run concurrently with another user of the port.
 */
class FakeAdsServer
{
public:
  /** @brief Starts listening and serving; check listening() before relying on it. */
  FakeAdsServer()
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int enable = 1;
    ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(ADS_TCP_PORT);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    listening_ = ::bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0 &&
      ::listen(listener_, 4) == 0;
    if (listening_) {
      acceptor_ = std::thread([this] {accept_connections();});
    }
  }

  /** @brief Stops listening and closes every connection. */
  ~FakeAdsServer()
  {
    stopping_.store(true);
    ::shutdown(listener_, SHUT_RDWR);
    ::close(listener_);
    if (acceptor_.joinable()) {
      acceptor_.join();
    }
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      for (const int connection : connections_) {
        ::shutdown(connection, SHUT_RDWR);
      }
    }
    for (std::thread & worker : workers_) {
      worker.join();
    }
  }

  FakeAdsServer(const FakeAdsServer &) = delete;
  FakeAdsServer & operator=(const FakeAdsServer &) = delete;

  /** @brief Whether the server holds the ADS port. */
  bool listening() const
  {
    return listening_;
  }

  /** @brief Adds a zero-filled symbol of size bytes. */
  void add_symbol(const std::string & name, std::size_t size)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const std::uint32_t handle = next_handle_++;
    handles_[name] = handle;
    symbols_[handle] = Symbol{name, std::vector<std::uint8_t>(size, 0), std::nullopt};
  }

  /** @brief Stores value at byte offset of a symbol. */
  template<typename Value>
  void set_value(const std::string & name, std::size_t offset, Value value)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::memcpy(symbols_.at(handles_.at(name)).bytes.data() + offset, &value, sizeof(value));
  }

  /** @brief Loads the value at byte offset of a symbol. */
  template<typename Value>
  Value value(const std::string & name, std::size_t offset) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    Value result{};
    std::memcpy(&result, symbols_.at(handles_.at(name)).bytes.data() + offset, sizeof(result));
    return result;
  }

  /** @brief Makes every sum-request item for the symbol fail with error_code, or succeed again with nothing. */
  void fail_symbol(const std::string & name, std::optional<std::uint32_t> error_code)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    symbols_.at(handles_.at(name)).item_error = error_code;
  }

  /** @brief Number of sum-write items that wrote the symbol. */
  std::size_t writes_to(const std::string & name) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto found = write_counts_.find(name);
    return found == write_counts_.end() ? 0 : found->second;
  }

  /** @brief Number of sum writes received. */
  std::size_t sum_writes() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return sum_writes_;
  }

  /** @brief While false, requests are swallowed unanswered, as by a dead link. */
  void set_responding(bool responding)
  {
    responding_.store(responding);
  }

private:
  struct Symbol
  {
    std::string name;
    std::vector<std::uint8_t> bytes;
    std::optional<std::uint32_t> item_error;
  };

  static constexpr std::size_t AMS_TCP_HEADER_SIZE = 6;
  static constexpr std::size_t AMS_HEADER_SIZE = 32;

  static bool receive_exactly(int connection, std::span<std::uint8_t> buffer)
  {
    std::size_t received = 0;
    while (received < buffer.size()) {
      const ssize_t count = ::recv(connection, buffer.data() + received, buffer.size() - received, 0);
      if (count <= 0) {
        return false;
      }
      received += static_cast<std::size_t>(count);
    }
    return true;
  }

  template<typename Value>
  static Value load(std::span<const std::uint8_t> bytes, std::size_t offset)
  {
    Value result{};
    std::memcpy(&result, bytes.data() + offset, sizeof(result));
    return result;
  }

  template<typename Value>
  static void append(std::vector<std::uint8_t> & bytes, Value value)
  {
    const std::size_t offset = bytes.size();
    bytes.resize(offset + sizeof(value));
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
  }

  void accept_connections()
  {
    while (!stopping_.load()) {
      const int connection = ::accept(listener_, nullptr, nullptr);
      if (connection < 0) {
        return;
      }
      const int enable = 1;
      ::setsockopt(connection, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
      const std::lock_guard<std::mutex> lock(mutex_);
      connections_.insert(connection);
      workers_.emplace_back([this, connection] {serve(connection);});
    }
  }

  void serve(int connection)
  {
    std::vector<std::uint8_t> tcp_header(AMS_TCP_HEADER_SIZE);
    while (receive_exactly(connection, tcp_header)) {
      const std::uint32_t frame_length = load<std::uint32_t>(tcp_header, 2);
      std::vector<std::uint8_t> frame(frame_length);
      if (!receive_exactly(connection, frame)) {
        break;
      }
      if (!responding_.load()) {
        continue;
      }
      const std::vector<std::uint8_t> reply = respond(frame);
      std::vector<std::uint8_t> message;
      append<std::uint16_t>(message, 0);
      append<std::uint32_t>(message, static_cast<std::uint32_t>(AMS_HEADER_SIZE + reply.size()));
      message.insert(message.end(), frame.begin() + 8, frame.begin() + 16);
      message.insert(message.end(), frame.begin(), frame.begin() + 8);
      message.insert(message.end(), frame.begin() + 16, frame.begin() + 18);
      append<std::uint16_t>(message, 0x0005);
      append<std::uint32_t>(message, static_cast<std::uint32_t>(reply.size()));
      append<std::uint32_t>(message, 0);
      message.insert(message.end(), frame.begin() + 28, frame.begin() + 32);
      message.insert(message.end(), reply.begin(), reply.end());
      ::send(connection, message.data(), message.size(), MSG_NOSIGNAL);
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    connections_.erase(connection);
    ::close(connection);
  }

  std::vector<std::uint8_t> respond(std::span<const std::uint8_t> frame)
  {
    const std::uint16_t command = load<std::uint16_t>(frame, 16);
    const std::span<const std::uint8_t> data = frame.subspan(AMS_HEADER_SIZE);
    std::vector<std::uint8_t> reply;
    const std::lock_guard<std::mutex> lock(mutex_);
    if (command == 0x0004) {
      append<std::uint32_t>(reply, 0);
      append<std::uint16_t>(reply, 5);
      append<std::uint16_t>(reply, 0);
    } else if (command == 0x0003) {
      append<std::uint32_t>(reply, 0);
    } else if (command == 0x0009) {
      const std::uint32_t group = load<std::uint32_t>(data, 0);
      const std::uint32_t offset = load<std::uint32_t>(data, 4);
      const std::uint32_t write_length = load<std::uint32_t>(data, 12);
      const std::span<const std::uint8_t> written = data.subspan(16, write_length);
      std::vector<std::uint8_t> payload;
      std::uint32_t result = 0;
      if (group == 0xF003) {
        const std::string name(written.begin(), written.end());
        const auto found = handles_.find(name);
        if (found == handles_.end()) {
          result = SYMBOL_NOT_FOUND;
        } else {
          append<std::uint32_t>(payload, found->second);
        }
      } else if (group == 0xF080) {
        std::vector<std::uint8_t> values;
        for (std::uint32_t item = 0; item < offset; ++item) {
          const std::uint32_t handle = load<std::uint32_t>(written, item * 12 + 4);
          const std::uint32_t length = load<std::uint32_t>(written, item * 12 + 8);
          const auto found = symbols_.find(handle);
          std::uint32_t item_error = found == symbols_.end() ? SYMBOL_NOT_FOUND : found->second.item_error.value_or(0);
          append<std::uint32_t>(payload, item_error);
          for (std::uint32_t byte = 0; byte < length; ++byte) {
            values.push_back(found != symbols_.end() && byte < found->second.bytes.size() ? found->second.bytes[byte] : 0);
          }
        }
        payload.insert(payload.end(), values.begin(), values.end());
      } else if (group == 0xF081) {
        ++sum_writes_;
        std::size_t data_offset = offset * 12;
        for (std::uint32_t item = 0; item < offset; ++item) {
          const std::uint32_t handle = load<std::uint32_t>(written, item * 12 + 4);
          const std::uint32_t length = load<std::uint32_t>(written, item * 12 + 8);
          const auto found = symbols_.find(handle);
          std::uint32_t item_error = found == symbols_.end() ? SYMBOL_NOT_FOUND : found->second.item_error.value_or(0);
          if (item_error == 0) {
            std::memcpy(found->second.bytes.data(), written.data() + data_offset, length);
            ++write_counts_[found->second.name];
          }
          append<std::uint32_t>(payload, item_error);
          data_offset += length;
        }
      }
      append<std::uint32_t>(reply, result);
      append<std::uint32_t>(reply, static_cast<std::uint32_t>(payload.size()));
      reply.insert(reply.end(), payload.begin(), payload.end());
    } else {
      append<std::uint32_t>(reply, DEVICE_INVALID_ACCESS);
    }
    return reply;
  }

  int listener_{-1};
  bool listening_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> responding_{true};
  std::thread acceptor_;
  mutable std::mutex mutex_;
  std::set<int> connections_;
  std::vector<std::thread> workers_;
  std::map<std::string, std::uint32_t> handles_;
  std::map<std::uint32_t, Symbol> symbols_;
  std::map<std::string, std::size_t> write_counts_;
  std::size_t sum_writes_{0};
  std::uint32_t next_handle_{0x100};
};

}  // namespace testing
}  // namespace beckhoff_ads_client
