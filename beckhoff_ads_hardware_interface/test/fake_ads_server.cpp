#include "fake_ads_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace beckhoff_ads_hardware_interface
{
namespace test
{

  namespace
  {
    constexpr uint16_t CMD_READ_DEVICE_INFO = 1;
    constexpr uint16_t CMD_READ = 2;
    constexpr uint16_t CMD_WRITE = 3;
    constexpr uint16_t CMD_READ_STATE = 4;
    constexpr uint16_t CMD_READ_WRITE = 9;

    constexpr uint32_t GROUP_SYM_HNDBYNAME = 0xF003;
    constexpr uint32_t GROUP_SYM_VALBYHND = 0xF005;
    constexpr uint32_t GROUP_SYM_RELEASEHND = 0xF006;
    constexpr uint32_t GROUP_SYM_INFOBYNAMEEX = 0xF009;
    constexpr uint32_t GROUP_SUMUP_READ = 0xF080;
    constexpr uint32_t GROUP_SUMUP_WRITE = 0xF081;

    constexpr uint32_t ERR_SERVICE_NOT_SUPPORTED = 0x701;
    constexpr uint32_t ERR_INVALID_SIZE = 0x705;
    constexpr uint32_t ERR_SYMBOL_NOT_FOUND = 0x710;
    constexpr uint32_t ERR_INVALID_HANDLE = 0x711;

    constexpr size_t AMS_TCP_HEADER_SIZE = 6;
    constexpr size_t AMS_HEADER_SIZE = 32;

    uint16_t getU16(const uint8_t *p)
    {
      uint16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }

    uint32_t getU32(const uint8_t *p)
    {
      uint32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }

    void putU16(std::vector<uint8_t> &out, uint16_t v)
    {
      const auto *p = reinterpret_cast<const uint8_t *>(&v);
      out.insert(out.end(), p, p + sizeof(v));
    }

    void putU32(std::vector<uint8_t> &out, uint32_t v)
    {
      const auto *p = reinterpret_cast<const uint8_t *>(&v);
      out.insert(out.end(), p, p + sizeof(v));
    }

    bool readExact(int fd, uint8_t *buffer, size_t size)
    {
      size_t done = 0;
      while (done < size)
      {
        const ssize_t n = ::recv(fd, buffer + done, size - done, 0);
        if (n <= 0)
        {
          return false;
        }
        done += static_cast<size_t>(n);
      }
      return true;
    }

    bool writeAll(int fd, const std::vector<uint8_t> &buffer)
    {
      size_t done = 0;
      while (done < buffer.size())
      {
        const ssize_t n = ::send(fd, buffer.data() + done, buffer.size() - done, MSG_NOSIGNAL);
        if (n <= 0)
        {
          return false;
        }
        done += static_cast<size_t>(n);
      }
      return true;
    }

    std::vector<uint8_t> resultOnly(uint32_t result)
    {
      std::vector<uint8_t> out;
      putU32(out, result);
      return out;
    }

    std::vector<uint8_t> resultWithData(uint32_t result, const std::vector<uint8_t> &data)
    {
      std::vector<uint8_t> out;
      putU32(out, result);
      putU32(out, static_cast<uint32_t>(data.size()));
      out.insert(out.end(), data.begin(), data.end());
      return out;
    }
  } // namespace

  FakeAdsServer::FakeAdsServer()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0)
    {
      throw std::runtime_error("FakeAdsServer: socket() failed");
    }
    const int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
        ::listen(listen_fd_, 4) != 0)
    {
      ::close(listen_fd_);
      throw std::runtime_error("FakeAdsServer: bind()/listen() failed");
    }
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);

    accept_thread_ = std::thread([this]
                                 { acceptLoop(); });
  }

  FakeAdsServer::~FakeAdsServer()
  {
    stop_ = true;
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    if (accept_thread_.joinable())
    {
      accept_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (int fd : client_fds_)
      {
        ::shutdown(fd, SHUT_RDWR);
      }
    }
    for (auto &thread : client_threads_)
    {
      if (thread.joinable())
      {
        thread.join();
      }
    }
    for (int fd : client_fds_)
    {
      ::close(fd);
    }
  }

  void FakeAdsServer::addSymbol(const std::string &name, std::vector<uint8_t> value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    symbols_[name].value = std::move(value);
  }

  std::vector<uint8_t> FakeAdsServer::symbolValue(const std::string &name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = symbols_.find(name);
    return it == symbols_.end() ? std::vector<uint8_t>{} : it->second.value;
  }

  std::vector<SumItem> FakeAdsServer::lastSumRead() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_sum_read_;
  }

  std::vector<std::vector<SumItem>> FakeAdsServer::sumWrites() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sum_writes_;
  }

  void FakeAdsServer::refuseSymbolInfo()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    refuse_symbol_info_ = true;
  }

  size_t FakeAdsServer::symbolInfoRequests() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return symbol_info_requests_;
  }

  bool FakeAdsServer::waitForSumWrites(size_t count, std::chrono::milliseconds timeout) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return sum_write_cv_.wait_for(lock, timeout, [this, count]
                                  { return sum_writes_.size() >= count; });
  }

  void FakeAdsServer::acceptLoop()
  {
    while (!stop_)
    {
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd < 0)
      {
        return;
      }
      const int one = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      std::lock_guard<std::mutex> lock(mutex_);
      client_fds_.push_back(fd);
      client_threads_.emplace_back([this, fd]
                                   { serve(fd); });
    }
  }

  void FakeAdsServer::serve(int fd)
  {
    while (!stop_)
    {
      uint8_t tcp_header[AMS_TCP_HEADER_SIZE];
      if (!readExact(fd, tcp_header, sizeof(tcp_header)))
      {
        return;
      }
      const uint32_t length = getU32(tcp_header + 2);
      std::vector<uint8_t> ams(length);
      if (length < AMS_HEADER_SIZE || !readExact(fd, ams.data(), length))
      {
        return;
      }
      const std::vector<uint8_t> payload = handleFrame(ams);

      std::vector<uint8_t> response;
      putU16(response, 0);
      putU32(response, static_cast<uint32_t>(AMS_HEADER_SIZE + payload.size()));
      response.insert(response.end(), ams.begin() + 8, ams.begin() + 16); // target = request source
      response.insert(response.end(), ams.begin(), ams.begin() + 8);      // source = request target
      putU16(response, getU16(ams.data() + 16));                           // command id
      putU16(response, 0x0005);                                            // response, ADS command
      putU32(response, static_cast<uint32_t>(payload.size()));
      putU32(response, 0);                                                 // AMS error
      putU32(response, getU32(ams.data() + 28));                           // invoke id
      response.insert(response.end(), payload.begin(), payload.end());
      if (!writeAll(fd, response))
      {
        return;
      }
    }
  }

  std::vector<uint8_t> FakeAdsServer::handleFrame(const std::vector<uint8_t> &ams)
  {
    const uint16_t command = getU16(ams.data() + 16);
    const uint8_t *data = ams.data() + AMS_HEADER_SIZE;
    const size_t data_size = ams.size() - AMS_HEADER_SIZE;

    switch (command)
    {
    case CMD_READ_DEVICE_INFO:
    {
      std::vector<uint8_t> out;
      putU32(out, 0);
      out.push_back(3);
      out.push_back(1);
      putU16(out, 4026);
      const char name[16] = "FakeAdsServer";
      out.insert(out.end(), name, name + sizeof(name));
      return out;
    }
    case CMD_READ_STATE:
    {
      std::vector<uint8_t> out;
      putU32(out, 0);
      putU16(out, 5); // ADSSTATE_RUN
      putU16(out, 0);
      return out;
    }
    case CMD_WRITE:
    {
      if (data_size >= 12 && getU32(data) == GROUP_SYM_RELEASEHND)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        handles_.erase(getU32(data + 12));
        return resultOnly(0);
      }
      return resultOnly(ERR_SERVICE_NOT_SUPPORTED);
    }
    case CMD_READ:
      return resultWithData(ERR_SERVICE_NOT_SUPPORTED, {});
    case CMD_READ_WRITE:
    {
      if (data_size < 16)
      {
        return resultWithData(ERR_INVALID_SIZE, {});
      }
      const uint32_t write_length = getU32(data + 12);
      if (data_size < 16 + static_cast<size_t>(write_length))
      {
        return resultWithData(ERR_INVALID_SIZE, {});
      }
      return handleReadWrite(getU32(data), getU32(data + 4), getU32(data + 8), data + 16, write_length);
    }
    default:
      return resultOnly(ERR_SERVICE_NOT_SUPPORTED);
    }
  }

  std::vector<uint8_t> FakeAdsServer::handleReadWrite(uint32_t group, uint32_t offset, uint32_t read_length,
                                                      const uint8_t *data, uint32_t write_length)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (group == GROUP_SYM_HNDBYNAME)
    {
      std::string name(reinterpret_cast<const char *>(data), write_length);
      name = name.c_str(); // drop a trailing NUL, if any
      if (symbols_.find(name) == symbols_.end())
      {
        return resultWithData(ERR_SYMBOL_NOT_FOUND, {});
      }
      const uint32_t handle = next_handle_++;
      handles_[handle] = name;
      std::vector<uint8_t> out;
      putU32(out, handle);
      return resultWithData(0, out);
    }

    if (group == GROUP_SYM_INFOBYNAMEEX)
    {
      ++symbol_info_requests_;
      if (refuse_symbol_info_)
      {
        return resultWithData(ERR_SERVICE_NOT_SUPPORTED, {});
      }
      std::string name(reinterpret_cast<const char *>(data), write_length);
      name = name.c_str();
      const auto it = symbols_.find(name);
      if (it == symbols_.end())
      {
        return resultWithData(ERR_SYMBOL_NOT_FOUND, {});
      }
      // AdsSymbolEntry, then the name, an empty type name and an empty comment, each NUL-terminated.
      std::vector<uint8_t> entry;
      const uint32_t entry_length = 30 + static_cast<uint32_t>(name.size()) + 3;
      putU32(entry, entry_length);
      putU32(entry, 0x4040);
      putU32(entry, 0);
      putU32(entry, static_cast<uint32_t>(it->second.value.size()));
      putU32(entry, 65);
      putU32(entry, 0);
      putU16(entry, static_cast<uint16_t>(name.size()));
      putU16(entry, 0);
      putU16(entry, 0);
      entry.insert(entry.end(), name.begin(), name.end());
      entry.insert(entry.end(), 3, 0);
      if (entry.size() > read_length)
      {
        return resultWithData(ERR_INVALID_SIZE, {});
      }
      return resultWithData(0, entry);
    }

    if (group == GROUP_SUMUP_READ)
    {
      const uint32_t count = offset;
      if (write_length < count * 12)
      {
        return resultWithData(ERR_INVALID_SIZE, {});
      }
      std::vector<uint8_t> errors;
      std::vector<uint8_t> values;
      std::vector<SumItem> items;
      for (uint32_t i = 0; i < count; ++i)
      {
        const uint8_t *header = data + i * 12;
        SumItem item;
        item.index_group = getU32(header);
        item.length = getU32(header + 8);
        const auto handle_it = handles_.find(getU32(header + 4));
        uint32_t error = 0;
        std::vector<uint8_t> value(item.length, 0);
        if (item.index_group != GROUP_SYM_VALBYHND || handle_it == handles_.end())
        {
          error = ERR_INVALID_HANDLE;
        }
        else
        {
          item.symbol = handle_it->second;
          const auto &stored = symbols_.at(item.symbol).value;
          if (item.length > stored.size())
          {
            error = ERR_INVALID_SIZE;
          }
          else
          {
            std::memcpy(value.data(), stored.data(), item.length);
          }
        }
        putU32(errors, error);
        values.insert(values.end(), value.begin(), value.end());
        items.push_back(std::move(item));
      }
      last_sum_read_ = std::move(items);
      errors.insert(errors.end(), values.begin(), values.end());
      if (errors.size() > read_length)
      {
        return resultWithData(ERR_INVALID_SIZE, {});
      }
      return resultWithData(0, errors);
    }

    if (group == GROUP_SUMUP_WRITE)
    {
      const uint32_t count = offset;
      if (write_length < count * 12)
      {
        return resultWithData(ERR_INVALID_SIZE, {});
      }
      std::vector<uint8_t> errors;
      std::vector<SumItem> items;
      size_t data_offset = count * 12;
      for (uint32_t i = 0; i < count; ++i)
      {
        const uint8_t *header = data + i * 12;
        SumItem item;
        item.index_group = getU32(header);
        item.length = getU32(header + 8);
        if (data_offset + item.length > write_length)
        {
          return resultWithData(ERR_INVALID_SIZE, {});
        }
        item.data.assign(data + data_offset, data + data_offset + item.length);
        data_offset += item.length;
        const auto handle_it = handles_.find(getU32(header + 4));
        uint32_t error = 0;
        if (item.index_group != GROUP_SYM_VALBYHND || handle_it == handles_.end())
        {
          error = ERR_INVALID_HANDLE;
        }
        else
        {
          item.symbol = handle_it->second;
          auto &stored = symbols_.at(item.symbol).value;
          if (item.length > stored.size())
          {
            error = ERR_INVALID_SIZE;
          }
          else
          {
            std::memcpy(stored.data(), item.data.data(), item.length);
          }
        }
        putU32(errors, error);
        items.push_back(std::move(item));
      }
      sum_writes_.push_back(std::move(items));
      sum_write_cv_.notify_all();
      return resultWithData(0, errors);
    }

    return resultWithData(ERR_SERVICE_NOT_SUPPORTED, {});
  }

} // namespace test
} // namespace beckhoff_ads_hardware_interface
