#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace beckhoff_ads_hardware_interface
{
namespace test
{

  /**
   * @brief One item of a SUM read or SUM write request, resolved to its PLC symbol
   */
  struct SumItem
  {
    std::string symbol;
    uint32_t index_group{0};
    uint32_t length{0};
    std::vector<uint8_t> data; // SUM write only
  };

  /**
   * @brief Minimal ADS server on localhost that answers the requests the hardware interface makes
   *
   * Serves symbol handles, SUM reads and SUM writes against an in-memory symbol table and
   * records every SUM request it receives.
   */
  class FakeAdsServer
  {
  public:
    FakeAdsServer();
    ~FakeAdsServer();

    FakeAdsServer(const FakeAdsServer &) = delete;
    FakeAdsServer &operator=(const FakeAdsServer &) = delete;

    /**
     * @brief Adds a PLC symbol with its initial value; its size is the value's size
     *
     * @param name The symbol name as the PLC reports it
     * @param value The initial bytes of the symbol
     */
    void addSymbol(const std::string &name, std::vector<uint8_t> value);

    /**
     * @brief The TCP port the server listens on
     *
     * @returns The port on 127.0.0.1
     */
    uint16_t port() const { return port_; }

    /**
     * @brief The current bytes of a symbol
     *
     * @param name The symbol name
     * @returns The symbol's bytes, empty when unknown
     */
    std::vector<uint8_t> symbolValue(const std::string &name) const;

    /**
     * @brief The items of the most recent SUM read request
     *
     * @returns The items in request order
     */
    std::vector<SumItem> lastSumRead() const;

    /**
     * @brief Every SUM write request received, oldest first
     *
     * @returns One item list per request, in request order
     */
    std::vector<std::vector<SumItem>> sumWrites() const;

    /**
     * @brief Makes every later symbol information request fail with ADSERR_DEVICE_SRVNOTSUPP
     */
    void refuseSymbolInfo();

    /**
     * @brief How many symbol information requests arrived
     *
     * @returns The count since the server started
     */
    size_t symbolInfoRequests() const;

    /**
     * @brief Waits until at least the given number of SUM writes arrived
     *
     * @param count The number of SUM writes to wait for
     * @param timeout How long to wait
     * @returns True when that many arrived in time
     */
    bool waitForSumWrites(size_t count, std::chrono::milliseconds timeout) const;

  private:
    struct Symbol
    {
      std::vector<uint8_t> value;
    };

    void acceptLoop();
    void serve(int fd);
    std::vector<uint8_t> handleFrame(const std::vector<uint8_t> &ams);
    std::vector<uint8_t> handleReadWrite(uint32_t group, uint32_t offset, uint32_t read_length,
                                         const uint8_t *data, uint32_t write_length);

    int listen_fd_{-1};
    uint16_t port_{0};
    std::atomic<bool> stop_{false};
    std::thread accept_thread_;
    std::vector<std::thread> client_threads_;
    std::vector<int> client_fds_;

    mutable std::mutex mutex_;
    mutable std::condition_variable sum_write_cv_;
    std::map<std::string, Symbol> symbols_;
    std::map<uint32_t, std::string> handles_;
    uint32_t next_handle_{0x1000};
    std::vector<SumItem> last_sum_read_;
    std::vector<std::vector<SumItem>> sum_writes_;
    size_t symbol_info_requests_{0};
    bool refuse_symbol_info_{false};
  };

} // namespace test
} // namespace beckhoff_ads_hardware_interface
