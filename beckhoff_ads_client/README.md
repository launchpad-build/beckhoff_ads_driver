# beckhoff_ads_client

A plain C++20 library (no ROS) that exchanges Beckhoff TwinCAT PLC symbols over ADS: one sum read and one sum write per cycle on background threads, with outage handling, heartbeat and statistics. `beckhoff_ads_hardware_interface` is built on it.

## Usage

```cpp
#include "beckhoff_ads_client/ads_client.hpp"

beckhoff_ads_client::ClientSettings settings;
settings.connection = {"192.168.1.20", "192.168.1.20.1.1", 851, "192.168.1.10.1.1"};

std::vector<beckhoff_ads_client::SymbolElement> reads(1);
reads[0].symbol = "MAIN.axis_position";
reads[0].type = beckhoff_ads_client::PlcType::Lreal;

beckhoff_ads_client::AdsClient client;
if (client.configure(settings, reads, {}).succeeded() && client.start()) {
  std::vector<double> values(reads.size());
  client.latest_sample(values);
}
```

`testing/fake_ads_server.hpp` is an in-process stand-in for a PLC, for tests.

## Tests

```bash
colcon test --packages-select beckhoff_ads_client
```
