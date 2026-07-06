# Changelog

<!-- towncrier release notes start -->

## 1.4.0 (2026-07-06)

### Features

- ADS sum read and write failures now log the decoded error meaning. The first failure of an outage names the target PLC address, and a recovery line reports the outage duration once the link is stable again. Boolean command value changes are logged at debug level at the write boundary. (ads-error-context)


## 1.3.0 (2026-07-01)

### Features

- Move the blocking ADS SUM read and write off the control loop onto dedicated worker threads. `write()` marshals the latest command buffer, hands it to a writer thread, and returns. The writer sends only the newest buffer, so no backlog builds. A reader thread runs the SUM read and caches the result for a non-blocking `read()`. The control-loop cycle time no longer tracks PLC or network latency. This clears the controller-manager overrun warnings at high update rates. An optional `read_poll_period_ms` hardware parameter paces the reader to cap PLC load.

### Fixes

- Release cached PLC symbol handles while the ADS device is still alive, before shutdown resets it or reconfigure replaces it. Each handle deleter calls `DeleteSymbolHandle` through the device. Releasing a handle after the device is gone dereferenced freed memory and segfaulted on Ctrl-C.


## 1.2.0 (2026-05-28)

### Features

- Adopt the HardwareComponentInterfaceParams signature for on_init and chain SystemInterface::on_init so the base sets up component state before the interface initialises its clock.


## 1.1.1 (2026-05-19)

### Fixes

- Hold the AdsHandle returned by GetHandle for the lifetime of each ADSDataLayout so SYM_RELEASEHND no longer fires immediately on resolve. Struct and array-of-struct member symbols can now be read without crashing the worker thread.


## 1.1.0 (2026-04-29)

### Features

- Add launchpad-build scaffolding via shared-workflows bootstrap script: CHANGELOG, towncrier configuration, news fragments directory, and shared release and news-fragment workflows. (L3H-128)
