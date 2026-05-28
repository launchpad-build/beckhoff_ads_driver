# Changelog

<!-- towncrier release notes start -->

## 1.2.0 (2026-05-28)

### Features

- Adopt the HardwareComponentInterfaceParams signature for on_init and chain SystemInterface::on_init so the base sets up component state before the interface initialises its clock.


## 1.1.1 (2026-05-19)

### Fixes

- Hold the AdsHandle returned by GetHandle for the lifetime of each ADSDataLayout so SYM_RELEASEHND no longer fires immediately on resolve. Struct and array-of-struct member symbols can now be read without crashing the worker thread.


## 1.1.0 (2026-04-29)

### Features

- Add launchpad-build scaffolding via shared-workflows bootstrap script: CHANGELOG, towncrier configuration, news fragments directory, and shared release and news-fragment workflows. (L3H-128)
