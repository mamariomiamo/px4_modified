# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## 0.0.2 - 2019-12-03

### Changed
- in file "src/modules/mc\_pos\_control/mc\_pos\_control\_main.cpp", change the condition for using RPT control by adding safety checks

### Changed

## 0.0.1 - 2019-12-02

### Added
- in file "msg/position\_setpoint.msg", added one more SETPOINT_TYPE, "SETPOINT\_TYPE\_RPT"
- in file "src/modules/mavlink/mavlink\_receiver.cpp", added flag and type mask to determine "SETPOINT\_TYPE\_RPT"
- in file "src/modules/mc\_pos\_control/mc\_pos\_control\_main.cpp", added RPT control for offboard control
