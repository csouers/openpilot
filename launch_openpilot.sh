#!/usr/bin/env bash

# pin the platform for fingerprinting (opendbc car_helpers -> CarParams.FingerprintSource.fixed).
# CarParamsCache is cleared on every manager start, so without this the VIN/FW query re-runs and
# can land on the wrong Honda variant. Lives here so it survives reboots and ignition cycles.
export FINGERPRINT="${FINGERPRINT:-HONDA_CIVIC_BOSCH}"

exec ./launch_chffrplus.sh
