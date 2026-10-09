#!/bin/sh
# Host-side test suite for the Arduino C++ firmware.
#
# Compiles the firmware's .cpp files against the minimal Arduino shims in
# test/shim and a fake I2C / scripted clock, so parsing and driver logic can be
# verified without a board attached. The .ino is NOT compiled here - it needs
# the real esp32 core (see README for the arduino-cli command).
#
#   sh test/run.sh
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SHIM="$ROOT/test/shim"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

CXX=${CXX:-g++}
FW="$ROOT/wheelchair_tracker"   # sketch dir name must match the .ino
FLAGS="-std=c++17 -Wall -Wextra -Wno-unused-parameter -I $SHIM -I $FW"

echo "== AT transport =="
$CXX $FLAGS -o "$OUT/at" \
    "$ROOT/test/test_sim7600_at.cpp" "$FW/Sim7600At.cpp"
"$OUT/at"

echo
echo "== LIS2DH + NMEA =="
$CXX $FLAGS -o "$OUT/imu" \
    "$ROOT/test/test_lis2dh_nmea.cpp" \
    "$FW/lis2dh.cpp" "$FW/Nmea.cpp"
"$OUT/imu"
