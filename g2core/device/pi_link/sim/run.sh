#!/bin/sh
# Host simulation of the Pi link's motion path: the real pi_link.cpp (with PI_LINK_MOTION=1) and
# the real st_prep_velocity_segment() from stepper.cpp, against a model of g2core's step
# generator. Checks step accuracy through reversals, the straight-line board stop, release and
# position sync, DISABLE and the velocity clamp. Needs g++ and zlib; Linux x86-64 (it maps the
# USART2/PDC registers at their SAM3X address, so it is built without PIE).
set -e
here=$(cd "$(dirname "$0")" && pwd)
src="$here/../../.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
mkdir -p "$out/stub"
for h in g2core.h config.h canonical_machine.h planner.h stepper.h encoder.h kinematics.h \
         spindle.h gpio.h report.h MotatePins.h MotateTimers.h; do
    echo '#include "sim_env.h"' > "$out/stub/$h"
done
cp "$here/sim_env.h" "$src/device/pi_link/pi_link.h" "$out/stub/"
sed 's/__asm__ volatile("dmb" ::: "memory")/__asm__ volatile("" ::: "memory")/' \
    "$src/device/pi_link/pi_link.cpp" > "$out/stub/pi_link_sim.cpp"
awk '/^stat_t st_prep_velocity_segment\(const float v0\[\], const float v1\[\], const uint16_t ticks\)/,/^}/' \
    "$src/stepper.cpp" > "$out/prep_extract.inc"
cp "$here/motion_sim.cpp" "$out/"
g++ -std=gnu++17 -O2 -Wall -Wno-unused-variable -no-pie -fno-pie -DPI_LINK_MOTION=1 -I"$out/stub" -I"$out" \
    -include array "$out/motion_sim.cpp" -lz -o "$out/motion_sim"
"$out/motion_sim"
