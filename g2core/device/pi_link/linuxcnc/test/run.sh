#!/bin/sh
# Host test of the LinuxCNC driver's door hold (driver_test.c): builds pilink.c against hal_stub.h
# and runs it against a model of the board over a pseudo-terminal. Needs gcc; Linux.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
for h in rtapi.h rtapi_app.h hal.h; do
    echo '#include "hal_stub.h"' > "$out/$h"
done
gcc -std=gnu11 -O1 -Wall -Wno-unused-function -I"$out" -I"$here" -I"$here/.." "$here/driver_test.c" -lutil -o "$out/driver_test"
"$out/driver_test"
