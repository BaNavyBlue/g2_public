#!/bin/bash
# Driven by bench-motor4.hal: start the command at motor 4's current count, take control,
# run 8 s of moves, give control back. Samples (cmd fb error vel in-control state, one line
# per period) go to bench-motor4.dat.
# It swings motor 4 +-4000 steps at 15,000 steps/s: on a mounted 4th axis that is +-225 degrees
# at about 2.3 rev/s. Only run it with nothing attached to motor 4, and say so with MOTOR4_FREE=1.
if [ "$MOTOR4_FREE" != 1 ]; then
    echo "bench-motor4: not run. Unplug the 4th axis from motor 4, then set MOTOR4_FREE=1." >&2
    exit 1
fi
sleep 0.5
halcmd setp sum2.0.in1 "$(halcmd getp pilink.3.counts)"
halcmd setp limit3.0.load 1; sleep 0.05; halcmd setp limit3.0.load 0
halsampler -n 11000 > bench-motor4.dat &
S=$!
sleep 0.2
halcmd setp pilink.enable 1
sleep 0.3
halcmd setp siggen.0.amplitude 4000
sleep 1
halcmd setp pilink.read.tmax 0; halcmd setp pilink.write.tmax 0; halcmd setp servo-thread.tmax 0
sleep 7
halcmd setp siggen.0.amplitude 0
sleep 1.2
halcmd setp pilink.enable 0
sleep 0.5
kill $S 2>/dev/null
wait $S 2>/dev/null
