# ScaleNav repeated AirSim test

`run_0_140.sh` repeatedly performs an isolated one-way flight from
`(0, 0, 1.6)` to `(0, 140, 1.6)`:

1. Call AirSim's MessagePack-RPC `reset` method.
2. Start a fresh ScaleNav stack and log session.
3. Wait for odometry and `collision=false`.
4. Call `/scalenav/reset_sim` to clear controller state and any latched stop.
5. Publish `/goal_pose` and wait until the aircraft is within `0.5 m` and below
   `0.3 m/s`, or until timeout/collision.
6. Stop the complete stack, record the result, and repeat.

AirSim must already be running. The default batch contains 10 trials; stop
cleanly at any point with `Ctrl-C`. Use `--count 0` for an unlimited run.

For a normal manual run, `scalenav_ws/scripts/start.sh` also resets AirSim by
default before launching any ROS nodes. Set `AIRSIM_RESET_ON_START=false` only
when deliberately preserving the current AirSim world state. The ROS service
`/scalenav/reset_sim` resets the local UAV dynamics and emergency-stop latch;
it does not reset AirSim itself.

The Bash default is controlled by `TEST_COUNT=10` near the top of
`run_0_140.sh`; edit that value to change the persistent default.

```bash
bash scalenav_ws/src/aut_test/run_0_140.sh
```

Run a finite batch or disable semantic planning with:

```bash
bash scalenav_ws/src/aut_test/run_0_140.sh --count 10 --timeout 90
bash scalenav_ws/src/aut_test/run_0_140.sh --count 10 --no-semantic
```

Validate dependencies and configuration without resetting AirSim or launching
ROS nodes:

```bash
bash scalenav_ws/src/aut_test/run_0_140.sh --dry-run
```

Each invocation creates `results/run_<time>_<pid>/summary.csv`, one JSON result
per trial, and one full stack console log per trial. ScaleNav's sensor and graph
logs remain in `log_scalenav/session_*`, with the matching session path recorded
in both CSV and JSON.
