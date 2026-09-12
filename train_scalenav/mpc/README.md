# YOPO ordered-bubble MPC prototype

This prototype keeps the complete route out of YOPO. The original YOPO-Simple
network receives depth, motion, and the route-derived local goal. Its terminal
state proposals and ordered route bubbles are passed to leap-c/acados.

Runtime environment:

```bash
export ACADOS_SOURCE_DIR=/mnt/code/lab/yopo/leap-c/external/acados
export LD_LIBRARY_PATH=/mnt/code/lab/yopo/leap-c/external/acados/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH=/mnt/code/lab/yopo/OpenSeek/train_scalenav:/mnt/code/lab/yopo/leap-c:/mnt/code/lab/yopo/leap-c/external/acados/interfaces/acados_template:${PYTHONPATH:-}
```

Run the differentiability and constraint regression:

```bash
cd /mnt/code/lab/yopo/OpenSeek/train_scalenav
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python -m pytest -q mpc/test_ordered_bubble_mpc.py
```

Run the 50-route, cross-scene point-cloud comparison:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python -m mpc.evaluate_yopo_mpc \
  --data /mnt/code/lab/yopo/OpenSeek/train_scalenav/dataset/train_large_001 \
  --output /mnt/code/lab/yopo/OpenSeek/train_scalenav/tmp/mpc_002_yopo_simple \
  --max-samples 50 --batch-size 16
```

The viewer overlays three results:

- purple: original YOPO-Simple fifth-order polynomial;
- green: original top-1 terminal proposal after bubble MPC;
- red: point-cloud-certified MPC selection, with route-center recovery only
  when none of the 15 YOPO proposals is certified.

The current numbered result is a feasibility result, not a training-ready
improvement. Raw top-1 MPC increases collision rate. Four of ten samples with
no certified MPC path still contain a safe original polynomial, showing that
the current stage-timed bubble OCP can damage an otherwise safe YOPO intent.

After preserving that safe-original fallback and aligning the execution limits
with the sampled state envelope (`v <= 9`, `a <= 12`, `jerk <= 40`), the full
900-route run is recorded at:

`../tmp/mpc_004_dynamics_fixed_full/comparison_report.json`

It reports 7.11% collision for certified MPC selection versus 25.22% for the
original YOPO-Simple polynomial on the same routes. This is a substantial
improvement, but not a zero-collision claim: 64 routes still have an initial
state that cannot be redirected safely inside the 1.667 s horizon under the
current point-cloud and dynamics contract.
