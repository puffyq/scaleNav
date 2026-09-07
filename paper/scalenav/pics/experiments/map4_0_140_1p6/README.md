# Map4 GCN-Training Follow-up (Backup Experiment)

This directory contains a backup result collected after training the Map4
35-m frontier GCN. It is retained as a candidate experiment and is not part
of the main paper tables.

## Single-trial result

- stack: `gcn`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- semantic query: `powerline`
- batch: `run_20260906_124506_1688186`
- flight log: `log_scalenav/session_20260906_124509_370`
- valid trials: `1/1`
- success: `100%` (1/1)
- collision: `0%`
- timeout: `0%`
- duration: `35.606 s`
- path length: `153.325 m`
- average speed: `4.306 m/s`
- maximum speed: `5.652 m/s`
- final position error: `0.105 m`
- path efficiency: `91.309%`

The run used `train_gcn/frontier_gcn_map4_35m.pt`. Relative to the earlier
ten-trial Map4 GCN batch, this trial has a shorter path and higher path
efficiency, but it also uses a lower-speed configuration and contains only one
valid trial. It is therefore evidence of a promising post-training result,
not an aggregate performance estimate.

## Reproducibility records

- `gcn_post_training_metrics.json`: copied from
  `scalenav_ws/src/aut_test/results/run_20260906_124506_1688186/metrics.json`
- `gcn_post_training_summary.csv`: copied from the same batch directory
- `gcn_post_training_config.json`: batch configuration snapshot
- `obstacle_map_trajectory_gcn_post_training.{png,pdf}`: exact-footprint
  obstacle backdrop with the trajectory from this post-training trial.
- `obstacle_map_trajectory_best.{png,pdf}`: earlier Map4 representative
  trajectory retained as a separate candidate.

The obstacle backdrop is a privileged RGB-D survey of the simulator scene,
not UE mesh ground truth.
