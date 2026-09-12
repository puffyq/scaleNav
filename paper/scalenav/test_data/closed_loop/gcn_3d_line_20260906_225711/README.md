# Map4 3-D Line-Bypass Backup Experiment

This record preserves the post-training closed-loop evaluation of the Map4
3-D frontier GCN. It is a backup experiment and is not included in the main
paper tables.

## Configuration

- batch: `run_20260906_225711_315755`
- stack: `gcn`
- checkpoint: `train_gcn/frontier_gcn_map4_3d_line.pt`
- checkpoint SHA-256:
  `d6b3c020b5a892cddecee1b82f6eb4bdff55cbd0d94589462c8338bc0ca40882`
- mission: `(0, 0, 3.5)` m to `(0, 140, 3.5)` m
- prompt: `line`
- timeout: `120 s`
- `GRAPH_FIXED_LAYER=false`
- `FIXED_ALTITUDE=false`

The experiment tests height-changing navigation around the line-obstacle
course. Although the start and goal share the same altitude, the graph and
YOPO executor are not constrained to a fixed planning layer.

## Batch result

- valid trials: 10
- success: 80% (8/10)
- collision: 20% (2/10)
- timeout: 0%
- successful path: `167.399 +/- 13.825 m`
- successful duration: `36.975 +/- 5.302 s`
- successful average speed: `4.564 +/- 0.327 m/s`
- successful maximum speed: `6.573 +/- 0.197 m/s`
- successful final error: `0.109 +/- 0.047 m`
- successful path efficiency: `84.129 +/- 6.899%`
- failed observed path: `10.463 +/- 0.628 m`

Successful trials were 1, 3, 4, 5, 6, 7, 8, and 9. Trials 2 and 10
collided early after 10.907 m and 10.019 m, respectively.

## 3-D trajectory evidence

The odometry logs show substantial vertical motion in every successful run:

| Trial | Altitude range (m) | Vertical span (m) |
| ---: | ---: | ---: |
| 1 | 1.720--3.528 | 1.808 |
| 3 | 1.230--4.043 | 2.813 |
| 4 | 1.296--3.682 | 2.386 |
| 5 | 2.044--3.775 | 1.731 |
| 6 | 1.353--3.737 | 2.384 |
| 7 | 1.945--3.930 | 1.984 |
| 8 | 1.824--3.581 | 1.758 |
| 9 | 1.377--4.311 | 2.934 |

Trial 1 is the shortest successful run: 149.203 m in 30.934 s, with
4.823 m/s average speed, 6.215 m/s maximum speed, 0.108 m final error, and
93.832% path efficiency. Its altitude changes by 1.808 m while completing
the line-bypass mission without collision.

These results demonstrate that the deployed stack can construct and execute
a non-fixed-height route in this Map4 line-obstacle task. They do not by
themselves establish general performance in arbitrary 3-D environments.

## Records

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes
- `trial_*.json`: per-trial terminal records
- raw flight logs: `log_scalenav/session_20260906_225714_272` and the session
  paths listed in `summary.csv`
- runner: `scalenav_ws/src/aut_test/run_3d_0_140.sh`
