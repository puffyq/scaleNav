# YOPO-Simple Snapshot Collection

The collector records training snapshots, not expert trajectories. YOPO-Simple
samples velocity, acceleration, and goals during training and computes the
physical trajectory cost from each scene's obstacle point cloud.

Start the copied UE map and AirSim RPC service first. Then collect a scene with
an existing point cloud:

```bash
PYTHONPATH=openseek:. \
  /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  -m openseek.data.snapshot_dataset \
  --output /mnt/code/lab/yopo/OpenSeek/data/TrainingData \
  --scene-id 0001 \
  --count 200 \
  --seed 1001 \
  --person-positions /mnt/code/lab/airsim/Colosseum/Unreal/Environments/BlocksV2/Saved/PersonSpawner/generated_people.json \
  --obstacle-ply /path/to/static_obstacles.ply \
  --airsim-root /mnt/code/lab/airsim/Colosseum/PythonClient
```

Alternatively, export static mesh vertices through the Colosseum RPC API:

```bash
PYTHONPATH=openseek:. \
  /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  -m openseek.data.snapshot_dataset \
  --output /mnt/code/lab/yopo/OpenSeek/data/TrainingData \
  --scene-id 0001 \
  --count 200 \
  --seed 1001 \
  --export-static-meshes \
  --airsim-root /mnt/code/lab/airsim/Colosseum/PythonClient
```

The result is:

```text
data/TrainingData/Scene_0001/
  data.toml
  tree.ply
  Textures/rgb_000000.png
  Textures/depth_000000.exr
```

The collector also back-projects each captured `DepthPlanar` frame into world
NED and appends a capped sample to `tree.ply`. This makes the training ESDF
match the geometry actually seen by the camera, including instanced foliage.
When `--person-positions` is supplied, UE `positionCm` entries are converted to
local-NED meters and capsule-like collision points are appended as well. The
static point cloud must be ASCII PLY (the format produced by
`--export-static-meshes`).

For an existing dataset, rebuild its maps without recapturing RGB-D:

```bash
bash scripts/19_rebuild_depth_maps.sh
```

Depth EXR values are meters. Each frame stores `depthMaxMeters=20.0`; the
OpenSeek text dataset converts it to the network range `[0, 1]` when loading.
RGB is retained for offline SCLIP/PEARL heatmap generation and is not an expert
label. The Colosseum bridge uses BGR byte order by default; pass
`--color-order rgb` only if the server is configured to return RGB bytes.

Validate before precomputing heatmaps or training:

```bash
PYTHONPATH=openseek:. \
  /mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  -m openseek.data.validate_snapshot_dataset data/TrainingData
```

The UE spawner writes the generated-person file to
`Saved/PersonSpawner/generated_people.json` after generation. If this file is
not present, the static mesh export does not include transient SkeletalMesh
actors, so the safety loss will not see people as obstacles.
