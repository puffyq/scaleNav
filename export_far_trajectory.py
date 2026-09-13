import json
from pathlib import Path


ROOT = Path("/mnt/code/lab/yopo/OpenSeek")
OUT = ROOT / "scalenav_ws/results/far_20260912_220907"
OUT.mkdir(parents=True, exist_ok=True)
sessions = [
    ROOT / "log_scalenav/session_20260912_220910_714",
    ROOT / "log_scalenav/session_20260912_221320_581",
]

for trial, session in enumerate(sessions, 1):
    rows = []
    for line in (session / "index.jsonl").open(encoding="utf-8"):
        record = json.loads(line)
        if record.get("kind") != "odom":
            continue
        data = record["data"]
        position = data["position"]
        if not rows or sum((position[i] - rows[-1][i]) ** 2 for i in range(3)) > 0.25 ** 2:
            rows.append([float(position[0]), float(position[1]), float(position[2])])
    with (OUT / f"trial_{trial}.csv").open("w", encoding="utf-8") as stream:
        stream.write("x,y,z\n")
        stream.writelines("{:.6f},{:.6f},{:.6f}\n".format(*row) for row in rows)
    print(f"trial {trial}: {len(rows)} points")

