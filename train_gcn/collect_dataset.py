#!/usr/bin/env python3
"""Collect graph samples and geometry labels from ScaleNav log sessions."""

from __future__ import annotations

import argparse
import glob
import os
import sys

import torch


SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "scalenav_ws", "scripts"))
from demo_gnn_frontier_policy import build_log_graph, load_log_frames  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs", nargs="*", default=None,
                        help="session directories; default: log_scalenav/session_*")
    parser.add_argument("--output", default="train_gcn/dataset.pt")
    parser.add_argument("--max-frames-per-session", type=int, default=0)
    args = parser.parse_args()

    sessions = args.logs or sorted(glob.glob("log_scalenav/session_*"))
    sessions = [path for path in sessions if os.path.isfile(os.path.join(path, "index.jsonl"))]
    if not sessions:
        raise SystemExit("no sessions containing index.jsonl")

    samples = []
    skipped = 0
    for session in sessions:
        try:
            frames = load_log_frames(session, args.max_frames_per_session)
        except (OSError, ValueError):
            skipped += 1
            continue
        for timing, snapshot, position in frames:
            try:
                data, target, planner_target, map_target = build_log_graph(
                    torch, _Data, timing, snapshot, position, "map")
            except (KeyError, ValueError, IndexError):
                skipped += 1
                continue
            samples.append({
                "x": data.x.cpu(),
                "edge_index": data.edge_index.cpu(),
                "edge_weight": data.edge_weight.cpu(),
                "frontier_index": data.frontier_index.cpu(),
                "frontier_columns": data.frontier_columns.cpu(),
                "safe_columns": data.safe_columns.cpu(),
                "target": int(target),
                "planner_target": int(planner_target),
                "map_target": int(map_target),
                "session": os.path.basename(session),
                "seq": int(timing["seq"]),
                "position": list(position),
            })
        print(f"session={session} frames={len(frames)} collected={len(samples)}")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    torch.save({"schema": "scalenav_gcn_dataset.v1", "samples": samples,
                "sessions": sessions}, args.output)
    print(f"wrote={args.output} samples={len(samples)} skipped={skipped}")


class _Data:
    """Minimal Data constructor expected by build_log_graph."""

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


if __name__ == "__main__":
    main()
