from __future__ import annotations

import argparse
import json
from pathlib import Path

from .snapshot_dataset import validate_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate an OpenSeek snapshot dataset.")
    parser.add_argument("data", type=Path)
    parser.add_argument(
        "--require-semantic",
        action="store_true",
        help="require one finite semantic_pearl_*.npy heatmap for every frame",
    )
    parser.add_argument(
        "--require-routes",
        action="store_true",
        help="require and validate routes.npz for every scene",
    )
    args = parser.parse_args()
    report = validate_dataset(
        args.data,
        require_semantic=args.require_semantic,
        require_routes=args.require_routes,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
