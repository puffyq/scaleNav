#!/usr/bin/env python3
"""Reset the AirSim world through the same MessagePack RPC used by ScaleNav."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[1] / "src"
AIRSIM_RENDERER_SRC = (
    WORKSPACE_SRC / "controller_airsim" / "src" / "airsim_renderer"
)
sys.path.insert(0, str(AIRSIM_RENDERER_SRC))

from airsim_renderer.rpc import MessagePackRpcClient  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Reset AirSim before a ScaleNav run")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    client = MessagePackRpcClient(args.host, args.port, args.timeout)
    try:
        if not client.call("ping"):
            raise RuntimeError("AirSim ping returned false")
        client.call("reset")
    finally:
        client.close()

    print(f"AirSim reset complete: {args.host}:{args.port}", flush=True)


if __name__ == "__main__":
    main()
