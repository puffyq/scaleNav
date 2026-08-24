#!/usr/bin/env python3
"""Deterministic cadence contract for the online EPIC pipeline."""

import argparse
import json
from pathlib import Path


RATES = {"depth": 10, "semantic": 2, "planner": 5, "skeleton": 5}
DURATION_S = 12


def simulate() -> dict:
    counts = {name: DURATION_S * rate + 1 for name, rate in RATES.items()}
    skeleton_period = 1.0 / RATES["skeleton"]
    rebuild_latency = 0.08
    max_graph_age = skeleton_period + rebuild_latency
    checks = {
        "stream_cadence": counts == {
            "depth": 121, "semantic": 25, "planner": 61, "skeleton": 61
        },
        "rebuild_has_no_backlog": rebuild_latency < skeleton_period,
        "planner_sees_fresh_graph": max_graph_age <= 0.3,
        "semantic_arrives_before_first_route_update": 1.0 / RATES["semantic"] <= 2.0,
    }
    return {
        "duration_s": DURATION_S,
        "rates_hz": RATES,
        "counts": counts,
        "rebuild_latency_s": rebuild_latency,
        "max_graph_age_s": max_graph_age,
        "checks": checks,
    }


def render_html(result: dict) -> str:
    rows = "".join(
        f"<tr><td>{name}</td><td class='{'pass' if passed else 'fail'}'>"
        f"{'PASS' if passed else 'FAIL'}</td></tr>"
        for name, passed in result["checks"].items()
    )
    payload = json.dumps(result, indent=2)
    return f"""<!doctype html><html><head><meta charset='utf-8'>
<title>EPIC Online Contract</title><style>
body{{font:14px sans-serif;margin:24px;background:#111827;color:#e5e7eb}}
table{{border-collapse:collapse}}td{{padding:8px 16px;border-bottom:1px solid #374151}}
.pass{{color:#34d399}}.fail{{color:#f87171}}pre{{background:#1f2937;padding:16px}}
</style></head><body><h1>EPIC Online Contract</h1><table>{rows}</table>
<pre>{payload}</pre></body></html>"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", required=True, type=Path)
    parser.add_argument("--html", required=True, type=Path)
    args = parser.parse_args()
    result = simulate()
    args.json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    args.html.write_text(render_html(result), encoding="utf-8")
    passed = sum(result["checks"].values())
    print(f"simulation: {passed}/{len(result['checks'])} checks passed")
    return 0 if all(result["checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
