#!/usr/bin/env python3
"""Aggregate repeated-test summary.csv files, retaining failed-flight distance."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def stats(rows: list[dict[str, str]], key: str) -> tuple[float | None, float | None]:
    values = [value for row in rows if (value := number(row, key)) is not None]
    if not values:
        return None, None
    return statistics.fmean(values), statistics.stdev(values) if len(values) > 1 else 0.0


FIELDS = [
    "method", "batch", "n", "success", "collision", "timeout",
    "success_rate_percent", "collision_rate_percent", "timeout_rate_percent",
    "successful_duration_mean_s", "successful_duration_sd_s",
    "successful_path_mean_m", "successful_path_sd_m",
    "successful_average_speed_mean_mps", "successful_average_speed_sd_mps",
    "successful_max_speed_mean_mps", "successful_max_speed_sd_mps",
    "failure_observed_duration_mean_s", "failure_observed_duration_sd_s",
    "failure_observed_path_mean_m", "failure_observed_path_sd_m",
    "failure_observed_average_speed_mean_mps", "failure_observed_average_speed_sd_mps",
    "failure_observed_max_speed_mean_mps", "failure_observed_max_speed_sd_mps",
]


def aggregate(method: str, batch: str, rows: list[dict[str, str]]) -> dict[str, object]:
    valid = [row for row in rows if row.get("outcome") in {"success", "collision", "timeout"}]
    successful = [row for row in valid if row.get("outcome") == "success"]
    failed = [row for row in valid if row.get("outcome") in {"collision", "timeout"}]
    collisions = [row for row in valid if row.get("outcome") == "collision"]
    timeouts = [row for row in valid if row.get("outcome") == "timeout"]
    result: dict[str, object] = {
        "method": method,
        "batch": batch,
        "n": len(valid),
        "success": len(successful),
        "collision": len(collisions),
        "timeout": len(timeouts),
        "success_rate_percent": 100.0 * len(successful) / len(valid) if valid else None,
        "collision_rate_percent": 100.0 * len(collisions) / len(valid) if valid else None,
        "timeout_rate_percent": 100.0 * len(timeouts) / len(valid) if valid else None,
    }
    names = {"duration_s": "duration", "path_m": "path", "average_speed_mps": "average_speed", "max_speed_mps": "max_speed"}
    for prefix, group in (("successful", successful), ("failure_observed", failed)):
        for source, name in names.items():
            mean, sd = stats(group, source)
            suffix = "s" if source == "duration_s" else "m" if source == "path_m" else "mps"
            result[f"{prefix}_{name}_mean_{suffix}"] = mean
            result[f"{prefix}_{name}_sd_{suffix}"] = sd
    return result


def tex_value(row: dict[str, object], prefix: str, metric: str) -> str:
    suffix = {"path": "m", "duration": "s", "average_speed": "mps", "max_speed": "mps"}[metric]
    mean = row.get(f"{prefix}_{metric}_mean_{suffix}")
    sd = row.get(f"{prefix}_{metric}_sd_{suffix}")
    if mean is None:
        return "--"
    return f"${float(mean):.2f}\\pm{float(sd):.2f}$"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", action="append", required=True, metavar="METHOD=SUMMARY_CSV")
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-tex", type=Path)
    args = parser.parse_args()
    records: list[dict[str, object]] = []
    for spec in args.batch:
        method, raw_path = spec.split("=", 1)
        path = Path(raw_path)
        records.append(aggregate(method, path.parent.name, read_rows(path)))
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=FIELDS, extrasaction="ignore", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(records)
    if args.output_tex:
        lines = [
            r"\begin{tabular}{lcccccccc}", r"\toprule",
            "Method & Success (\\%) & Collision (\\%) & Timeout (\\%) & $L_{\\rm succ}$ (m) & $L_{\\rm obs}$ (m) & Time (s) & Avg. speed (m/s) & Max. speed (m/s) " + r"\\",
            r"\midrule",
        ]
        for row in records:
            lines.append(
                f"{row['method']} & {float(row['success_rate_percent']):.0f} & "
                f"{float(row['collision_rate_percent']):.0f} & {float(row['timeout_rate_percent']):.0f} & "
                f"{tex_value(row, 'successful', 'path')} & "
                f"{tex_value(row, 'failure_observed', 'path')} & "
                f"{tex_value(row, 'successful', 'duration')} & "
                f"{tex_value(row, 'successful', 'average_speed')} & "
                f"{tex_value(row, 'successful', 'max_speed')} " + r"\\"
            )
        lines.extend([r"\bottomrule", r"\end{tabular}", r"\textit{\footnotesize $L_{\rm obs}$ is collision/timeout-truncated observed path, not completion path.}"])
        args.output_tex.parent.mkdir(parents=True, exist_ok=True)
        args.output_tex.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
