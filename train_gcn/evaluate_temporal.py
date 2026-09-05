#!/usr/bin/env python3
"""Tune causal direction smoothing on validation sessions and evaluate once on test."""

from __future__ import annotations

import argparse

import torch

from train import (accuracy_metrics, build_model, direction_metrics,
                   sample_to_device, session_groups)


def predict(model, samples, device, switch_penalty):
    predictions = []
    targets = []
    with torch.no_grad():
        for group in session_groups(samples).values():
            hidden = None
            previous = -1
            for raw_sample in group:
                sample = sample_to_device(raw_sample, device)
                logits, hidden = model(sample, hidden, previous)
                if previous >= 0 and switch_penalty > 0:
                    column_distance = torch.arange(5, device=device).sub(previous).abs()
                    logits = logits - switch_penalty * column_distance
                previous = int(logits.argmax())
                predictions.append(previous)
                targets.append(int(sample["target"]))
    return predictions, targets


def report(name, predictions, targets):
    accuracy, macro = accuracy_metrics(predictions, targets)
    within_one, mae = direction_metrics(predictions, targets)
    confusion = [[0] * 5 for _ in range(5)]
    for prediction, target in zip(predictions, targets):
        confusion[target][prediction] += 1
    print(f"{name} accuracy={accuracy:.3f} macro_accuracy={macro:.3f} "
          f"within_one={within_one:.3f} column_mae={mae:.3f}")
    print("confusion_rows_gt=" + ";".join(",".join(map(str, row)) for row in confusion))
    return accuracy, macro


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--device", default="cuda", choices=("cpu", "cuda"))
    parser.add_argument("--penalties", type=float, nargs="*",
                        default=(0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.6))
    args = parser.parse_args()

    device = torch.device(args.device)
    bundle = torch.load(args.dataset, weights_only=False)
    checkpoint = torch.load(args.model, map_location=device, weights_only=False)
    samples = bundle["samples"]
    val_sessions = set(checkpoint["val_sessions"])
    test_sessions = set(checkpoint["test_sessions"])
    val = [sample for sample in samples if sample["session"] in val_sessions]
    test = [sample for sample in samples if sample["session"] in test_sessions]
    model = build_model(int(samples[0]["x"].shape[1]), checkpoint["architecture"]).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()

    candidates = []
    for penalty in args.penalties:
        predictions, targets = predict(model, val, device, penalty)
        accuracy, macro = accuracy_metrics(predictions, targets)
        candidates.append((macro, accuracy, -penalty, penalty))
        print(f"validation penalty={penalty:.3f} accuracy={accuracy:.3f} macro_accuracy={macro:.3f}")
    penalty = max(candidates)[-1]
    print(f"selected_penalty={penalty:.3f}")
    predictions, targets = predict(model, test, device, penalty)
    report("test", predictions, targets)


if __name__ == "__main__":
    main()
