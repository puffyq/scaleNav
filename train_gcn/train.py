#!/usr/bin/env python3
"""Train and evaluate a GCN five-column frontier policy."""

from __future__ import annotations

import argparse
from collections import defaultdict
import copy
import random

import torch
from torch import nn


class WeightedGCN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.linear = nn.Linear(input_dim, output_dim)

    def forward(self, x, edge_index, edge_weight):
        source, target = edge_index
        messages = x[source] * edge_weight[:, None]
        # Match GCNConv's self-loop behavior so every node retains its own
        # candidate features instead of being replaced by neighbor averages.
        aggregate = x.clone()
        aggregate.index_add_(0, target, messages)
        degree = torch.ones(x.shape[0], device=x.device)
        degree.index_add_(0, target, edge_weight)
        return self.linear(aggregate / degree.clamp_min(1e-6)[:, None])


class FrontierGCN(nn.Module):
    def __init__(self, input_dim=12, hidden_dim=64):
        super().__init__()
        self.conv1 = WeightedGCN(input_dim, hidden_dim)
        self.conv2 = WeightedGCN(hidden_dim, hidden_dim)
        self.gru = nn.GRUCell(hidden_dim + 5, hidden_dim)
        self.head = nn.Sequential(nn.Linear(hidden_dim * 2, hidden_dim),
                                  nn.ReLU(), nn.Linear(hidden_dim, 1))

    def forward(self, sample, hidden=None, previous=-1):
        x = torch.relu(self.conv1(sample["x"], sample["edge_index"], sample["edge_weight"]))
        x = torch.relu(self.conv2(x, sample["edge_index"], sample["edge_weight"]))
        previous_onehot = torch.zeros((1, 5), device=x.device)
        if 0 <= previous < 5:
            previous_onehot[0, previous] = 1.0
        hidden = self.gru(torch.cat([x.mean(0, keepdim=True), previous_onehot], -1), hidden)
        values = self.head(torch.cat([x[sample["frontier_index"]],
                                      hidden.expand(sample["frontier_index"].numel(), -1)], -1)).squeeze(-1)
        logits = torch.full((5,), torch.finfo(x.dtype).min, device=x.device)
        logits[sample["frontier_columns"]] = values
        return logits.masked_fill(~sample["safe_columns"].bool(), torch.finfo(x.dtype).min), hidden


def sample_to_device(sample, device):
    return {key: value.to(device) if torch.is_tensor(value) else value
            for key, value in sample.items()}


def session_groups(samples):
    groups = defaultdict(list)
    for sample in samples:
        groups[sample["session"]].append(sample)
    return {session: sorted(group, key=lambda sample: sample["seq"])
            for session, group in groups.items()}


def accuracy_metrics(predictions, targets):
    accuracy = sum(a == b for a, b in zip(predictions, targets)) / max(1, len(targets))
    per_class = []
    for column in range(5):
        indices = [i for i, target in enumerate(targets) if target == column]
        if indices:
            per_class.append(sum(predictions[i] == column for i in indices) / len(indices))
    return accuracy, sum(per_class) / max(1, len(per_class))


def run_epoch(model, samples, device, optimizer=None, class_weights=None):
    training = optimizer is not None
    model.train(training)
    predictions = []
    targets = []
    groups = session_groups(samples)
    sessions = list(groups)
    if training:
        random.shuffle(sessions)
    for session in sessions:
        hidden = None
        previous = -1
        for raw_sample in groups[session]:
            sample = sample_to_device(raw_sample, device)
            logits, hidden = model(sample, hidden, previous)
            target = torch.tensor([sample["target"]], dtype=torch.long, device=device)
            loss = nn.functional.cross_entropy(logits[None], target, weight=class_weights)
            if training:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                hidden = hidden.detach()
            prediction = int(logits.argmax())
            predictions.append(prediction)
            targets.append(sample["target"])
            previous = prediction
    return accuracy_metrics(predictions, targets)


def baseline_report(samples):
    if not samples:
        return 0.0, 0.0, 0.0
    targets = [s["target"] for s in samples]
    planner = [s["planner_target"] for s in samples]
    planner_accuracy, planner_macro = accuracy_metrics(planner, targets)
    switches = comparisons = 0
    for group in session_groups(samples).values():
        switches += sum(a["planner_target"] != b["planner_target"]
                        for a, b in zip(group, group[1:]))
        comparisons += max(0, len(group) - 1)
    return planner_accuracy, planner_macro, switches / max(1, comparisons)


def prediction_switch_rate(model, samples, device):
    switches = comparisons = 0
    for group in session_groups(samples).values():
        hidden = None
        previous = -1
        predictions = []
        for raw_sample in group:
            sample = sample_to_device(raw_sample, device)
            with torch.no_grad():
                logits, hidden = model(sample, hidden, previous)
            prediction = int(logits.argmax())
            predictions.append(prediction)
            previous = prediction
        switches += sum(a != b for a, b in zip(predictions, predictions[1:]))
        comparisons += max(0, len(predictions) - 1)
    return switches / max(1, comparisons)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="train_gcn/dataset.pt")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--test-ratio", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--save", default="train_gcn/frontier_gcn.pt")
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    args = parser.parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    bundle = torch.load(args.dataset, weights_only=False)
    samples = bundle["samples"]
    if not samples:
        raise SystemExit("dataset contains no samples")
    device_name = "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    if device_name == "auto":
        device_name = "cpu"
    if device_name == "cuda" and not torch.cuda.is_available():
        raise SystemExit("CUDA requested but torch.cuda.is_available() is false")
    device = torch.device(device_name)
    sessions = sorted({sample["session"] for sample in samples})
    random.shuffle(sessions)
    test_count = max(1, int(len(sessions) * args.test_ratio))
    val_count = max(1, int(len(sessions) * args.val_ratio))
    test_sessions = set(sessions[:test_count])
    val_sessions = set(sessions[test_count:test_count + val_count])
    train_sessions = set(sessions[test_count + val_count:])
    train = [s for s in samples if s["session"] in train_sessions]
    val = [s for s in samples if s["session"] in val_sessions]
    test = [s for s in samples if s["session"] in test_sessions]
    if not train or not val or not test:
        # A tiny dataset cannot support session-level splitting. Use disjoint
        # time-ordered portions so samples are still never shared.
        ordered = sorted(samples, key=lambda sample: (sample["session"], sample["seq"]))
        train_end = max(1, int(len(ordered) * (1.0 - args.val_ratio - args.test_ratio)))
        val_end = max(train_end + 1, int(len(ordered) * (1.0 - args.test_ratio)))
        train, val, test = ordered[:train_end], ordered[train_end:val_end], ordered[val_end:]
    model = FrontierGCN().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-3)
    counts = [sum(sample["target"] == column for sample in train) for column in range(5)]
    class_weights = torch.tensor(
        [len(train) / max(1, 5 * count) for count in counts],
        dtype=torch.float32, device=device)
    planner_acc, planner_macro, planner_switch_rate = baseline_report(test)
    test_targets = [sample["target"] for sample in test]
    majority = max(range(5), key=lambda column: counts[column])
    majority_acc, majority_macro = accuracy_metrics([majority] * len(test), test_targets)
    print(f"device={device} samples={len(samples)} train={len(train)} val={len(val)} "
          f"test={len(test)} "
          f"sessions={len(sessions)} train_label_counts={counts}")
    print(f"baseline_majority column={majority} test_accuracy={majority_acc:.3f} "
          f"macro_accuracy={majority_macro:.3f}")
    print(f"baseline_planner test_map_accuracy={planner_acc:.3f} macro_accuracy={planner_macro:.3f} "
          f"switch_rate={planner_switch_rate:.3f}")
    best_macro = -1.0
    best_epoch = 0
    best_state = None
    for epoch in range(1, args.epochs + 1):
        train_acc, train_macro = run_epoch(model, train, device, optimizer, class_weights)
        val_acc, val_macro = run_epoch(model, val, device)
        if val_macro > best_macro:
            best_macro = val_macro
            best_epoch = epoch
            best_state = copy.deepcopy(model.state_dict())
        if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs:
            print(f"epoch={epoch:03d} train_acc={train_acc:.3f} train_macro={train_macro:.3f} "
                  f"val_acc={val_acc:.3f} val_macro={val_macro:.3f}")
    model.load_state_dict(best_state)
    test_acc, test_macro = run_epoch(model, test, device)
    gcn_switch_rate = prediction_switch_rate(model, test, device)
    print(f"best_epoch={best_epoch} validation_macro_accuracy={best_macro:.3f}")
    print(f"comparison test_map_accuracy={test_acc:.3f} test_macro_accuracy={test_macro:.3f} "
          f"switch_rate={gcn_switch_rate:.3f} "
          f"delta_accuracy={test_acc - planner_acc:+.3f} "
          f"delta_macro_accuracy={test_macro - planner_macro:+.3f} "
          f"delta_switch_rate={gcn_switch_rate - planner_switch_rate:+.3f}")
    torch.save({"model": model.state_dict(), "input_dim": 12,
                "best_epoch": best_epoch,
                "train_sessions": sorted(train_sessions),
                "val_sessions": sorted(val_sessions),
                "test_sessions": sorted(test_sessions)}, args.save)
    print(f"saved={args.save}")


if __name__ == "__main__":
    main()
