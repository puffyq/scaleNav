import argparse

import torch
from torch.utils.data import DataLoader

from config.config import cfg
from text_tracker.dataset import TextYopoDataset
from text_tracker.heatmap import sample_heatmap_at_body_directions


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a Text YOPO TorchScript model.")
    parser.add_argument("--data", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--approach-probability", type=float, default=1.0)
    parser.add_argument("--pearl-enter-threshold", type=float, default=0.08)
    args = parser.parse_args()

    dataset = TextYopoDataset(
        args.data,
        seed=100000,
        approach_probability=args.approach_probability,
        pearl_enter_threshold=args.pearl_enter_threshold,
    )
    loader = DataLoader(dataset, batch_size=32, shuffle=False)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = torch.jit.load(args.model, map_location=device).eval()
    candidate_counts = torch.zeros(int(cfg["traj_num"]), dtype=torch.long)
    heatmap_correct = 0
    selected_heatmap = 0.0
    approach_total = 0
    outputs_finite = True
    with torch.inference_mode():
        for batch in loader:
            image = batch["image"].to(device)
            obs = batch["obs"].to(device)
            endstate, score = model(image, obs)
            outputs_finite &= torch.isfinite(endstate).all().item()
            outputs_finite &= torch.isfinite(score).all().item()
            selected = score.flatten(1).argmin(dim=1)
            selected_cpu = selected.cpu()
            candidate_counts += torch.bincount(
                selected_cpu, minlength=int(cfg["traj_num"])
            )
            heatmap_grid = sample_heatmap_at_body_directions(
                image[:, 1],
                endstate[:, :3].permute(0, 2, 3, 1),
                horizontal_fov_deg=float(cfg["horizon_camera_fov"]),
                vertical_fov_deg=max(float(cfg["vertical_camera_fov"]), 1.0),
                horizontal_only=int(cfg["vertical_num"]) == 1,
            ).flatten(1)
            heatmap_correct += (selected == heatmap_grid.argmax(dim=1)).sum().item()
            selected_heatmap += heatmap_grid.gather(1, selected[:, None]).sum().item()
            approach_total += int(batch["approach"].sum().item())

    sample_count = len(dataset)
    print(f"Samples: {sample_count}")
    print(f"Outputs finite: {outputs_finite}")
    print(f"Selected candidates: {candidate_counts.tolist()}")
    print(f"Search samples: {sample_count - approach_total}")
    print(f"Approach samples: {approach_total}")
    print(f"Heatmap top-1: {heatmap_correct / max(sample_count, 1):.4f}")
    print(f"Selected heatmap value: {selected_heatmap / max(sample_count, 1):.4f}")


if __name__ == "__main__":
    main()
