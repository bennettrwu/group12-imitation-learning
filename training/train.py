"""
Train a steering imitation-learning model on processed GEM bag data.

Usage:
    python -m training.train --model cnn      --epochs 30
    python -m training.train --model cnn_lstm --epochs 100 --batch-size 256
    python -m training.train --model cnn_node --epochs 100 --batch-size 256
"""

import argparse
import csv
from pathlib import Path

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Subset
from torchvision import transforms
from tqdm import tqdm

from .dataset.steering_dataset import SteeringDataset, collate_fn
from .models import MODELS, SEQ_LEN

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


def build_transform(image_size: int = 224, augment: bool = False):
    layers: list[nn.Module] = [
        transforms.Resize((image_size, image_size), antialias=True)
    ]
    if augment:
        layers.append(transforms.ColorJitter(brightness=0.3))
    layers.append(transforms.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD))
    return transforms.Compose(layers)


def epoch_loss(model, loader, device, optimizer=None) -> float:
    train = optimizer is not None
    model.train(train)
    loss_fn = nn.MSELoss(reduction="sum")
    total_loss = 0.0
    total_n = 0
    ctx = torch.enable_grad() if train else torch.no_grad()
    with ctx:
        for batch in tqdm(loader):
            rgb = batch["rgb_seq"].to(device, non_blocking=True)
            y = batch["steering"].to(device, non_blocking=True)
            pred = model(rgb)
            pred.squeeze(-1)        # New line to ensure no broadcasting is necessary
            loss = loss_fn(pred, y)
            if train:
                optimizer.zero_grad()
                (loss / rgb.size(0)).backward()
                # Attempting clipping to prevent exploding gradients
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()
            total_loss += loss.item()
            total_n += rgb.size(0)
    return total_loss / total_n


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", choices=list(MODELS), required=True)
    p.add_argument("--data-root", default="data/processed_sim")
    p.add_argument("--output-dir", default="training/checkpoints")
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--val-fraction", type=float, default=0.15)
    p.add_argument("--num-workers", type=int, default=4)
    p.add_argument("--image-size", type=int, default=224)
    p.add_argument("--seed", type=int, default=22)
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = p.parse_args()

    torch.manual_seed(args.seed)
    device = torch.device(args.device)
    out_dir = Path(args.output_dir) / args.model
    out_dir.mkdir(parents=True, exist_ok=True)

    seq_len = SEQ_LEN[args.model]

    train_tf = build_transform(args.image_size, augment=True)
    val_tf   = build_transform(args.image_size, augment=False)
    train_full = SteeringDataset(args.data_root, seq_len=seq_len, rgb_transform=train_tf, augment=True)
    val_full   = SteeringDataset(args.data_root, seq_len=seq_len, rgb_transform=val_tf)

    n = len(train_full)
    n_val = int(n * args.val_fraction)
    # Removing random ordering
    perm = torch.randperm(n, generator=torch.Generator().manual_seed(args.seed)).tolist()
    val_idx, train_idx = perm[:n_val], perm[n_val:]
    # indices = torch.arange(n)   # Sequential frame ordering
    # val_idx, train_idx = indices[:n_val], indices[n_val:]
    train_ds = Subset(train_full, train_idx)
    val_ds   = Subset(val_full,   val_idx)

    print(f"windows: {n} total -> train {len(train_ds)} / val {len(val_ds)} ({args.val_fraction:.0%} random)")
    print(f"model: {args.model} (seq_len={seq_len})  device: {device}")

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        collate_fn=collate_fn,
        pin_memory=(device.type == "cuda"),
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        collate_fn=collate_fn,
        pin_memory=(device.type == "cuda"),
    )

    model = MODELS[args.model]().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)
    # New, attempting a scheduler
    # scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
    #     optimizer, mode='min', patience=5, factor=0.5
    # )

    history = []
    best_val = float("inf")
    # patience = 0
    for epoch in range(1, args.epochs + 1):
        train_mse = epoch_loss(model, train_loader, device, optimizer)
        val_mse = epoch_loss(model, val_loader, device)

        # scheduler.step(val_mse)         # Added a scheduler

        history.append((epoch, train_mse, val_mse))

        is_best = False
        if val_mse < best_val:
            best_val = val_mse
            torch.save(model.state_dict(), out_dir / "best.pt")
            is_best = True
            # patience = 0
        # else:
        #     patience += 1

        print(
            f"epoch {epoch:3d}/{args.epochs}\ttrain {train_mse:.5f}\tval {val_mse:.5f}{' *' if is_best else ''}"
        )

        # if patience >= 25:
        #     break

    # Save history CSV
    with open(out_dir / "history.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["epoch", "train_mse", "val_mse"])
        writer.writerows(history)

    # Reload best weights and export TorchScript for ROS2 inference.
    model.load_state_dict(torch.load(out_dir / "best.pt", map_location=device))
    model.eval()
    example = torch.randn(1, seq_len, 3, args.image_size, args.image_size, device=device)
    traced = torch.jit.trace(model, example)
    torch.jit.save(traced, str(out_dir / "best.ts.pt"))
    print(f"best val MSE: {best_val:.5f}\tsaved at: {out_dir}/best.pt + best.ts.pt")


if __name__ == "__main__":
    main()
