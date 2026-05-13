"""
Modified GEM Dataset to provide image sequences instead of single frames
"""

import csv
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


class SteeringDataset(Dataset):
    def __init__(
        self,
        root: str,
        seq_len: int = 1,
        sequences: list[str] | None = None,
        rgb_transform=None,
        augment: bool = False
    ):
        """
        Args:
            root:          Dataset root containing sequence subdirectories.
            seq_len:       Number of consecutive frames to stack (1 = single frame).
            sequences:     List of sequence names to include. None = all.
            rgb_transform: torchvision transforms applied to each RGB tensor.
        """
        self.root          = Path(root)
        self.seq_len       = seq_len
        self.rgb_transform = rgb_transform
        self.augment = augment

        self._index    = []  # (seq_dir, seq_name, frame_idx, ctrl_ts)
        self._controls = {}  # seq_name -> {timestamp_ns -> steering_output_rad}

        all_seqs = sorted(p.name for p in self.root.iterdir() if p.is_dir())
        selected = sequences if sequences is not None else all_seqs

        for seq in selected:
            seq_dir   = self.root / seq
            ts_file   = seq_dir / 'calibration' / 'timestamps.csv'
            ctrl_file = seq_dir / 'controls' / 'data.csv'

            if not ts_file.exists() or not ctrl_file.exists():
                continue

            ctrl_map = {}
            with open(ctrl_file) as f:
                for row in csv.DictReader(f):
                    ctrl_map[int(row['timestamp_ns'])] = float(row['steering_output_rad'])
            self._controls[seq] = ctrl_map

            with open(ts_file) as f:
                for row in csv.DictReader(f):
                    ctrl_ts = int(row['ctrl_ts']) if row.get('ctrl_ts') else None
                    if ctrl_ts is None or ctrl_ts not in ctrl_map:
                        continue
                    self._index.append((
                        seq_dir, seq,
                        int(row['frame_idx']),
                        ctrl_ts,
                    ))

        self._seq_end_idx = []
        for i in range(seq_len - 1, len(self._index)):
            seq_name = self._index[i][1]
            if all(self._index[i - k][1] == seq_name for k in range(seq_len)):
                self._seq_end_idx.append(i)
                # self._seq_end_idx.append((i, False)) # original
                # if augment:
                #     self._seq_end_idx.append((i,True)) # flipped

    def __len__(self) -> int:
        return len(self._seq_end_idx)

    def _load_rgb(self, seq_dir: Path, frame_idx: int) -> torch.Tensor:
        rgb_path = seq_dir / 'front_camera' / 'rgb' / f'{frame_idx:06d}.jpg'
        rgb = cv2.cvtColor(cv2.imread(str(rgb_path)), cv2.COLOR_BGR2RGB)
        rgb_t = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        if self.rgb_transform is not None:
            rgb_t = self.rgb_transform(rgb_t)
        return rgb_t

    def __getitem__(self, idx: int) -> dict:
        end = self._seq_end_idx[idx]
        # print("Acceptable length: ", len(self._seq_end_idx))
        # print("index: ", idx)
        # end, flipped = self._seq_end_idx[idx] # unpack 'flipped' flag

        frames = [self._index[end - k] for k in reversed(range(self.seq_len))]
        rgb_seq = torch.stack([self._load_rgb(seq_dir, fi) for seq_dir, _, fi, _ in frames])

        seq_name, _, frame_idx, ctrl_ts = frames[-1][1], frames[-1][0], frames[-1][2], frames[-1][3]
        # steering = torch.tensor(self._controls[seq_name][ctrl_ts], dtype=torch.float32)
        steering = self._controls[seq_name][ctrl_ts]

        # if flipped:
        #     rgb_seq = torch.flip(rgb_seq, dims=[-1])
        #     steering = -steering

        return {
            'rgb_seq':   rgb_seq,    # [T, 3, H, W]
            'steering':  torch.tensor(steering, dtype=torch.float32),   # scalar
            'sequence':  seq_name,
            'frame_idx': frame_idx,
        }


def collate_fn(batch: list[dict]) -> dict:
    return {
        'rgb_seq':   torch.stack([b['rgb_seq']  for b in batch]),
        'steering':  torch.stack([b['steering'] for b in batch]),
        'sequence':  [b['sequence']  for b in batch],
        'frame_idx': [b['frame_idx'] for b in batch],
    }