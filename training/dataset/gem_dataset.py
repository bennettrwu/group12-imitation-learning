"""
PyTorch dataset for GEM multi-sensor data.

Usage:
    from gem_dataset import GEMDataset, collate_fn
    from torch.utils.data import DataLoader

    dataset = GEMDataset('/path/to/dataset')
    loader  = DataLoader(dataset, batch_size=4, collate_fn=collate_fn)

    for batch in loader:
        rgb      = batch['rgb']      # [B, 3, H, W]  float32 in [0, 1]
        depth    = batch['depth']    # [B, 1, H, W]  float32 in meters
        lidar    = batch['lidar']    # list of [N_i, 4] tensors (x y z intensity)
        gnss     = batch['gnss']     # [B, 6]  (lat_deg, lon_deg, altitude_m,
                                     #          heading_deg, pitch_deg, roll_deg)
        controls = batch['controls'] # [B, 10] (steering_output, steering_cmd, steering_manual,
                                     #          vehicle_speed,
                                     #          accel_output, accel_cmd, accel_manual,
                                     #          brake_output, brake_cmd,
                                     #          shift_output [0=PARK,1=REV,2=NEU,3=FWD])
"""

import csv
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


class GEMDataset(Dataset):
    def __init__(
        self,
        root: str,
        sequences: list[str] | None = None,
        max_points: int | None = None,
        rgb_transform=None,
    ):
        """
        Args:
            root:          Dataset root containing sequence subdirectories.
            sequences:     List of sequence names to include. None = all.
            max_points:    Truncate/pad LiDAR to this many points. None = variable length.
            rgb_transform: torchvision transforms applied to the RGB tensor.
        """
        self.root          = Path(root)
        self.max_points    = max_points
        self.rgb_transform = rgb_transform

        self._index    = []  # (seq_dir, seq_name, frame_idx, gnss_ts, ctrl_ts)
        self._gnss     = {}  # seq_name -> {timestamp_ns -> row dict}
        self._controls = {}  # seq_name -> {timestamp_ns -> row dict}

        all_seqs = sorted(p.name for p in self.root.iterdir() if p.is_dir())
        selected = sequences if sequences is not None else all_seqs

        for seq in selected:
            seq_dir   = self.root / seq
            ts_file   = seq_dir / 'calibration' / 'timestamps.csv'
            gnss_file = seq_dir / 'gnss' / 'data.csv'
            ctrl_file = seq_dir / 'controls' / 'data.csv'

            if not ts_file.exists() or not gnss_file.exists():
                continue

            gnss_map = {}
            with open(gnss_file) as f:
                for row in csv.DictReader(f):
                    gnss_map[int(row['timestamp_ns'])] = row
            self._gnss[seq] = gnss_map

            ctrl_map = {}
            if ctrl_file.exists():
                with open(ctrl_file) as f:
                    for row in csv.DictReader(f):
                        ctrl_map[int(row['timestamp_ns'])] = row
            self._controls[seq] = ctrl_map

            with open(ts_file) as f:
                for row in csv.DictReader(f):
                    ctrl_ts  = int(row['ctrl_ts'])  if row.get('ctrl_ts')  else None
                    gnss_ts  = int(row['gnss_ts'])  if row.get('gnss_ts')  else None
                    self._index.append((
                        seq_dir, seq,
                        int(row['frame_idx']),
                        gnss_ts,
                        ctrl_ts,
                    ))

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, idx: int) -> dict:
        seq_dir, seq_name, frame_idx, gnss_ts, ctrl_ts = self._index[idx]
        name = f'{frame_idx:06d}'

        # RGB
        rgb_path = seq_dir / 'front_camera' / 'rgb' / f'{name}.jpg'
        rgb = cv2.cvtColor(cv2.imread(str(rgb_path)), cv2.COLOR_BGR2RGB)
        rgb_t = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        if self.rgb_transform is not None:
            rgb_t = self.rgb_transform(rgb_t)

        # Depth (16-bit PNG in mm → float32 meters)
        depth_path = seq_dir / 'front_camera' / 'depth' / f'{name}.png'
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
        depth_t = torch.from_numpy(depth).unsqueeze(0)

        # LiDAR (N x 4 float32 binary)
        lidar_path = seq_dir / 'lidar' / f'{name}.bin'
        points = np.fromfile(str(lidar_path), dtype=np.float32).reshape(-1, 4)
        if self.max_points is not None:
            if len(points) >= self.max_points:
                points = points[:self.max_points]
            else:
                pad = np.zeros((self.max_points - len(points), 4), dtype=np.float32)
                points = np.vstack([points, pad])
        lidar_t = torch.from_numpy(points)

        # GNSS [6]: lat_deg, lon_deg, altitude_m, heading_deg, pitch_deg, roll_deg
        g = self._gnss[seq_name][gnss_ts] if gnss_ts and gnss_ts in self._gnss[seq_name] else {}
        gnss_t = torch.tensor([
            float(g.get('latitude_deg',  'nan')), float(g.get('longitude_deg', 'nan')),
            float(g.get('altitude_m',    'nan')),
            float(g.get('heading_deg',   'nan')), float(g.get('pitch_deg',     'nan')),
            float(g.get('roll_deg',      'nan')),
        ], dtype=torch.float32)

        # Controls [10]: steering_output, steering_cmd, steering_manual,
        #                 vehicle_speed,
        #                 accel_output, accel_cmd, accel_manual,
        #                 brake_output, brake_cmd,
        #                 shift_output (0=PARK, 1=REVERSE, 2=NEUTRAL, 3=FORWARD)
        ctrl_map = self._controls.get(seq_name, {})
        if ctrl_ts and ctrl_ts in ctrl_map:
            c = ctrl_map[ctrl_ts]
            controls_t = torch.tensor([
                float(c['steering_output_rad']), float(c['steering_cmd_rad']), float(c['steering_manual_rad']),
                float(c['vehicle_speed_mps']),
                float(c['accel_output']),        float(c['accel_cmd']),        float(c['accel_manual']),
                float(c['brake_output']),        float(c['brake_cmd']),
                float(c['shift_output']),
            ], dtype=torch.float32)
        else:
            controls_t = torch.full((10,), float('nan'))

        return {
            'rgb':       rgb_t,
            'depth':     depth_t,
            'lidar':     lidar_t,
            'gnss':      gnss_t,
            'controls':  controls_t,
            'sequence':  seq_name,
            'frame_idx': frame_idx,
        }


def collate_fn(batch: list[dict]) -> dict:
    """Custom collate that handles variable-length LiDAR point clouds."""
    return {
        'rgb':       torch.stack([b['rgb']      for b in batch]),
        'depth':     torch.stack([b['depth']    for b in batch]),
        'lidar':     [b['lidar'] for b in batch],  # list of [N_i, 4]
        'gnss':      torch.stack([b['gnss']     for b in batch]),
        'controls':  torch.stack([b['controls'] for b in batch]),
        'sequence':  [b['sequence']  for b in batch],
        'frame_idx': [b['frame_idx'] for b in batch],
    }
