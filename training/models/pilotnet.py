"""
PilotNet conv backbone - shared by CNN-LSTM and CNN-NODE.
"""

import torch
import torch.nn as nn


class PilotNetBackbone(nn.Module):
    def __init__(self, out_dim: int = 64):
        super().__init__()
        self.convs = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2),
            nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2),
            nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2),
            nn.ReLU(),
            nn.Conv2d(48, 64, 3),
            nn.ReLU(),
            nn.Conv2d(64, 64, 3),
            nn.ReLU(),
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.proj = nn.Linear(64, out_dim) if out_dim != 64 else nn.Identity()
        self.out_dim = out_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        f = self.convs(x)
        f = self.pool(f).flatten(1)
        return self.proj(f)
