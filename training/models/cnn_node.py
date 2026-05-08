"""
CNN-NODE steering model
"""

import torch
import torch.nn as nn
from torchdiffeq import odeint

from .pilotnet import PilotNetBackbone


class _ODEFunc(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(dim, dim),
            nn.Tanh(),
            nn.Linear(dim, dim),
        )

    def forward(self, _t: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        return self.net(y)


class CNNNode(nn.Module):
    def __init__(self, seq_len: int = 3, feature_dim: int = 64, solver: str = "rk4"):
        super().__init__()
        self.backbone = PilotNetBackbone(out_dim=feature_dim)

        state_dim = seq_len * feature_dim
        self.ode_func = _ODEFunc(state_dim)
        self.solver = solver
        self.register_buffer("integration_time", torch.tensor([0.0, 1.0]))

        self.head = nn.Sequential(
            nn.Linear(state_dim, 50),
            nn.ReLU(),
            nn.Linear(50, 25),
            nn.ReLU(),
            nn.Linear(25, 1),
        )

    def forward(self, rgb_seq: torch.Tensor) -> torch.Tensor:
        B = rgb_seq.size(0)
        feats = self.backbone(rgb_seq.flatten(0, 1))
        y0 = feats.view(B, -1)
        out = odeint(self.ode_func, y0, self.integration_time, method=self.solver)
        return self.head(out[-1]).squeeze(-1)
