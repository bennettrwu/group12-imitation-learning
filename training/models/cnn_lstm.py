"""
CNN-LSTM steering model
"""

import torch
import torch.nn as nn

from .pilotnet import PilotNetBackbone


class CNNLSTM(nn.Module):
    def __init__(self, hidden_size: int = 64):
        super().__init__()
        self.backbone = PilotNetBackbone(out_dim=64)
        self.lstm = nn.LSTM(input_size=64, hidden_size=hidden_size, batch_first=True)
        self.head = nn.Linear(hidden_size, 1)

    def forward(self, rgb_seq: torch.Tensor) -> torch.Tensor:
        B, T = rgb_seq.shape[:2]
        feats = self.backbone(rgb_seq.flatten(0, 1))
        feats = feats.view(B, T, -1)
        h_0 = feats.new_zeros(1, B, self.lstm.hidden_size)
        c_0 = feats.new_zeros(1, B, self.lstm.hidden_size)
        _, (h_n, _) = self.lstm(feats, (h_0, c_0))
        return self.head(h_n.squeeze(0)).squeeze(-1)
