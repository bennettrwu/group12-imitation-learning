"""
CNN steering model
"""

import torch
import torch.nn as nn
from torchvision import models


class CNN(nn.Module):
    def __init__(self):
        super().__init__()

        weights = models.ResNet18_Weights.IMAGENET1K_V1
        backbone = models.resnet18(weights=weights)
        in_features = backbone.fc.in_features

        self.backbone = nn.Sequential(*list(backbone.children())[:-1])

        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(in_features, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
        )

    def forward(self, rgb_seq: torch.Tensor) -> torch.Tensor:
        x = rgb_seq[:, -1]
        feat = self.backbone(x)
        return self.head(feat).squeeze(-1)
