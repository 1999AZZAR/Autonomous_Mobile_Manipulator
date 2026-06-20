import json
import numpy as np
from typing import Optional, List

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    TORCH_AVAILABLE = True
except ImportError:
    torch = None
    nn = None
    F = None
    TORCH_AVAILABLE = False


COMMANDS = ['f', 'b', 'q', 'e', 'z', 'x', 't', 'y', 's']
NUM_CLASSES = len(COMMANDS)


class MLPDecisionModel(nn.Module):
    def __init__(self, input_dim: int = 205, hidden_dims: List[int] = None):
        super().__init__()
        if hidden_dims is None:
            hidden_dims = [256, 128, 64]

        layers = []
        prev = input_dim
        for h in hidden_dims:
            layers.extend([
                nn.Linear(prev, h),
                nn.ReLU(),
                nn.Dropout(0.2),
            ])
            prev = h
        layers.append(nn.Linear(prev, NUM_CLASSES))

        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)

    def predict(self, feature_vector: np.ndarray) -> tuple:
        self.eval()
        with torch.no_grad():
            x = torch.from_numpy(feature_vector).float().unsqueeze(0)
            logits = self(x)
            probs = F.softmax(logits, dim=1).squeeze(0).numpy()
            pred_idx = int(np.argmax(probs))
            return COMMANDS[pred_idx], float(probs[pred_idx]), {
                cmd: float(prob) for cmd, prob in zip(COMMANDS, probs)
            }

    def predict_batch(self, features: np.ndarray) -> np.ndarray:
        self.eval()
        with torch.no_grad():
            x = torch.from_numpy(features).float()
            logits = self(x)
            return F.softmax(logits, dim=1).numpy()

    def save_model(self, path: str):
        torch.save(self.state_dict(), path)

    def load_model(self, path: str):
        self.load_state_dict(torch.load(path, map_location='cpu'))
        self.eval()

    def export_onnx(self, path: str, input_dim: int = 205):
        self.eval()
        dummy = torch.randn(1, input_dim)
        torch.onnx.export(
            self, dummy, path,
            input_names=['features'],
            output_names=['logits'],
            dynamic_axes={'features': {0: 'batch'}},
            opset_version=14,
        )

    @staticmethod
    def create_dummy(input_dim: int = 205):
        if not TORCH_AVAILABLE:
            return None
        return MLPDecisionModel(input_dim=input_dim)

    @staticmethod
    def save_config(path: str, input_dim: int, hidden_dims: List[int],
                    commands: List[str], norm_params: Optional[dict] = None):
        config = {
            'input_dim': input_dim,
            'hidden_dims': hidden_dims,
            'commands': commands,
            'norm_params': norm_params or {},
        }
        with open(path, 'w') as f:
            json.dump(config, f, indent=2)

    @staticmethod
    def load_config(path: str) -> dict:
        with open(path) as f:
            return json.load(f)
