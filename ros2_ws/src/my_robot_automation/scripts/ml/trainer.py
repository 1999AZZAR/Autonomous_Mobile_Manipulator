import json
import time
import numpy as np
from typing import Dict, List
from dataclasses import dataclass

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from .mlp_model import MLPDecisionModel, COMMANDS, NUM_CLASSES


@dataclass
class TrainingResult:
    epochs_completed: int
    best_val_accuracy: float
    best_val_loss: float
    training_time: float
    history: List[Dict]


class MLPTrainer:
    def __init__(self, model: MLPDecisionModel, lr: float = 0.001,
                 weight_decay: float = 1e-4):
        self.model = model
        self.criterion = nn.CrossEntropyLoss()
        self.optimizer = torch.optim.Adam(
            model.parameters(), lr=lr, weight_decay=weight_decay
        )
        self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, mode='min', patience=5, factor=0.5
        )
        self.history = []

    def train(self, X: np.ndarray, y: np.ndarray,
              epochs: int = 50, batch_size: int = 64,
              val_split: float = 0.2,
              patience: int = 10,
              log_callback=None) -> TrainingResult:
        start_time = time.time()

        y_encoded = np.array([COMMANDS.index(cmd) for cmd in y], dtype=np.int64)

        indices = np.random.permutation(len(X))
        split = int(len(X) * (1 - val_split))
        train_idx = indices[:split]
        val_idx = indices[split:]

        X_train = torch.from_numpy(X[train_idx]).float()
        y_train = torch.from_numpy(y_encoded[train_idx]).long()
        X_val = torch.from_numpy(X[val_idx]).float()
        y_val = torch.from_numpy(y_encoded[val_idx]).long()

        train_loader = DataLoader(
            TensorDataset(X_train, y_train),
            batch_size=batch_size, shuffle=True
        )

        best_val_loss = float('inf')
        best_val_acc = 0.0
        best_state = None
        epochs_no_improve = 0
        best_epoch = 0

        for epoch in range(epochs):
            self.model.train()
            train_loss = 0.0
            for batch_X, batch_y in train_loader:
                self.optimizer.zero_grad()
                outputs = self.model(batch_X)
                loss = self.criterion(outputs, batch_y)
                loss.backward()
                self.optimizer.step()
                train_loss += loss.item() * batch_X.size(0)

            train_loss /= len(train_idx)

            self.model.eval()
            with torch.no_grad():
                val_outputs = self.model(X_val)
                val_loss = self.criterion(val_outputs, y_val).item()
                val_preds = torch.argmax(val_outputs, dim=1)
                val_acc = (val_preds == y_val).float().mean().item()

            self.scheduler.step(val_loss)
            current_lr = self.optimizer.param_groups[0]['lr']

            epoch_log = {
                'epoch': epoch + 1,
                'train_loss': round(train_loss, 4),
                'val_loss': round(val_loss, 4),
                'val_accuracy': round(val_acc, 4),
                'lr': current_lr,
            }
            self.history.append(epoch_log)

            if log_callback:
                log_callback(epoch_log)

            if val_loss < best_val_loss - 1e-4:
                best_val_loss = val_loss
                best_val_acc = val_acc
                best_state = self.model.state_dict().copy()
                best_epoch = epoch
                epochs_no_improve = 0
            else:
                epochs_no_improve += 1

            if epochs_no_improve >= patience:
                break

        if best_state is not None:
            self.model.load_state_dict(best_state)

        elapsed = time.time() - start_time
        return TrainingResult(
            epochs_completed=best_epoch + 1,
            best_val_accuracy=best_val_acc,
            best_val_loss=best_val_loss,
            training_time=elapsed,
            history=self.history,
        )

    def evaluate(self, X: np.ndarray, y: np.ndarray) -> Dict:
        self.model.eval()
        y_encoded = np.array([COMMANDS.index(cmd) for cmd in y], dtype=np.int64)

        with torch.no_grad():
            x_t = torch.from_numpy(X).float()
            y_t = torch.from_numpy(y_encoded).long()
            outputs = self.model(x_t)
            loss = self.criterion(outputs, y_t).item()
            preds = torch.argmax(outputs, dim=1)
            accuracy = (preds == y_t).float().mean().item()

            cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=np.int64)
            for t, p in zip(y_encoded, preds.numpy()):
                cm[t, p] += 1

        precision = {}
        recall = {}
        for i, cmd in enumerate(COMMANDS):
            tp = cm[i, i]
            fp = cm[:, i].sum() - tp
            fn = cm[i, :].sum() - tp
            precision[cmd] = float(tp / (tp + fp)) if (tp + fp) > 0 else 0.0
            recall[cmd] = float(tp / (tp + fn)) if (tp + fn) > 0 else 0.0

        return {
            'loss': loss,
            'accuracy': accuracy,
            'precision': precision,
            'recall': recall,
            'confusion_matrix': cm.tolist(),
            'commands': COMMANDS,
        }

    def save_training_history(self, path: str):
        with open(path, 'w') as f:
            json.dump(self.history, f, indent=2)
