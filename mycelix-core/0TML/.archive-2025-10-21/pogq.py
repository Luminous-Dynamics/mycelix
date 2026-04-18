# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root"""
PoGQ+Reputation - Proof of Good Quality with Reputation System

Integration of the parent directory's PoGQ system with 0TML baselines.

Reference: Internal ZeroTrustML specification
          September 25, 2025 implementation (parent directory)

Algorithm:
1. Each client generates PoGQ proof for their gradient:
   - Quality score based on loss improvement, gradient norm, convergence
   - Cryptographic commitment to gradient
   - Zero-knowledge proof of quality
2. Server verifies proofs and filters invalid updates
3. Reputation system tracks historical quality:
   - Exponential decay of old scores
   - Trust weight combines current quality + historical reputation
4. Weighted aggregation using trust weights

Byzantine Tolerance:
- Detects sophisticated attacks via quality metrics
- Reputation history identifies persistently malicious clients
- No theoretical upper bound (empirically tested up to 30% Byzantine)
"""

import sys
from pathlib import Path

# Add parent directory to path to import PoGQ system
parent_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(parent_dir))

from typing import List, Dict, Optional
import numpy as np
import torch
import torch.nn as nn
from dataclasses import dataclass

# Import the complete PoGQ system from parent directory
from pogq_system import ProofOfGoodQuality, PoGQProof


@dataclass
class PoGQConfig:
    """Configuration for PoGQ+Reputation system."""
    learning_rate: float = 0.01
    local_epochs: int = 1
    batch_size: int = 10
    num_clients: int = 10
    fraction_clients: float = 1.0
    quality_threshold: float = 0.3  # Minimum quality score to accept
    reputation_decay: float = 0.95  # Decay factor for historical reputation
    reputation_penalty: float = 0.2  # Penalty for low-quality updates


class PoGQServer:
    """
    PoGQ+Reputation Server.

    Implements Byzantine-robust FL using:
    - PoGQ proof generation and verification
    - Reputation tracking with exponential decay
    - Trust-weighted aggregation
    """

    def __init__(self, model: nn.Module, config: PoGQConfig, device: str = 'cpu'):
        """
        Initialize PoGQ server.

        Args:
            model: PyTorch model to train federally
            config: PoGQ configuration
        """
        self.model = model
        self.config = config
        self.device = device

        # Move model to device
        self.model = self.model.to(self.device)
        self.round = 0

        # Initialize PoGQ system
        self.pogq = ProofOfGoodQuality(quality_threshold=config.quality_threshold)

        # Statistics tracking
        self.stats = {
            'rounds': [],
            'train_loss': [],
            'train_accuracy': [],
            'test_loss': [],
            'test_accuracy': [],
            'num_accepted': [],  # Track how many clients accepted
            'avg_quality_score': [],  # Average PoGQ score
            'rejected_clients': [],  # Track rejected clients
        }

    def get_model_weights(self) -> List[np.ndarray]:
        """Get current global model weights."""
        return [param.cpu().detach().numpy().copy() for param in self.model.parameters()]

    def set_model_weights(self, weights: List[np.ndarray]):
        """Set global model weights."""
        with torch.no_grad():
            for param, new_weight in zip(self.model.parameters(), weights):
                param.copy_(torch.tensor(new_weight, device=self.device))

    def flatten_weights(self, weights: List[np.ndarray]) -> np.ndarray:
        """Flatten list of weight arrays into single vector."""
        return np.concatenate([w.flatten() for w in weights])

    def compute_gradient(self, old_weights: List[np.ndarray],
                        new_weights: List[np.ndarray]) -> List[np.ndarray]:
        """Compute gradient as difference between weights."""
        return [new_w - old_w for new_w, old_w in zip(new_weights, old_weights)]

    def aggregate(self, client_updates: List[Dict],
                 global_weights_before: List[np.ndarray]) -> List[np.ndarray]:
        """
        Aggregate client updates using PoGQ+Reputation weighted averaging.

        PoGQ+Rep aggregation:
            1. Generate PoGQ proofs for each client
            2. Verify proofs and filter invalid updates
            3. Get trust weights (current quality + historical reputation)
            4. Weighted average: w = Σ (trust_i / Σ trust_j) * w_i

        Args:
            client_updates: List of dicts with keys:
                - 'weights': Client's local weights
                - 'num_samples': Number of samples
                - 'loss_before': Loss before local training
                - 'loss_after': Loss after local training
                - 'client_id': Client identifier

        Returns:
            Aggregated weights
        """
        n = len(client_updates)

        # Generate PoGQ proofs for all clients
        proofs = []
        for i, update in enumerate(client_updates):
            # Compute gradient
            gradient = self.compute_gradient(global_weights_before, update['weights'])
            gradient_np = self.flatten_weights(gradient)

            # Get previous gradient for convergence scoring (if available)
            previous_gradient = None  # TODO: Track gradients across rounds

            # Generate PoGQ proof
            proof = self.pogq.generate_proof(
                client_id=update['client_id'],
                round_number=self.round,
                gradient=gradient_np,
                loss_before=update.get('loss_before', 1.0),
                loss_after=update.get('loss_after', 0.9),
                dataset_size=update['num_samples'],
                computation_time=update.get('computation_time', 1.0),
                previous_gradient=previous_gradient,
                global_statistics=None  # TODO: Track global gradient stats
            )
            proofs.append(proof)

        # Verify proofs and filter
        valid_updates = []
        valid_proofs = []
        rejected_clients = []

        for update, proof in zip(client_updates, proofs):
            is_valid, quality_score = self.pogq.verify_proof(proof)

            if is_valid:
                valid_updates.append(update)
                valid_proofs.append(proof)
            else:
                rejected_clients.append(update['client_id'])

        # Track statistics
        self.stats['num_accepted'].append(len(valid_updates))
        self.stats['rejected_clients'].append(rejected_clients)

        if len(valid_updates) == 0:
            # All clients rejected - return unchanged weights
            print(f"⚠️  All {n} clients rejected! Returning unchanged weights.")
            self.stats['avg_quality_score'].append(0.0)
            return global_weights_before

        # Calculate trust weights
        trust_weights = []
        quality_scores = []

        for update, proof in zip(valid_updates, valid_proofs):
            # Get historical reputation
            reputation = self.pogq.get_client_reputation(update['client_id'])

            # Calculate trust weight (combines current quality + historical reputation)
            trust_weight = self.pogq.calculate_trust_weight(proof, reputation)
            trust_weights.append(trust_weight)
            quality_scores.append(proof.quality_score)

        # Normalize trust weights
        total_trust = sum(trust_weights)
        if total_trust <= 0:
            # Fallback to uniform weighting
            trust_weights = [1.0 / len(valid_updates)] * len(valid_updates)
        else:
            trust_weights = [w / total_trust for w in trust_weights]

        # Track average quality score
        self.stats['avg_quality_score'].append(np.mean(quality_scores))

        # Weighted averaging
        num_layers = len(valid_updates[0]['weights'])
        aggregated_weights = [
            np.zeros_like(valid_updates[0]['weights'][i])
            for i in range(num_layers)
        ]

        for update, weight in zip(valid_updates, trust_weights):
            for i, layer_weights in enumerate(update['weights']):
                aggregated_weights[i] += weight * layer_weights

        print(f"PoGQ: Accepted {len(valid_updates)}/{n} clients, "
              f"avg quality={np.mean(quality_scores):.3f}, "
              f"rejected={len(rejected_clients)}")

        return aggregated_weights

    def train_round(self, clients: List['PoGQClient']) -> Dict:
        """
        Execute one round of federated training with PoGQ+Rep.

        Args:
            clients: List of PoGQClient instances

        Returns:
            Dictionary with round statistics
        """
        self.round += 1

        # Select clients
        num_selected = max(1, int(self.config.fraction_clients * len(clients)))
        selected_clients = np.random.choice(clients, size=num_selected, replace=False)

        # Get current global weights (BEFORE training)
        global_weights_before = self.get_model_weights()

        # Each client trains locally
        client_updates = []
        for client in selected_clients:
            # Send global weights to client
            client.set_model_weights(global_weights_before)

            # Client trains locally and returns update with loss info
            client_update = client.train()
            client_updates.append(client_update)

        # Aggregate using PoGQ+Reputation
        aggregated_weights = self.aggregate(client_updates, global_weights_before)

        # Update global model
        self.set_model_weights(aggregated_weights)

        # Calculate statistics (from accepted clients only)
        accepted_updates = [u for u in client_updates
                           if u['client_id'] not in self.stats['rejected_clients'][-1]]

        if len(accepted_updates) > 0:
            avg_loss = np.mean([u['loss'] for u in accepted_updates])
            avg_accuracy = np.mean([u['accuracy'] for u in accepted_updates])
        else:
            avg_loss = 0.0
            avg_accuracy = 0.0

        # Record statistics
        self.stats['rounds'].append(self.round)
        self.stats['train_loss'].append(avg_loss)
        self.stats['train_accuracy'].append(avg_accuracy)

        return {
            'round': self.round,
            'num_clients': len(selected_clients),
            'num_accepted': self.stats['num_accepted'][-1],
            'train_loss': avg_loss,
            'train_accuracy': avg_accuracy,
            'avg_quality_score': self.stats['avg_quality_score'][-1],
        }


class PoGQClient:
    """
    PoGQ Client.

    Standard FL client that returns additional metadata for PoGQ proof generation.
    """

    def __init__(
        self,
        client_id: str,
        model: nn.Module,
        train_data: torch.utils.data.DataLoader,
        config: PoGQConfig,
        device: str = 'cpu',
        is_byzantine: bool = False
    ):
        """
        Initialize PoGQ client.

        Args:
            client_id: Unique identifier
            model: PyTorch model
            train_data: DataLoader with client's training data
            config: PoGQ configuration
            device: 'cpu' or 'cuda'
            is_byzantine: If True, client sends malicious updates
        """
        self.client_id = client_id
        self.model = model
        self.train_data = train_data
        self.config = config
        self.device = device

        # Move model to device
        self.model = self.model.to(self.device)
        self.is_byzantine = is_byzantine

        # Calculate number of samples
        self.num_samples = len(train_data.dataset)

        # Optimizer for local training
        self.optimizer = torch.optim.SGD(
            self.model.parameters(),
            lr=config.learning_rate
        )

        # Loss function
        self.criterion = nn.CrossEntropyLoss()

    def get_model_weights(self) -> List[np.ndarray]:
        """Get current local model weights."""
        return [param.cpu().detach().numpy().copy() for param in self.model.parameters()]

    def set_model_weights(self, weights: List[np.ndarray]):
        """Set local model weights."""
        with torch.no_grad():
            for param, new_weight in zip(self.model.parameters(), weights):
                param.copy_(torch.tensor(new_weight, device=self.device))

    def evaluate_loss(self) -> float:
        """Evaluate current model loss on local data."""
        self.model.eval()
        total_loss = 0.0
        num_batches = 0

        with torch.no_grad():
            for data, target in self.train_data:
                data, target = data.to(self.device), target.to(self.device)
                output = self.model(data)
                loss = self.criterion(output, target)
                total_loss += loss.item()
                num_batches += 1

        return total_loss / max(num_batches, 1)

    def generate_byzantine_update(self) -> List[np.ndarray]:
        """Generate malicious update for Byzantine client."""
        weights = self.get_model_weights()

        # Random noise attack (aggressive)
        byzantine_weights = [
            w + np.random.normal(0, 10.0, w.shape)
            for w in weights
        ]

        return byzantine_weights

    def train(self) -> Dict:
        """
        Train local model for E epochs.

        Returns additional metadata for PoGQ proof generation:
        - loss_before: Loss before local training
        - loss_after: Loss after local training
        - computation_time: Time taken for training

        Returns:
            Dictionary with weights, metrics, and PoGQ metadata
        """
        import time

        # Byzantine clients return malicious updates
        if self.is_byzantine:
            return {
                'weights': self.generate_byzantine_update(),
                'num_samples': self.num_samples,
                'loss': 10.0,  # Fake high loss
                'accuracy': 0.1,  # Fake low accuracy
                'loss_before': 1.0,
                'loss_after': 10.0,  # Worse after "training"
                'computation_time': 0.001,  # Suspiciously fast
                'client_id': self.client_id,
            }

        # Measure loss before training
        start_time = time.time()
        loss_before = self.evaluate_loss()

        # Normal training
        self.model.train()
        epoch_losses = []
        epoch_accuracies = []

        for epoch in range(self.config.local_epochs):
            batch_losses = []
            correct = 0
            total = 0

            for batch_idx, (data, target) in enumerate(self.train_data):
                data, target = data.to(self.device), target.to(self.device)

                # Forward pass
                self.optimizer.zero_grad()
                output = self.model(data)
                loss = self.criterion(output, target)

                # Backward pass
                loss.backward()
                self.optimizer.step()

                # Track statistics
                batch_losses.append(loss.item())
                _, predicted = output.max(1)
                total += target.size(0)
                correct += predicted.eq(target).sum().item()

            epoch_losses.append(np.mean(batch_losses))
            epoch_accuracies.append(correct / total)

        # Measure loss after training
        loss_after = self.evaluate_loss()
        computation_time = time.time() - start_time

        return {
            'weights': self.get_model_weights(),
            'num_samples': self.num_samples,
            'loss': np.mean(epoch_losses),
            'accuracy': np.mean(epoch_accuracies),
            'loss_before': loss_before,
            'loss_after': loss_after,
            'computation_time': computation_time,
            'client_id': self.client_id,
        }


def create_pogq_experiment(
    model_fn: callable,
    train_data_splits: List[torch.utils.data.DataLoader],
    test_data: Optional[torch.utils.data.DataLoader] = None,
    config: Optional[PoGQConfig] = None,
    byzantine_clients: Optional[List[int]] = None,
    device: str = 'cpu'
) -> tuple:
    """
    Create a PoGQ+Reputation experiment.

    Args:
        model_fn: Function that returns a fresh PyTorch model
        train_data_splits: List of DataLoaders (one per client)
        test_data: Optional test DataLoader
        config: PoGQ configuration
        byzantine_clients: List of client indices to make Byzantine
        device: 'cpu' or 'cuda'

    Returns:
        (server, clients, test_data) tuple

    Example:
        >>> server, clients, test_data = create_pogq_experiment(
        ...     model_fn,
        ...     train_splits,
        ...     test_data,
        ...     config=PoGQConfig(quality_threshold=0.3),
        ...     byzantine_clients=[5, 7, 9]  # 30% Byzantine (3/10)
        ... )
    """
    if config is None:
        config = PoGQConfig()

    if byzantine_clients is None:
        byzantine_clients = []

    # Create server with global model
    global_model = model_fn()
    server = PoGQServer(global_model, config, device)

    # Create clients
    clients = []
    for i, train_data in enumerate(train_data_splits):
        client_model = model_fn()
        is_byzantine = i in byzantine_clients

        client = PoGQClient(
            client_id=f"client_{i}",
            model=client_model,
            train_data=train_data,
            config=config,
            device=device,
            is_byzantine=is_byzantine
        )
        clients.append(client)

    return server, clients, test_data


# ============================================================================
# Evaluation Functions
# ============================================================================

def evaluate_global_model(
    model: nn.Module,
    test_data: torch.utils.data.DataLoader,
    device: str = 'cpu'
) -> Dict[str, float]:
    """Evaluate global model on test data."""
    model.eval()
    criterion = nn.CrossEntropyLoss()

    test_loss = 0
    correct = 0
    total = 0

    with torch.no_grad():
        for data, target in test_data:
            data, target = data.to(device), target.to(device)
            output = model(data)

            test_loss += criterion(output, target).item()
            _, predicted = output.max(1)
            total += target.size(0)
            correct += predicted.eq(target).sum().item()

    return {
        'test_loss': test_loss / len(test_data),
        'test_accuracy': correct / total
    }


if __name__ == "__main__":
    print("✅ PoGQ+Reputation baseline implementation loaded")
    print("   Reference: ZeroTrustML specification (Internal)")
    print("   Algorithm: Quality proofs + Reputation tracking")
    print(f"   Key: Cryptographic quality verification + trust weighting")
