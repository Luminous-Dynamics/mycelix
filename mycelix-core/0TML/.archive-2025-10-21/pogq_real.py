#!/usr/bin/env python3

# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root"""
REAL PoGQ+Reputation Baseline for Hybrid-ZeroTrustML

This is the REAL implementation that:
1. Measures actual loss before/after on real data
2. Uses persistent SQLite reputation database
3. Achieves superior Byzantine detection

Copied from parent directory's hybrid_fl_pogq_reputation.py
Adapted to match 0TML baseline interface
"""

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import numpy as np
from typing import Dict, List, Tuple, Optional
import hashlib
import json
import sqlite3
from dataclasses import dataclass
from datetime import datetime
import time
from pathlib import Path

# ==============================================================================
# REAL PROOF OF GRADIENT QUALITY
# ==============================================================================

@dataclass
class EnhancedGradientProof:
    """Proof that a gradient improves the model"""
    node_id: str
    round_number: int
    gradient_hash: str
    loss_before: float
    loss_after: float
    data_samples: int
    computation_time: float
    timestamp: str
    validation_loss: Optional[float] = None

class RealProofOfGradientQuality:
    """
    REAL implementation of PoGQ that actually validates gradients
    Not just hashing and random numbers!
    """

    def __init__(self, model: nn.Module, loss_fn: nn.Module, device: str = 'cpu'):
        self.model = model
        self.loss_fn = loss_fn
        self.device = device
        self.proof_history = []

    def generate_proof(self,
                      gradient: Dict[str, torch.Tensor],
                      local_data: DataLoader,
                      node_id: str,
                      round_number: int) -> EnhancedGradientProof:
        """
        Generate REAL proof by testing gradient quality
        """
        start_time = time.time()

        # Save original model state
        original_state = {k: v.clone() for k, v in self.model.state_dict().items()}

        # Calculate loss BEFORE applying gradient
        self.model.eval()
        loss_before = 0.0
        num_samples = 0

        with torch.no_grad():
            for batch_x, batch_y in local_data:
                batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                outputs = self.model(batch_x)
                loss = self.loss_fn(outputs, batch_y)
                loss_before += loss.item() * batch_x.size(0)
                num_samples += batch_x.size(0)
                if num_samples >= 100:  # Sample for efficiency
                    break

        loss_before /= num_samples

        # Apply gradient to model
        with torch.no_grad():
            for name, param in self.model.named_parameters():
                if name in gradient:
                    param.data -= 0.01 * gradient[name].to(self.device)  # Learning rate 0.01

        # Calculate loss AFTER applying gradient
        loss_after = 0.0
        samples_after = 0

        with torch.no_grad():
            for batch_x, batch_y in local_data:
                batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                outputs = self.model(batch_x)
                loss = self.loss_fn(outputs, batch_y)
                loss_after += loss.item() * batch_x.size(0)
                samples_after += batch_x.size(0)
                if samples_after >= 100:
                    break

        loss_after /= samples_after

        # Restore original model
        self.model.load_state_dict(original_state)

        # Create gradient hash
        grad_bytes = b''.join([g.cpu().numpy().tobytes() for g in gradient.values()])
        gradient_hash = hashlib.sha256(grad_bytes).hexdigest()[:16]

        computation_time = time.time() - start_time

        proof = EnhancedGradientProof(
            node_id=node_id,
            round_number=round_number,
            gradient_hash=gradient_hash,
            loss_before=loss_before,
            loss_after=loss_after,
            data_samples=num_samples,
            computation_time=computation_time,
            timestamp=datetime.now().isoformat()
        )

        self.proof_history.append(proof)
        return proof

    def verify_proof(self, proof: EnhancedGradientProof) -> Tuple[bool, float]:
        """
        Verify that a gradient proof shows improvement
        Returns (is_valid, quality_score)
        """
        # Check that loss decreased
        if proof.loss_after >= proof.loss_before:
            return False, 0.0  # No improvement

        # Calculate improvement rate
        improvement = (proof.loss_before - proof.loss_after) / max(proof.loss_before, 1e-8)

        # Suspicious if improvement is too large (likely fake)
        if improvement > 0.5:  # >50% improvement in one step is suspicious
            return False, 0.0

        # Suspicious if computation was too fast
        if proof.computation_time < 0.001:  # Less than 1ms is fake
            return False, 0.0

        # Calculate quality score (0-1)
        quality = min(improvement * 10, 1.0)  # Scale improvement to 0-1

        # Adjust quality based on computation time (longer = more thorough)
        if proof.computation_time > 0.1:
            quality *= 1.1

        quality = min(quality, 1.0)

        return True, quality


def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Compute cosine similarity with safeguards against zero norms."""
    a = np.asarray(a)
    b = np.asarray(b)
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    if denom == 0:
        return 0.0
    return float(np.dot(a, b) / denom)


def analyze_gradient_quality(gradient: np.ndarray, reference_gradients: List[np.ndarray]) -> float:
    """Return a quality score in [0, 1] comparing a gradient to references.

    The original grant demo expected a function that highlights inverted or
    highly deviant gradients. We approximate that behaviour by measuring the
    cosine similarity against the mean honest gradient and penalising large
    magnitude deviations. A score near 1.0 indicates honest behaviour, while
    values near 0 signal potential Byzantine activity.
    """

    if not reference_gradients:
        return 0.0

    honest_stack = np.stack(reference_gradients, axis=0)
    honest_mean = np.mean(honest_stack, axis=0)

    cos = _cosine_similarity(gradient, honest_mean)
    # Map cosine similarity [-1, 1] to [0, 1]
    score = (cos + 1.0) / 2.0

    # Penalise if the gradient magnitude is far larger than the honest mean
    honest_norm = np.linalg.norm(honest_mean) + 1e-8
    grad_norm = np.linalg.norm(gradient)
    magnitude_ratio = grad_norm / honest_norm
    if magnitude_ratio > 1.5:
        score /= magnitude_ratio

    return float(np.clip(score, 0.0, 1.0))

# ==============================================================================
# PERSISTENT REPUTATION SYSTEM
# ==============================================================================

class PersistentReputationSystem:
    """
    Reputation system with SQLite persistence
    Tracks long-term node behavior across rounds
    """

    def __init__(self, db_path: str = "fl_reputation.db"):
        self.db_path = db_path
        self.current_reputations = {}
        self._init_database()
        self._load_reputations()

    def _init_database(self):
        """Initialize SQLite database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS reputation_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                node_id TEXT NOT NULL,
                round_number INTEGER NOT NULL,
                reputation_score REAL NOT NULL,
                proof_valid BOOLEAN NOT NULL,
                contribution_quality REAL NOT NULL,
                timestamp TEXT NOT NULL
            )
        """)

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS node_reputation (
                node_id TEXT PRIMARY KEY,
                current_reputation REAL NOT NULL,
                total_rounds INTEGER NOT NULL,
                successful_rounds INTEGER NOT NULL,
                last_updated TEXT NOT NULL
            )
        """)

        conn.commit()
        conn.close()

    def _load_reputations(self):
        """Load current reputations from database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("SELECT node_id, current_reputation FROM node_reputation")
        rows = cursor.fetchall()

        for node_id, reputation in rows:
            self.current_reputations[node_id] = reputation

        conn.close()

    def update_reputation(self,
                         node_id: str,
                         proof_valid: bool,
                         contribution_quality: float,
                         round_number: int) -> float:
        """
        Update node reputation based on contribution
        Returns new reputation score
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        # Get current reputation or initialize
        current_rep = self.current_reputations.get(node_id, 0.5)

        # Calculate reputation change
        if proof_valid:
            # Reward good behavior
            delta = min(0.1, contribution_quality * 0.1)
            new_rep = min(1.0, current_rep + delta)
        else:
            # Punish bad behavior more severely
            delta = -0.2
            new_rep = max(0.0, current_rep + delta)

        # Add momentum: recent behavior matters more
        cursor.execute("""
            SELECT contribution_quality
            FROM reputation_history
            WHERE node_id = ?
            ORDER BY round_number DESC
            LIMIT 10
        """, (node_id,))

        recent_history = cursor.fetchall()
        if len(recent_history) >= 5:
            recent_avg = np.mean([h[0] for h in recent_history[:5]])
            older_avg = np.mean([h[0] for h in recent_history[5:]])

            if recent_avg > older_avg * 1.2:
                new_rep = min(1.0, new_rep + 0.05)  # Improving
            elif recent_avg < older_avg * 0.8:
                new_rep = max(0.0, new_rep - 0.05)  # Degrading

        # Update database
        cursor.execute("""
            INSERT INTO reputation_history
            (node_id, round_number, reputation_score, proof_valid,
             contribution_quality, timestamp)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (node_id, round_number, new_rep, proof_valid,
              contribution_quality, datetime.now().isoformat()))

        cursor.execute("""
            INSERT OR REPLACE INTO node_reputation
            (node_id, current_reputation, total_rounds,
             successful_rounds, last_updated)
            VALUES (?, ?,
                    COALESCE((SELECT total_rounds FROM node_reputation
                              WHERE node_id = ?), 0) + 1,
                    COALESCE((SELECT successful_rounds FROM node_reputation
                              WHERE node_id = ?), 0) + ?,
                    ?)
        """, (node_id, new_rep, node_id, node_id,
              1 if proof_valid else 0, datetime.now().isoformat()))

        conn.commit()
        conn.close()

        self.current_reputations[node_id] = new_rep
        return new_rep

    def get_weight(self, node_id: str) -> float:
        """Get aggregation weight based on reputation"""
        rep = self.current_reputations.get(node_id, 0.5)

        # Sigmoid transformation for smooth weighting
        weight = 1 / (1 + np.exp(-10 * (rep - 0.5)))

        return max(0.01, weight)  # Minimum weight

# ==============================================================================
# BASELINE WRAPPER FOR HYBRID-ZEROTRUSTML
# ==============================================================================

class PoGQServer:
    """
    PoGQ+Reputation Server for 0TML baseline interface

    This wraps the REAL PoGQ+Reputation system to match the
    standard baseline interface expected by the experiment runner.
    """

    def __init__(self, model: nn.Module, config: dict, device: str = 'cpu'):
        """
        Initialize PoGQ+Reputation server

        Args:
            model: PyTorch model to train federally
            config: Configuration dict with:
                - quality_threshold: Minimum quality score (default: 0.3)
                - reputation_decay: Decay factor (default: 0.95)
                - db_path: SQLite database path (optional)
        """
        self.model = model
        self.config = config
        self.device = device

        # Move model to device
        self.model = self.model.to(self.device)
        self.round = 0

        # Initialize loss function
        self.loss_fn = nn.CrossEntropyLoss()

        # Initialize REAL PoGQ and Reputation systems
        self.pogq = RealProofOfGradientQuality(model, self.loss_fn, device)

        # Create fresh database for each experiment
        db_path = config.get('db_path', f'fl_reputation_experiment_{id(self)}.db')
        # Remove old database if exists
        if Path(db_path).exists():
            Path(db_path).unlink()
        self.reputation = PersistentReputationSystem(db_path)

        # Statistics tracking
        self.stats = {
            'rounds': [],
            'train_loss': [],
            'train_accuracy': [],
            'test_loss': [],
            'test_accuracy': [],
            'num_accepted': [],
            'avg_quality_score': [],
            'rejected_clients': [],
        }

    def get_model_weights(self) -> List[np.ndarray]:
        """Get current global model weights"""
        return [param.cpu().detach().numpy().copy() for param in self.model.parameters()]

    def aggregate(self, client_updates: List[Dict], client_data_loaders: Dict[int, DataLoader]) -> None:
        """
        Aggregate client updates using REAL PoGQ+Reputation

        Args:
            client_updates: List of dicts with 'client_id', 'weights', 'num_samples'
            client_data_loaders: Dict mapping client_id to their DataLoader
        """
        self.round += 1

        # Convert client weights to gradients
        global_params = {name: param.data.clone() for name, param in self.model.named_parameters()}

        node_gradients = {}
        for update in client_updates:
            client_id = update['client_id']
            client_weights = update['weights']

            # Calculate gradient as difference
            gradient = {}
            for i, (name, param) in enumerate(self.model.named_parameters()):
                gradient[name] = global_params[name] - torch.from_numpy(client_weights[i]).to(self.device)

            node_gradients[f"client_{client_id}"] = gradient

        # Generate and verify proofs
        valid_gradients = {}
        node_qualities = {}
        rejected_clients = []

        for node_id, gradient in node_gradients.items():
            client_id = int(node_id.split('_')[1])

            # Generate proof using client's data
            proof = self.pogq.generate_proof(
                gradient,
                client_data_loaders[client_id],
                node_id,
                self.round
            )

            # Verify proof
            is_valid, quality = self.pogq.verify_proof(proof)

            if is_valid:
                valid_gradients[node_id] = gradient
                node_qualities[node_id] = quality
            else:
                rejected_clients.append(client_id)

            # Update reputation
            self.reputation.update_reputation(
                node_id, is_valid, quality if is_valid else 0, self.round
            )

        # Weighted aggregation based on reputation
        aggregated = None
        total_weight = 0.0

        for node_id, gradient in valid_gradients.items():
            weight = self.reputation.get_weight(node_id)
            quality_boost = node_qualities[node_id]
            final_weight = weight * (0.5 + 0.5 * quality_boost)

            if aggregated is None:
                aggregated = {k: v.clone() * final_weight for k, v in gradient.items()}
            else:
                for k, v in gradient.items():
                    aggregated[k] += v * final_weight

            total_weight += final_weight

        # Normalize and apply
        if aggregated and total_weight > 0:
            with torch.no_grad():
                for name, param in self.model.named_parameters():
                    if name in aggregated:
                        update = aggregated[name] / total_weight
                        param.data -= update  # Apply gradient

        # Track statistics
        self.stats['num_accepted'].append(len(valid_gradients))
        self.stats['avg_quality_score'].append(
            np.mean(list(node_qualities.values())) if node_qualities else 0.0
        )
        self.stats['rejected_clients'].append(rejected_clients)
