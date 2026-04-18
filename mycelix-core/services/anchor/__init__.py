# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root"""
Holochain → Ethereum Anchor Service

Periodically syncs reputation data from Holochain DHT to Ethereum/Polygon
by building Merkle trees and anchoring roots on-chain.

Usage:
    from services.anchor import AnchorService

    service = AnchorService(network="polygon-amoy")
    await service.run()

Cost Analysis:
    - Holochain: FREE for all FL operations
    - Anchor: ~$0.01/tx on Polygon (maybe 24 tx/day = $0.24/day)
    - Monthly: ~$7.20 total vs $10,000+ for pure Ethereum
"""

from .anchor_service import (
    AnchorService,
    AnchorResult,
    ReputationUpdate,
    MerkleTree,
    MerkleProof,
    HolochainPoller,
    EthereumAnchor,
    NETWORKS,
)

__all__ = [
    "AnchorService",
    "AnchorResult",
    "ReputationUpdate",
    "MerkleTree",
    "MerkleProof",
    "HolochainPoller",
    "EthereumAnchor",
    "NETWORKS",
]
