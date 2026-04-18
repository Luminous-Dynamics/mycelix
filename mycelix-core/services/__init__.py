# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root"""
Mycelix Services

Background services for the hybrid Holochain/Ethereum architecture.

Services:
    anchor: Holochain → Ethereum anchor service for periodic Merkle root syncing
"""

from . import anchor

__all__ = ["anchor"]
