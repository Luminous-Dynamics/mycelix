#!/usr/bin/env python3

# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root"""
Holochain → Ethereum Anchor Service

Periodically syncs reputation data from Holochain DHT to Ethereum/Polygon
by building Merkle trees and anchoring roots on-chain.

This is the "Layer 2" component of the hybrid architecture:
- Holochain: FREE, high-frequency operations (99.9% of transactions)
- Ethereum/Polygon: PAID, periodic anchoring (maybe 24 tx/day = ~$0.24/day on Polygon)

Architecture:
    ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
    │   Holochain     │     │  Anchor Service │     │    Ethereum/    │
    │      DHT        │────▶│                 │────▶│    Polygon      │
    │                 │     │  - Collect data │     │                 │
    │  - Gradients    │     │  - Build Merkle │     │  - Merkle roots │
    │  - Reputation   │     │  - Anchor roots │     │  - Disputes     │
    │  - Byzantine    │     │  - Store proofs │     │  - Payments     │
    └─────────────────┘     └─────────────────┘     └─────────────────┘

Usage:
    python anchor_service.py --network sepolia --interval 3600

    # Or as a module
    from services.anchor import AnchorService
    service = AnchorService(network="polygon-amoy")
    await service.run()
"""

import os
import sys
import asyncio
import json
import hashlib
import argparse
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Dict, Any, Optional, Tuple
from pathlib import Path

# Add parent paths for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "0TML" / "src"))

try:
    from web3 import Web3
    from eth_account import Account
    HAS_WEB3 = True
except ImportError:
    HAS_WEB3 = False
    Web3 = None
    Account = None

try:
    import websockets
    import msgpack
    HAS_HOLOCHAIN = True
except ImportError:
    HAS_HOLOCHAIN = False


# Network configurations
NETWORKS = {
    "sepolia": {
        "chain_id": 11155111,
        "rpc_url": "https://ethereum-sepolia-rpc.publicnode.com",
        "reputation_contract": "0xf3B343888a9b82274cEfaa15921252DB6c5f48C9",
        "explorer": "https://sepolia.etherscan.io",
    },
    "polygon-amoy": {
        "chain_id": 80002,
        "rpc_url": "https://rpc-amoy.polygon.technology",
        "reputation_contract": "0x0000000000000000000000000000000000000000",  # TBD
        "explorer": "https://amoy.polygonscan.com",
    },
    "polygon": {
        "chain_id": 137,
        "rpc_url": "https://polygon-rpc.com",
        "reputation_contract": "0x0000000000000000000000000000000000000000",  # TBD
        "explorer": "https://polygonscan.com",
    },
    "local": {
        "chain_id": 31337,
        "rpc_url": "http://127.0.0.1:8545",
        "reputation_contract": "0x5FbDB2315678afecb367f032d93F642f64180aa3",
        "explorer": "http://localhost:8545",
    },
}

REPUTATION_ABI = [
    {
        "inputs": [
            {"name": "root", "type": "bytes32"},
            {"name": "epoch", "type": "uint256"},
        ],
        "name": "anchorMerkleRoot",
        "outputs": [],
        "stateMutability": "nonpayable",
        "type": "function",
    },
    {
        "inputs": [{"name": "epoch", "type": "uint256"}],
        "name": "getMerkleRoot",
        "outputs": [{"name": "", "type": "bytes32"}],
        "stateMutability": "view",
        "type": "function",
    },
]


@dataclass
class ReputationUpdate:
    """Single reputation update from Holochain"""
    node_id: str
    score: float
    round_num: int
    timestamp: int
    byzantine_flags: int = 0
    pogq_score: float = 0.0


@dataclass
class MerkleProof:
    """Merkle proof for a single leaf"""
    leaf_hash: bytes
    proof: List[bytes]
    index: int


@dataclass
class AnchorResult:
    """Result of an anchoring operation"""
    success: bool
    epoch: int
    merkle_root: str
    tx_hash: Optional[str] = None
    gas_used: Optional[int] = None
    block_number: Optional[int] = None
    error: Optional[str] = None
    updates_count: int = 0


class MerkleTree:
    """Simple Merkle tree implementation for reputation anchoring"""

    def __init__(self, leaves: List[bytes]):
        self.leaves = leaves
        self.tree = self._build_tree(leaves)
        self.root = self.tree[-1][0] if self.tree else b"\x00" * 32

    def _hash(self, data: bytes) -> bytes:
        """SHA256 hash"""
        return hashlib.sha256(data).digest()

    def _hash_pair(self, left: bytes, right: bytes) -> bytes:
        """Hash two nodes together (sorted for consistency)"""
        if left < right:
            return self._hash(left + right)
        return self._hash(right + left)

    def _build_tree(self, leaves: List[bytes]) -> List[List[bytes]]:
        """Build the tree from leaves"""
        if not leaves:
            return [[b"\x00" * 32]]

        # Hash leaves
        current_level = [self._hash(leaf) for leaf in leaves]
        tree = [current_level]

        # Build up the tree
        while len(current_level) > 1:
            next_level = []
            for i in range(0, len(current_level), 2):
                left = current_level[i]
                right = current_level[i + 1] if i + 1 < len(current_level) else left
                next_level.append(self._hash_pair(left, right))
            tree.append(next_level)
            current_level = next_level

        return tree

    def get_proof(self, index: int) -> MerkleProof:
        """Get proof for a leaf at index"""
        if index >= len(self.leaves):
            raise ValueError(f"Index {index} out of range")

        proof = []
        current_index = index

        for level in self.tree[:-1]:  # Exclude root
            sibling_index = current_index ^ 1  # XOR to get sibling
            if sibling_index < len(level):
                proof.append(level[sibling_index])
            current_index //= 2

        return MerkleProof(
            leaf_hash=self.tree[0][index],
            proof=proof,
            index=index,
        )

    def verify_proof(self, leaf: bytes, proof: MerkleProof) -> bool:
        """Verify a Merkle proof"""
        computed_hash = self._hash(leaf)
        index = proof.index

        for sibling_hash in proof.proof:
            if index % 2 == 0:
                computed_hash = self._hash_pair(computed_hash, sibling_hash)
            else:
                computed_hash = self._hash_pair(sibling_hash, computed_hash)
            index //= 2

        return computed_hash == self.root


class HolochainPoller:
    """Polls Holochain for reputation updates"""

    def __init__(
        self,
        admin_url: str = "ws://localhost:8888",
        app_url: str = "ws://localhost:8889",
        app_id: str = "mycelix-fl",
    ):
        self.admin_url = admin_url
        self.app_url = app_url
        self.app_id = app_id
        self.app_ws = None
        self.cell_id = None
        self.request_id = 0

    async def connect(self):
        """Connect to Holochain conductor"""
        if not HAS_HOLOCHAIN:
            raise ImportError("websockets and msgpack required: pip install websockets msgpack")

        self.app_ws = await websockets.connect(
            self.app_url,
            additional_headers={"Origin": "http://localhost"},
            ping_interval=20,
        )

        # Discover cell ID
        admin_ws = await websockets.connect(
            self.admin_url,
            additional_headers={"Origin": "http://localhost"},
        )

        try:
            request = {"type": "list_apps", "value": {"status_filter": None}}
            await admin_ws.send(msgpack.packb(request))
            response = msgpack.unpackb(await admin_ws.recv(), raw=False)

            for app in response.get("value", {}).get("apps", []):
                if app.get("installed_app_id") == self.app_id:
                    cell_data = app.get("cell_data", [])
                    if cell_data:
                        self.cell_id = cell_data[0].get("cell_id")
                        break
        finally:
            await admin_ws.close()

        print(f"Connected to Holochain (cell: {self.cell_id[:16] if self.cell_id else 'unknown'}...)")

    async def disconnect(self):
        """Disconnect from Holochain"""
        if self.app_ws:
            await self.app_ws.close()
            self.app_ws = None

    async def _call_zome(self, fn_name: str, payload: Any) -> Any:
        """Call FL coordinator zome function"""
        if not self.app_ws or not self.cell_id:
            raise RuntimeError("Not connected")

        self.request_id += 1
        request = {
            "type": "call_zome",
            "data": {
                "cell_id": self.cell_id,
                "zome_name": "federated_learning_coordinator",
                "fn_name": fn_name,
                "payload": payload,
                "cap_secret": None,
                "provenance": None,
            },
            "id": str(self.request_id),
        }

        await self.app_ws.send(msgpack.packb(request))
        response = msgpack.unpackb(await self.app_ws.recv(), raw=False)

        if "error" in response:
            raise RuntimeError(f"Zome error: {response['error']}")

        data = response.get("data", {})
        return data.get("payload") if "payload" in data else data

    async def get_reputation_updates(
        self,
        since_epoch: int,
    ) -> List[ReputationUpdate]:
        """Get all reputation updates since an epoch"""
        updates = []

        try:
            # Get all nodes and their reputations
            # This would call a zome function that returns updates since epoch
            # For now, simulate by getting recent gradients
            result = await self._call_zome("get_round_matl_stats", since_epoch)

            if isinstance(result, dict):
                # Parse reputation data from stats
                for node_id, data in result.get("node_stats", {}).items():
                    updates.append(ReputationUpdate(
                        node_id=node_id,
                        score=data.get("reputation", 0.5),
                        round_num=data.get("last_round", 0),
                        timestamp=data.get("timestamp", 0),
                        byzantine_flags=data.get("byzantine_flags", 0),
                        pogq_score=data.get("avg_pogq", 0.0),
                    ))

        except Exception as e:
            print(f"Warning: Failed to get reputation updates: {e}")

        return updates

    async def get_all_reputations(self) -> List[ReputationUpdate]:
        """Get all current reputations (for initial sync)

        Uses asyncio.gather for parallel fetching of round data.
        """
        updates = []

        try:
            # Parallel fetch of last 10 rounds
            async def fetch_round(round_num: int):
                try:
                    return round_num, await self._call_zome("get_round_gradients", round_num)
                except Exception:
                    return round_num, None

            # Fetch all rounds in parallel
            tasks = [fetch_round(r) for r in range(1, 11)]
            results = await asyncio.gather(*tasks, return_exceptions=True)

            for result in results:
                if isinstance(result, Exception):
                    continue
                round_num, gradients = result
                if gradients is None:
                    continue

                for item in (gradients if isinstance(gradients, list) else []):
                    _, gradient = item if isinstance(item, tuple) else (None, item)
                    if isinstance(gradient, dict):
                        updates.append(ReputationUpdate(
                            node_id=gradient.get("node_id", ""),
                            score=gradient.get("reputation_score", 0.5),
                            round_num=round_num,
                            timestamp=gradient.get("timestamp", 0),
                            pogq_score=gradient.get("pogq_score", 0.0),
                        ))

        except Exception as e:
            print(f"Warning: Failed to get reputations: {e}")

        return updates


class EthereumAnchor:
    """Anchors Merkle roots to Ethereum/Polygon"""

    def __init__(
        self,
        network: str = "sepolia",
        private_key: Optional[str] = None,
    ):
        if not HAS_WEB3:
            raise ImportError("web3 required: pip install web3")

        self.network = network
        self.config = NETWORKS.get(network, NETWORKS["sepolia"])

        self.w3 = Web3(Web3.HTTPProvider(self.config["rpc_url"]))

        self.account = None
        if private_key:
            if not private_key.startswith("0x"):
                private_key = f"0x{private_key}"
            self.account = Account.from_key(private_key)

        self.contract = self.w3.eth.contract(
            address=Web3.to_checksum_address(self.config["reputation_contract"]),
            abi=REPUTATION_ABI,
        )

    async def anchor_merkle_root(
        self,
        root: bytes,
        epoch: int,
    ) -> AnchorResult:
        """Anchor a Merkle root to the blockchain"""
        if not self.account:
            return AnchorResult(
                success=False,
                epoch=epoch,
                merkle_root=root.hex(),
                error="No private key configured",
            )

        try:
            # Build transaction
            tx = self.contract.functions.anchorMerkleRoot(
                root,
                epoch,
            ).build_transaction({
                "from": self.account.address,
                "nonce": self.w3.eth.get_transaction_count(self.account.address),
                "gas": 100000,
                "gasPrice": self.w3.eth.gas_price,
            })

            # Sign and send
            signed_tx = self.w3.eth.account.sign_transaction(tx, self.account.key)
            tx_hash = self.w3.eth.send_raw_transaction(signed_tx.rawTransaction)

            # Wait for receipt
            receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)

            return AnchorResult(
                success=receipt["status"] == 1,
                epoch=epoch,
                merkle_root=root.hex(),
                tx_hash=tx_hash.hex(),
                gas_used=receipt["gasUsed"],
                block_number=receipt["blockNumber"],
            )

        except Exception as e:
            return AnchorResult(
                success=False,
                epoch=epoch,
                merkle_root=root.hex(),
                error=str(e),
            )

    async def get_anchored_root(self, epoch: int) -> Optional[bytes]:
        """Get the anchored Merkle root for an epoch"""
        try:
            root = self.contract.functions.getMerkleRoot(epoch).call()
            return root if root != b"\x00" * 32 else None
        except Exception:
            return None

    def get_tx_url(self, tx_hash: str) -> str:
        """Get block explorer URL for a transaction"""
        return f"{self.config['explorer']}/tx/{tx_hash}"


class AnchorService:
    """
    Main anchor service that coordinates Holochain → Ethereum syncing.

    Runs periodically to:
    1. Poll Holochain for reputation updates
    2. Build Merkle tree of updates
    3. Anchor root to Ethereum/Polygon
    4. Store proofs back to Holochain
    """

    def __init__(
        self,
        network: str = "sepolia",
        private_key: Optional[str] = None,
        holochain_admin_url: str = "ws://localhost:8888",
        holochain_app_url: str = "ws://localhost:8889",
        anchor_interval: int = 3600,  # 1 hour default
        min_updates: int = 1,  # Minimum updates before anchoring
    ):
        self.network = network
        self.private_key = private_key or os.environ.get("PRIVATE_KEY")
        self.anchor_interval = anchor_interval
        self.min_updates = min_updates

        self.holochain = HolochainPoller(
            admin_url=holochain_admin_url,
            app_url=holochain_app_url,
        )
        self.ethereum = EthereumAnchor(
            network=network,
            private_key=self.private_key,
        )

        self.current_epoch = 0
        self.last_anchor_time = 0
        self.pending_updates: List[ReputationUpdate] = []
        self.anchor_history: List[AnchorResult] = []
        self.running = False

    def _serialize_update(self, update: ReputationUpdate) -> bytes:
        """Serialize a reputation update for Merkle tree"""
        data = {
            "node_id": update.node_id,
            "score": update.score,
            "round_num": update.round_num,
            "timestamp": update.timestamp,
            "byzantine_flags": update.byzantine_flags,
            "pogq_score": update.pogq_score,
        }
        return json.dumps(data, sort_keys=True).encode()

    async def collect_updates(self) -> List[ReputationUpdate]:
        """Collect reputation updates from Holochain"""
        try:
            updates = await self.holochain.get_reputation_updates(self.current_epoch)
            print(f"Collected {len(updates)} reputation updates from Holochain")
            return updates
        except Exception as e:
            print(f"Failed to collect updates: {e}")
            return []

    async def anchor_epoch(self) -> AnchorResult:
        """Anchor current pending updates to blockchain"""
        if len(self.pending_updates) < self.min_updates:
            return AnchorResult(
                success=False,
                epoch=self.current_epoch,
                merkle_root="",
                error=f"Not enough updates ({len(self.pending_updates)} < {self.min_updates})",
            )

        # Build Merkle tree
        leaves = [self._serialize_update(u) for u in self.pending_updates]
        tree = MerkleTree(leaves)

        print(f"Built Merkle tree with {len(leaves)} leaves, root: {tree.root.hex()[:16]}...")

        # Anchor to blockchain
        result = await self.ethereum.anchor_merkle_root(tree.root, self.current_epoch)
        result.updates_count = len(self.pending_updates)

        if result.success:
            print(f"Anchored epoch {self.current_epoch} to {self.network}")
            print(f"  TX: {self.ethereum.get_tx_url(result.tx_hash)}")
            print(f"  Gas used: {result.gas_used}")

            # Store proofs (would send back to Holochain)
            for i, update in enumerate(self.pending_updates):
                proof = tree.get_proof(i)
                # TODO: Store proof in Holochain for future verification
                pass

            # Reset for next epoch
            self.current_epoch += 1
            self.pending_updates = []
            self.last_anchor_time = int(datetime.now().timestamp())
            self.anchor_history.append(result)
        else:
            print(f"Failed to anchor epoch {self.current_epoch}: {result.error}")

        return result

    async def run_once(self) -> Optional[AnchorResult]:
        """Run a single anchor cycle"""
        # Collect new updates
        new_updates = await self.collect_updates()
        self.pending_updates.extend(new_updates)

        # Remove duplicates (by node_id, keeping latest)
        seen = {}
        for update in reversed(self.pending_updates):
            if update.node_id not in seen:
                seen[update.node_id] = update
        self.pending_updates = list(reversed(seen.values()))

        # Check if we should anchor
        now = int(datetime.now().timestamp())
        time_since_anchor = now - self.last_anchor_time

        should_anchor = (
            len(self.pending_updates) >= self.min_updates
            and time_since_anchor >= self.anchor_interval
        )

        if should_anchor:
            return await self.anchor_epoch()

        print(f"Not anchoring yet: {len(self.pending_updates)} updates, "
              f"{time_since_anchor}s since last anchor")
        return None

    async def run(self):
        """Run the anchor service continuously"""
        print(f"Starting Anchor Service")
        print(f"  Network: {self.network}")
        print(f"  Interval: {self.anchor_interval}s")
        print(f"  Min updates: {self.min_updates}")

        try:
            await self.holochain.connect()
            self.running = True

            while self.running:
                try:
                    await self.run_once()
                except Exception as e:
                    print(f"Error in anchor cycle: {e}")

                # Sleep until next cycle (check every minute)
                await asyncio.sleep(60)

        finally:
            await self.holochain.disconnect()
            self.running = False

    def stop(self):
        """Stop the anchor service"""
        self.running = False

    def get_status(self) -> Dict[str, Any]:
        """Get current service status"""
        return {
            "running": self.running,
            "network": self.network,
            "current_epoch": self.current_epoch,
            "pending_updates": len(self.pending_updates),
            "last_anchor_time": self.last_anchor_time,
            "total_anchors": len(self.anchor_history),
            "anchor_interval": self.anchor_interval,
        }


async def main():
    """CLI entry point"""
    parser = argparse.ArgumentParser(description="Holochain → Ethereum Anchor Service")
    parser.add_argument(
        "--network",
        choices=list(NETWORKS.keys()),
        default="sepolia",
        help="Target blockchain network",
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=3600,
        help="Anchor interval in seconds (default: 3600)",
    )
    parser.add_argument(
        "--min-updates",
        type=int,
        default=1,
        help="Minimum updates before anchoring (default: 1)",
    )
    parser.add_argument(
        "--holochain-admin",
        default="ws://localhost:8888",
        help="Holochain admin WebSocket URL",
    )
    parser.add_argument(
        "--holochain-app",
        default="ws://localhost:8889",
        help="Holochain app WebSocket URL",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run once and exit",
    )

    args = parser.parse_args()

    service = AnchorService(
        network=args.network,
        holochain_admin_url=args.holochain_admin,
        holochain_app_url=args.holochain_app,
        anchor_interval=args.interval,
        min_updates=args.min_updates,
    )

    if args.once:
        await service.holochain.connect()
        try:
            result = await service.run_once()
            if result:
                print(f"Anchor result: {result}")
        finally:
            await service.holochain.disconnect()
    else:
        await service.run()


if __name__ == "__main__":
    asyncio.run(main())
