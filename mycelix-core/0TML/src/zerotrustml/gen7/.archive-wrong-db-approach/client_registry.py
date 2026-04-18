# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
"""
Phase 2.5 Week 2: Client Registry & Nonce Tracking

Database-backed storage for:
  1. Client public keys and metadata
  2. Used nonces for replay attack prevention
  3. Participation statistics
  4. Automatic nonce cleanup

Supports both SQLite (development) and PostgreSQL (production).

Usage:
    # Initialize registry
    registry = ClientRegistry("sqlite:///ztml_clients.db")

    # Register client
    client_id = registry.register_client(public_key)

    # Verify proof with nonce tracking
    if not registry.is_nonce_used(client_id, nonce):
        # Verify proof...
        registry.mark_nonce_used(client_id, nonce, round_number, timestamp)
    else:
        # Replay attack detected!
        pass
"""

import time
import sqlite3
from typing import Optional, List, Tuple
from dataclasses import dataclass
from pathlib import Path
import hashlib


@dataclass
class ClientInfo:
    """Information about a registered client"""
    client_id: bytes
    public_key: bytes
    registered_at: int
    last_seen: int
    total_rounds: int
    reputation_score: float


class ClientRegistry:
    """
    Database-backed client registry for Phase 2.5

    Manages:
        - Client registrations (public keys)
        - Nonce tracking (replay attack prevention)
        - Participation statistics
        - Automatic cleanup of expired nonces

    Database Schema:
        clients(
            client_id BLOB PRIMARY KEY,
            public_key BLOB NOT NULL,
            registered_at INTEGER NOT NULL,
            last_seen INTEGER,
            total_rounds INTEGER DEFAULT 0,
            reputation_score REAL DEFAULT 1.0
        )

        nonces(
            client_id BLOB NOT NULL,
            nonce BLOB NOT NULL,
            round_number INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            PRIMARY KEY (client_id, nonce),
            FOREIGN KEY (client_id) REFERENCES clients(client_id)
        )
    """

    def __init__(
        self,
        database_url: str = "sqlite:///ztml_clients.db",
        auto_cleanup: bool = True,
        nonce_max_age: int = 3600,
    ):
        """
        Initialize client registry

        Args:
            database_url: Database connection URL
                - SQLite: "sqlite:///path/to/db.db" (default for development)
                - PostgreSQL: "postgresql://user:pass@host:port/dbname" (production)
            auto_cleanup: Automatically clean up expired nonces on operations
            nonce_max_age: Maximum nonce age in seconds before cleanup (default: 1 hour)
        """
        self.database_url = database_url
        self.auto_cleanup = auto_cleanup
        self.nonce_max_age = nonce_max_age

        # Parse database URL
        if database_url.startswith("sqlite://"):
            self.db_type = "sqlite"
            self.db_path = database_url.replace("sqlite:///", "")
            # Handle in-memory database
            if self.db_path.startswith(":memory:"):
                self.conn = sqlite3.connect(":memory:", check_same_thread=False)
            else:
                # Ensure parent directory exists for file-based database
                Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
                self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        elif database_url.startswith("postgresql://"):
            self.db_type = "postgresql"
            raise NotImplementedError(
                "PostgreSQL support not yet implemented. "
                "Use SQLite for now: 'sqlite:///ztml_clients.db'"
            )
        else:
            raise ValueError(f"Unsupported database URL: {database_url}")

        # Create tables
        self._create_tables()

    def _create_tables(self):
        """Create database tables if they don't exist"""
        cursor = self.conn.cursor()

        # Create clients table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS clients (
                client_id BLOB PRIMARY KEY,
                public_key BLOB NOT NULL,
                registered_at INTEGER NOT NULL,
                last_seen INTEGER,
                total_rounds INTEGER DEFAULT 0,
                reputation_score REAL DEFAULT 1.0
            )
        """)

        # Create nonces table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS nonces (
                client_id BLOB NOT NULL,
                nonce BLOB NOT NULL,
                round_number INTEGER NOT NULL,
                timestamp INTEGER NOT NULL,
                PRIMARY KEY (client_id, nonce),
                FOREIGN KEY (client_id) REFERENCES clients(client_id)
            )
        """)

        # Create indexes for performance
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_nonces_round
            ON nonces(round_number)
        """)

        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_nonces_timestamp
            ON nonces(timestamp)
        """)

        self.conn.commit()

    def register_client(self, public_key: bytes) -> bytes:
        """
        Register new client

        Args:
            public_key: Dilithium5 public key (2592 bytes)

        Returns:
            client_id: SHA-256 hash of public key (32 bytes)

        Raises:
            ValueError: If client already registered
        """
        # Compute client ID (SHA-256 of public key)
        client_id = hashlib.sha256(public_key).digest()

        # Check if already registered
        cursor = self.conn.cursor()
        cursor.execute("SELECT client_id FROM clients WHERE client_id = ?", (client_id,))
        if cursor.fetchone() is not None:
            raise ValueError(f"Client already registered: {client_id.hex()[:16]}...")

        # Insert new client
        timestamp = int(time.time())
        cursor.execute(
            """
            INSERT INTO clients (client_id, public_key, registered_at, last_seen, total_rounds, reputation_score)
            VALUES (?, ?, ?, ?, 0, 1.0)
            """,
            (client_id, public_key, timestamp, timestamp)
        )
        self.conn.commit()

        return client_id

    def get_client(self, client_id: bytes) -> Optional[ClientInfo]:
        """
        Retrieve client information

        Args:
            client_id: Client ID (32 bytes)

        Returns:
            ClientInfo or None if not found
        """
        cursor = self.conn.cursor()
        cursor.execute(
            """
            SELECT client_id, public_key, registered_at, last_seen, total_rounds, reputation_score
            FROM clients
            WHERE client_id = ?
            """,
            (client_id,)
        )

        row = cursor.fetchone()
        if row is None:
            return None

        return ClientInfo(
            client_id=row[0],
            public_key=row[1],
            registered_at=row[2],
            last_seen=row[3],
            total_rounds=row[4],
            reputation_score=row[5],
        )

    def get_public_key(self, client_id: bytes) -> Optional[bytes]:
        """
        Get client's public key

        Args:
            client_id: Client ID (32 bytes)

        Returns:
            Public key bytes or None if not found
        """
        cursor = self.conn.cursor()
        cursor.execute("SELECT public_key FROM clients WHERE client_id = ?", (client_id,))
        row = cursor.fetchone()
        return row[0] if row is not None else None

    def is_nonce_used(self, client_id: bytes, nonce: bytes) -> bool:
        """
        Check if nonce has been used

        Args:
            client_id: Client ID (32 bytes)
            nonce: Nonce bytes (32 bytes)

        Returns:
            True if nonce already used, False otherwise
        """
        # Auto-cleanup expired nonces if enabled
        if self.auto_cleanup:
            self._cleanup_expired_nonces()

        cursor = self.conn.cursor()
        cursor.execute(
            "SELECT 1 FROM nonces WHERE client_id = ? AND nonce = ?",
            (client_id, nonce)
        )
        return cursor.fetchone() is not None

    def mark_nonce_used(
        self,
        client_id: bytes,
        nonce: bytes,
        round_number: int,
        timestamp: int,
    ) -> None:
        """
        Mark nonce as used

        Args:
            client_id: Client ID (32 bytes)
            nonce: Nonce bytes (32 bytes)
            round_number: Training round number
            timestamp: Unix timestamp when proof was generated

        Raises:
            ValueError: If nonce already used (replay attack)
        """
        if self.is_nonce_used(client_id, nonce):
            raise ValueError(f"Nonce already used (replay attack detected): {nonce.hex()[:16]}...")

        cursor = self.conn.cursor()
        cursor.execute(
            """
            INSERT INTO nonces (client_id, nonce, round_number, timestamp)
            VALUES (?, ?, ?, ?)
            """,
            (client_id, nonce, round_number, timestamp)
        )
        self.conn.commit()

    def update_participation(self, client_id: bytes) -> None:
        """
        Update client participation statistics

        Args:
            client_id: Client ID (32 bytes)
        """
        timestamp = int(time.time())
        cursor = self.conn.cursor()
        cursor.execute(
            """
            UPDATE clients
            SET last_seen = ?, total_rounds = total_rounds + 1
            WHERE client_id = ?
            """,
            (timestamp, client_id)
        )
        self.conn.commit()

    def cleanup_old_nonces(self, max_age_seconds: int = None) -> int:
        """
        Remove expired nonces

        Args:
            max_age_seconds: Maximum nonce age (default: self.nonce_max_age)

        Returns:
            Number of nonces deleted
        """
        if max_age_seconds is None:
            max_age_seconds = self.nonce_max_age

        cutoff_timestamp = int(time.time()) - max_age_seconds
        cursor = self.conn.cursor()
        cursor.execute("DELETE FROM nonces WHERE timestamp < ?", (cutoff_timestamp,))
        deleted_count = cursor.rowcount
        self.conn.commit()

        return deleted_count

    def _cleanup_expired_nonces(self):
        """Internal: Clean up expired nonces (called automatically)"""
        self.cleanup_old_nonces(self.nonce_max_age)

    def get_client_count(self) -> int:
        """Get number of registered clients"""
        cursor = self.conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM clients")
        return cursor.fetchone()[0]

    def get_nonce_count(self) -> int:
        """Get number of stored nonces"""
        cursor = self.conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM nonces")
        return cursor.fetchone()[0]

    def get_client_nonces(self, client_id: bytes) -> List[Tuple[bytes, int, int]]:
        """
        Get all nonces for a specific client

        Args:
            client_id: Client ID (32 bytes)

        Returns:
            List of (nonce, round_number, timestamp) tuples
        """
        cursor = self.conn.cursor()
        cursor.execute(
            """
            SELECT nonce, round_number, timestamp
            FROM nonces
            WHERE client_id = ?
            ORDER BY timestamp DESC
            """,
            (client_id,)
        )
        return cursor.fetchall()

    def clear_all_nonces(self) -> int:
        """
        Clear all nonces (e.g., at round boundary)

        Returns:
            Number of nonces deleted
        """
        cursor = self.conn.cursor()
        cursor.execute("DELETE FROM nonces")
        deleted_count = cursor.rowcount
        self.conn.commit()
        return deleted_count

    def close(self):
        """Close database connection"""
        if hasattr(self, 'conn'):
            self.conn.close()

    def __enter__(self):
        """Context manager support"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager support"""
        self.close()


# Example usage
if __name__ == "__main__":
    print("Phase 2.5 Week 2: Client Registry Example")
    print("=" * 60)

    # Create in-memory SQLite database for demo
    with ClientRegistry("sqlite:///:memory:") as registry:
        # Simulate client registration
        import os

        # Generate fake public key (2592 bytes for Dilithium5)
        public_key_1 = os.urandom(2592)
        public_key_2 = os.urandom(2592)

        # Register clients
        client_id_1 = registry.register_client(public_key_1)
        client_id_2 = registry.register_client(public_key_2)

        print(f"✅ Registered client 1: {client_id_1.hex()[:16]}...")
        print(f"✅ Registered client 2: {client_id_2.hex()[:16]}...")
        print(f"✅ Total clients: {registry.get_client_count()}")
        print()

        # Simulate nonce usage
        nonce_1 = os.urandom(32)
        nonce_2 = os.urandom(32)

        # Check nonce not used
        assert not registry.is_nonce_used(client_id_1, nonce_1)
        print("✅ Nonce 1 not used (expected)")

        # Mark nonce as used
        registry.mark_nonce_used(client_id_1, nonce_1, round_number=1, timestamp=int(time.time()))
        print("✅ Nonce 1 marked as used")

        # Check nonce now used
        assert registry.is_nonce_used(client_id_1, nonce_1)
        print("✅ Nonce 1 detected as used (replay prevention)")

        # Try to reuse nonce (should fail)
        try:
            registry.mark_nonce_used(client_id_1, nonce_1, round_number=1, timestamp=int(time.time()))
            print("❌ ERROR: Nonce reuse should have failed!")
        except ValueError as e:
            print(f"✅ Nonce reuse prevented: {str(e)[:50]}...")

        print()
        print(f"✅ Total nonces tracked: {registry.get_nonce_count()}")

        # Test cleanup
        print("✅ Testing nonce cleanup...")
        old_count = registry.get_nonce_count()
        deleted = registry.cleanup_old_nonces(max_age_seconds=0)  # Delete all
        new_count = registry.get_nonce_count()
        print(f"✅ Deleted {deleted} expired nonces ({old_count} -> {new_count})")

        # Update participation
        registry.update_participation(client_id_1)
        client_info = registry.get_client(client_id_1)
        print(f"✅ Client 1 participation updated: {client_info.total_rounds} rounds")

        print()
        print("🎉 Client Registry: ALL TESTS PASSED")
