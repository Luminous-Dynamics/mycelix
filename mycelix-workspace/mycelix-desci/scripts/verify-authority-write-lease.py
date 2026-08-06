#!/usr/bin/env python3
"""Independently verify a Mycelix-DeSci authority-write lease.

This verifier intentionally reimplements the v1 canonical codec rather than
calling repository Rust code. It checks the Ed25519 signature, trusted issuer,
lease lifetime, and optional deployment/database/epoch/scope bindings.
"""

from __future__ import annotations

import argparse
import calendar
import hashlib
import json
import re
import struct
import sys
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, NoReturn

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

PROTOCOL = "mycelix-desci-authority-write-lease"
PROTOCOL_VERSION = 1
CODEC = "mycelix-canonical-binary-v1"
SCHEMA_VERSION = 1
LEASE_DOMAIN = b"MYCELIX-DESCI-AUTHORITY-WRITE-LEASE\x00"
SIGNED_DOMAIN = b"MYCELIX-DESCI-SIGNED-AUTHORITY-WRITE-LEASE\x00"
MAX_DURATION_SECONDS = 15 * 60

PHASE_CODES = {"bootstrap": 1, "epoch": 2}
SCOPE_CODES = {
    "scientific_event": 1,
    "credential_registry": 2,
    "credential_governance": 3,
    "database_epoch_promotion": 4,
    "recovery_reconciliation": 5,
    "delivery_acknowledgement": 6,
    "checkpoint_mirror": 7,
    "outbox_delivery": 8,
    "schema_migration": 9,
}
RFC3339_RE = re.compile(
    r"^(?P<date>\d{4}-\d{2}-\d{2})T(?P<time>\d{2}:\d{2}:\d{2})"
    r"(?:\.(?P<fraction>\d{1,9}))?Z$"
)


def fail(message: str) -> NoReturn:
    raise ValueError(message)


def require_dict(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        fail(f"{label} must be a JSON object")
    return value


def require_text(value: Any, label: str, max_bytes: int = 4096) -> str:
    if not isinstance(value, str):
        fail(f"{label} must be text")
    encoded = value.encode("utf-8")
    if not encoded or len(encoded) > max_bytes or any(ord(char) < 32 or ord(char) == 127 for char in value):
        fail(f"{label} must contain 1-{max_bytes} printable UTF-8 bytes")
    return value


def require_u64(value: Any, label: str, *, positive: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        fail(f"{label} must be an integer")
    minimum = 1 if positive else 0
    if value < minimum or value > 0xFFFF_FFFF_FFFF_FFFF:
        fail(f"{label} is outside the unsigned 64-bit range")
    return value


def require_u16(value: Any, label: str) -> int:
    value = require_u64(value, label)
    if value > 0xFFFF:
        fail(f"{label} is outside the unsigned 16-bit range")
    return value


def require_bytes(value: Any, length: int, label: str) -> bytes:
    if isinstance(value, str):
        try:
            decoded = bytes.fromhex(value)
        except ValueError as error:
            fail(f"{label} is not valid hexadecimal: {error}")
    elif isinstance(value, list) and all(isinstance(item, int) and not isinstance(item, bool) and 0 <= item <= 255 for item in value):
        decoded = bytes(value)
    else:
        fail(f"{label} must be a hexadecimal string or byte array")
    if len(decoded) != length:
        fail(f"{label} must contain exactly {length} bytes")
    return decoded


def parse_rfc3339(value: Any, label: str) -> tuple[int, int, str]:
    text = require_text(value, label, 64)
    match = RFC3339_RE.fullmatch(text)
    if match is None:
        fail(f"{label} must be UTC RFC3339 with at most nine fractional digits")
    base = datetime.strptime(
        f"{match.group('date')}T{match.group('time')}", "%Y-%m-%dT%H:%M:%S"
    ).replace(tzinfo=timezone.utc)
    seconds = calendar.timegm(base.utctimetuple())
    nanos = int((match.group("fraction") or "").ljust(9, "0") or "0")
    return seconds, nanos, text


def parse_at(value: str | None) -> tuple[int, int]:
    if value is None:
        now = datetime.now(timezone.utc)
        return calendar.timegm(now.utctimetuple()), now.microsecond * 1000
    seconds, nanos, _ = parse_rfc3339(value, "--at")
    return seconds, nanos


def instant_ns(value: tuple[int, int]) -> int:
    return value[0] * 1_000_000_000 + value[1]


def push_len(buffer: bytearray, length: int) -> None:
    if length < 0 or length > 0xFFFF_FFFF:
        fail("canonical field length exceeds u32")
    buffer.extend(struct.pack(">I", length))


def push_bytes(buffer: bytearray, value: bytes) -> None:
    push_len(buffer, len(value))
    buffer.extend(value)


def push_text(buffer: bytearray, value: str) -> None:
    push_bytes(buffer, value.encode("utf-8"))


def push_u16(buffer: bytearray, value: int) -> None:
    buffer.extend(struct.pack(">H", value))


def push_u64(buffer: bytearray, value: int) -> None:
    buffer.extend(struct.pack(">Q", value))


def push_i64(buffer: bytearray, value: int) -> None:
    buffer.extend(struct.pack(">q", value))


def push_option_u64(buffer: bytearray, value: Any, label: str) -> None:
    if value is None:
        buffer.append(0)
    else:
        buffer.append(1)
        push_u64(buffer, require_u64(value, label, positive=True))


def push_option_hash(buffer: bytearray, value: Any, label: str) -> None:
    if value is None:
        buffer.append(0)
    else:
        buffer.append(1)
        buffer.extend(require_bytes(value, 32, label))


def push_datetime(buffer: bytearray, value: tuple[int, int, str]) -> None:
    seconds, nanos, _ = value
    push_i64(buffer, seconds)
    buffer.extend(struct.pack(">I", nanos))


@dataclass(frozen=True)
class CanonicalLease:
    signing_bytes: bytes
    public_key: bytes
    signature: bytes
    lease: dict[str, Any]
    lease_hash_sha256: str
    signing_bytes_sha256: str


def canonicalize(envelope_value: Any) -> CanonicalLease:
    envelope = require_dict(envelope_value, "signed lease")
    expected_envelope = {
        "protocol",
        "protocol_version",
        "codec",
        "schema_version",
        "lease",
        "signer_key_id",
        "signer_public_key",
        "signature",
    }
    unknown = set(envelope) - expected_envelope
    missing = expected_envelope - set(envelope)
    if unknown or missing:
        fail(f"signed lease fields mismatch; missing={sorted(missing)}, unknown={sorted(unknown)}")

    protocol = require_text(envelope["protocol"], "protocol", 256)
    protocol_version = require_u16(envelope["protocol_version"], "protocol_version")
    codec = require_text(envelope["codec"], "codec", 256)
    schema_version = require_u16(envelope["schema_version"], "schema_version")
    if (protocol, protocol_version, codec, schema_version) != (
        PROTOCOL,
        PROTOCOL_VERSION,
        CODEC,
        SCHEMA_VERSION,
    ):
        fail("unsupported authority-write lease protocol envelope")

    lease = require_dict(envelope["lease"], "lease")
    expected_lease = {
        "lease_id",
        "deployment_id",
        "primary_id",
        "database_system_identifier",
        "postgres_timeline",
        "generation",
        "phase",
        "epoch_number",
        "epoch_hash",
        "allowed_scopes",
        "issued_at",
        "not_before",
        "expires_at",
        "reason",
    }
    unknown = set(lease) - expected_lease
    missing = expected_lease - set(lease)
    if unknown or missing:
        fail(f"lease fields mismatch; missing={sorted(missing)}, unknown={sorted(unknown)}")

    try:
        lease_id = uuid.UUID(require_text(lease["lease_id"], "lease_id", 64))
    except ValueError as error:
        fail(f"invalid lease_id: {error}")
    deployment_id = require_text(lease["deployment_id"], "deployment_id", 256)
    primary_id = require_text(lease["primary_id"], "primary_id", 256)
    system_id = require_text(
        lease["database_system_identifier"], "database_system_identifier", 256
    )
    timeline = require_u64(lease["postgres_timeline"], "postgres_timeline", positive=True)
    generation = require_u64(lease["generation"], "generation", positive=True)
    phase = require_text(lease["phase"], "phase", 32)
    if phase not in PHASE_CODES:
        fail(f"unknown phase {phase!r}")

    scopes_value = lease["allowed_scopes"]
    if not isinstance(scopes_value, list) or not scopes_value:
        fail("allowed_scopes must be a nonempty JSON array")
    scopes: list[str] = []
    for index, value in enumerate(scopes_value):
        scope = require_text(value, f"allowed_scopes[{index}]", 64)
        if scope not in SCOPE_CODES:
            fail(f"unknown write scope {scope!r}")
        scopes.append(scope)
    if len(scopes) != len(set(scopes)):
        fail("allowed_scopes contains duplicates")
    scopes.sort(key=SCOPE_CODES.__getitem__)

    issued_at = parse_rfc3339(lease["issued_at"], "issued_at")
    not_before = parse_rfc3339(lease["not_before"], "not_before")
    expires_at = parse_rfc3339(lease["expires_at"], "expires_at")
    issued_ns = instant_ns(issued_at[:2])
    not_before_ns = instant_ns(not_before[:2])
    expires_ns = instant_ns(expires_at[:2])
    if not_before_ns < issued_ns:
        fail("lease cannot become active before issuance")
    if expires_ns <= not_before_ns:
        fail("lease must expire after activation")
    if expires_ns - not_before_ns > MAX_DURATION_SECONDS * 1_000_000_000:
        fail("lease duration exceeds 15 minutes")

    epoch_number = lease["epoch_number"]
    epoch_hash = lease["epoch_hash"]
    if phase == "bootstrap":
        if epoch_number is not None or epoch_hash is not None:
            fail("bootstrap lease must not name an epoch")
    else:
        if epoch_number is None or epoch_hash is None:
            fail("epoch lease must name an epoch number and hash")

    reason = require_text(lease["reason"], "reason", 4096)
    lease_bytes = bytearray(LEASE_DOMAIN)
    lease_bytes.extend(lease_id.bytes)
    push_text(lease_bytes, deployment_id)
    push_text(lease_bytes, primary_id)
    push_text(lease_bytes, system_id)
    push_u64(lease_bytes, timeline)
    push_u64(lease_bytes, generation)
    lease_bytes.append(PHASE_CODES[phase])
    push_option_u64(lease_bytes, epoch_number, "epoch_number")
    push_option_hash(lease_bytes, epoch_hash, "epoch_hash")
    push_len(lease_bytes, len(scopes))
    for scope in scopes:
        lease_bytes.append(SCOPE_CODES[scope])
    push_datetime(lease_bytes, issued_at)
    push_datetime(lease_bytes, not_before)
    push_datetime(lease_bytes, expires_at)
    push_text(lease_bytes, reason)

    signer_key_id = require_text(envelope["signer_key_id"], "signer_key_id", 256)
    public_key = require_bytes(envelope["signer_public_key"], 32, "signer_public_key")
    signature = require_bytes(envelope["signature"], 64, "signature")

    signing_bytes = bytearray(SIGNED_DOMAIN)
    push_text(signing_bytes, protocol)
    push_u16(signing_bytes, protocol_version)
    push_text(signing_bytes, codec)
    push_u16(signing_bytes, schema_version)
    push_bytes(signing_bytes, bytes(lease_bytes))
    push_text(signing_bytes, signer_key_id)
    signing_bytes.extend(public_key)

    return CanonicalLease(
        signing_bytes=bytes(signing_bytes),
        public_key=public_key,
        signature=signature,
        lease=lease,
        lease_hash_sha256=hashlib.sha256(bytes(lease_bytes)).hexdigest(),
        signing_bytes_sha256=hashlib.sha256(bytes(signing_bytes)).hexdigest(),
    )


def load_json(path: Path, label: str) -> Any:
    try:
        if path.is_symlink() or not path.is_file():
            fail(f"{label} must be a regular non-symbolic-link file: {path}")
        return json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        fail(f"cannot read {label} {path}: {error}")
    except json.JSONDecodeError as error:
        fail(f"invalid JSON in {label} {path}: {error}")


def load_trust(path: Path) -> set[bytes]:
    value = load_json(path, "trust file")
    if not isinstance(value, list) or not value:
        fail("trust file must be a nonempty JSON array of Ed25519 public-key hex strings")
    keys = {require_bytes(item, 32, "trusted public key") for item in value}
    if len(keys) != len(value):
        fail("trust file contains duplicate public keys")
    return keys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lease", type=Path, help="signed lease JSON")
    parser.add_argument("--trust-file", required=True, type=Path)
    parser.add_argument("--at", help="verification time in UTC RFC3339; defaults to now")
    parser.add_argument("--clock-skew-seconds", type=int, default=30)
    parser.add_argument("--deployment-id")
    parser.add_argument("--primary-id")
    parser.add_argument("--database-system-identifier")
    parser.add_argument("--postgres-timeline", type=int)
    parser.add_argument("--minimum-generation", type=int)
    parser.add_argument("--phase", choices=sorted(PHASE_CODES))
    parser.add_argument("--epoch-number", type=int)
    parser.add_argument("--epoch-hash", help="32-byte hexadecimal content hash")
    parser.add_argument("--scope", choices=sorted(SCOPE_CODES))
    args = parser.parse_args()

    try:
        if not 0 <= args.clock_skew_seconds <= 300:
            fail("clock skew must be between zero and 300 seconds")
        canonical = canonicalize(load_json(args.lease, "signed lease"))
        trusted = load_trust(args.trust_file)
        if canonical.public_key not in trusted:
            fail("lease signer public key is not trusted")
        try:
            Ed25519PublicKey.from_public_bytes(canonical.public_key).verify(
                canonical.signature, canonical.signing_bytes
            )
        except InvalidSignature:
            fail("invalid Ed25519 lease signature")

        lease = canonical.lease
        now = parse_at(args.at)
        not_before = parse_rfc3339(lease["not_before"], "not_before")[:2]
        expires_at = parse_rfc3339(lease["expires_at"], "expires_at")[:2]
        now_ns = instant_ns(now)
        if now_ns + args.clock_skew_seconds * 1_000_000_000 < instant_ns(not_before):
            fail("lease is not active yet")
        # Expiration is strict by protocol: skew cannot extend it.
        if now_ns >= instant_ns(expires_at):
            fail("lease has expired")

        bindings = {
            "deployment_id": args.deployment_id,
            "primary_id": args.primary_id,
            "database_system_identifier": args.database_system_identifier,
            "postgres_timeline": args.postgres_timeline,
            "phase": args.phase,
            "epoch_number": args.epoch_number,
        }
        for field, expected in bindings.items():
            if expected is not None and lease[field] != expected:
                fail(f"{field} mismatch: expected {expected!r}, found {lease[field]!r}")
        if args.minimum_generation is not None and lease["generation"] < args.minimum_generation:
            fail(
                f"generation rollback: expected at least {args.minimum_generation}, "
                f"found {lease['generation']}"
            )
        if args.epoch_hash is not None:
            expected_hash = require_bytes(args.epoch_hash, 32, "--epoch-hash")
            actual_hash = require_bytes(lease["epoch_hash"], 32, "epoch_hash")
            if actual_hash != expected_hash:
                fail("epoch_hash mismatch")
        if args.scope is not None and args.scope not in lease["allowed_scopes"]:
            fail(f"lease does not authorize scope {args.scope!r}")

        print("authority write lease: VALID")
        print(f"  signer_public_key={canonical.public_key.hex()}")
        print(f"  deployment_id={lease['deployment_id']}")
        print(f"  primary_id={lease['primary_id']}")
        print(f"  generation={lease['generation']}")
        print(f"  phase={lease['phase']}")
        print(f"  expires_at={lease['expires_at']}")
        print(f"  lease_canonical_sha256={canonical.lease_hash_sha256}")
        print(f"  signing_bytes_sha256={canonical.signing_bytes_sha256}")
        return 0
    except (ValueError, OSError) as error:
        print(f"authority write lease: INVALID: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
