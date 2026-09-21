#!/usr/bin/env python3
"""Independent FIN-MKT-001 V1 canonical-vector oracle.

Uses only Python stdlib and the frozen JSON fixture. It does not import or invoke
Rust production code.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct

DOMAIN = b"MYCELIX_FIN_MKT_ORDER_INTENT_V1\0"
REVISION = 1
EXPECTED_LEN = 671
EXPECTED_SHA256 = "37d7854bb7012da68bf12cbd76f9a8ca46d3e8364140a6d275168da2f0721f71"


def text(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack(">I", len(encoded)) + encoded


def digest32(value: str) -> bytes:
    if len(value) != 64 or value != value.lower():
        raise ValueError("digest must be 64 lowercase hex characters")
    raw = bytes.fromhex(value)
    if len(raw) != 32:
        raise ValueError("digest must be 32 bytes")
    return raw


def profile(value: dict) -> bytes:
    return (
        text(value["profile_id"])
        + struct.pack(">I", value["revision"])
        + digest32(value["digest"])
    )


def subject(value: dict) -> bytes:
    return profile(value["subject_profile"]) + text(value["subject_id"])


def instrument(value: dict) -> bytes:
    return profile(value["instrument_profile"]) + text(value["instrument_id"])


def amount(value: dict) -> bytes:
    atomic = value["atomic_units"]
    if atomic < 0 or atomic > 2**64 - 1:
        raise ValueError("atomic amount outside u64")
    return struct.pack(">Q", atomic) + text(value["asset"])


def quantity(value: dict) -> bytes:
    if list(value) == ["Units"]:
        tag = b"\x00"
        body = value["Units"]
    elif list(value) == ["Notional"]:
        tag = b"\x01"
        body = value["Notional"]
    else:
        raise ValueError("unknown quantity class")
    if body["amount"]["atomic_units"] == 0:
        raise ValueError("zero quantity")
    return tag + profile(body["unit_profile"]) + amount(body["amount"])


def market_price(value: dict) -> bytes:
    if value["quote_amount"]["atomic_units"] == 0:
        raise ValueError("zero price")
    return profile(value["pricing_profile"]) + amount(value["quote_amount"])


def order_terms(value: dict) -> bytes:
    keys = list(value)
    if keys == ["Market"]:
        return b"\x00"
    if keys == ["Limit"]:
        return b"\x01" + market_price(value["Limit"]["price"])
    if keys == ["Stop"]:
        return b"\x02" + market_price(value["Stop"]["price"])
    if keys == ["StopLimit"]:
        body = value["StopLimit"]
        return (
            b"\x03"
            + market_price(body["stop_price"])
            + market_price(body["limit_price"])
        )
    raise ValueError("unknown order terms")


def build(fixture: dict) -> bytes:
    side_tags = {"AcquireLong": 0, "ReduceLong": 1}
    tif_tags = {
        "Day": 0,
        "GoodTilCanceled": 1,
        "ImmediateOrCancel": 2,
        "FillOrKill": 3,
    }

    output = bytearray(DOMAIN)
    output += struct.pack(">I", REVISION)
    output += subject(fixture["intent_subject"])
    output += subject(fixture["account_subject"])
    output += instrument(fixture["instrument"])
    output.append(side_tags[fixture["side"]])
    output += quantity(fixture["quantity"])
    output += order_terms(fixture["order_terms"])
    output.append(tif_tags[fixture["time_in_force"]])
    output += profile(fixture["execution_profile"])
    idem = fixture["semantic_idempotency"]
    output += profile(idem["idempotency_profile"])
    output += text(idem["semantic_id"])

    upstream = fixture["upstream_economic_effect_commitment"]
    if upstream is None:
        output.append(0)
    else:
        output.append(1)
        output += digest32(upstream)

    return bytes(output)


def main() -> None:
    fixture_path = Path(__file__).parents[2] / "test-vectors" / "order-intent-v1.json"
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    canonical = build(fixture)
    actual_hash = hashlib.sha256(canonical).hexdigest()

    assert len(canonical) == EXPECTED_LEN, (len(canonical), EXPECTED_LEN)
    assert actual_hash == EXPECTED_SHA256, (actual_hash, EXPECTED_SHA256)
    print(f"FIN-MKT-001 oracle PASS: bytes={len(canonical)} sha256={actual_hash}")


if __name__ == "__main__":
    main()
