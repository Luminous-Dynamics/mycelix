#!/usr/bin/env python3
"""Independent Python stdlib conformance for Mycelix accounting Merkle v2."""

from __future__ import annotations

import hashlib
from typing import Any

from accounting_wire_conformance import serialize_wire

LEAF_DOMAIN = b"mycelix-accounting-merkle-leaf-v2\0"
NODE_DOMAIN = b"mycelix-accounting-merkle-node-v2\0"
EMPTY_DOMAIN = b"mycelix-accounting-merkle-empty-v2\0"


def leaf_hash(value: Any) -> bytes:
    return hashlib.sha256(LEAF_DOMAIN + serialize_wire(value).encode("utf-8")).digest()


def node_hash(left: bytes, right: bytes) -> bytes:
    if len(left) != 32 or len(right) != 32:
        raise ValueError("Merkle v2 child hashes must be 32 bytes")
    return hashlib.sha256(NODE_DOMAIN + left + right).digest()


def merkle_root(values: list[Any] | tuple[Any, ...]) -> str:
    if not values:
        return hashlib.sha256(EMPTY_DOMAIN).hexdigest()
    level = [leaf_hash(value) for value in values]
    while len(level) > 1:
        level = [
            node_hash(level[index], level[index + 1] if index + 1 < len(level) else level[index])
            for index in range(0, len(level), 2)
        ]
    return level[0].hex()


def build_proof(values: list[Any] | tuple[Any, ...], index: int) -> tuple[tuple[str, str], ...]:
    if index < 0 or index >= len(values):
        raise ValueError("Merkle v2 proof index out of range")
    level = [leaf_hash(value) for value in values]
    cursor = index
    steps: list[tuple[str, str]] = []
    while len(level) > 1:
        is_right = cursor % 2 == 1
        sibling_index = cursor - 1 if is_right else min(cursor + 1, len(level) - 1)
        steps.append(("left" if is_right else "right", level[sibling_index].hex()))
        cursor //= 2
        level = [
            node_hash(level[position], level[position + 1] if position + 1 < len(level) else level[position])
            for position in range(0, len(level), 2)
        ]
    return tuple(steps)


def verify_proof(value: Any, index: int, count: int, steps: tuple[tuple[str, str], ...], expected_root: str) -> bool:
    if count <= 0 or index < 0 or index >= count or len(expected_root) != 64:
        return False
    current = leaf_hash(value)
    cursor = index
    width = count
    for side, sibling_hex in steps:
        try:
            sibling = bytes.fromhex(sibling_hex)
        except ValueError:
            return False
        if len(sibling) != 32:
            return False
        expected_side = "left" if cursor % 2 == 1 else "right"
        if side != expected_side:
            return False
        if cursor % 2 == 0 and cursor + 1 >= width and sibling != current:
            return False
        current = node_hash(sibling, current) if side == "left" else node_hash(current, sibling)
        cursor //= 2
        width = (width + 1) // 2
    return current.hex() == expected_root


GOLDEN_ROOTS: tuple[tuple[str, tuple[Any, ...], str], ...] = (
    (
        "empty",
        (),
        "c3f59a327d432c5ca62e0c83780a2f7b4979d2d67cec42949f69ca0312ba5d5e",
    ),
    (
        "null leaf",
        (None,),
        "34e2e4c226ea9bc64493fc48384c60a761a90656e5d4f2c790ed9ed11ef9367e",
    ),
    (
        "arbitrary precision integer leaf",
        (123456789012345678901234567890,),
        "9de130683cdd49dd1b8ade66ffa1aa030274eab646607732e72327ca5871f8ca",
    ),
    (
        "two leaves with Unicode and negative zero",
        ({"z": 3, "a": "é"}, -0.0),
        "327a078e7127c79e8c66099c1b3a7e5f367d07ca18524f1fb06e19cb0005cd5d",
    ),
    (
        "odd-width tree with UTF-8 key-order vector",
        (None, 1.5, {"\uE000": "bmp", "𐀀": "astral"}),
        "1ae6b649185c96953b157f9226fb286a45056f03e7f682c2fa268aa958f63531",
    ),
)


def run_conformance() -> None:
    for name, values, expected_root in GOLDEN_ROOTS:
        assert merkle_root(values) == expected_root, f"{name}: Merkle v2 root mismatch"

    odd_values = GOLDEN_ROOTS[-1][1]
    odd_root = GOLDEN_ROOTS[-1][2]
    for index, value in enumerate(odd_values):
        proof = build_proof(odd_values, index)
        assert verify_proof(value, index, len(odd_values), proof, odd_root), f"proof failed at index {index}"

    proof = build_proof(odd_values, 1)
    assert not verify_proof("tampered", 1, len(odd_values), proof, odd_root)
    first_side, first_hash = proof[0]
    wrong_side = "right" if first_side == "left" else "left"
    assert not verify_proof(odd_values[1], 1, len(odd_values), ((wrong_side, first_hash), *proof[1:]), odd_root)
    assert not verify_proof(odd_values[1], 1, len(odd_values), proof, "f" * 64)


def main() -> int:
    run_conformance()
    print("accounting Merkle v2 Python stdlib conformance: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
