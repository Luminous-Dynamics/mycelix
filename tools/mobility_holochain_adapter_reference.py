#!/usr/bin/env python3
"""Independent contract checks for the concrete mobility Holochain adapter."""

from __future__ import annotations

from pathlib import Path

ADAPTER_DIR = (
    Path(__file__).resolve().parent.parent
    / "mycelix-commons/crates/mobility-qualification-holochain-adapter"
)
LIB = ADAPTER_DIR / "src/lib.rs"
CARGO = ADAPTER_DIR / "Cargo.toml"

EXPECTED_DISPATCH = (
    ("ValidRecord", "ActionHash", "must_get_valid_record"),
    ("Action", "ActionHash", "must_get_action"),
    ("Entry", "EntryHash", "must_get_entry"),
)


def main() -> int:
    cargo = CARGO.read_text(encoding="utf-8")
    source = LIB.read_text(encoding="utf-8")

    for fragment in (
        'hdi = "=0.8.0"',
        'mobility-configuration-qualification = { path = "../mobility-configuration-qualification" }',
        "[workspace]",
    ):
        if fragment not in cargo:
            raise SystemExit(f"adapter manifest missing required fragment: {fragment}")

    forbidden = (
        "IdentityRef.id",
        "hash_entry(",
        "hash_action(",
        "blake2",
        "get_links(",
        "get_links_details(",
        "SystemTime",
        "Instant",
    )
    for fragment in forbidden:
        if fragment in source:
            raise SystemExit(
                f"adapter contains forbidden semantic/address derivation fragment: {fragment}"
            )

    for retrieval, address, host_fn in EXPECTED_DISPATCH:
        for fragment in (retrieval, address, host_fn):
            if fragment not in source:
                raise SystemExit(f"adapter missing required dispatch fragment: {fragment}")

    if "QualificationDecision::Valid" not in source:
        raise SystemExit("adapter must preserve the pure valid decision")
    if "QualificationDecision::Invalid" not in source:
        raise SystemExit("adapter must preserve the pure invalid decision")
    if "QualificationDecision::Unresolved" not in source:
        raise SystemExit("adapter must preserve the pure unresolved decision")
    if (
        'unreachable!("HolochainDependencyBindingSet prevents address-kind mismatch")'
        not in source
    ):
        raise SystemExit("host retrieval must be unreachable after binding-time type validation")

    print("mobility Holochain adapter independent reference qualification: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
