#!/usr/bin/env python3
"""Materialize, but do not commit, the exact Mycelix/Pulse Holochain 0.6.3 candidate.

This script is intentionally narrow: canonical workspace pins + every currently known
Pulse Holochain island. It is run only by the qualification workflow; product source
is materialized later from a qualified patch.
"""

from __future__ import annotations

from pathlib import Path
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"
CONTRACT = WORKSPACE / "holochain-cohort.toml"


def replace_exact(path: Path, old: str, new: str, *, count: int | None = None) -> int:
    text = path.read_text()
    observed = text.count(old)
    if observed == 0:
        raise SystemExit(f"expected text not found in {path}: {old!r}")
    if count is not None and observed != count:
        raise SystemExit(
            f"unexpected replacement cardinality in {path}: {old!r}: {observed} != {count}"
        )
    path.write_text(text.replace(old, new))
    return observed


def set_manifest_versions(path: Path, replacements: list[tuple[str, str]]) -> None:
    for old, new in replacements:
        replace_exact(path, old, new, count=1)


def main() -> None:
    contract = tomllib.loads(CONTRACT.read_text())
    target = contract["next_0_6"]

    if target != {
        "holonix_rev": "21d3a9f3eb1d7a533fc816ab53825f3ef35b0007",
        "holochain": "0.6.3",
        "hdk": "0.6.3",
        "hdi": "0.7.3",
        "holochain_client": "0.8.3",
        "kitsune2": "0.4.1",
        "lair_keystore": "0.6.3",
    }:
        raise SystemExit(f"next_0_6 contract changed unexpectedly: {target!r}")

    # Canonical workspace cohort.
    set_manifest_versions(
        WORKSPACE / "Cargo.toml",
        [
            ('hdk = "=0.6.1"', 'hdk = "=0.6.3"'),
            ('hdi = "=0.7.1"', 'hdi = "=0.7.3"'),
            ('holochain = "=0.6.1"', 'holochain = "=0.6.3"'),
            ('holochain_client = "=0.8.1"', 'holochain_client = "=0.8.3"'),
            ('holochain_types = "=0.6.1"', 'holochain_types = "=0.6.3"'),
            ('holochain_zome_types = "=0.6.1"', 'holochain_zome_types = "=0.6.3"'),
            ('holo_hash = "=0.6.1"', 'holo_hash = "=0.6.3"'),
            ('holochain_integrity_types = "=0.6.1"', 'holochain_integrity_types = "=0.6.3"'),
            ('holochain_state = "=0.6.1"', 'holochain_state = "=0.6.3"'),
            ('holochain_p2p = "=0.6.1"', 'holochain_p2p = "=0.6.3"'),
            ('holochain_keystore = "=0.6.1"', 'holochain_keystore = "=0.6.3"'),
            ('holochain_sqlite = "=0.6.1"', 'holochain_sqlite = "=0.6.3"'),
        ],
    )

    # Active Pulse zome workspace.
    set_manifest_versions(
        WORKSPACE / "mycelix-pulse/holochain/Cargo.toml",
        [
            ('hdk = "=0.6.1"', 'hdk = "=0.6.3"'),
            ('hdi = "=0.7.1"', 'hdi = "=0.7.3"'),
            ('holochain_integrity_types = "=0.6.1"', 'holochain_integrity_types = "=0.6.3"'),
            ('holochain_zome_types = "=0.6.1"', 'holochain_zome_types = "=0.6.3"'),
            ('holo_hash = "=0.6.1"', 'holo_hash = "=0.6.3"'),
            ('hdk_derive = "=0.6.1"', 'hdk_derive = "=0.6.3"'),
        ],
    )

    # Pulse SweetConductor harness.
    set_manifest_versions(
        WORKSPACE / "mycelix-pulse/tests/Cargo.toml",
        [
            ('version = "=0.6.1", features = ["sweettest"]', 'version = "=0.6.3", features = ["sweettest"]'),
            ('holochain_types = { version = "=0.6.1" }', 'holochain_types = { version = "=0.6.3" }'),
            ('holo_hash = { version = "=0.6.1" }', 'holo_hash = { version = "=0.6.3" }'),
            ('hdi = "=0.7.1"', 'hdi = "=0.7.3"'),
        ],
    )

    # Standalone hApp integrity crate.
    set_manifest_versions(
        WORKSPACE / "mycelix-pulse/happ/dna/integrity/Cargo.toml",
        [
            ('hdi = "=0.7.1"', 'hdi = "=0.7.3"'),
            ('holochain_integrity_types = "=0.6.1"', 'holochain_integrity_types = "=0.6.3"'),
            ('holochain_zome_types = "=0.6.1"', 'holochain_zome_types = "=0.6.3"'),
            ('holo_hash = "=0.6.1"', 'holo_hash = "=0.6.3"'),
            ('hdk_derive = "=0.6.1"', 'hdk_derive = "=0.6.3"'),
        ],
    )

    # Legacy backend/CLI API islands: force them into the same 0.6.3 API cohort
    # so compilation, rather than version coexistence, decides whether migration is valid.
    set_manifest_versions(
        WORKSPACE / "mycelix-pulse/happ/backend-rs/Cargo.toml",
        [
            ('holochain_client = "=0.8.1"', 'holochain_client = "=0.8.3"'),
            ('holochain_types = "0.5"', 'holochain_types = "=0.6.3"'),
            ('holochain_zome_types = "0.5"', 'holochain_zome_types = "=0.6.3"'),
            ('holochain_keystore = "0.5"', 'holochain_keystore = "=0.6.3"'),
        ],
    )
    set_manifest_versions(
        WORKSPACE / "mycelix-pulse/happ/cli/Cargo.toml",
        [
            ('holochain_client = "=0.8.1"', 'holochain_client = "=0.8.3"'),
            ('holochain_types = "0.5"', 'holochain_types = "=0.6.3"'),
            ('holochain_conductor_api = "0.5"', 'holochain_conductor_api = "=0.6.3"'),
        ],
    )

    # Legacy simplified DNA tree.
    replace_exact(
        WORKSPACE / "mycelix-pulse/happ/dna/dna/integrity/Cargo.toml",
        'hdi = "0.7.0"',
        'hdi = "=0.7.3"',
        count=1,
    )
    for rel in (
        "mycelix-pulse/happ/dna/dna/zomes/mail_messages/Cargo.toml",
        "mycelix-pulse/happ/dna/dna/zomes/trust_filter/Cargo.toml",
    ):
        replace_exact(WORKSPACE / rel, 'hdk = "0.6.0"', 'hdk = "=0.6.3"', count=1)

    # Move Nix tooling to the exact upstream Holonix commit whose lock binds
    # Holochain 0.6.3 / Kitsune2 0.4.1 / Lair 0.6.3. Nix itself regenerates
    # flake.lock in the workflow; this script never fabricates a lockfile.
    replace_exact(
        WORKSPACE / "flake.nix",
        'url = "github:holochain/holonix/d21b3543";',
        'url = "github:holochain/holonix/21d3a9f3eb1d7a533fc816ab53825f3ef35b0007";',
        count=1,
    )

    # Rewrite the compatibility contract to the candidate's intended state.
    text = CONTRACT.read_text()
    replacements = [
        ('state = "quarantined-drift"', 'state = "aligned"'),
        ('holochain = "0.6.1"', 'holochain = "0.6.3"'),
        ('hdk = "0.6.1"', 'hdk = "0.6.3"'),
        ('hdi = "0.7.1"', 'hdi = "0.7.3"'),
        ('holochain_client = "0.8.1"', 'holochain_client = "0.8.3"'),
        ('holochain_types = "0.6.1"', 'holochain_types = "0.6.3"'),
        ('holochain_zome_types = "0.6.1"', 'holochain_zome_types = "0.6.3"'),
        ('holochain_integrity_types = "0.6.1"', 'holochain_integrity_types = "0.6.3"'),
        ('holochain_state = "0.6.1"', 'holochain_state = "0.6.3"'),
        ('holochain_p2p = "0.6.1"', 'holochain_p2p = "0.6.3"'),
        ('holochain_keystore = "0.6.1"', 'holochain_keystore = "0.6.3"'),
        ('holochain_sqlite = "0.6.1"', 'holochain_sqlite = "0.6.3"'),
        ('holo_hash = "0.6.1"', 'holo_hash = "0.6.3"'),
        ('holonix_rev = "d21b35431e425e615bc05da790987380a84b8280"', f'holonix_rev = "{target["holonix_rev"]}"'),
        ('holochain_ref = "holochain-0.6.0"', 'holochain_ref = "holochain-0.6.3"'),
        ('kitsune2_ref = "v0.3.2"', 'kitsune2_ref = "v0.4.1"'),
        ('require_rust_nix_alignment = false', 'require_rust_nix_alignment = true'),
    ]
    for old, new in replacements:
        if old not in text:
            raise SystemExit(f"cohort contract expected text not found: {old!r}")
        text = text.replace(old, new)

    text = text.replace(
        'quarantine_reason = "Existing baseline drift: Cargo resolves Holochain 0.6.1/Kitsune2 0.4.1 while Holonix resolves Holochain 0.6.0/Kitsune2 0.3.2. The 0.6.3 migration tranche must remove this quarantine rather than silently preserving it."',
        'alignment_reason = "Canonical Rust, Pulse, and Holonix surfaces are bound to the coherent upstream 0.6.3 cohort."',
    )

    marker = "# Known legacy islands."
    if marker not in text:
        raise SystemExit("cohort quarantine marker missing")
    text = text.split(marker, 1)[0].rstrip()
    text += """

# Former compatibility islands remain explicitly bound after migration. They do
# not disappear from the theorem merely because they joined the canonical cohort.
[[aligned_surface]]
path = "mycelix-workspace/mycelix-pulse/happ/dna/integrity/Cargo.toml"
reason = "Standalone hApp integrity crate migrated to the 0.6.3 cohort."
hdi = "0.7.3"
holochain_integrity_types = "0.6.3"
holochain_zome_types = "0.6.3"
holo_hash = "0.6.3"
hdk_derive = "0.6.3"

[[aligned_surface]]
path = "mycelix-workspace/mycelix-pulse/happ/backend-rs/Cargo.toml"
reason = "Pulse backend API surface migrated to the 0.6.3 cohort."
holochain_client = "0.8.3"
holochain_types = "0.6.3"
holochain_zome_types = "0.6.3"
holochain_keystore = "0.6.3"

[[aligned_surface]]
path = "mycelix-workspace/mycelix-pulse/happ/cli/Cargo.toml"
reason = "Pulse CLI API surface migrated to the 0.6.3 cohort."
holochain_client = "0.8.3"
holochain_types = "0.6.3"
holochain_conductor_api = "0.6.3"

[[aligned_surface]]
path = "mycelix-workspace/mycelix-pulse/happ/dna/dna/integrity/Cargo.toml"
reason = "Simplified legacy DNA integrity surface migrated to HDI 0.7.3."
hdi = "0.7.3"

[[aligned_surface]]
path = "mycelix-workspace/mycelix-pulse/happ/dna/dna/zomes/mail_messages/Cargo.toml"
reason = "Simplified legacy DNA mail_messages coordinator migrated to HDK 0.6.3."
hdk = "0.6.3"

[[aligned_surface]]
path = "mycelix-workspace/mycelix-pulse/happ/dna/dna/zomes/trust_filter/Cargo.toml"
reason = "Simplified legacy DNA trust_filter coordinator migrated to HDK 0.6.3."
hdk = "0.6.3"
"""
    CONTRACT.write_text(text)

    print("Materialized Holochain 0.6.3 Pulse candidate with explicit aligned-surface bindings.")


if __name__ == "__main__":
    main()
