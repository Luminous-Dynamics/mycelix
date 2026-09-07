#!/usr/bin/env python3
"""Verify resolved Cargo graphs contain only the qualified Holochain cohort.

Top-level manifest equality is necessary but insufficient: Cargo can resolve multiple
versions of the same protocol family and still compile. This checker runs metadata for
each migration surface and rejects mixed protocol generations while treating upstream
crates that version independently (for example holochain_chc) explicitly rather than
assuming every `holochain_*` package shares the main Holochain version.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"

SURFACES = {
    "pulse-active-zomes": WORKSPACE / "mycelix-pulse/holochain/Cargo.toml",
    "pulse-sweetconductor": WORKSPACE / "mycelix-pulse/tests/Cargo.toml",
    "pulse-happ-integrity": WORKSPACE / "mycelix-pulse/happ/dna/integrity/Cargo.toml",
    "pulse-backend": WORKSPACE / "mycelix-pulse/happ/backend-rs/Cargo.toml",
    "pulse-cli": WORKSPACE / "mycelix-pulse/happ/cli/Cargo.toml",
    "pulse-simple-integrity": WORKSPACE / "mycelix-pulse/happ/dna/dna/integrity/Cargo.toml",
    "pulse-simple-messages": WORKSPACE / "mycelix-pulse/happ/dna/dna/zomes/mail_messages/Cargo.toml",
    "pulse-simple-trust": WORKSPACE / "mycelix-pulse/happ/dna/dna/zomes/trust_filter/Cargo.toml",
}

# These crates are released on the main Holochain 0.6.x version line. Keep this
# list closed-world: encountering a new `holochain_*` package is a qualification
# failure until its release/version relationship is explicitly classified.
HOLOCHAIN_RELEASE_COUPLED = {
    "holochain",
    "holochain_cascade",
    "holochain_conductor_api",
    "holochain_conductor_config",
    "holochain_integrity_types",
    "holochain_keystore",
    "holochain_metrics",
    "holochain_nonce",
    "holochain_p2p",
    "holochain_secure_primitive",
    "holochain_sqlite",
    "holochain_state",
    "holochain_state_types",
    "holochain_timestamp",
    "holochain_trace",
    "holochain_types",
    "holochain_util",
    "holochain_websocket",
    "holochain_zome_types",
}


class UnclassifiedFamilyPackage(ValueError):
    pass


def load_contract() -> dict:
    return tomllib.loads((WORKSPACE / "holochain-cohort.toml").read_text())


def expected_version(name: str, contract: dict) -> str | None:
    rust = contract["rust"]
    target = contract["next_0_6"]

    if name == "hdi":
        return rust["hdi"]
    if name in {"hdk", "hdk_derive"}:
        return rust["hdk"]
    if name == "holochain_client":
        return rust["holochain_client"]
    if name == "holo_hash":
        return rust["holo_hash"]
    if name in {"holochain_serialized_bytes", "holochain_serialized_bytes_derive"}:
        return rust["holochain_serialized_bytes"]
    if name.startswith("holochain_wasmer_"):
        return rust["holochain_wasmer_host"]
    if name == "holochain_chc":
        return target["holochain_chc"]
    if name in HOLOCHAIN_RELEASE_COUPLED:
        return rust["holochain"]
    if name.startswith("holochain_"):
        raise UnclassifiedFamilyPackage(
            f"unclassified Holochain-family package {name!r}; classify its upstream version line explicitly"
        )
    if name == "kitsune2" or name.startswith("kitsune2_"):
        return rust["kitsune2"]
    if name == "lair_keystore" or name.startswith("lair_keystore_"):
        return rust["lair_keystore"]
    return None


def cargo_metadata(manifest: Path) -> dict:
    proc = subprocess.run(
        [
            "cargo",
            "metadata",
            "--format-version",
            "1",
            "--manifest-path",
            str(manifest),
        ],
        cwd=ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"cargo metadata failed for {manifest.relative_to(ROOT)}\n{proc.stderr}"
        )
    return json.loads(proc.stdout)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    contract = load_contract()
    if contract.get("state") != "aligned":
        raise SystemExit(
            "resolved-graph qualification is only valid after the candidate contract is aligned"
        )
    if not contract["policy"].get("require_rust_nix_alignment"):
        raise SystemExit("aligned candidate must require Rust/Nix alignment")

    failures: list[str] = []
    evidence: dict[str, dict[str, list[str]]] = {}

    for surface, manifest in SURFACES.items():
        if not manifest.is_file():
            failures.append(f"{surface}: missing manifest {manifest.relative_to(ROOT)}")
            continue

        try:
            metadata = cargo_metadata(manifest)
        except (RuntimeError, json.JSONDecodeError) as exc:
            failures.append(f"{surface}: {exc}")
            continue

        observed: dict[str, set[str]] = {}
        for package in metadata.get("packages", []):
            name = package["name"]
            try:
                expected = expected_version(name, contract)
            except UnclassifiedFamilyPackage as exc:
                failures.append(f"{surface}:{exc}")
                continue
            if expected is None:
                continue
            version = package["version"]
            observed.setdefault(name, set()).add(version)
            if version != expected:
                failures.append(
                    f"{surface}:{name}: resolved {version}, expected cohort {expected}"
                )

        if not observed:
            failures.append(f"{surface}: resolved graph contained no tracked Holochain-family package")

        evidence[surface] = {
            name: sorted(versions) for name, versions in sorted(observed.items())
        }

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")

    for surface, packages in evidence.items():
        summary = ", ".join(
            f"{name}={'/'.join(versions)}" for name, versions in packages.items()
        )
        print(f"{surface}: {summary}")

    if failures:
        raise SystemExit("Resolved Holochain graph qualification failed:\n- " + "\n- ".join(failures))

    print(f"Resolved Holochain cohort graphs OK across {len(evidence)} Pulse surfaces.")


if __name__ == "__main__":
    main()
