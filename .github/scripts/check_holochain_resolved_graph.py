#!/usr/bin/env python3
"""Verify resolved Cargo graphs contain only the qualified Holochain cohort.

Top-level manifest equality is necessary but insufficient: Cargo can resolve multiple
versions of the same protocol family and still compile. This checker resolves every
migration surface, records the concrete Cargo workspace/lockfile that produced that
graph, and rejects mixed protocol generations using the shared fail-closed release
family classifier.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tomllib

from holochain_release_family import UnclassifiedFamilyPackage, expected_family_version

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


def load_contract() -> dict:
    return tomllib.loads((WORKSPACE / "holochain-cohort.toml").read_text())


def relative_repo_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(ROOT.resolve()))
    except ValueError as exc:
        raise RuntimeError(f"resolved Cargo path escaped repository: {resolved}") from exc


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
    surfaces_evidence: dict[str, dict] = {}

    for surface, manifest in SURFACES.items():
        if not manifest.is_file():
            failures.append(f"{surface}: missing manifest {manifest.relative_to(ROOT)}")
            continue

        try:
            metadata = cargo_metadata(manifest)
            workspace_root = Path(metadata["workspace_root"])
            lockfile = workspace_root / "Cargo.lock"
            workspace_root_rel = relative_repo_path(workspace_root)
            lockfile_rel = relative_repo_path(lockfile)
        except (RuntimeError, KeyError, json.JSONDecodeError) as exc:
            failures.append(f"{surface}: {exc}")
            continue

        if not lockfile.is_file() or lockfile.stat().st_size == 0:
            failures.append(
                f"{surface}: resolved graph has no concrete non-empty lockfile at {lockfile_rel}"
            )

        observed: dict[str, set[str]] = {}
        for package in metadata.get("packages", []):
            name = package["name"]
            try:
                expected = expected_family_version(name, contract)
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

        surfaces_evidence[surface] = {
            "manifest": str(manifest.relative_to(ROOT)),
            "workspace_root": workspace_root_rel,
            "lockfile": lockfile_rel,
            "tracked_packages": {
                name: sorted(versions) for name, versions in sorted(observed.items())
            },
        }

    unique_locks = sorted(
        {item["lockfile"] for item in surfaces_evidence.values() if item.get("lockfile")}
    )
    evidence = {
        "schema": 2,
        "surface_count": len(surfaces_evidence),
        "unique_lockfile_count": len(unique_locks),
        "lockfiles": unique_locks,
        "surfaces": surfaces_evidence,
    }

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")

    for surface, item in surfaces_evidence.items():
        summary = ", ".join(
            f"{name}={'/'.join(versions)}"
            for name, versions in item["tracked_packages"].items()
        )
        print(f"{surface}: lock={item['lockfile']}; {summary}")

    if failures:
        raise SystemExit("Resolved Holochain graph qualification failed:\n- " + "\n- ".join(failures))

    print(
        f"Resolved Holochain cohort graphs OK across {len(surfaces_evidence)} Pulse surfaces "
        f"using {len(unique_locks)} concrete Cargo lockfiles."
    )


if __name__ == "__main__":
    main()
