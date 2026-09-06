#!/usr/bin/env python3
"""Fail closed when Mycelix Holochain compatibility surfaces drift unexpectedly."""

from __future__ import annotations

from pathlib import Path
import re
import tomllib

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT / "mycelix-workspace"


def load_toml(path: Path) -> dict:
    return tomllib.loads(path.read_text())


def dep_version(value) -> str | None:
    if isinstance(value, str):
        return value.lstrip("=")
    if isinstance(value, dict):
        version = value.get("version")
        return version.lstrip("=") if isinstance(version, str) else None
    return None


def ref_version(ref: str, prefix: str = "") -> str:
    if prefix and not ref.startswith(prefix):
        raise ValueError(f"expected ref {ref!r} to start with {prefix!r}")
    return ref[len(prefix) :] if prefix else ref


def require_equal(failures: list[str], label: str, got, want) -> None:
    if got != want:
        failures.append(f"{label}: observed {got!r}, expected {want!r}")


def verify_manifest_binding(failures: list[str], kind: str, binding: dict) -> None:
    path = ROOT / binding["path"]
    manifest = load_toml(path)
    deps = manifest.get("dependencies", {})
    for name, want in binding.items():
        if name in {"path", "reason"}:
            continue
        require_equal(
            failures,
            f"{kind}:{binding['path']}:{name}",
            dep_version(deps.get(name)),
            want,
        )


def main() -> None:
    contract = load_toml(WORKSPACE / "holochain-cohort.toml")
    root = load_toml(WORKSPACE / "Cargo.toml")
    flake_lock = load_toml(WORKSPACE / "flake.lock")
    pulse = load_toml(WORKSPACE / "mycelix-pulse/holochain/Cargo.toml")
    pulse_tests = load_toml(WORKSPACE / "mycelix-pulse/tests/Cargo.toml")

    failures: list[str] = []
    rust = contract["rust"]
    canonical = root["workspace"]["dependencies"]

    for name, want in rust.items():
        if name not in canonical:
            failures.append(f"workspace dependency missing canonical Holochain member {name!r}")
            continue
        require_equal(failures, f"workspace.{name}", dep_version(canonical[name]), want)

    pulse_deps = pulse["workspace"]["dependencies"]
    for name in (
        "hdk",
        "hdi",
        "holochain_integrity_types",
        "holochain_zome_types",
        "holo_hash",
    ):
        require_equal(
            failures,
            f"pulse.holochain.{name}",
            dep_version(pulse_deps.get(name)),
            rust[name],
        )

    test_deps = pulse_tests["dev-dependencies"]
    for name in ("holochain", "holochain_types", "holo_hash", "hdi"):
        require_equal(
            failures,
            f"pulse.tests.{name}",
            dep_version(test_deps.get(name)),
            rust[name],
        )

    nodes = flake_lock["nodes"]
    nix = contract["nix"]
    require_equal(failures, "flake.holonix.rev", nodes["holonix"]["locked"]["rev"], nix["holonix_rev"])
    require_equal(failures, "flake.holochain.ref", nodes["holochain"]["original"]["ref"], nix["holochain_ref"])
    require_equal(failures, "flake.kitsune2.ref", nodes["kitsune2"]["original"]["ref"], nix["kitsune2_ref"])
    require_equal(failures, "flake.lair.ref", nodes["lair-keystore"]["original"]["ref"], nix["lair_keystore_ref"])

    rust_holochain = rust["holochain"]
    nix_holochain = ref_version(nix["holochain_ref"], "holochain-")
    rust_kitsune = rust["kitsune2"]
    nix_kitsune = ref_version(nix["kitsune2_ref"], "v")
    rust_lair = rust["lair_keystore"]
    nix_lair = ref_version(nix["lair_keystore_ref"], "v")
    aligned = (
        rust_holochain == nix_holochain
        and rust_kitsune == nix_kitsune
        and rust_lair == nix_lair
    )

    require_alignment = contract["policy"]["require_rust_nix_alignment"]
    state = contract["state"]
    if require_alignment and not aligned:
        failures.append("policy requires Rust/Nix Holochain alignment but the resolved cohorts differ")
    if state == "quarantined-drift" and aligned:
        failures.append("contract still says quarantined-drift even though Rust/Nix cohorts are aligned")
    if state == "aligned" and not aligned:
        failures.append("contract says aligned but Rust/Nix cohorts differ")
    if not require_alignment and state != "quarantined-drift":
        failures.append("disabling alignment is only permitted under explicit quarantined-drift state")

    quarantines = contract.get("quarantine", [])
    aligned_surfaces = contract.get("aligned_surface", [])
    if state == "aligned" and quarantines:
        failures.append("aligned state cannot retain Holochain compatibility quarantines")
    if state == "quarantined-drift" and aligned_surfaces:
        failures.append("baseline quarantined-drift state cannot claim migrated aligned surfaces")

    for binding in quarantines:
        verify_manifest_binding(failures, "quarantine", binding)
    for binding in aligned_surfaces:
        verify_manifest_binding(failures, "aligned", binding)

    next_06 = contract["next_0_6"]
    if not re.fullmatch(r"[0-9a-f]{40}", next_06["holonix_rev"]):
        failures.append("next_0_6.holonix_rev must be an exact 40-hex commit")
    for key in ("holochain", "hdk", "hdi", "holochain_client", "kitsune2", "lair_keystore"):
        if not re.fullmatch(r"\d+\.\d+\.\d+", next_06[key]):
            failures.append(f"next_0_6.{key} must be an exact semantic version")

    if failures:
        raise SystemExit("Holochain cohort contract failed:\n- " + "\n- ".join(failures))

    print(f"Holochain cohort contract OK: state={state}")
    print(
        "Rust/Nix alignment: "
        + ("aligned" if aligned else "QUARANTINED DRIFT")
        + f" (Rust Holochain {rust_holochain}, Nix Holochain {nix_holochain})"
    )
    print(f"Explicit quarantines: {len(quarantines)}")
    print(f"Explicit aligned surfaces: {len(aligned_surfaces)}")
    print(
        "Next coherent 0.6 cohort: "
        f"Holochain {next_06['holochain']}, HDK {next_06['hdk']}, HDI {next_06['hdi']}, "
        f"client {next_06['holochain_client']}"
    )


if __name__ == "__main__":
    main()
