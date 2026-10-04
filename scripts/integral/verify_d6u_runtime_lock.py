#!/usr/bin/env python3
"""Verify the resolved D6U substrate packages in the generated Cargo.lock."""

import json
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_d6u_runtime_lock.py CARGO_LOCK")

    lock_path = Path(sys.argv[1])
    lock = tomllib.loads(lock_path.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    package_versions = {
        "holochain": manifest["substrate"]["holochain"],
        "hdk": manifest["substrate"]["hdk"],
        "hdi": manifest["substrate"]["hdi"],
        "holochain_keystore": manifest["substrate"]["holochain"],
        "holochain_nonce": manifest["substrate"]["holochain"],
        "holochain_serialized_bytes": manifest["substrate"]["holochain_serialized_bytes"],
        "holochain_types": manifest["substrate"]["holochain"],
    }

    packages = lock.get("package", [])
    for package_name, expected_version in package_versions.items():
        matches = [package for package in packages if package.get("name") == package_name]
        assert matches, f"Cargo.lock must contain {package_name!r}"
        observed_versions = {package.get("version") for package in matches}
        assert observed_versions == {expected_version}, (
            f"Cargo.lock version mismatch for {package_name!r}: "
            f"expected={expected_version!r}, observed={sorted(observed_versions)!r}"
        )
        for package in matches:
            assert package.get("source", "").startswith("registry+"), (
                f"Cargo.lock substrate package {package_name!r} must come from a registry"
            )
            assert package.get("checksum"), (
                f"Cargo.lock substrate package {package_name!r} must include a checksum"
            )

    print(
        "verified D6U Cargo.lock substrate packages: "
        + ", ".join(f"{name}={version}" for name, version in package_versions.items())
    )


if __name__ == "__main__":
    main()
