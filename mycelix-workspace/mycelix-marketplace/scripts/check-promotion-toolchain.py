#!/usr/bin/env python3
"""Verify the promotion toolchain contract without silently accepting drift."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = json.loads((ROOT / "contracts/promotion-toolchain-v1.json").read_text())


def run(*args: str) -> str:
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise SystemExit(f"failed to execute {' '.join(args)}: {exc}") from exc


def binary(name: str, env_name: str | None = None) -> Path:
    raw = os.environ.get(env_name, "") if env_name else ""
    resolved = Path(raw).expanduser().resolve() if raw else None
    if resolved and resolved.is_file():
        return resolved
    found = shutil.which(name)
    if not found:
        suffix = f" or set {env_name}" if env_name else ""
        raise SystemExit(f"required binary not found: {name}{suffix}")
    return Path(found).resolve()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def exact_version(output: str, expected: str, label: str) -> None:
    if not re.search(rf"(?<![0-9.]){re.escape(expected)}(?![0-9.])", output):
        raise SystemExit(f"{label} version drift: expected {expected}, got {output!r}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--allow-missing-holochain", action="store_true")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args()

    rust_expected = CONTRACT["rust"]["channel"]
    rustc = run("rustc", "--version")
    cargo = run("cargo", "--version")
    exact_version(rustc, rust_expected, "rustc")
    exact_version(run("rustup", "show", "active-toolchain"), rust_expected, "rustup")
    targets = run("rustup", "target", "list", "--installed").splitlines()
    required_target = CONTRACT["rust"]["required_target"]
    if required_target not in targets:
        raise SystemExit(f"missing Rust target: {required_target}")

    node = run("node", "--version")
    match = re.fullmatch(r"v(\d+)\..*", node)
    if not match:
        raise SystemExit(f"could not parse Node version: {node}")
    major = int(match.group(1))
    node_contract = CONTRACT["node"]
    if not node_contract["minimum_major"] <= major <= node_contract["maximum_major"]:
        raise SystemExit(f"Node major {major} is outside the supported range")

    package = json.loads((ROOT / "frontend-leptos/bridge/package.json").read_text())
    actual_client = package["dependencies"][CONTRACT["browser_client"]["package"]]
    expected_client = CONTRACT["browser_client"]["version"]
    if actual_client != expected_client:
        raise SystemExit(
            f"browser client drift: expected {expected_client}, got {actual_client}"
        )

    result: dict[str, object] = {
        "contract_version": CONTRACT["version"],
        "rustc": rustc,
        "cargo": cargo,
        "node": node,
        "npm": run("npm", "--version"),
        "python": run("python3", "--version"),
        "openssl": run("openssl", "version"),
        "browser_client": actual_client,
    }

    for name, env_name in (("holochain", "HOLOCHAIN_BIN"), ("hc", "HC_BIN")):
        try:
            path = binary(name, env_name)
        except SystemExit:
            if args.allow_missing_holochain:
                result[name] = {"present": False}
                continue
            raise
        output = run(str(path), "--version")
        exact_version(output, CONTRACT["holochain"]["version"], name)
        result[name] = {
            "present": True,
            "path": str(path),
            "version": output,
            "sha256": sha256(path),
        }

    if args.as_json:
        print(json.dumps(result, sort_keys=True, indent=2))
    else:
        print("promotion toolchain contract: PASS")
        for key, value in result.items():
            print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
