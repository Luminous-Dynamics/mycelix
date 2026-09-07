#!/usr/bin/env python3
"""Capture a canonical, machine-checkable Holochain qualification environment capsule."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

HEX40 = re.compile(r"^[0-9a-f]{40}$")


def run(*argv: str, required: bool = True) -> str | None:
    if shutil.which(argv[0]) is None:
        if required:
            raise SystemExit(f"required qualification tool not found: {argv[0]}")
        return None
    proc = subprocess.run(argv, check=False, capture_output=True, text=True)
    output = (proc.stdout + proc.stderr).strip()
    if proc.returncode != 0:
        if required:
            raise SystemExit(
                f"qualification tool failed ({proc.returncode}): {' '.join(argv)}\n{output}"
            )
        return None
    return output


def first_line(value: str | None) -> str | None:
    if value is None:
        return None
    lines = value.splitlines()
    return lines[0].strip() if lines else ""


def parse_os_release(path: Path = Path("/etc/os-release")) -> dict[str, str]:
    if not path.is_file():
        raise SystemExit("qualification requires /etc/os-release")
    result: dict[str, str] = {}
    for raw in path.read_text().splitlines():
        raw = raw.strip()
        if not raw or raw.startswith("#") or "=" not in raw:
            continue
        key, value = raw.split("=", 1)
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {'\"', "'"}:
            value = value[1:-1]
        result[key] = value
    return result


def parse_actions(values: list[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise SystemExit(f"--action must be NAME=SHA: {value!r}")
        name, sha = value.split("=", 1)
        if not name or name in result:
            raise SystemExit(f"invalid/duplicate action name: {name!r}")
        if not HEX40.fullmatch(sha):
            raise SystemExit(f"action {name!r} is not pinned to a 40-hex commit: {sha!r}")
        result[name] = sha
    if not result:
        raise SystemExit("at least one --action binding is required")
    return dict(sorted(result.items()))


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--lineage-output", required=True)
    parser.add_argument("--runner-label", required=True)
    parser.add_argument("--expected-rust", required=True)
    parser.add_argument("--expected-nix", required=True)
    parser.add_argument("--nix-installer-url", required=True)
    parser.add_argument("--action", action="append", default=[])
    args = parser.parse_args()

    actions = parse_actions(args.action)
    os_release = parse_os_release()

    if args.runner_label != "ubuntu-24.04":
        raise SystemExit(f"unexpected qualification runner label: {args.runner_label!r}")
    if os_release.get("ID") != "ubuntu" or os_release.get("VERSION_ID") != "24.04":
        raise SystemExit(
            "qualification runner is not Ubuntu 24.04: "
            f"ID={os_release.get('ID')!r} VERSION_ID={os_release.get('VERSION_ID')!r}"
        )

    rustc_vv = run("rustc", "-vV")
    cargo_version = run("cargo", "--version")
    nix_version = run("nix", "--version")
    git_version = run("git", "--version")
    python_version = sys.version.replace("\n", " ")

    rust_release = None
    for line in (rustc_vv or "").splitlines():
        if line.startswith("release: "):
            rust_release = line.removeprefix("release: ").strip()
            break
    if rust_release != args.expected_rust:
        raise SystemExit(
            f"rustc release {rust_release!r} != expected {args.expected_rust!r}"
        )
    if args.expected_nix not in (nix_version or ""):
        raise SystemExit(
            f"nix version {nix_version!r} does not contain expected {args.expected_nix!r}"
        )

    optional_native = {
        "cc": first_line(run("cc", "--version", required=False)),
        "ld": first_line(run("ld", "--version", required=False)),
        "cmake": first_line(run("cmake", "--version", required=False)),
        "pkg_config": first_line(run("pkg-config", "--version", required=False)),
        "openssl": first_line(run("openssl", "version", required=False)),
    }

    runner = {
        "requested_label": args.runner_label,
        "runner_os": os.environ.get("RUNNER_OS", "unknown"),
        "runner_arch": os.environ.get("RUNNER_ARCH", "unknown"),
        "image_os": os.environ.get("ImageOS", "unknown"),
        "image_version": os.environ.get("ImageVersion", "unknown"),
    }
    kernel = {
        "system": platform.system(),
        "release": platform.release(),
        "version": platform.version(),
        "machine": platform.machine(),
    }
    tools = {
        "git": git_version,
        "python": python_version,
        "rustc_vv": rustc_vv,
        "cargo": cargo_version,
        "nix": nix_version,
        "native": optional_native,
    }
    nix_installer = {
        "version": args.expected_nix,
        "url": args.nix_installer_url,
    }

    environment_lineage = {
        "schema": 1,
        "runner": runner,
        "kernel": kernel,
        "os_release": dict(sorted(os_release.items())),
        "actions": actions,
        "nix_installer": nix_installer,
        "tools": tools,
    }
    lineage_sha256 = canonical_sha256(environment_lineage)

    capsule = {
        "schema": 2,
        "environment_lineage_sha256": lineage_sha256,
        "environment_lineage": environment_lineage,
        "workflow": {
            "ref": os.environ.get("GITHUB_WORKFLOW_REF", "unknown"),
            "sha": os.environ.get("GITHUB_WORKFLOW_SHA", "unknown"),
            "run_id": os.environ.get("GITHUB_RUN_ID", "unknown"),
            "run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT", "unknown"),
            "event_name": os.environ.get("GITHUB_EVENT_NAME", "unknown"),
        },
    }

    output = Path(args.output)
    lineage_output = Path(args.lineage_output)
    output.parent.mkdir(parents=True, exist_ok=True)
    lineage_output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(capsule, indent=2, sort_keys=True) + "\n")
    lineage_output.write_text(f"{lineage_sha256}\n")

    if len(lineage_sha256) != 64:
        raise SystemExit("invalid environment lineage digest")
    print(f"Holochain qualification environment lineage: {lineage_sha256}")


if __name__ == "__main__":
    main()
