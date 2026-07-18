#!/usr/bin/env python3
"""Render a concrete Marketplace hApp manifest from verified DNA bundle paths."""
from __future__ import annotations

import argparse
import os
from pathlib import Path


def verified_bundle(raw: str, label: str) -> Path:
    path = Path(raw).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"{label} DNA bundle does not exist: {path}")
    if path.suffix != ".dna":
        raise SystemExit(f"{label} bundle must end in .dna: {path}")
    return path


def yaml_path(path: Path) -> str:
    # POSIX absolute paths are valid scalar values. Quote defensively for spaces.
    return '"' + str(path).replace('"', '\\"') + '"'


def role(name: str, dna: Path, deferred: bool) -> str:
    return f"""  - name: {name}
    provisioning:
      strategy: create
      deferred: {str(deferred).lower()}
    dna:
      path: {yaml_path(dna)}
      modifiers:
        network_seed: ~
        properties: ~
      clone_limit: 0
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--marketplace-dna", required=True)
    parser.add_argument("--finance-dna", required=True)
    parser.add_argument("--identity-dna")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    marketplace = verified_bundle(args.marketplace_dna, "Marketplace")
    finance = verified_bundle(args.finance_dna, "Finance")
    identity = verified_bundle(args.identity_dna, "Identity") if args.identity_dna else None
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    parts = [
        "---\n",
        "manifest_version: \"0\"\n",
        "name: mycelix_marketplace\n",
        "description: |\n",
        "  Settlement-capable Mycelix Marketplace deployment.\n",
        "  Generated from explicit Marketplace and Finance DNA bundles.\n",
        "roles:\n",
        role("marketplace", marketplace, False),
        role("finance", finance, False),
    ]
    if identity:
        parts.append(role("identity", identity, True))
    output.write_text("".join(parts))
    os.chmod(output, 0o644)
    print(output)


if __name__ == "__main__":
    main()
