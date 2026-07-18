#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def quoted(path: Path) -> str:
    return '"' + str(path.resolve()).replace('"', '\\"') + '"'


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--admin-port", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not (1024 <= args.admin_port <= 65535):
        raise SystemExit("admin port must be between 1024 and 65535")
    args.data_root.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "---\n"
        f"data_root_path: {quoted(args.data_root)}\n"
        "keystore:\n"
        "  type: danger_test_keystore\n"
        "admin_interfaces:\n"
        "  - driver:\n"
        "      type: websocket\n"
        f"      port: {args.admin_port}\n"
    )
    print(args.output)


if __name__ == "__main__":
    main()
