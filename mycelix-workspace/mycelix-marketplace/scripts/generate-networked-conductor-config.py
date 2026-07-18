#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ipaddress
from pathlib import Path
from urllib.parse import urlparse


def quoted(value: str | Path) -> str:
    return '"' + str(value).replace('"', '\\"') + '"'


def validate_endpoint(value: str, schemes: set[str], allow_public: bool, label: str) -> str:
    parsed = urlparse(value)
    if parsed.scheme not in schemes or not parsed.hostname:
        raise SystemExit(f"{label} must use {sorted(schemes)} and include a hostname")
    if allow_public:
        return value
    host = parsed.hostname.lower()
    if host == "localhost":
        return value
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise SystemExit(
            f"{label} must use localhost or a literal private/loopback address unless --allow-public-network-services is set"
        ) from error
    if not (address.is_loopback or address.is_private):
        raise SystemExit(f"{label} points to a public address; pass --allow-public-network-services only for an explicitly reviewed run")
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--admin-port", type=int, required=True)
    parser.add_argument("--bootstrap-url", required=True)
    parser.add_argument("--signal-url", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-public-network-services", action="store_true")
    args = parser.parse_args()

    if not (1024 <= args.admin_port <= 65535):
        raise SystemExit("admin port must be between 1024 and 65535")
    bootstrap = validate_endpoint(
        args.bootstrap_url,
        {"http", "https"},
        args.allow_public_network_services,
        "bootstrap URL",
    )
    signal = validate_endpoint(
        args.signal_url,
        {"ws", "wss"},
        args.allow_public_network_services,
        "signal URL",
    )
    args.data_root.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "---\n"
        f"data_root_path: {quoted(args.data_root.resolve())}\n"
        "keystore:\n"
        "  type: danger_test_keystore\n"
        "admin_interfaces:\n"
        "  - driver:\n"
        "      type: websocket\n"
        f"      port: {args.admin_port}\n"
        "network:\n"
        "  base64_auth_material: null\n"
        f"  bootstrap_url: {quoted(bootstrap)}\n"
        f"  signal_url: {quoted(signal)}\n"
        "  webrtc_config: null\n"
        "  target_arc_factor: 1\n"
        "  report: none\n"
        "  advanced: null\n"
        "request_timeout_s: 60\n"
        "db_sync_strategy: Resilient\n"
        "tuning_params: null\n"
    )
    print(args.output)


if __name__ == "__main__":
    main()
