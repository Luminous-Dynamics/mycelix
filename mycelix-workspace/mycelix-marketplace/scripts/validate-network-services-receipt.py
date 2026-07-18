#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
from pathlib import Path
from urllib.parse import urlparse

HEX = set("0123456789abcdef")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def private_endpoint(value: object, schemes: set[str], label: str) -> str:
    if not isinstance(value, str):
        raise SystemExit(f"{label} must be a URL string")
    parsed = urlparse(value)
    if parsed.scheme not in schemes or not parsed.hostname:
        raise SystemExit(f"{label} must use {sorted(schemes)} and include a hostname")
    host = parsed.hostname.lower()
    if host == "localhost":
        return value
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise SystemExit(f"{label} must use localhost or a literal private/loopback address") from error
    if not (address.is_loopback or address.is_private):
        raise SystemExit(f"{label} must not use a public address for isolated network evidence")
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("receipt", type=Path)
    parser.add_argument("--implementation", type=Path, required=True)
    args = parser.parse_args()
    value = json.loads(args.receipt.read_text())
    if value.get("schema_version") != 1:
        raise SystemExit("network service receipt schema_version must be 1")
    if value.get("isolation") != "local_controlled":
        raise SystemExit("network services must report local_controlled isolation")
    private_endpoint(value.get("bootstrap_url"), {"http", "https"}, "bootstrap_url")
    private_endpoint(value.get("signal_url"), {"ws", "wss"}, "signal_url")
    expected = digest(args.implementation)
    actual = value.get("implementation_sha256")
    if not isinstance(actual, str) or len(actual) != 64 or any(ch not in HEX for ch in actual.lower()):
        raise SystemExit("implementation_sha256 must be a lowercase SHA-256 digest")
    if actual != expected:
        raise SystemExit("network service receipt does not bind the supplied implementation")
    print(json.dumps(value, sort_keys=True))


if __name__ == "__main__":
    main()
