#!/usr/bin/env python3
"""Independent parser for TCG PC-client binary TPM2 event logs."""
from __future__ import annotations

import argparse
import hashlib
import json
import struct
import tempfile
from pathlib import Path
from typing import Any

PARSER_ID = "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1"
SPECID_SIGNATURE = b"Spec ID Event03"
EV_NO_ACTION = 0x00000003
EV_SEPARATOR = 0x00000004

ALG_NAMES = {
    0x0004: "sha1",
    0x000B: "sha256",
    0x000C: "sha384",
    0x000D: "sha512",
    0x0012: "sm3_256",
}
ALG_SIZES = {
    "sha1": 20,
    "sha256": 32,
    "sha384": 48,
    "sha512": 64,
    "sm3_256": 32,
}
EVENT_NAMES = {
    EV_NO_ACTION: "EV_NO_ACTION",
    EV_SEPARATOR: "EV_SEPARATOR",
    0x00000005: "EV_ACTION",
    0x00000008: "EV_S_CRTM_VERSION",
    0x80000001: "EV_EFI_VARIABLE_DRIVER_CONFIG",
    0x80000002: "EV_EFI_VARIABLE_BOOT",
    0x80000003: "EV_EFI_BOOT_SERVICES_APPLICATION",
    0x80000004: "EV_EFI_BOOT_SERVICES_DRIVER",
    0x80000005: "EV_EFI_RUNTIME_SERVICES_DRIVER",
    0x80000006: "EV_EFI_GPT_EVENT",
    0x80000007: "EV_EFI_ACTION",
    0x80000008: "EV_EFI_PLATFORM_FIRMWARE_BLOB",
    0x80000010: "EV_EFI_HCRTM_EVENT",
    0x800000E0: "EV_EFI_VARIABLE_AUTHORITY",
}


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def need(data: bytes, offset: int, size: int, reason: str) -> bytes:
    if offset < 0 or size < 0 or offset + size > len(data):
        raise ValueError(reason)
    return data[offset : offset + size]


def u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def parse_specid(data: bytes) -> tuple[int, dict[str, int], int]:
    need(data, 0, 32, "truncated legacy SpecID header")
    if u32(data, 0) != 0:
        raise ValueError("SpecID PCR index must be 0")
    if u32(data, 4) != EV_NO_ACTION:
        raise ValueError("first event must be EV_NO_ACTION SpecID")
    if data[8:28] != b"\x00" * 20:
        raise ValueError("SpecID legacy digest must be zero")
    event_size = u32(data, 28)
    event = need(data, 32, event_size, "truncated SpecID event")
    if len(event) < 16 + 4 + 4:
        raise ValueError("SpecID payload too small")
    if not event.startswith(SPECID_SIGNATURE):
        raise ValueError("missing Spec ID Event03 signature")

    offset = 16 + 4 + 4
    number_of_algorithms = u32(event, offset)
    offset += 4
    if number_of_algorithms == 0:
        raise ValueError("SpecID algorithm table empty")

    algorithms: dict[str, int] = {}
    for _ in range(number_of_algorithms):
        need(event, offset, 4, "truncated SpecID algorithm table")
        alg_id = u16(event, offset)
        digest_size = u16(event, offset + 2)
        name = ALG_NAMES.get(alg_id)
        if name is None:
            raise ValueError(f"unsupported hash algorithm id 0x{alg_id:04x}")
        if digest_size != ALG_SIZES[name]:
            raise ValueError(f"SpecID digest size mismatch for {name}")
        if name in algorithms:
            raise ValueError(f"duplicate SpecID algorithm {name}")
        algorithms[name] = digest_size
        offset += 4

    need(event, offset, 1, "missing SpecID vendor info size")
    vendor_size = event[offset]
    offset += 1
    need(event, offset, vendor_size, "truncated SpecID vendor info")
    offset += vendor_size
    if offset != event_size:
        raise ValueError("unexpected trailing SpecID payload bytes")

    return event_size, algorithms, 32 + event_size


def parse(binary: bytes) -> dict[str, Any]:
    if not binary:
        raise ValueError("empty event log")

    spec_event_size, algorithms, offset = parse_specid(binary)
    events: list[dict[str, Any]] = [
        {
            "sequence": 0,
            "pcr": 0,
            "event_type": "EV_NO_ACTION",
            "payload_hex": binary[32 : 32 + spec_event_size].hex(),
            "digests": {"legacy-sha1": binary[8:28].hex()},
            "session_id": None,
        }
    ]

    sequence = 1
    while offset < len(binary):
        need(binary, offset, 12, "truncated TCG_EVENT2 header")
        pcr = u32(binary, offset)
        event_type_code = u32(binary, offset + 4)
        digest_count = u32(binary, offset + 8)
        offset += 12

        if digest_count == 0:
            raise ValueError(f"event {sequence} has zero digests")

        digests: dict[str, str] = {}
        for _ in range(digest_count):
            need(binary, offset, 2, f"event {sequence} truncated digest algorithm")
            alg_id = u16(binary, offset)
            offset += 2
            name = ALG_NAMES.get(alg_id)
            if name is None:
                raise ValueError(f"event {sequence} uses unsupported algorithm 0x{alg_id:04x}")
            digest_size = algorithms.get(name)
            if digest_size is None:
                raise ValueError(f"event {sequence} uses algorithm absent from SpecID")
            digest = need(binary, offset, digest_size, f"event {sequence} truncated digest")
            offset += digest_size
            if name in digests:
                raise ValueError(f"event {sequence} has duplicate {name} digest")
            digests[name] = digest.hex()

        need(binary, offset, 4, f"event {sequence} missing EventSize")
        event_size = u32(binary, offset)
        offset += 4
        payload = need(binary, offset, event_size, f"event {sequence} truncated Event payload")
        offset += event_size

        item: dict[str, Any] = {
            "sequence": sequence,
            "pcr": pcr,
            "event_type_code": event_type_code,
            "event_type": EVENT_NAMES.get(
                event_type_code, f"UNKNOWN_0x{event_type_code:08x}"
            ),
            "payload_hex": payload.hex(),
            "digests": digests,
            "session_id": None,
        }
        if "sha256" in digests:
            item["digest_sha256"] = digests["sha256"]
        events.append(item)
        sequence += 1

    if offset != len(binary):
        raise ValueError("unconsumed event-log bytes")

    return {
        "profile_id": "mycelix.security.platform.binary-eventlog.extraction",
        "profile_version": "0.1.0",
        "parser_id": PARSER_ID,
        "parser_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "binary_sha256": hashlib.sha256(binary).hexdigest(),
        "specid_event_size": spec_event_size,
        "algorithms": algorithms,
        "events": events,
    }


def self_test() -> int:
    sha1 = 0x0004
    sha256 = 0x000B

    signature = SPECID_SIGNATURE + b"\x00" * (16 - len(SPECID_SIGNATURE))
    spec_payload = (
        signature
        + struct.pack("<I", 0)
        + bytes([0, 2, 0, 2])
        + struct.pack("<I", 2)
        + struct.pack("<HH", sha1, 20)
        + struct.pack("<HH", sha256, 32)
        + bytes([0])
    )
    legacy = struct.pack("<II", 0, EV_NO_ACTION) + (b"\x00" * 20) + struct.pack("<I", len(spec_payload)) + spec_payload

    payload1 = b"\x00\x00\x00\x00"
    payload2 = b"MeasuredAction"
    event1 = (
        struct.pack("<III", 0, EV_SEPARATOR, 1)
        + struct.pack("<H", sha256)
        + hashlib.sha256(payload1).digest()
        + struct.pack("<I", len(payload1))
        + payload1
    )
    event2 = (
        struct.pack("<III", 4, 0x80000007, 1)
        + struct.pack("<H", sha256)
        + hashlib.sha256(payload2).digest()
        + struct.pack("<I", len(payload2))
        + payload2
    )
    binary = legacy + event1 + event2
    parsed = parse(binary)

    if parsed["events"][1]["payload_hex"] != payload1.hex():
        print("payload extraction: FAIL")
        return 1
    if parsed["events"][1]["digest_sha256"] != hashlib.sha256(payload1).hexdigest():
        print("digest extraction: FAIL")
        return 1
    if parsed["events"][2]["payload_hex"] != payload2.hex():
        print("second payload extraction: FAIL")
        return 1

    truncated = binary[:-1]
    try:
        parse(truncated)
    except ValueError:
        pass
    else:
        print("truncated-log rejection: FAIL")
        return 1

    bad_alg = bytearray(binary)
    bad_alg[len(legacy) + 12 : len(legacy) + 14] = struct.pack("<H", 0x1234)
    try:
        parse(bytes(bad_alg))
    except ValueError:
        pass
    else:
        print("unknown-algorithm rejection: FAIL")
        return 1

    print("raw TCG PC-client event-log parser self-test: PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--parse", metavar="EVENTLOG")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    path = Path(args.parse)
    parsed = parse(path.read_bytes())
    rendered = json.dumps(parsed, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
