#!/usr/bin/env python3
"""Bounded adapter from tpm2_eventlog YAML v1 to Mycelix canonical replay input."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any

ADAPTER_ID = "mycelix.pc-client.tpm2-eventlog-yaml-v1-adapter"
RAW_PAYLOAD_PARSER_ID = "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1"
RAW_PAYLOAD_PARSER_SCRIPT = Path(__file__).resolve().with_name("parse_mycelix_raw_tpm2_eventlog_v0_1.py")
VERSION_RE = re.compile(r"(?m)^\s*version:\s*(\d+)\s*$")
EVENT_RE = re.compile(
    r"(?ms)^\s*-\s*EventNum:\s*(?P<num>\d+)\s*\n"
    r"(?P<body>.*?)(?=^\s*-\s*EventNum:\s*\d+\s*$|^\s*pcrs:\s*$|\Z)"
)
PCR_RE = re.compile(r"^\s*PCRIndex:\s*(\d+)\s*$", re.MULTILINE)
TYPE_RE = re.compile(r"^\s*EventType:\s*([^\s#]+)\s*$", re.MULTILINE)
SHA256_DIGEST_RE = re.compile(
    r"^\s*-?\s*AlgorithmId:\s*sha256\s*$\n\s*Digest:\s*[\"']?([0-9A-Fa-fx]+)[\"']?\s*$",
    re.MULTILINE,
)
STARTUP_HEX_RE = re.compile(r"537461727475704c6f63616c69747900([0-9A-Fa-f]{2})")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def validate_selection(selection: str) -> list[int]:
    bank, sep, rest = selection.partition(":")
    if bank != "sha256" or not sep:
        raise ValueError("adapter only supports sha256 PCR selection")
    items = rest.split(",")
    if not items or any(not item.isdigit() for item in items):
        raise ValueError("invalid PCR selection")
    wanted = [int(item) for item in items]
    if len(wanted) != len(set(wanted)):
        raise ValueError("duplicate PCR selection")
    return wanted


def parse_observed_pcr_json(path: Path, selection: str) -> dict[str, str]:
    wanted = validate_selection(selection)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("observed PCR JSON must be an object")
    expected = {str(index) for index in wanted}
    if set(value) != expected:
        raise ValueError("observed PCR JSON does not exactly match PCR selection")
    normalized: dict[str, str] = {}
    for key, digest in value.items():
        if not isinstance(digest, str):
            raise ValueError(f"observed PCR{key} is not a string")
        digest = digest.lower().removeprefix("0x")
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise ValueError(f"invalid observed PCR{key}")
        normalized[key] = digest
    return {str(index): normalized[str(index)] for index in sorted(wanted)}


def parse_sha256_digest(body: str) -> str | None:
    matches = SHA256_DIGEST_RE.findall(body)
    if len(matches) > 1:
        raise ValueError("ambiguous multiple SHA-256 digests for one event")
    if not matches:
        return None
    digest = matches[0].lower().removeprefix("0x")
    if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
        raise ValueError("invalid SHA-256 event digest")
    return digest


def parse_startup_locality(body: str) -> int | None:
    match = STARTUP_HEX_RE.search(body)
    if not match:
        return None
    locality = int(match.group(1), 16)
    if locality > 4:
        raise ValueError(f"invalid StartupLocality {locality}")
    return locality


def adapt(
    yaml_text: str,
    binary_eventlog: Path,
    observed_pcr_json: Path,
    session_id: str,
    pcr_selection: str,
    payload_json: Path | None = None,
) -> dict[str, Any]:
    version = VERSION_RE.search(yaml_text)
    if not version or int(version.group(1)) != 1:
        raise ValueError("only tpm2_eventlog YAML version 1 is supported")

    payloads: dict[int, dict[str, Any]] = {}
    raw_payload_metadata: dict[str, Any] = {}
    if payload_json is not None:
        raw = json.loads(payload_json.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError("raw payload parser output must be an object")
        if raw.get("parser_id") != RAW_PAYLOAD_PARSER_ID:
            raise ValueError("unexpected raw payload parser id")
        if raw.get("binary_sha256") != sha256_file(binary_eventlog):
            raise ValueError("raw payload parser binary binding mismatch")
        if raw.get("parser_source_sha256") != sha256_file(RAW_PAYLOAD_PARSER_SCRIPT):
            raise ValueError("raw payload parser source binding mismatch")
        raw_payload_metadata = {
            "parser_id": raw["parser_id"],
            "binary_sha256": raw["binary_sha256"],
            "source_sha256": raw["parser_source_sha256"],
        }
        raw_events = raw.get("events")
        if not isinstance(raw_events, list):
            raise ValueError("raw payload parser output must contain an events list")
        for item in raw_events:
            if not isinstance(item, dict) or not isinstance(item.get("sequence"), int):
                raise ValueError("raw payload parser emitted an invalid event")
            payloads[item["sequence"]] = item
        if len(payloads) != len(raw_events):
            raise ValueError("raw payload parser emitted duplicate event sequence")
    events: list[dict[str, Any]] = []
    previous_event_num = -1
    for event_match in EVENT_RE.finditer(yaml_text):
        event_num = int(event_match.group("num"))
        body = event_match.group("body")
        pcr_match = PCR_RE.search(body)
        type_match = TYPE_RE.search(body)
        if not pcr_match or not type_match:
            raise ValueError(f"event {event_num} missing PCRIndex or EventType")
        if event_num <= previous_event_num:
            raise ValueError("EventNum is not strictly increasing")
        previous_event_num = event_num

        event_type = type_match.group(1)
        event: dict[str, Any] = {
            "sequence": event_num,
            "pcr": int(pcr_match.group(1)),
            "event_type": event_type,
            "session_id": session_id,
        }
        digest = parse_sha256_digest(body)
        if digest is not None:
            event["digest_sha256"] = digest
        if event_type == "EV_NO_ACTION":
            locality = parse_startup_locality(body)
            if locality is not None:
                event["startup_locality"] = locality
        elif digest is None:
            raise ValueError(f"event {event_num} has no SHA-256 digest")

        if payload_json is not None:
            raw_event = payloads.get(event_num)
            if raw_event is None:
                raise ValueError(f"raw parser is missing EventNum {event_num}")
            if raw_event.get("pcr") != event["pcr"] or raw_event.get("event_type") != event["event_type"]:
                raise ValueError(f"raw/YAML event identity mismatch at EventNum {event_num}")
            raw_digest = raw_event.get("digest_sha256")
            if digest is not None and raw_digest != digest:
                raise ValueError(f"raw/YAML SHA-256 digest mismatch at EventNum {event_num}")
            payload_hex = raw_event.get("payload_hex")
            if not isinstance(payload_hex, str):
                raise ValueError(f"raw parser is missing payload bytes at EventNum {event_num}")
            event["payload_hex"] = payload_hex.lower().removeprefix("0x")
        events.append(event)

    if not events:
        raise ValueError("no events found")

    observed = parse_observed_pcr_json(observed_pcr_json, pcr_selection)
    return {
        "profile_id": "mycelix.security.platform.eventlog.reconstruction",
        "profile_version": "0.1.0",
        "event_log_sha256": sha256_file(binary_eventlog),
        "session_id": session_id,
        "pcr_bank": "sha256",
        "pcr_selection": pcr_selection,
        "events": events,
        "observed_pcr_values": observed,
        "adapter_id": ADAPTER_ID,
        "adapter_source_sha256": sha256_file(Path(__file__).resolve()),
        "adapter_yaml_version": 1,
        "observed_pcr_values_source": observed_pcr_json.name,
        "payload_parser_source": payload_json.name if payload_json is not None else None,
        "payload_parser_sha256": sha256_file(payload_json) if payload_json is not None else None,
        "payload_parser_metadata": raw_payload_metadata,
    }


def self_test() -> int:
    binary = Path(__file__).resolve()
    d11, d22, zeros = "11" * 32, "22" * 32, "0" * 64
    simple_yaml = f"""---
version: 1
events:
  - EventNum: 0
    PCRIndex: 0
    EventType: EV_NO_ACTION
    Digest: "{zeros}"
    EventSize: 32
    SpecID:
      - Signature: Spec ID Event03
  - EventNum: 1
    PCRIndex: 4
    EventType: EV_EFI_BOOT_SERVICES_APPLICATION
    DigestCount: 2
    Digests:
      - AlgorithmId: sha1
        Digest: "0000000000000000000000000000000000000000"
      - AlgorithmId: sha256
        Digest: "{d11}"
    EventSize: 1
    Event: "fixture"
  - EventNum: 2
    PCRIndex: 4
    EventType: EV_EFI_ACTION
    DigestCount: 1
    Digests:
      - AlgorithmId: sha256
        Digest: "{d22}"
    EventSize: 1
    Event: "fixture"
pcrs:
  sha256:
    0 : 0x0000000000000000000000000000000000000000000000000000000000000000
    4 : 0x0000000000000000000000000000000000000000000000000000000000000000
"""

    digest_path = binary.parent / (binary.name + ".adapter-test-bin")
    observed_path = binary.parent / (binary.name + ".adapter-test-observed.json")
    digest_path.write_bytes(b"adapter-fixture")
    observed_path.write_text(
        json.dumps({"0": zeros, "4": zeros}) + "\n",
        encoding="utf-8",
    )
    try:
        result = adapt(
            simple_yaml, digest_path, observed_path, "adapter-self-test", "sha256:0,4"
        )
        if result["events"][0].get("digest_sha256") is not None:
            print("EV_NO_ACTION handling: FAIL")
            return 1
        if result["events"][1]["digest_sha256"] != d11:
            print("SHA-256 digest selection: FAIL")
            return 1
        if result["observed_pcr_values"] != {"0": zeros, "4": zeros}:
            print("independent observed-PCR source: FAIL")
            return 1

        startup_yaml = simple_yaml.replace(
            "EventType: EV_EFI_BOOT_SERVICES_APPLICATION",
            "EventType: EV_NO_ACTION",
            1,
        ).replace(
            '    Event: "fixture"',
            '    Event: "537461727475704c6f63616c6974790003"',
            1,
        )
        startup = adapt(
            startup_yaml, digest_path, observed_path, "adapter-self-test", "sha256:0,4"
        )
        if startup["events"][1].get("startup_locality") != 3:
            print("StartupLocality detection: FAIL")
            return 1

        ambiguous = simple_yaml.replace(
            f'      - AlgorithmId: sha256\n        Digest: "{d11}"',
            f'      - AlgorithmId: sha256\n        Digest: "{d11}"\n      - AlgorithmId: sha256\n        Digest: "{d22}"',
            1,
        )
        try:
            adapt(
                ambiguous,
                digest_path,
                observed_path,
                "adapter-self-test",
                "sha256:0,4",
            )
        except ValueError:
            pass
        else:
            print("Ambiguous SHA-256 detection: FAIL")
            return 1

        v2_yaml = simple_yaml.replace("version: 1", "version: 2", 1)
        try:
            adapt(v2_yaml, digest_path, observed_path, "adapter-self-test", "sha256:0,4")
        except ValueError:
            pass
        else:
            print("YAML v2 rejection: FAIL")
            return 1

        print("tpm2_eventlog YAML v1 adapter self-test: PASS")
        return 0
    finally:
        digest_path.unlink(missing_ok=True)
        observed_path.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--adapt", metavar="YAML")
    parser.add_argument("--binary-eventlog", metavar="BINARY_EVENTLOG")
    parser.add_argument("--observed-pcr-json", metavar="OBSERVED_PCR_JSON")
    parser.add_argument("--payload-json", metavar="PAYLOAD_JSON")
    parser.add_argument("--session-id")
    parser.add_argument("--pcr-selection", default="sha256:0,2,4,7")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    if not args.binary_eventlog or not args.session_id or not args.observed_pcr_json:
        parser.error(
            "--binary-eventlog, --observed-pcr-json and --session-id are required with --adapt"
        )

    result = adapt(
        Path(args.adapt).read_text(encoding="utf-8"),
        Path(args.binary_eventlog),
        Path(args.observed_pcr_json),
        args.session_id,
        args.pcr_selection,
        Path(args.payload_json) if args.payload_json else None,
    )
    if args.output:
        Path(args.output).write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
