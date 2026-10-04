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
VERSION_RE = re.compile(r"(?m)^\s*version:\s*(\d+)\s*$")
EVENT_RE = re.compile(
    r"(?ms)^\s*-\s*EventNum:\s*(?P<num>\d+)\s*\n"
    r"(?P<body>.*?)(?=^\s*-\s*EventNum:\s*\d+\s*$|^\s*pcrs:\s*$|\Z)"
)
PCR_RE = re.compile(r"^\s*PCRIndex:\s*(\d+)\s*$", re.MULTILINE)
TYPE_RE = re.compile(r"^\s*EventType:\s*([^\s#]+)\s*$", re.MULTILINE)
SHA256_DIGEST_RE = re.compile(
    r"^\s*AlgorithmId:\s*sha256\s*$\n\s*Digest:\s*[\"']?([0-9A-Fa-fx]+)[\"']?\s*$",
    re.MULTILINE,
)
PCR_VALUE_RE = re.compile(
    r"^\s*(\d+)\s*:\s*(?:0x)?([0-9A-Fa-f]{64})\s*$", re.MULTILINE
)
STARTUP_HEX_RE = re.compile(r"537461727475704c6f63616c69747900([0-9A-Fa-f]{2})")

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def parse_sha256_digest(body: str) -> str | None:
    matches = SHA256_DIGEST_RE.findall(body)
    if len(matches) > 1:
        raise ValueError("ambiguous multiple SHA-256 digests for one event")
    if not matches:
        return None
    digest = matches[0].lower()
    if digest.startswith("0x"):
        digest = digest[2:]
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

def parse_observed_pcrs(text: str, selection: str) -> dict[str, str]:
    bank, sep, rest = selection.partition(":")
    if bank != "sha256" or not sep:
        raise ValueError("adapter only supports sha256 PCR selection")
    wanted = [int(x) for x in rest.split(",") if x.isdigit()]
    if not wanted or len(wanted) != len(set(wanted)):
        raise ValueError("invalid PCR selection")
    marker = re.search(r"(?m)^\s*pcrs:\s*$", text)
    if not marker:
        raise ValueError("missing pcrs section")
    tail = text[marker.end():]
    sha_marker = re.search(r"(?m)^\s*sha256\s*:\s*$", tail)
    if not sha_marker:
        raise ValueError("missing sha256 pcrs section")
    sha_section = tail[sha_marker.end():]
    next_bank = re.search(r"(?m)^\s*[A-Za-z][A-Za-z0-9_-]*\s*:\s*$", sha_section)
    if next_bank:
        sha_section = sha_section[:next_bank.start()]
    values: dict[str, str] = {}
    for match in PCR_VALUE_RE.finditer(sha_section):
        index = int(match.group(1))
        if index in wanted:
            if str(index) in values:
                raise ValueError(f"duplicate observed PCR{index}")
            values[str(index)] = match.group(2).lower()
    missing = [str(index) for index in wanted if str(index) not in values]
    if missing:
        raise ValueError("missing observed PCRs: " + ",".join(missing))
    return {str(index): values[str(index)] for index in sorted(wanted)}

def adapt(yaml_text: str, binary_eventlog: Path, session_id: str, pcr_selection: str) -> dict[str, Any]:
    version = VERSION_RE.search(yaml_text)
    if not version or int(version.group(1)) != 1:
        raise ValueError("only tpm2_eventlog YAML version 1 is supported")
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
        events.append(event)
    if not events:
        raise ValueError("no events found")
    observed = parse_observed_pcrs(yaml_text, pcr_selection)
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
    # The binary input only supplies a provenance digest in this self-test.
    digest_path = binary.parent / (binary.name + ".adapter-test-bin")
    digest_path.write_bytes(b"adapter-fixture")
    try:
        result = adapt(simple_yaml, digest_path, "adapter-self-test", "sha256:0,4")
        if result["events"][0].get("digest_sha256") is not None:
            print("EV_NO_ACTION handling: FAIL")
            return 1
        if result["events"][1]["digest_sha256"] != d11:
            print("SHA-256 digest selection: FAIL")
            return 1
        startup_yaml = simple_yaml.replace(
            "EventType: EV_EFI_BOOT_SERVICES_APPLICATION", "EventType: EV_NO_ACTION", 1
        ).replace(
            '    Event: "fixture"', '    Event: "537461727475704c6f63616c6974790003"', 1
        )
        startup = adapt(startup_yaml, digest_path, "adapter-self-test", "sha256:0,4")
        if startup["events"][1].get("startup_locality") != 3:
            print("StartupLocality detection: FAIL")
            return 1
        ambiguous = simple_yaml.replace(
            f'      - AlgorithmId: sha256\n        Digest: "{d11}"',
            f'      - AlgorithmId: sha256\n        Digest: "{d11}"\n      - AlgorithmId: sha256\n        Digest: "{d22}"',
            1,
        )
        try:
            adapt(ambiguous, digest_path, "adapter-self-test", "sha256:0,4")
        except ValueError:
            pass
        else:
            print("Ambiguous SHA-256 detection: FAIL")
            return 1
        print("tpm2_eventlog YAML v1 adapter self-test: PASS")
        return 0
    finally:
        digest_path.unlink(missing_ok=True)

def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--adapt", metavar="YAML")
    parser.add_argument("--binary-eventlog", metavar="BINARY_EVENTLOG")
    parser.add_argument("--session-id")
    parser.add_argument("--pcr-selection", default="sha256:0,2,4,7")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.binary_eventlog or not args.session_id:
        parser.error("--binary-eventlog and --session-id are required with --adapt")
    result = adapt(Path(args.adapt).read_text(encoding="utf-8"), Path(args.binary_eventlog), args.session_id, args.pcr_selection)
    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())