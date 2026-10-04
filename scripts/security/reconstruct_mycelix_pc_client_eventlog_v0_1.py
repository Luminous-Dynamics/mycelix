#!/usr/bin/env python3
"""Independent SHA-256 PCR replay for Mycelix PC-client event logs v0.1."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-pc-client-eventlog-reconstruction-v0.1.json"


def canonical_hash(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    return hashlib.sha256(raw).hexdigest()


def valid_digest(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(c in "0123456789abcdef" for c in value)
    )


def self_hash(value: dict[str, Any], field: str) -> str:
    clone = copy.deepcopy(value)
    clone.pop(field, None)
    return canonical_hash(clone)


def reconstruct(stream: dict[str, Any]) -> tuple[str, str, str | None]:
    required = {
        "profile_id", "profile_version", "pcr_bank", "target_pcr",
        "events", "observed_pcr_sha256"
    }
    missing = required - set(stream)
    if missing:
        return "DENY", "missing-" + "-".join(sorted(missing)), None
    if stream["profile_id"] != "mycelix.security.platform.eventlog.reconstruction":
        return "DENY", "profile-id-mismatch", None
    if stream["profile_version"] != "0.1.0":
        return "DENY", "profile-version-mismatch", None
    if stream["pcr_bank"] != "sha256":
        return "DENY", "wrong-bank", None
    if not isinstance(stream["target_pcr"], int) or stream["target_pcr"] < 0:
        return "DENY", "invalid-target-pcr", None
    if not isinstance(stream["events"], list) or not stream["events"]:
        return "DENY", "empty-event-log", None
    if not valid_digest(stream["observed_pcr_sha256"]):
        return "INDETERMINATE", "missing-or-invalid-observed-pcr", None

    state = bytes(32)
    previous_sequence = -1
    target_seen = False
    for event in stream["events"]:
        if not isinstance(event, dict):
            return "DENY", "event-not-object", None
        for key in ("sequence", "pcr", "event_type", "digest_sha256"):
            if key not in event:
                return "DENY", "missing-event-" + key, None
        if not isinstance(event["sequence"], int) or event["sequence"] <= previous_sequence:
            return "DENY", "non-increasing-sequence", None
        previous_sequence = event["sequence"]
        if not isinstance(event["pcr"], int) or event["pcr"] < 0:
            return "DENY", "invalid-pcr-index", None
        if not valid_digest(event["digest_sha256"]):
            return "DENY", "malformed-digest", None
        if event.get("digest_algorithm", "sha256") != "sha256":
            return "DENY", "digest-algorithm-substitution", None
        if event["pcr"] == stream["target_pcr"]:
            target_seen = True
            measurement = bytes.fromhex(event["digest_sha256"])
            state = hashlib.sha256(state + measurement).digest()

    if not target_seen:
        return "INDETERMINATE", "target-pcr-missing-from-events", None

    reconstructed = state.hex()
    if reconstructed != stream["observed_pcr_sha256"]:
        return "DENY", "reconstructed-pcr-mismatch", reconstructed
    return "PASS", "pcr-reconstruction-matches-observed", reconstructed


def fixture() -> dict[str, Any]:
    a = "11" * 32
    b = "22" * 32
    state = hashlib.sha256(bytes(32) + bytes.fromhex(a)).digest()
    state = hashlib.sha256(state + bytes.fromhex(b)).digest()
    return {
        "profile_id": "mycelix.security.platform.eventlog.reconstruction",
        "profile_version": "0.1.0",
        "pcr_bank": "sha256",
        "target_pcr": 4,
        "session_id": "session-20261004-0001",
        "events": [
            {
                "sequence": 1,
                "pcr": 4,
                "event_type": "EV_EFI_BOOT_SERVICES_APPLICATION",
                "digest_sha256": a,
                "session_id": "session-20261004-0001"
            },
            {
                "sequence": 2,
                "pcr": 4,
                "event_type": "EV_SEPARATOR",
                "digest_sha256": b,
                "session_id": "session-20261004-0001"
            },
            {
                "sequence": 3,
                "pcr": 7,
                "event_type": "EV_EFI_VARIABLE_AUTHORITY",
                "digest_sha256": "33" * 32,
                "session_id": "session-20261004-0001"
            }
        ],
        "observed_pcr_sha256": state.hex()
    }


def mutate(base: dict[str, Any], name: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    if name == "canonical-valid":
        return value
    if name == "digest-substitution":
        value["events"][0]["digest_sha256"] = "44" * 32
    elif name == "event-removal":
        value["events"].pop(1)
    elif name == "event-insertion":
        value["events"].insert(1, {
            "sequence": 4, "pcr": 4, "event_type": "EV_ACTION",
            "digest_sha256": "55" * 32, "session_id": value["session_id"]
        })
    elif name == "event-reordering":
        value["events"][0], value["events"][1] = value["events"][1], value["events"][0]
    elif name == "pcr-index-substitution":
        value["events"][0]["pcr"] = 5
    elif name == "sequence-duplicate":
        value["events"][1]["sequence"] = value["events"][0]["sequence"]
    elif name == "malformed-digest":
        value["events"][0]["digest_sha256"] = "aa"
    elif name == "wrong-bank":
        value["pcr_bank"] = "sha1"
    elif name == "expected-pcr-substitution":
        value["observed_pcr_sha256"] = "66" * 32
    elif name == "missing-expected-pcr":
        value["observed_pcr_sha256"] = ""
    elif name == "empty-event-log":
        value["events"] = []
    elif name == "key-order-permutation":
        value = dict(reversed(list(value.items())))
        value["events"] = [dict(reversed(list(e.items()))) for e in value["events"]]
    elif name == "metadata-only-substitution":
        value["events"][0]["event_type"] = "VENDOR_UNTRUSTED_LABEL"
    elif name == "reconstruction-unavailable":
        value["observed_pcr_sha256"] = "not-available"
    elif name == "invalid-initial-pcr":
        value["initial_pcr_sha256"] = "77" * 32
    elif name == "digest-algorithm-substitution":
        value["events"][0]["digest_algorithm"] = "sha1"
    elif name == "target-pcr-missing-from-events":
        for event in value["events"]:
            event["pcr"] = 7
    elif name == "multi-pcr-canonical":
        return value
    elif name == "target-event-cross-session":
        value["events"][0]["session_id"] = "other-session"
    elif name == "separator-event-canonical":
        return value
    elif name == "reconstructed-pcr-mismatch":
        value["observed_pcr_sha256"] = "88" * 32
    else:
        raise KeyError(name)
    return value


def validate(stream: dict[str, Any]) -> tuple[str, str]:
    if "initial_pcr_sha256" in stream and stream["initial_pcr_sha256"] != "00" * 32:
        return "DENY", "invalid-initial-pcr"
    expected_session = stream.get("session_id")
    if expected_session is not None:
        for event in stream.get("events", []):
            if event.get("session_id", expected_session) != expected_session:
                return "DENY", "cross-session-event"
    return reconstruct(stream)[:2]


def self_test(contract: dict[str, Any]) -> int:
    base = fixture()
    failures: list[str] = []
    for vector in contract["vectors"]:
        state, reason = validate(mutate(base, vector["mutation"]))
        ok = state == vector["expected"]
        print(f'{"[PASS]" if ok else "[FAIL]"} {vector["id"]}: expected={vector["expected"]} got={state} reason={reason}')
        if not ok:
            failures.append(vector["id"])
    print()
    print(f"PC-client event-log reconstruction qualification: {len(contract['vectors']) - len(failures)}/{len(contract['vectors'])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--reconstruct", metavar="EVENT_STREAM")
    args = parser.parse_args()
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    if args.self_test:
        return self_test(contract)
    stream = json.loads(Path(args.reconstruct).read_text(encoding="utf-8"))
    state, reason, reconstructed = reconstruct(stream)
    print(f"Reconstruction: {state} ({reason})")
    if reconstructed:
        print(f"reconstructed_pcr_sha256={reconstructed}")
    return 0 if state == "PASS" else (2 if state == "INDETERMINATE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
