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
RECONSTRUCTION_VERIFIER_ID = "mycelix.pc-client.eventlog-reconstruction.v0.1"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_hash(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    return hashlib.sha256(raw).hexdigest()


def valid_digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def selection_ids(selection: str) -> list[int]:
    bank, sep, rest = selection.partition(":")
    if bank != "sha256" or not sep or not rest:
        raise ValueError("unsupported PCR selection")
    if any(not item.isdigit() for item in rest.split(",")):
        raise ValueError("invalid PCR selection")
    ids = [int(item) for item in rest.split(",")]
    if not ids or len(ids) != len(set(ids)):
        raise ValueError("invalid or duplicate PCR selection")
    return ids


def state_hash(values: dict[str, str]) -> str:
    return canonical_hash(
        {"bank": "sha256", "values": {key: values[key] for key in sorted(values, key=int)}}
    )


def reconstruct(stream: dict[str, Any]) -> tuple[str, str, dict[str, str] | None]:
    required = {
        "profile_id", "profile_version", "event_log_sha256", "session_id",
        "pcr_bank", "pcr_selection", "events", "observed_pcr_values",
    }
    missing = required - set(stream)
    if missing:
        return "DENY", "missing-" + ",".join(sorted(missing)), None
    if stream["profile_id"] != "mycelix.security.platform.eventlog.reconstruction":
        return "DENY", "profile-id-mismatch", None
    if stream["profile_version"] != "0.1.0":
        return "DENY", "profile-version-mismatch", None
    if stream["pcr_bank"] != "sha256":
        return "DENY", "wrong-bank", None
    if not valid_digest(stream["event_log_sha256"]):
        return "DENY", "invalid-event-log-digest", None

    try:
        ids = selection_ids(stream["pcr_selection"])
    except (TypeError, ValueError) as exc:
        return "DENY", str(exc), None

    events = stream["events"]
    if not isinstance(events, list) or not events:
        return "DENY", "empty-event-log", None

    observed = stream["observed_pcr_values"]
    if not isinstance(observed, dict):
        return "INDETERMINATE", "missing-observed-pcr-map", None

    expected_keys = {str(index) for index in ids}
    if set(observed) != expected_keys:
        return "INDETERMINATE", "observed-pcr-selection-incomplete", None
    if any(not valid_digest(value) for value in observed.values()):
        return "INDETERMINATE", "invalid-observed-pcr-value", None

    states = {str(index): bytes(32) for index in ids}
    previous_sequence = -1
    seen_sequences: set[int] = set()

    for event in events:
        if not isinstance(event, dict):
            return "DENY", "event-not-object", None
        for key in ("sequence", "pcr", "event_type", "digest_sha256"):
            if key not in event:
                return "DENY", "missing-event-" + key, None

        sequence = event["sequence"]
        pcr = event["pcr"]
        if not isinstance(sequence, int) or sequence <= previous_sequence:
            return "DENY", "non-increasing-sequence", None
        if sequence in seen_sequences:
            return "DENY", "duplicate-sequence", None
        previous_sequence = sequence
        seen_sequences.add(sequence)

        if not isinstance(pcr, int) or pcr < 0:
            return "DENY", "invalid-pcr-index", None
        if event.get("digest_algorithm", "sha256") != "sha256":
            return "DENY", "digest-algorithm-substitution", None
        if event["event_type"] == "EV_EFI_HCRTM_EVENT" and pcr == 0:
            return "INDETERMINATE", "hcrtm-initial-state-adjustment-unsupported", None
        if not valid_digest(event["digest_sha256"]):
            return "DENY", "malformed-digest", None
        if event.get("session_id", stream["session_id"]) != stream["session_id"]:
            return "DENY", "cross-session-event", None

        if event["event_type"] == "EV_NO_ACTION":
            continue

        key = str(pcr)
        if key in states:
            states[key] = hashlib.sha256(
                states[key] + bytes.fromhex(event["digest_sha256"])
            ).digest()

    reconstructed = {key: value.hex() for key, value in states.items()}
    if reconstructed != observed:
        return "DENY", "reconstructed-pcr-mismatch", reconstructed
    return "PASS", "pcr-reconstruction-matches-observed", reconstructed


def fixture() -> dict[str, Any]:
    selection = "sha256:0,2,4,7"
    session = "session-20261004-0001"
    events = [
        {"sequence": 1, "pcr": 4, "event_type": "EV_EFI_BOOT_SERVICES_APPLICATION", "digest_sha256": "11" * 32, "session_id": session},
        {"sequence": 2, "pcr": 4, "event_type": "EV_SEPARATOR", "digest_sha256": "22" * 32, "session_id": session},
        {"sequence": 3, "pcr": 7, "event_type": "EV_EFI_VARIABLE_AUTHORITY", "digest_sha256": "33" * 32, "session_id": session},
        {"sequence": 4, "pcr": 0, "event_type": "EV_ACTION", "digest_sha256": "44" * 32, "session_id": session},
        {"sequence": 5, "pcr": 2, "event_type": "EV_ACTION", "digest_sha256": "55" * 32, "session_id": session},
    ]
    states = {str(index): bytes(32) for index in selection_ids(selection)}
    for event in events:
        key = str(event["pcr"])
        states[key] = hashlib.sha256(states[key] + bytes.fromhex(event["digest_sha256"])).digest()
    observed = {key: value.hex() for key, value in states.items()}
    return {
        "profile_id": "mycelix.security.platform.eventlog.reconstruction",
        "profile_version": "0.1.0",
        "event_log_sha256": "aa" * 32,
        "session_id": session,
        "pcr_bank": "sha256",
        "pcr_selection": selection,
        "events": events,
        "observed_pcr_values": observed,
    }


def mutate(base: dict[str, Any], name: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    if name == "canonical-valid":
        return value
    if name == "digest-substitution":
        value["events"][0]["digest_sha256"] = "66" * 32
    elif name == "event-removal":
        value["events"].pop(1)
    elif name == "event-insertion":
        value["events"].insert(
            1,
            {
                "sequence": 6,
                "pcr": 4,
                "event_type": "EV_ACTION",
                "digest_sha256": "77" * 32,
                "session_id": value["session_id"],
            },
        )
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
        value["observed_pcr_values"]["4"] = "88" * 32
    elif name == "missing-expected-pcr":
        del value["observed_pcr_values"]["4"]
    elif name == "empty-event-log":
        value["events"] = []
    elif name == "key-order-permutation":
        value = dict(reversed(list(value.items())))
        value["events"] = [dict(reversed(list(event.items()))) for event in value["events"]]
        value["observed_pcr_values"] = dict(reversed(list(value["observed_pcr_values"].items())))
    elif name == "metadata-only-substitution":
        value["events"][0]["event_type"] = "VENDOR_UNTRUSTED_LABEL"
    elif name == "reconstruction-unavailable":
        value["reconstruction_status"] = "INDETERMINATE"
    elif name == "invalid-initial-pcr":
        value["initial_pcr_sha256"] = "77" * 32
    elif name == "digest-algorithm-substitution":
        value["events"][0]["digest_algorithm"] = "sha1"
    elif name == "target-pcr-missing-from-events":
        for event in value["events"]:
            if event["pcr"] == 4:
                event["pcr"] = 5
    elif name == "multi-pcr-canonical":
        return value
    elif name == "target-event-cross-session":
        value["events"][0]["session_id"] = "other-session"
    elif name == "separator-event-canonical":
        return value
    elif name == "reconstructed-pcr-mismatch":
        value["observed_pcr_values"]["4"] = "99" * 32
    else:
        raise KeyError(name)
    return value


def validate(stream: dict[str, Any]) -> tuple[str, str]:
    if stream.get("initial_pcr_sha256") not in (None, "00" * 32):
        return "DENY", "invalid-initial-pcr"
    if stream.get("reconstruction_status") == "INDETERMINATE":
        return "INDETERMINATE", "reconstruction-unavailable"
    return reconstruct(stream)[:2]


def build_result(stream: dict[str, Any], input_path: Path) -> dict[str, Any]:
    status, reason, reconstructed = reconstruct(stream)
    observed = stream.get("observed_pcr_values")
    reconstructed_hash = state_hash(reconstructed) if reconstructed is not None else None
    observed_hash = state_hash(observed) if isinstance(observed, dict) else None
    result = {
        "profile_id": stream.get("profile_id"),
        "profile_version": stream.get("profile_version"),
        "event_log_sha256": stream.get("event_log_sha256"),
        "session_id": stream.get("session_id"),
        "pcr_bank": stream.get("pcr_bank"),
        "pcr_selection": stream.get("pcr_selection"),
        "event_count": len(stream.get("events", [])),
        "reconstructed_pcr_values": reconstructed,
        "reconstructed_pcrs_sha256": reconstructed_hash,
        "observed_pcr_values": observed,
        "observed_pcrs_sha256": observed_hash,
        "match": status == "PASS",
        "reconstruction_status": status,
        "reason": reason,
        "input_sha256": sha256_file(input_path),
        "verifier_id": RECONSTRUCTION_VERIFIER_ID,
        "verifier_source_sha256": sha256_file(Path(__file__).resolve()),
    }
    result["content_sha256"] = canonical_hash(result)
    return result


def self_test(contract: dict[str, Any]) -> int:
    failures: list[str] = []
    base = fixture()

    for vector in contract["vectors"]:
        state, reason = validate(mutate(base, vector["mutation"]))
        ok = state == vector["expected"]
        print(
            f'{"[PASS]" if ok else "[FAIL]"} {vector["id"]}: '
            f"expected={vector['expected']} got={state} reason={reason}"
        )
        if not ok:
            failures.append(vector["id"])

    print()
    print(
        "PC-client event-log reconstruction qualification: "
        f"{len(contract['vectors']) - len(failures)}/{len(contract['vectors'])} vectors passed"
    )
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--reconstruct", metavar="EVENT_STREAM")
    parser.add_argument("--output", metavar="RESULT_JSON")
    args = parser.parse_args()

    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    if args.self_test:
        return self_test(contract)

    stream = json.loads(Path(args.reconstruct).read_text(encoding="utf-8"))
    result = build_result(stream, Path(args.reconstruct).resolve())
    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    status = result["reconstruction_status"]
    return 0 if status == "PASS" else (2 if status == "INDETERMINATE" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
