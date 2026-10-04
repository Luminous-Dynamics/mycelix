#!/usr/bin/env python3
"""Verify event-payload to recorded SHA-256 digest coherence."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
VERIFIER_ID = "mycelix.pc-client.event-payload-digest-coherence.v0.1"

# These are the event types for which current tpm2-tools directly hashes event->Event
# and compares the result against each recorded digest.
VERIFIABLE_EVENT_TYPES = {
    "EV_S_CRTM_VERSION",
    "EV_SEPARATOR",
    "EV_EFI_VARIABLE_DRIVER_CONFIG",
    "EV_EFI_GPT_EVENT",
}


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_hash(value: Any) -> str:
    raw = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode()
    return hashlib.sha256(raw).hexdigest()


def self_hash(value: dict[str, Any]) -> str:
    clone = dict(value)
    clone.pop("content_sha256", None)
    return canonical_hash(clone)


def valid_digest(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(c in "0123456789abcdef" for c in value)
    )


def verify_event(event: dict[str, Any]) -> dict[str, Any]:
    event_number = event.get("sequence")
    event_type = event.get("event_type")
    digest = event.get("digest_sha256")

    base = {
        "event_number": event_number,
        "event_type": event_type,
        "recorded_sha256": digest,
    }

    if event_type == "EV_NO_ACTION":
        return {
            **base,
            "state": "INDETERMINATE",
            "reason": "non-extending-control-event",
            "recomputed_sha256": None,
        }

    if not valid_digest(digest):
        return {
            **base,
            "state": "DENY",
            "reason": "missing-or-invalid-recorded-sha256",
            "recomputed_sha256": None,
        }

    if event_type not in VERIFIABLE_EVENT_TYPES:
        return {
            **base,
            "state": "INDETERMINATE",
            "reason": "event-type-not-payload-reconstructable-by-profile",
            "recomputed_sha256": None,
        }

    payload_hex = event.get("payload_hex")
    if not isinstance(payload_hex, str):
        return {
            **base,
            "state": "INDETERMINATE",
            "reason": "canonical-payload-bytes-unavailable",
            "recomputed_sha256": None,
        }

    payload_hex = payload_hex.lower().removeprefix("0x")
    if len(payload_hex) % 2 != 0:
        return {
            **base,
            "state": "DENY",
            "reason": "payload-hex-odd-length",
            "recomputed_sha256": None,
        }
    try:
        payload = bytes.fromhex(payload_hex)
    except ValueError:
        return {
            **base,
            "state": "DENY",
            "reason": "payload-hex-invalid",
            "recomputed_sha256": None,
        }

    recomputed = hashlib.sha256(payload).hexdigest()
    state = "PASS" if recomputed == digest else "DENY"
    return {
        **base,
        "state": state,
        "reason": (
            "payload-sha256-matches-recorded-digest"
            if state == "PASS"
            else "payload-sha256-mismatch"
        ),
        "recomputed_sha256": recomputed,
        "payload_sha256": hashlib.sha256(payload).hexdigest(),
        "payload_size": len(payload),
    }


def verify(stream: dict[str, Any], input_path: Path) -> dict[str, Any]:
    required = {
        "profile_id",
        "profile_version",
        "event_log_sha256",
        "session_id",
        "pcr_bank",
        "pcr_selection",
        "events",
    }
    missing = sorted(required - set(stream))
    if missing:
        return {
            "verifier_id": VERIFIER_ID,
            "verifier_source_sha256": sha256_file(Path(__file__).resolve()),
            "state": "DENY",
            "reason": "missing-" + ",".join(missing),
            "events": [],
            "input_sha256": sha256_file(input_path),
        }

    results = []
    for event in stream["events"]:
        if not isinstance(event, dict):
            results.append(
                {
                    "event_number": None,
                    "event_type": None,
                    "state": "DENY",
                    "reason": "event-not-object",
                    "recomputed_sha256": None,
                }
            )
            continue
        results.append(verify_event(event))

    states = {item["state"] for item in results}
    if "DENY" in states:
        overall = "DENY"
        reason = "one-or-more-event-payload-coherence-failures"
    elif "INDETERMINATE" in states:
        overall = "INDETERMINATE"
        reason = "one-or-more-event-payload-coherence-checks-out-of-scope"
    else:
        overall = "PASS"
        reason = "all-payload-verifiable-events-match"

    result = {
        "profile_id": "mycelix.security.event-payload-digest-coherence",
        "profile_version": "0.1.0",
        "verifier_id": VERIFIER_ID,
        "verifier_source_sha256": sha256_file(Path(__file__).resolve()),
        "input_sha256": sha256_file(input_path),
        "event_count": len(results),
        "event_results": results,
        "state": overall,
        "reason": reason,
    }
    result["content_sha256"] = self_hash(result)
    return result


def fixture() -> dict[str, Any]:
    payload = bytes.fromhex("00000000")
    digest = hashlib.sha256(payload).hexdigest()
    return {
        "profile_id": "mycelix.security.platform.eventlog.reconstruction",
        "profile_version": "0.1.0",
        "event_log_sha256": "aa" * 32,
        "session_id": "payload-self-test",
        "pcr_bank": "sha256",
        "pcr_selection": "sha256:0,4",
        "events": [
            {
                "sequence": 1,
                "pcr": 0,
                "event_type": "EV_SEPARATOR",
                "digest_sha256": digest,
                "payload_hex": "00000000",
            }
        ],
    }


def self_test() -> int:
    with __import__("tempfile").TemporaryDirectory(prefix="mycelix-payload-coherence-") as td:
        path = Path(td) / "input.json"
        base = fixture()
        path.write_text(json.dumps(base, sort_keys=True), encoding="utf-8")

        valid = verify(base, path)
        if valid["state"] != "PASS":
            print("Known payload-verifiable event PASS: FAIL")
            return 1

        forged = copy.deepcopy(base)
        forged["events"][0]["digest_sha256"] = "11" * 32
        result = verify(forged, path)
        if result["state"] != "DENY":
            print("Digest substitution DENY: FAIL")
            return 1

        unavailable = copy.deepcopy(base)
        unavailable["events"][0].pop("payload_hex")
        result = verify(unavailable, path)
        if result["state"] != "INDETERMINATE":
            print("Missing canonical payload INDETERMINATE: FAIL")
            return 1

        unknown = copy.deepcopy(base)
        unknown["events"][0]["event_type"] = "EV_EFI_ACTION"
        result = verify(unknown, path)
        if result["state"] != "INDETERMINATE":
            print("Out-of-scope event type INDETERMINATE: FAIL")
            return 1

        control = copy.deepcopy(base)
        control["events"][0]["event_type"] = "EV_NO_ACTION"
        control["events"][0].pop("digest_sha256")
        result = verify(control, path)
        if result["state"] != "INDETERMINATE":
            print("EV_NO_ACTION control INDETERMINATE: FAIL")
            return 1

        malformed = copy.deepcopy(base)
        malformed["events"][0]["payload_hex"] = "abc"
        result = verify(malformed, path)
        if result["state"] != "DENY":
            print("Malformed payload DENY: FAIL")
            return 1

        print("Event payload -> digest coherence self-test: PASS")
        print("PASS: directly verifiable payload")
        print("DENY: digest/payload mismatch")
        print("INDETERMINATE: unavailable or out-of-scope payload")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--verify", metavar="EVENT_STREAM")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    input_path = Path(args.verify).resolve()
    stream = json.loads(input_path.read_text(encoding="utf-8"))
    result = verify(stream, input_path)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS": 0, "INDETERMINATE": 2, "DENY": 1}[result["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
