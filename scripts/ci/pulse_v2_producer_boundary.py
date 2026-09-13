#!/usr/bin/env python3
"""Fail-closed source invariant for Pulse V2 realtime producer authority.

The durable V2 coordinator may emit no wake yet, or it may emit the shared
information-poor PulseRealtimeHintV1. It must never regress to legacy MailSignal
payloads or put message authority fields onto the realtime transport.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
COORDINATOR = ROOT / "mycelix-workspace/mycelix-pulse/holochain/zomes/messages/coordinator/src/lib.rs"


def fail(message: str) -> None:
    print(f"producer-boundary: FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def extract_function(source: str, name: str) -> str:
    marker = re.search(rf"\bpub\s+fn\s+{re.escape(name)}\s*\(", source)
    if marker is None:
        fail(f"missing function {name}")

    brace = source.find("{", marker.start())
    if brace < 0:
        fail(f"missing body for {name}")

    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[marker.start() : index + 1]

    fail(f"unterminated body for {name}")
    raise AssertionError("unreachable")


def main() -> None:
    source = COORDINATOR.read_text(encoding="utf-8")
    send_v2 = extract_function(source, "send_email_v2")

    forbidden = {
        "MailSignal": "legacy realtime envelope type",
        "EmailReceived": "legacy message-arrival authority shape",
        "DeliveryConfirmed": "legacy delivery authority shape",
        "ReadReceiptReceived": "legacy read-receipt authority shape",
        "encrypted_subject": "message content metadata",
    }
    for token, meaning in forbidden.items():
        if token in send_v2:
            fail(f"send_email_v2 contains {meaning}: {token}")

    has_remote_wake = "send_remote_signal(" in send_v2
    if has_remote_wake:
        required = {
            "PulseRealtimeHintV1::inbox_changed_v2()": "shared minimal V2 wake constructor",
            "ExternIO::encode": "typed Holochain signal encoding",
        }
        for token, meaning in required.items():
            if token not in send_v2:
                fail(f"V2 remote wake is missing {meaning}: {token}")

        if not re.search(r"send_remote_signal\s*\([^,]+,\s*vec!\s*\[\s*input\.recipient(?:\.clone\(\))?\s*\]", send_v2, re.S):
            fail("V2 wake must target only input.recipient")

        best_effort_patterns = (
            "let _ = send_remote_signal",
            "if let Err(",
        )
        if not any(pattern in send_v2 for pattern in best_effort_patterns):
            fail("V2 wake must be best-effort and must not determine durable send success")

        print("producer-boundary: PASS: send_email_v2 emits only the shared minimal V2 wake")
    else:
        print("producer-boundary: PASS: send_email_v2 emits no realtime wake yet")

    recv = extract_function(source, "recv_remote_signal")
    v2_receiver_tokens = ("PulseRealtimeHintV1", "decode_realtime_hint")
    if any(token in recv for token in v2_receiver_tokens):
        if "emit_signal" not in recv:
            fail("V2 receive path recognizes the minimal hint but does not forward a local wake")
        if "MailSignal" in recv and "decode_realtime_hint" not in recv:
            fail("V2 receive path may not reinterpret the minimal hint as legacy MailSignal authority")
        print("producer-boundary: PASS: recv_remote_signal has an explicit V2 hint path")
    else:
        print("producer-boundary: INFO: recv_remote_signal is still legacy-only; implementation tranche remains pending")


if __name__ == "__main__":
    main()
