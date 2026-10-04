#!/usr/bin/env python3
"""Cross-check raw binary extraction, YAML adapter, and payload coherence."""
from __future__ import annotations

import hashlib
import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

SECURITY = Path(__file__).resolve().parent
RAW = SECURITY / "parse_mycelix_raw_tpm2_eventlog_v0_1.py"
ADAPTER = SECURITY / "adapt_mycelix_tpm2_eventlog_yaml_v1_v0_1.py"
PAYLOAD = SECURITY / "verify_mycelix_event_payload_digest_coherence_v0_1.py"


def make_log(payload: bytes) -> bytes:
    signature = b"Spec ID Event03" + b"\x00"
    spec = (
        signature
        + struct.pack("<I", 0)
        + bytes([0, 2, 0, 2])
        + struct.pack("<I", 2)
        + struct.pack("<HH", 0x0004, 20)
        + struct.pack("<HH", 0x000B, 32)
        + bytes([0])
    )
    legacy = (
        struct.pack("<II", 0, 0x00000003)
        + b"\x00" * 20
        + struct.pack("<I", len(spec))
        + spec
    )
    event = (
        struct.pack("<III", 0, 0x00000004, 1)
        + struct.pack("<H", 0x000B)
        + hashlib.sha256(payload).digest()
        + struct.pack("<I", len(payload))
        + payload
    )
    return legacy + event


def call(script: Path, args: list[str]) -> int:
    proc = subprocess.run(
        [sys.executable, str(script), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    return proc.returncode


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="mycelix-raw-yaml-crosscheck-") as td:
        root = Path(td)
        binary = root / "eventlog.bin"
        raw_json = root / "raw.json"
        yaml = root / "eventlog.yaml"
        observed = root / "observed.json"
        canonical = root / "canonical.json"
        result = root / "payload-result.json"

        payload = b"\x00\x00\x00\x00"
        binary.write_bytes(make_log(payload))

        if call(RAW, ["--parse", str(binary), "--output", str(raw_json)]) != 0:
            print("raw parser execution: FAIL")
            return 1
        raw = json.loads(raw_json.read_text(encoding="utf-8"))
        event = raw["events"][1]
        if event["payload_hex"] != payload.hex():
            print("raw payload extraction: FAIL")
            return 1

        pcr = hashlib.sha256(bytes(32) + payload).hexdigest()
        observed.write_text(
            json.dumps({"0": "0" * 64}, sort_keys=True) + "\n", encoding="utf-8"
        )
        # A SEP event extends PCR0, so the actual observed value must include it.
        observed.write_text(
            json.dumps({"0": pcr}, sort_keys=True) + "\n", encoding="utf-8"
        )

        digest = hashlib.sha256(payload).hexdigest()
        yaml.write_text(
            f"""---
version: 1
events:
  - EventNum: 0
    PCRIndex: 0
    EventType: EV_NO_ACTION
    Digest: "{'0' * 64}"
    EventSize: {len(raw["events"][0]["payload_hex"]) // 2}
    SpecID:
      - Signature: Spec ID Event03
  - EventNum: 1
    PCRIndex: 0
    EventType: EV_SEPARATOR
    DigestCount: 1
    Digests:
      - AlgorithmId: sha256
        Digest: "{digest}"
    EventSize: {len(payload)}
    Event: "00000000"
pcrs:
  sha256:
    0 : 0x{pcr}
""",
            encoding="utf-8",
        )

        if call(
            ADAPTER,
            [
                "--adapt", str(yaml),
                "--binary-eventlog", str(binary),
                "--observed-pcr-json", str(observed),
                "--payload-json", str(raw_json),
                "--session-id", "raw-yaml-crosscheck",
                "--pcr-selection", "sha256:0",
                "--output", str(canonical),
            ],
        ) != 0:
            print("YAML adapter/raw cross-check: FAIL")
            return 1

        canon = json.loads(canonical.read_text(encoding="utf-8"))
        if canon["events"][1]["payload_hex"] != payload.hex():
            print("canonical payload propagation: FAIL")
            return 1

        # Payload coherence is INDETERMINATE overall because the SpecID control
        # event is non-extending, but the directly verifiable separator must PASS.
        rc = call(PAYLOAD, ["--verify", str(canonical), "--output", str(result)])
        payload_result = json.loads(result.read_text(encoding="utf-8"))
        separator = next(x for x in payload_result["event_results"] if x["event_number"] == 1)
        if rc != 2 or separator["state"] != "PASS":
            print("payload coherence composition: FAIL")
            return 1

        # Same digest, changed raw payload: the independent byte parser detects
        # the exact altered bytes and payload coherence must reject the result.
        altered = root / "altered.bin"
        altered.write_bytes(make_log(b"\x00\x00\x00\x01"))
        altered_json = root / "altered-raw.json"
        if call(RAW, ["--parse", str(altered), "--output", str(altered_json)]) != 0:
            print("altered raw parse: FAIL")
            return 1
        altered_canonical = root / "altered-canonical.json"
        if call(
            ADAPTER,
            [
                "--adapt", str(yaml),
                "--binary-eventlog", str(altered),
                "--observed-pcr-json", str(observed),
                "--payload-json", str(altered_json),
                "--session-id", "raw-yaml-crosscheck",
                "--pcr-selection", "sha256:0",
                "--output", str(altered_canonical),
            ]
        ) != 0:
            print("altered adapter composition: FAIL")
            return 1
        altered_result = root / "altered-result.json"
        call(PAYLOAD, ["--verify", str(altered_canonical), "--output", str(altered_result)])
        altered_payload = json.loads(altered_result.read_text(encoding="utf-8"))
        separator = next(
            x for x in altered_payload["event_results"] if x["event_number"] == 1
        )
        if altered_payload["state"] != "DENY" or separator["state"] != "DENY":
            print("raw-payload alteration rejection: FAIL")
            return 1

        # YAML identity drift must be rejected before replay/verification.
        drift_yaml = yaml.read_text(encoding="utf-8").replace(
            "EventType: EV_SEPARATOR", "EventType: EV_EFI_ACTION", 1
        )
        drift = root / "drift.yaml"
        drift.write_text(drift_yaml, encoding="utf-8")
        drift_canonical = root / "drift.json"
        if call(
            ADAPTER,
            [
                "--adapt", str(drift),
                "--binary-eventlog", str(binary),
                "--observed-pcr-json", str(observed),
                "--payload-json", str(raw_json),
                "--session-id", "raw-yaml-crosscheck",
                "--pcr-selection", "sha256:0",
                "--output", str(drift_canonical),
            ]
        ) == 0:
            print("YAML/raw identity drift rejection: FAIL")
            return 1

        print("raw binary -> YAML cross-check: PASS")
        print("payload coherence composition: PASS")
        print("raw payload alteration rejection: PASS")
        print("YAML/raw identity drift rejection: PASS")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
