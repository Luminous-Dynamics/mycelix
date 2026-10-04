#!/usr/bin/env python3
"""Composition test: tpm2_eventlog YAML adapter -> independent PCR reconstruction."""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

SECURITY = Path(__file__).resolve().parent
ADAPTER = SECURITY / "adapt_mycelix_tpm2_eventlog_yaml_v1_v0_1.py"
RECON = SECURITY / "reconstruct_mycelix_pc_client_eventlog_v0_1.py"


def run(script: Path, args: list[str]) -> dict:
    proc = subprocess.run(
        [sys.executable, str(script), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode not in (0, 1, 2):
        raise RuntimeError(proc.stdout + proc.stderr)
    return {"returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}


def pcr_extend(state: bytes, digest_hex: str) -> bytes:
    return hashlib.sha256(state + bytes.fromhex(digest_hex)).digest()


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="mycelix-adapter-replay-") as td:
        root = Path(td)
        binary = root / "eventlog.bin"
        yaml = root / "eventlog.yaml"
        observed = root / "observed-pcr-values.json"
        canonical = root / "eventlog-reconstruction-input.json"
        result = root / "eventlog-reconstruction.json"

        binary.write_bytes(b"real-binary-placeholder-for-composition-test-v1")
        d11, d22, zeros = "11" * 32, "22" * 32, "0" * 64
        pcr4 = pcr_extend(pcr_extend(bytes(32), d11), d22).hex()
        observed.write_text(
            json.dumps({"0": zeros, "4": pcr4}, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        yaml.write_text(
            f"""---
version: 1
events:
- EventNum: 0
  PCRIndex: 0
  EventType: EV_NO_ACTION
  Digest: "{zeros}"
  EventSize: 33
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
0 : 0x{zeros}
4 : 0x{pcr4}
""",
            encoding="utf-8",
        )

        adapted = run(
            ADAPTER,
            [
                "--adapt", str(yaml),
                "--binary-eventlog", str(binary),
                "--observed-pcr-json", str(observed),
                "--session-id", "composition-self-test",
                "--pcr-selection", "sha256:0,4",
                "--output", str(canonical),
            ],
        )
        if adapted["returncode"] != 0:
            print("adapter -> canonical input: FAIL")
            return 1

        reconstructed = run(
            RECON,
            ["--reconstruct", str(canonical), "--output", str(result)],
        )
        receipt = json.loads(result.read_text(encoding="utf-8"))
        if reconstructed["returncode"] != 0 or receipt["reconstruction_status"] != "PASS":
            print("canonical input -> independent reconstruction: FAIL")
            return 1

        forged_observed = {"0": zeros, "4": "ff" * 32}
        observed.write_text(json.dumps(forged_observed, sort_keys=True) + "\n", encoding="utf-8")
        forged_input = root / "forged-input.json"
        adapted_forged = run(
            ADAPTER,
            [
                "--adapt", str(yaml),
                "--binary-eventlog", str(binary),
                "--observed-pcr-json", str(observed),
                "--session-id", "composition-self-test",
                "--pcr-selection", "sha256:0,4",
                "--output", str(forged_input),
            ],
        )
        if adapted_forged["returncode"] != 0:
            print("forged observed map adapter path: FAIL")
            return 1
        forged_result = root / "forged-result.json"
        forged_replay = run(
            RECON,
            ["--reconstruct", str(forged_input), "--output", str(forged_result)],
        )
        forged_receipt = json.loads(forged_result.read_text(encoding="utf-8"))
        if forged_replay["returncode"] != 1 or forged_receipt["reconstruction_status"] != "DENY":
            print("forged observed map rejection: FAIL")
            return 1

        startup_yaml = yaml.read_text(encoding="utf-8").replace(
            "EventType: EV_EFI_BOOT_SERVICES_APPLICATION",
            "EventType: EV_NO_ACTION",
            1,
        ).replace('  Event: "fixture"', '  Event: "537461727475704c6f63616c6974790003"', 1)
        startup_yaml_path = root / "startup.yaml"
        startup_yaml_path.write_text(startup_yaml, encoding="utf-8")
        startup_pcr0 = (bytes(31) + b"\x03").hex()
        startup_observed = root / "startup-observed.json"
        startup_observed.write_text(
            json.dumps({"0": startup_pcr0, "4": pcr4}, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        startup_input = root / "startup-input.json"
        startup_adapt = run(
            ADAPTER,
            [
                "--adapt", str(startup_yaml_path),
                "--binary-eventlog", str(binary),
                "--observed-pcr-json", str(startup_observed),
                "--session-id", "composition-startup",
                "--pcr-selection", "sha256:0,4",
                "--output", str(startup_input),
            ],
        )
        startup_result = root / "startup-result.json"
        startup_replay = run(
            RECON,
            ["--reconstruct", str(startup_input), "--output", str(startup_result)],
        )
        startup_receipt = json.loads(startup_result.read_text(encoding="utf-8"))
        if startup_adapt["returncode"] != 0 or startup_replay["returncode"] != 0 or startup_receipt["reconstruction_status"] != "PASS":
            print("StartupLocality adapter -> replay composition: FAIL")
            return 1

        print("adapter -> reconstruction composition: PASS")
        print("independent observed-PCR provenance: PASS")
        print("forged observed-PCR rejection: PASS")
        print("StartupLocality composition: PASS")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
