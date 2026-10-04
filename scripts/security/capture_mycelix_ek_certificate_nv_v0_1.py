#!/usr/bin/env python3
"""Capture EK certificates from TPM NV only, without network fallback."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-certificate-nv-capture.v0.1"
TCG_LOW_CERT_HANDLES = {0x01C00002, 0x01C0000A}
TCG_HIGH_CERT_MIN = 0x01C00012
TCG_HIGH_CERT_MAX = 0x01C07FFF

def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()

def parse_handles(text: str) -> list[int]:
    values = {int(raw, 16) for raw in re.findall(r"0x[0-9a-fA-F]+", text)}
    return sorted(values)

def is_ek_certificate_handle(handle: int) -> bool:
    if handle in TCG_LOW_CERT_HANDLES:
        return True
    return TCG_HIGH_CERT_MIN <= handle <= TCG_HIGH_CERT_MAX and handle % 2 == 0

def classify_handles(handles: list[int]) -> dict[str, list[str]]:
    return {
        "candidate_ek_certificate_handles": [
            f"0x{h:08x}" for h in handles if is_ek_certificate_handle(h)
        ],
        "other_nv_handles": [
            f"0x{h:08x}" for h in handles if not is_ek_certificate_handle(h)
        ],
    }

def source_policy(command: list[str]) -> dict[str, Any]:
    return {
        "mode": "TPM_NV_ONLY",
        "network_url_present": any(
            token.startswith(("http://", "https://")) for token in command
        ),
        "explicit_network_option_present": any(
            token in {"-X", "--allow-unverified"} for token in command
        ),
        "offline_option_present": any(
            token in {"-x", "--offline"} for token in command
        ),
        "raw_output_requested": "--raw" in command,
    }

def run(command: list[str], env: dict[str, str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    value: dict[str, Any] = {
        "verifier_id": VERIFIER_ID,
        "state": state,
        "reason": reason,
    }
    if details is not None:
        value["details"] = details
    value["content_sha256"] = canonical_hash(value)
    return value

def self_test() -> int:
    inventory = "
".join([
        "0x01c00002",
        "0x01c00004",
        "0x01c00012",
        "0x01c00014",
        "0x01000000",
    ])
    classified = classify_handles(parse_handles(inventory))
    if "0x01c00002" not in classified["candidate_ek_certificate_handles"]:
        print("low-range certificate classification: FAIL")
        return 1
    if "0x01c00012" not in classified["candidate_ek_certificate_handles"]:
        print("high-range certificate classification: FAIL")
        return 1
    if "0x01c00004" in classified["candidate_ek_certificate_handles"]:
        print("non-certificate handle misclassified: FAIL")
        return 1
    policy = source_policy([
        "tpm2_getekcertificate", "--raw", "-o", "ek-cert-rsa.der",
    ])
    if policy["mode"] != "TPM_NV_ONLY" or policy["network_url_present"] or policy["explicit_network_option_present"]:
        print("NV-only policy: FAIL")
        return 1
    print("EK NV-only certificate capture semantic corpus: PASS")
    return 0

def capture(out: Path, env: dict[str, str]) -> int:
    out.mkdir(parents=True, exist_ok=True)
    inventory_path = out / "ek-nv-index-handles.txt"
    transcript_path = out / "ek-certificate-capture-transcript.json"
    result_path = out / "ek-certificate-capture.json"
    rsa_path = out / "ek-cert-rsa.der"
    ecc_path = out / "ek-cert-ecc.der"
    if shutil.which("tpm2_getcap") is None or shutil.which("tpm2_getekcertificate") is None:
        result_value = result("INDETERMINATE", "required-tpm2-tools-unavailable")
        result_path.write_text(json.dumps(result_value, indent=2, sort_keys=True) + "
", encoding="utf-8")
        return 2

    inv = run(["tpm2_getcap", "handles-nv-index"], env, out)
    inventory_path.write_text(inv.stdout + inv.stderr, encoding="utf-8")
    classified = classify_handles(parse_handles(inventory_path.read_text(encoding="utf-8")))
    for path in (rsa_path, ecc_path):
        if path.exists():
            path.unlink()

    command = [
        "tpm2_getekcertificate",
        "--raw",
        "-o", str(rsa_path),
        "-o", str(ecc_path),
    ]
    proc = run(command, env, out)
    present = {
        kind: {
            "present": path.is_file(),
            "sha256": sha256_file(path) if path.is_file() else None,
            "size": path.stat().st_size if path.is_file() else 0,
        }
        for kind, path in (("rsa", rsa_path), ("ecc", ecc_path))
    }
    transcript = {
        "inventory_command": ["tpm2_getcap", "handles-nv-index"],
        "inventory_returncode": inv.returncode,
        "inventory_stdout_sha256": hashlib.sha256(inv.stdout.encode()).hexdigest(),
        "inventory_stderr_sha256": hashlib.sha256(inv.stderr.encode()).hexdigest(),
        "certificate_command": command,
        "certificate_returncode": proc.returncode,
        "certificate_stdout_sha256": hashlib.sha256(proc.stdout.encode()).hexdigest(),
        "certificate_stderr_sha256": hashlib.sha256(proc.stderr.encode()).hexdigest(),
        "source_policy": source_policy(command),
        "candidate_handles": classified["candidate_ek_certificate_handles"],
    }
    transcript_path.write_text(json.dumps(transcript, indent=2, sort_keys=True) + "
", encoding="utf-8")

    policy = source_policy(command)
    if policy["network_url_present"] or policy["explicit_network_option_present"] or policy["offline_option_present"]:
        state, reason = "DENY", "certificate-source-policy-invalid"
    elif proc.returncode == 0 and any(v["present"] for v in present.values()):
        state, reason = "PASS", "ek-certificate-captured-from-tpm-nv-path"
    elif proc.returncode == 0:
        state, reason = "INDETERMINATE", "no-ek-certificate-artifact-returned"
    elif classified["candidate_ek_certificate_handles"]:
        state, reason = "INDETERMINATE", "ek-certificate-nv-present-but-retrieval-failed"
    else:
        state, reason = "INDETERMINATE", "no-tcg-ek-certificate-nv-index-observed"
    result_value = result(state, reason, {
        "source_policy": policy,
        "inventory_sha256": sha256_file(inventory_path),
        "transcript_sha256": sha256_file(transcript_path),
        "candidate_handles": classified["candidate_ek_certificate_handles"],
        "artifacts": present,
    })
    result_path.write_text(json.dumps(result_value, indent=2, sort_keys=True) + "
", encoding="utf-8")
    return {"PASS": 0, "DENY": 1, "INDETERMINATE": 2}[state]

def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--capture", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.output:
        parser.error("--output is required with --capture")
    env = os.environ.copy()
    return capture(Path(args.output).resolve(), env)

if __name__ == "__main__":
    raise SystemExit(main())
