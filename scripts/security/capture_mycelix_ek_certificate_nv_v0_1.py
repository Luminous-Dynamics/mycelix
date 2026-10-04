#!/usr/bin/env python3
"""Capture TCG EK certificates directly from their TPM NV indices."""
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

# TCG-defined default-template EK certificate NV indices used by tpm2-tools.
EK_CERTIFICATE_HANDLES = {
    0x01C00002: "rsa-legacy-2048",
    0x01C0000A: "ecc-legacy-nist-p256",
    0x01C00012: "rsa-2048",
    0x01C00014: "ecc-nist-p256",
    0x01C00016: "ecc-nist-p384",
    0x01C00018: "ecc-nist-p521",
    0x01C0001A: "ecc-sm2-p256",
    0x01C0001C: "rsa-3072",
    0x01C0001E: "rsa-4096",
}

def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()

def parse_handles(text: str) -> list[int]:
    return sorted({int(raw, 16) for raw in re.findall(r"0x[0-9a-fA-F]+", text)})

def classify_handles(handles: list[int]) -> dict[str, list[dict[str, str]]]:
    candidates: list[dict[str, str]] = []
    other: list[dict[str, str]] = []
    for handle in handles:
        entry = {"handle": f"0x{handle:08x}"}
        if handle in EK_CERTIFICATE_HANDLES:
            entry["profile_id"] = EK_CERTIFICATE_HANDLES[handle]
            candidates.append(entry)
        else:
            other.append(entry)
    return {
        "candidate_ek_certificate_handles": candidates,
        "other_nv_handles": other,
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
    handles = parse_handles(
        "\n".join(
            [
                "0x01c00002",
                "0x01c00004",
                "0x01c00012",
                "0x01c00014",
                "0x01c00020",
                "0x01000000",
            ]
        )
    )
    classified = classify_handles(handles)
    candidate_ids = {item["handle"] for item in classified["candidate_ek_certificate_handles"]}
    if candidate_ids != {"0x01c00002", "0x01c00012", "0x01c00014"}:
        print("exact TCG EK certificate handle registry: FAIL")
        return 1
    if any(item["handle"] == "0x01c00020" for item in classified["candidate_ek_certificate_handles"]):
        print("reserved automotive handle misclassified as EK certificate: FAIL")
        return 1
    print("EK NV certificate capture semantic corpus: PASS")
    print("exact TCG/tpm2-tools certificate handle registry: PASS")
    return 0

def capture(out: Path, env: dict[str, str]) -> int:
    out.mkdir(parents=True, exist_ok=True)
    inventory_path = out / "ek-nv-index-handles.txt"
    transcript_path = out / "ek-certificate-capture-transcript.json"
    result_path = out / "ek-certificate-capture.json"

    if shutil.which("tpm2_getcap") is None or shutil.which("tpm2_nvread") is None:
        result_value = result("INDETERMINATE", "required-tpm2-tools-unavailable")
        result_path.write_text(
            json.dumps(result_value, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return 2

    inventory = run(["tpm2_getcap", "handles-nv-index"], env, out)
    inventory_path.write_text(inventory.stdout + inventory.stderr, encoding="utf-8")
    handles = parse_handles(inventory_path.read_text(encoding="utf-8"))
    classified = classify_handles(handles)

    artifacts: dict[str, dict[str, Any]] = {}
    commands: list[dict[str, Any]] = []
    rsa_candidates = [
        (handle, label)
        for handle, label in EK_CERTIFICATE_HANDLES.items()
        if label.startswith("rsa-") and handle in handles
    ]
    ecc_candidates = [
        (handle, label)
        for handle, label in EK_CERTIFICATE_HANDLES.items()
        if label.startswith("ecc-") and handle in handles
    ]

    for handle, label in rsa_candidates + ecc_candidates:
        output = out / f"ek-cert-{label}.der"
        command = ["tpm2_nvread", f"{handle}", "-o", str(output)]
        proc = run(command, env, out)
        entry = {
            "handle": f"0x{handle:08x}",
            "profile_id": label,
            "command": command,
            "returncode": proc.returncode,
            "stdout_sha256": hashlib.sha256(proc.stdout.encode()).hexdigest(),
            "stderr_sha256": hashlib.sha256(proc.stderr.encode()).hexdigest(),
            "present": output.is_file(),
            "sha256": sha256_file(output) if output.is_file() else None,
            "size": output.stat().st_size if output.is_file() else 0,
        }
        commands.append(entry)
        artifacts[label] = {
            "handle": entry["handle"],
            "present": entry["present"],
            "sha256": entry["sha256"],
            "size": entry["size"],
        }

    rsa2048 = artifacts.get("rsa-2048") or artifacts.get("rsa-legacy-2048")
    selected_rsa_path = None
    if rsa2048 and rsa2048["present"]:
        selected_rsa_path = out / {
            "0x01c00012": "ek-cert-rsa-2048.der",
            "0x01c00002": "ek-cert-rsa-legacy-2048.der",
        }.get(rsa2048["handle"], "")
        if selected_rsa_path and selected_rsa_path.is_file():
            shutil.copy2(selected_rsa_path, out / "ek-cert-rsa.der")
    transcript = {
        "inventory_command": ["tpm2_getcap", "handles-nv-index"],
        "inventory_returncode": inventory.returncode,
        "inventory_stdout_sha256": hashlib.sha256(inventory.stdout.encode()).hexdigest(),
        "inventory_stderr_sha256": hashlib.sha256(inventory.stderr.encode()).hexdigest(),
        "certificate_reads": commands,
        "source_policy": {
            "mode": "TPM_NV_ONLY",
            "network_access": False,
            "network_url_present": False,
            "web_certificate_option_present": False,
            "offline_remote_lookup": False,
            "raw_bytes_preserved": True,
        },
    }
    transcript_path.write_text(
        json.dumps(transcript, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    if rsa2048 and rsa2048["present"]:
        state = "PASS"
        reason = "rsa-ek-certificate-read-directly-from-tcg-nv-index"
    elif classified["candidate_ek_certificate_handles"]:
        state = "INDETERMINATE"
        reason = "candidate-ek-certificate-nv-index-present-but-rsa-certificate-unavailable"
    else:
        state = "INDETERMINATE"
        reason = "no-tcg-ek-certificate-nv-index-observed"

    return_value = result(
        state,
        reason,
        {
            "source_policy": transcript["source_policy"],
            "inventory_sha256": sha256_file(inventory_path),
            "transcript_sha256": sha256_file(transcript_path),
            "candidate_handles": classified["candidate_ek_certificate_handles"],
            "artifacts": artifacts,
            "rsa_certificate": rsa2048,
            "rsa_certificate_present": bool(rsa2048 and rsa2048["present"]),
            "rsa_certificate_sha256": sha256_file(out / "ek-cert-rsa.der") if (out / "ek-cert-rsa.der").is_file() else None,
        },
    )
    result_path.write_text(
        json.dumps(return_value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
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
    return capture(Path(args.output).resolve(), os.environ.copy())

if __name__ == "__main__":
    raise SystemExit(main())
