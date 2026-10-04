#!/usr/bin/env python3
"""Executable coherence verifier/capture helper for Mycelix platform Evidence v0.1.

This module never upgrades missing or ambiguous platform observations to PASS.
It has three intentionally separate modes:
  --self-test : deterministic semantic corpus, no hardware claim
  --verify    : verify an existing capture bundle
  --capture   : collect raw physical TPM/PC-client artifacts when available

The capture path is conservative: an actual platform claim requires an
independently produced event-log reconstruction result and trusted-time input.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import secrets
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-platform-evidence-capture-v0.1.json"


def decision(state: str, reason: str) -> tuple[str, str]:
    return state, reason


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_json_hash(value: Any) -> str:
    data = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    return sha256_bytes(data)


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def run(argv: Sequence[str], env: dict[str, str], cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(argv),
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def mutate_fixture(base: dict[str, Any], mutation: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    if mutation == "canonical-valid":
        return value
    if mutation == "session-id-substitution":
        value["session_id"] = "session-attacker"
    elif mutation == "boot-id-substitution":
        value["boot_id"] = "boot-attacker"
    elif mutation == "device-path-substitution":
        value["tpm"]["device_path"] = "/dev/tpm0"
    elif mutation == "tpm-properties-substitution":
        value["tpm"]["properties_sha256"] = "0" * 64
    elif mutation == "event-log-digest-substitution":
        value["event_log"]["sha256"] = "1" * 64
        value["reconstruction"]["event_log_sha256"] = "1" * 64
    elif mutation == "parser-profile-substitution":
        value["event_log"]["parser_profile_id"] = "other-profile"
    elif mutation == "pcr-selection-substitution":
        value["quote"]["pcr_selection"] = "sha256:0,2,4"
    elif mutation == "nonce-substitution":
        value["quote"]["nonce_sha256"] = "2" * 64
    elif mutation == "attestation-key-substitution":
        value["quote"]["attestation_key_sha256"] = "3" * 64
    elif mutation == "toolchain-substitution":
        value["toolchain"]["tpm2_tools_version"] = "0.0.0-attacker"
    elif mutation == "reference-version-rollback":
        value["reference_values"]["version"] = "0.0.0"
    elif mutation == "reconstruction-event-log-substitution":
        value["reconstruction"]["event_log_sha256"] = "4" * 64
    elif mutation == "reconstruction-pcr-selection-substitution":
        value["reconstruction"]["pcr_selection"] = "sha256:7"
    elif mutation == "reconstruction-result-substitution":
        value["reconstruction"]["result_sha256"] = "5" * 64
    elif mutation == "reconstruction-failure":
        value["reconstruction"]["status"] = "FAIL"
    elif mutation == "reconstruction-unavailable":
        value["reconstruction"]["status"] = "INDETERMINATE"
    elif mutation == "trusted-time-unavailable":
        value["trusted_time"]["available"] = False
    elif mutation == "key-order-permutation":
        value = dict(reversed(list(value.items())))
        for key in ("tpm", "event_log", "quote", "toolchain", "reference_values", "trusted_time", "reconstruction"):
            value[key] = dict(reversed(list(value[key].items())))
    elif mutation == "post-quote-pcr-mismatch":
        value["live_observation"]["pcr_post_sha256"] = "6" * 64
    else:
        raise KeyError(mutation)
    return value


def validate_semantics(manifest: dict[str, Any], bundle: Path | None = None) -> tuple[str, str]:
    required_top = (
        "profile_id",
        "profile_version",
        "session_id",
        "boot_id",
        "tpm",
        "event_log",
        "quote",
        "toolchain",
        "reference_values",
        "trusted_time",
        "reconstruction",
        "live_observation",
        "claim_ceiling",
    )
    for field in required_top:
        if field not in manifest:
            return decision("DENY", f"missing-{field}")

    if manifest["profile_id"] != "mycelix.security.platform.evidence.capture":
        return decision("DENY", "profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return decision("DENY", "profile-version-mismatch")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return decision("DENY", "claim-ceiling-mismatch")

    for section, fields in {
        "tpm": ("device_path", "properties_sha256", "identity_digest"),
        "event_log": ("sha256", "parser_profile_id", "parser_profile_version"),
        "quote": ("pcr_selection", "nonce_sha256", "attestation_key_sha256"),
        "toolchain": ("tpm2_tools_version", "tss_version_evidence_sha256"),
        "reference_values": ("version", "sha256"),
        "trusted_time": ("sha256", "available", "policy_trusted", "local_clock_only"),
        "reconstruction": ("status", "event_log_sha256", "pcr_selection", "result_sha256"),
        "live_observation": ("pcr_post_sha256", "selection"),
    }.items():
        value = manifest.get(section)
        if not isinstance(value, dict):
            return decision("DENY", f"{section}-not-object")
        for field in fields:
            if field not in value:
                return decision("DENY", f"missing-{section}.{field}")

    if not manifest["tpm"]["identity_digest"]:
        return decision("DENY", "empty-tpm-identity")
    if not manifest["session_id"] or not manifest["boot_id"]:
        return decision("DENY", "empty-session-or-boot-id")

    if manifest["event_log"]["parser_profile_id"] != "tcg.pc-client.event-log":
        return decision("DENY", "event-log-parser-profile-mismatch")
    if manifest["event_log"]["parser_profile_version"] != "1.0":
        return decision("DENY", "event-log-parser-version-mismatch")

    if manifest["quote"]["pcr_selection"] != manifest["reconstruction"]["pcr_selection"]:
        return decision("DENY", "quote-reconstruction-pcr-selection-mismatch")
    if manifest["quote"]["pcr_selection"] != manifest["live_observation"]["selection"]:
        return decision("DENY", "quote-live-pcr-selection-mismatch")

    if manifest["event_log"]["sha256"] != manifest["reconstruction"]["event_log_sha256"]:
        return decision("DENY", "event-log-reconstruction-digest-mismatch")

    if manifest["quote"]["pcr_selection"] != "sha256:0,2,4,7":
        return decision("DENY", "unexpected-qualified-pcr-selection")

    if manifest["toolchain"]["tpm2_tools_version"] != "5.8":
        return decision("DENY", "tpm2-tools-version-mismatch")

    if manifest["reference_values"]["version"] in {"0.0.0", "", "unknown"}:
        return decision("DENY", "reference-version-rollback-or-empty")

    if not manifest["trusted_time"]["available"]:
        return decision("INDETERMINATE", "trusted-time-unavailable")
    if not manifest["trusted_time"]["local_clock_only"] and not manifest["trusted_time"]["policy_trusted"]:
        return decision("INDETERMINATE", "trusted-time-not-policy-trusted")
    if manifest["trusted_time"]["local_clock_only"]:
        return decision("DENY", "local-clock-is-not-authoritative")

    reconstruction_status = manifest["reconstruction"]["status"]
    if reconstruction_status == "FAIL":
        return decision("DENY", "event-log-reconstruction-failed")
    if reconstruction_status != "PASS":
        return decision("INDETERMINATE", "event-log-reconstruction-unavailable")

    if bundle is not None:
        expected_files = {
            "tpm-properties.txt": manifest["tpm"]["properties_sha256"],
            "eventlog.bin": manifest["event_log"]["sha256"],
            "reference-values.json": manifest["reference_values"]["sha256"],
            "trusted-time.json": manifest["trusted_time"]["sha256"],
            "eventlog-reconstruction.json": manifest["reconstruction"]["result_sha256"],
            "quote.msg": manifest.get("artifacts", {}).get("quote_message_sha256"),
            "quote.sig": manifest.get("artifacts", {}).get("quote_signature_sha256"),
            "pcr-post.yaml": manifest["live_observation"]["pcr_post_sha256"],
            "ak.pub": manifest["quote"]["attestation_key_sha256"],
        }
        for rel, expected in expected_files.items():
            if not expected:
                return decision("DENY", f"missing-manifest-digest-{rel}")
            path = bundle / rel
            if not path.is_file():
                return decision("DENY", f"missing-artifact-{rel}")
            if sha256_file(path) != expected:
                return decision("DENY", f"artifact-digest-mismatch-{rel}")

    return decision("PASS", "capture-session-coherent")


def run_quote_check(bundle: Path) -> tuple[str, str]:
    binary = shutil.which("tpm2_checkquote")
    if binary is None:
        return decision("INDETERMINATE", "tpm2_checkquote-unavailable")

    manifest = load_json(bundle / "capture-session.json")
    nonce_path = bundle / "nonce.bin"
    if not nonce_path.is_file():
        return decision("DENY", "missing-nonce-file")
    nonce_sha = sha256_file(nonce_path)
    if nonce_sha != manifest["quote"]["nonce_sha256"]:
        return decision("DENY", "nonce-file-digest-mismatch")

    proc = run(
        [
            binary,
            "-u", str(bundle / "ak.pub"),
            "-m", str(bundle / "quote.msg"),
            "-s", str(bundle / "quote.sig"),
            "-f", str(bundle / "pcr-post.yaml"),
            "-g", "sha256",
            "-q", nonce_path.read_bytes().hex(),
            "-l", manifest["quote"]["pcr_selection"],
        ],
        os.environ.copy(),
        bundle,
        check=False,
    )
    if proc.returncode != 0:
        return decision("DENY", "tpm2-checkquote-rejected")
    return decision("PASS", "tpm2-checkquote-verified")


def self_test(contract: dict[str, Any]) -> int:
    fixture = {
        "profile_id": "mycelix.security.platform.evidence.capture",
        "profile_version": "0.1.0",
        "session_id": "session-20261004-0001",
        "boot_id": "boot-20261004-0001",
        "tpm": {
            "device_path": "/dev/tpmrm0",
            "properties_sha256": "a" * 64,
            "identity_digest": "b" * 64,
        },
        "event_log": {
            "sha256": "c" * 64,
            "parser_profile_id": "tcg.pc-client.event-log",
            "parser_profile_version": "1.0",
        },
        "quote": {
            "pcr_selection": "sha256:0,2,4,7",
            "nonce_sha256": "d" * 64,
            "attestation_key_sha256": "e" * 64,
        },
        "toolchain": {
            "tpm2_tools_version": "5.8",
            "tss_version_evidence_sha256": "f" * 64,
        },
        "reference_values": {
            "version": "pc-client-rim-2026.1",
            "sha256": "1" * 64,
        },
        "trusted_time": {
            "sha256": "2" * 64,
            "available": True,
            "policy_trusted": True,
            "local_clock_only": False,
        },
        "reconstruction": {
            "status": "PASS",
            "event_log_sha256": "c" * 64,
            "pcr_selection": "sha256:0,2,4,7",
            "result_sha256": "3" * 64,
        },
        "live_observation": {
            "pcr_post_sha256": "4" * 64,
            "selection": "sha256:0,2,4,7",
        },
        "artifacts": {
            "quote_message_sha256": "5" * 64,
            "quote_signature_sha256": "6" * 64,
        },
        "claim_ceiling": "ReferenceModelOnly",
    }

    failures: list[str] = []
    for vector in contract["vectors"]:
        manifest = mutate_fixture(fixture, vector["mutation"])
        got, reason = validate_semantics(manifest)
        ok = got == vector["expected"]
        marker = "PASS" if ok else "FAIL"
        line = f'[{marker}] {vector["id"]}: expected={vector["expected"]} got={got} reason={reason}'
        print(line)
        if not ok:
            failures.append(line)

    print()
    print(f"Platform capture coherence qualification: {len(contract['vectors']) - len(failures)}/{len(contract['vectors'])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


def detect_device() -> Path | None:
    for candidate in (Path("/dev/tpmrm0"), Path("/dev/tpm0")):
        if candidate.exists():
            return candidate
    return None


def detect_event_log() -> Path | None:
    candidates = (
        Path("/sys/kernel/security/tpm0/binary_bios_measurements"),
        Path("/sys/kernel/security/tpm1/binary_bios_measurements"),
    )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.R_OK):
            return candidate
    return None


def require_capture_tools() -> list[str]:
    names = (
        "tpm2_getcap",
        "tpm2_pcrread",
        "tpm2_quote",
        "tpm2_createek",
        "tpm2_createak",
        "tpm2_checkquote",
        "tpm2_eventlog",
    )
    return [name for name in names if shutil.which(name) is None]


def capture(args: argparse.Namespace) -> int:
    missing = require_capture_tools()
    device = detect_device()
    event_log = detect_event_log()

    blockers: list[str] = []
    if missing:
        blockers.append("missing tools: " + ", ".join(missing))
    if device is None:
        blockers.append("no /dev/tpmrm* or /dev/tpm* device")
    if event_log is None:
        blockers.append("no readable PC-client binary event log")

    for required in (args.reference_values, args.trusted_time, args.nonce_file):
        if not required or not Path(required).is_file():
            blockers.append(f"required external input unavailable: {required or '<not supplied>'}")

    if blockers:
        print("TPM PLATFORM CAPTURE: NOT EXECUTED")
        for blocker in blockers:
            print("- " + blocker)
        return 2

    out = Path(args.output).resolve() if args.output else Path(tempfile.mkdtemp(prefix="mycelix-platform-capture-"))
    out.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["TPM2TOOLS_TCTI"] = f"device:{device}"

    # The capture process is intentionally single-session and creates all
    # temporary key contexts inside the output directory.
    (out / "nonce.bin").write_bytes(Path(args.nonce_file).read_bytes())
    shutil.copy2(event_log, out / "eventlog.bin")
    shutil.copy2(args.reference_values, out / "reference-values.json")
    shutil.copy2(args.trusted_time, out / "trusted-time.json")

    props = run(["tpm2_getcap", "properties-fixed"], env, out)
    (out / "tpm-properties.txt").write_text(props.stdout, encoding="utf-8")

    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    session_id = f"linuxboot-{boot_id}-{sha256_file(out / 'eventlog.bin')[:16]}"

    selection = args.pcr_selection
    pre = run(["tpm2_pcrread", selection], env, out)
    (out / "pcr-pre.yaml").write_text(pre.stdout, encoding="utf-8")

    run([
        "tpm2_createek", "-Q", "-c", str(out / "ek.ctx"), "-G", "rsa",
        "-u", str(out / "ek.pub")
    ], env, out)
    run([
        "tpm2_createak", "-Q", "-C", str(out / "ek.ctx"),
        "-c", str(out / "ak.ctx"), "-G", "rsa", "-g", "sha256",
        "-s", "rsassa", "-u", str(out / "ak.pub"), "-n", str(out / "ak.name")
    ], env, out)

    nonce = (out / "nonce.bin").read_bytes()
    run([
        "tpm2_quote", "-Q", "-c", str(out / "ak.ctx"),
        "-l", selection, "-q", nonce.hex(),
        "-m", str(out / "quote.msg"), "-s", str(out / "quote.sig"),
        "-g", "sha256"
    ], env, out)

    post = run(["tpm2_pcrread", selection], env, out)
    (out / "pcr-post.yaml").write_text(post.stdout, encoding="utf-8")

    # Parsing is captured as a separate artifact. It is not declared to be
    # reconstruction merely because the parser accepted the binary log.
    parsed = run(["tpm2_eventlog", str(out / "eventlog.bin")], env, out, check=False)
    (out / "eventlog-parsed.yaml").write_text(parsed.stdout + parsed.stderr, encoding="utf-8")

    trusted = load_json(out / "trusted-time.json")
    if trusted.get("local_clock_only") or not trusted.get("policy_trusted", False):
        time_state = "INDETERMINATE"
    else:
        time_state = "PASS"

    reconstruction = {
        "status": "INDETERMINATE",
        "reason": "independent-event-log-reconstruction-result-not-yet-supplied",
        "event_log_sha256": sha256_file(out / "eventlog.bin"),
        "pcr_selection": selection,
        "verifier_id": "external-reconstruction-required",
        "result_sha256": "",
    }
    (out / "eventlog-reconstruction.json").write_text(
        json.dumps(reconstruction, indent=2) + "\n", encoding="utf-8"
    )

    manifest = {
        "profile_id": "mycelix.security.platform.evidence.capture",
        "profile_version": "0.1.0",
        "session_id": session_id,
        "boot_id": boot_id,
        "tpm": {
            "device_path": str(device),
            "properties_sha256": sha256_file(out / "tpm-properties.txt"),
            "identity_digest": canonical_json_hash({
                "device_path": str(device),
                "properties_sha256": sha256_file(out / "tpm-properties.txt"),
            }),
        },
        "event_log": {
            "sha256": sha256_file(out / "eventlog.bin"),
            "parser_profile_id": "tcg.pc-client.event-log",
            "parser_profile_version": "1.0",
        },
        "quote": {
            "pcr_selection": selection,
            "nonce_sha256": sha256_file(out / "nonce.bin"),
            "attestation_key_sha256": sha256_file(out / "ak.pub"),
        },
        "toolchain": {
            "tpm2_tools_version": args.tpm2_tools_version,
            "tss_version_evidence_sha256": sha256_bytes(args.tss_version_evidence.encode()),
        },
        "reference_values": {
            "version": load_json(out / "reference-values.json").get("version", "unknown"),
            "sha256": sha256_file(out / "reference-values.json"),
        },
        "trusted_time": {
            "sha256": sha256_file(out / "trusted-time.json"),
            "available": bool(trusted.get("available", False)),
            "policy_trusted": bool(trusted.get("policy_trusted", False)),
            "local_clock_only": bool(trusted.get("local_clock_only", False)),
        },
        "reconstruction": reconstruction,
        "live_observation": {
            "pcr_post_sha256": sha256_file(out / "pcr-post.yaml"),
            "selection": selection,
        },
        "artifacts": {
            "quote_message_sha256": sha256_file(out / "quote.msg"),
            "quote_signature_sha256": sha256_file(out / "quote.sig"),
        },
        "capture_status": "CAPTURED_RAW_EVIDENCE",
        "trusted_time_capture_state": time_state,
        "claim_ceiling": "ReferenceModelOnly",
    }
    (out / "capture-session.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    print(f"Raw platform Evidence captured to: {out}")
    print("Capture status: CAPTURED_RAW_EVIDENCE")
    print("Qualification status: NOT QUALIFIED")
    print("Event-log reconstruction: INDETERMINATE until independent result is supplied")
    print("Claim ceiling: ReferenceModelOnly")
    return 0


def verify(args: argparse.Namespace) -> int:
    bundle = Path(args.bundle).resolve()
    manifest_path = bundle / "capture-session.json"
    if not manifest_path.is_file():
        print("PLATFORM EVIDENCE: DENY: missing capture-session.json")
        return 1

    manifest = load_json(manifest_path)
    semantic_state, semantic_reason = validate_semantics(manifest, bundle)
    print(f"Semantic coherence: {semantic_state} ({semantic_reason})")

    if semantic_state != "PASS":
        return 2 if semantic_state == "INDETERMINATE" else 1

    quote_state, quote_reason = run_quote_check(bundle)
    print(f"TPM Quote verification: {quote_state} ({quote_reason})")
    if quote_state != "PASS":
        return 2 if quote_state == "INDETERMINATE" else 1

    print("PLATFORM EVIDENCE: PASS")
    print("Claim ceiling: ReferenceModelOnly")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--verify", dest="bundle")
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--output")
    parser.add_argument("--reference-values")
    parser.add_argument("--trusted-time")
    parser.add_argument("--nonce-file")
    parser.add_argument("--os-image-digest")
    parser.add_argument("--workload-digest")
    parser.add_argument("--pcr-selection", default="sha256:0,2,4,7")
    parser.add_argument("--tpm2-tools-version", default="5.8")
    parser.add_argument("--tss-version-evidence", default="external-package-build-provenance")
    args = parser.parse_args()

    contract = load_json(CONTRACT)

    modes = int(args.self_test) + int(args.bundle is not None) + int(args.capture)
    if modes != 1:
        parser.error("select exactly one of --self-test, --verify BUNDLE, or --capture")

    if args.self_test:
        return self_test(contract)
    if args.bundle is not None:
        return verify(args)
    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
