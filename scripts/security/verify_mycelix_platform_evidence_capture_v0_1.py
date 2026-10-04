#!/usr/bin/env python3
"""Executable platform Evidence capture/coherence verifier for Mycelix v0.1."""
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


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()


def canonical_hash(value: Any) -> str:
    return sha256_bytes(canonical_json_bytes(value))


def self_hash(value: dict[str, Any], field: str) -> str:
    clone = copy.deepcopy(value)
    clone.pop(field, None)
    return canonical_hash(clone)


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def run(argv: Sequence[str], env: dict[str, str], cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(argv), cwd=cwd, env=env, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=check
    )


def session_binding(manifest: dict[str, Any]) -> str:
    payload = {
        "session_id": manifest["session_id"],
        "boot_id": manifest["boot_id"],
        "tpm_identity_digest": manifest["tpm"]["identity_digest"],
        "event_log_sha256": manifest["event_log"]["sha256"],
        "pcr_selection": manifest["quote"]["pcr_selection"],
        "nonce_sha256": manifest["quote"]["nonce_sha256"],
        "attestation_key_sha256": manifest["quote"]["attestation_key_sha256"],
        "tpm2_tools_version": manifest["toolchain"]["tpm2_tools_version"],
        "reference_version": manifest["reference_values"]["version"],
        "reference_sha256": manifest["reference_values"]["sha256"],
        "trusted_time_sha256": manifest["trusted_time"]["sha256"],
        "reconstruction_content_sha256": manifest["reconstruction"]["content_sha256"],
    }
    return canonical_hash(payload)


def mutation_manifest() -> dict[str, Any]:
    reconstruction = {
        "status": "PASS",
        "reason": "independent-reconstruction-fixture",
        "event_log_sha256": "c" * 64,
        "pcr_selection": "sha256:0,2,4,7",
        "verifier_id": "fixture-reconstruction-v1",
        "reconstructed_pcr_sha256": "4" * 64,
    }
    reconstruction["content_sha256"] = self_hash(reconstruction, "content_sha256")
    value = {
        "profile_id": "mycelix.security.platform.evidence.capture",
        "profile_version": "0.1.0",
        "session_id": "session-20261004-0001",
        "boot_id": "boot-20261004-0001",
        "tpm": {
            "device_path": "/dev/tpmrm0",
            "properties_sha256": "a" * 64,
            "identity_digest": "",
        },
        "event_log": {
            "sha256": "c" * 64,
            "parser_profile_id": "tcg.pc-client.event-log",
            "parser_profile_version": "1.0",
            "parser_output_sha256": "7" * 64,
        },
        "quote": {
            "pcr_selection": "sha256:0,2,4,7",
            "nonce_sha256": "d" * 64,
            "attestation_key_sha256": "e" * 64,
            "quoted_pcr_sha256": "4" * 64,
        },
        "challenge": {"sha256": "d" * 64},
        "toolchain": {
            "tpm2_tools_version": "5.8",
            "tss_version_evidence_sha256": "f" * 64,
        },
        "reference_values": {"version": "pc-client-rim-2026.1", "sha256": "1" * 64},
        "trusted_time": {
            "sha256": "2" * 64,
            "available": True,
            "policy_trusted": True,
            "local_clock_only": False,
        },
        "reconstruction": reconstruction,
        "live_observation": {
            "pcr_post_sha256": "4" * 64,
            "selection": "sha256:0,2,4,7",
        },
        "artifacts": {
            "quote_message_sha256": "5" * 64,
            "quote_signature_sha256": "6" * 64,
            "attestation_key_sha256": "e" * 64,
            "reconstruction_file_sha256": "8" * 64,
        },
        "os_image_digest": "sha256:" + "9" * 64,
        "workload_digest": "sha256:" + "a" * 64,
        "claim_ceiling": "ReferenceModelOnly",
    }
    value["tpm"]["identity_digest"] = canonical_hash({
        "device_path": value["tpm"]["device_path"],
        "properties_sha256": value["tpm"]["properties_sha256"],
    })
    value["session_binding_sha256"] = session_binding(value)
    return value


def mutate(value: dict[str, Any], name: str) -> dict[str, Any]:
    out = copy.deepcopy(value)
    if name == "canonical-valid":
        return out
    if name == "session-id-substitution":
        out["session_id"] = "session-attacker"
    elif name == "boot-id-substitution":
        out["boot_id"] = "boot-attacker"
    elif name == "device-path-substitution":
        out["tpm"]["device_path"] = "/dev/tpm0"
    elif name == "tpm-properties-substitution":
        out["tpm"]["properties_sha256"] = "0" * 64
    elif name == "event-log-digest-substitution":
        out["event_log"]["sha256"] = "1" * 64
    elif name == "parser-profile-substitution":
        out["event_log"]["parser_profile_id"] = "other-profile"
    elif name == "pcr-selection-substitution":
        out["quote"]["pcr_selection"] = "sha256:0,2,4"
    elif name == "nonce-substitution":
        out["quote"]["nonce_sha256"] = "2" * 64
    elif name == "attestation-key-substitution":
        out["quote"]["attestation_key_sha256"] = "3" * 64
    elif name == "toolchain-substitution":
        out["toolchain"]["tpm2_tools_version"] = "0.0.0-attacker"
    elif name == "reference-version-rollback":
        out["reference_values"]["version"] = "0.0.0"
    elif name == "reconstruction-event-log-substitution":
        out["reconstruction"]["event_log_sha256"] = "4" * 64
    elif name == "reconstruction-pcr-selection-substitution":
        out["reconstruction"]["pcr_selection"] = "sha256:7"
    elif name == "reconstruction-result-substitution":
        out["reconstruction"]["content_sha256"] = "5" * 64
    elif name == "reconstruction-failure":
        out["reconstruction"]["status"] = "FAIL"
    elif name == "reconstruction-unavailable":
        out["reconstruction"]["status"] = "INDETERMINATE"
    elif name == "trusted-time-unavailable":
        out["trusted_time"]["available"] = False
    elif name == "key-order-permutation":
        out = dict(reversed(list(out.items())))
        for key in ("tpm","event_log","quote","challenge","toolchain","reference_values","trusted_time","reconstruction","live_observation","artifacts"):
            out[key] = dict(reversed(list(out[key].items())))
    elif name == "post-quote-pcr-mismatch":
        out["live_observation"]["pcr_post_sha256"] = "6" * 64
    else:
        raise KeyError(name)
    return out


def validate_semantics(manifest: dict[str, Any]) -> tuple[str, str]:
    denies: list[str] = []
    indeterminate: list[str] = []

    required = {
        "profile_id","profile_version","session_id","boot_id",
        "tpm","event_log","quote","challenge","toolchain",
        "reference_values","trusted_time","reconstruction",
        "live_observation","artifacts","claim_ceiling","session_binding_sha256",
        "os_image_digest","workload_digest"
    }
    for field in sorted(required - set(manifest)):
        denies.append(f"missing-{field}")

    if denies:
        return "DENY", ";".join(denies)

    if manifest["profile_id"] != "mycelix.security.platform.evidence.capture":
        denies.append("profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        denies.append("profile-version-mismatch")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        denies.append("claim-ceiling-mismatch")

    for section, fields in {
        "tpm": ("device_path","properties_sha256","identity_digest"),
        "event_log": ("sha256","parser_profile_id","parser_profile_version","parser_output_sha256"),
        "quote": ("pcr_selection","nonce_sha256","attestation_key_sha256","quoted_pcr_sha256"),
        "challenge": ("sha256",),
        "toolchain": ("tpm2_tools_version","tss_version_evidence_sha256"),
        "reference_values": ("version","sha256"),
        "trusted_time": ("sha256","available","policy_trusted","local_clock_only"),
        "reconstruction": ("status","event_log_sha256","pcr_selection","content_sha256"),
        "live_observation": ("pcr_post_sha256","selection"),
        "artifacts": ("quote_message_sha256","quote_signature_sha256","attestation_key_sha256","reconstruction_file_sha256"),
    }.items():
        value = manifest.get(section)
        if not isinstance(value, dict):
            denies.append(f"{section}-not-object")
            continue
        for field in fields:
            if field not in value:
                denies.append(f"missing-{section}.{field}")

    if not manifest["session_id"] or not manifest["boot_id"]:
        denies.append("empty-session-or-boot-id")

    expected_identity = canonical_hash({
        "device_path": manifest["tpm"]["device_path"],
        "properties_sha256": manifest["tpm"]["properties_sha256"],
    })
    if manifest["tpm"]["identity_digest"] != expected_identity:
        denies.append("tpm-identity-binding-mismatch")

    if manifest["event_log"]["parser_profile_id"] != "tcg.pc-client.event-log":
        denies.append("event-log-parser-profile-mismatch")
    if manifest["event_log"]["parser_profile_version"] != "1.0":
        denies.append("event-log-parser-version-mismatch")

    selection = manifest["quote"]["pcr_selection"]
    if selection != "sha256:0,2,4,7":
        denies.append("unexpected-qualified-pcr-selection")
    if selection != manifest["reconstruction"]["pcr_selection"]:
        denies.append("quote-reconstruction-pcr-selection-mismatch")
    if selection != manifest["live_observation"]["selection"]:
        denies.append("quote-live-pcr-selection-mismatch")

    if manifest["quote"]["nonce_sha256"] != manifest["challenge"]["sha256"]:
        denies.append("nonce-challenge-binding-mismatch")
    if manifest["quote"]["attestation_key_sha256"] != manifest["artifacts"]["attestation_key_sha256"]:
        denies.append("attestation-key-artifact-binding-mismatch")
    if manifest["quote"]["quoted_pcr_sha256"] != manifest["live_observation"]["pcr_post_sha256"]:
        denies.append("quoted-live-pcr-binding-mismatch")

    if manifest["event_log"]["sha256"] != manifest["reconstruction"]["event_log_sha256"]:
        denies.append("event-log-reconstruction-digest-mismatch")
    if manifest["toolchain"]["tpm2_tools_version"] != "5.8":
        denies.append("tpm2-tools-version-mismatch")
    if manifest["reference_values"]["version"] in {"0.0.0", "", "unknown"}:
        denies.append("reference-version-rollback-or-empty")

    reconstruction = manifest["reconstruction"]
    if reconstruction["status"] == "FAIL":
        denies.append("event-log-reconstruction-failed")
    elif reconstruction["status"] != "PASS":
        indeterminate.append("event-log-reconstruction-unavailable")

    trusted = manifest["trusted_time"]
    if trusted["local_clock_only"]:
        denies.append("local-clock-is-not-authoritative")
    elif not trusted["available"] or not trusted["policy_trusted"]:
        indeterminate.append("trusted-time-unavailable-or-untrusted")

    if manifest["reconstruction"]["content_sha256"] != self_hash(manifest["reconstruction"], "content_sha256"):
        denies.append("reconstruction-content-integrity-mismatch")

    expected_session_binding = session_binding(manifest)
    if manifest["session_binding_sha256"] != expected_session_binding:
        denies.append("session-binding-mismatch")

    if denies:
        return "DENY", ";".join(denies)
    if indeterminate:
        return "INDETERMINATE", ";".join(indeterminate)
    return "PASS", "capture-session-coherent"


def self_test(contract: dict[str, Any]) -> int:
    fixture = mutation_manifest()
    failures: list[str] = []
    for vector in contract["vectors"]:
        mutated = mutate(fixture, vector["mutation"])
        got, reason = validate_semantics(mutated)
        ok = got == vector["expected"]
        line = f'{"[PASS]" if ok else "[FAIL]"} {vector["id"]}: expected={vector["expected"]} got={got} reason={reason}'
        print(line)
        if not ok:
            failures.append(line)
    print()
    print(f"Platform capture coherence qualification: {len(contract['vectors']) - len(failures)}/{len(contract['vectors'])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


def run_quote_check(bundle: Path) -> tuple[str, str]:
    binary = shutil.which("tpm2_checkquote")
    if binary is None:
        return "INDETERMINATE", "tpm2_checkquote-unavailable"

    manifest = load_json(bundle / "capture-session.json")
    nonce = bundle / "nonce.bin"
    if sha256_file(nonce) != manifest["challenge"]["sha256"]:
        return "DENY", "nonce-file-digest-mismatch"

    proc = run([
        binary, "-u", str(bundle / "ak.pub"),
        "-m", str(bundle / "quote.msg"), "-s", str(bundle / "quote.sig"),
        "-f", str(bundle / "pcr-post.yaml"), "-g", "sha256",
        "-q", nonce.read_bytes().hex(), "-l", manifest["quote"]["pcr_selection"]
    ], os.environ.copy(), bundle, check=False)
    if proc.returncode != 0:
        return "DENY", "tpm2-checkquote-rejected"
    return "PASS", "tpm2-checkquote-verified"


def verify_bundle(args: argparse.Namespace) -> int:
    bundle = Path(args.bundle).resolve()
    manifest_path = bundle / "capture-session.json"
    if not manifest_path.is_file():
        print("PLATFORM EVIDENCE: DENY: missing capture-session.json")
        return 1

    manifest = load_json(manifest_path)
    semantic_state, semantic_reason = validate_semantics(manifest)
    print(f"Semantic coherence: {semantic_state} ({semantic_reason})")
    if semantic_state != "PASS":
        return 2 if semantic_state == "INDETERMINATE" else 1

    # Raw artifact integrity is checked before invoking the external TPM
    # verifier. These hashes are untrusted claims until the files agree.
    checks = {
        "tpm-properties.txt": manifest["tpm"]["properties_sha256"],
        "eventlog.bin": manifest["event_log"]["sha256"],
        "reference-values.json": manifest["reference_values"]["sha256"],
        "trusted-time.json": manifest["trusted_time"]["sha256"],
        "quote.msg": manifest["artifacts"]["quote_message_sha256"],
        "quote.sig": manifest["artifacts"]["quote_signature_sha256"],
        "ak.pub": manifest["quote"]["attestation_key_sha256"],
        "pcr-post.yaml": manifest["live_observation"]["pcr_post_sha256"],
        "nonce.bin": manifest["challenge"]["sha256"],
        "eventlog-parsed.yaml": manifest["event_log"]["parser_output_sha256"],
    }
    for rel, expected in checks.items():
        path = bundle / rel
        if not path.is_file():
            print(f"PLATFORM EVIDENCE: DENY: missing-artifact-{rel}")
            return 1
        if sha256_file(path) != expected:
            print(f"PLATFORM EVIDENCE: DENY: artifact-digest-mismatch-{rel}")
            return 1

    reconstruction_path = bundle / "eventlog-reconstruction.json"
    if not reconstruction_path.is_file():
        print("PLATFORM EVIDENCE: DENY: missing-artifact-eventlog-reconstruction.json")
        return 1
    reconstruction = load_json(reconstruction_path)
    if reconstruction.get("content_sha256") != manifest["reconstruction"]["content_sha256"]:
        print("PLATFORM EVIDENCE: DENY: reconstruction-content-binding-mismatch")
        return 1
    if self_hash(reconstruction, "content_sha256") != reconstruction.get("content_sha256"):
        print("PLATFORM EVIDENCE: DENY: reconstruction-self-hash-mismatch")
        return 1
    if sha256_file(reconstruction_path) != manifest["artifacts"]["reconstruction_file_sha256"]:
        print("PLATFORM EVIDENCE: DENY: artifact-digest-mismatch-eventlog-reconstruction.json")
        return 1

    quote_state, quote_reason = run_quote_check(bundle)
    print(f"TPM Quote verification: {quote_state} ({quote_reason})")
    if quote_state != "PASS":
        return 2 if quote_state == "INDETERMINATE" else 1

    print("PLATFORM EVIDENCE: PASS")
    print("Claim ceiling: ReferenceModelOnly")
    return 0


def detect_device() -> Path | None:
    for candidate in (Path("/dev/tpmrm0"), Path("/dev/tpm0")):
        if candidate.exists():
            return candidate
    return None


def detect_event_log() -> Path | None:
    candidates = (Path("/sys/kernel/security/tpm0/binary_bios_measurements"),)
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.R_OK):
            return candidate
    return None


def require_capture_tools() -> list[str]:
    names = ("tpm2_getcap","tpm2_pcrread","tpm2_quote","tpm2_createek","tpm2_createak","tpm2_checkquote","tpm2_eventlog")
    return [name for name in names if shutil.which(name) is None]


def observed_tool_versions(output_dir: Path, env: dict[str, str]) -> dict[str, str]:
    names = ("tpm2_getcap","tpm2_pcrread","tpm2_quote","tpm2_createek","tpm2_createak","tpm2_checkquote","tpm2_eventlog")
    versions: dict[str, str] = {}
    for name in names:
        proc = run([name, "--version"], env, output_dir, check=False)
        text = (proc.stdout + proc.stderr).strip().splitlines()
        if not text:
            raise RuntimeError(f"no version output for {name}")
        versions[name] = text[0]
    return versions


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
    if not args.reference_values or not Path(args.reference_values).is_file():
        blockers.append("reference-values.json not supplied")
    if not args.trusted_time or not Path(args.trusted_time).is_file():
        blockers.append("trusted-time.json not supplied")
    if not args.nonce_file or not Path(args.nonce_file).is_file():
        blockers.append("fresh external verifier nonce not supplied")
    if not args.os_image_digest or not args.workload_digest:
        blockers.append("exact OS-image and workload digests not supplied")
    if blockers:
        print("TPM PLATFORM CAPTURE: NOT EXECUTED")
        for blocker in blockers:
            print("- " + blocker)
        return 2

    out = Path(args.output).resolve() if args.output else Path(tempfile.mkdtemp(prefix="mycelix-platform-capture-"))
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["TPM2TOOLS_TCTI"] = f"device:{device}"

    shutil.copy2(args.reference_values, out / "reference-values.json")
    shutil.copy2(args.trusted_time, out / "trusted-time.json")
    shutil.copy2(args.nonce_file, out / "nonce.bin")
    shutil.copy2(event_log, out / "eventlog.bin")

    boot_before = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    props_before = run(["tpm2_getcap","properties-fixed"], env, out)
    (out / "tpm-properties.txt").write_text(props_before.stdout, encoding="utf-8")
    prop_digest = sha256_file(out / "tpm-properties.txt")
    versions = observed_tool_versions(out, env)
    (out / "tool-versions.json").write_text(json.dumps(versions, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    pre = run(["tpm2_pcrread", args.pcr_selection], env, out)
    (out / "pcr-pre.yaml").write_text(pre.stdout, encoding="utf-8")

    run(["tpm2_createek","-Q","-c",str(out/"ek.ctx"),"-G","rsa","-u",str(out/"ek.pub")], env, out)
    run(["tpm2_createak","-Q","-C",str(out/"ek.ctx"),"-c",str(out/"ak.ctx"),"-G","rsa","-g","sha256","-s","rsassa","-u",str(out/"ak.pub"),"-n",str(out/"ak.name")], env, out)

    nonce = (out / "nonce.bin").read_bytes()
    run(["tpm2_quote","-Q","-c",str(out/"ak.ctx"),"-l",args.pcr_selection,"-q",nonce.hex(),"-m",str(out/"quote.msg"),"-s",str(out/"quote.sig"),"-g","sha256"], env, out)

    post = run(["tpm2_pcrread", args.pcr_selection], env, out)
    (out / "pcr-post.yaml").write_text(post.stdout, encoding="utf-8")

    props_after = run(["tpm2_getcap","properties-fixed"], env, out)
    if sha256_bytes(props_after.stdout.encode()) != prop_digest:
        raise RuntimeError("TPM fixed properties changed during capture")
    boot_after = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    if boot_after != boot_before:
        raise RuntimeError("OS boot_id changed during capture")

    parsed = run(["tpm2_eventlog", str(out/"eventlog.bin")], env, out, check=False)
    (out / "eventlog-parsed.yaml").write_text(parsed.stdout + parsed.stderr, encoding="utf-8")
    parser_status = "PASS" if parsed.returncode == 0 else "FAIL"

    trusted = load_json(out / "trusted-time.json")
    reconstruction = {
        "status": "INDETERMINATE",
        "reason": "independent-event-log-reconstruction-result-not-yet-supplied",
        "event_log_sha256": sha256_file(out / "eventlog.bin"),
        "pcr_selection": args.pcr_selection,
        "verifier_id": "external-reconstruction-required",
        "reconstructed_pcr_sha256": "",
    }
    reconstruction["content_sha256"] = self_hash(reconstruction, "content_sha256")
    (out / "eventlog-reconstruction.json").write_text(json.dumps(reconstruction, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    manifest = {
        "profile_id":"mycelix.security.platform.evidence.capture",
        "profile_version":"0.1.0",
        "session_id":f"linuxboot-{boot_before}-{sha256_file(out/'eventlog.bin')[:16]}",
        "boot_id":boot_before,
        "tpm":{
            "device_path":str(device),
            "properties_sha256":prop_digest,
            "identity_digest":""
        },
        "event_log":{
            "sha256":sha256_file(out/"eventlog.bin"),
            "parser_profile_id":"tcg.pc-client.event-log",
            "parser_profile_version":"1.0",
            "parser_output_sha256":sha256_file(out/"eventlog-parsed.yaml"),
            "parser_status":parser_status
        },
        "quote":{
            "pcr_selection":args.pcr_selection,
            "nonce_sha256":sha256_file(out/"nonce.bin"),
            "attestation_key_sha256":sha256_file(out/"ak.pub"),
            "quoted_pcr_sha256":sha256_file(out/"pcr-post.yaml")
        },
        "challenge":{"sha256":sha256_file(out/"nonce.bin"),"origin":"external-verifier-supplied"},
        "toolchain":{
            "tpm2_tools_version": "5.8" if all(v.startswith("5.8") for v in versions.values()) else "mismatch",
            "tss_version_evidence_sha256":sha256_bytes(args.tss_version_evidence.encode()),
            "observed_tool_versions_sha256":sha256_file(out/"tool-versions.json")
        },
        "reference_values":{
            "version":load_json(out/"reference-values.json").get("version","unknown"),
            "sha256":sha256_file(out/"reference-values.json")
        },
        "trusted_time":{
            "sha256":sha256_file(out/"trusted-time.json"),
            "available":bool(trusted.get("available",False)),
            "policy_trusted":bool(trusted.get("policy_trusted",False)),
            "local_clock_only":bool(trusted.get("local_clock_only",False))
        },
        "reconstruction":reconstruction,
        "live_observation":{
            "pcr_post_sha256":sha256_file(out/"pcr-post.yaml"),
            "selection":args.pcr_selection
        },
        "artifacts":{
            "quote_message_sha256":sha256_file(out/"quote.msg"),
            "quote_signature_sha256":sha256_file(out/"quote.sig"),
            "attestation_key_sha256":sha256_file(out/"ak.pub"),
            "reconstruction_file_sha256":sha256_file(out/"eventlog-reconstruction.json")
        },
        "os_image_digest":args.os_image_digest,
        "workload_digest":args.workload_digest,
        "capture_status":"CAPTURED_RAW_EVIDENCE",
        "claim_ceiling":"ReferenceModelOnly"
    }
    manifest["tpm"]["identity_digest"] = canonical_hash({
        "device_path":manifest["tpm"]["device_path"],
        "properties_sha256":manifest["tpm"]["properties_sha256"],
    })
    manifest["session_binding_sha256"] = session_binding(manifest)
    (out/"capture-session.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    print(f"Raw platform Evidence captured to: {out}")
    print("Capture status: CAPTURED_RAW_EVIDENCE")
    print("Qualification status: NOT QUALIFIED")
    if parser_status != "PASS":
        print("Event-log parser: FAIL")
    else:
        print("Event-log parser: PASS")
    print("Event-log reconstruction: INDETERMINATE until independently supplied")
    print("Claim ceiling: ReferenceModelOnly")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--verify", metavar="BUNDLE")
    mode.add_argument("--capture", action="store_true")
    parser.add_argument("--output")
    parser.add_argument("--reference-values")
    parser.add_argument("--trusted-time")
    parser.add_argument("--nonce-file")
    parser.add_argument("--os-image-digest")
    parser.add_argument("--workload-digest")
    parser.add_argument("--pcr-selection", default="sha256:0,2,4,7")
    parser.add_argument("--tss-version-evidence", default="external-package-build-provenance")
    args = parser.parse_args()
    contract = load_json(CONTRACT)
    if args.self_test:
        return self_test(contract)
    if args.verify:
        return verify_bundle(args)
    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
