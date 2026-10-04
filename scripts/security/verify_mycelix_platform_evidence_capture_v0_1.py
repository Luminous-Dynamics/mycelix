#!/usr/bin/env python3
"""Executable platform Evidence capture/coherence verifier for Mycelix v0.1."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-platform-evidence-capture-v0.1.json"
RECONSTRUCTION_SCRIPT = ROOT / "scripts/security/reconstruct_mycelix_pc_client_eventlog_v0_1.py"
ADAPTER_SCRIPT = ROOT / "scripts/security/adapt_mycelix_tpm2_eventlog_yaml_v1_v0_1.py"
RAW_EVENTLOG_PARSER_SCRIPT = ROOT / "scripts/security/parse_mycelix_raw_tpm2_eventlog_v0_1.py"
PAYLOAD_COHERENCE_SCRIPT = ROOT / "scripts/security/verify_mycelix_event_payload_digest_coherence_v0_1.py"
PUBLIC_NAME_VERIFIER_SCRIPT = ROOT / "scripts/security/verify_mycelix_tpm_public_name_coherence_v0_1.py"
PUBLIC_NAME_VERIFIER_ID = "mycelix.tpm.public-name-coherence.v0.1"
EK_TEMPLATE_VERIFIER_SCRIPT = ROOT / "scripts/security/verify_mycelix_ek_template_appraisal_v0_1.py"
EK_TEMPLATE_VERIFIER_ID = "mycelix.tpm.ek-template-appraisal.v0.1"
EK_CERTIFICATE_CAPTURE_SCRIPT = ROOT / "scripts/security/capture_mycelix_ek_certificate_nv_v0_1.py"
EK_CERTIFICATE_CAPTURE_ID = "mycelix.tpm.ek-certificate-nv-capture.v0.1"
EK_CERT_SPki_VERIFIER_SCRIPT = ROOT / "scripts/security/verify_mycelix_ek_cert_spki_binding_v0_1.py"
EK_CERT_SPki_VERIFIER_ID = "mycelix.tpm.ek-cert-spki-binding.v0.1"
RECONSTRUCTION_VERIFIER_ID = "mycelix.pc-client.eventlog-reconstruction.v0.1"


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        c in "0123456789abcdef" for c in value
    )


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_hash(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    return sha256_bytes(raw)


def self_hash(value: dict[str, Any], field: str) -> str:
    clone = copy.deepcopy(value)
    clone.pop(field, None)
    return canonical_hash(clone)


def parse_selection(selection: str) -> list[int]:
    bank, sep, rest = selection.partition(":")
    if bank != "sha256" or not sep or not rest:
        raise ValueError("unsupported PCR selection")
    values = [int(item) for item in rest.split(",") if item.isdigit()]
    if not values or len(values) != len(set(values)):
        raise ValueError("invalid or duplicate PCR selection")
    return values


def parse_pcrread_sha256(text: str, selection: str) -> dict[str, str]:
    wanted = set(parse_selection(selection))
    current_bank: str | None = None
    values: dict[str, str] = {}

    for line in text.splitlines():
        header = re.match(r"^\s*([A-Za-z0-9_-]+)\s*:\s*$", line)
        if header:
            current_bank = header.group(1).lower()
            continue
        if current_bank != "sha256":
            continue
        match = re.match(r"^\s*(\d+)\s*:\s*([0-9A-Fa-f]+)\s*$", line)
        if not match:
            continue
        index = int(match.group(1))
        digest = match.group(2).lower()
        if index not in wanted:
            continue
        if len(digest) != 64:
            raise ValueError(f"PCR{index} is not a SHA-256 value")
        if str(index) in values:
            raise ValueError(f"duplicate PCR{index}")
        values[str(index)] = digest

    if current_bank != "sha256" and not values:
        raise ValueError("sha256 PCR bank missing from tpm2_pcrread output")

    missing = wanted - {int(index) for index in values}
    if missing:
        raise ValueError("missing selected PCRs: " + ",".join(map(str, sorted(missing))))

    return {str(index): values[str(index)] for index in sorted(wanted)}


def pcr_values_hash(values: dict[str, str]) -> str:
    return canonical_hash(
        {"bank": "sha256", "values": {key: values[key] for key in sorted(values, key=int)}}
    )


def session_binding(manifest: dict[str, Any]) -> str:
    return canonical_hash(
        {
            "session_id": manifest["session_id"],
            "boot_id": manifest["boot_id"],
            "tpm_identity_digest": manifest["tpm"]["identity_digest"],
            "ek_public_sha256": manifest["tpm"]["ek_public_sha256"],
            "event_log_sha256": manifest["event_log"]["sha256"],
            "pcr_selection": manifest["quote"]["pcr_selection"],
            "pcr_post_artifact_sha256": manifest["live_observation"]["pcr_post_artifact_sha256"],
            "pcr_values_sha256": manifest["live_observation"]["pcr_values_sha256"],
            "pcr_values_file_sha256": manifest["live_observation"]["pcr_values_file_sha256"],
            "nonce_sha256": manifest["quote"]["nonce_sha256"],
            "challenge_origin": manifest["challenge"]["origin"],
            "attestation_key_sha256": manifest["quote"]["attestation_key_sha256"],
            "tpm2_tools_version": manifest["toolchain"]["tpm2_tools_version"],
            "observed_tool_versions_sha256": manifest["toolchain"]["observed_tool_versions_sha256"],
            "tss_version_evidence_sha256": manifest["toolchain"]["tss_version_evidence_sha256"],
            "reference_version": manifest["reference_values"]["version"],
            "reference_sha256": manifest["reference_values"]["sha256"],
            "trusted_time_sha256": manifest["trusted_time"]["sha256"],
            "reconstruction_content_sha256": manifest["reconstruction"]["content_sha256"],
            "reconstruction_input_sha256": manifest["reconstruction"]["input_sha256"],
            "reconstruction_verifier_id": manifest["reconstruction"]["verifier_id"],
            "reconstruction_verifier_source_sha256": manifest["reconstruction"]["verifier_source_sha256"],
            "os_image_digest": manifest["os_image_digest"],
            "workload_digest": manifest["workload_digest"],
            "raw_eventlog_parser_status": manifest["raw_eventlog"]["status"],
            "raw_eventlog_parser_output_sha256": manifest["raw_eventlog"]["output_sha256"],
            "raw_eventlog_parser_source_sha256": manifest["raw_eventlog"]["source_sha256"],
            "payload_coherence_status": manifest["payload_coherence"]["status"],
            "payload_coherence_output_sha256": manifest["payload_coherence"]["output_sha256"],
            "payload_coherence_source_sha256": manifest["payload_coherence"]["source_sha256"],
            "payload_coherence_input_sha256": manifest["payload_coherence"]["input_sha256"],
            "public_name_ak_status": manifest["public_name_coherence"]["ak"]["status"],
            "public_name_ak_output_sha256": manifest["public_name_coherence"]["ak"]["output_sha256"],
            "public_name_ak_input_sha256": manifest["public_name_coherence"]["ak"]["input_sha256"],
            "public_name_ak_wire_sha256": manifest["public_name_coherence"]["ak"]["wire_sha256"],
            "public_name_ak_name_sha256": manifest["public_name_coherence"]["ak"]["name_sha256"],
            "public_name_ak_qname_sha256": manifest["public_name_coherence"]["ak"]["qname_sha256"],
            "public_name_ak_source_sha256": manifest["public_name_coherence"]["ak"]["source_sha256"],
            "public_name_ek_status": manifest["public_name_coherence"]["ek"]["status"],
            "public_name_ek_output_sha256": manifest["public_name_coherence"]["ek"]["output_sha256"],
            "public_name_ek_input_sha256": manifest["public_name_coherence"]["ek"]["input_sha256"],
            "public_name_ek_wire_sha256": manifest["public_name_coherence"]["ek"]["wire_sha256"],
            "public_name_ek_name_sha256": manifest["public_name_coherence"]["ek"]["name_sha256"],
            "public_name_ek_qname_sha256": manifest["public_name_coherence"]["ek"]["qname_sha256"],
            "public_name_ek_source_sha256": manifest["public_name_coherence"]["ek"]["source_sha256"],
            "ek_template_status": manifest["ek_template_appraisal"]["status"],
            "ek_template_verifier_id": manifest["ek_template_appraisal"]["verifier_id"],
            "ek_template_output_sha256": manifest["ek_template_appraisal"]["output_sha256"],
            "ek_template_input_sha256": manifest["ek_template_appraisal"]["input_sha256"],
            "ek_template_wire_sha256": manifest["ek_template_appraisal"]["wire_sha256"],
            "ek_template_source_sha256": manifest["ek_template_appraisal"]["source_sha256"],
            "ek_template_transcript_sha256": manifest["ek_template_appraisal"]["transcript_sha256"],
            "ek_certificate_capture_status": manifest["ek_certificate_capture"]["status"],
            "ek_certificate_capture_verifier_id": manifest["ek_certificate_capture"]["verifier_id"],
            "ek_certificate_capture_result_sha256": manifest["ek_certificate_capture"]["result_sha256"],
            "ek_certificate_capture_inventory_sha256": manifest["ek_certificate_capture"]["inventory_sha256"],
            "ek_certificate_capture_transcript_sha256": manifest["ek_certificate_capture"]["transcript_sha256"],
            "ek_certificate_capture_script_sha256": manifest["ek_certificate_capture"]["script_sha256"],
            "ek_certificate_capture_source_mode": manifest["ek_certificate_capture"]["source_mode"],
            "ek_certificate_capture_candidate_handles": manifest["ek_certificate_capture"]["candidate_handles"],
            "ek_certificate_rsa_sha256": manifest["ek_certificate_capture"]["rsa_certificate_sha256"] or "",
            "ek_certificate_rsa_present": manifest["ek_certificate_capture"]["rsa_certificate_present"],
            "ek_cert_spki_status": manifest["ek_cert_spki_binding"]["status"],
            "ek_cert_spki_verifier_id": manifest["ek_cert_spki_binding"]["verifier_id"],
            "ek_cert_spki_input_sha256": manifest["ek_cert_spki_binding"]["input_sha256"] or "",
            "ek_cert_spki_output_sha256": manifest["ek_cert_spki_binding"]["output_sha256"] or "",
            "ek_cert_spki_certificate_sha256": manifest["ek_cert_spki_binding"]["certificate_sha256"] or "",
            "ek_cert_spki_certificate_source_sha256": manifest["ek_cert_spki_binding"]["certificate_source_sha256"] or "",
            "ek_cert_spki_ek_public_source_sha256": manifest["ek_cert_spki_binding"]["ek_public_source_sha256"] or "",
        }
    )


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def run(
    argv: Sequence[str],
    env: dict[str, str],
    cwd: Path,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(argv),
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def fixture_manifest() -> dict[str, Any]:
    live = {"0": "aa" * 32, "2": "bb" * 32, "4": "cc" * 32, "7": "dd" * 32}
    reconstruction = copy.deepcopy(live)
    value: dict[str, Any] = {
        "profile_id": "mycelix.security.platform.evidence.capture",
        "profile_version": "0.1.0",
        "session_id": "session-20261004-0001",
        "boot_id": "boot-20261004-0001",
        "tpm": {
            "device_path": "/dev/tpmrm0",
            "properties_sha256": "a" * 64,
            "ek_public_sha256": "b" * 64,
            "identity_digest": "",
        },
        "event_log": {
            "sha256": "c" * 64,
            "parser_profile_id": "tcg.pc-client.event-log",
            "parser_profile_version": "1.0",
            "parser_status": "PASS",
            "parser_output_sha256": "7" * 64,
        },
        "raw_eventlog": {
            "status": "PASS",
            "parser_id": "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1",
            "output_sha256": "8" * 64,
            "source_sha256": sha256_file(RAW_EVENTLOG_PARSER_SCRIPT),
            "binary_sha256": "c" * 64,
        },
        "public_name_coherence": {
            "ak": {
                "status": "INDETERMINATE",
                "verifier_id": PUBLIC_NAME_VERIFIER_ID,
                "output_sha256": "aa" * 32,
                "input_sha256": "ab" * 32,
                "wire_sha256": "ac" * 32,
                "name_sha256": "ad" * 32,
                "qname_sha256": "ae" * 32,
                "source_sha256": "af" * 32,
            },
            "ek": {
                "status": "INDETERMINATE",
                "verifier_id": PUBLIC_NAME_VERIFIER_ID,
                "output_sha256": "ba" * 32,
                "input_sha256": "bb" * 32,
                "wire_sha256": "bc" * 32,
                "name_sha256": "bd" * 32,
                "qname_sha256": "be" * 32,
                "source_sha256": "bf" * 32,
            },
        },
        "ek_template_appraisal": {
            "status": "INDETERMINATE",
            "verifier_id": EK_TEMPLATE_VERIFIER_ID,
            "output_sha256": "ca" * 32,
            "input_sha256": "cb" * 32,
            "wire_sha256": "cc" * 32,
            "source_sha256": "cd" * 32,
            "transcript_sha256": "ce" * 32,
        },
        "ek_certificate_capture": {
            "status": "INDETERMINATE",
            "verifier_id": EK_CERTIFICATE_CAPTURE_ID,
            "reason": "no-tcg-ek-certificate-nv-index-observed",
            "result_sha256": "d1" * 32,
            "inventory_sha256": "d2" * 32,
            "transcript_sha256": "d3" * 32,
            "script_sha256": sha256_file(EK_CERTIFICATE_CAPTURE_SCRIPT),
            "source_mode": "TPM_NV_ONLY",
            "candidate_handles": [],
            "rsa_certificate_present": False,
            "rsa_certificate_sha256": None,
        },
        "ek_cert_spki_binding": {
            "status": "INDETERMINATE",
            "verifier_id": EK_CERT_SPki_VERIFIER_ID,
            "reason": "rsa-ek-certificate-not-present",
            "input_sha256": None,
            "output_sha256": None,
            "certificate_sha256": None,
            "ek_public_wire_sha256": "bc" * 32,
            "certificate_source_sha256": "d3" * 32,
            "ek_public_source_sha256": "bf" * 32,
        },
        "quote": {
            "pcr_selection": "sha256:0,2,4,7",
            "nonce_sha256": "d" * 64,
            "attestation_key_sha256": "e" * 64,
        },
        "challenge": {"sha256": "d" * 64, "origin": "external-verifier-supplied"},
        "toolchain": {
            "tpm2_tools_version": "5.8",
            "observed_tool_versions_sha256": "0" * 64,
            "tss_version_evidence_sha256": "f" * 64,
        },
        "reference_values": {"version": "pc-client-rim-2026.1", "sha256": "1" * 64},
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
            "reconstructed_pcrs_sha256": pcr_values_hash(reconstruction),
            "input_sha256": "3" * 64,
            "verifier_id": RECONSTRUCTION_VERIFIER_ID,
            "verifier_source_sha256": "4" * 64,
        },
        "live_observation": {
            "selection": "sha256:0,2,4,7",
            "pcr_post_artifact_sha256": "4" * 64,
            "pcr_values_sha256": pcr_values_hash(live),
            "pcr_values_file_sha256": "a" * 64,
        },
        "payload_coherence": {
            "status": "INDETERMINATE",
            "output_sha256": "9" * 64,
            "source_sha256": sha256_file(PAYLOAD_COHERENCE_SCRIPT),
            "input_sha256": "3" * 64,
        },
        "ek_template_appraisal": {
            "status": "INDETERMINATE",
            "verifier_id": EK_TEMPLATE_VERIFIER_ID,
            "output_sha256": "ca" * 32,
            "input_sha256": "cb" * 32,
            "wire_sha256": "cc" * 32,
            "source_sha256": "cd" * 32,
            "transcript_sha256": "ce" * 32,
        },
        "public_name_coherence": {
            "ak": {
                "status": "INDETERMINATE",
                "verifier_id": PUBLIC_NAME_VERIFIER_ID,
                "output_sha256": "aa" * 32,
                "input_sha256": "ab" * 32,
                "wire_sha256": "ac" * 32,
                "name_sha256": "ad" * 32,
                "qname_sha256": "ae" * 32,
                "source_sha256": "af" * 32,
            },
            "ek": {
                "status": "INDETERMINATE",
                "verifier_id": PUBLIC_NAME_VERIFIER_ID,
                "output_sha256": "ba" * 32,
                "input_sha256": "bb" * 32,
                "wire_sha256": "bc" * 32,
                "name_sha256": "bd" * 32,
                "qname_sha256": "be" * 32,
                "source_sha256": "bf" * 32,
            },
        },
        "artifacts": {
            "quote_message_sha256": "5" * 64,
            "quote_signature_sha256": "6" * 64,
            "attestation_key_sha256": "e" * 64,
            "reconstruction_file_sha256": "8" * 64,
            "reconstruction_input_sha256": "3" * 64,
            "observed_pcr_values_file_sha256": "a" * 64,
            "raw_eventlog_output_sha256": "8" * 64,
            "payload_coherence_output_sha256": "9" * 64,
            "public_name_ak_output_sha256": "aa" * 32,
            "public_name_ak_input_sha256": "ab" * 32,
            "public_name_ak_wire_sha256": "ac" * 32,
            "public_name_ak_name_sha256": "ad" * 32,
            "public_name_ak_qname_sha256": "ae" * 32,
            "public_name_ak_source_sha256": "af" * 32,
            "public_name_ek_output_sha256": "ba" * 32,
            "public_name_ek_input_sha256": "bb" * 32,
            "public_name_ek_wire_sha256": "bc" * 32,
            "public_name_ek_name_sha256": "bd" * 32,
            "public_name_ek_qname_sha256": "be" * 32,
            "public_name_ek_source_sha256": "bf" * 32,
            "ek_template_output_sha256": "ca" * 32,
            "ek_template_input_sha256": "cb" * 32,
            "ek_template_wire_sha256": "cc" * 32,
            "ek_template_source_sha256": "cd" * 32,
            "ek_template_transcript_sha256": "ce" * 32,
            "tss_version_evidence_sha256": "f" * 64,
            "ek_public_sha256": "b" * 64,
        },
        "os_image_digest": "sha256:" + "9" * 64,
        "workload_digest": "sha256:" + "a" * 64,
        "claim_ceiling": "ReferenceModelOnly",
    }
    value["tpm"]["identity_digest"] = canonical_hash(
        {
            "device_path": value["tpm"]["device_path"],
            "properties_sha256": value["tpm"]["properties_sha256"],
            "ek_public_sha256": value["tpm"]["ek_public_sha256"],
        }
    )
    value["reconstruction"]["content_sha256"] = self_hash(
        value["reconstruction"], "content_sha256"
    )
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
    elif name == "ek-substitution":
        out["tpm"]["ek_public_sha256"] = "f" * 64
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
        out["reconstruction"]["reconstructed_pcrs_sha256"] = "5" * 64
    elif name == "reconstruction-failure":
        out["reconstruction"]["status"] = "FAIL"
    elif name == "reconstruction-unavailable":
        out["reconstruction"]["status"] = "INDETERMINATE"
        out["reconstruction"]["content_sha256"] = self_hash(
            out["reconstruction"], "content_sha256"
        )
        out["session_binding_sha256"] = session_binding(out)
    elif name == "trusted-time-unavailable":
        out["trusted_time"]["available"] = False
    elif name == "key-order-permutation":
        out = dict(reversed(list(out.items())))
        for key in (
            "tpm",
            "event_log",
            "quote",
            "challenge",
            "toolchain",
            "reference_values",
            "trusted_time",
            "reconstruction",
            "live_observation",
            "artifacts",
            "public_name_coherence",
        ):
            out[key] = dict(reversed(list(out[key].items())))
    elif name == "post-quote-pcr-value-mismatch":
        out["live_observation"]["pcr_values_sha256"] = "6" * 64
    elif name == "public-name-ak-status-deny":
        out["public_name_coherence"]["ak"]["status"] = "DENY"
    elif name == "public-name-ek-verifier-substitution":
        out["public_name_coherence"]["ek"]["verifier_id"] = "other-verifier"
    elif name == "public-name-ak-wire-substitution":
        out["public_name_coherence"]["ak"]["wire_sha256"] = "12" * 32
    elif name == "public-name-cross-key-splice":
        out["public_name_coherence"]["ak"]["wire_sha256"] = out["public_name_coherence"]["ek"]["wire_sha256"]
    elif name == "public-name-ak-source-substitution":
        out["public_name_coherence"]["ak"]["source_sha256"] = "13" * 32
    elif name == "deny-over-indeterminate":
        out["event_log"]["parser_status"] = "FAIL"
        out["reconstruction"]["status"] = "INDETERMINATE"
        out["reconstruction"]["content_sha256"] = self_hash(
            out["reconstruction"], "content_sha256"
        )
        out["session_binding_sha256"] = session_binding(out)
    else:
        raise KeyError(name)
    return out


def validate_semantics(manifest: dict[str, Any]) -> tuple[str, str]:
    denies: list[str] = []
    indeterminate: list[str] = []

    required = {
        "profile_id",
        "profile_version",
        "session_id",
        "boot_id",
        "tpm",
        "event_log",
        "quote",
        "challenge",
        "toolchain",
        "reference_values",
        "trusted_time",
        "reconstruction",
        "live_observation",
        "artifacts",
        "ek_certificate_capture",
        "ek_cert_spki_binding",
        "claim_ceiling",
        "session_binding_sha256",
        "os_image_digest",
        "workload_digest",
    }
    denies.extend("missing-" + field for field in sorted(required - set(manifest)))
    if denies:
        return "DENY", ";".join(denies)

    if manifest["profile_id"] != "mycelix.security.platform.evidence.capture":
        denies.append("profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        denies.append("profile-version-mismatch")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        denies.append("claim-ceiling-mismatch")

    sections = {
        "tpm": ("device_path", "properties_sha256", "ek_public_sha256", "identity_digest"),
        "event_log": (
            "sha256",
            "parser_profile_id",
            "parser_profile_version",
            "parser_status",
            "parser_output_sha256",
        ),
        "raw_eventlog": (
            "status",
            "parser_id",
            "output_sha256",
            "source_sha256",
            "binary_sha256",
        ),
        "payload_coherence": (
            "status",
            "output_sha256",
            "source_sha256",
            "input_sha256",
        ),
        "quote": ("pcr_selection", "nonce_sha256", "attestation_key_sha256"),
        "challenge": ("sha256", "origin"),
        "toolchain": (
            "tpm2_tools_version",
            "observed_tool_versions_sha256",
            "tss_version_evidence_sha256",
        ),
        "reference_values": ("version", "sha256"),
        "trusted_time": ("sha256", "available", "policy_trusted", "local_clock_only"),
        "reconstruction": (
            "status",
            "event_log_sha256",
            "pcr_selection",
            "reconstructed_pcrs_sha256",
            "content_sha256",
            "input_sha256",
            "verifier_id",
            "verifier_source_sha256",
        ),
        "live_observation": ("selection", "pcr_post_artifact_sha256", "pcr_values_sha256", "pcr_values_file_sha256"),
        "artifacts": (
            "quote_message_sha256",
            "quote_signature_sha256",
            "attestation_key_sha256",
            "reconstruction_file_sha256",
            "reconstruction_input_sha256",
            "observed_pcr_values_file_sha256",
            "raw_eventlog_output_sha256",
            "payload_coherence_output_sha256",
            "tss_version_evidence_sha256",
            "ek_public_sha256",
        ),
    }

    ek_template = manifest.get("ek_template_appraisal")
    if not isinstance(ek_template, dict):
        denies.append("ek-template-appraisal-not-object")
    else:
        for field in ("status","verifier_id","output_sha256","input_sha256","wire_sha256","source_sha256","transcript_sha256"):
            if field not in ek_template:
                denies.append(f"missing-ek-template.{field}")
        if ek_template.get("verifier_id") != EK_TEMPLATE_VERIFIER_ID:
            denies.append("ek-template-verifier-id-mismatch")
        if ek_template.get("status") not in {"PASS","INDETERMINATE"}:
            denies.append("ek-template-invalid-state")
        fields = ("output_sha256","input_sha256","wire_sha256","source_sha256","transcript_sha256")
        if not all(valid_hash(ek_template.get(field)) for field in fields):
            denies.append("ek-template-digest-invalid")
        artifact_map = {
            "output_sha256":"ek_template_output_sha256",
            "input_sha256":"ek_template_input_sha256",
            "wire_sha256":"ek_template_wire_sha256",
            "source_sha256":"ek_template_source_sha256",
            "transcript_sha256":"ek_template_transcript_sha256",
        }
        for field, artifact_key in artifact_map.items():
            if valid_hash(ek_template.get(field)) and manifest["artifacts"].get(artifact_key) != ek_template[field]:
                denies.append(f"ek-template-{field}-artifact-binding-mismatch")
        if ek_template.get("wire_sha256") != manifest.get("public_name_coherence",{}).get("ek",{}).get("wire_sha256"):
            denies.append("ek-template-public-name-wire-mismatch")
    public_name = manifest.get("public_name_coherence")
    if not isinstance(public_name, dict):
        denies.append("public-name-coherence-not-object")
    else:
        for role in ("ak", "ek"):
            entry = public_name.get(role)
            if not isinstance(entry, dict):
                denies.append(f"public-name-{role}-not-object")
                continue
            for field in ("status", "verifier_id", "output_sha256", "input_sha256", "wire_sha256", "name_sha256", "qname_sha256", "source_sha256"):
                if field not in entry:
                    denies.append(f"missing-public-name-{role}.{field}")
    for section, fields in sections.items():
        value = manifest.get(section)
        if not isinstance(value, dict):
            denies.append(section + "-not-object")
            continue
        for field in fields:
            if field not in value:
                denies.append(f"missing-{section}.{field}")

    expected_identity = canonical_hash(
        {
            "device_path": manifest["tpm"]["device_path"],
            "properties_sha256": manifest["tpm"]["properties_sha256"],
            "ek_public_sha256": manifest["tpm"]["ek_public_sha256"],
        }
    )
    if manifest["tpm"]["identity_digest"] != expected_identity:
        denies.append("tpm-identity-binding-mismatch")

    event_log = manifest["event_log"]
    if event_log["parser_profile_id"] != "tcg.pc-client.event-log":
        denies.append("event-log-parser-profile-mismatch")
    if event_log["parser_profile_version"] != "1.0":
        denies.append("event-log-parser-version-mismatch")
    if event_log["parser_status"] != "PASS":
        denies.append("event-log-parser-failed")

    selection = manifest["quote"]["pcr_selection"]
    if selection != "sha256:0,2,4,7":
        denies.append("unexpected-qualified-pcr-selection")
    if selection != manifest["reconstruction"]["pcr_selection"]:
        denies.append("quote-reconstruction-pcr-selection-mismatch")
    if selection != manifest["live_observation"]["selection"]:
        denies.append("quote-live-pcr-selection-mismatch")

    if manifest["quote"]["nonce_sha256"] != manifest["challenge"]["sha256"]:
        denies.append("nonce-challenge-binding-mismatch")
    if manifest["challenge"]["origin"] in {
        "local-wall-clock",
        "local-generated-clock",
        "wall-clock",
    }:
        denies.append("non-authoritative-challenge-origin")
    if manifest["quote"]["attestation_key_sha256"] != manifest["artifacts"]["attestation_key_sha256"]:
        denies.append("attestation-key-artifact-binding-mismatch")
    if manifest["tpm"]["ek_public_sha256"] != manifest["artifacts"]["ek_public_sha256"]:
        denies.append("ek-artifact-binding-mismatch")
    if manifest["live_observation"]["pcr_values_file_sha256"] != manifest["artifacts"]["observed_pcr_values_file_sha256"]:
        denies.append("observed-pcr-file-binding-mismatch")
    if manifest["raw_eventlog"]["status"] != "PASS":
        denies.append("raw-eventlog-parser-failed")
    if manifest["raw_eventlog"]["parser_id"] != "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1":
        denies.append("raw-eventlog-parser-id-mismatch")
    if manifest["raw_eventlog"]["binary_sha256"] != manifest["event_log"]["sha256"]:
        denies.append("raw-eventlog-binary-binding-mismatch")
    if manifest["payload_coherence"]["source_sha256"] != sha256_file(PAYLOAD_COHERENCE_SCRIPT):
        denies.append("payload-coherence-source-binding-mismatch")
    if manifest["payload_coherence"]["input_sha256"] != manifest["reconstruction"]["input_sha256"]:
        denies.append("payload-coherence-input-binding-mismatch")
    for role in ("ak", "ek"):
        entry = manifest["public_name_coherence"][role]
        prefix = "public_name_ak" if role == "ak" else "public_name_ek"
        if entry["verifier_id"] != PUBLIC_NAME_VERIFIER_ID:
            denies.append(f"public-name-{role}-verifier-id-mismatch")
        if entry["status"] not in {"PASS", "INDETERMINATE"}:
            denies.append(f"public-name-{role}-invalid-state")
        fields = ("output_sha256","input_sha256","wire_sha256","name_sha256","qname_sha256","source_sha256")
        if not all(valid_hash(entry[field]) for field in fields):
            denies.append(f"public-name-{role}-digest-invalid")
        artifact_map = {
            "output_sha256": f"{prefix}_output_sha256",
            "input_sha256": f"{prefix}_input_sha256",
            "wire_sha256": f"{prefix}_wire_sha256",
            "name_sha256": f"{prefix}_name_sha256",
            "qname_sha256": f"{prefix}_qname_sha256",
            "source_sha256": f"{prefix}_source_sha256",
        }
        for field, artifact_key in artifact_map.items():
            if valid_hash(entry[field]) and manifest["artifacts"].get(artifact_key) != entry[field]:
                denies.append(f"public-name-{role}-{field}-artifact-binding-mismatch")

    if (
        manifest["reconstruction"]["status"] == "PASS"
        and manifest["reconstruction"]["reconstructed_pcrs_sha256"]
        != manifest["live_observation"]["pcr_values_sha256"]
    ):
        denies.append("reconstructed-live-pcr-values-mismatch")

    if manifest["event_log"]["sha256"] != manifest["reconstruction"]["event_log_sha256"]:
        denies.append("event-log-reconstruction-digest-mismatch")
    if manifest["toolchain"]["tpm2_tools_version"] != "5.8":
        denies.append("tpm2-tools-version-mismatch")
    if (
        manifest["toolchain"]["tss_version_evidence_sha256"]
        != manifest["artifacts"]["tss_version_evidence_sha256"]
    ):
        denies.append("tss-provenance-artifact-binding-mismatch")
    if manifest["reference_values"]["version"] in {"0.0.0", "", "unknown"}:
        denies.append("reference-version-rollback-or-empty")

    reconstruction = manifest["reconstruction"]
    if reconstruction["status"] == "FAIL":
        denies.append("event-log-reconstruction-failed")
    elif reconstruction["status"] != "PASS":
        indeterminate.append("event-log-reconstruction-unavailable")
    if reconstruction["content_sha256"] != self_hash(reconstruction, "content_sha256"):
        denies.append("reconstruction-content-integrity-mismatch")

    trusted = manifest["trusted_time"]
    if trusted["local_clock_only"]:
        denies.append("local-clock-is-not-authoritative")
    elif not trusted["available"] or not trusted["policy_trusted"]:
        indeterminate.append("trusted-time-unavailable-or-untrusted")

    if manifest["session_binding_sha256"] != session_binding(manifest):
        denies.append("session-binding-mismatch")

    if denies:
        return "DENY", ";".join(denies)
    if indeterminate:
        return "INDETERMINATE", ";".join(indeterminate)
    return "PASS", "capture-session-coherent"


def self_test(contract: dict[str, Any]) -> int:
    fixture = fixture_manifest()
    sample = """sha256 :
  0  : aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  2  : bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  4  : cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
  7  : dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd
"""
    parsed = parse_pcrread_sha256(sample, "sha256:0,2,4,7")
    expected = {"0": "aa" * 32, "2": "bb" * 32, "4": "cc" * 32, "7": "dd" * 32}
    if parsed != expected:
        print("PCR parser self-test: FAIL")
        return 1
    print("PCR parser self-test: PASS")

    failures: list[str] = []
    for vector in contract["vectors"]:
        state, reason = validate_semantics(mutate(fixture, vector["mutation"]))
        ok = state == vector["expected"]
        print(
            f'{"[PASS]" if ok else "[FAIL]"} {vector["id"]}: '
            f"expected={vector['expected']} got={state} reason={reason}"
        )
        if not ok:
            failures.append(vector["id"])

    print()
    print(
        "Platform capture coherence qualification: "
        f"{len(contract['vectors']) - len(failures)}/{len(contract['vectors'])} vectors passed"
    )
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

    proc = run(
        [
            binary,
            "-u",
            str(bundle / "ak.pub"),
            "-m",
            str(bundle / "quote.msg"),
            "-s",
            str(bundle / "quote.sig"),
            "-f",
            str(bundle / "pcr-post.yaml"),
            "-g",
            "sha256",
            "-q",
            nonce.read_bytes().hex(),
            "-l",
            manifest["quote"]["pcr_selection"],
        ],
        os.environ.copy(),
        bundle,
        check=False,
    )
    return (
        ("PASS", "tpm2-checkquote-verified")
        if proc.returncode == 0
        else ("DENY", "tpm2-checkquote-rejected")
    )


def validate_raw_eventlog_result(raw: dict[str, Any], manifest: dict[str, Any]) -> tuple[str, str]:
    required = {"profile_id", "profile_version", "parser_id", "parser_source_sha256", "binary_sha256", "events", "content_sha256"}
    missing = sorted(required - set(raw))
    if missing:
        return "DENY", "raw-eventlog-missing-" + ",".join(missing)
    if raw["profile_id"] != "mycelix.security.platform.binary-eventlog.extraction":
        return "DENY", "raw-eventlog-profile-mismatch"
    if raw["profile_version"] != "0.1.0":
        return "DENY", "raw-eventlog-version-mismatch"
    if raw["parser_id"] != "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1":
        return "DENY", "raw-eventlog-parser-id-mismatch"
    if raw["parser_source_sha256"] != sha256_file(RAW_EVENTLOG_PARSER_SCRIPT):
        return "DENY", "raw-eventlog-parser-source-mismatch"
    if raw["binary_sha256"] != manifest["event_log"]["sha256"]:
        return "DENY", "raw-eventlog-binary-binding-mismatch"
    if raw["content_sha256"] != self_hash(raw, "content_sha256"):
        return "DENY", "raw-eventlog-content-integrity-mismatch"
    events = raw["events"]
    if not isinstance(events, list) or not events:
        return "DENY", "raw-eventlog-events-invalid"
    previous = -1
    for event in events:
        if not isinstance(event, dict):
            return "DENY", "raw-eventlog-event-not-object"
        if not isinstance(event.get("sequence"), int) or event["sequence"] <= previous:
            return "DENY", "raw-eventlog-sequence-invalid"
        previous = event["sequence"]
        if not isinstance(event.get("pcr"), int) or not 0 <= event["pcr"] < 24:
            return "DENY", "raw-eventlog-pcr-index-invalid"
        if not isinstance(event.get("event_type"), str) or not event["event_type"]:
            return "DENY", "raw-eventlog-type-invalid"
        payload = event.get("payload_hex")
        if not isinstance(payload, str) or len(payload) % 2 or any(c not in "0123456789abcdefABCDEF" for c in payload):
            return "DENY", "raw-eventlog-payload-hex-invalid"
        digests = event.get("digests")
        if not isinstance(digests, dict):
            return "DENY", "raw-eventlog-digests-invalid"
        if "sha256" in digests and event.get("digest_sha256") != digests["sha256"]:
            return "DENY", "raw-eventlog-sha256-digest-binding-mismatch"
    return "PASS", "raw-eventlog-structure-valid"

def validate_reconstruction_result(
    reconstruction: dict[str, Any],
    manifest: dict[str, Any],
    live_values: dict[str, str],
    input_path: Path,
) -> tuple[str, str]:
    required = {
        "profile_id","profile_version","event_log_sha256","session_id","pcr_bank",
        "pcr_selection","event_count","reconstructed_pcr_values","reconstructed_pcrs_sha256",
        "observed_pcr_values","observed_pcrs_sha256","match","reconstruction_status",
        "reason","input_sha256","verifier_id","verifier_source_sha256","content_sha256",
    }
    missing = sorted(required - set(reconstruction))
    if missing:
        return "DENY", "reconstruction-result-missing-" + ",".join(missing)
    if reconstruction["profile_id"] != "mycelix.security.platform.eventlog.reconstruction":
        return "DENY", "reconstruction-result-profile-mismatch"
    if reconstruction["profile_version"] != "0.1.0":
        return "DENY", "reconstruction-result-version-mismatch"
    if reconstruction["event_log_sha256"] != manifest["event_log"]["sha256"]:
        return "DENY", "reconstruction-result-eventlog-mismatch"
    if reconstruction["session_id"] != manifest["session_id"]:
        return "DENY", "reconstruction-result-session-mismatch"
    if reconstruction["pcr_bank"] != "sha256":
        return "DENY", "reconstruction-result-bank-mismatch"
    if reconstruction["pcr_selection"] != manifest["live_observation"]["selection"]:
        return "DENY", "reconstruction-result-selection-mismatch"
    if not isinstance(reconstruction["event_count"], int) or reconstruction["event_count"] <= 0:
        return "DENY", "reconstruction-result-event-count-invalid"
    if reconstruction["reconstruction_status"] != "PASS" or reconstruction["match"] is not True:
        return "DENY", "reconstruction-result-not-qualified"
    if not isinstance(reconstruction["reconstructed_pcr_values"], dict):
        return "DENY", "reconstruction-result-reconstructed-map-invalid"
    if not isinstance(reconstruction["observed_pcr_values"], dict):
        return "DENY", "reconstruction-result-observed-map-invalid"

    try:
        selected = parse_selection(reconstruction["pcr_selection"])
    except (TypeError, ValueError):
        return "DENY", "reconstruction-result-selection-invalid"
    expected_keys = {str(index) for index in selected}
    for label in ("reconstructed_pcr_values", "observed_pcr_values"):
        values = reconstruction[label]
        if set(values) != expected_keys:
            return "DENY", "reconstruction-result-" + label + "-selection-mismatch"
        if any(
            not isinstance(value, str)
            or len(value) != 64
            or any(char not in "0123456789abcdef" for char in value)
            for value in values.values()
        ):
            return "DENY", "reconstruction-result-" + label + "-invalid-digest"

    if pcr_values_hash(reconstruction["reconstructed_pcr_values"]) != reconstruction["reconstructed_pcrs_sha256"]:
        return "DENY", "reconstruction-result-reconstructed-state-hash-mismatch"
    if pcr_values_hash(reconstruction["observed_pcr_values"]) != reconstruction["observed_pcrs_sha256"]:
        return "DENY", "reconstruction-result-observed-state-hash-mismatch"
    if reconstruction["reconstructed_pcr_values"] != reconstruction["observed_pcr_values"]:
        return "DENY", "reconstruction-result-map-disagreement"
    if reconstruction["reconstructed_pcrs_sha256"] != manifest["reconstruction"]["reconstructed_pcrs_sha256"]:
        return "DENY", "reconstruction-result-manifest-hash-mismatch"
    if reconstruction["observed_pcrs_sha256"] != manifest["live_observation"]["pcr_values_sha256"]:
        return "DENY", "reconstruction-result-live-hash-mismatch"
    if reconstruction["observed_pcr_values"] != live_values:
        return "DENY", "reconstruction-result-live-map-mismatch"
    if reconstruction["input_sha256"] != sha256_file(input_path):
        return "DENY", "reconstruction-result-input-digest-mismatch"
    if reconstruction["input_sha256"] != manifest["reconstruction"]["input_sha256"]:
        return "DENY", "reconstruction-result-manifest-input-mismatch"
    if reconstruction["verifier_id"] != RECONSTRUCTION_VERIFIER_ID:
        return "DENY", "reconstruction-result-verifier-id-mismatch"
    if reconstruction["verifier_source_sha256"] != sha256_file(RECONSTRUCTION_SCRIPT):
        return "DENY", "reconstruction-result-verifier-source-mismatch"
    if reconstruction["content_sha256"] != self_hash(reconstruction, "content_sha256"):
        return "DENY", "reconstruction-result-content-hash-mismatch"

    source = load_json(input_path)
    if source.get("session_id") != manifest["session_id"]:
        return "DENY", "reconstruction-input-session-mismatch"
    if source.get("event_log_sha256") != manifest["event_log"]["sha256"]:
        return "DENY", "reconstruction-input-eventlog-mismatch"
    if source.get("pcr_selection") != manifest["live_observation"]["selection"]:
        return "DENY", "reconstruction-input-selection-mismatch"
    events = source.get("events")
    if not isinstance(events, list) or len(events) != reconstruction["event_count"]:
        return "DENY", "reconstruction-input-event-count-mismatch"
    return "PASS", "reconstruction-result-schema-valid"


def run_independent_reconstruction(input_path: Path, output_path: Path, cwd: Path) -> tuple[str, str]:
    if not RECONSTRUCTION_SCRIPT.is_file():
        return "DENY", "reconstruction-verifier-missing"
    proc = run(
        [sys.executable, str(RECONSTRUCTION_SCRIPT), "--reconstruct", str(input_path), "--output", str(output_path)],
        os.environ.copy(), cwd, check=False,
    )
    if proc.returncode == 2:
        return "INDETERMINATE", "independent-reconstruction-indeterminate"
    if proc.returncode != 0:
        return "DENY", "independent-reconstruction-failed"
    return "PASS", "independent-reconstruction-executed"

def run_independent_raw_eventlog_parser(
    eventlog_path: Path, output_path: Path, cwd: Path
) -> tuple[str, str]:
    if not RAW_EVENTLOG_PARSER_SCRIPT.is_file():
        return "DENY", "raw-eventlog-parser-missing"
    proc = run(
        [
            sys.executable,
            str(RAW_EVENTLOG_PARSER_SCRIPT),
            "--parse",
            str(eventlog_path),
            "--output",
            str(output_path),
        ],
        os.environ.copy(),
        cwd,
        check=False,
    )
    if proc.returncode != 0:
        return "DENY", "independent-raw-eventlog-parser-failed"
    return "PASS", "independent-raw-eventlog-parser-executed"


def run_ek_template_verifier(input_path: Path, output_path: Path, cwd: Path) -> tuple[str, str]:
    if not EK_TEMPLATE_VERIFIER_SCRIPT.is_file():
        return "DENY", "ek-template-verifier-missing"
    proc = run(
        [
            sys.executable,
            str(EK_TEMPLATE_VERIFIER_SCRIPT),
            "--verify",
            str(input_path),
            "--output",
            str(output_path),
        ],
        os.environ.copy(),
        cwd,
        check=False,
    )
    if proc.returncode == 0:
        return "PASS", "ek-template-verifier-executed"
    if proc.returncode == 2:
        return "INDETERMINATE", "ek-template-verifier-indeterminate"
    return "DENY", "ek-template-verifier-failed"


def run_public_name_verifier(input_path: Path, output_path: Path, cwd: Path) -> tuple[str, str]:
    if not PUBLIC_NAME_VERIFIER_SCRIPT.is_file():
        return "DENY", "public-name-verifier-missing"
    proc = run(
        [sys.executable, str(PUBLIC_NAME_VERIFIER_SCRIPT), "--verify", str(input_path), "--output", str(output_path)],
        os.environ.copy(), cwd, check=False,
    )
    if proc.returncode == 0:
        return "PASS", "public-name-verifier-executed"
    if proc.returncode == 2:
        return "INDETERMINATE", "public-name-verifier-indeterminate"
    return "DENY", "public-name-verifier-failed"


def normalized_tpm2b_public_body(path: Path) -> bytes:
    raw = path.read_bytes()
    if len(raw) < 2:
        raise ValueError(f"{path.name}: TPM2B_PUBLIC wrapper is truncated")
    declared = int.from_bytes(raw[:2], "big")
    if declared != len(raw) - 2:
        raise ValueError(f"{path.name}: TPM2B_PUBLIC wrapper size mismatch")
    return raw[2:]


def run_public_name_verifier(input_path: Path, output_path: Path, cwd: Path) -> tuple[str, str]:
    if not PUBLIC_NAME_VERIFIER_SCRIPT.is_file():
        return "DENY", "public-name-verifier-missing"
    proc = run(
        [
            sys.executable,
            str(PUBLIC_NAME_VERIFIER_SCRIPT),
            "--verify",
            str(input_path),
            "--output",
            str(output_path),
        ],
        os.environ.copy(),
        cwd,
        check=False,
    )
    if proc.returncode == 0:
        return "PASS", "public-name-verifier-executed"
    if proc.returncode == 2:
        return "INDETERMINATE", "public-name-verifier-indeterminate"
    return "DENY", "public-name-verifier-failed"


def verify_bundle(args: argparse.Namespace) -> int:
    bundle = Path(args.bundle).resolve()
    manifest_path = bundle / "capture-session.json"
    if not manifest_path.is_file():
        print("PLATFORM EVIDENCE: DENY: missing capture-session.json")
        return 1

    manifest = load_json(manifest_path)
    state, reason = validate_semantics(manifest)
    print(f"Semantic coherence: {state} ({reason})")
    if state != "PASS":
        return 2 if state == "INDETERMINATE" else 1

    checks = {
        "tpm-properties.txt": manifest["tpm"]["properties_sha256"],
        "ek.pub": manifest["tpm"]["ek_public_sha256"],
        "eventlog.bin": manifest["event_log"]["sha256"],
        "eventlog-parsed.yaml": manifest["event_log"]["parser_output_sha256"],
        "reference-values.json": manifest["reference_values"]["sha256"],
        "trusted-time.json": manifest["trusted_time"]["sha256"],
        "quote.msg": manifest["artifacts"]["quote_message_sha256"],
        "quote.sig": manifest["artifacts"]["quote_signature_sha256"],
        "ak.pub": manifest["quote"]["attestation_key_sha256"],
        "pcr-post.yaml": manifest["live_observation"]["pcr_post_artifact_sha256"],
        "nonce.bin": manifest["challenge"]["sha256"],
        "tool-versions.json": manifest["toolchain"]["observed_tool_versions_sha256"],
        "tss-version-evidence.txt": manifest["toolchain"]["tss_version_evidence_sha256"],
        "eventlog-reconstruction-input.json": manifest["reconstruction"]["input_sha256"],
        "observed-pcr-values.json": manifest["live_observation"]["pcr_values_file_sha256"],
        "ek-nv-index-handles.txt": manifest["ek_certificate_capture"]["inventory_sha256"],
        "ek-certificate-capture-transcript.json": manifest["ek_certificate_capture"]["transcript_sha256"],
        "ek-certificate-capture.json": manifest["ek_certificate_capture"]["result_sha256"],
        "raw-eventlog.json": manifest["raw_eventlog"]["output_sha256"],
        "payload-coherence.json": manifest["payload_coherence"]["output_sha256"],
        "ak-public-name-input.json": manifest["public_name_coherence"]["ak"]["input_sha256"],
        "ak-public-name-coherence.json": manifest["public_name_coherence"]["ak"]["output_sha256"],
        "ak.tpmt": manifest["public_name_coherence"]["ak"]["wire_sha256"],
        "ak.name.readpublic": manifest["public_name_coherence"]["ak"]["name_sha256"],
        "ak.qname.readpublic": manifest["public_name_coherence"]["ak"]["qname_sha256"],
        "ek-public-name-input.json": manifest["public_name_coherence"]["ek"]["input_sha256"],
        "ek-public-name-coherence.json": manifest["public_name_coherence"]["ek"]["output_sha256"],
        "ek.tpmt": manifest["public_name_coherence"]["ek"]["wire_sha256"],
        "ek.name.readpublic": manifest["public_name_coherence"]["ek"]["name_sha256"],
        "ek.qname.readpublic": manifest["public_name_coherence"]["ek"]["qname_sha256"],
        "eventlog-reconstruction.json": manifest["artifacts"]["reconstruction_file_sha256"],
    }
    for relative, expected in checks.items():
        path = bundle / relative
        if not path.is_file():
            print(f"PLATFORM EVIDENCE: DENY: missing-artifact-{relative}")
            return 1
        if sha256_file(path) != expected:
            print(f"PLATFORM EVIDENCE: DENY: artifact-digest-mismatch-{relative}")
            return 1

    cert_capture = manifest["ek_certificate_capture"]
    cert_capture_result = load_json(bundle / "ek-certificate-capture.json")
    if cert_capture_result.get("verifier_id") != EK_CERTIFICATE_CAPTURE_ID:
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-result-verifier-id-mismatch")
        return 1
    if cert_capture_result.get("state") != cert_capture["status"]:
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-result-state-mismatch")
        return 1
    if cert_capture_result.get("content_sha256") != self_hash(cert_capture_result, "content_sha256"):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-result-content-hash-mismatch")
        return 1
    if cert_capture_result.get("details", {}).get("source_policy", {}).get("mode") != "TPM_NV_ONLY":
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-not-nv-only")
        return 1
    if cert_capture_result.get("details", {}).get("source_policy", {}).get("network_url_present"):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-network-url-present")
        return 1
    if cert_capture_result.get("details", {}).get("source_policy", {}).get("explicit_network_option_present"):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-network-option-present")
        return 1
    if cert_capture.get("result_sha256") != sha256_file(bundle / "ek-certificate-capture.json"):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-result-digest-mismatch")
        return 1
    if cert_capture.get("inventory_sha256") != sha256_file(bundle / "ek-nv-index-handles.txt"):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-inventory-digest-mismatch")
        return 1
    if cert_capture.get("transcript_sha256") != sha256_file(bundle / "ek-certificate-capture-transcript.json"):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-transcript-digest-mismatch")
        return 1
    if cert_capture.get("script_sha256") != sha256_file(EK_CERTIFICATE_CAPTURE_SCRIPT):
        print("PLATFORM EVIDENCE: DENY: ek-certificate-capture-script-digest-mismatch")
        return 1

    if cert_capture.get("rsa_certificate_present"):
        if cert_capture.get("rsa_certificate_sha256") != sha256_file(bundle / "ek-cert-rsa.der"):
            print("PLATFORM EVIDENCE: DENY: ek-certificate-rsa-digest-mismatch")
            return 1
        spki_input = bundle / "ek-cert-spki-input.json"
        spki_output = bundle / "ek-cert-spki-binding.json"
        if not spki_input.is_file() or not spki_output.is_file():
            print("PLATFORM EVIDENCE: DENY: missing-ek-cert-spki-receipt")
            return 1
        spki_state, spki_reason = run_ek_cert_spki_verifier(
            spki_input, spki_output, bundle, os.environ.copy()
        )
        print(f"Independent EK certificate SPKI verifier: {spki_state} ({spki_reason})")
        if spki_state == "DENY":
            return 1
        spki_result = load_json(spki_output)
        if spki_result.get("verifier_id") != EK_CERT_SPki_VERIFIER_ID:
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-verifier-id-mismatch")
            return 1
        if spki_result.get("state") != manifest["ek_cert_spki_binding"]["status"]:
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-state-binding-mismatch")
            return 1
        if spki_result.get("certificate_der_sha256") != cert_capture["rsa_certificate_sha256"]:
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-certificate-capture-mismatch")
            return 1
        if spki_result.get("ek_public_wire_sha256") != manifest["public_name_coherence"]["ek"]["wire_sha256"]:
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-ek-wire-mismatch")
            return 1
        if spki_result.get("content_sha256") != self_hash(spki_result, "content_sha256"):
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-content-hash-mismatch")
            return 1
        if manifest["ek_cert_spki_binding"]["input_sha256"] != sha256_file(spki_input):
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-input-digest-mismatch")
            return 1
        if manifest["ek_cert_spki_binding"]["output_sha256"] != sha256_file(spki_output):
            print("PLATFORM EVIDENCE: DENY: ek-cert-spki-output-digest-mismatch")
            return 1
    else:
        if manifest["ek_cert_spki_binding"]["status"] != "INDETERMINATE":
            print("PLATFORM EVIDENCE: DENY: missing-rsa-certificate-must-be-indeterminate")
            return 1

    try:
        values = parse_pcrread_sha256(
            (bundle / "pcr-post.yaml").read_text(encoding="utf-8"),
            manifest["live_observation"]["selection"],
        )
    except ValueError as exc:
        print(f"PLATFORM EVIDENCE: DENY: invalid-pcr-post-state: {exc}")
        return 1

    if pcr_values_hash(values) != manifest["live_observation"]["pcr_values_sha256"]:
        print("PLATFORM EVIDENCE: DENY: pcr-values-state-hash-mismatch")
        return 1
    observed_file_values = load_json(bundle / "observed-pcr-values.json")
    if observed_file_values != values:
        print("PLATFORM EVIDENCE: DENY: observed-pcr-json-mismatch")
        return 1

    reconstruction_path = bundle / "eventlog-reconstruction.json"
    reconstruction = load_json(reconstruction_path)
    raw_eventlog = load_json(bundle / "raw-eventlog.json")
    raw_state, raw_reason = validate_raw_eventlog_result(raw_eventlog, manifest)
    print(f"Raw event-log validation: {raw_state} ({raw_reason})")
    if raw_state != "PASS":
        return 1 if raw_state == "DENY" else 2
    with tempfile.TemporaryDirectory(prefix="mycelix-independent-raw-eventlog-") as td:
        independent_raw_path = Path(td) / "raw-eventlog.json"
        independent_raw_state, independent_raw_reason = run_independent_raw_eventlog_parser(
            bundle / "eventlog.bin", independent_raw_path, bundle
        )
        print(
            f"Independent raw event-log parser execution: "
            f"{independent_raw_state} ({independent_raw_reason})"
        )
        if independent_raw_state != "PASS":
            return 1 if independent_raw_state == "DENY" else 2
        independent_raw = load_json(independent_raw_path)
        if independent_raw != raw_eventlog:
            print("PLATFORM EVIDENCE: DENY: supplied raw event-log result differs from independent execution")
            return 1
    if self_hash(reconstruction, "content_sha256") != reconstruction.get("content_sha256"):
        print("PLATFORM EVIDENCE: DENY: reconstruction-self-hash-mismatch")
        return 1

    input_path = bundle / "eventlog-reconstruction-input.json"
    if manifest["reconstruction"]["status"] != "PASS":
        print("PLATFORM EVIDENCE: DENY: reconstruction-not-qualified")
        return 1
    if not input_path.is_file():
        print("PLATFORM EVIDENCE: DENY: missing-eventlog-reconstruction-input.json")
        return 1

    result_state, result_reason = validate_reconstruction_result(reconstruction, manifest, values, input_path)
    print(f"Reconstruction result validation: {result_state} ({result_reason})")
    if result_state != "PASS":
        return 1 if result_state == "DENY" else 2

    with tempfile.TemporaryDirectory(prefix="mycelix-independent-reconstruction-") as td:
        independent_path = Path(td) / "reconstruction.json"
        independent_state, independent_reason = run_independent_reconstruction(input_path, independent_path, bundle)
        print(f"Independent reconstruction execution: {independent_state} ({independent_reason})")
        if independent_state != "PASS":
            return 1 if independent_state == "DENY" else 2
        independent = load_json(independent_path)
        if independent != reconstruction:
            print("PLATFORM EVIDENCE: DENY: supplied reconstruction differs from independent execution")
            return 1

    ek_template_input = bundle / "ek-template-input.json"
    ek_template_output = bundle / "ek-template-appraisal.json"
    ek_template_state, ek_template_reason = run_ek_template_verifier(
        ek_template_input, ek_template_output, bundle
    )
    print(f"Independent EK template appraisal: {ek_template_state} ({ek_template_reason})")
    if ek_template_state == "DENY":
        return 1
    ek_template_result = load_json(ek_template_output)
    if ek_template_result.get("verifier_id") != EK_TEMPLATE_VERIFIER_ID:
        print("PLATFORM EVIDENCE: DENY: ek-template-verifier-id-mismatch")
        return 1
    if ek_template_result.get("state") != manifest["ek_template_appraisal"]["status"]:
        print("PLATFORM EVIDENCE: DENY: ek-template-state-binding-mismatch")
        return 1
    if ek_template_result.get("input_sha256") != sha256_file(ek_template_input):
        print("PLATFORM EVIDENCE: DENY: ek-template-input-mismatch")
        return 1
    if ek_template_result.get("content_sha256") != self_hash(ek_template_result, "content_sha256"):
        print("PLATFORM EVIDENCE: DENY: ek-template-content-hash-mismatch")
        return 1

    for role in ("ak", "ek"):
        role_input = bundle / f"{role}-public-name-input.json"
        role_output = bundle / f"{role}-public-name-coherence.json"
        state, reason = run_public_name_verifier(role_input, role_output, bundle)
        print(f"Independent {role.upper()} public-area/name verifier: {state} ({reason})")
        if state == "DENY":
            return 1
        supplied = load_json(role_output)
        entry = manifest["public_name_coherence"][role]
        input_data = load_json(role_input)
        if supplied.get("verifier_id") != PUBLIC_NAME_VERIFIER_ID:
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-verifier-id-mismatch")
            return 1
        if supplied.get("state") not in {"PASS", "INDETERMINATE"}:
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-invalid-result-state")
            return 1
        if supplied.get("state") != entry["status"]:
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-state-binding-mismatch")
            return 1
        if supplied.get("input_sha256") != sha256_file(role_input):
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-input-mismatch")
            return 1
        if input_data.get("public_wire_sha256") != entry["wire_sha256"]:
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-wire-input-binding-mismatch")
            return 1
        if input_data.get("readpublic_source_sha256") != entry["source_sha256"]:
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-source-input-binding-mismatch")
            return 1
        details = supplied.get("details")
        if isinstance(details, dict) and details.get("public_area_sha256") != entry["wire_sha256"]:
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-result-wire-binding-mismatch")
            return 1
        if isinstance(details, dict) and details.get("name_hex") != input_data.get("name_hex"):
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-result-name-binding-mismatch")
            return 1
        if supplied.get("content_sha256") != self_hash(supplied, "content_sha256"):
            print(f"PLATFORM EVIDENCE: DENY: public-name-{role}-content-hash-mismatch")
            return 1

    payload_result = load_json(bundle / "payload-coherence.json")
    if payload_result.get("profile_id") != "mycelix.security.event-payload-digest-coherence":
        print("PLATFORM EVIDENCE: DENY: payload-coherence-profile-mismatch")
        return 1
    if payload_result.get("profile_version") != "0.1.0":
        print("PLATFORM EVIDENCE: DENY: payload-coherence-version-mismatch")
        return 1
    if payload_result.get("verifier_id") != "mycelix.pc-client.event-payload-digest-coherence.v0.1":
        print("PLATFORM EVIDENCE: DENY: payload-coherence-verifier-id-mismatch")
        return 1
    if payload_result.get("verifier_source_sha256") != sha256_file(PAYLOAD_COHERENCE_SCRIPT):
        print("PLATFORM EVIDENCE: DENY: payload-coherence-source-mismatch")
        return 1
    if payload_result.get("input_sha256") != sha256_file(input_path):
        print("PLATFORM EVIDENCE: DENY: payload-coherence-input-mismatch")
        return 1
    if payload_result.get("content_sha256") != self_hash(payload_result, "content_sha256"):
        print("PLATFORM EVIDENCE: DENY: payload-coherence-self-hash-mismatch")
        return 1
    payload_state = payload_result.get("state")
    if payload_state == "DENY":
        print("PLATFORM EVIDENCE: DENY: payload-coherence-failure")
        return 1
    if payload_state not in {"PASS", "INDETERMINATE"}:
        print("PLATFORM EVIDENCE: DENY: payload-coherence-state-invalid")
        return 1
    if manifest["payload_coherence"]["status"] != payload_state:
        print("PLATFORM EVIDENCE: DENY: payload-coherence-manifest-state-mismatch")
        return 1

    quote_state, quote_reason = run_quote_check(bundle)
    print(f"TPM Quote verification: {quote_state} ({quote_reason})")
    if quote_state != "PASS":
        return 2 if quote_state == "INDETERMINATE" else 1

    print("PLATFORM EVIDENCE: PASS")
    print("Claim ceiling: ReferenceModelOnly")
    if reconstruction.get("status") == "DENY":
        return 1
    if reconstruction.get("status") == "INDETERMINATE":
        return 2
    return 0


def detect_device() -> Path | None:
    for candidate in (Path("/dev/tpmrm0"), Path("/dev/tpm0")):
        if candidate.exists():
            return candidate
    return None


def detect_event_log() -> Path | None:
    candidate = Path("/sys/kernel/security/tpm0/binary_bios_measurements")
    return candidate if candidate.is_file() and os.access(candidate, os.R_OK) else None


def require_capture_tools() -> list[str]:
    names = (
        "tpm2_getcap",
        "tpm2_pcrread",
        "tpm2_quote",
        "tpm2_createek",
        "tpm2_createak",
        "tpm2_checkquote",
        "tpm2_eventlog",
        "tpm2_readpublic",
        "tpm2_getekcertificate",
    )
    return [name for name in names if shutil.which(name) is None]


def observed_tool_versions(out: Path, env: dict[str, str]) -> dict[str, str]:
    names = (
        "tpm2_getcap",
        "tpm2_pcrread",
        "tpm2_quote",
        "tpm2_createek",
        "tpm2_createak",
        "tpm2_checkquote",
        "tpm2_eventlog",
        "tpm2_readpublic",
        "tpm2_getekcertificate",
    )
    result: dict[str, str] = {}
    for name in names:
        proc = run([name, "--version"], env, out, check=False)
        lines = (proc.stdout + proc.stderr).strip().splitlines()
        if not lines:
            raise RuntimeError(f"no version output for {name}")
        result[name] = lines[0]
    return result


def capture_public_name_observation(
    role: str,
    context_path: Path,
    public_artifact: Path,
    out: Path,
    env: dict[str, str],
    tool_version: str,
) -> dict[str, str]:
    wire_path = out / f"{role}.tpmt"
    name_path = out / f"{role}.name.readpublic"
    qname_path = out / f"{role}.qname.readpublic"
    transcript_path = out / f"{role}.readpublic-transcript.json"
    input_path = out / f"{role}-public-name-input.json"
    result_path = out / f"{role}-public-name-coherence.json"

    argv = [
        "tpm2_readpublic",
        "-Q",
        "-c",
        str(context_path),
        "-f",
        "tpmt",
        "-o",
        str(wire_path),
        "-n",
        str(name_path),
        "-q",
        str(qname_path),
    ]
    proc = run(argv, env, out, check=False)
    transcript = {
        "role": role,
        "command": argv,
        "tool_version": tool_version,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }
    transcript_path.write_text(
        json.dumps(transcript, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if proc.returncode != 0:
        raise RuntimeError(f"tpm2_readpublic failed for {role}: {proc.stderr}")
    for path in (wire_path, name_path, qname_path):
        if not path.is_file():
            raise RuntimeError(f"tpm2_readpublic did not produce {path.name}")

    public_body = normalized_tpm2b_public_body(public_artifact)
    if public_body != wire_path.read_bytes():
        raise RuntimeError(
            f"{role} ReadPublic TPMT_PUBLIC does not match the public artifact used by attestation"
        )

    public_name_input = {
        "profile_id": "mycelix.security.tpm.public-name-coherence",
        "profile_version": "0.1.0",
        "verification_mode": "LiveVerifierSession",
        "claim_ceiling": "ReferenceModelOnly",
        "object_role": role.upper(),
        "public_format": "TPMT_PUBLIC",
        "public_wire_hex": wire_path.read_bytes().hex(),
        "public_wire_sha256": sha256_file(wire_path),
        "name_hex": name_path.read_bytes().hex(),
        "readpublic_state": "PASS",
        "readpublic_source_sha256": sha256_file(transcript_path),
    }
    input_path.write_text(
        json.dumps(public_name_input, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    verifier = run(
        [
            sys.executable,
            str(PUBLIC_NAME_VERIFIER_SCRIPT),
            "--verify",
            str(input_path),
            "--output",
            str(result_path),
        ],
        env,
        out,
        check=False,
    )
    if verifier.returncode not in (0, 2):
        raise RuntimeError(
            f"public-name verifier failed for {role}: {verifier.stdout}{verifier.stderr}"
        )
    result = load_json(result_path)
    if result.get("state") not in {"PASS", "INDETERMINATE"}:
        raise RuntimeError(
            f"public-name verifier returned DENY for {role}: {result.get('reason')}"
        )
    return {
        "status": result["state"],
        "verifier_id": result["verifier_id"],
        "output_sha256": sha256_file(result_path),
        "input_sha256": sha256_file(input_path),
        "wire_sha256": sha256_file(wire_path),
        "name_sha256": sha256_file(name_path),
        "qname_sha256": sha256_file(qname_path),
        "source_sha256": sha256_file(transcript_path),
    }


def capture_ek_template_observation(
    ek_context: Path,
    ek_public_wire: Path,
    ek_name: Path,
    ek_qname: Path,
    create_transcript_path: Path,
    out: Path,
    env: dict[str, str],
) -> dict[str, str]:
    input_path = out / "ek-template-input.json"
    result_path = out / "ek-template-appraisal.json"

    public_input = {
        "profile_id": "mycelix.security.tpm.ek-template-appraisal",
        "profile_version": "0.1.0",
        "verification_mode": "LiveVerifierSession",
        "claim_ceiling": "ReferenceModelOnly",
        "object_role": "EK",
        "public_format": "TPMT_PUBLIC",
        "public_wire_hex": ek_public_wire.read_bytes().hex(),
        "public_wire_sha256": sha256_file(ek_public_wire),
        "name_hex": ek_name.read_bytes().hex(),
        "qualified_name_hex": ek_qname.read_bytes().hex(),
        "creation_provenance": {
            "state": "PASS",
            "tool_id": "tpm2_createek",
            "hierarchy": "TPM_RH_ENDORSEMENT",
            "template_mode": "default-low-range",
            "transcript_sha256": sha256_file(create_transcript_path),
        },
    }
    input_path.write_text(
        json.dumps(public_input, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    proc = run(
        [
            sys.executable,
            str(EK_TEMPLATE_VERIFIER_SCRIPT),
            "--verify",
            str(input_path),
            "--output",
            str(result_path),
        ],
        env,
        out,
        check=False,
    )
    if proc.returncode not in (0, 2):
        raise RuntimeError(
            f"EK template verifier failed: {proc.stdout}{proc.stderr}"
        )
    verified = load_json(result_path)
    if verified.get("state") not in {"PASS", "INDETERMINATE"}:
        raise RuntimeError(
            f"EK template verifier returned DENY: {verified.get('reason')}"
        )
    return {
        "status": verified["state"],
        "verifier_id": verified["verifier_id"],
        "output_sha256": sha256_file(result_path),
        "input_sha256": sha256_file(input_path),
        "wire_sha256": sha256_file(ek_public_wire),
        "source_sha256": sha256_file(EK_TEMPLATE_VERIFIER_SCRIPT),
        "transcript_sha256": sha256_file(create_transcript_path),
    }

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
    if not args.tss_version_evidence_file or not Path(args.tss_version_evidence_file).is_file():
        blockers.append("tss-version-evidence.txt not supplied")
    if not args.os_image_digest or not args.workload_digest:
        blockers.append("exact OS-image and workload digests not supplied")
    if args.pcr_selection != "sha256:0,2,4,7":
        blockers.append("qualified PCR selection is fixed to sha256:0,2,4,7")

    if blockers:
        print("TPM PLATFORM CAPTURE: NOT EXECUTED")
        for blocker in blockers:
            print("- " + blocker)
        return 2

    out = Path(args.output).resolve() if args.output else Path(
        tempfile.mkdtemp(prefix="mycelix-platform-capture-")
    )
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["TPM2TOOLS_TCTI"] = f"device:{device}"

    boot_before = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()

    for source, destination in (
        (args.reference_values, "reference-values.json"),
        (args.trusted_time, "trusted-time.json"),
        (args.nonce_file, "nonce.bin"),
        (args.tss_version_evidence_file, "tss-version-evidence.txt"),
    ):
        shutil.copy2(source, out / destination)
    shutil.copy2(event_log, out / "eventlog.bin")
    props = run(["tpm2_getcap", "properties-fixed"], env, out)
    (out / "tpm-properties.txt").write_text(props.stdout, encoding="utf-8")
    prop_hash = sha256_file(out / "tpm-properties.txt")

    versions = observed_tool_versions(out, env)
    (out / "tool-versions.json").write_text(
        json.dumps(versions, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if any(not version.startswith("5.8") for version in versions.values()):
        raise RuntimeError("observed tpm2-tools version mismatch; expected 5.8")

    pre = run(["tpm2_pcrread", args.pcr_selection], env, out)
    (out / "pcr-pre.yaml").write_text(pre.stdout, encoding="utf-8")

    run(
        ["tpm2_createek", "-Q", "-c", str(out / "ek.ctx"), "-G", "rsa", "-u", str(out / "ek.pub")],
        env,
        out,
    )
    run(
        [
            "tpm2_createak", "-Q", "-C", str(out / "ek.ctx"),
            "-c", str(out / "ak.ctx"), "-G", "rsa", "-g", "sha256",
            "-s", "rsassa", "-u", str(out / "ak.pub"), "-n", str(out / "ak.name"),
        ],
        env,
        out,
    )

    public_name_ak = capture_public_name_observation(
        "ak", out / "ak.ctx", out / "ak.pub", out, env, versions["tpm2_readpublic"]
    )
    public_name_ek = capture_public_name_observation(
        "ek", out / "ek.ctx", out / "ek.pub", out, env, versions["tpm2_readpublic"]
    )

    ek_template_appraisal = capture_ek_template_observation(
        out / "ek.ctx",
        out / "ek.tpmt",
        out / "ek.name.readpublic",
        out / "ek.qname.readpublic",
        out / "ek-create-transcript.json",
        out,
        env,
    )

    nonce = (out / "nonce.bin").read_bytes()
    if not nonce:
        raise RuntimeError("external verifier nonce is empty")
    run(
        [
            "tpm2_quote", "-Q", "-c", str(out / "ak.ctx"), "-l", args.pcr_selection,
            "-q", nonce.hex(), "-m", str(out / "quote.msg"), "-s", str(out / "quote.sig"),
            "-g", "sha256",
        ],
        env,
        out,
    )

    post = run(["tpm2_pcrread", args.pcr_selection], env, out)
    (out / "pcr-post.yaml").write_text(post.stdout, encoding="utf-8")
    live_values = parse_pcrread_sha256(post.stdout, args.pcr_selection)
    (out / "observed-pcr-values.json").write_text(
        json.dumps(live_values, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    props_after = run(["tpm2_getcap", "properties-fixed"], env, out)
    if sha256_bytes(props_after.stdout.encode()) != prop_hash:
        raise RuntimeError("TPM fixed properties changed during capture")
    boot_after = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
    if boot_after != boot_before:
        raise RuntimeError("OS boot_id changed during capture")

    session_id = f"linuxboot-{boot_before}-{sha256_file(out / 'eventlog.bin')[:16]}"
    parsed = run(
        ["tpm2_eventlog", "--eventlog-version=1", str(out / "eventlog.bin")],
        env,
        out,
        check=False,
    )
    (out / "eventlog-parsed.yaml").write_text(
        parsed.stdout + parsed.stderr, encoding="utf-8"
    )
    if parsed.returncode != 0:
        raise RuntimeError("tpm2_eventlog parser failed; raw evidence preserved but not qualified")

    raw_eventlog_path = out / "raw-eventlog.json"
    raw_parse = run(
        [sys.executable, str(RAW_EVENTLOG_PARSER_SCRIPT), "--parse", str(out / "eventlog.bin"), "--output", str(raw_eventlog_path)],
        env, out, check=False,
    )
    if raw_parse.returncode != 0:
        raise RuntimeError("independent raw event-log parser failed; raw evidence preserved but not qualified")
    trusted = load_json(out / "trusted-time.json")
    input_path = out / "eventlog-reconstruction-input.json"

    reconstruction_path = out / "eventlog-reconstruction.json"
    adapter = run(
        [
            sys.executable,
            str(ADAPTER_SCRIPT),
            "--adapt",
            str(out / "eventlog-parsed.yaml"),
            "--binary-eventlog",
            str(out / "eventlog.bin"),
            "--observed-pcr-json",
            str(out / "observed-pcr-values.json"),
            "--payload-json",
            str(raw_eventlog_path),
            "--session-id",
            session_id,
            "--pcr-selection",
            args.pcr_selection,
            "--output",
            str(input_path),
        ],
        env,
        out,
        check=False,
    )
    if adapter.returncode == 0:
        replay = run(
            [
                sys.executable,
                str(RECONSTRUCTION_SCRIPT),
                "--reconstruct",
                str(input_path),
                "--output",
                str(reconstruction_path),
            ],
            env,
            out,
            check=False,
        )
        if reconstruction_path.is_file():
            reconstruction = load_json(reconstruction_path)
        else:
            reconstruction = {
                "status": "DENY",
                "reason": "reconstruction-executable-produced-no-result",
                "event_log_sha256": sha256_file(out / "eventlog.bin"),
                "pcr_selection": args.pcr_selection,
                "verifier_id": RECONSTRUCTION_VERIFIER_ID,
                "verifier_source_sha256": sha256_file(RECONSTRUCTION_SCRIPT),
                "input_sha256": sha256_file(input_path),
                "reconstructed_pcrs_sha256": "",
            }
            reconstruction["content_sha256"] = self_hash(reconstruction, "content_sha256")
            reconstruction_path.write_text(
                json.dumps(reconstruction, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
        if replay.returncode not in (0, 1, 2):
            raise RuntimeError("independent event-log reconstruction process failed")
        payload_coherence_path = out / "payload-coherence.json"
        payload_proc = run(
            [
                sys.executable,
                str(PAYLOAD_COHERENCE_SCRIPT),
                "--verify",
                str(input_path),
                "--output",
                str(payload_coherence_path),
            ],
            env,
            out,
            check=False,
        )
        if payload_proc.returncode not in (0, 2) and not payload_coherence_path.is_file():
            raise RuntimeError("payload coherence verifier failed without producing a result")
        payload_coherence = load_json(payload_coherence_path)
    else:
        input_path.write_text(
            json.dumps({
                "status": "INDETERMINATE",
                "reason": "eventlog-yaml-adapter-failed",
                "session_id": session_id,
                "event_log_sha256": sha256_file(out / "eventlog.bin"),
                "pcr_selection": args.pcr_selection,
            }, sort_keys=True, indent=2) + "\n",
            encoding="utf-8",
        )
        reconstruction = {
            "status": "INDETERMINATE",
            "reason": "eventlog-yaml-adapter-failed",
            "event_log_sha256": sha256_file(out / "eventlog.bin"),
            "pcr_selection": args.pcr_selection,
            "verifier_id": "external-reconstruction-required",
            "verifier_source_sha256": "",
            "input_sha256": sha256_file(input_path),
            "reconstructed_pcrs_sha256": "",
        }
        reconstruction["content_sha256"] = self_hash(reconstruction, "content_sha256")
        reconstruction_path.write_text(
            json.dumps(reconstruction, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        payload_coherence_path = out / "payload-coherence.json"
        payload_coherence = {
            "profile_id": "mycelix.security.event-payload-digest-coherence",
            "profile_version": "0.1.0",
            "verifier_id": "mycelix.pc-client.event-payload-digest-coherence.v0.1",
            "verifier_source_sha256": sha256_file(PAYLOAD_COHERENCE_SCRIPT),
            "input_sha256": sha256_file(input_path),
            "event_count": 0,
            "event_results": [],
            "state": "INDETERMINATE",
            "reason": "canonical-input-unavailable",
        }
        payload_coherence["content_sha256"] = self_hash(payload_coherence, "content_sha256")
        payload_coherence_path.write_text(
            json.dumps(payload_coherence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    ek_hash = sha256_file(out / "ek.pub")
    manifest: dict[str, Any] = {
        "profile_id": "mycelix.security.platform.evidence.capture",
        "profile_version": "0.1.0",
        "session_id": session_id,
        "boot_id": boot_before,
        "tpm": {
            "device_path": str(device),
            "properties_sha256": prop_hash,
            "ek_public_sha256": ek_hash,
            "identity_digest": "",
        },
        "event_log": {
            "sha256": sha256_file(out / "eventlog.bin"),
            "parser_profile_id": "tcg.pc-client.event-log",
            "parser_profile_version": "1.0",
            "parser_status": "PASS",
            "parser_output_sha256": sha256_file(out / "eventlog-parsed.yaml"),
        },
        "raw_eventlog": {
            "status": "PASS",
            "parser_id": "mycelix.pc-client.raw-tpm2-eventlog-parser.v0.1",
            "output_sha256": sha256_file(raw_eventlog_path),
            "source_sha256": sha256_file(RAW_EVENTLOG_PARSER_SCRIPT),
            "binary_sha256": sha256_file(out / "eventlog.bin"),
        },
        "payload_coherence": {
            "status": payload_coherence.get("state", "DENY"),
            "output_sha256": sha256_file(payload_coherence_path),
            "source_sha256": sha256_file(PAYLOAD_COHERENCE_SCRIPT),
            "input_sha256": sha256_file(input_path),
        },
        "quote": {
            "pcr_selection": args.pcr_selection,
            "nonce_sha256": sha256_file(out / "nonce.bin"),
            "attestation_key_sha256": sha256_file(out / "ak.pub"),
        },
        "challenge": {
            "sha256": sha256_file(out / "nonce.bin"),
            "origin": "external-verifier-supplied",
        },
        "toolchain": {
            "tpm2_tools_version": "5.8",
            "observed_tool_versions_sha256": sha256_file(out / "tool-versions.json"),
            "tss_version_evidence_sha256": sha256_file(out / "tss-version-evidence.txt"),
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
            "selection": args.pcr_selection,
            "pcr_post_artifact_sha256": sha256_file(out / "pcr-post.yaml"),
            "pcr_values_sha256": pcr_values_hash(live_values),
            "pcr_values_file_sha256": sha256_file(out / "observed-pcr-values.json"),
        },
        "artifacts": {
            "quote_message_sha256": sha256_file(out / "quote.msg"),
            "quote_signature_sha256": sha256_file(out / "quote.sig"),
            "attestation_key_sha256": sha256_file(out / "ak.pub"),
            "reconstruction_file_sha256": sha256_file(out / "eventlog-reconstruction.json"),
            "reconstruction_input_sha256": sha256_file(out / "eventlog-reconstruction-input.json"),
            "observed_pcr_values_file_sha256": sha256_file(out / "observed-pcr-values.json"),
            "raw_eventlog_output_sha256": sha256_file(raw_eventlog_path),
            "tss_version_evidence_sha256": sha256_file(out / "tss-version-evidence.txt"),
            "ek_public_sha256": ek_hash,
        },
        "os_image_digest": args.os_image_digest,
        "workload_digest": args.workload_digest,
        "capture_status": "CAPTURED_RAW_EVIDENCE",
        "claim_ceiling": "ReferenceModelOnly",
    }

    manifest["tpm"]["identity_digest"] = canonical_hash(
        {
            "device_path": manifest["tpm"]["device_path"],
            "properties_sha256": manifest["tpm"]["properties_sha256"],
            "ek_public_sha256": manifest["tpm"]["ek_public_sha256"],
        }
    )
    manifest["session_binding_sha256"] = session_binding(manifest)
    (out / "capture-session.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    print(f"Raw platform Evidence captured to: {out}")
    print("Capture status: CAPTURED_RAW_EVIDENCE")
    print("Qualification status: NOT QUALIFIED")
    print("Event-log parser: PASS")
    print(f"Event-log reconstruction: {reconstruction.get('status', 'INDETERMINATE')}")
    print(f"Event-log reconstruction reason: {reconstruction.get('reason', 'unknown')}")
    print(f"Payload coherence: {payload_coherence.get('state', 'DENY')} ({payload_coherence.get('reason', 'unknown')})")
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
    parser.add_argument("--tss-version-evidence-file")
    parser.add_argument("--os-image-digest")
    parser.add_argument("--workload-digest")
    parser.add_argument("--pcr-selection", default="sha256:0,2,4,7")
    args = parser.parse_args()

    contract = load_json(CONTRACT)
    if args.self_test:
        return self_test(contract)
    if args.verify:
        return verify_bundle(args)
    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
