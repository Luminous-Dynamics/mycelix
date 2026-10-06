#!/usr/bin/env python3
"""Verify Mycelix TPM AK -> EK lineage evidence under an explicit claim boundary."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ak-ek-lineage.v0.1"
ATTRIBUTES_VERIFIER_ID = "mycelix.tpm.ak-public-attributes.v0.1"
ATTRIBUTES_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_ak_public_attributes_v0_1.py")
PUBLIC_NAME_VERIFIER_ID = "mycelix.tpm.public-name-coherence.v0.1"
PUBLIC_NAME_VERIFIER_SCRIPT = Path(__file__).with_name("verify_mycelix_tpm_public_name_coherence_v0_1.py")
SHA256_NAME_ALG = "sha256"
SHA256_ALG_ID = bytes.fromhex("000b")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()


def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        char in "0123456789abcdef" for char in value
    )


def normalize_hex(value: Any, field: str) -> bytes:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a hex string")
    normalized = value.lower().removeprefix("0x")
    if len(normalized) % 2 or any(char not in "0123456789abcdef" for char in normalized):
        raise ValueError(f"{field} is not canonical hexadecimal")
    return bytes.fromhex(normalized)


def validate_name(value: Any, field: str) -> bytes:
    raw = normalize_hex(value, field)
    if len(raw) != 34 or raw[:2] != SHA256_ALG_ID:
        raise ValueError(f"{field} must be a SHA-256 TPM Name")
    return raw


def validate_qname(value: Any, field: str) -> bytes:
    raw = normalize_hex(value, field)
    if len(raw) != 34 or raw[:2] != SHA256_ALG_ID:
        raise ValueError(f"{field} must be a SHA-256 Qualified Name")
    return raw


def expected_qname(parent_qname: bytes, object_name: bytes) -> bytes:
    return SHA256_ALG_ID + hashlib.sha256(parent_qname + object_name).digest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_public_name_verifier(binding: dict[str, Any]) -> dict[str, Any]:
    verifier_input = binding.get("verifier_input")
    if not isinstance(verifier_input, dict):
        return result("DENY", "ak-public-name-verifier-input-missing")
    if not PUBLIC_NAME_VERIFIER_SCRIPT.is_file():
        return result("DENY", "ak-public-name-verifier-missing")
    with tempfile.TemporaryDirectory(prefix="mycelix-public-name-") as td:
        root = Path(td)
        input_path = root / "public-name-input.json"
        output_path = root / "public-name-output.json"
        input_path.write_text(
            json.dumps(verifier_input, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(PUBLIC_NAME_VERIFIER_SCRIPT),
                "--verify",
                str(input_path),
                "--output",
                str(output_path),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode not in (0, 1, 2):
            return result("DENY", "ak-public-name-verifier-execution-error", {"stderr": proc.stderr})
        if not output_path.is_file():
            return result("DENY", "ak-public-name-verifier-produced-no-output")
        try:
            generated = json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "ak-public-name-verifier-output-invalid", {"error": str(exc)})
        if generated.get("verifier_id") != PUBLIC_NAME_VERIFIER_ID:
            return result("DENY", "ak-public-name-result-verifier-id-mismatch")
        expected_input_sha = sha256_file(input_path)
        expected_output_sha = sha256_file(output_path)
        if binding.get("input_sha256") != expected_input_sha:
            return result("DENY", "ak-public-name-verifier-input-digest-mismatch")
        if binding.get("output_sha256") != expected_output_sha:
            return result("DENY", "ak-public-name-verifier-output-digest-mismatch")
        return generated


def run_public_attributes_verifier(binding: dict[str, Any]) -> dict[str, Any]:
    verifier_input = binding.get("verifier_input")
    if not isinstance(verifier_input, dict):
        return result("DENY", "ak-public-attributes-verifier-input-missing")
    if not ATTRIBUTES_VERIFIER_SCRIPT.is_file():
        return result("DENY", "ak-public-attributes-verifier-missing")
    with tempfile.TemporaryDirectory(prefix="mycelix-ak-attributes-") as td:
        root = Path(td)
        input_path = root / "attributes-input.json"
        output_path = root / "attributes-output.json"
        input_path.write_text(
            json.dumps(verifier_input, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [sys.executable, str(ATTRIBUTES_VERIFIER_SCRIPT), "--verify", str(input_path), "--output", str(output_path)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode not in (0, 2):
            return result("DENY", "ak-public-attributes-verifier-failed", {"stderr": proc.stderr})
        if not output_path.is_file():
            return result("DENY", "ak-public-attributes-verifier-produced-no-output")
        try:
            generated = json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return result("DENY", "ak-public-attributes-verifier-output-invalid", {"error": str(exc)})
        if generated.get("verifier_id") != ATTRIBUTES_VERIFIER_ID:
            return result("DENY", "ak-public-attributes-result-verifier-id-mismatch")
        return generated


def result(
    state: str,
    reason: str,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    value: dict[str, Any] = {
        "verifier_id": VERIFIER_ID,
        "state": state,
        "reason": reason,
    }
    if details:
        value["details"] = details
    return value


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id",
        "profile_version",
        "verification_mode",
        "claim_ceiling",
        "session_id",
        "tpm_identity_digest",
        "ek",
        "ak",
        "parentage",
        "public_name_binding",
        "public_attributes_binding",
        "ek_credential",
        "credential_activation",
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY", "missing-required-fields", {"fields": missing})

    if manifest["profile_id"] != "mycelix.security.tpm.ak-ek-lineage":
        return result("DENY", "profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY", "profile-version-mismatch")
    if manifest["verification_mode"] not in {
        "ReferenceModelOnly",
        "OfflineBundle",
        "LiveVerifierSession",
    }:
        return result("DENY", "verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY", "claim-ceiling-mismatch")
    if not valid_hash(manifest["tpm_identity_digest"]):
        return result("DENY", "tpm-identity-digest-invalid")

    ek = manifest["ek"]
    ak = manifest["ak"]
    parentage = manifest["parentage"]
    public_name = manifest["public_name_binding"]
    credential = manifest["ek_credential"]
    activation = manifest["credential_activation"]

    if not isinstance(ek, dict) or not isinstance(ak, dict):
        return result("DENY", "key-sections-invalid")
    if not isinstance(parentage, dict) or not isinstance(public_name, dict):
        return result("DENY", "lineage-binding-sections-invalid")
    if not isinstance(credential, dict) or not isinstance(activation, dict):
        return result("DENY", "credential-sections-invalid")

    for section_name, section, fields in (
        ("ek", ek, ("public_sha256", "name_hex", "qualified_name_hex")),
        (
            "ak",
            ak,
            (
                "public_sha256",
                "public_area_sha256",
                "name_hex",
                "qualified_name_hex",
                "name_alg",
            ),
        ),
    ):
        for field in fields:
            if field not in section:
                return result(
                    "DENY",
                    "missing-field",
                    {"field": f"{section_name}.{field}"},
                )

    if not valid_hash(ek["public_sha256"]) or not valid_hash(ak["public_sha256"]):
        return result("DENY", "key-public-digest-invalid")
    if ak["name_alg"] != SHA256_NAME_ALG:
        return result("DENY", "unsupported-ak-name-algorithm")

    public_name = manifest["public_name_binding"]
    if not isinstance(public_name, dict):
        return result("DENY", "public-name-binding-invalid")
    for field in (
        "state", "method", "verifier_id", "public_sha256", "name_sha256",
        "public_area_sha256", "source_sha256", "input_sha256", "output_sha256",
        "verifier_input",
    ):
        if field not in public_name:
            return result("DENY", "missing-field", {"field": f"public_name_binding.{field}"})
    if public_name["verifier_id"] != PUBLIC_NAME_VERIFIER_ID:
        return result("DENY", "ak-public-name-verifier-id-mismatch")
    if public_name["state"] not in {"PASS", "INDETERMINATE"}:
        return result("DENY", "public-name-state-invalid")
    for field in ("public_sha256", "name_sha256", "public_area_sha256", "source_sha256", "input_sha256", "output_sha256"):
        if not valid_hash(public_name[field]):
            return result("DENY", "public-name-digest-invalid", {"field": field})
    if public_name["source_sha256"] != sha256_file(PUBLIC_NAME_VERIFIER_SCRIPT):
        return result("DENY", "public-name-verifier-source-mismatch")
    if public_name["method"] != "same-tpm-readpublic-context":
        return result("DENY", "ak-public-name-binding-method-invalid")
    if public_name["state"] == "INDETERMINATE":
        return result("INDETERMINATE", "ak-public-name-binding-indeterminate")
    generated_public_name = run_public_name_verifier(public_name)
    if generated_public_name.get("verifier_id") != PUBLIC_NAME_VERIFIER_ID:
        return generated_public_name
    if generated_public_name.get("state") != "PASS":
        return result("DENY", "ak-public-name-reexecution-not-pass")
    public_name_details = generated_public_name.get("details")
    if not isinstance(public_name_details, dict):
        return result("DENY", "ak-public-name-result-details-missing")
    if public_name_details.get("public_area_sha256") != public_name["public_area_sha256"]:
        return result("DENY", "ak-public-name-result-area-mismatch")
    if public_name_details.get("public_area_sha256") != manifest["ak"]["public_area_sha256"]:
        return result("DENY", "ak-public-name-result-ak-area-mismatch")
    if public_name_details.get("name_hex") != manifest["ak"]["name_hex"]:
        return result("DENY", "ak-public-name-result-name-mismatch")

    attributes_binding = manifest["public_attributes_binding"]
    if not isinstance(attributes_binding, dict):
        return result("DENY", "public-attributes-binding-invalid")
    for field in ("state", "verifier_id", "public_area_sha256", "derived_fixedTPM", "derived_fixedParent", "source_sha256", "verifier_input"):
        if field not in attributes_binding:
            return result("DENY", "missing-field", {"field": f"public_attributes_binding.{field}"})
    if attributes_binding["verifier_id"] != ATTRIBUTES_VERIFIER_ID:
        return result("DENY", "public-attributes-verifier-id-mismatch")
    if attributes_binding["state"] not in {"PASS", "INDETERMINATE"}:
        return result("DENY", "public-attributes-state-invalid")
    for field in ("public_area_sha256", "source_sha256"):
        if not valid_hash(attributes_binding[field]):
            return result("DENY", "public-attributes-digest-invalid", {"field": field})
    if attributes_binding["source_sha256"] != sha256_file(ATTRIBUTES_VERIFIER_SCRIPT):
        return result("DENY", "public-attributes-verifier-source-mismatch")
    generated_attributes = run_public_attributes_verifier(attributes_binding)
    if generated_attributes.get("verifier_id") != ATTRIBUTES_VERIFIER_ID:
        return generated_attributes
    if generated_attributes.get("state") != attributes_binding["state"]:
        return result("DENY", "public-attributes-result-state-mismatch")
    if generated_attributes.get("state") == "DENY":
        return result("DENY", "ak-public-attributes-verification-denied")
    if generated_attributes.get("state") == "INDETERMINATE":
        return result("INDETERMINATE", "ak-public-attributes-verification-indeterminate")
    details = generated_attributes.get("details")
    if not isinstance(details, dict):
        return result("DENY", "ak-public-attributes-details-missing")
    if attributes_binding["derived_fixedTPM"] is not True or attributes_binding["derived_fixedParent"] is not True:
        return result("DENY", "public-attributes-derived-fixed-bits-not-set")
    if details.get("fixedTPM") is not True or details.get("fixedParent") is not True:
        return result("DENY", "public-attributes-derived-fixed-bits-mismatch")
    if details.get("public_area_sha256") != attributes_binding["public_area_sha256"]:
        return result("DENY", "public-attributes-area-digest-mismatch")
    if details.get("public_area_sha256") != ak["public_area_sha256"]:
        return result("DENY", "public-attributes-ak-area-digest-mismatch")
    if details.get("name_hex") != ak["name_hex"]:
        return result("DENY", "public-attributes-name-mismatch")

    try:
        ek_name = validate_name(ek["name_hex"], "ek.name_hex")
        ek_qname = validate_qname(ek["qualified_name_hex"], "ek.qualified_name_hex")
        ak_name = validate_name(ak["name_hex"], "ak.name_hex")
        ak_qname = validate_qname(ak["qualified_name_hex"], "ak.qualified_name_hex")
        parent_qname = validate_qname(
            parentage["parent_qualified_name_hex"],
            "parentage.parent_qualified_name_hex",
        )
    except ValueError as exc:
        return result(
            "DENY",
            "invalid-tpm-name-encoding",
            {"error": str(exc)},
        )

    if parentage.get("parent_type") != "EK":
        return result("DENY", "parent-type-is-not-ek")
    if parent_qname != ek_qname:
        return result("DENY", "parent-qualified-name-does-not-equal-ek")

    expected = expected_qname(parent_qname, ak_name)
    if ak_qname != expected:
        return result(
            "DENY",
            "ak-qualified-name-parentage-mismatch",
            {
                "expected_qualified_name_hex": expected.hex(),
                "observed_qualified_name_hex": ak_qname.hex(),
            },
        )

    if public_name.get("public_sha256") != ak["public_sha256"]:
        return result("DENY", "ak-public-name-binding-public-digest-mismatch")
    if not valid_hash(public_name.get("public_sha256")) or public_name["public_sha256"] != ak["public_sha256"]:
        return result("DENY", "ak-public-name-binding-public-digest-mismatch")

    try:
        recorded_name_sha256 = public_name["name_sha256"]
        if not valid_hash(recorded_name_sha256):
            return result("DENY", "ak-public-name-binding-name-digest-invalid")
        if recorded_name_sha256 != hashlib.sha256(ak_name).hexdigest():
            return result("DENY", "ak-public-name-binding-name-digest-mismatch")
    except KeyError:
        return result(
            "DENY",
            "missing-field",
            {"field": "public_name_binding.name_sha256"},
        )

    if public_name.get("method") != "same-tpm-readpublic-context":
        return result("DENY", "ak-public-name-binding-method-invalid")
    if public_name.get("state") != "PASS":
        if public_name.get("state") == "INDETERMINATE":
            return result("INDETERMINATE", "ak-public-name-binding-indeterminate")
        return result("DENY", "ak-public-name-binding-failed")

    for field in ("material_sha256", "bound_ek_public_sha256"):
        if not valid_hash(credential.get(field)):
            return result(
                "DENY",
                "ek-credential-digest-invalid",
                {"field": field},
            )
    if credential["bound_ek_public_sha256"] != ek["public_sha256"]:
        return result("DENY", "ek-credential-not-bound-to-observed-ek")
    if credential.get("mode") not in {"provider", "tpm-resident"}:
        return result("DENY", "ek-credential-mode-invalid")
    if credential.get("state") == "DENY":
        return result("DENY", "ek-credential-appraisal-denied")
    if credential.get("state") == "INDETERMINATE":
        return result("INDETERMINATE", "ek-credential-appraisal-indeterminate")
    if credential.get("state") != "PASS":
        return result("DENY", "ek-credential-state-invalid")

    activation_required = {
        "protocol",
        "state",
        "scope",
        "session_id",
        "tpm_identity_digest",
        "challenge_origin",
        "challenge_sha256",
        "credential_blob_sha256",
        "ek_public_sha256",
        "ak_name_hex",
    }
    missing_activation = sorted(activation_required - set(activation))
    if missing_activation:
        return result(
            "DENY",
            "missing-activation-fields",
            {"fields": missing_activation},
        )
    if activation["protocol"] != "TPM2_MakeCredential+TPM2_ActivateCredential":
        return result("DENY", "credential-activation-protocol-mismatch")
    if activation["session_id"] != manifest["session_id"]:
        return result("DENY", "credential-activation-session-mismatch")
    if activation["tpm_identity_digest"] != manifest["tpm_identity_digest"]:
        return result("DENY", "credential-activation-tpm-identity-mismatch")
    if activation["ek_public_sha256"] != ek["public_sha256"]:
        return result("DENY", "credential-activation-ek-mismatch")

    try:
        activation_name = validate_name(
            activation["ak_name_hex"],
            "credential_activation.ak_name_hex",
        )
    except ValueError as exc:
        return result(
            "DENY",
            "credential-activation-ak-name-invalid",
            {"error": str(exc)},
        )
    if activation_name != ak_name:
        return result("DENY", "credential-activation-ak-name-mismatch")

    for field in ("challenge_sha256", "credential_blob_sha256"):
        if not valid_hash(activation[field]):
            return result(
                "DENY",
                "credential-activation-digest-invalid",
                {"field": field},
            )
    if activation["challenge_origin"] != "external-verifier-supplied":
        return result("DENY", "credential-activation-challenge-not-external")

    activation_state = activation["state"]
    if activation_state == "DENY":
        return result("DENY", "credential-activation-denied")
    if activation_state != "PASS":
        return result("INDETERMINATE", "credential-activation-indeterminate")

    scope = activation["scope"]
    if scope == "ReferenceModelOnly":
        for field in ("expected_secret_sha256", "activated_secret_sha256"):
            if not valid_hash(activation.get(field)):
                return result(
                    "DENY",
                    "reference-activation-secret-digest-invalid",
                    {"field": field},
                )
        if activation["expected_secret_sha256"] != activation["activated_secret_sha256"]:
            return result("DENY", "reference-activation-secret-mismatch")
    elif scope == "LiveVerifierSession":
        return result(
            "INDETERMINATE",
            "live-activation-execution-not-integrated-into-static-verifier",
        )
    elif scope == "OfflineBundle":
        return result(
            "INDETERMINATE",
            "offline-activation-receipt-requires-independent-verifier-authentication",
        )
    else:
        return result("DENY", "credential-activation-scope-invalid")

    if manifest["verification_mode"] == "OfflineBundle":
        return result(
            "INDETERMINATE",
            "offline-lineage-cannot-claim-live-credential-activation",
        )

    if manifest["verification_mode"] == "LiveVerifierSession":
        return result(
            "INDETERMINATE",
            "live-lineage-execution-not-integrated-into-static-verifier",
        )

    if manifest["verification_mode"] == "ReferenceModelOnly" and scope != "ReferenceModelOnly":
        return result(
            "INDETERMINATE",
            "reference-model-requires-reference-activation-scope",
        )

    return result(
        "PASS",
        "ak-ek-lineage-verified",
        {
            "ek_name_sha256": hashlib.sha256(ek_name).hexdigest(),
            "ak_name_sha256": hashlib.sha256(ak_name).hexdigest(),
            "ak_qualified_name_hex": ak_qname.hex(),
            "parent_qualified_name_hex": parent_qname.hex(),
        },
    )


def fixture() -> dict[str, Any]:
    ek_name = bytes.fromhex("000b" + "11" * 32)
    ek_qname = bytes.fromhex("000b" + "22" * 32)
    synthetic_ak_public_area = bytes.fromhex("0001000b00000032") + bytes(64)
    ak_name = SHA256_ALG_ID + hashlib.sha256(synthetic_ak_public_area).digest()
    ak_qname = expected_qname(ek_qname, ak_name)
    tpm_id = "44" * 32
    ak_public = "55" * 32
    ek_public = "66" * 32
    secret = "77" * 32

    return {
        "profile_id": "mycelix.security.tpm.ak-ek-lineage",
        "profile_version": "0.1.0",
        "verification_mode": "ReferenceModelOnly",
        "claim_ceiling": "ReferenceModelOnly",
        "session_id": "ak-ek-lineage-self-test",
        "tpm_identity_digest": tpm_id,
        "ek": {
            "public_sha256": ek_public,
            "name_hex": ek_name.hex(),
            "qualified_name_hex": ek_qname.hex(),
        },
        "ak": {
            "public_sha256": ak_public,
            "public_area_sha256": hashlib.sha256(synthetic_ak_public_area).hexdigest(),
            "name_hex": ak_name.hex(),
            "qualified_name_hex": ak_qname.hex(),
            "name_alg": "sha256",
        },
        "parentage": {
            "parent_type": "EK",
            "parent_qualified_name_hex": ek_qname.hex(),
        },
        "public_name_binding": {
            "state": "PASS",
            "method": "same-tpm-readpublic-context",
            "public_sha256": ak_public,
            "name_sha256": hashlib.sha256(ak_name).hexdigest(),
            "public_area_sha256": hashlib.sha256(
                bytes.fromhex("0001000b00000032") + bytes(64)
            ).hexdigest(),
            "verifier_id": PUBLIC_NAME_VERIFIER_ID,
            "source_sha256": sha256_file(PUBLIC_NAME_VERIFIER_SCRIPT),
            "input_sha256": "",
            "output_sha256": "",
            "verifier_input": {
                "profile_id": "mycelix.security.tpm.public-name-coherence",
                "profile_version": "0.1.0",
                "verification_mode": "ReferenceModelOnly",
                "claim_ceiling": "ReferenceModelOnly",
                "object_role": "AK",
                "public_format": "TPMT_PUBLIC",
                "public_wire_hex": (bytes.fromhex("0001000b00000032") + bytes(64)).hex(),
                "public_wire_sha256": hashlib.sha256(
                    bytes.fromhex("0001000b00000032") + bytes(64)
                ).hexdigest(),
                "name_hex": ak_name.hex(),
                "readpublic_state": "PASS",
                "readpublic_source_sha256": "aa" * 32,
            },
        },
        "public_attributes_binding": {
            "state": "PASS",
            "verifier_id": ATTRIBUTES_VERIFIER_ID,
            "public_area_sha256": hashlib.sha256(
                bytes.fromhex("0001000b00000032") + bytes(64)
            ).hexdigest(),
            "derived_fixedTPM": True,
            "derived_fixedParent": True,
            "source_sha256": sha256_file(ATTRIBUTES_VERIFIER_SCRIPT),
            "verifier_input": {
                "profile_id": "mycelix.security.tpm.ak-public-attributes",
                "profile_version": "0.1.0",
                "verification_mode": "ReferenceModelOnly",
                "claim_ceiling": "ReferenceModelOnly",
                "object_role": "AK",
                "public_format": "TPMT_PUBLIC",
                "public_wire_hex": (bytes.fromhex("0001000b00000032") + bytes(64)).hex(),
                "public_wire_sha256": hashlib.sha256(
                    bytes.fromhex("0001000b00000032") + bytes(64)
                ).hexdigest(),
                "name_hex": (
                    SHA256_ALG_ID
                    + hashlib.sha256(
                        bytes.fromhex("0001000b00000032") + bytes(64)
                    ).digest()
                ).hex(),
                "readpublic_state": "PASS",
                "readpublic_source_sha256": "aa" * 32,
            },
        },
        "ek_credential": {
            "state": "PASS",
            "mode": "provider",
            "material_sha256": "88" * 32,
            "bound_ek_public_sha256": ek_public,
        },
        "credential_activation": {
            "protocol": "TPM2_MakeCredential+TPM2_ActivateCredential",
            "state": "PASS",
            "scope": "ReferenceModelOnly",
            "session_id": "ak-ek-lineage-self-test",
            "tpm_identity_digest": tpm_id,
            "challenge_origin": "external-verifier-supplied",
            "challenge_sha256": "99" * 32,
            "credential_blob_sha256": "aa" * 32,
            "ek_public_sha256": ek_public,
            "ak_name_hex": ak_name.hex(),
            "expected_secret_sha256": secret,
            "activated_secret_sha256": secret,
        },
    }


def refresh_public_name_binding(value: dict[str, Any]) -> None:
    binding = value["public_name_binding"]
    with tempfile.TemporaryDirectory(prefix="mycelix-public-name-refresh-") as td:
        work = Path(td)
        input_path = work / "input.json"
        output_path = work / "output.json"
        input_path.write_text(
            json.dumps(binding["verifier_input"], indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        proc = subprocess.run(
            [
                sys.executable,
                str(PUBLIC_NAME_VERIFIER_SCRIPT),
                "--verify",
                str(input_path),
                "--output",
                str(output_path),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"public-name verifier fixture failed: {proc.stderr}")
        binding["input_sha256"] = sha256_file(input_path)
        binding["output_sha256"] = sha256_file(output_path)


def mutate_public_name_input(value: dict[str, Any]) -> None:
    verifier_input = value["public_name_binding"]["verifier_input"]
    body = bytearray(hex_bytes(verifier_input["public_wire_hex"], "public_wire_hex"))
    body[-1] ^= 0xFF
    verifier_input["public_wire_hex"] = bytes(body).hex()
    verifier_input["public_wire_sha256"] = hashlib.sha256(body).hexdigest()
    verifier_input["name_hex"] = (
        SHA256_ALG_ID + hashlib.sha256(body).digest()
    ).hex()


def mutate_attribute_input(value: dict[str, Any], attrs: int) -> None:
    body = bytearray(
        hex_bytes(
            value["public_attributes_binding"]["verifier_input"]["public_wire_hex"],
            "public_wire_hex",
        )
    )
    body[4:8] = attrs.to_bytes(4, "big")
    value["public_attributes_binding"]["verifier_input"]["public_wire_hex"] = bytes(body).hex()
    value["public_attributes_binding"]["verifier_input"]["public_wire_sha256"] = hashlib.sha256(body).hexdigest()
    value["public_attributes_binding"]["verifier_input"]["name_hex"] = (
        SHA256_ALG_ID + hashlib.sha256(body).digest()
    ).hex()


def mutate_attribute_wire_tail(value: dict[str, Any]) -> None:
    body = bytearray(
        hex_bytes(
            value["public_attributes_binding"]["verifier_input"]["public_wire_hex"],
            "public_wire_hex",
        )
    )
    body[-1] ^= 0xFF
    value["public_attributes_binding"]["verifier_input"]["public_wire_hex"] = bytes(body).hex()
    value["public_attributes_binding"]["verifier_input"]["public_wire_sha256"] = hashlib.sha256(body).hexdigest()
    value["public_attributes_binding"]["verifier_input"]["name_hex"] = (
        SHA256_ALG_ID + hashlib.sha256(body).digest()
    ).hex()


def self_test() -> int:
    base = fixture()
    refresh_public_name_binding(base)
    cases: list[tuple[str, str, Any]] = [
        ("canonical-valid", "PASS", lambda x: x),
        ("ak-qualified-name-substitution", "DENY", lambda x: x["ak"].update({"qualified_name_hex": "000b" + "ff" * 32})),
        ("parent-qname-substitution", "DENY", lambda x: x["parentage"].update({"parent_qualified_name_hex": "000b" + "ee" * 32})),
        ("parent-type-substitution", "DENY", lambda x: x["parentage"].update({"parent_type": "owner"})),
        ("fixedTPM-cleared", "DENY", lambda x: mutate_attribute_input(x, 0x30)),
        ("fixedParent-cleared", "DENY", lambda x: mutate_attribute_input(x, 0x22)),
        ("name-algorithm-substitution", "DENY", lambda x: x["ak"].update({"name_alg": "sha1"})),
        ("public-name-public-substitution", "DENY", lambda x: x["ak"].update({"public_sha256": "bb" * 32})),
        ("public-name-name-substitution", "DENY", lambda x: x["public_name_binding"].update({"name_sha256": "cc" * 32})),
        ("public-name-binding-indeterminate", "INDETERMINATE", lambda x: x["public_name_binding"].update({"state": "INDETERMINATE"})),
        ("public-name-verifier-substitution", "DENY", lambda x: x["public_name_binding"].update({"verifier_id": "other-verifier"})),
        ("public-name-source-substitution", "DENY", lambda x: x["public_name_binding"].update({"source_sha256": "12" * 32})),
        ("public-name-input-substitution", "DENY", lambda x: mutate_public_name_input(x)),
        ("public-name-area-substitution", "DENY", lambda x: x["public_name_binding"].update({"public_area_sha256": "13" * 32})),
        ("ek-credential-binding-substitution", "DENY", lambda x: x["ek_credential"].update({"bound_ek_public_sha256": "dd" * 32})),
        ("ek-credential-unavailable", "INDETERMINATE", lambda x: x["ek_credential"].update({"state": "INDETERMINATE"})),
        ("activation-denied", "DENY", lambda x: x["credential_activation"].update({"state": "DENY"})),
        ("activation-indeterminate", "INDETERMINATE", lambda x: x["credential_activation"].update({"state": "INDETERMINATE"})),
        ("activation-ek-substitution", "DENY", lambda x: x["credential_activation"].update({"ek_public_sha256": "de" * 32})),
        ("activation-protocol-substitution", "DENY", lambda x: x["credential_activation"].update({"protocol": "local-secret-import"})),
        ("activation-ak-substitution", "DENY", lambda x: x["credential_activation"].update({"ak_name_hex": "000b" + "ab" * 32})),
        ("activation-tpm-substitution", "DENY", lambda x: x["credential_activation"].update({"tpm_identity_digest": "ef" * 32})),
        ("offline-activation", "INDETERMINATE", lambda x: (x.update({"verification_mode": "OfflineBundle"}), x["credential_activation"].update({"scope": "OfflineBundle"}))),
        ("live-without-observation", "INDETERMINATE", lambda x: (x.update({"verification_mode": "LiveVerifierSession"}), x["credential_activation"].update({"scope": "LiveVerifierSession"}))),
        ("attributes-verifier-substitution", "DENY", lambda x: x["public_attributes_binding"].update({"verifier_id": "other-verifier"})),
        ("attributes-derived-fixedTPM-substitution", "DENY", lambda x: x["public_attributes_binding"].update({"derived_fixedTPM": False})),
        ("attributes-source-substitution", "DENY", lambda x: x["public_attributes_binding"].update({"source_sha256": "12" * 32})),
        ("public-area-cross-object-splice", "DENY", mutate_attribute_wire_tail),
        ("activation-secret-substitution", "DENY", lambda x: x["credential_activation"].update({"activated_secret_sha256": "01" * 32})),
    ]

    for name, expected_state, mutate in cases:
        candidate = copy.deepcopy(base)
        mutate(candidate)
        observed = verify(candidate)
        if observed["state"] != expected_state:
            print(f"{name}: FAIL (expected {expected_state}, got {observed['state']})")
            return 1

    permuted = json.loads(json.dumps(base, sort_keys=True))
    observed = verify(permuted)
    if observed["state"] != "PASS":
        print("key-order-permutation: FAIL")
        return 1

    print("AK/EK lineage semantic corpus: PASS")
    print("28 adversarial mutations plus canonical and key-order controls: PASS")
    print("Live/offline activation remains explicitly bounded")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--verify", metavar="MANIFEST")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    path = Path(args.verify).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise SystemExit("manifest must be a JSON object")

    verified = verify(manifest)
    output = {
        "profile_id": "mycelix.security.tpm.ak-ek-lineage",
        "profile_version": "0.1.0",
        "verifier_id": VERIFIER_ID,
        "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        **verified,
    }
    output["content_sha256"] = canonical_hash(
        {key: value for key, value in output.items() if key != "content_sha256"}
    )
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS": 0, "DENY": 1, "INDETERMINATE": 2}[verified["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
