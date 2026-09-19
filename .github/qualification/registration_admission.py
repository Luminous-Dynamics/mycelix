#!/usr/bin/env python3
"""Non-authoritative preflight -> qualification registration admission checker."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any

INTENT_SCHEMA = "mycelix-qualification-registration-intent-v1"
CHILD_SCHEMA = "mycelix-qualification-preflight-v1"
ENV_SCHEMA = "mycelix-preflight-execution-environment-v1"
INTENT_DOMAIN = b"MYCELIX_QUALIFICATION_REGISTRATION_INTENT_V1\0"
RECEIPT_DOMAIN = b"MYCELIX_PREFLIGHT_RECEIPT_V1\0"
ENV_DOMAIN = b"MYCELIX_PREFLIGHT_EXECUTION_ENVIRONMENT_V1\0"
ADMISSION_DOMAIN = b"MYCELIX_QUALIFICATION_REGISTRATION_ADMISSION_V1\0"
IMPL_DOMAIN = b"MYCELIX_QUALIFICATION_REGISTRATION_ADMISSION_IMPLEMENTATION_V1\0"
MAX_JSON = 8 * 1024 * 1024
OID = re.compile(r"^[0-9a-f]{40}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")
ID = re.compile(r"^[a-z0-9][a-z0-9._-]{0,127}$")
REPOSITORY = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")

INTENT_FIELDS = {
    "schema",
    "intent_id",
    "repository",
    "subject_sha",
    "predecessor_sha",
    "preflight_profile_id",
    "preflight_profile_commitment",
    "preflight_implementation_commitment",
    "environment_adapter_implementation_commitment",
    "preflight_environment_commitment",
    "qualification_workflow_path",
    "qualification_workflow_commit_sha",
    "qualification_workflow_blob_sha1",
    "registration_mode",
}

# Exact successful receipt surface emitted by EVIDENCE-CI-004 + 004A v1.
# If that producer surface changes, this checker should get a new reviewed
# revision rather than silently accepting an expanded authority/data surface.
ENVIRONMENT_FIELDS = {
    "schema",
    "policy_revision",
    "adapter_implementation_commitment",
    "platform_system",
    "platform_machine",
    "python_implementation",
    "python_version",
    "python",
    "git",
    "rustup",
    "rustup_home",
    "toolchain",
    "cargo",
    "rustfmt",
    "cargo_fmt",
    "path_policy",
    "bound_input_policy",
    "inherited_environment",
    "locale",
    "timezone",
    "python_hash_seed",
    "python_isolated_mode",
}

ELIGIBLE_RECEIPT_FIELDS = {
    "schema",
    "preflight_implementation_commitment",
    "profile_id",
    "profile_commitment",
    "repository",
    "origin",
    "subject_sha",
    "required_parent_sha",
    "changed_paths",
    "classification",
    "qualification_result",
    "qualification_authority",
    "python_version",
    "git_executable",
    "rustup_executable",
    "checks",
    "tracked_materialization_commitment",
    "rustup_toolchain_list",
    "cargo_probe",
    "rustfmt_probe",
    "reason",
    "environment_adapter_schema",
    "environment_adapter_implementation_commitment",
    "bound_profile_raw_sha256",
    "bound_profile_commitment",
    "bound_preflight_raw_sha256",
    "bound_preflight_implementation_commitment",
    "preflight_environment",
    "preflight_environment_commitment",
    "child_process",
}

CLASS_EXIT = {"ADMISSIBLE_TO_REQUEST": 0, "REFUSED": 2, "INVALID": 4}

_IMPL_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT = hashlib.sha256(IMPL_DOMAIN + _IMPL_BYTES).hexdigest()
del _IMPL_BYTES


class AdmissionError(RuntimeError):
    pass


def _closed_object(raw: bytes, label: str) -> dict[str, Any]:
    if len(raw) > MAX_JSON:
        raise AdmissionError(f"{label} exceeds size bound")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise AdmissionError(f"{label} is not UTF-8") from exc

    def closed_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise AdmissionError(f"duplicate JSON key in {label}: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(text, object_pairs_hook=closed_pairs)
    except json.JSONDecodeError as exc:
        raise AdmissionError(f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise AdmissionError(f"{label} must be a JSON object")
    return value


def _load(path: Path, label: str) -> dict[str, Any]:
    try:
        return _closed_object(path.read_bytes(), label)
    except OSError as exc:
        raise AdmissionError(f"cannot read {label}") from exc


def _canonical(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def _commit(domain: bytes, value: Any) -> str:
    return hashlib.sha256(domain + _canonical(value)).hexdigest()


def _canonical_path(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value.startswith("/")
        or "\\" in value
        or "\0" in value
        or any(part in ("", ".", "..") for part in value.split("/"))
    ):
        raise AdmissionError(f"non-canonical workflow path: {value!r}")
    return value


def _expect_hash(value: object, pattern: re.Pattern[str], label: str) -> str:
    if not isinstance(value, str) or not pattern.fullmatch(value):
        raise AdmissionError(f"invalid {label}")
    return value


def _validate_intent(intent: dict[str, Any]) -> str:
    if set(intent) != INTENT_FIELDS:
        raise AdmissionError("registration intent schema/fields mismatch")
    if intent["schema"] != INTENT_SCHEMA:
        raise AdmissionError("registration intent schema mismatch")
    if not isinstance(intent["intent_id"], str) or not ID.fullmatch(intent["intent_id"]):
        raise AdmissionError("invalid intent_id")
    if not isinstance(intent["repository"], str) or not REPOSITORY.fullmatch(intent["repository"]):
        raise AdmissionError("invalid repository")
    _expect_hash(intent["subject_sha"], OID, "subject SHA")
    _expect_hash(intent["predecessor_sha"], OID, "predecessor SHA")
    if not isinstance(intent["preflight_profile_id"], str) or not ID.fullmatch(intent["preflight_profile_id"]):
        raise AdmissionError("invalid preflight_profile_id")
    for field in (
        "preflight_profile_commitment",
        "preflight_implementation_commitment",
        "environment_adapter_implementation_commitment",
        "preflight_environment_commitment",
    ):
        _expect_hash(intent[field], SHA256, field)
    _canonical_path(intent["qualification_workflow_path"])
    _expect_hash(
        intent["qualification_workflow_commit_sha"], OID,
        "qualification workflow commit SHA",
    )
    _expect_hash(
        intent["qualification_workflow_blob_sha1"], OID,
        "qualification workflow blob SHA-1",
    )
    if intent["registration_mode"] != "manual-request-v1":
        raise AdmissionError("unsupported registration_mode")
    return _commit(INTENT_DOMAIN, intent)


def _validate_sha256_field(receipt: dict[str, Any], field: str) -> None:
    _expect_hash(receipt.get(field), SHA256, f"receipt {field}")


def _validate_eligible_receipt(receipt: dict[str, Any]) -> str:
    if set(receipt) != ELIGIBLE_RECEIPT_FIELDS:
        raise AdmissionError("eligible receipt schema/fields mismatch")
    if receipt.get("schema") != CHILD_SCHEMA:
        raise AdmissionError("preflight receipt schema mismatch")
    if receipt.get("environment_adapter_schema") != ENV_SCHEMA:
        raise AdmissionError("environment adapter schema mismatch")
    if receipt.get("classification") != "ELIGIBLE":
        raise AdmissionError("preflight receipt is not ELIGIBLE")
    if receipt.get("qualification_result") is not None:
        raise AdmissionError("preflight receipt attempted qualification result")
    if receipt.get("qualification_authority") is not False:
        raise AdmissionError("preflight receipt attempted qualification authority")

    if not isinstance(receipt.get("repository"), str) or not REPOSITORY.fullmatch(receipt["repository"]):
        raise AdmissionError("invalid receipt repository")
    _expect_hash(receipt.get("subject_sha"), OID, "receipt subject SHA")
    _expect_hash(receipt.get("required_parent_sha"), OID, "receipt predecessor SHA")
    if not isinstance(receipt.get("profile_id"), str) or not ID.fullmatch(receipt["profile_id"]):
        raise AdmissionError("invalid receipt profile_id")

    for field in (
        "profile_commitment",
        "preflight_implementation_commitment",
        "environment_adapter_implementation_commitment",
        "preflight_environment_commitment",
        "bound_profile_raw_sha256",
        "bound_profile_commitment",
        "bound_preflight_raw_sha256",
        "bound_preflight_implementation_commitment",
        "tracked_materialization_commitment",
    ):
        _validate_sha256_field(receipt, field)

    if receipt["bound_profile_commitment"] != receipt["profile_commitment"]:
        raise AdmissionError("receipt profile binding is internally inconsistent")
    if (
        receipt["bound_preflight_implementation_commitment"]
        != receipt["preflight_implementation_commitment"]
    ):
        raise AdmissionError("receipt implementation binding is internally inconsistent")

    environment = receipt.get("preflight_environment")
    if not isinstance(environment, dict) or set(environment) != ENVIRONMENT_FIELDS:
        raise AdmissionError("preflight environment schema/fields mismatch")
    if environment.get("schema") != ENV_SCHEMA:
        raise AdmissionError("embedded preflight environment schema mismatch")
    if (
        environment.get("adapter_implementation_commitment")
        != receipt["environment_adapter_implementation_commitment"]
    ):
        raise AdmissionError("environment adapter binding is internally inconsistent")
    if _commit(ENV_DOMAIN, environment) != receipt["preflight_environment_commitment"]:
        raise AdmissionError("preflight environment commitment mismatch")
    if environment.get("python_version") != receipt.get("python_version"):
        raise AdmissionError("Python environment identity is internally inconsistent")

    for tool_name, receipt_field in (("git", "git_executable"), ("rustup", "rustup_executable")):
        observed = environment.get(tool_name)
        child_observed = receipt.get(receipt_field)
        if not isinstance(observed, dict) or not isinstance(child_observed, dict):
            raise AdmissionError(f"missing {tool_name} executable identity")
        _expect_hash(observed.get("sha256"), SHA256, f"environment {tool_name} SHA-256")
        _expect_hash(child_observed.get("sha256"), SHA256, f"child {tool_name} SHA-256")
        if observed["sha256"] != child_observed["sha256"]:
            raise AdmissionError(f"{tool_name} executable identity is internally inconsistent")

    checks = receipt.get("checks")
    if not isinstance(checks, list) or len(checks) != 1 or not isinstance(checks[0], dict):
        raise AdmissionError("eligible receipt must contain exactly one registered check")
    check = checks[0]
    if check.get("id") != "rustfmt" or check.get("status") != "PASS":
        raise AdmissionError("eligible receipt lacks rustfmt PASS")

    child = receipt.get("child_process")
    if not isinstance(child, dict) or set(child) != {"exit_code", "stdout_sha256", "stderr_sha256"}:
        raise AdmissionError("invalid child_process receipt")
    if child["exit_code"] != 0:
        raise AdmissionError("ELIGIBLE child process did not exit 0")
    _expect_hash(child["stdout_sha256"], SHA256, "child stdout SHA-256")
    _expect_hash(child["stderr_sha256"], SHA256, "child stderr SHA-256")

    if receipt.get("reason") != "registered rustfmt preflight gate passed":
        raise AdmissionError("unexpected ELIGIBLE reason")

    return _commit(RECEIPT_DOMAIN, receipt)


def evaluate(receipt: dict[str, Any], intent: dict[str, Any]) -> dict[str, Any]:
    intent_commitment = _validate_intent(intent)

    # A syntactically valid preflight result that is explicitly not eligible is
    # a registration refusal, not malformed registration input. Keep this path
    # non-authoritative without requiring the successful-receipt field surface.
    classification = receipt.get("classification")
    if classification in {"NOT_ELIGIBLE", "UNAVAILABLE", "INVALID"}:
        if receipt.get("qualification_result") is not None:
            raise AdmissionError("preflight receipt attempted qualification result")
        if receipt.get("qualification_authority") is not False:
            raise AdmissionError("preflight receipt attempted qualification authority")
        return {
            "schema": "mycelix-qualification-registration-admission-v1",
            "registration_admission_implementation_commitment": IMPLEMENTATION_COMMITMENT,
            "classification": "REFUSED",
            "reason": f"preflight classification is {classification}",
            "mismatches": ["classification"],
            "intent_commitment": intent_commitment,
            "preflight_receipt_commitment": _commit(RECEIPT_DOMAIN, receipt),
            "registration_authority": False,
            "workflow_dispatched": False,
            "workflow_identity_verified": False,
            "qualification_result": None,
            "qualification_authority": False,
        }
    if classification != "ELIGIBLE":
        raise AdmissionError("preflight receipt has unknown classification")

    receipt_commitment = _validate_eligible_receipt(receipt)

    comparisons = {
        "repository": (receipt["repository"], intent["repository"]),
        "subject_sha": (receipt["subject_sha"], intent["subject_sha"]),
        "predecessor_sha": (
            receipt["required_parent_sha"], intent["predecessor_sha"]
        ),
        "preflight_profile_id": (
            receipt["profile_id"], intent["preflight_profile_id"]
        ),
        "preflight_profile_commitment": (
            receipt["profile_commitment"],
            intent["preflight_profile_commitment"],
        ),
        "preflight_implementation_commitment": (
            receipt["preflight_implementation_commitment"],
            intent["preflight_implementation_commitment"],
        ),
        "environment_adapter_implementation_commitment": (
            receipt["environment_adapter_implementation_commitment"],
            intent["environment_adapter_implementation_commitment"],
        ),
        "preflight_environment_commitment": (
            receipt["preflight_environment_commitment"],
            intent["preflight_environment_commitment"],
        ),
    }
    mismatches = [name for name, (actual, expected) in comparisons.items() if actual != expected]
    if mismatches:
        return {
            "schema": "mycelix-qualification-registration-admission-v1",
            "registration_admission_implementation_commitment": IMPLEMENTATION_COMMITMENT,
            "classification": "REFUSED",
            "reason": "preflight receipt does not match registration intent",
            "mismatches": mismatches,
            "intent_commitment": intent_commitment,
            "preflight_receipt_commitment": receipt_commitment,
            "registration_authority": False,
            "workflow_dispatched": False,
            "workflow_identity_verified": False,
            "qualification_result": None,
            "qualification_authority": False,
        }

    core = {
        "schema": "mycelix-qualification-registration-admission-v1",
        "registration_admission_implementation_commitment": IMPLEMENTATION_COMMITMENT,
        "classification": "ADMISSIBLE_TO_REQUEST",
        "intent_commitment": intent_commitment,
        "preflight_receipt_commitment": receipt_commitment,
        "repository": intent["repository"],
        "subject_sha": intent["subject_sha"],
        "predecessor_sha": intent["predecessor_sha"],
        "qualification_workflow_path": intent["qualification_workflow_path"],
        "qualification_workflow_commit_sha": intent["qualification_workflow_commit_sha"],
        "qualification_workflow_blob_sha1": intent["qualification_workflow_blob_sha1"],
        "registration_mode": intent["registration_mode"],
        "registration_authority": False,
        "workflow_dispatched": False,
        "workflow_identity_verified": False,
        "qualification_result": None,
        "qualification_authority": False,
    }
    core["admission_commitment"] = _commit(ADMISSION_DOMAIN, core)
    return core


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--intent", required=True)
    args = parser.parse_args()
    try:
        receipt = _load(Path(args.receipt), "preflight receipt")
        intent = _load(Path(args.intent), "registration intent")
        result = evaluate(receipt, intent)
    except (OSError, AdmissionError) as exc:
        result = {
            "schema": "mycelix-qualification-registration-admission-v1",
            "registration_admission_implementation_commitment": IMPLEMENTATION_COMMITMENT,
            "classification": "INVALID",
            "reason": str(exc),
            "registration_authority": False,
            "workflow_dispatched": False,
            "workflow_identity_verified": False,
            "qualification_result": None,
            "qualification_authority": False,
        }
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), ensure_ascii=False))
    return CLASS_EXIT.get(result["classification"], 4)


if __name__ == "__main__":
    raise SystemExit(main())
