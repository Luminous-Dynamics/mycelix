from __future__ import annotations

import hashlib

from qcap_canon import (
    CapsuleError,
    EMPTY_SHA256,
    H64,
    IDENT,
    capsule_commitment,
    commitment,
    exact_keys,
    hex_value,
)
from qcap4_context import validate_execution_context_v4

RECEIPT4_DOMAIN = b"MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V4\0"
RUNNER_FAILURE_REASONS = {
    "OutputLimitExceeded",
    "Timeout",
    "ProcessStartFailure",
    "OutputDrainTimeout",
    "WorktreeMaterializationFailure",
    "WorktreeCleanupFailure",
    "ArtifactIntegrityFailure",
    "GateReportedRunnerFailure",
    "UnexpectedExitCode",
    "RunnerInternalFailure",
    "ContainmentFailure",
}
RESULT_KEYS = {
    "id",
    "status",
    "effective_exit_code",
    "captured_output_sha256",
    "captured_output_bytes",
    "output_truncated",
    "runner_failure_reason",
}


def receipt_commitment_v4(receipt_body):
    return commitment(RECEIPT4_DOMAIN, receipt_body)


def not_run_result(gate_id):
    return {
        "id": gate_id,
        "status": "GateNotRun",
        "effective_exit_code": None,
        "captured_output_sha256": EMPTY_SHA256,
        "captured_output_bytes": 0,
        "output_truncated": False,
        "runner_failure_reason": None,
    }


def gate_result(gate_id, status, code, captured, output_truncated=False, reason=None):
    return {
        "id": gate_id,
        "status": status,
        "effective_exit_code": code,
        "captured_output_sha256": hashlib.sha256(captured).hexdigest(),
        "captured_output_bytes": len(captured),
        "output_truncated": bool(output_truncated),
        "runner_failure_reason": reason,
    }


def validate_gate_result(result):
    exact_keys(result, RESULT_KEYS, "gate result")
    if not isinstance(result["id"], str) or not IDENT.fullmatch(result["id"]):
        raise CapsuleError("gate result id invalid")
    hex_value(result["captured_output_sha256"], H64, "captured output digest")
    count = result["captured_output_bytes"]
    if not isinstance(count, int) or isinstance(count, bool) or count < 0:
        raise CapsuleError("captured output bytes invalid")
    if not isinstance(result["output_truncated"], bool):
        raise CapsuleError("output_truncated invalid")

    status = result["status"]
    code = result["effective_exit_code"]
    reason = result["runner_failure_reason"]
    truncated = result["output_truncated"]

    if status == "GateNotRun":
        if (
            code is not None
            or count != 0
            or result["captured_output_sha256"] != EMPTY_SHA256
            or truncated
            or reason is not None
        ):
            raise CapsuleError("GateNotRun fields invalid")
        return
    if status == "GatePass":
        if code != 0 or reason is not None or truncated:
            raise CapsuleError("GatePass fields invalid")
        return
    if status == "GateFail":
        if code != 10 or reason is not None or truncated:
            raise CapsuleError("GateFail fields invalid")
        return
    if status == "RunnerInfrastructureFailure":
        if code != 20 or reason not in RUNNER_FAILURE_REASONS:
            raise CapsuleError("runner failure fields invalid")
        if (reason == "OutputLimitExceeded") != truncated:
            raise CapsuleError("output truncation/reason mismatch")
        return
    raise CapsuleError("gate status invalid")


def attempt_verdict(results):
    infrastructure_failure = False
    theorem_failure = False
    for result in results:
        validate_gate_result(result)
        status = result["status"]
        if infrastructure_failure:
            if status != "GateNotRun":
                raise CapsuleError("only GateNotRun may follow runner failure")
            continue
        if status == "GateNotRun":
            raise CapsuleError("GateNotRun before runner failure")
        if status == "RunnerInfrastructureFailure":
            infrastructure_failure = True
        elif status == "GateFail":
            theorem_failure = True
    if infrastructure_failure:
        return "RunnerInfrastructureFailure"
    if theorem_failure:
        return "CompletedConjunctiveFail"
    return "CompletedConjunctivePass"


def compose_receipt_v4(manifest, attempt_id, context, limits, containment_profile, results):
    if not isinstance(attempt_id, str) or not IDENT.fullmatch(attempt_id):
        raise CapsuleError("attempt id invalid")
    validate_execution_context_v4(context, manifest, limits, containment_profile)
    gate_ids = [gate["id"] for gate in manifest["gates"]]
    if [result.get("id") for result in results] != gate_ids:
        raise CapsuleError("gate order mismatch")

    body = {
        "receipt_format_revision": 4,
        "capsule_commitment": capsule_commitment(manifest),
        "theorem_id": manifest["theorem_id"],
        "theorem_revision": manifest["theorem_revision"],
        "repository_identity": manifest["repository_identity"],
        "product_subject_sha": manifest["product_subject_sha"],
        "attempt_id": attempt_id,
        "execution_context": context,
        "gate_results": results,
        "verdict": attempt_verdict(results),
        "claim": manifest["claim"],
        "nonclaims": manifest["nonclaims"],
    }
    body["receipt_commitment"] = receipt_commitment_v4(body)
    return body


def verify_receipt_v4(receipt, manifest, limits, containment_profile):
    keys = {
        "receipt_format_revision",
        "capsule_commitment",
        "theorem_id",
        "theorem_revision",
        "repository_identity",
        "product_subject_sha",
        "attempt_id",
        "execution_context",
        "gate_results",
        "verdict",
        "claim",
        "nonclaims",
        "receipt_commitment",
    }
    exact_keys(receipt, keys, "receipt")
    if receipt["receipt_format_revision"] != 4:
        raise CapsuleError("unsupported receipt")

    supplied = hex_value(receipt["receipt_commitment"], H64, "receipt commitment")
    body = dict(receipt)
    body.pop("receipt_commitment")
    if receipt_commitment_v4(body) != supplied:
        raise CapsuleError("receipt commitment mismatch")

    for key, expected in (
        ("capsule_commitment", capsule_commitment(manifest)),
        ("theorem_id", manifest["theorem_id"]),
        ("theorem_revision", manifest["theorem_revision"]),
        ("repository_identity", manifest["repository_identity"]),
        ("product_subject_sha", manifest["product_subject_sha"]),
        ("claim", manifest["claim"]),
        ("nonclaims", manifest["nonclaims"]),
    ):
        if receipt[key] != expected:
            raise CapsuleError(f"receipt {key} mismatch")

    validate_execution_context_v4(
        receipt["execution_context"], manifest, limits, containment_profile
    )
    gate_ids = [gate["id"] for gate in manifest["gates"]]
    if (
        not isinstance(receipt["gate_results"], list)
        or [result.get("id") for result in receipt["gate_results"]] != gate_ids
    ):
        raise CapsuleError("receipt gate set mismatch")
    if receipt["verdict"] != attempt_verdict(receipt["gate_results"]):
        raise CapsuleError("receipt verdict mismatch")
    return True
