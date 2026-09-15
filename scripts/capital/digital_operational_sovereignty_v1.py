#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-digital-operational-sovereignty-v1"
RECEIPT_VERSION = "mycelix-digital-operational-sovereignty-receipt-v1"
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX40 = re.compile(r"^[0-9a-f]{40}$")
_HEX64 = re.compile(r"^[0-9a-f]{64}$")
DIMENSIONS = (
    "reproducible_deployment",
    "administrator_recovery",
    "trust_root_rotation",
    "secrets_migration",
    "backup_restore",
    "data_schema_export",
    "sbom_inventory",
    "observability_runbooks",
    "disaster_recovery_exercise",
    "operator_replacement_exercise",
    "operational_documentation",
    "continuity_plan",
)
STATES = {"PASS", "FAIL", "NOT_ASSESSED", "NOT_APPLICABLE"}
PROFILE_KEYS = {
    "profile_version", "project_id", "parent_subject_sha",
    "parent_transition_receipt_sha256", "required_dimensions",
}
ASSESSMENT_KEYS = {
    "assessment_id", "project_id", "profile_sha256",
    "parent_transition_receipt_sha256", "outgoing_operator_ref",
    "assessor_ref", "scope_ref", "controls", "evidence_ref",
}
NONCLAIMS = (
    "operational readiness is not handover acceptance",
    "operational readiness is not legal title transfer",
    "operational readiness is not democratic legitimacy",
    "operational readiness is not physical asset-condition assurance",
    "assessment reference separation is not proof of external identity authenticity",
    "operational readiness does not imply future service performance or cybersecurity certification",
)

class SovereigntyError(ValueError):
    pass

@dataclass(frozen=True)
class QualifiedSovereignty:
    _receipt: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v: Any) -> bytes:
    return json.dumps(v, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")

def sha256_hex(v: Any) -> str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()

def _exact(obj: dict[str, Any], keys: set[str], ctx: str) -> None:
    if set(obj) != keys:
        raise SovereigntyError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")

def _id(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not _ID.fullmatch(v):
        raise SovereigntyError(f"{ctx}: invalid bounded identifier")
    return v

def _ref(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not v or len(v.encode("utf-8")) > 512 or v != v.strip():
        raise SovereigntyError(f"{ctx}: invalid reference")
    return v

def validate_profile(p: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(p, dict):
        raise SovereigntyError("profile: expected object")
    _exact(p, PROFILE_KEYS, "profile")
    if p["profile_version"] != PROFILE_VERSION:
        raise SovereigntyError("profile.profile_version: unsupported")
    _id(p["project_id"], "profile.project_id")
    if not isinstance(p["parent_subject_sha"], str) or not _HEX40.fullmatch(p["parent_subject_sha"]):
        raise SovereigntyError("profile.parent_subject_sha: invalid")
    if not isinstance(p["parent_transition_receipt_sha256"], str) or not _HEX64.fullmatch(p["parent_transition_receipt_sha256"]):
        raise SovereigntyError("profile.parent_transition_receipt_sha256: invalid")
    required = p["required_dimensions"]
    if not isinstance(required, list) or not required:
        raise SovereigntyError("profile.required_dimensions: expected non-empty array")
    if len(set(required)) != len(required):
        raise SovereigntyError("profile.required_dimensions: duplicate")
    unknown = set(required) - set(DIMENSIONS)
    if unknown:
        raise SovereigntyError(f"profile.required_dimensions: unknown={sorted(unknown)}")
    return p

def validate_parent(parent: dict[str, Any], p: dict[str, Any]) -> None:
    if not isinstance(parent, dict):
        raise SovereigntyError("parent_receipt: expected object")
    if sha256_hex(parent) != p["parent_transition_receipt_sha256"]:
        raise SovereigntyError("parent_receipt: semantic digest mismatch")
    for k in ("project_id", "financial_state", "remaining_claim_units"):
        if k not in parent:
            raise SovereigntyError(f"parent_receipt: missing {k}")
    if parent["project_id"] != p["project_id"]:
        raise SovereigntyError("parent_receipt: project mismatch")
    amount = parent["remaining_claim_units"]
    if isinstance(amount, bool) or not isinstance(amount, int) or amount < 0:
        raise SovereigntyError("parent_receipt.remaining_claim_units: invalid")

def validate_assessment(a: dict[str, Any], p: dict[str, Any]) -> None:
    if not isinstance(a, dict):
        raise SovereigntyError("assessment: expected object")
    _exact(a, ASSESSMENT_KEYS, "assessment")
    _id(a["assessment_id"], "assessment.assessment_id")
    if a["project_id"] != p["project_id"]:
        raise SovereigntyError("assessment.project_id: project substitution")
    if a["profile_sha256"] != sha256_hex(p):
        raise SovereigntyError("assessment.profile_sha256: profile substitution")
    if a["parent_transition_receipt_sha256"] != p["parent_transition_receipt_sha256"]:
        raise SovereigntyError("assessment.parent_transition_receipt_sha256: parent substitution")
    operator = _ref(a["outgoing_operator_ref"], "assessment.outgoing_operator_ref")
    assessor = _ref(a["assessor_ref"], "assessment.assessor_ref")
    if operator == assessor:
        raise SovereigntyError("assessment: outgoing operator cannot be sole assessor in v1")
    _ref(a["scope_ref"], "assessment.scope_ref")
    _ref(a["evidence_ref"], "assessment.evidence_ref")
    controls = a["controls"]
    if not isinstance(controls, dict):
        raise SovereigntyError("assessment.controls: expected object")
    _exact(controls, set(DIMENSIONS), "assessment.controls")
    required = set(p["required_dimensions"])
    for dim in DIMENSIONS:
        state = controls[dim]
        if state not in STATES:
            raise SovereigntyError(f"assessment.controls.{dim}: unsupported state")
        if dim in required and state == "NOT_APPLICABLE":
            raise SovereigntyError(f"assessment.controls.{dim}: required dimension cannot be NOT_APPLICABLE")
        if dim not in required and state != "NOT_APPLICABLE":
            raise SovereigntyError(f"assessment.controls.{dim}: non-required dimension must be NOT_APPLICABLE")

def qualify(p: dict[str, Any], parent: dict[str, Any], assessment: dict[str, Any]) -> QualifiedSovereignty:
    validate_profile(p)
    validate_parent(parent, p)
    validate_assessment(assessment, p)
    required = set(p["required_dimensions"])
    failed = sorted(dim for dim in required if assessment["controls"][dim] == "FAIL")
    unassessed = sorted(dim for dim in required if assessment["controls"][dim] == "NOT_ASSESSED")
    if failed:
        state = "REMEDIATION_REQUIRED"
    elif unassessed:
        state = "ASSESSMENT_INCOMPLETE"
    else:
        state = "OPERATIONAL_TRANSFER_READY"
    blockers = [f"FAIL:{x}" for x in failed] + [f"NOT_ASSESSED:{x}" for x in unassessed]
    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": p["project_id"],
        "profile_sha256": sha256_hex(p),
        "parent_subject_sha": p["parent_subject_sha"],
        "parent_transition_receipt_sha256": p["parent_transition_receipt_sha256"],
        "parent_financial_state": parent["financial_state"],
        "parent_remaining_claim_units": parent["remaining_claim_units"],
        "assessment_id": assessment["assessment_id"],
        "assessment_sha256": sha256_hex(assessment),
        "outgoing_operator_ref": assessment["outgoing_operator_ref"],
        "assessor_ref": assessment["assessor_ref"],
        "required_dimensions": list(p["required_dimensions"]),
        "controls": {k: assessment["controls"][k] for k in DIMENSIONS},
        "readiness_state": state,
        "blockers": blockers,
        "handover_accepted": False,
        "legal_transition_complete": False,
        "nonclaims": list(NONCLAIMS),
    }
    return QualifiedSovereignty(receipt)

def load_case(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != {"profile", "parent_receipt", "assessment"}:
        raise SovereigntyError("case: expected exact keys")
    return data["profile"], data["parent_receipt"], data["assessment"]

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    a = ap.parse_args()
    p, parent, assessment = load_case(a.case)
    receipt = qualify(p, parent, assessment).receipt()
    text = json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    if a.receipt_out:
        a.receipt_out.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
