#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-handover-acceptance-v1"
RECEIPT_VERSION = "mycelix-handover-acceptance-receipt-v1"
READINESS_VERSION = "mycelix-digital-operational-sovereignty-receipt-v1"
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX64 = re.compile(r"^[0-9a-f]{64}$")
INVENTORY_ITEMS = (
    "deployment_bundle",
    "administrator_recovery_material",
    "trust_root_transition_material",
    "secrets_migration_package",
    "backup_restore_package",
    "data_schema_export_package",
    "sbom_inventory",
    "observability_runbooks",
    "disaster_recovery_package",
    "operator_replacement_package",
    "operational_documentation",
    "continuity_plan",
)
INVENTORY_STATES = {"PRESENT", "MISSING"}
DESIGNATION_STATES = {"ACTIVE", "PENDING", "REVOKED"}
PROFILE_KEYS = {
    "profile_version", "project_id", "readiness_profile_sha256",
    "readiness_registry_id", "readiness_designation_authority_ref",
    "acceptance_authority_ref", "waiver_authority_ref",
    "outgoing_custodian_ref", "incoming_custodian_ref",
    "required_inventory_items", "waivable_inventory_items",
    "legal_effect_mode",
}
DESIGNATION_KEYS = {
    "designation_id", "registry_id", "registry_epoch", "project_id",
    "profile_sha256", "designated_readiness_receipt_sha256",
    "state", "authority_ref", "evidence_ref",
}
EVENT_KEYS = {
    "event_id", "project_id", "profile_sha256", "readiness_receipt_sha256",
    "acceptance_authority_ref", "outgoing_custodian_ref", "incoming_custodian_ref",
    "inventory", "continuity_state", "waivers", "evidence_ref",
}
WAIVER_KEYS = {
    "waiver_id", "requirement", "authority_ref", "reason_ref",
    "remediation_owner_ref", "remediation_epoch_ref", "retention_ref",
}
NONCLAIMS = (
    "operational custody acceptance is not legal title transfer",
    "operational custody acceptance is not constitutional stewardship transition",
    "handover acceptance is not democratic legitimacy",
    "handover acceptance is not regulatory or property-law compliance",
    "waiver acceptance does not rewrite the historical readiness receipt",
    "receipt validity is not proof of external authority identity or signature authenticity",
    "handover acceptance does not establish current post-acceptance condition after later material events",
)

class AcceptanceError(ValueError):
    pass

@dataclass(frozen=True)
class QualifiedAcceptance:
    _receipt: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v: Any) -> bytes:
    return json.dumps(v, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")

def sha256_hex(v: Any) -> str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()

def _exact(obj: dict[str, Any], keys: set[str], ctx: str) -> None:
    if set(obj) != keys:
        raise AcceptanceError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")

def _id(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not _ID.fullmatch(v):
        raise AcceptanceError(f"{ctx}: invalid bounded identifier")
    return v

def _ref(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not v or len(v.encode("utf-8")) > 512 or v != v.strip():
        raise AcceptanceError(f"{ctx}: invalid reference")
    return v

def _hex64(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not _HEX64.fullmatch(v):
        raise AcceptanceError(f"{ctx}: invalid sha256")
    return v

def validate_profile(p: dict[str, Any]) -> None:
    if not isinstance(p, dict):
        raise AcceptanceError("profile: expected object")
    _exact(p, PROFILE_KEYS, "profile")
    if p["profile_version"] != PROFILE_VERSION:
        raise AcceptanceError("profile.profile_version: unsupported")
    _id(p["project_id"], "profile.project_id")
    _hex64(p["readiness_profile_sha256"], "profile.readiness_profile_sha256")
    _id(p["readiness_registry_id"], "profile.readiness_registry_id")
    for k in ("readiness_designation_authority_ref", "acceptance_authority_ref", "waiver_authority_ref", "outgoing_custodian_ref", "incoming_custodian_ref"):
        _ref(p[k], f"profile.{k}")
    if p["outgoing_custodian_ref"] == p["incoming_custodian_ref"]:
        raise AcceptanceError("profile: outgoing and incoming custodians must differ")
    required = p["required_inventory_items"]
    waivable = p["waivable_inventory_items"]
    if not isinstance(required, list) or not required or len(set(required)) != len(required):
        raise AcceptanceError("profile.required_inventory_items: invalid")
    if not isinstance(waivable, list) or len(set(waivable)) != len(waivable):
        raise AcceptanceError("profile.waivable_inventory_items: invalid")
    if set(required) - set(INVENTORY_ITEMS):
        raise AcceptanceError("profile.required_inventory_items: unknown item")
    if set(waivable) - set(required):
        raise AcceptanceError("profile.waivable_inventory_items: must be subset of required")
    if p["legal_effect_mode"] != "CUSTODY_ONLY":
        raise AcceptanceError("profile.legal_effect_mode: v1 supports only CUSTODY_ONLY")

def validate_readiness(r: dict[str, Any], p: dict[str, Any]) -> None:
    if not isinstance(r, dict):
        raise AcceptanceError("readiness_receipt: expected object")
    for k in ("receipt_version", "project_id", "profile_sha256", "readiness_state", "handover_accepted", "legal_transition_complete"):
        if k not in r:
            raise AcceptanceError(f"readiness_receipt: missing {k}")
    if r["receipt_version"] != READINESS_VERSION:
        raise AcceptanceError("readiness_receipt.receipt_version: unsupported")
    if r["project_id"] != p["project_id"]:
        raise AcceptanceError("readiness_receipt: project mismatch")
    if r["profile_sha256"] != p["readiness_profile_sha256"]:
        raise AcceptanceError("readiness_receipt: profile mismatch")
    if r["readiness_state"] != "OPERATIONAL_TRANSFER_READY":
        raise AcceptanceError("readiness_receipt: not operationally ready")
    if r["handover_accepted"] is not False or r["legal_transition_complete"] is not False:
        raise AcceptanceError("readiness_receipt: authority contamination")

def validate_designation(d: dict[str, Any], p: dict[str, Any], readiness_sha: str) -> None:
    if not isinstance(d, dict):
        raise AcceptanceError("readiness_designation: expected object")
    _exact(d, DESIGNATION_KEYS, "readiness_designation")
    _id(d["designation_id"], "readiness_designation.designation_id")
    if d["registry_id"] != p["readiness_registry_id"]:
        raise AcceptanceError("readiness_designation: registry substitution")
    epoch = d["registry_epoch"]
    if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0 or epoch > 10**12:
        raise AcceptanceError("readiness_designation.registry_epoch: invalid")
    if d["project_id"] != p["project_id"]:
        raise AcceptanceError("readiness_designation: project mismatch")
    if d["profile_sha256"] != sha256_hex(p):
        raise AcceptanceError("readiness_designation: profile substitution")
    _hex64(d["designated_readiness_receipt_sha256"], "readiness_designation.designated_readiness_receipt_sha256")
    if d["state"] not in DESIGNATION_STATES:
        raise AcceptanceError("readiness_designation.state: unsupported")
    _ref(d["authority_ref"], "readiness_designation.authority_ref")
    _ref(d["evidence_ref"], "readiness_designation.evidence_ref")
    if d["authority_ref"] != p["readiness_designation_authority_ref"]:
        raise AcceptanceError("readiness_designation: authority mismatch")
    if d["state"] != "ACTIVE":
        raise AcceptanceError(f"readiness_designation: not current ({d['state']})")
    if d["designated_readiness_receipt_sha256"] != readiness_sha:
        raise AcceptanceError("readiness_designation: supplied readiness receipt is stale")

def validate_event(e: dict[str, Any], p: dict[str, Any], readiness_sha: str) -> list[dict[str, Any]]:
    if not isinstance(e, dict):
        raise AcceptanceError("acceptance_event: expected object")
    _exact(e, EVENT_KEYS, "acceptance_event")
    _id(e["event_id"], "acceptance_event.event_id")
    if e["project_id"] != p["project_id"]:
        raise AcceptanceError("acceptance_event: project substitution")
    if e["profile_sha256"] != sha256_hex(p):
        raise AcceptanceError("acceptance_event: profile substitution")
    if e["readiness_receipt_sha256"] != readiness_sha:
        raise AcceptanceError("acceptance_event: readiness substitution")
    if e["acceptance_authority_ref"] != p["acceptance_authority_ref"]:
        raise AcceptanceError("acceptance_event: acceptance authority mismatch")
    if e["outgoing_custodian_ref"] != p["outgoing_custodian_ref"]:
        raise AcceptanceError("acceptance_event: outgoing custodian mismatch")
    if e["incoming_custodian_ref"] != p["incoming_custodian_ref"]:
        raise AcceptanceError("acceptance_event: incoming custodian mismatch")
    _ref(e["evidence_ref"], "acceptance_event.evidence_ref")
    if e["continuity_state"] != "PASS":
        raise AcceptanceError("acceptance_event: continuity must PASS")
    inventory = e["inventory"]
    if not isinstance(inventory, dict):
        raise AcceptanceError("acceptance_event.inventory: expected object")
    _exact(inventory, set(INVENTORY_ITEMS), "acceptance_event.inventory")
    for item, state in inventory.items():
        if state not in INVENTORY_STATES:
            raise AcceptanceError(f"acceptance_event.inventory.{item}: unsupported state")
    waivers = e["waivers"]
    if not isinstance(waivers, list):
        raise AcceptanceError("acceptance_event.waivers: expected array")
    seen_ids, seen_requirements = set(), set()
    normalized = []
    for i, w in enumerate(waivers):
        if not isinstance(w, dict):
            raise AcceptanceError(f"acceptance_event.waivers[{i}]: expected object")
        _exact(w, WAIVER_KEYS, f"acceptance_event.waivers[{i}]")
        wid = _id(w["waiver_id"], f"acceptance_event.waivers[{i}].waiver_id")
        req = w["requirement"]
        if wid in seen_ids:
            raise AcceptanceError("acceptance_event.waivers: duplicate waiver_id")
        if req in seen_requirements:
            raise AcceptanceError("acceptance_event.waivers: duplicate requirement")
        seen_ids.add(wid); seen_requirements.add(req)
        if req not in p["waivable_inventory_items"]:
            raise AcceptanceError(f"acceptance_event.waivers[{i}]: requirement not waivable")
        if inventory[req] != "MISSING":
            raise AcceptanceError(f"acceptance_event.waivers[{i}]: waiver requires missing item")
        if w["authority_ref"] != p["waiver_authority_ref"]:
            raise AcceptanceError(f"acceptance_event.waivers[{i}]: waiver authority mismatch")
        for k in ("reason_ref", "remediation_owner_ref", "remediation_epoch_ref", "retention_ref"):
            _ref(w[k], f"acceptance_event.waivers[{i}].{k}")
        normalized.append(w)
    required = set(p["required_inventory_items"])
    missing = sorted(item for item in required if inventory[item] == "MISSING")
    waived = set(w["requirement"] for w in normalized)
    unwaived = [item for item in missing if item not in waived]
    if unwaived:
        raise AcceptanceError(f"acceptance_event: unwaived missing inventory={unwaived}")
    return normalized

def qualify(p: dict[str, Any], readiness: dict[str, Any], designation: dict[str, Any], event: dict[str, Any]) -> QualifiedAcceptance:
    validate_profile(p)
    validate_readiness(readiness, p)
    readiness_sha = sha256_hex(readiness)
    validate_designation(designation, p, readiness_sha)
    waivers = validate_event(event, p, readiness_sha)
    state = "CUSTODY_ACCEPTED_WITH_WAIVERS" if waivers else "CUSTODY_ACCEPTED"
    missing = sorted(k for k in p["required_inventory_items"] if event["inventory"][k] == "MISSING")
    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": p["project_id"],
        "profile_sha256": sha256_hex(p),
        "readiness_receipt_sha256": readiness_sha,
        "readiness_profile_sha256": p["readiness_profile_sha256"],
        "readiness_state": readiness["readiness_state"],
        "readiness_currentness": "CURRENT",
        "readiness_registry_id": designation["registry_id"],
        "readiness_registry_epoch": designation["registry_epoch"],
        "readiness_designation_sha256": sha256_hex(designation),
        "acceptance_event_id": event["event_id"],
        "acceptance_event_sha256": sha256_hex(event),
        "acceptance_authority_ref": event["acceptance_authority_ref"],
        "outgoing_custodian_ref": event["outgoing_custodian_ref"],
        "incoming_custodian_ref": event["incoming_custodian_ref"],
        "inventory_sha256": sha256_hex(event["inventory"]),
        "missing_inventory": missing,
        "waivers": sorted(waivers, key=lambda w: w["requirement"]),
        "waiver_set_sha256": sha256_hex(sorted(waivers, key=lambda w: w["requirement"])),
        "continuity_state": event["continuity_state"],
        "handover_acceptance_state": state,
        "operational_custody_accepted": True,
        "inventory_acceptance_state": "ACCEPTED_WITH_WAIVERS" if waivers else "COMPLETE",
        "legal_title_transition_established": False,
        "constitutional_stewardship_transition_established": False,
        "nonclaims": list(NONCLAIMS),
    }
    return QualifiedAcceptance(receipt)

def load_case(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    keys = {"profile", "readiness_receipt", "readiness_designation", "acceptance_event"}
    if not isinstance(data, dict) or set(data) != keys:
        raise AcceptanceError("case: expected exact keys")
    return data["profile"], data["readiness_receipt"], data["readiness_designation"], data["acceptance_event"]

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    a = ap.parse_args()
    receipt = qualify(*load_case(a.case)).receipt()
    text = json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    if a.receipt_out:
        a.receipt_out.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
