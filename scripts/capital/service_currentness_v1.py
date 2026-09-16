#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-service-currentness-designated-v1"
RECEIPT_VERSION = "mycelix-service-currentness-receipt-v1"
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
_HEX40 = re.compile(r"^[0-9a-f]{40}$")
_HEX64 = re.compile(r"^[0-9a-f]{64}$")

PROFILE_KEYS = {
    "profile_version", "project_id", "service_gate_subject_sha",
    "service_gate_profile_sha256", "designation_registry_id", "max_events",
}
DESIGNATION_KEYS = {
    "designation_id", "registry_id", "registry_epoch", "project_id",
    "profile_sha256", "designated_measurement_id",
    "designated_service_receipt_sha256", "designation_state",
    "authority_ref", "evidence_ref",
}
EVENT_KEYS = {
    "seq", "event_id", "project_id", "profile_sha256", "designation_id",
    "prev_event_sha256", "kind", "target_measurement_id",
    "authority_ref", "evidence_ref",
}
DESIGNATION_STATES = {"ACTIVE", "PENDING", "REVOKED"}
EVENT_KINDS = {"MaterialInvalidation", "PendingRemeasurement", "RevokeEvidence"}
PRECEDENCE = {"CURRENT": 0, "STALE": 1, "PENDING": 2, "REVOKED": 3}
NONCLAIMS = (
    "historical service validity is not current distribution authority",
    "currentness does not establish truth of external service measurements",
    "currentness does not establish authenticity of external authority or evidence references",
    "currentness does not mutate or increase the investor claim",
    "currentness is relative to the supplied designation lineage, not local wall-clock time",
    "CURRENT does not itself establish distribution eligibility",
)

class CurrentnessError(ValueError):
    pass

@dataclass(frozen=True)
class QualifiedCurrentness:
    _receipt: dict[str, Any]
    def receipt(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._receipt))

def canonical_bytes(v: Any) -> bytes:
    return json.dumps(v, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")

def sha256_hex(v: Any) -> str:
    return hashlib.sha256(canonical_bytes(v)).hexdigest()

def _exact(obj: dict[str, Any], keys: set[str], ctx: str) -> None:
    if set(obj) != keys:
        raise CurrentnessError(f"{ctx}: key mismatch missing={sorted(keys-set(obj))} unknown={sorted(set(obj)-keys)}")

def _id(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not _ID.fullmatch(v):
        raise CurrentnessError(f"{ctx}: invalid bounded identifier")
    return v

def _ref(v: Any, ctx: str) -> str:
    if not isinstance(v, str) or not v or len(v.encode("utf-8")) > 512 or v != v.strip():
        raise CurrentnessError(f"{ctx}: invalid reference")
    return v

def validate_profile(p: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(p, dict):
        raise CurrentnessError("profile: expected object")
    _exact(p, PROFILE_KEYS, "profile")
    if p["profile_version"] != PROFILE_VERSION:
        raise CurrentnessError("profile.profile_version: unsupported")
    _id(p["project_id"], "profile.project_id")
    _id(p["designation_registry_id"], "profile.designation_registry_id")
    if not isinstance(p["service_gate_subject_sha"], str) or not _HEX40.fullmatch(p["service_gate_subject_sha"]):
        raise CurrentnessError("profile.service_gate_subject_sha: invalid")
    if not isinstance(p["service_gate_profile_sha256"], str) or not _HEX64.fullmatch(p["service_gate_profile_sha256"]):
        raise CurrentnessError("profile.service_gate_profile_sha256: invalid")
    m = p["max_events"]
    if isinstance(m, bool) or not isinstance(m, int) or m < 0 or m > 100_000:
        raise CurrentnessError("profile.max_events: invalid")
    return p

def validate_service_receipt(r: dict[str, Any], p: dict[str, Any]) -> None:
    if not isinstance(r, dict):
        raise CurrentnessError("service_receipt: expected object")
    for k in ("project_id", "profile_sha256", "measurement_id", "snapshot_sha256", "distribution_eligibility"):
        if k not in r:
            raise CurrentnessError(f"service_receipt: missing {k}")
    if r["project_id"] != p["project_id"]:
        raise CurrentnessError("service_receipt: project mismatch")
    if r["profile_sha256"] != p["service_gate_profile_sha256"]:
        raise CurrentnessError("service_receipt: service profile mismatch")
    _id(r["measurement_id"], "service_receipt.measurement_id")
    if not isinstance(r["snapshot_sha256"], str) or not _HEX64.fullmatch(r["snapshot_sha256"]):
        raise CurrentnessError("service_receipt.snapshot_sha256: invalid")

def validate_designation(d: dict[str, Any], p: dict[str, Any]) -> None:
    if not isinstance(d, dict):
        raise CurrentnessError("designation: expected object")
    _exact(d, DESIGNATION_KEYS, "designation")
    _id(d["designation_id"], "designation.designation_id")
    if d["registry_id"] != p["designation_registry_id"]:
        raise CurrentnessError("designation.registry_id: registry substitution")
    if d["project_id"] != p["project_id"]:
        raise CurrentnessError("designation.project_id: project substitution")
    if d["profile_sha256"] != sha256_hex(p):
        raise CurrentnessError("designation.profile_sha256: profile substitution")
    e = d["registry_epoch"]
    if isinstance(e, bool) or not isinstance(e, int) or e < 0 or e > 10**18:
        raise CurrentnessError("designation.registry_epoch: invalid")
    _id(d["designated_measurement_id"], "designation.designated_measurement_id")
    if not isinstance(d["designated_service_receipt_sha256"], str) or not _HEX64.fullmatch(d["designated_service_receipt_sha256"]):
        raise CurrentnessError("designation.designated_service_receipt_sha256: invalid")
    if d["designation_state"] not in DESIGNATION_STATES:
        raise CurrentnessError("designation.designation_state: unsupported")
    _ref(d["authority_ref"], "designation.authority_ref")
    _ref(d["evidence_ref"], "designation.evidence_ref")

def validate_events(events: Any, p: dict[str, Any], d: dict[str, Any]) -> list[dict[str, Any]]:
    if not isinstance(events, list):
        raise CurrentnessError("events: expected array")
    if len(events) > p["max_events"]:
        raise CurrentnessError("events: exceeds max_events")
    psha = sha256_hex(p)
    out = []
    ids = set()
    for i, ev in enumerate(events):
        if not isinstance(ev, dict):
            raise CurrentnessError(f"events[{i}]: expected object")
        _exact(ev, EVENT_KEYS, f"events[{i}]")
        if ev["seq"] != i:
            raise CurrentnessError(f"events[{i}].seq: expected {i}")
        eid = _id(ev["event_id"], f"events[{i}].event_id")
        if eid in ids:
            raise CurrentnessError(f"events[{i}].event_id: duplicate")
        ids.add(eid)
        if ev["project_id"] != p["project_id"]:
            raise CurrentnessError(f"events[{i}].project_id: project substitution")
        if ev["profile_sha256"] != psha:
            raise CurrentnessError(f"events[{i}].profile_sha256: profile substitution")
        if ev["designation_id"] != d["designation_id"]:
            raise CurrentnessError(f"events[{i}].designation_id: designation substitution")
        prev = ev["prev_event_sha256"]
        if i == 0:
            if prev is not None:
                raise CurrentnessError("events[0].prev_event_sha256: must be null")
        elif prev != sha256_hex(out[-1]):
            raise CurrentnessError(f"events[{i}].prev_event_sha256: broken event chain")
        if ev["kind"] not in EVENT_KINDS:
            raise CurrentnessError(f"events[{i}].kind: unsupported")
        _id(ev["target_measurement_id"], f"events[{i}].target_measurement_id")
        _ref(ev["authority_ref"], f"events[{i}].authority_ref")
        _ref(ev["evidence_ref"], f"events[{i}].evidence_ref")
        out.append(ev)
    return out

def _raise_state(current: str, candidate: str) -> str:
    return candidate if PRECEDENCE[candidate] > PRECEDENCE[current] else current

def qualify(p: dict[str, Any], service_receipt: dict[str, Any], designation: dict[str, Any], events: Any) -> QualifiedCurrentness:
    validate_profile(p)
    validate_service_receipt(service_receipt, p)
    validate_designation(designation, p)
    history = validate_events(events, p, designation)

    measurement_id = service_receipt["measurement_id"]
    service_digest = sha256_hex(service_receipt)
    state = "CURRENT"
    blockers: list[str] = []

    if designation["designated_measurement_id"] != measurement_id or designation["designated_service_receipt_sha256"] != service_digest:
        state = _raise_state(state, "STALE")
        blockers.append("NOT_DESIGNATED_CURRENT")

    if designation["designation_state"] == "PENDING":
        state = _raise_state(state, "PENDING")
        blockers.append("DESIGNATION_PENDING")
    elif designation["designation_state"] == "REVOKED":
        state = _raise_state(state, "REVOKED")
        blockers.append("DESIGNATION_REVOKED")

    for ev in history:
        if ev["target_measurement_id"] != measurement_id:
            continue
        if ev["kind"] == "MaterialInvalidation":
            state = _raise_state(state, "STALE")
            blockers.append("MATERIAL_INVALIDATION")
        elif ev["kind"] == "PendingRemeasurement":
            state = _raise_state(state, "PENDING")
            blockers.append("PENDING_REMEASUREMENT")
        elif ev["kind"] == "RevokeEvidence":
            state = _raise_state(state, "REVOKED")
            blockers.append("EVIDENCE_REVOKED")

    blockers = sorted(set(blockers))
    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": PROFILE_VERSION,
        "project_id": p["project_id"],
        "profile_sha256": sha256_hex(p),
        "service_gate_subject_sha": p["service_gate_subject_sha"],
        "service_gate_profile_sha256": p["service_gate_profile_sha256"],
        "service_receipt_sha256": service_digest,
        "measurement_id": measurement_id,
        "service_distribution_eligibility": service_receipt["distribution_eligibility"],
        "designation_id": designation["designation_id"],
        "designation_registry_id": designation["registry_id"],
        "designation_registry_epoch": designation["registry_epoch"],
        "designation_sha256": sha256_hex(designation),
        "event_history_sha256": sha256_hex(history),
        "event_chain_tip_sha256": None if not history else sha256_hex(history[-1]),
        "event_count": len(history),
        "currentness_state": state,
        "blockers": blockers,
        "claim_modified": False,
        "uses_local_wall_clock": False,
        "nonclaims": list(NONCLAIMS),
    }
    return QualifiedCurrentness(receipt)

def load_case(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != {"profile", "service_receipt", "designation", "events"}:
        raise CurrentnessError("case: expected exact keys")
    return data["profile"], data["service_receipt"], data["designation"], data["events"]

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("case", type=Path)
    ap.add_argument("--receipt-out", type=Path)
    a = ap.parse_args()
    p, r, d, ev = load_case(a.case)
    receipt = qualify(p, r, d, ev).receipt()
    text = json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    if a.receipt_out:
        a.receipt_out.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
