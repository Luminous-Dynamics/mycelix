#!/usr/bin/env python3
"""Fail-closed structural validator for AMSAP-001."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import sys

EXPECTED_SCHEMA = "mycelix-amsap-constitution-v0.1"
EXPECTED_PARENT = "303f624152120b78ed41b2f8ebe6165b324a5ef2"
EXPECTED_AXES = ["C", "V", "A", "I", "R", "L", "G"]
EXPECTED_EVIDENCE_BANDS = ["UNASSESSED", "WEAK", "MODERATE", "STRONG", "EXCEPTIONAL", "CONTESTED"]
EXPECTED_PROTECTIONS = ["P0", "P1", "P2", "P3"]
EXPECTED_IMPACTS = ["W0", "W1", "W2", "W3"]
EXPECTED_DECISIONS = ["ALLOW", "ALLOW_WITH_SAFEGUARDS", "REQUIRE_INDEPENDENT_REVIEW", "TEMPORARILY_PRESERVE_AND_REVIEW", "DENY", "EMERGENCY_CONTAIN"]
REQUIRED_DERIVATIONS = {
    "intelligence->consciousness", "self_report->personhood", "agency->consciousness",
    "consciousness->legal_standing", "valence->legal_standing", "consciousness->governance",
    "valence->governance", "agency->governance", "protection_bundle->governance",
    "instance_count->vote_count", "compute->population", "wealth->artificial_population",
    "artificial_status->liability_erasure",
}
REQUIRED_INVARIANTS = [f"AMSAP-C{i:02d}" for i in range(1, 16)]


def fail(message: str):
    raise SystemExit(f"AMSAP-001 validation failed: {message}")


def reject_probability_keys(value, path="root"):
    if isinstance(value, dict):
        for key, child in value.items():
            lowered = key.lower()
            if "consciousness_probability" in lowered or "sentience_probability" in lowered:
                fail(f"unsupported pseudo-precision field at {path}.{key}")
            reject_probability_keys(child, f"{path}.{key}")
    elif isinstance(value, list):
        for i, child in enumerate(value):
            reject_probability_keys(child, f"{path}[{i}]")


def main() -> int:
    policy_path = Path(__file__).resolve().parents[1] / "docs" / "amsap" / "amsap-v0.1.json"
    raw = policy_path.read_bytes()
    policy = json.loads(raw)

    if policy.get("schema") != EXPECTED_SCHEMA:
        fail("wrong schema")
    if policy.get("program") != "AMSAP-001":
        fail("wrong program id")
    if policy.get("parent", {}).get("program") != "AC-001":
        fail("wrong constitutional parent")
    if policy.get("parent", {}).get("commit") != EXPECTED_PARENT:
        fail("wrong exact AC-001 parent")

    axes = policy.get("axes")
    if not isinstance(axes, list) or [axis.get("id") for axis in axes] != EXPECTED_AXES:
        fail("axes must be exactly C,V,A,I,R,L,G in canonical order")
    if len({axis["id"] for axis in axes}) != len(EXPECTED_AXES):
        fail("duplicate axis")
    for axis in axes:
        if axis.get("automatic_authority") is not False:
            fail(f"{axis['id']} may not mint automatic authority")
    for axis in axes[:5]:
        if axis.get("kind") not in {"scientific", "observational"}:
            fail(f"{axis['id']} must remain scientific/observational")
    for axis in axes[5:]:
        if axis.get("kind") != "explicit_grant":
            fail(f"{axis['id']} must remain an explicit grant")

    if policy.get("evidence_bands") != EXPECTED_EVIDENCE_BANDS:
        fail("evidence band vocabulary drift")
    profile_fields = policy.get("evidence_profile_fields")
    if not isinstance(profile_fields, list) or len(profile_fields) != 10 or len(set(profile_fields)) != 10:
        fail("evidence profile must contain exactly ten unique dimensions")

    protections = policy.get("protection_bundles")
    if [p.get("id") for p in protections or []] != EXPECTED_PROTECTIONS:
        fail("protection bundle vocabulary drift")
    for protection in protections:
        if protection.get("implies_personhood") is not False:
            fail(f"{protection['id']} may not imply personhood")
        if protection.get("implies_governance") is not False:
            fail(f"{protection['id']} may not imply governance")

    if policy.get("welfare_impact_classes") != EXPECTED_IMPACTS:
        fail("welfare impact vocabulary drift")
    if policy.get("decision_outputs") != EXPECTED_DECISIONS:
        fail("decision output vocabulary drift")
    if policy.get("default_artificial_legal_standing") != "L0":
        fail("default artificial legal standing must remain L0")
    if policy.get("default_artificial_governance_authority") != "G0":
        fail("default artificial governance authority must remain G0")

    derivations = policy.get("prohibited_derivations")
    if set(derivations or []) != REQUIRED_DERIVATIONS or len(derivations) != len(REQUIRED_DERIVATIONS):
        fail("prohibited derivation set drift")

    invariants = policy.get("invariants")
    ids = [item.get("id") for item in invariants or []]
    if ids != REQUIRED_INVARIANTS:
        fail("constitutional invariant IDs must be AMSAP-C01..AMSAP-C15 exactly")
    if any(not item.get("text") for item in invariants):
        fail("empty invariant text")

    required_nonclaims = {
        "current_ai_consciousness", "current_ai_nonconsciousness", "legal_personhood",
        "political_authority", "deployment_currentness", "external_effect_authority",
    }
    if set(policy.get("nonclaims") or []) != required_nonclaims:
        fail("nonclaim set drift")

    reject_probability_keys(policy)
    canonical = json.dumps(policy, sort_keys=True, separators=(",", ":")).encode()
    digest = hashlib.sha256(canonical).hexdigest()
    print(f"AMSAP-001 OK {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
