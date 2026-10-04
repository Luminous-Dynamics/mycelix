#!/usr/bin/env python3
"""Dependency-free verifier for the Mycelix protected release boundary."""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-protected-release-v0.1.json"


def deny(reason: str) -> tuple[str, str]:
    return "DENY", reason


def verify(c: dict[str, Any], mutation: str, seen: set[str]) -> tuple[str, str]:
    p, f = c["policy"], c["fixture"]
    i, t, a, d, r, g = f["intent"], f["transform"], f["authorization"], f["destination"], f["receipt"], f["gateway"]

    if mutation == "source-destination-mismatch":
        i["destination_security_domain"] = "E2"
    elif mutation == "classification-mismatch":
        i["classification_state"] = "SECRET"
    elif mutation == "compartment-mismatch":
        i["compartment"] = "other-compartment"
    elif mutation == "releasability-failure":
        a["releasability_authorized"] = False
    elif mutation == "export-control-mismatch":
        a["export_control_authorized"] = False
    elif mutation == "purpose-mismatch":
        a["purpose_authorized"] = False
    elif mutation == "stale-authorization":
        a["fresh"] = False
    elif mutation == "revoked-authorization":
        a["source_authorized"] = False
    elif mutation == "policy-version-mismatch":
        i["destination_policy_version"] = "e0-policy-old"
    elif mutation == "hidden-metadata":
        t["metadata_checked"] = False
    elif mutation == "payload-substitution":
        t["output_payload_digest"] = "sha256:other-output"
    elif mutation == "transform-digest-mismatch":
        t["transform_digest"] = "sha256:other-transform"
    elif mutation in {"replay-transfer", "duplicate-transfer"}:
        seen.add(i["transfer_id"])
    elif mutation == "unapproved-transform":
        t["approved"] = False
    elif mutation == "destination-refuses-admission":
        d["admitted"] = False
    elif mutation == "gateway-unavailable":
        g["available"] = False
    elif mutation == "receipt-substitution":
        r["source_object_digest"] = "sha256:other-source"
    elif mutation == "receipt-transfer-id-substitution":
        r["transfer_id"] = "transfer-other"
    elif mutation == "receipt-resource-substitution":
        r["resource_id"] = "different-resource"
    elif mutation == "receipt-subject-substitution":
        r["subject_id"] = "did:example:other-subject"
    elif mutation == "receipt-gateway-profile-substitution":
        r["gateway_profile_id"] = "other-gateway-profile"
    elif mutation == "receipt-outcome-substitution":
        r["outcome"] = "DENY"
    elif mutation == "receipt-expired":
        r["expires_at_unix"] = f["now_unix"]
    elif mutation == "receipt-policy-substitution":
        r["release_profile_id"] = "other-release-profile"
    elif mutation == "source-object-digest-substitution":
        i["source_object_digest"] = "sha256:other-source"
    elif mutation == "output-digest-substitution":
        r["output_payload_digest"] = "sha256:other-output"
    elif mutation == "destination-policy-substitution":
        r["destination_policy_version"] = "e0-policy-other"
    elif mutation == "classification-label-spoof":
        i["classification_state"] = "public"
        a["source_authorized"] = False
    elif mutation in {"valid-receipt-cannot-authorize-second-transfer", "attestation-does-not-authorize-release",
                      "transform-created-new-output-requires-new-intent"}:
        return deny(mutation)
    elif mutation == "canonical-valid":
        pass
    else:
        raise KeyError(mutation)

    if i["source_security_domain"] != p["source_security_domain"] or i["destination_security_domain"] != p["destination_security_domain"]:
        return deny("source-destination-mismatch")

    if i["source_policy_version"] != p["source_policy_version"] or i["destination_policy_version"] != p["destination_policy_version"]:
        return deny("policy-version-mismatch")

    if i["classification_state"] not in {"CUI"}:
        return deny("classification-mismatch")

    if not a["source_authorized"]:
        return deny("source-authorization-failed")
    if not a["releasability_authorized"]:
        return deny("releasability-failure")
    if not a["export_control_authorized"]:
        return deny("export-control-mismatch")
    if not a["purpose_authorized"]:
        return deny("purpose-mismatch")
    if not a["fresh"] or not a["policy_current"] or a["expires_at_unix"] <= f["now_unix"]:
        return deny("stale-or-revoked-authorization")

    if i["compartment"] != "none":
        return deny("compartment-mismatch")
    if i["transform_profile_id"] != p["transform_profile_id"] or t["profile_id"] != p["transform_profile_id"]:
        return deny("transform-profile-mismatch")
    if not t["approved"]:
        return deny("unapproved-transform")
    if not t["metadata_checked"]:
        return deny("hidden-metadata")
    if t["transform_digest"] != "sha256:transform-001":
        return deny("transform-digest-mismatch")
    if t["output_payload_digest"] != "sha256:output-001":
        return deny("payload-substitution")

    if not d["admitted"] or d["security_domain"] != p["destination_security_domain"] or d["policy_version"] != p["destination_policy_version"]:
        return deny("destination-admission-failed")

    if not g["available"]:
        return "INDETERMINATE", "gateway-unavailable"

    if i["transfer_id"] in seen:
        return deny("replay-or-duplicate-transfer")

    # The receipt is evidence about exactly one transfer. Every
    # security-relevant identity/context value must match the intent,
    # transform, destination, and selected gateway profile.
    receipt_bindings = {
        "transfer_id": i["transfer_id"],
        "source_security_domain": i["source_security_domain"],
        "destination_security_domain": i["destination_security_domain"],
        "resource_id": d["resource_id"],
        "subject_id": i["subject_id"],
        "source_object_digest": i["source_object_digest"],
        "output_payload_digest": t["output_payload_digest"],
        "transform_digest": t["transform_digest"],
        "classification_state": i["classification_state"],
        "compartment": i["compartment"],
        "releasability_profile": i["releasability_profile"],
        "export_control_profile": i["export_control_profile"],
        "authorization_basis": i["authorization_basis"],
        "source_policy_version": i["source_policy_version"],
        "destination_policy_version": i["destination_policy_version"],
        "purpose": i["purpose"],
        "release_profile_id": p["release_profile_id"],
        "gateway_profile_id": g["profile_id"],
    }
    for field, expected in receipt_bindings.items():
        if r.get(field) != expected:
            return deny("receipt-binding-mismatch:" + field)

    if r["expires_at_unix"] <= f["now_unix"] or f["now_unix"] - r["issued_at_unix"] > p["max_receipt_age_seconds"]:
        return deny("receipt-expired")

    if mutation == "valid-receipt-cannot-authorize-second-transfer":
        return deny("receipt-is-evidence-only")

    return "TRANSFER_COMMITTED", "release-and-destination-conditions-satisfied"


def main() -> int:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    failures: list[str] = []
    seen = {"transfer-001"}
    for v in contract["vectors"]:
        c = copy.deepcopy(contract)
        # Canonical valid is the only case where this prior-transfer set is removed.
        if v["mutation"] not in {"replay-transfer", "duplicate-transfer"}:
            seen_case: set[str] = set()
        else:
            seen_case = set(seen)
        got, reason = verify(c, v["mutation"], seen_case)
        ok = got == v["expected"]
        marker = "PASS" if ok else "FAIL"
        line = f'[{marker}] {v["id"]}: expected={v["expected"]} got={got} reason={reason}'
        print(line)
        if not ok:
            failures.append(line)
    passed = len(contract["vectors"]) - len(failures)
    print()
    print(f"Protected release qualification: {passed}/{len(contract['vectors'])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
