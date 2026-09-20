#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DISPOSITIONS = (
    "AdmittedProtectedReadUnderProfile",
    "Refused",
    "AuthorizationUnproven",
    "PurposeExpiredOrOutOfScope",
    "StorageUnavailable",
    "ObjectAbsent",
    "ObjectStale",
    "ObjectConflict",
    "EnvelopeInvalid",
    "RecipientKeyStateInvalidOrStale",
    "AccountabilityCommitFailed",
    "RequiredAttestationMissing",
    "OfflineLeaseExpiredOrUnproven",
    "BreakGlassAuthorityUnproven",
    "ReleaseProjectionRequired",
)


def decide(i: dict[str, Any]) -> str:
    # 1. classification / topology fit
    if i["data_classified"] is not True:
        return "Refused"
    if i["storage_profile"] != "admitted":
        return "Refused"

    # 2. storage availability
    if i["storage_availability"] != "available":
        return "StorageUnavailable"

    # 3. exact object/currentness
    object_map = {
        "absent": "ObjectAbsent",
        "stale": "ObjectStale",
        "conflict": "ObjectConflict",
    }
    if i["object_state"] in object_map:
        return object_map[i["object_state"]]
    if i["object_state"] != "current":
        return "Refused"

    # 4. envelope validity, when required
    if i["envelope_required"]:
        if i["envelope_state"] != "valid":
            return "EnvelopeInvalid"
    elif i["envelope_state"] not in {"not_applicable", "valid"}:
        return "Refused"

    # 5. recipient-key state, when required
    if i["recipient_required"]:
        if i["recipient_state"] != "admitted":
            return "RecipientKeyStateInvalidOrStale"
    elif i["recipient_state"] not in {"not_applicable", "admitted"}:
        return "Refused"

    # 6. current authorization. receipt_outcome_hint is intentionally ignored.
    if i["authorization_state"] != "current":
        return "AuthorizationUnproven"

    # 7. purpose / scope
    if i["purpose_state"] != "current":
        return "PurposeExpiredOrOutOfScope"

    # 8. evidence roles
    if i["attestations_state"] != "ready":
        return "RequiredAttestationMissing"

    # 9. special mode gates
    mode = i["mode"]
    if mode == "offline":
        if i["offline_lease_state"] != "valid":
            return "OfflineLeaseExpiredOrUnproven"
    elif mode == "break_glass":
        if i["break_glass_authority_state"] != "valid":
            return "BreakGlassAuthorityUnproven"
    elif mode != "online":
        return "Refused"

    # 10. commit-before-disclose accountability
    if i["accountability_state"] != "committed":
        return "AccountabilityCommitFailed"

    # 11. final pre-disclosure revalidation/linearization
    lin = i["linearization_state"]
    if lin == "object_changed":
        return "ObjectStale"
    if lin == "recipient_key_changed":
        return "RecipientKeyStateInvalidOrStale"
    if lin == "purpose_expired":
        return "PurposeExpiredOrOutOfScope"
    if lin in {"authorization_changed", "policy_changed", "unavailable"}:
        return "AuthorizationUnproven"
    if lin != "unchanged":
        return "Refused"

    # 12. protected read != publication
    if i["destination"] == "public_release":
        return "ReleaseProjectionRequired"
    if i["destination"] != "protected_reader":
        return "Refused"

    return "AdmittedProtectedReadUnderProfile"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--receipt", required=True)
    args = ap.parse_args()

    manifest = json.loads(Path(args.manifest).read_text())
    assert manifest["schema"] == "sup-civ-000d3b-admission-oracle-v1"
    assert manifest["parent_exact_head"] == "4ebb8389c1f3a11492ec417a0f1f49ec7a6f8d75"
    assert tuple(manifest["dispositions"]) == DISPOSITIONS

    results = []
    for row in manifest["cases"]:
        got = decide(row["input"])
        expected = row["expected"]
        if got != expected:
            raise AssertionError(f"{row['id']}: expected {expected}, got {got}")
        results.append({"id": row["id"], "disposition": got})

    admitted = sum(
        r["disposition"] == "AdmittedProtectedReadUnderProfile" for r in results
    )
    receipt = {
        "schema": "sup-civ-000d3b-admission-oracle-receipt-v1",
        "authority": "synthetic-composition-reference-only",
        "parent_exact_head": manifest["parent_exact_head"],
        "case_count": len(results),
        "admit_count": admitted,
        "non_admit_count": len(results) - admitted,
        "results": results,
    }
    Path(args.receipt).write_text(
        json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n"
    )
    print(
        f"PASS_SUP_CIV_000D3B cases={len(results)} "
        f"admitted={admitted} non_admit={len(results) - admitted}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
