#!/usr/bin/env python3
"""Dependency-free executable semantic verifier for the Mycelix RATS contract v0.1.

This harness deliberately models semantic inputs to the verifier and relying party.
The booleans such as signature_valid stand for an upstream cryptographic verification
oracle; this file does not claim to implement TPM quote parsing or signature checks.
"""
from __future__ import annotations

import copy
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs" / "security" / "mycelix-rats-attestation-contract-v0.1.json"


def parse_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(timezone.utc)


def decision(state: str, reason: str) -> tuple[str, str]:
    return state, reason


def verify_evidence(
    evidence: dict[str, Any],
    policy: dict[str, Any],
    challenge_nonce: str,
    now: datetime,
    seen_evidence_ids: set[str],
) -> tuple[str, str]:
    required = policy["required_evidence_claims"]
    missing = [field for field in required if field not in evidence]
    if missing:
        return decision("DENY", "missing-required-claim")

    if not evidence["signature_valid"]:
        return decision("DENY", "bad-signature")

    if evidence["trust_anchor_id"] != policy["trusted_anchor_id"]:
        return decision("DENY", "unknown-trust-anchor")

    if evidence["verifier_profile_id"] != policy["verifier_profile_id"]:
        return decision("DENY", "verifier-profile-substitution")

    if evidence["measurement_profile_id"] != policy["measurement_profile_id"]:
        return decision("DENY", "measurement-profile-substitution")

    if evidence["security_domain"] != policy["security_domain"]:
        return decision("DENY", "cross-domain-evidence")

    if evidence["nonce"] != challenge_nonce:
        return decision("DENY", "wrong-nonce")

    if evidence["audience"] != policy["required_audience"]:
        return decision("DENY", "wrong-audience")

    issued = parse_time(evidence["issued_at"])
    expires = parse_time(evidence["expires_at"])
    if issued > now:
        skew = (issued - now).total_seconds()
        if skew > policy["future_skew_seconds"]:
            return decision("DENY", "future-dated-evidence")

    if expires <= now:
        return decision("DENY", "stale-evidence")

    age = (now - issued).total_seconds()
    if age > policy["max_evidence_age_seconds"]:
        return decision("DENY", "stale-evidence")

    if evidence["evidence_id"] in seen_evidence_ids:
        return decision("DENY", "exact-replay")

    return decision("PASS", "meets-appraisal-policy")


def verify_attestation_result(
    result: dict[str, Any],
    request: dict[str, Any],
    policy: dict[str, Any],
    now: datetime,
) -> tuple[str, str]:
    required = policy["required_result_bindings"]
    missing = [field for field in required if field not in result]
    if missing:
        return decision("DENY", "missing-result-binding")

    if not result["result_signature_valid"]:
        return decision("DENY", "result-bad-signature")

    if result["verifier_profile_id"] != policy["verifier_profile_id"]:
        return decision("DENY", "verifier-profile-substitution")

    if result["security_domain"] != policy["security_domain"]:
        return decision("DENY", "cross-domain-result")

    if result["audience"] != policy["required_audience"]:
        return decision("DENY", "result-wrong-audience")

    if result["nonce"] != request["nonce"]:
        return decision("DENY", "result-replay-or-nonce-mismatch")

    if result["subject_id"] != request["subject_id"]:
        return decision("DENY", "result-subject-substitution")

    if result["device_id"] != request["device_id"]:
        return decision("DENY", "result-device-substitution")

    if result["workload_id"] != request["workload_id"]:
        return decision("DENY", "result-workload-substitution")

    if result["policy_version"] != request["policy_version"]:
        return decision("DENY", "policy-version-downgrade")

    if parse_time(result["issued_at"]) > now:
        return decision("DENY", "future-dated-result")

    if parse_time(result["expires_at"]) <= now:
        return decision("DENY", "result-expired")

    age = (now - parse_time(result["issued_at"])).total_seconds()
    if age > policy["attestation_result_max_age_seconds"]:
        return decision("DENY", "result-too-old")

    return decision("PASS", "attestation-result-appraised")


def authorize(
    result: dict[str, Any],
    request: dict[str, Any],
    policy: dict[str, Any],
    now: datetime,
) -> tuple[str, str]:
    result_state, result_reason = verify_attestation_result(result, request, policy, now)
    if result_state != "PASS":
        return result_state, result_reason

    checks = policy["required_local_authorization"]
    for check in checks:
        if not request.get(check, False):
            return decision("DENY", check)

    # Explicitly prevent an attestation result from becoming a portable bearer
    # authorization: authorization is recomputed against the current request.
    if request["security_domain"] != result["security_domain"]:
        return decision("DENY", "cross-domain-result")

    if request["audience"] != result["audience"]:
        return decision("DENY", "result-wrong-audience")

    return decision("PASS", "local-policy-authorized")


def mutate_evidence(base: dict[str, Any], mutation: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    mutations = {
        "bad-signature": lambda x: x.update(signature_valid=False),
        "unknown-trust-anchor": lambda x: x.update(trust_anchor_id="unknown-anchor"),
        "wrong-nonce": lambda x: x.update(nonce="wrong-nonce"),
        "wrong-audience": lambda x: x.update(audience="other-audience"),
        "stale-evidence": lambda x: x.update(
            issued_at="2026-10-04T11:50:00Z",
            expires_at="2026-10-04T11:55:00Z",
        ),
        "future-dated-evidence": lambda x: x.update(
            issued_at="2026-10-04T13:00:00Z",
            expires_at="2026-10-04T13:05:00Z",
        ),
        "measurement-profile-substitution": lambda x: x.update(
            measurement_profile_id="other-profile"
        ),
        "cross-domain-evidence": lambda x: x.update(security_domain="E2"),
        "missing-required-claim": lambda x: x.pop("workload_measurement"),
        "verifier-profile-substitution": lambda x: x.update(
            verifier_profile_id="other-verifier"
        ),
        "exact-replay": lambda x: x,
        "key-order-permutation": lambda x: dict(reversed(list(x.items()))),
    }
    mutations[mutation](value)
    return value


def mutate_result(base: dict[str, Any], mutation: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    mutations = {
        "canonical-valid": lambda x: x,
        "result-bad-signature": lambda x: x.update(result_signature_valid=False),
        "result-expired": lambda x: x.update(
            expires_at="2026-10-04T11:59:59Z"
        ),
        "result-wrong-audience": lambda x: x.update(audience="other-audience"),
        "result-subject-substitution": lambda x: x.update(
            subject_id="did:example:other-subject"
        ),
        "result-workload-substitution": lambda x: x.update(
            workload_id="workload-other"
        ),
        "policy-version-downgrade": lambda x: x.update(
            policy_version="e1-policy-old"
        ),
        "unauthorized-resource": lambda x: None,
        "unauthorized-purpose": lambda x: None,
        "unauthorized-export-control": lambda x: None,
        "attestation-does-not-grant-subject-authority": lambda x: None,
        "cross-domain-result": lambda x: x.update(security_domain="E2"),
        "missing-result-binding": lambda x: x.pop("evidence_digest"),
        "result-replay-with-new-request-nonce": lambda x: x,
        "unauthorized-delegation": lambda x: None,
    }
    mutator = mutations[mutation]
    if mutation in {
        "unauthorized-resource",
        "unauthorized-purpose",
        "unauthorized-export-control",
        "attestation-does-not-grant-subject-authority",
        "unauthorized-delegation",
    }:
        return value
    mutator(value)
    return value


def run_vector(vector: dict[str, Any], fixture: dict[str, Any], policy: dict[str, Any]) -> tuple[bool, str]:
    now = parse_time(fixture["now"])
    if vector["stage"] == "evidence":
        evidence = mutate_evidence(fixture["canonical_evidence"], vector["mutation"])
        # The replay vector is intentionally evaluated with the canonical evidence
        # already observed; all other evidence gets a fresh set.
        seen = {"evidence-001"} if vector["mutation"] == "exact-replay" else set()
        got, reason = verify_evidence(
            evidence,
            policy,
            fixture["challenge_nonce"],
            now,
            seen,
        )
    else:
        result = mutate_result(fixture["canonical_result"], vector["mutation"])
        request = copy.deepcopy(fixture["canonical_request"])
        if vector["mutation"] == "unauthorized-resource":
            request["resource_authorized"] = False
        elif vector["mutation"] == "unauthorized-purpose":
            request["purpose_authorized"] = False
        elif vector["mutation"] == "unauthorized-export-control":
            request["export_control_authorized"] = False
        elif vector["mutation"] == "attestation-does-not-grant-subject-authority":
            request["subject_authorized"] = False
        elif vector["mutation"] == "unauthorized-delegation":
            request["delegation_authorized"] = False
        elif vector["mutation"] == "result-replay-with-new-request-nonce":
            request["nonce"] = "nonce-NEW"
        got, reason = authorize(result, request, policy, now)

    expected = vector["expected"]
    ok = got == expected
    return ok, f'{vector["id"]}: expected={expected} got={got} reason={reason}'


def main() -> int:
    with CONTRACT.open("r", encoding="utf-8") as handle:
        contract = json.load(handle)

    policy = contract["policy"]
    fixture = contract["fixture"]
    failures = []

    for vector in contract["vectors"]:
        ok, line = run_vector(vector, fixture, policy)
        marker = "PASS" if ok else "FAIL"
        print(f"[{marker}] {line}")
        if not ok:
            failures.append(line)

    print()
    print(
        f"RATS semantic qualification: {len(contract['vectors']) - len(failures)}/"
        f"{len(contract['vectors'])} vectors passed"
    )
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
