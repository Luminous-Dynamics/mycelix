#!/usr/bin/env python3
"""Dependency-free fail-closed PEP decision algebra."""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-pep-decision-boundary-v0.1.json"


REQUIRED = [
    "subject_identity",
    "device_posture",
    "workload_identity",
    "security_domain",
    "policy_version",
    "freshness",
    "purpose",
    "resource_authorization",
    "releasability",
    "export_control",
    "delegation",
]


def evaluate(inputs: dict[str, str]) -> tuple[str, str]:
    unknown = [k for k, v in inputs.items() if k in REQUIRED and v not in {"PASS", "DENY", "INDETERMINATE"}]
    if unknown:
        return "INDETERMINATE", "unknown-input-state"

    missing = [k for k in REQUIRED if k not in inputs]
    if missing:
        return "INDETERMINATE", "missing-required-input"

    denied = [k for k in REQUIRED if inputs[k] == "DENY"]
    if denied:
        return "DENY", "required-input-deny:" + denied[0]

    indeterminate = [k for k in REQUIRED if inputs[k] == "INDETERMINATE"]
    if indeterminate:
        return "INDETERMINATE", "required-input-indeterminate:" + indeterminate[0]

    return "PASS", "all-required-inputs-pass"


def mutate(base: dict[str, str], mutation: str) -> dict[str, str]:
    value = copy.deepcopy(base)
    if mutation == "canonical-all-pass":
        return value
    if mutation == "input-order-permutation":
        return dict(reversed(list(value.items())))
    if mutation == "same-inputs-repeat":
        return value
    mapping = {
        "subject-identity-deny": "subject_identity",
        "device-posture-deny": "device_posture",
        "workload-identity-deny": "workload_identity",
        "security-domain-deny": "security_domain",
        "policy-version-deny": "policy_version",
        "freshness-deny": "freshness",
        "purpose-deny": "purpose",
        "resource-authorization-deny": "resource_authorization",
        "releasability-deny": "releasability",
        "export-control-deny": "export_control",
        "delegation-deny": "delegation",
        "policy-revoked": "policy_version",
        "release-receipt-without-current-authorization": "resource_authorization",
        "network-reachability-without-resource-authorization": "resource_authorization",
        "freshness-indeterminate": "freshness",
        "deny-plus-indeterminate": "subject_identity",
        "all-indeterminate": "subject_identity",
        "identity-pass-device-workload-deny": "device_posture",
        "decision-output-bearer-token-attempt": "delegation",
        "unknown-input-state": "purpose",
        "attestation-result-alone": "resource_authorization",
    }
    if mutation == "deny-plus-indeterminate":
        value["subject_identity"] = "DENY"
        value["freshness"] = "INDETERMINATE"
        return value
    if mutation == "all-indeterminate":
        for k in REQUIRED:
            value[k] = "INDETERMINATE"
        return value
    if mutation == "policy-revoked":
        value["policy_version"] = "DENY"
        return value
    if mutation == "decision-output-bearer-token-attempt":
        value["delegation"] = "DENY"
        return value
    if mutation == "freshness-indeterminate":
        value["freshness"] = "INDETERMINATE"
        return value
    if mutation == "unknown-input-state":
        value["purpose"] = "UNKNOWN"
        return value
    if mutation == "attestation-result-alone":
        value["resource_authorization"] = "DENY"
        return value
    if mutation in mapping:
        value[mapping[mutation]] = "DENY"
        return value
    raise KeyError(mutation)


def main() -> int:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    base = contract["fixture"]["inputs"]
    failures: list[str] = []

    for vector in contract["vectors"]:
        inputs = mutate(base, vector["mutation"])
        got, reason = evaluate(inputs)
        if vector["mutation"] == "same-inputs-repeat":
            got, reason = evaluate(mutate(base, vector["mutation"]))
        ok = got == vector["expected"]
        marker = "PASS" if ok else "FAIL"
        line = f'[{marker}] {vector["id"]}: expected={vector["expected"]} got={got} reason={reason}'
        print(line)
        if not ok:
            failures.append(line)

    # Explicit output-shape guard: the evaluator has no bearer token/capability output.
    forbidden_outputs = {"bearer_capability", "portable_authorization_token", "release_receipt"}
    if forbidden_outputs & set(contract.get("output_semantics", {})):
        sem = contract["output_semantics"]
        if any(sem.get(k, False) for k in forbidden_outputs):
            failures.append("output-semantics-forbidden-value-enabled")

    passed = len(contract["vectors"]) - len(failures)
    print()
    print(f"PEP decision qualification: {passed}/{len(contract['vectors'])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
