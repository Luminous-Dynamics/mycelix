#!/usr/bin/env python3
"""Dependency-free interval-aware RATS freshness verifier."""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs" / "security" / "mycelix-rats-trusted-time-binding-v0.1.json"


def decision(state: str, reason: str) -> tuple[str, str]:
    return state, reason


def validate_time_evidence(
    now: dict[str, Any],
    policy: dict[str, Any],
    request: dict[str, Any],
) -> tuple[str, str]:
    if not now.get("available", False) or not now.get("structurally_valid", False):
        return decision("INDETERMINATE", "time-source-unavailable-or-invalid")

    if not now.get("policy_trusted", False):
        return decision("INDETERMINATE", "time-not-policy-trusted")

    if now["local_clock_only"] if "local_clock_only" in now else False:
        return decision("DENY", "local-clock-not-authoritative")

    if now["earliest"] > now["latest"]:
        return decision("DENY", "reversed-time-interval")

    if now["latest"] - now["earliest"] > policy["max_interval_width_seconds"]:
        return decision("DENY", "excessive-interval-width")

    exact_pairs = (
        ("source_namespace", "time_source_namespace"),
        ("source_instance", "time_source_instance"),
        ("source_trust_domain", "time_source_trust_domain"),
        ("source_profile_id", "time_source_profile_id"),
        ("source_profile_version", "time_source_profile_version"),
        ("time_scale_profile", "time_scale_profile"),
        ("verifier_profile_id", "verifier_profile_id"),
    )
    for observed, expected in exact_pairs:
        if now[observed] != policy[expected]:
            return decision("DENY", expected + "-mismatch")

    if policy["require_request_binding"] and (
        now.get("request_binding_sha256") != request["request_binding_sha256"]
    ):
        return decision("DENY", "missing-or-wrong-request-binding")

    return decision("PASS", "policy-trusted-time-admitted")


def freshness(
    now: dict[str, Any],
    result: dict[str, Any],
    policy: dict[str, Any],
) -> tuple[str, str]:
    lo, hi = now["earliest"], now["latest"]
    issued, expires = result["issued_at"], result["expires_at"]

    # Validity is [issued, expires). If the entire trusted-now interval is
    # before issuance, the result is certainly future-dated. If only part of
    # it is before issuance, timing is ambiguous and must not produce PASS.
    if hi < issued:
        return decision("DENY", "result-entirely-future-dated")
    if lo < issued <= hi:
        return decision("INDETERMINATE", "issuance-straddles-trusted-time")

    if lo >= expires:
        return decision("DENY", "result-entirely-expired")
    if lo < expires <= hi:
        return decision("INDETERMINATE", "expiry-straddles-trusted-time")

    min_age = lo - issued
    max_age = hi - issued
    if min_age > policy["max_result_age_seconds"]:
        return decision("DENY", "result-too-old")
    if max_age > policy["max_result_age_seconds"]:
        return decision("INDETERMINATE", "max-age-straddles-trusted-time")

    return decision("PASS", "result-fresh-under-entire-trusted-interval")


def mutate_time(base: dict[str, Any], mutation: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    if mutation == "canonical-valid":
        return value
    if mutation == "interval-entirely-after-expiry":
        value.update(earliest=1791115231, latest=1791115232)
    elif mutation == "interval-entirely-before-issuance":
        value.update(earliest=1791115160, latest=1791115161)
    elif mutation in {"interval-straddles-expiry", "nonce-valid-but-time-ambiguous"}:
        value.update(earliest=1791115228, latest=1791115232)
    elif mutation == "interval-straddles-max-age":
        value.update(earliest=1791115225, latest=1791115232)
    elif mutation == "time-source-unavailable":
        value["available"] = False
    elif mutation == "unadmitted-source-profile":
        value["source_profile_id"] = "other-profile"
    elif mutation == "trust-domain-substitution":
        value["source_trust_domain"] = "other-operator"
    elif mutation == "verifier-profile-substitution":
        value["verifier_profile_id"] = "other-verifier"
    elif mutation == "excessive-interval-width":
        value.update(earliest=1791115200, latest=1791115210)
    elif mutation == "local-clock-masquerades-as-trusted":
        value["local_clock_only"] = True
    elif mutation == "fresh-nonce-stale-time":
        value.update(earliest=1791115200, latest=1791115201)
    elif mutation == "missing-time-evidence":
        value = {"available": False}
    elif mutation == "missing-request-binding":
        value["request_binding_sha256"] = "sha256:other-request"
    elif mutation == "future-dated-inside-uncertainty":
        value.update(earliest=1791115168, latest=1791115172)
    elif mutation == "reversed-time-interval":
        value.update(earliest=1791115203, latest=1791115202)
    else:
        raise KeyError(mutation)
    return value


def mutate_result(base: dict[str, Any], mutation: str) -> dict[str, Any]:
    value = copy.deepcopy(base)
    if mutation == "fresh-nonce-stale-time":
        value.update(issued_at=1791115100, expires_at=1791115150)
    return value


def main() -> int:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    policy = contract["policy"]
    fixture = contract["fixture"]
    failures: list[str] = []

    for vector in contract["vectors"]:
        now = mutate_time(fixture["trusted_now"], vector["mutation"])
        request = copy.deepcopy(fixture["request"])
        result = mutate_result(fixture["result"], vector["mutation"])

        if vector["mutation"] == "future-dated-inside-uncertainty":
            result.update(issued_at=1791115201, expires_at=1791115261)
        elif vector["mutation"] == "fresh-nonce-stale-time":
            request["nonce"] = "nonce-NEW"

        admission, admission_reason = validate_time_evidence(now, policy, request)
        if admission != "PASS":
            got, reason = admission, admission_reason
        else:
            if vector["mutation"] == "missing-time-evidence":
                got, reason = "INDETERMINATE", "time-evidence-missing"
            else:
                got, reason = freshness(now, result, policy)

        ok = got == vector["expected"]
        marker = "PASS" if ok else "FAIL"
        line = f'[{marker}] {vector["id"]}: expected={vector["expected"]} got={got} reason={reason}'
        print(line)
        if not ok:
            failures.append(line)

    print()
    print(f"RATS trusted-time qualification: {len(contract["vectors"]) - len(failures)}/{len(contract["vectors"])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
