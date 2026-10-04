#!/usr/bin/env python3
"""Dependency-free deterministic comparative-security benchmark.

This is a reference-model harness. It computes attack reachability from
explicitly declared topology/authorization requirements. It does not model
any real government or commercial system unless the exact topology is supplied.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-comparative-benchmark-v0.1.json"


def reachable_resources(profile: dict[str, Any], event: str) -> list[str]:
    compromised = set(profile["compromise_effects"].get(event, []))
    if event in {"revoked-credential", "stale-device-posture", "stale-policy"}:
        return []

    reached: list[str] = []
    for resource, required in profile["protected_resource_requirements"].items():
        if set(required).issubset(compromised):
            reached.append(resource)

    # A synthetic perimeter profile may explicitly model network reachability
    # as a compromise primitive rather than a resource attribute.
    if "network_reachability" in compromised:
        reached = sorted(profile["protected_resource_requirements"])

    return sorted(reached)


def measure(profile: dict[str, Any], event: str) -> dict[str, Any]:
    reached = reachable_resources(profile, event)
    costs = profile["trust_boundary_cost"]
    return {
        "reachable_resources_after_compromise": len(reached),
        "reachable_resource_ids": reached,
        "max_boundary_depth": max((costs[r] for r in reached), default=0),
        "standing_privilege_count": profile["standing_privilege_count"],
        "revocation_latency_seconds": profile["revocation_latency_seconds"],
        "cross_domain_exposure_count": profile["cross_domain_exposure_count"],
    }


def assert_guardrails(profile_id: str, event: str, result: dict[str, Any]) -> None:
    # These are architecture-neutral sanity checks, not superiority claims.
    if result["reachable_resources_after_compromise"] < 0:
        raise AssertionError(f"{profile_id}/{event}: negative reachability")
    if result["max_boundary_depth"] < 0:
        raise AssertionError(f"{profile_id}/{event}: negative boundary count")
    if result["standing_privilege_count"] < 0:
        raise AssertionError(f"{profile_id}/{event}: negative privilege count")
    if result["revocation_latency_seconds"] < 0:
        raise AssertionError(f"{profile_id}/{event}: negative revocation latency")
    if result["cross_domain_exposure_count"] < 0:
        raise AssertionError(f"{profile_id}/{event}: negative cross-domain exposure")


def main() -> int:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    failures: list[str] = []

    print("Mycelix comparative-security reference benchmark v0.1")
    print("Claim ceiling: ReferenceModelOnly")
    print("No composite security score is computed.")
    print()

    for profile_id, profile in contract["architectures"].items():
        print(f"## {profile_id}")
        for scenario in contract["scenarios"]:
            event = scenario["event"]
            result = measure(profile, event)
            assert_guardrails(profile_id, event, result)

            expected_map = contract["expected_reference_results"].get(profile_id, {})
            if event in expected_map and result["reachable_resources_after_compromise"] != expected_map[event]:
                failures.append(
                    f"{profile_id}/{event}: expected reach="
                    f"{expected_map[event]} got={result['reachable_resources_after_compromise']}"
                )

            print(
                f"{scenario['id']} {event}: "
                f"reach={result['reachable_resources_after_compromise']} "
                f"max_boundary_depth={result['max_boundary_depth']} "
                f"standing_privilege={result['standing_privilege_count']} "
                f"revocation_s={result['revocation_latency_seconds']} "
                f"cross_domain_exposure={result['cross_domain_exposure_count']}"
            )
        print()

    print(f"Comparative benchmark qualification: {len(contract['scenarios']) * len(contract['architectures'])} scenario/profile executions")
    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1

    print("Reference-model checks: PASS")
    print("Qualification ceiling: ReferenceModelOnly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
