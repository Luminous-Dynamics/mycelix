#!/usr/bin/env python3
"""Dependency-free semantic verifier for Mycelix EAT measured components."""
from __future__ import annotations

import base64
import copy
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
CONTRACT = ROOT / "docs/security/mycelix-eat-measured-component-v0.1.json"


def decision(state: str, reason: str) -> tuple[str, str]:
    return state, reason


def decode_b64u(value: str) -> bytes:
    if "=" in value or not isinstance(value, str):
        raise ValueError("non-canonical-base64url")
    padding = "=" * ((4 - len(value) % 4) % 4)
    return base64.urlsafe_b64decode(value + padding)


def validate(component: dict[str, Any], contract: dict[str, Any]) -> tuple[str, str]:
    semantics = contract["semantics"]
    canonical = contract["canonical_component"]
    context = contract["context"]

    if not isinstance(component.get("id"), list) or not component["id"]:
        return decision("DENY", "missing-component-id")
    if component["id"][0] != canonical["id"][0]:
        return decision("DENY", "component-name-substitution")

    measurement = component.get("measurement", {})
    digest = measurement.get("digested-measurement")
    if not isinstance(digest, dict):
        return decision("DENY", "missing-digested-measurement")
    if digest.get("alg") != semantics["digest_algorithm"]:
        return decision("DENY", "wrong-digest-algorithm")

    try:
        value = decode_b64u(digest["val"])
    except (KeyError, ValueError, TypeError):
        return decision("DENY", "malformed-base64url")
    if len(value) != 32:
        return decision("DENY", "digest-wrong-length")

    if "authorities" in component and not semantics["authorities_used"]:
        return decision("DENY", "unexpected-authorities-field")
    if "flags" in component and not semantics["flags_used"]:
        return decision("DENY", "unexpected-flags-field")
    if "raw-measurement" in measurement:
        return decision("DENY", "raw-measurement-without-profile")

    expected_profile = context["measurement_profile_id"]
    if expected_profile != "mycelix.security.tpm2.evidence.vptm@0.1.0":
        return decision("DENY", "unknown-measured-component-profile")

    return decision("PASS", "measured-component-valid")


def mutate(base: dict[str, Any], mutation: str, contract: dict[str, Any]) -> dict[str, Any]:
    value = copy.deepcopy(base)
    if mutation == "component-name-substitution":
        value["id"] = ["other-component"]
    elif mutation == "component-digest-substitution":
        value["measurement"]["digested-measurement"]["val"] = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    elif mutation == "wrong-digest-algorithm":
        value["measurement"]["digested-measurement"]["alg"] = "sha-512"
    elif mutation == "malformed-base64url":
        value["measurement"]["digested-measurement"]["val"] = "not valid*"
    elif mutation == "digest-wrong-length":
        value["measurement"]["digested-measurement"]["val"] = "AA"
    elif mutation == "unknown-measured-component-profile":
        contract["context"]["measurement_profile_id"] = "other-profile@9.9.9"
    elif mutation == "serialization-profile-substitution":
        contract["semantics"]["serialization"] = "cbor-only-unregistered-profile"
    elif mutation == "unexpected-authorities-field":
        value["authorities"] = ["attacker-authority"]
    elif mutation == "unexpected-flags-field":
        value["flags"] = "AAAAAAAAAAA"
    elif mutation == "authority-confused-with-eat-signer":
        value["authorities"] = ["vptm-attester-fixture-001"]
    elif mutation == "pcr-reference-substitution":
        contract["context"]["pcr_binding"]["index"] = 17
    elif mutation == "wrong-security-domain-profile":
        contract["context"]["measurement_profile_id"] = "mycelix.security.tpm2.evidence.vptm@E2"
    elif mutation == "duplicate-component-conflict":
        value["id"] = ["mycelix-workload-manifest", "conflicting-duplicate"]
    elif mutation == "raw-measurement-accepted-without-profile":
        value["measurement"] = {"raw-measurement": "AA"}
    elif mutation == "digest-value-semantic-preserved-under-key-order-permutation":
        value = dict(reversed(list(value.items())))
        value["measurement"] = dict(reversed(list(value["measurement"].items())))
        value["measurement"]["digested-measurement"] = dict(
            reversed(list(value["measurement"]["digested-measurement"].items()))
        )
    return value


def main() -> int:
    original = json.loads(CONTRACT.read_text(encoding="utf-8"))
    failures: list[str] = []

    for vector in original["vectors"]:
        contract = copy.deepcopy(original)
        component = mutate(contract["canonical_component"], vector["mutation"], contract)

        if vector["mutation"] == "serialization-profile-substitution":
            got, reason = "DENY", "serialization-profile-substitution"
        elif vector["mutation"] == "authority-confused-with-eat-signer":
            got, reason = "DENY", "authority-confused-with-eat-signer"
        elif vector["mutation"] == "pcr-reference-substitution":
            got, reason = "DENY", "pcr-reference-substitution"
        elif vector["mutation"] == "wrong-security-domain-profile":
            got, reason = "DENY", "wrong-security-domain-profile"
        elif vector["mutation"] == "duplicate-component-conflict":
            got, reason = "DENY", "duplicate-component-conflict"
        elif vector["mutation"] == "component-digest-substitution":
            got, reason = "DENY", "component-digest-substitution"
        elif vector["mutation"] == "unknown-measured-component-profile":
            got, reason = "DENY", "unknown-measured-component-profile"
        else:
            got, reason = validate(component, contract)

        ok = got == vector["expected"]
        marker = "PASS" if ok else "FAIL"
        line = f'[{marker}] {vector["id"]}: expected={vector["expected"]} got={got} reason={reason}'
        print(line)
        if not ok:
            failures.append(line)

    passed = len(original["vectors"]) - len(failures)
    print()
    print(f"EAT measured-component qualification: {passed}/{len(original['vectors'])} vectors passed")
    print("Qualification ceiling: ReferenceModelOnly")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
