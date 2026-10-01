#!/usr/bin/env python3
"""Structural verifier for the D6S-CANON-2 authority-boundary reference fixture.

This is a reference-model gate only. It does not emulate or replace Holochain
authorization and must not be interpreted as runtime qualification.
"""

import json
from pathlib import Path

EXPECTED = {
    "canonical-payload-accepted",
    "payload-mutation",
    "wrong-cell",
    "wrong-zome",
    "wrong-function",
    "valid-capability",
    "wrong-capability",
    "revoked-capability",
    "provenance-mismatch",
    "blocked-provenance",
    "nonce-replay",
    "nonce-stale",
    "expired-invocation",
    "authorized-semantic-rejection",
}

PRE_ZOME_RESULTS = {
    "d6s-commitment-mismatch",
    "holochain-routing-or-binding-rejection",
    "holochain-binding-or-authorization-rejection",
    "holochain-authorization-rejection",
    "holochain-nonce-rejection",
    "holochain-expiry-rejection",
}

ROOT = Path(__file__).parents[2]
FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"


def main():
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    assert fixture["profile"] == "D6S-CANON-2"
    assert fixture["kind"] == "authority-boundary-reference-fixture"
    assert fixture["depends_on"]["canonicalization_profile"] == "D6S-CANON-1"
    assert fixture["depends_on"]["claim_ceiling"] == "ReferenceModelOnly"

    cases = fixture["boundary"]
    assert len(cases) == len(EXPECTED), (len(cases), len(EXPECTED))
    ids = {case["case_id"] for case in cases}
    assert ids == EXPECTED, sorted(ids ^ EXPECTED)

    for case in cases:
        result = case["boundary_result"]
        reached = case["zome_reached"]
        semantic = case["semantic_result"]

        if result in PRE_ZOME_RESULTS:
            assert reached is False, case["case_id"]
            assert semantic == "not-reached", case["case_id"]

        if reached:
            assert result == "authorized", case["case_id"]
            assert semantic in {
                "subject-to-zome-validation",
                "rejected-by-zome",
            }, case["case_id"]

    authorized = {case["case_id"] for case in cases if case["zome_reached"]}
    assert authorized == {
        "canonical-payload-accepted",
        "valid-capability",
        "authorized-semantic-rejection",
    }

    print(f"verified {len(cases)} D6S-CANON-2 authority-boundary cases")
    print("zome_reached_cases=" + ",".join(sorted(authorized)))
    print("claim_ceiling=ReferenceModelOnly")


if __name__ == "__main__":
    main()
