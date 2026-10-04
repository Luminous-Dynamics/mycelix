#!/usr/bin/env python3
"""Verify complete D6U runtime case coverage and observed substrate witnesses."""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_d6u_runtime_evidence.py TEST_LOG")

    log_path = Path(sys.argv[1])
    log = log_path.read_text(encoding="utf-8")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    expected = set(manifest["supported_reference_cases"])
    expected_outcomes = manifest["case_outcomes"]
    assert set(expected_outcomes) == expected
    assert set(expected_outcomes.values()) <= set(manifest["evidence_outcome_classes"])

    observed = {}
    for line in log.splitlines():
        if not line.startswith("D6U_CASE\t"):
            continue
        parts = line.split("\t")
        assert len(parts) == 4 and parts[3] == "PASS", f"malformed case line: {line!r}"
        case_id, outcome = parts[1], parts[2]
        assert case_id not in observed, f"duplicate D6U_CASE observation: {case_id}"
        observed[case_id] = outcome

    assert set(observed) == expected, f"coverage mismatch: {set(observed) ^ expected}"
    assert observed == expected_outcomes, f"outcome mismatch: {observed}"

    expected_supplemental = set(manifest.get("supplemental_substrate_checks", []))
    supplemental_expected_fragments = {
        "future-expiry-rejection": "Future",
        "wrong-zome-routing": "Zome not found: Zome 'wrong-zome' not found",
        "wrong-function-routing": (
            "Attempted to call a zome function that doesn't exist: "
            "Zome: coordinator Fn no_such_function"
        ),
    }
    assert expected_supplemental == set(supplemental_expected_fragments)

    supplemental_observed = {}
    for line in log.splitlines():
        if not line.startswith("D6U_SUBSTRATE_CHECK\t"):
            continue
        parts = line.split("\t")
        assert len(parts) == 4 and parts[3] == "PASS", (
            f"malformed substrate-check line: {line!r}"
        )
        check_id, reason = parts[1], parts[2]
        assert check_id not in supplemental_observed, (
            f"duplicate D6U_SUBSTRATE_CHECK observation: {check_id}"
        )
        assert "\t" not in reason, f"substrate witness must be a single field: {line!r}"
        expected_fragment = supplemental_expected_fragments[check_id]
        assert expected_fragment in log, (
            f"raw runtime log is missing substrate witness for {check_id!r}: "
            f"{expected_fragment!r}"
        )
        assert expected_fragment in reason, (
            f"substrate witness for {check_id!r} does not contain the expected "
            f"runtime fragment {expected_fragment!r}: {reason!r}"
        )
        supplemental_observed[check_id] = reason

    assert set(supplemental_observed) == expected_supplemental, (
        f"supplemental coverage mismatch: "
        f"{set(supplemental_observed) ^ expected_supplemental}"
    )
    print(
        f"verified D6U runtime case coverage and outcomes: "
        f"{len(observed)}/{len(expected)}; "
        f"supplemental={len(supplemental_observed)}"
    )


if __name__ == "__main__":
    main()
