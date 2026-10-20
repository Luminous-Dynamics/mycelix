#!/usr/bin/env python3
"""Verify complete D6U runtime case coverage and observed substrate witnesses."""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
MANIFEST = ROOT / "docs/integral/d6u-runtime-manifest.json"
D6S2_FIXTURE = ROOT / "docs/integral/d6s-canon-2-authority-boundary-fixture.json"


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_d6u_runtime_evidence.py TEST_LOG")

    log_path = Path(sys.argv[1])
    log = log_path.read_text(encoding="utf-8")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    fixture = json.loads(D6S2_FIXTURE.read_text(encoding="utf-8"))
    fixture_cases = {case["case_id"]: case for case in fixture["boundary"]}
    expected = set(manifest["supported_reference_cases"])
    expected_zome_reached = {case_id: fixture_cases[case_id]["zome_reached"] for case_id in expected}
    expected_outcomes = manifest["case_outcomes"]
    assert set(expected_outcomes) == expected
    assert set(expected_outcomes.values()) <= set(manifest["evidence_outcome_classes"])

    observed = {}
    for line in log.splitlines():
        if not line.startswith("D6U_CASE\t"):
            continue
        parts = line.split("\t")
        assert len(parts) == 5 and parts[4] == "PASS", f"malformed case line: {line!r}"
        case_id, outcome, reachability = parts[1], parts[2], parts[3]
        assert case_id not in observed, f"duplicate D6U_CASE observation: {case_id}"
        assert reachability in {"zome-reached=true", "zome-reached=false"}, (
            f"malformed zome reachability for {case_id!r}: {reachability!r}"
        )
        observed[case_id] = {
            "outcome": outcome,
            "zome_reached": reachability == "zome-reached=true",
        }

    assert set(observed) == expected, f"coverage mismatch: {set(observed) ^ expected}"
    observed_outcomes = {case_id: data["outcome"] for case_id, data in observed.items()}
    observed_reachability = {case_id: data["zome_reached"] for case_id, data in observed.items()}
    assert observed_outcomes == expected_outcomes, f"outcome mismatch: {observed_outcomes}"
    assert observed_reachability == expected_zome_reached, (
        f"zome reachability mismatch: expected={expected_zome_reached}, "
        f"observed={observed_reachability}"
    )

    expected_application = set(manifest.get("supplemental_application_checks", []))
    application_expected_fragments = {
        "probe-local-d6s-commitment-mutation": "result=d6s-commitment-mismatch;zome-reached=true",
    }
    assert expected_application == set(application_expected_fragments)

    expected_supplemental = set(manifest.get("supplemental_substrate_checks", []))
    supplemental_expected_fragments = {
        "future-expiry-rejection": "Future",
        "wrong-zome-routing": "Zome not found: Zome 'wrong-zome' not found",
        "wrong-function-routing": (
            "Attempted to call a zome function that doesn't exist: "
            "Zome: coordinator Fn no_such_function"
        ),
        "wrong-cell-routing": "CellMissing(",
    }
    assert expected_supplemental == set(supplemental_expected_fragments)

    witness_expected_fragments = supplemental_expected_fragments
    witness_observed = {}
    for line in log.splitlines():
        if not line.startswith("D6U_RUNTIME_WITNESS\t"):
            continue
        parts = line.split("\t", 2)
        assert len(parts) == 3, f"malformed runtime witness line: {line!r}"
        witness_id, witness = parts[1], parts[2]
        assert "\t" not in witness, f"runtime witness must be a single field: {line!r}"
        assert witness_id not in witness_observed, (
            f"duplicate D6U_RUNTIME_WITNESS observation: {witness_id}"
        )
        assert witness_id in witness_expected_fragments, (
            f"unexpected D6U_RUNTIME_WITNESS observation: {witness_id}"
        )
        expected_fragment = witness_expected_fragments[witness_id]
        assert witness.strip(), f"runtime witness for {witness_id!r} must be non-empty"
        if expected_fragment:
            assert expected_fragment in witness, (
                f"runtime witness for {witness_id!r} does not contain the expected "
                f"substrate fragment {expected_fragment!r}: {witness!r}"
            )
        witness_observed[witness_id] = witness

    assert set(witness_observed) == expected_supplemental, (
        f"runtime witness coverage mismatch: "
        f"{set(witness_observed) ^ expected_supplemental}"
    )

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
        assert check_id in supplemental_expected_fragments, (
            f"unexpected D6U_SUBSTRATE_CHECK observation: {check_id}"
        )
        assert "\t" not in reason, f"substrate witness must be a single field: {line!r}"
        expected_fragment = supplemental_expected_fragments[check_id]
        assert reason.strip(), f"substrate witness for {check_id!r} must be non-empty"
        if expected_fragment:
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
    assert supplemental_observed == witness_observed, (
        "substrate PASS records must exactly match observed runtime witnesses"
    )

    application_observed = {}
    for line in log.splitlines():
        if not line.startswith("D6U_APPLICATION_CHECK\t"):
            continue
        parts = line.split("\t")
        assert len(parts) == 4 and parts[3] == "PASS", (
            f"malformed application-check line: {line!r}"
        )
        check_id, evidence = parts[1], parts[2]
        assert check_id not in application_observed, (
            f"duplicate D6U_APPLICATION_CHECK observation: {check_id}"
        )
        assert check_id in application_expected_fragments, (
            f"unexpected D6U_APPLICATION_CHECK observation: {check_id}"
        )
        assert "\t" not in evidence
        expected_fragment = application_expected_fragments[check_id]
        assert expected_fragment in evidence, (
            f"application check {check_id!r} missing expected evidence "
            f"{expected_fragment!r}: {evidence!r}"
        )
        application_observed[check_id] = evidence

    assert set(application_observed) == expected_application, (
        f"application-check coverage mismatch: "
        f"{set(application_observed) ^ expected_application}"
    )
    print(
        f"verified D6U runtime case coverage and outcomes: "
        f"{len(observed)}/{len(expected)}; "
        f"supplemental={len(supplemental_observed)}; "
        f"application={len(application_observed)}"
    )


if __name__ == "__main__":
    main()
