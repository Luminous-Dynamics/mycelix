#!/usr/bin/env python3
"""Verify complete D6U runtime case coverage from the captured test log."""

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
    print(f"verified D6U runtime case coverage and outcomes: {len(observed)}/{len(expected)}")


if __name__ == "__main__":
    main()
