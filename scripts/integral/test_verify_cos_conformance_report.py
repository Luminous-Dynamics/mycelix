#!/usr/bin/env python3
"""Adversarial tests for the independent COS conformance-report verifier."""
from __future__ import annotations

import copy
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VERIFIER = ROOT / "scripts/integral/verify_cos_conformance_report.py"


def run_verifier(path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(VERIFIER), str(path)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def require_rejected(path: Path, marker: str) -> None:
    result = run_verifier(path)
    if result.returncode == 0:
        raise AssertionError(f"mutated report unexpectedly passed: {marker}")
    if marker not in result.stderr:
        raise AssertionError(
            f"expected failure marker {marker!r}; stderr={result.stderr!r}"
        )


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_verify_cos_conformance_report.py REPORT.json", file=sys.stderr)
        return 2

    source = Path(sys.argv[1])
    if not source.is_absolute():
        source = ROOT / source
    report = json.loads(source.read_text(encoding="utf-8"))

    baseline = run_verifier(source)
    if baseline.returncode != 0:
        print(baseline.stdout, end="")
        print(baseline.stderr, end="", file=sys.stderr)
        raise AssertionError("canonical report must pass before mutation tests")

    with tempfile.TemporaryDirectory(prefix="cos-report-verifier-") as tmp:
        temp = Path(tmp)

        overclaim = copy.deepcopy(report)
        overclaim["records"][0]["claim_ceiling"] = "Production deployment independently verified."
        overclaim_path = temp / "overclaim.json"
        overclaim_path.write_text(json.dumps(overclaim), encoding="utf-8")
        require_rejected(overclaim_path, "REPORT_ROW_CLAIM_CEILING_MISMATCH COS-N-001")

        origin_laundering = copy.deepcopy(report)
        origin_laundering["records"][12]["provenance_origin"] = "local origin"
        origin_path = temp / "origin-laundering.json"
        origin_path.write_text(json.dumps(origin_laundering), encoding="utf-8")
        require_rejected(origin_path, "REPORT_FOREIGN_ORIGIN_BOUNDARY_MISSING COS-N-013")

        duplicate_key_text = source.read_text(encoding="utf-8")
        needle = '"test_id": "COS-N-001"'
        if needle not in duplicate_key_text:
            raise AssertionError("canonical report serialization changed unexpectedly")
        duplicate_path = temp / "duplicate-key.json"
        duplicate_path.write_text(
            duplicate_key_text.replace(needle, needle + ', "test_id": "COS-N-001"', 1),
            encoding="utf-8",
        )
        require_rejected(duplicate_path, "duplicate JSON key: test_id")

    print("PASS: independent report verifier rejects claim-ceiling inflation, foreign-origin laundering, and duplicate JSON keys.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
