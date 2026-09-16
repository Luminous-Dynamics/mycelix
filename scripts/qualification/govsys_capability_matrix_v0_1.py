#!/usr/bin/env python3
"""Validate GOVSYS-001 whole-of-government capability matrix v0.1."""

from __future__ import annotations

import pathlib
import re
import sys
import tomllib
from collections import Counter

ROOT = pathlib.Path(__file__).resolve().parents[2]
MATRIX = ROOT / "mycelix-workspace" / "docs" / "government" / "capability-matrix-v0.1.toml"
README = ROOT / "mycelix-workspace" / "docs" / "government" / "README.md"

EXPECTED_SCHEMA = "mycelix-government-capability-matrix-v0.1"
ALLOWED_STATUS = {"strong_foundation", "partial", "missing", "external_by_design"}
COFOG = {f"{n:02d}" for n in range(1, 11)}
ID_RE = re.compile(r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
FORBIDDEN_STATUS_WORDS = {"complete", "solved", "production_ready", "government_ready"}
REQUIRED_FIELDS = {
    "id",
    "name",
    "cofog",
    "govtech",
    "status",
    "domains",
    "evidence",
    "gap",
    "next",
    "boundary",
}


def fail(message: str) -> None:
    raise SystemExit(f"GOVSYS-001 FAIL: {message}")


def require_nonempty_string(row: dict, field: str, capability_id: str) -> None:
    value = row[field]
    if not isinstance(value, str) or not value.strip():
        fail(f"{capability_id}: {field} must be a non-empty string")


def require_string_list(row: dict, field: str, capability_id: str, *, allow_empty: bool = False) -> None:
    value = row[field]
    if not isinstance(value, list) or any(not isinstance(item, str) or not item.strip() for item in value):
        fail(f"{capability_id}: {field} must be a list of non-empty strings")
    if not allow_empty and not value:
        fail(f"{capability_id}: {field} must not be empty")


def main() -> int:
    if not MATRIX.is_file():
        fail(f"missing matrix: {MATRIX}")
    if not README.is_file():
        fail(f"missing README: {README}")

    data = tomllib.loads(MATRIX.read_text(encoding="utf-8"))

    if data.get("schema_version") != EXPECTED_SCHEMA:
        fail("schema_version drift")

    declared_status = data.get("status_values")
    if declared_status != ["strong_foundation", "partial", "missing", "external_by_design"]:
        fail("status_values must preserve the exact conservative v0.1 ordering")

    if set(data.get("cofog_divisions", [])) != COFOG:
        fail("cofog_divisions must contain exactly 01..10")

    rows = data.get("capability")
    if not isinstance(rows, list) or not rows:
        fail("matrix must contain capability rows")

    ids: set[str] = set()
    status_counts: Counter[str] = Counter()
    cofog_counts: Counter[str] = Counter()

    for index, row in enumerate(rows, start=1):
        if not isinstance(row, dict):
            fail(f"row {index} is not a table")

        missing_fields = REQUIRED_FIELDS - row.keys()
        extra_fields = row.keys() - REQUIRED_FIELDS
        if missing_fields:
            fail(f"row {index} missing fields: {sorted(missing_fields)}")
        if extra_fields:
            fail(f"row {index} has undeclared fields: {sorted(extra_fields)}")

        capability_id = row["id"]
        if not isinstance(capability_id, str) or not ID_RE.fullmatch(capability_id):
            fail(f"row {index}: invalid capability id {capability_id!r}")
        if capability_id in ids:
            fail(f"duplicate capability id: {capability_id}")
        ids.add(capability_id)

        for field in ("name", "gap", "next", "boundary"):
            require_nonempty_string(row, field, capability_id)

        require_string_list(row, "govtech", capability_id)
        require_string_list(row, "domains", capability_id)
        require_string_list(row, "evidence", capability_id, allow_empty=True)

        status = row["status"]
        if status not in ALLOWED_STATUS:
            fail(f"{capability_id}: invalid status {status!r}")
        if status in FORBIDDEN_STATUS_WORDS:
            fail(f"{capability_id}: overclaiming status is forbidden")

        cofog = row["cofog"]
        if cofog not in COFOG:
            fail(f"{capability_id}: invalid COFOG division {cofog!r}")

        if status in {"strong_foundation", "partial"} and not row["evidence"]:
            fail(f"{capability_id}: positive classification requires concrete evidence")

        if status == "missing":
            if row["evidence"]:
                fail(f"{capability_id}: missing rows must not cite positive implementation evidence")
            if row["next"].strip().lower() in {"none", "n/a", "na"}:
                fail(f"{capability_id}: missing row requires a concrete next tranche")

        if status == "external_by_design":
            if len(row["boundary"].strip()) < 24:
                fail(f"{capability_id}: external-by-design row requires an explicit boundary")

        # This census must never hide remaining work behind a positive label.
        if status == "strong_foundation" and len(row["gap"].strip()) < 24:
            fail(f"{capability_id}: strong foundation must still state its remaining gap")

        status_counts[status] += 1
        cofog_counts[cofog] += 1

    missing_cofog = sorted(COFOG - cofog_counts.keys())
    if missing_cofog:
        fail(f"matrix lacks COFOG coverage for divisions: {missing_cofog}")

    matrix_text = MATRIX.read_text(encoding="utf-8").lower()
    for forbidden in ('status = "complete"', 'status = "solved"', 'status = "production_ready"', 'status = "government_ready"'):
        if forbidden in matrix_text:
            fail(f"forbidden overclaiming classification found: {forbidden}")

    readme_text = README.read_text(encoding="utf-8")
    for needle in (
        "Mycelix may represent institutional authority",
        "There is intentionally no `complete`",
        "GOVSYS-002",
        "ADMIN-001",
        "REGISTRY-001",
        "PFM-001",
        "PROC-001",
        "STATS-001",
        "REGULATORY-001",
    ):
        if needle not in readme_text:
            fail(f"README contract missing: {needle!r}")

    print(f"GOVSYS-001 PASS: {len(rows)} capability rows")
    print("status counts:", dict(sorted(status_counts.items())))
    print("COFOG counts:", dict(sorted(cofog_counts.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
