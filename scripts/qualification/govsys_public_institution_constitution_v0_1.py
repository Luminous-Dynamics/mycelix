#!/usr/bin/env python3
"""Validate GOVSYS-002 public institution composition constitution v0.1."""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "mycelix-workspace" / "docs" / "government" / "PUBLIC_INSTITUTION_CONSTITUTION_V0.1.md"

REQUIRED = [
    "Institution != Mycelix platform",
    "DHT presence != constitutional legitimacy",
    "AuthorityGrant != administrative decision",
    "identity != office",
    "registry entry != universal legal truth",
    "Application != entitlement",
    "AdministrativeFinality != JudicialFinality",
    "Decision != effect",
    "OutcomeUnknown != ProvenNotApplied",
    "PI-003 — authority and evidence cannot self-justify cyclically",
    "PI-004 — one consequential decision uses one coherent evidence cut",
    "PI-006 — conflict is first-class",
    "conflict != max(timestamp)",
    "PI-007 — review does not mutate the reviewed act",
    "PI-009 — public records and public disclosure are separate",
    "PI-012 — public money preserves distinct authority stages",
    "Budget != Appropriation",
    "PI-013 — procurement award does not create settlement authority",
    "Award != Contract",
    "PI-014 — oversight findings are not self-executing sanctions",
    "PI-015 — official statistics have a distinct epistemic identity",
    "PI-016 — AI is advisory unless separately authorized",
    "PI-017 — emergency powers are explicit, narrow, and expiring",
    "PI-018 — coercive physical acts are not ordinary software authority",
    "PI-020 — frontend presentation cannot strengthen institutional truth",
    "ADMIN-001",
    "REGISTRY-001",
    "RECORDS-001",
]

FORBIDDEN = [
    "mycelix is the sovereign",
    "dht is truth",
    "latest timestamp wins",
    "highest reputation wins",
    "ai may grant public authority",
    "emergency bypasses review",
    "award authorizes payment",
]


def fail(message: str) -> None:
    raise SystemExit(f"GOVSYS-002 FAIL: {message}")


def main() -> int:
    if not DOC.is_file():
        fail(f"missing constitution: {DOC}")

    text = DOC.read_text(encoding="utf-8")
    lower = text.lower()

    for needle in REQUIRED:
        if needle not in text:
            fail(f"missing required invariant: {needle!r}")

    for needle in FORBIDDEN:
        if needle in lower:
            fail(f"forbidden authority shortcut found: {needle!r}")

    numbered = [f"PI-{n:03d}" for n in range(1, 21)]
    for invariant in numbered:
        if text.count(invariant) != 1:
            fail(f"{invariant} must appear exactly once")

    ordering = [text.index(invariant) for invariant in numbered]
    if ordering != sorted(ordering):
        fail("PI-001..PI-020 must remain in canonical order")

    if text.count("!=") < 25:
        fail("constitution lost explicit non-equivalence boundaries")

    if "universal government superuser" not in lower:
        fail("deliberate non-feature boundary for superuser is missing")
    if "national master database" not in lower:
        fail("deliberate non-feature boundary for master database is missing")
    if "autonomous ai decision authority" not in lower:
        fail("deliberate non-feature boundary for autonomous AI authority is missing")

    print("GOVSYS-002 PASS: PI-001..PI-020 constitutional boundaries preserved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
