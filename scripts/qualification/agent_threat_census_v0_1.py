#!/usr/bin/env python3
"""Validate AGENT-001 adversary/threat census v0.1."""

from __future__ import annotations

import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
CENSUS = (
    ROOT
    / "mycelix-workspace"
    / "docs"
    / "agents"
    / "agent-threat-census-v0.1.json"
)
DOC = (
    ROOT
    / "mycelix-workspace"
    / "docs"
    / "agents"
    / "AGENT_ADVERSARY_CENSUS_V0.1.md"
)

EXPECTED_IDS = [f"AT-{n:03d}" for n in range(1, 51)]
ALLOWED_COVERAGE = {
    "future_executable_attack",
    "protocol_dependent_blocker",
    "residual_research_gap",
}
ALLOWED_CLASSES = {
    "identity",
    "intent",
    "authority",
    "runtime",
    "action",
    "credentials",
    "protocol",
    "effect",
    "evidence",
    "privacy",
    "system",
    "safety",
}
AA_IDS = {f"AA-{n:03d}" for n in range(1, 25)}
FORBIDDEN_POSITIVE_STATUSES = {
    "mitigated",
    "resolved",
    "secure",
    "production_ready",
    "deployment_ready",
}


def fail(message: str) -> None:
    print(f"AGENT-001 qualification failed: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    if not CENSUS.is_file():
        fail(f"missing census: {CENSUS.relative_to(ROOT)}")
    if not DOC.is_file():
        fail(f"missing narrative: {DOC.relative_to(ROOT)}")

    try:
        data = json.loads(CENSUS.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        fail(f"invalid census JSON: {exc}")

    if data.get("schema_version") != "agent-threat-census-v0.1":
        fail("unexpected schema_version")
    if data.get("subject") != "AGENT-001":
        fail("subject must be AGENT-001")
    if data.get("parent_constitution") != "AGENT-000":
        fail("parent_constitution must be AGENT-000")
    if data.get("full_agent_security_claim_blocked") is not True:
        fail("full_agent_security_claim_blocked must remain true")

    if set(data.get("coverage_vocabulary", [])) != ALLOWED_COVERAGE:
        fail("coverage_vocabulary must equal the closed v0.1 set")

    threats = data.get("threats")
    if not isinstance(threats, list):
        fail("threats must be an array")

    observed_ids = [t.get("id") for t in threats if isinstance(t, dict)]
    if observed_ids != EXPECTED_IDS:
        fail(
            "threat IDs must appear exactly once in canonical order "
            f"{EXPECTED_IDS[0]}..{EXPECTED_IDS[-1]}; observed={observed_ids}"
        )

    coverage_counts = {kind: 0 for kind in ALLOWED_COVERAGE}

    for threat in threats:
        if set(threat) != {
            "id",
            "name",
            "class",
            "adversary",
            "description",
            "invariant_refs",
            "coverage",
            "next_tranche",
        }:
            fail(f"{threat.get('id')}: unexpected/missing fields")

        tid = threat["id"]
        for field in ("name", "adversary", "description", "next_tranche"):
            value = threat[field]
            if not isinstance(value, str) or not value.strip():
                fail(f"{tid}: {field} must be non-empty canonical text")
            if value != value.strip():
                fail(f"{tid}: {field} has outer whitespace")

        if threat["class"] not in ALLOWED_CLASSES:
            fail(f"{tid}: unknown class {threat['class']!r}")

        coverage = threat["coverage"]
        if coverage not in ALLOWED_COVERAGE:
            fail(f"{tid}: unknown coverage {coverage!r}")
        coverage_counts[coverage] += 1

        refs = threat["invariant_refs"]
        if not isinstance(refs, list) or not refs:
            fail(f"{tid}: invariant_refs must be a non-empty array")
        if len(refs) != len(set(refs)):
            fail(f"{tid}: duplicate invariant_refs")
        unknown = set(refs) - AA_IDS
        if unknown:
            fail(f"{tid}: unknown AGENT-000 invariant refs: {sorted(unknown)}")

        lowered = {
            str(value).lower()
            for value in threat.values()
            if isinstance(value, str)
        }
        if lowered & FORBIDDEN_POSITIVE_STATUSES:
            fail(f"{tid}: positive security status is forbidden in AGENT-001")

    if any(count == 0 for count in coverage_counts.values()):
        fail(f"every coverage bucket must be represented: {coverage_counts}")

    narrative = DOC.read_text(encoding="utf-8")
    required_narrative = [
        "AGENT-001 PASS != full agent security",
        "Authorized != Correct",
        "Authorized != Wise",
        "Authorized != SafeOutcome",
        "`future_executable_attack` is a requirement, not evidence",
        "Passing AGENT-001 proves threat accountability only.",
    ]
    missing = [item for item in required_narrative if item not in narrative]
    if missing:
        fail("narrative missing required claim boundaries: " + ", ".join(missing))

    table_ids = re.findall(r"^\| (AT-\d{3}) \|", narrative, flags=re.MULTILINE)
    if table_ids != EXPECTED_IDS:
        fail("narrative threat table must exactly mirror machine-readable IDs")

    print("AGENT-001 adversary census: PASS")
    print(f"  threats: {len(threats)}")
    for kind in sorted(coverage_counts):
        print(f"  {kind}: {coverage_counts[kind]}")
    print("  full agent security claim blocked: true")


if __name__ == "__main__":
    main()
