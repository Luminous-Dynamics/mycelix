#!/usr/bin/env python3
"""Validate AGENT-000 Mycelix Agent Authority Constitution v0.1."""

from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = (
    ROOT
    / "mycelix-workspace"
    / "docs"
    / "agents"
    / "AGENT_AUTHORITY_CONSTITUTION_V0.1.md"
)

IDS = [f"AA-{n:03d}" for n in range(1, 25)]

REQUIRED = [
    "PrincipalIdentity != AgentIdentity",
    "StableAgentPrincipal != RuntimeInstance",
    "RuntimeAttestation != Authority",
    "Mission != Intent",
    "IntentProposal != QualifiedIntent",
    "QualifiedIntent != AuthorityGrant",
    "Authentication != Authorization",
    "Competence != Authority",
    "Reputation != Authority",
    "Intelligence != Authority",
    "Delegation != AuthorityMint",
    "QualifiedExactAction != ExternalEffect",
    "ReturnedError != DefinitelyNotCommitted",
    "Receipt != ChainOfThought",
    "AA-011 — consumable authority is conserved across fan-out",
    "retained_budget(parent)",
    "sum(active_child_allocations)",
    "<= qualified_parent_budget",
    "AA-015 — deterministic enforcement decides constraint fit",
    "A model may not self-certify that its own proposed action is authorized.",
    "AA-016 — long-lived secrets remain outside model context",
    "AA-017 — adapters cannot mint or widen authority",
    "AA-019 — effect uncertainty is typed and fail-closed",
    "IndeterminateCommit",
    "AA-020 — receipts preserve accountability without requiring hidden reasoning",
    "AA-021 — disclosure is minimum-sufficient and purpose-bound",
    "AA-023 — consequential action evidence is independently reconstructible",
    "AA-024 — model correctness is never a security assumption",
    "Security-critical boundaries must not depend on a model voluntarily obeying prose",
    "Protocol adapters should follow the core theorem rather than define it.",
]

FORBIDDEN_SHORTCUT_MARKERS = [
    "model_says_authorized = true",
    "trusted_agent = true",
    "latest wins",
    "duplicate consumable authority created by delegation fan-out",
    "automatic retry after an ambiguous external effect",
    "chain-of-thought logging as an accountability requirement",
]

OVERCLAIM_TERMS = [
    "AGENT-000 proves agent security",
    "AGENT-000 proves model safety",
    "AGENT-000 authorizes external effects",
    "Mycelix makes AI trustworthy",
    "all agents are trustworthy",
]


def fail(message: str) -> None:
    print(f"AGENT-000 qualification failed: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    if not DOC.is_file():
        fail(f"missing normative corpus: {DOC.relative_to(ROOT)}")

    text = DOC.read_text(encoding="utf-8")

    missing = [item for item in REQUIRED if item not in text]
    if missing:
        fail("missing required constitutional statements: " + ", ".join(missing))

    headings = re.findall(r"^### (AA-\d{3}) — ", text, flags=re.MULTILINE)
    if headings != IDS:
        fail(
            "invariant IDs must appear exactly once in canonical order "
            f"{IDS[0]}..{IDS[-1]}; observed={headings}"
        )

    for marker in FORBIDDEN_SHORTCUT_MARKERS:
        if text.count(marker) != 1:
            fail(f"forbidden-shortcut marker must appear exactly once: {marker!r}")

    for term in OVERCLAIM_TERMS:
        if term in text:
            fail(f"forbidden overclaim present: {term!r}")

    required_planes = [
        "### 1. Principal / legitimacy plane",
        "### 2. Agent identity plane",
        "### 3. Runtime / provenance plane",
        "### 4. Intent plane",
        "### 5. Authority plane",
        "### 6. Action plane",
        "### 7. Effect plane",
        "### 8. Evidence / accountability plane",
    ]
    positions = [text.find(plane) for plane in required_planes]
    if any(pos < 0 for pos in positions):
        fail("all eight planes must be present")
    if positions != sorted(positions):
        fail("eight planes must remain in canonical order")

    if "AGENT-001  explicit adversary/threat census" not in text:
        fail("AGENT-001 must remain the immediate adversary-accountability child")

    print("AGENT-000 agent authority constitution: PASS")
    print(f"  invariants: {len(IDS)}")
    print("  planes: 8")
    print("  runtime behavior changed: no")


if __name__ == "__main__":
    main()
