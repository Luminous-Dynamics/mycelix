#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/lex-net/LEX_NET_CONSTITUTION_V1.md"
MANIFEST = ROOT / "docs/lex-net/lex_net_000_manifest.json"

EXPECTED_IDS = [f"LN-{i:03d}" for i in range(1, 13)]
EXPECTED_THREATS = [
    "foreign-authentication-promoted-to-local-authority",
    "technical-conformance-promoted-to-legal-validity",
    "caller-selected-law-forum-treated-as-dispositive",
    "stale-or-superseded-external-profile-reused-as-current",
    "assurance-provider-self-recognition",
    "cross-border-identity-equivalence-by-string-match",
    "treaty-or-convention-status-inferred-from-software-configuration",
    "external-decision-evidence-treated-as-self-executing-enforcement",
    "ai-legal-interpretation-treated-as-binding-decision",
    "private-mycelix-governance-overrides-mandatory-law",
    "translation-drops-reservations-exclusions-uncertainty-or-nonclaims",
    "validator-majority-or-reputation-becomes-universal-recognition-authority",
]
EXPECTED_CONTINUATION = [
    "LEX-NET-001", "LEX-NET-002", "LEX-NET-003", "LEX-NET-004",
    "LEX-NET-005", "LEX-NET-006", "LEX-NET-007", "LEX-NET-008",
    "LEX-NET-Q1",
]
REQUIRED_DOC_PHRASES = [
    "shared protocol convention",
    "foreign evidence does not directly import foreign authority",
    "same textual identifier",
    "TCK PASS",
    "The network remains infrastructure for institutions. It is not the sovereign.",
]
FORBIDDEN_POSITIVE_CLAIMS = [
    "LEX-NET is international law",
    "LEX-NET is legally binding",
    "Mycelix determines applicable law",
    "Mycelix is the adjudicator",
    "SCITT receipt is authority",
]


def fail(msg: str) -> None:
    raise SystemExit(f"LEX-NET-000 validation failed: {msg}")


def main() -> None:
    if not DOC.is_file() or not MANIFEST.is_file():
        fail("required constitution/manifest file missing")

    doc = DOC.read_text(encoding="utf-8")
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    if data.get("profile") != "mycelix-lex-net-constitution-v1":
        fail("unexpected manifest profile")
    if data.get("status") != "normative-research-architecture":
        fail("unexpected status")
    if data.get("governing_theorem") != "shared protocol convention != law != treaty != jurisdiction != adjudicative authority":
        fail("governing theorem changed")

    invariants = data.get("invariants")
    if not isinstance(invariants, list):
        fail("invariants must be a list")
    ids = [item.get("id") for item in invariants]
    if ids != EXPECTED_IDS:
        fail(f"invariant order/census mismatch: {ids}")
    names = [item.get("name") for item in invariants]
    if len(names) != len(set(names)) or any(not isinstance(n, str) or not n for n in names):
        fail("invariant names must be unique non-empty strings")

    if data.get("threats") != EXPECTED_THREATS:
        fail("threat census/order changed")
    if data.get("continuation") != EXPECTED_CONTINUATION:
        fail("continuation order changed")

    refs = data.get("reference_families")
    if not isinstance(refs, list) or len(refs) < 8:
        fail("reference-family census incomplete")
    for ref in refs:
        if ref.get("mapping_status") != "reference-only":
            fail("LEX-NET-000 may only register external families as reference-only")
        if not ref.get("authority") or not ref.get("instrument"):
            fail("reference family missing authority/instrument")

    nonclaims = data.get("mandatory_nonclaims")
    if not isinstance(nonclaims, list) or len(nonclaims) != 10:
        fail("mandatory nonclaim census changed")
    for claim in nonclaims:
        if claim.lower() not in doc.lower():
            fail(f"mandatory nonclaim absent from constitution: {claim}")

    for phrase in REQUIRED_DOC_PHRASES:
        if phrase not in doc:
            fail(f"required normative phrase missing: {phrase}")

    for forbidden in FORBIDDEN_POSITIVE_CLAIMS:
        if forbidden.lower() in doc.lower():
            fail(f"forbidden overclaim present: {forbidden}")

    headings = [line.split(" — ", 1)[0].strip("# ") for line in doc.splitlines() if line.startswith("## LN-")]
    if headings != EXPECTED_IDS:
        fail(f"document invariant heading order/census mismatch: {headings}")

    print(json.dumps({
        "profile": data["profile"],
        "invariants": len(invariants),
        "threats": len(EXPECTED_THREATS),
        "references": len(refs),
        "continuation": len(EXPECTED_CONTINUATION),
        "status": "PASS",
        "nonclaims_preserved": True,
        "grants_legal_authority": False,
        "grants_external_effect_authority": False,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
