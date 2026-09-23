#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/sovereign-dpi/SOV_DPI_ADOPTION_CONSTITUTION_V1.md"
MANIFEST = ROOT / "docs/sovereign-dpi/sov_dpi_000_manifest.json"

EXPECTED_IDS = [f"SDPI-{i:03d}" for i in range(1, 13)]
EXPECTED_NEXT = [f"SOV-DPI-{i:03d}" for i in range(1, 7)]
REQUIRED_THREATS = {
    "foreign-control-plane-dependency",
    "domestic-executive-authority-widening",
    "agency-privilege-creep",
    "permanent-emergency-authority",
    "model-output-promoted-to-public-authority",
    "cross-domain-person-linked-surveillance-joins",
    "challenge-appeal-remedy-removal",
    "provider-lock-in",
    "low-resource-exclusion",
    "partition-authority-widening",
    "corrupted-registry-procurement-evidence-inputs",
    "donor-development-finance-capture",
    "capital-to-protocol-authority-conversion",
    "foreign-evidence-to-local-authority-conversion",
    "luminous-dynamics-disappearance-or-compromise",
}
REQUIRED_DEPENDENCIES = {"#870", "#28", "#31", "#489", "#783", "#883", "#798"}
FORBIDDEN_OVERCLAIMS = (
    "is legally compliant",
    "is human-rights certified",
    "is production ready",
    "is nationally approved",
    "authorizes external effects",
)


def fail(message: str) -> None:
    raise SystemExit(f"SOV-DPI-000 validation failed: {message}")


def main() -> int:
    if not DOC.is_file() or not MANIFEST.is_file():
        fail("required constitution or manifest is missing")

    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    doc = DOC.read_text(encoding="utf-8")

    if data.get("schema") != "mycelix.sov-dpi.adoption-constitution.v1":
        fail("unexpected manifest schema")
    if data.get("tranche") != "SOV-DPI-000" or data.get("issue") != 1025:
        fail("manifest tranche/issue binding drifted")
    if data.get("governing_theorem") != "national customization != constitutional degradation":
        fail("governing theorem drifted")

    invariants = data.get("invariants")
    if not isinstance(invariants, list):
        fail("invariants must be a list")
    ids = [item.get("id") for item in invariants if isinstance(item, dict)]
    if ids != EXPECTED_IDS:
        fail(f"invariant IDs/order must be exactly {EXPECTED_IDS!r}")
    if len({item.get("name") for item in invariants}) != len(EXPECTED_IDS):
        fail("invariant names must be unique")

    threats = data.get("threats")
    if not isinstance(threats, list) or set(threats) != REQUIRED_THREATS or len(threats) != len(REQUIRED_THREATS):
        fail("threat census is incomplete, duplicated, or widened without review")

    dependencies = data.get("dependencies")
    if not isinstance(dependencies, list):
        fail("dependencies must be a list")
    refs = {item.get("ref") for item in dependencies if isinstance(item, dict)}
    if refs != REQUIRED_DEPENDENCIES:
        fail("required predecessor/dependency references drifted")

    if data.get("next") != EXPECTED_NEXT:
        fail("SOV-DPI-001..006 continuation ordering drifted")

    for invariant_id in EXPECTED_IDS:
        if doc.count(invariant_id) != 1:
            fail(f"{invariant_id} must appear exactly once in normative document")

    required_doc_phrases = (
        "national customization != constitutional degradation",
        "local sovereignty + interoperable evidence + bounded authority + provider replaceability + common invariant floor",
        "This document does not establish legal validity",
        "authorizes no external effect",
    )
    for phrase in required_doc_phrases:
        if phrase not in doc:
            fail(f"required document phrase missing: {phrase}")

    lower_doc = doc.lower()
    for claim in FORBIDDEN_OVERCLAIMS:
        if claim in lower_doc:
            fail(f"forbidden overclaim present: {claim}")

    nonclaims = data.get("nonclaims")
    if not isinstance(nonclaims, list) or "external-effect-authority" not in nonclaims:
        fail("explicit external-effect-authority nonclaim missing")

    print("SOV-DPI-000 manifest/document validation: PASS")
    print(f"invariants={len(EXPECTED_IDS)} threats={len(REQUIRED_THREATS)} dependencies={len(REQUIRED_DEPENDENCIES)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
