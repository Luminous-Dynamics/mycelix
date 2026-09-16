#!/usr/bin/env python3
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/lex-net/LEX_NET_SEMANTIC_TRANSLATION_V1.md"
MANIFEST = ROOT / "docs/lex-net/lex_net_017_manifest.json"

EXPECTED_PARENT = "4dadc07eecf8eab67d50d8526dd7e6d3015de9a7"
EXPECTED_IDS = [f"ST-{i:03d}" for i in range(1, 13)]
EXPECTED_RELATIONS = [
    "Lossless", "RepresentationOnly", "Narrowing", "Widening",
    "Aggregation", "Decomposition", "Derived", "Dropped", "Partial",
    "Ambiguous", "ConflictPreserving", "Unsupported",
]
EXPECTED_DISPOSITIONS = ["Translated", "Refused", "Indeterminate"]
EXPECTED_FIXTURE_IDS = [
    "lossless-identity",
    "representation-only-exact-unit-normalization",
    "narrowing-explicitly-permitted",
    "widening-not-permitted",
    "aggregation-explicitly-permitted",
    "decomposition-explicitly-permitted",
    "derived-with-qualified-rule",
    "dropped-context-refused",
    "partial-explicitly-permitted",
    "ambiguous-mapping",
    "conflict-preserved",
    "unsupported-profile-version",
    "unknown-collapsed-to-false",
    "timezone-dropped",
    "identifier-scheme-dropped",
    "reservation-dropped",
    "ai-proposal-unqualified",
    "roundtrip-equal-state-collapse",
]
FACT_TO_RELATION = {
    "identity": "Lossless",
    "representation_only": "RepresentationOnly",
    "narrowing": "Narrowing",
    "widening": "Widening",
    "aggregation": "Aggregation",
    "decomposition": "Decomposition",
    "derived": "Derived",
    "dropped": "Dropped",
    "partial": "Partial",
    "ambiguous": "Ambiguous",
    "conflict_preserving": "ConflictPreserving",
    "unsupported": "Unsupported",
}
REQUIRED_DOC_PHRASES = [
    "round-trip equality is representation evidence only",
    "unknown is not false",
    "translation grants no stronger institutional status",
    "A TranslationReceipt is a typed evidentiary stage.",
    "Each arrow requires its own qualified transition.",
]


def fail(msg: str) -> None:
    raise SystemExit(f"LEX-NET-017 validation failed: {msg}")


def normalize_prose(text: str) -> str:
    text = text.replace("`", "")
    return re.sub(r"\s+", " ", text.casefold()).strip()


def receipt(relation: str, disposition: str, reason: str) -> dict:
    return {
        "relation": relation,
        "disposition": disposition,
        "reason": reason,
        "grants_factual_truth": False,
        "grants_legal_equivalence": False,
        "grants_recognition": False,
        "grants_authority_equivalence": False,
        "grants_external_effect_authority": False,
    }


def evaluate(fixture: dict) -> dict:
    relation = FACT_TO_RELATION.get(fixture.get("fact"))
    if relation is None:
        return receipt("Unsupported", "Refused", "unknown-fact-class")

    if relation == "Unsupported":
        return receipt(relation, "Refused", "profile-or-mapping-unsupported")

    if fixture.get("mapping_qualified") is False:
        return receipt(relation, "Indeterminate", "mapping-qualification-unestablished")

    if relation == "Derived" and fixture.get("derivation_bound") is not True:
        return receipt(relation, "Indeterminate", "derivation-provenance-unestablished")

    if fixture.get("hard_violation"):
        return receipt(relation, "Refused", "hard-semantic-invariant-violation")

    if relation == "Ambiguous":
        return receipt(relation, "Indeterminate", "semantic-relation-ambiguous")

    if relation not in fixture.get("allowed_relations", []):
        return receipt(relation, "Refused", "relation-not-permitted-for-target-use")

    return receipt(relation, "Translated", "relation-permitted")


def main() -> None:
    if not DOC.is_file() or not MANIFEST.is_file():
        fail("required profile/manifest file missing")

    doc = DOC.read_text(encoding="utf-8")
    doc_norm = normalize_prose(doc)
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    if data.get("profile") != "mycelix-lex-net-semantic-translation-v1":
        fail("unexpected profile id")
    if data.get("qualified_parent_head") != EXPECTED_PARENT:
        fail("qualified parent head changed")
    if data.get("governing_theorem") != (
        "syntactic transform != semantic equivalence != factual truth != legal equivalence != recognition != authority equivalence"
    ):
        fail("governing theorem changed")
    if data.get("relations") != EXPECTED_RELATIONS:
        fail("semantic relation census/order changed")
    if data.get("dispositions") != EXPECTED_DISPOSITIONS:
        fail("disposition census/order changed")

    invariants = data.get("invariants")
    if not isinstance(invariants, list):
        fail("invariants must be a list")
    ids = [item.get("id") for item in invariants]
    if ids != EXPECTED_IDS:
        fail(f"invariant order/census mismatch: {ids}")
    names = [item.get("name") for item in invariants]
    if len(names) != len(set(names)) or any(not isinstance(n, str) or not n for n in names):
        fail("invariant names must be unique non-empty strings")

    mapping = data.get("reference_mapping")
    if mapping != {
        "mapping_id": "synthetic-crosswalk",
        "mapping_version": "1.0.0",
        "mapping_commitment": "sha256:synthetic-lex-net-017-v1",
    }:
        fail("reference mapping identity changed")

    fixtures = data.get("fixtures")
    if not isinstance(fixtures, list):
        fail("fixtures must be a list")
    fixture_ids = [fixture.get("id") for fixture in fixtures]
    if fixture_ids != EXPECTED_FIXTURE_IDS:
        fail(f"fixture census/order mismatch: {fixture_ids}")

    observed_relations = set()
    observed_dispositions = set()
    for fixture in fixtures:
        expected = fixture.get("expected")
        if not isinstance(expected, dict):
            fail(f"fixture {fixture.get('id')} missing expected result")
        actual = evaluate(fixture)
        observed_relations.add(actual["relation"])
        observed_dispositions.add(actual["disposition"])
        comparable = {
            "relation": actual["relation"],
            "disposition": actual["disposition"],
            "reason": actual["reason"],
        }
        if comparable != expected:
            fail(
                f"fixture {fixture.get('id')} mismatch: "
                f"expected={expected!r} actual={comparable!r}"
            )
        if any([
            actual["grants_factual_truth"],
            actual["grants_legal_equivalence"],
            actual["grants_recognition"],
            actual["grants_authority_equivalence"],
            actual["grants_external_effect_authority"],
        ]):
            fail(f"fixture {fixture.get('id')} illegally grants stronger status")

        if (
            fixture.get("round_trip_equal")
            and actual["relation"] == "Ambiguous"
            and actual["disposition"] == "Translated"
        ):
            fail(f"fixture {fixture.get('id')} launders round-trip equality into translation")

    if observed_relations != set(EXPECTED_RELATIONS):
        fail(f"fixture corpus does not exercise all relations: {sorted(observed_relations)}")
    if observed_dispositions != set(EXPECTED_DISPOSITIONS):
        fail(f"fixture corpus does not exercise all dispositions: {sorted(observed_dispositions)}")

    nonclaims = data.get("mandatory_nonclaims")
    if not isinstance(nonclaims, list) or len(nonclaims) != 10:
        fail("mandatory nonclaim census changed")
    for claim in nonclaims:
        if normalize_prose(claim) not in doc_norm:
            fail(f"mandatory nonclaim absent from profile: {claim}")

    for phrase in REQUIRED_DOC_PHRASES:
        if normalize_prose(phrase) not in doc_norm:
            fail(f"required profile phrase missing: {phrase}")

    headings = [
        line.split(" — ", 1)[0].strip("# ")
        for line in doc.splitlines()
        if line.startswith("## ST-")
    ]
    if headings != EXPECTED_IDS:
        fail(f"profile invariant heading order/census mismatch: {headings}")

    forbidden = [
        "schema-valid target output establishes semantic equivalence",
        "round-trip equality proves semantic equivalence",
        "translation grants local recognition",
        "translation grants local authority",
        "translation establishes legal equivalence",
    ]
    for phrase in forbidden:
        if normalize_prose(phrase) in doc_norm:
            fail(f"forbidden positive claim present: {phrase}")

    print(json.dumps({
        "profile": data["profile"],
        "qualified_parent_head": data["qualified_parent_head"],
        "invariants": len(invariants),
        "fixtures": len(fixtures),
        "relations_exercised": sorted(observed_relations),
        "dispositions_exercised": sorted(observed_dispositions),
        "status": "PASS",
        "grants_factual_truth": False,
        "grants_legal_equivalence": False,
        "grants_recognition": False,
        "grants_authority_equivalence": False,
        "grants_external_effect_authority": False,
        "network_access_required": False,
        "ambient_clock_required": False,
        "ai_inference_required": False,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
