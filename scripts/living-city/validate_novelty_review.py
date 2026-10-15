#!/usr/bin/env python3
"""Validate Living-City novelty/legibility reviews and non-compensatory routing.

CI validation glue only. No individual identities are stored and no network access occurs.
"""
from __future__ import annotations
import copy
import json
import statistics
import sys
from pathlib import Path
from jsonschema import Draft202012Validator, FormatChecker

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "schemas/living-city-novelty-review-v0.1.schema.json"
PLAN_PATH = ROOT / "fixtures/living-city/evaluations/johannesburg-civic-corridor-novelty-review-plan.v0.1.json"
CONCEPT_MANIFEST_PATH = ROOT / "fixtures/living-city/manifest.v0.1.json"
SOURCE_PATH = ROOT / "fixtures/living-city/evidence/sources.v0.1.json"
EVIDENCE_CLAIMS_DIR = ROOT / "fixtures/living-city/evidence/claims"
DIMENSIONS = ("novelty", "surprise", "legibility", "usefulness", "context_fit", "feasibility", "delight")
REQUIRED_PERSPECTIVES = {"affected_community", "accessibility_and_disability", "delivery_and_maintenance"}
# Provisional governance tripwire, not an empirically calibrated effect-size threshold.
DISAGREEMENT_DIMENSIONS = ("legibility", "usefulness", "context_fit", "feasibility")
DISAGREEMENT_RANGE = 3
GATE_KEYS = ("life_safety", "accessibility", "rights_and_consent", "environmental_and_land_feasibility", "operating_owner_and_funding", "anti_displacement", "privacy_and_data_minimization")


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"{path.relative_to(ROOT)} is not readable valid JSON: {exc}")


def describe(errors) -> str:
    return "; ".join(f"{list(e.path)} [{e.validator}]: {e.message}" for e in sorted(errors, key=lambda e: (list(map(str, e.path)), e.message))) or "no validation errors"


def load_catalogs():
    manifest = load_json(CONCEPT_MANIFEST_PATH)
    concepts = {}
    for item in manifest.get("fixtures", []):
        if item.get("expect_valid") is True:
            concept = load_json(ROOT / item["path"])
            key = (concept["concept_id"], concept["version"])
            if key in concepts:
                fail(f"duplicate concept reference in manifest: {key!r}")
            concepts[key] = concept
    source_ids = {item["source_id"] for item in load_json(SOURCE_PATH)}
    evidence_ids = set()
    for path in sorted(EVIDENCE_CLAIMS_DIR.glob("*.json")):
        evidence = load_json(path)
        if evidence["evidence_id"] in evidence_ids:
            fail(f"duplicate evidence ID: {evidence['evidence_id']!r}")
        evidence_ids.add(evidence["evidence_id"])
    return concepts, source_ids, evidence_ids


def median_score(review: dict, dimension: str):
    values = [rating["scores"][dimension]["score"] for rating in review["ratings"]]
    return statistics.median(values) if values else None


def perspective_score_range(review: dict, dimension: str) -> int:
    """Range across perspective-level aggregates; never combine or erase group profiles."""
    values = [rating["scores"][dimension]["score"] for rating in review["ratings"]]
    return max(values) - min(values) if len(values) > 1 else 0


def recommend(review: dict, evidence_ids: set[str]) -> str:
    if review["status"] == "protocol_only":
        return "not_evaluated"
    gates = [review["hard_gates"][key]["status"] for key in GATE_KEYS]
    if "fail" in gates:
        return "stop_or_reframe"
    if review["status"] == "contested":
        return "hold_for_evidence"
    # A favorable median must not conceal major divergence on decision-critical dimensions.
    if any(perspective_score_range(review, dimension) >= DISAGREEMENT_RANGE for dimension in DISAGREEMENT_DIMENSIONS):
        return "hold_for_evidence"
    if any(status not in {"pass", "not_applicable"} for status in gates):
        return "hold_for_evidence"
    if review["stage"] in {"exploration", "comparison", "prototype"} and median_score(review, "surprise") >= 4 and median_score(review, "legibility") <= 2:
        return "prototype_for_comprehension_testing"
    if review["comparison_set"]["status"] != "ready" or len(review["comparison_set"]["reference_ids"]) < 3:
        return "hold_for_evidence"
    if not review["evidence_refs"] or any(ref not in evidence_ids for ref in review["evidence_refs"]):
        return "hold_for_evidence"
    perspectives = {rating["perspective"] for rating in review["ratings"]}
    if not REQUIRED_PERSPECTIVES.issubset(perspectives):
        return "hold_for_evidence"
    if set(review["affected_groups"]).intersection({"transit_operators", "informal_traders"}) and "operator_and_local_livelihoods" not in perspectives:
        return "hold_for_evidence"
    # Dimensions are assessed separately; novelty is not a gate and cannot compensate for weak usefulness.
    if any(median_score(review, d) < 3 for d in ("legibility", "usefulness", "context_fit", "feasibility")):
        return "hold_for_evidence"
    return "eligible_for_human_pilot_review"


def semantic_errors(review: dict, concepts: dict, source_ids: set[str], evidence_ids: set[str]) -> list[str]:
    errors = []
    cref = review["concept_ref"]
    key = (cref["concept_id"], cref["concept_version"])
    concept = concepts.get(key)
    if concept is None:
        errors.append(f"unresolved concept/version reference {key!r}")
    elif cref["hypothesis_id"] not in {h["hypothesis_id"] for h in concept["design_hypotheses"]}:
        errors.append(f"unresolved hypothesis reference {cref['hypothesis_id']!r} for {key!r}")
    perspectives = [rating["perspective"] for rating in review["ratings"]]
    if len(perspectives) != len(set(perspectives)):
        errors.append("a perspective may appear only once; aggregate by group without storing individual identities")
    for ref in review["comparison_set"]["source_refs"]:
        if ref not in source_ids:
            errors.append(f"unresolved comparison source reference {ref!r}")
    for ref in review["evidence_refs"]:
        if ref not in evidence_ids:
            errors.append(f"unresolved evidence reference {ref!r}")
    for reference in review["comparison_set"]["reference_ids"]:
        if reference.startswith("evidence:") and reference.removeprefix("evidence:") not in evidence_ids:
            errors.append(f"unresolved evidence reference in comparison set {reference!r}")
        if reference.startswith("source:") and reference.removeprefix("source:") not in source_ids:
            errors.append(f"unresolved source reference in comparison set {reference!r}")
    if review["status"] == "contested" and not review["dissent_summary"]:
        errors.append("contested review must preserve dissent_summary")
    large_disagreements = [dimension for dimension in DIMENSIONS if perspective_score_range(review, dimension) >= DISAGREEMENT_RANGE]
    if large_disagreements and not review["dissent_summary"]:
        errors.append("large cross-perspective rating differences must be preserved in dissent_summary: " + ", ".join(large_disagreements))
    expected = recommend(review, evidence_ids)
    if review["recommendation"] != expected:
        errors.append(f"declared recommendation {review['recommendation']!r} does not match policy recommendation {expected!r}")
    return errors


def make_synthetic_review(plan: dict, evidence_id: str) -> dict:
    """Ephemeral test object; never persisted as real community evidence."""
    value = copy.deepcopy(plan)
    value["stage"] = "comparison"
    value["status"] = "panel_evaluated"
    value["comparison_set"] = {
        "status":"ready",
        "description":"Synthetic test-only comparison set; not a real precedent assessment.",
        "method":"Synthetic controlled data used to test routing invariants only.",
        "reference_ids":["source:joburg-metroview-announcement","source:nccis-climate-services-catalogue","source:sarva-climate-risk-tool-overview"],
        "source_refs":["joburg-metroview-announcement","nccis-climate-services-catalogue","sarva-climate-risk-tool-overview"],
        "limitations":["Synthetic validator input; never treat as an actual novelty comparison."]
    }
    value["evidence_refs"] = [evidence_id]
    value["ratings"] = []
    for perspective in ("affected_community","accessibility_and_disability","delivery_and_maintenance","design_and_technical","operator_and_local_livelihoods"):
        scores = {}
        for dimension in DIMENSIONS:
            scores[dimension] = {"score":4 if dimension != "surprise" else 3,"rationale":"Synthetic score used solely to exercise a CI routing invariant."}
        value["ratings"].append({
            "perspective":perspective,"sample_size_band":"3-5","scores":scores,
            "notes":"Synthetic group aggregate; no real respondent or community assessment.","dissent_notes":[]
        })
    value["hard_gates"] = {key:{"status":"pass","rationale":"Synthetic pass solely for deterministic validator testing; not an actual clearance."} for key in GATE_KEYS}
    value["recommendation"] = "eligible_for_human_pilot_review"
    value["dissent_summary"] = []
    value["limitations"] = ["Synthetic test object generated in memory for CI only.","Not a real review, community endorsement, safety assessment, or authorization to build."]
    return value


def main() -> None:
    schema = load_json(SCHEMA_PATH)
    try:
        Draft202012Validator.check_schema(schema)
    except Exception as exc:
        fail(f"novelty review schema invalid under Draft 2020-12: {exc}")
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    plan = load_json(PLAN_PATH)
    errors = list(validator.iter_errors(plan))
    if errors:
        fail(f"protocol-only review plan invalid: {describe(errors)}")
    concepts, source_ids, evidence_ids = load_catalogs()
    semantic = semantic_errors(plan, concepts, source_ids, evidence_ids)
    if semantic:
        fail("protocol-only plan semantic validation failed: " + "; ".join(semantic))
    print("PASS: protocol-only plan remains not_evaluated; no scores or community endorsement implied.")

    synthetic = make_synthetic_review(plan, next(iter(sorted(evidence_ids))))
    errors = list(validator.iter_errors(synthetic))
    if errors:
        fail(f"synthetic routing fixture invalid: {describe(errors)}")
    semantic = semantic_errors(synthetic, concepts, source_ids, evidence_ids)
    if semantic:
        fail("synthetic routing fixture failed: " + "; ".join(semantic))
    print("PASS: synthetic high-legibility/usefulness example routes only to human pilot review, not approval.")

    bad_evidence = copy.deepcopy(synthetic)
    bad_evidence["evidence_refs"] = ["evidence-does-not-exist"]
    if not any("unresolved evidence reference" in error for error in semantic_errors(bad_evidence, concepts, source_ids, evidence_ids)):
        fail("mutation: unresolved evidence reference not rejected")
    print("PASS mutation: unresolved evidence reference rejected")

    failed_gate = copy.deepcopy(synthetic)
    failed_gate["hard_gates"]["accessibility"]["status"] = "fail"
    if recommend(failed_gate, evidence_ids) != "stop_or_reframe" or not any("does not match policy recommendation" in error for error in semantic_errors(failed_gate, concepts, source_ids, evidence_ids)):
        fail("mutation: failed hard gate cannot be outweighed by creativity ratings")
    print("PASS mutation: hard-gate failure overrides novelty/usefulness ratings")

    odd_and_unclear = copy.deepcopy(synthetic)
    for rating in odd_and_unclear["ratings"]:
        rating["scores"]["surprise"]["score"] = 5
        rating["scores"]["legibility"]["score"] = 1
    odd_and_unclear["recommendation"] = "prototype_for_comprehension_testing"
    if recommend(odd_and_unclear, evidence_ids) != "prototype_for_comprehension_testing" or semantic_errors(odd_and_unclear, concepts, source_ids, evidence_ids):
        fail("mutation: high surprise and low legibility did not route to comprehension prototype")
    print("PASS mutation: high-surprise/low-legibility concept routes to comprehension prototype only")

    unknown_gate_with_oddity = copy.deepcopy(odd_and_unclear)
    unknown_gate_with_oddity["hard_gates"]["life_safety"]["status"] = "unknown"
    unknown_gate_with_oddity["recommendation"] = "hold_for_evidence"
    if recommend(unknown_gate_with_oddity, evidence_ids) != "hold_for_evidence" or semantic_errors(unknown_gate_with_oddity, concepts, source_ids, evidence_ids):
        fail("mutation: unknown hard gate must block comprehension-prototype routing")
    print("PASS mutation: unknown hard gate takes precedence over high-surprise/low-legibility routing")

    missing_voice = copy.deepcopy(synthetic)
    missing_voice["ratings"] = [item for item in missing_voice["ratings"] if item["perspective"] != "affected_community"]
    missing_voice["recommendation"] = "hold_for_evidence"
    if semantic_errors(missing_voice, concepts, source_ids, evidence_ids):
        fail("mutation: missing affected-community perspective should hold, not invalidate the record")
    if recommend(missing_voice, evidence_ids) != "hold_for_evidence":
        fail("mutation: missing affected-community perspective incorrectly eligible")
    print("PASS mutation: missing affected-community voice blocks human pilot review")

    polarized_usefulness = copy.deepcopy(synthetic)
    polarized_usefulness["ratings"][0]["scores"]["usefulness"]["score"] = 2
    for rating in polarized_usefulness["ratings"][1:]:
        rating["scores"]["usefulness"]["score"] = 5
    if median_score(polarized_usefulness, "usefulness") < 4 or perspective_score_range(polarized_usefulness, "usefulness") != DISAGREEMENT_RANGE:
        fail("mutation setup: polarized usefulness must retain a favorable median and hit the exact three-point tripwire")
    polarized_usefulness["recommendation"] = "hold_for_evidence"
    polarized_usefulness["dissent_summary"] = ["Synthetic-only control: perspective-level usefulness differs sharply; no actual community review is represented."]
    if recommend(polarized_usefulness, evidence_ids) != "hold_for_evidence" or semantic_errors(polarized_usefulness, concepts, source_ids, evidence_ids):
        fail("mutation: favorable usefulness median concealed a major cross-perspective disagreement")
    print("PASS mutation: exact three-point usefulness split holds despite a favorable median")

    subthreshold_split = copy.deepcopy(synthetic)
    subthreshold_split["ratings"][0]["scores"]["usefulness"]["score"] = 3
    if perspective_score_range(subthreshold_split, "usefulness") != DISAGREEMENT_RANGE - 1:
        fail("mutation setup: subthreshold control must have a two-point usefulness range")
    if recommend(subthreshold_split, evidence_ids) != "eligible_for_human_pilot_review" or semantic_errors(subthreshold_split, concepts, source_ids, evidence_ids):
        fail("mutation: disagreement tripwire incorrectly triggers below its provisional threshold")
    print("PASS mutation: two-point usefulness split remains eligible for human review; no automatic approval")

    erased_disagreement = copy.deepcopy(polarized_usefulness)
    erased_disagreement["dissent_summary"] = []
    if not any("large cross-perspective rating differences must be preserved" in error for error in semantic_errors(erased_disagreement, concepts, source_ids, evidence_ids)):
        fail("mutation: major cross-perspective rating difference was allowed without a preserved dissent summary")
    print("PASS mutation: major cross-perspective difference cannot be hidden by an empty dissent summary")

    protocol_laundering = copy.deepcopy(plan)
    protocol_laundering["ratings"] = copy.deepcopy(synthetic["ratings"])
    if not list(validator.iter_errors(protocol_laundering)):
        fail("mutation: protocol-only record carrying scores was not rejected")
    print("PASS mutation: protocol-only record cannot carry panel scores")

    incomplete_comparator = copy.deepcopy(synthetic)
    incomplete_comparator["comparison_set"]["status"] = "incomplete"
    if not list(validator.iter_errors(incomplete_comparator)):
        fail("mutation: evaluated novelty without ready comparison set was not rejected by schema")
    print("PASS mutation: evaluated novelty requires documented comparison set")

    print("PASS: 11 novelty/legibility routing controls; no single composite creativity score; no automatic approval.")


if __name__ == "__main__":
    main()
