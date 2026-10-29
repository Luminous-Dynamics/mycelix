#!/usr/bin/env python3
"""Validate the draft Praxis NRES policy-learning fixture without network access.

This checks internal fixture/course consistency only. It does not validate
factual claims, educational effectiveness, policy merits, or legal compliance.
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_PATH = ROOT / "examples/courses/civic-resilience-policy-benchmark-fixture-001.json"
COURSE_PATH = ROOT / "examples/courses/civic-resilience-democratic-defence.json"
WORKBOOK_PATH = ROOT / "examples/courses/civic-resilience-capstone-workbook-001.json"


def validate(fixture: dict[str, Any], course: dict[str, Any]) -> list[str]:
    errors: list[str] = []

    if fixture.get("benchmark_id") != "PRAXIS-NRES-002-FIXTURE-001":
        errors.append("unexpected benchmark_id")
    if fixture.get("schema_version") != "1.0.0":
        errors.append("unsupported schema_version")
    if fixture.get("status") != "synthetic-prototype-for-review":
        errors.append("fixture must remain explicitly synthetic and draft")

    scope = fixture.get("scope", {})
    if scope.get("setting") != "fictional municipality":
        errors.append("scenario must identify a fictional municipality")
    if not fixture.get("scenario", {}).get("known_facts"):
        errors.append("scenario must list its known facts")

    options = fixture.get("policy_options_to_compare", [])
    option_ids = [option.get("id") for option in options]
    if len(options) != 4:
        errors.append("exactly four policy options are required for this frozen fixture")
    if any(not isinstance(option_id, str) or not option_id for option_id in option_ids):
        errors.append("every policy option needs a non-empty id")
    if len(option_ids) != len(set(option_ids)):
        errors.append("policy option IDs must be unique")
    for option in options:
        if not option.get("name") or not option.get("description"):
            errors.append(f"policy option {option.get('id')!r} needs name and description")
        if not option.get("unknowns_to_surface"):
            errors.append(f"policy option {option.get('id')!r} must state unknowns")

    rubric = fixture.get("rubric", {})
    dimensions = rubric.get("dimensions", [])
    dimension_ids = [dimension.get("id") for dimension in dimensions]
    if len(dimensions) != 8:
        errors.append("exactly eight criterion-referenced rubric dimensions are required")
    if len(dimension_ids) != len(set(dimension_ids)):
        errors.append("rubric dimension IDs must be unique")
    if any(dimension.get("points") != 4 for dimension in dimensions):
        errors.append("each rubric dimension must have four points available")
    expected_max = sum(dimension.get("points", 0) for dimension in dimensions)
    if rubric.get("maximum_points") != expected_max:
        errors.append("rubric maximum_points must equal the sum of dimension points")
    if rubric.get("scale") != "0-4 per dimension; points support feedback and must not be used to rank political beliefs":
        errors.append("rubric must preserve its non-political-scoring boundary")

    variants = fixture.get("adversarial_variants", [])
    variant_ids = [variant.get("id") for variant in variants]
    if len(variants) < 8:
        errors.append("at least eight adversarial variants are required")
    if len(variant_ids) != len(set(variant_ids)):
        errors.append("adversarial variant IDs must be unique")
    for variant in variants:
        if not variant.get("mutation") or not variant.get("expected_skill"):
            errors.append(f"variant {variant.get('id')!r} needs mutation and expected_skill")

    if len(fixture.get("hard_fail_conditions", [])) < 6:
        errors.append("hard-fail conditions are incomplete")
    deliverable = fixture.get("required_learner_deliverable", {})
    if len(deliverable.get("must_include", [])) < 8:
        errors.append("learner deliverable contract is incomplete")

    constraints = fixture.get("synthetic_constraints", {})
    if constraints.get("planning_horizon_hours") != 72:
        errors.append("planning horizon must remain 72 hours for this fixture")
    budget = constraints.get("planning_budget", {})
    hours = constraints.get("deployable_trained_responder_hours", {})
    if not isinstance(budget.get("available"), (int, float)) or budget.get("available", 0) <= 0:
        errors.append("synthetic planning budget must be positive")
    if not isinstance(hours.get("available"), (int, float)) or hours.get("available", 0) <= 0:
        errors.append("synthetic responder-hour availability must be positive")
    floors = constraints.get("minimum_service_floor_percent", {})
    if not floors or any(not isinstance(value, (int, float)) or not 0 <= value <= 100 for value in floors.values()):
        errors.append("service floors must be percentages between 0 and 100")
    if "toy exercise inputs" not in constraints.get("constraint_note", ""):
        errors.append("synthetic constraint disclaimer is missing")

    # Praxis course contract boundary: the draft remains non-personalized,
    # and lesson-hour metadata agrees with the syllabus.
    if course.get("course_id") != "civic-resilience-democratic-defence-v0-1":
        errors.append("unexpected Praxis course_id")
    if course.get("fl_enabled") is not False:
        errors.append("federated learning must remain disabled for this draft course")
    if course.get("model_id") is not None:
        errors.append("draft course must not bind a personalization model")
    if course.get("metadata", {}).get("no_political_affiliation_or_enlistment_scoring") is not True:
        errors.append("course must forbid political-affiliation and enlistment-propensity scoring")
    if course.get("metadata", {}).get("fixtures_are_synthetic") is not True:
        errors.append("course must label assessment fixtures synthetic")
    syllabus = course.get("syllabus", [])
    total_hours = sum(module.get("estimated_hours", 0) for module in syllabus)
    if not syllabus or total_hours != course.get("estimated_hours"):
        errors.append("course estimated_hours must equal the syllabus hour sum")
    if course.get("metadata", {}).get("assessment_fixture") != FIXTURE_PATH.relative_to(ROOT).as_posix():
        errors.append("course must reference the exact assessment fixture path")
    if course.get("metadata", {}).get("benchmark_spec") != "docs/curriculum/PRAXIS_NRES_002_BENCHMARK.md":
        errors.append("course must reference the benchmark specification")

    if not fixture.get("evaluation_protocol", {}).get("non_claims"):
        errors.append("evaluation protocol must state non-claims")
    sources = fixture.get("sources", [])
    if not sources or any(not source.get("title") or not source.get("url") for source in sources):
        errors.append("every source needs a title and URL")

    return errors



def validate_workbook(
    workbook: dict[str, Any],
    fixture: dict[str, Any],
    course: dict[str, Any],
) -> list[str]:
    errors: list[str] = []

    if workbook.get("workbook_id") != "PRAXIS-NRES-003-WORKBOOK-001":
        errors.append("unexpected workbook_id")
    if workbook.get("schema_version") != "1.0.0":
        errors.append("unsupported workbook schema_version")
    if workbook.get("status") != "draft-review-only-not-active":
        errors.append("workbook must remain a draft and inactive")
    if workbook.get("active_in_curriculum") is not False:
        errors.append("workbook must not be activated in the curriculum graph")
    if workbook.get("course_id") != course.get("course_id"):
        errors.append("workbook course_id must match the proposal course")
    if workbook.get("benchmark_fixture") != FIXTURE_PATH.relative_to(ROOT).as_posix():
        errors.append("workbook must reference the exact public fixture path")
    if workbook.get("scoring_specification") != "docs/curriculum/PRAXIS_NRES_003_SCORING_ANCHORS.md":
        errors.append("workbook must reference its anchored scoring specification")
    if workbook.get("study_protocol") != "docs/curriculum/PRAXIS_NRES_004_PREREGISTRATION.md":
        errors.append("workbook must reference the study preregistration protocol")

    course_metadata = course.get("metadata", {})
    if course_metadata.get("capstone_workbook") != WORKBOOK_PATH.relative_to(ROOT).as_posix():
        errors.append("course metadata must reference the exact capstone workbook")
    if course_metadata.get("scoring_specification") != workbook.get("scoring_specification"):
        errors.append("course and workbook scoring-specification references must agree")
    if course_metadata.get("study_protocol") != workbook.get("study_protocol"):
        errors.append("course and workbook study-protocol references must agree")

    sections = workbook.get("sections", [])
    if not isinstance(sections, list):
        errors.append("workbook sections must be an array")
        sections = []
    section_ids = [section.get("id") for section in sections if isinstance(section, dict)]
    if len(sections) != 8:
        errors.append("workbook must contain exactly eight capstone sections")
    if len(section_ids) != len(set(section_ids)):
        errors.append("workbook section IDs must be unique")
    for section in sections:
        if not isinstance(section, dict):
            errors.append("every workbook section must be an object")
            continue
        if not section.get("title") or not section.get("prompts") or not section.get("evidence_to_submit"):
            errors.append(f"workbook section {section.get('id')!r} needs title, prompts and evidence_to_submit")

    expected_dimensions = [dimension.get("id") for dimension in fixture.get("rubric", {}).get("dimensions", [])]
    if workbook.get("rubric_dimensions") != expected_dimensions:
        errors.append("workbook rubric dimensions must match the fixture rubric in order")
    required_sections = workbook.get("required_submission_sections", [])
    if not isinstance(required_sections, list) or len(required_sections) < 10:
        errors.append("workbook required submission contract is incomplete")
    guardrails = workbook.get("non_compensable_guardrails", [])
    if not isinstance(guardrails, list) or len(guardrails) < 5:
        errors.append("workbook non-compensable guardrails are incomplete")

    expected_labels = fixture.get("required_learner_deliverable", {}).get("required_uncertainty_labels", [])
    if workbook.get("required_uncertainty_labels") != expected_labels:
        errors.append("workbook uncertainty labels must match the frozen fixture contract")

    learner_data = workbook.get("learner_data", {})
    if not isinstance(learner_data, dict):
        errors.append("workbook learner_data must be an object")
        learner_data = {}
    if learner_data.get("default") != "local-only":
        errors.append("workbook learner data must default to local-only")
    if learner_data.get("federated_learning_enabled") is not False:
        errors.append("federated learning must remain disabled for this workbook")
    if learner_data.get("political_or_enlistment_inference") is not False:
        errors.append("workbook must prohibit political and enlistment inference")

    validation = workbook.get("validation", {})
    if not isinstance(validation, dict):
        errors.append("workbook validation metadata must be an object")
        validation = {}
    if validation.get("public_fixture_role") != "development-and-formative-practice-only":
        errors.append("public fixture must be labeled development/formative only")
    if validation.get("public_variants_are_held_out") is not False:
        errors.append("public variants must not be represented as held out")
    if not validation.get("confirmatory_evaluation_requires"):
        errors.append("workbook must specify a separate confirmatory holdout requirement")
    if not validation.get("no_claims"):
        errors.append("workbook must state its validity and activation non-claims")

    if workbook.get("estimated_hours") != next(
        (module.get("estimated_hours") for module in course.get("syllabus", []) if module.get("module") == 8),
        None,
    ):
        errors.append("workbook estimated_hours must match the capstone module")

    return errors


def self_test_workbook(
    workbook: dict[str, Any],
    fixture: dict[str, Any],
    course: dict[str, Any],
) -> int:
    cases: list[tuple[str, dict[str, Any], str]] = []

    active = copy.deepcopy(workbook)
    active["active_in_curriculum"] = True
    cases.append(("active workbook", active, "workbook must not be activated"))

    politicized = copy.deepcopy(workbook)
    politicized["learner_data"]["political_or_enlistment_inference"] = True
    cases.append(("political inference enabled", politicized, "workbook must prohibit political"))

    leaked = copy.deepcopy(workbook)
    leaked["validation"]["public_variants_are_held_out"] = True
    cases.append(("public variants misrepresented as held out", leaked, "public variants must not be represented"))

    rubric_drift = copy.deepcopy(workbook)
    rubric_drift["rubric_dimensions"] = ["R1"]
    cases.append(("rubric dimension drift", rubric_drift, "workbook rubric dimensions must match"))

    malformed_sections = copy.deepcopy(workbook)
    malformed_sections["sections"] = None
    cases.append(("malformed sections", malformed_sections, "workbook sections must be an array"))

    malformed_learner_data = copy.deepcopy(workbook)
    malformed_learner_data["learner_data"] = None
    cases.append(("malformed learner data", malformed_learner_data, "workbook learner_data must be an object"))

    malformed_validation = copy.deepcopy(workbook)
    malformed_validation["validation"] = None
    cases.append(("malformed validation metadata", malformed_validation, "workbook validation metadata must be an object"))

    for name, test_workbook, expected in cases:
        errors = validate_workbook(test_workbook, fixture, course)
        if not any(expected in error for error in errors):
            print(f"SELF-TEST FAIL: {name}: expected rejection containing {expected!r}", file=sys.stderr)
            return 1

    print(f"SELF-TEST PASS: {len(cases)} workbook negative controls rejected invalid scope, scoring and split mutations")
    return 0


def self_test(fixture: dict[str, Any], course: dict[str, Any]) -> int:
    cases: list[tuple[str, dict[str, Any], dict[str, Any], str]] = []

    duplicate_option = copy.deepcopy(fixture)
    duplicate_option["policy_options_to_compare"][1]["id"] = duplicate_option["policy_options_to_compare"][0]["id"]
    cases.append(("duplicate option ID", duplicate_option, copy.deepcopy(course), "policy option IDs must be unique"))

    fake_claim = copy.deepcopy(fixture)
    fake_claim["synthetic_constraints"]["constraint_note"] = "real city estimate"
    cases.append(("missing synthetic disclaimer", fake_claim, copy.deepcopy(course), "synthetic constraint disclaimer is missing"))

    changed_points = copy.deepcopy(fixture)
    changed_points["rubric"]["maximum_points"] = 999
    cases.append(("incorrect rubric total", changed_points, copy.deepcopy(course), "rubric maximum_points must equal the sum of dimension points"))

    duplicate_variant = copy.deepcopy(fixture)
    duplicate_variant["adversarial_variants"][1]["id"] = duplicate_variant["adversarial_variants"][0]["id"]
    cases.append(("duplicate adversarial variant", duplicate_variant, copy.deepcopy(course), "adversarial variant IDs must be unique"))

    enabled_fl = copy.deepcopy(course)
    enabled_fl["fl_enabled"] = True
    cases.append(("federated learning enabled", copy.deepcopy(fixture), enabled_fl, "federated learning must remain disabled"))

    wrong_hours = copy.deepcopy(course)
    wrong_hours["estimated_hours"] += 1
    cases.append(("course hour mismatch", copy.deepcopy(fixture), wrong_hours, "course estimated_hours must equal the syllabus hour sum"))

    bad_scoring = copy.deepcopy(course)
    bad_scoring["metadata"]["no_political_affiliation_or_enlistment_scoring"] = False
    cases.append(("political scoring enabled", copy.deepcopy(fixture), bad_scoring, "course must forbid political-affiliation"))

    for name, test_fixture, test_course, expected in cases:
        errors = validate(test_fixture, test_course)
        if not any(expected in error for error in errors):
            print(f"SELF-TEST FAIL: {name}: expected rejection containing {expected!r}", file=sys.stderr)
            return 1

    print(f"SELF-TEST PASS: {len(cases)} negative controls rejected invalid fixture/course mutations")
    return 0


def main() -> int:
    if not FIXTURE_PATH.is_file() or not COURSE_PATH.is_file() or not WORKBOOK_PATH.is_file():
        print("ERROR: required course, fixture or capstone workbook file is missing", file=sys.stderr)
        return 2

    try:
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        course = json.loads(COURSE_PATH.read_text(encoding="utf-8"))
        workbook = json.loads(WORKBOOK_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"ERROR: cannot read valid JSON inputs: {error}", file=sys.stderr)
        return 2

    errors = validate(fixture, course)
    errors.extend(validate_workbook(workbook, fixture, course))
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print("PASS: course, fixture and workbook internal consistency only; no educational or policy-effect claim")
    if "--self-test" in sys.argv[1:]:
        base_status = self_test(fixture, course)
        if base_status != 0:
            return base_status
        return self_test_workbook(workbook, fixture, course)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
