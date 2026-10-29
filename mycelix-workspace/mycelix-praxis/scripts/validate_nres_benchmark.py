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


def self_test(fixture: dict[str, Any], course: dict[str, Any]) -> int:
    cases: list[tuple[str, Any]] = []

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
    if not FIXTURE_PATH.is_file() or not COURSE_PATH.is_file():
        print("ERROR: required course or fixture file is missing", file=sys.stderr)
        return 2

    try:
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        course = json.loads(COURSE_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"ERROR: cannot read valid JSON inputs: {error}", file=sys.stderr)
        return 2

    errors = validate(fixture, course)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print("PASS: fixture/course internal consistency only; no educational or policy-effect claim")
    if "--self-test" in sys.argv[1:]:
        return self_test(fixture, course)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
