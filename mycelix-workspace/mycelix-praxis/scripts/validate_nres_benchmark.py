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
CAPABILITY_LOOP_PATH = ROOT / "examples/courses/civic-resilience-capability-loop-fixture-001.json"


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



def validate_capability_loop(loop: dict[str, Any], fixture: dict[str, Any]) -> list[str]:
    errors: list[str] = []

    if loop.get("benchmark_id") != "PRAXIS-NRES-005-CAPABILITY-LOOP-001":
        errors.append("unexpected capability-loop benchmark_id")
    if loop.get("schema_version") != "1.0.0":
        errors.append("unsupported capability-loop schema_version")
    if loop.get("status") != "synthetic-demonstration-for-review":
        errors.append("capability-loop fixture must remain explicitly synthetic")
    if loop.get("evaluation_split") != "public-development-only":
        errors.append("capability-loop fixture must remain public development data")
    if loop.get("active_in_curriculum") is not False:
        errors.append("capability-loop fixture must remain inactive")
    if loop.get("scenario_id") != fixture.get("scenario", {}).get("event_id"):
        errors.append("capability-loop scenario_id must match the NRES-002 fixture")

    objectives = loop.get("objectives", [])
    objective_ids = [item.get("id") for item in objectives if isinstance(item, dict)]
    if not isinstance(objectives, list) or not objectives:
        errors.append("capability-loop objectives must be a non-empty array")
        objectives = []
    if len(objective_ids) != len(set(objective_ids)):
        errors.append("capability-loop objective IDs must be unique")

    measures = loop.get("measure_catalog", [])
    measure_ids = [item.get("id") for item in measures if isinstance(item, dict)]
    if len(measure_ids) != len(set(measure_ids)):
        errors.append("capability-loop measure IDs must be unique")
    for measure in measures:
        if not isinstance(measure, dict):
            errors.append("every measure definition must be an object")
            continue
        for field in ("name", "layer", "numerator", "denominator", "unit", "formula", "guardrail"):
            if not measure.get(field):
                errors.append(f"measure {measure.get('id')!r} is missing {field}")

    required_measure_ids = {
        "M-OBJECTIVE-EVIDENCE", "M-SERVICE-FLOOR", "M-RESOURCE-FEASIBILITY",
        "M-AUTHORITY-EVIDENCE", "M-ACCESSIBILITY-COVERAGE",
        "M-ACTION-VERIFICATION", "M-REPEAT-DEFECT",
        "M-LEARNING-TRANSFER", "M-COST-PER-VERIFIED-IMPROVEMENT",
    }
    if not required_measure_ids.issubset(set(measure_ids)):
        errors.append("capability-loop measure catalog is missing required measures")

    cycles = loop.get("cycles", [])
    cycle_ids = [item.get("cycle_id") for item in cycles if isinstance(item, dict)]
    if not isinstance(cycles, list) or len(cycles) < 2:
        errors.append("capability-loop requires at least two exercise cycles")
        cycles = []
    if len(cycle_ids) != len(set(cycle_ids)):
        errors.append("capability-loop cycle IDs must be unique")
    cycle_by_id = {item.get("cycle_id"): item for item in cycles if isinstance(item, dict)}

    objectives_set = set(objective_ids)
    service_floors = fixture.get("synthetic_constraints", {}).get("minimum_service_floor_percent", {})
    expected_services = set(service_floors)

    for cycle in cycles:
        if not isinstance(cycle, dict):
            errors.append("every capability-loop cycle must be an object")
            continue
        cycle_id = cycle.get("cycle_id")
        if cycle.get("data_status") != "illustrative-synthetic":
            errors.append(f"{cycle_id}: all public example cycle data must be labeled synthetic")

        scheduled = cycle.get("scheduled_objective_ids", [])
        if set(scheduled) != objectives_set or len(scheduled) != len(objectives_set):
            errors.append(f"{cycle_id}: scheduled objectives must cover each declared objective exactly once")

        observations = cycle.get("objective_observations", [])
        observed_ids = [item.get("objective_id") for item in observations if isinstance(item, dict)]
        if set(observed_ids) != objectives_set or len(observed_ids) != len(objectives_set):
            errors.append(f"{cycle_id}: objective observations must cover each objective exactly once")
        for observation in observations:
            if not isinstance(observation, dict):
                errors.append(f"{cycle_id}: objective observation must be an object")
                continue
            if observation.get("accepted_observation") is True and not observation.get("evidence_ids"):
                errors.append(f"{cycle_id}: accepted observation must include evidence IDs")

        service_observations = cycle.get("service_floor_observations", [])
        service_names = [item.get("service") for item in service_observations if isinstance(item, dict)]
        if set(service_names) != expected_services or len(service_names) != len(expected_services):
            errors.append(f"{cycle_id}: service observations must cover every fixture service exactly once")
        for observation in service_observations:
            if not isinstance(observation, dict):
                errors.append(f"{cycle_id}: service-floor observation must be an object")
                continue
            service = observation.get("service")
            if observation.get("required_floor_percent") != service_floors.get(service):
                errors.append(f"{cycle_id}: {service!r} floor must match the frozen NRES-002 fixture")
            value = observation.get("observed_percent")
            if not isinstance(value, (int, float)) or not 0 <= value <= 100:
                errors.append(f"{cycle_id}: {service!r} observed percentage must be between 0 and 100")
            if not observation.get("evidence_id"):
                errors.append(f"{cycle_id}: {service!r} observation needs an evidence_id")

        resources = cycle.get("resources", {})
        expected_budget = fixture.get("synthetic_constraints", {}).get("planning_budget", {}).get("available")
        expected_hours = fixture.get("synthetic_constraints", {}).get("deployable_trained_responder_hours", {}).get("available")
        for available_key, committed_key, expected, label in (
            ("planning_budget_available", "planning_budget_committed", expected_budget, "planning budget"),
            ("responder_hours_available", "responder_hours_committed", expected_hours, "responder hours"),
        ):
            available, committed = resources.get(available_key), resources.get(committed_key)
            if available != expected:
                errors.append(f"{cycle_id}: {label} availability must match the frozen NRES-002 fixture")
            if not isinstance(committed, (int, float)) or not isinstance(available, (int, float)) or committed < 0 or committed > available:
                errors.append(f"{cycle_id}: {label} commitment must be non-negative and within availability")

        authority = cycle.get("authority_evidence", {})
        decisions, complete = authority.get("decisions_sampled"), authority.get("decisions_with_role_basis_and_review_path")
        if not isinstance(decisions, int) or not isinstance(complete, int) or decisions < 0 or complete < 0 or complete > decisions:
            errors.append(f"{cycle_id}: authority-evidence numerator/denominator is invalid")

        accessibility = cycle.get("accessibility", {})
        requirements, tested, passing = accessibility.get("requirements_specified"), accessibility.get("requirements_tested"), accessibility.get("requirements_passing")
        if not all(isinstance(value, int) for value in (requirements, tested, passing)) or not 0 <= passing <= tested <= requirements:
            errors.append(f"{cycle_id}: accessibility counts must satisfy 0 <= passing <= tested <= specified")

        evidence = cycle.get("evidence_records", [])
        evidence_ids = [item.get("id") for item in evidence if isinstance(item, dict)]
        if len(evidence_ids) != len(set(evidence_ids)):
            errors.append(f"{cycle_id}: evidence IDs must be unique within a cycle")
        evidence_id_set = set(evidence_ids)
        for observation in observations:
            if isinstance(observation, dict):
                referenced_ids = observation.get("evidence_ids", [])
                if any(reference not in evidence_id_set for reference in referenced_ids):
                    errors.append(f"{cycle_id}: objective observation references missing evidence")
        for observation in service_observations:
            if isinstance(observation, dict) and observation.get("evidence_id") not in evidence_id_set:
                errors.append(f"{cycle_id}: service-floor observation references missing evidence")

    actions = loop.get("corrective_actions", [])
    action_ids = [item.get("action_id") for item in actions if isinstance(item, dict)]
    if len(action_ids) != len(set(action_ids)):
        errors.append("corrective-action IDs must be unique")
    for action in actions:
        if not isinstance(action, dict):
            errors.append("every corrective action must be an object")
            continue
        if action.get("opened_cycle_id") not in cycle_by_id or action.get("due_by_cycle_id") not in cycle_by_id:
            errors.append(f"{action.get('action_id')!r}: opened and due cycles must exist")
        retest_id = action.get("retest_cycle_id")
        if retest_id is not None and retest_id not in cycle_by_id:
            errors.append(f"{action.get('action_id')!r}: retest cycle must exist")
        if not action.get("owner_role"):
            errors.append(f"{action.get('action_id')!r}: corrective action needs an accountable owner role")
        if action.get("status") not in {"open", "closed-verified", "blocked", "late", "not-tested", "reopened"}:
            errors.append(f"{action.get('action_id')!r}: unsupported corrective-action status")
        if action.get("status") == "closed-verified":
            if not action.get("completion_evidence_id") or action.get("independently_verified") is not True:
                errors.append(f"{action.get('action_id')!r}: verified closure requires completion evidence and independent verification")
            if not retest_id or action.get("retest_result") != "pass":
                errors.append(f"{action.get('action_id')!r}: verified closure requires a successful retest")
            retest_cycle = cycle_by_id.get(retest_id, {})
            retest_evidence_ids = {
                evidence.get("id") for evidence in retest_cycle.get("evidence_records", [])
                if isinstance(evidence, dict)
            }
            if action.get("completion_evidence_id") not in retest_evidence_ids:
                errors.append(f"{action.get('action_id')!r}: completion evidence must exist in the retest cycle")
            objective = next((item for item in objectives if item.get("id") == action.get("retest_objective_id")), None)
            if objective is None or action.get("retest_service") != objective.get("service"):
                errors.append(f"{action.get('action_id')!r}: retest objective and service mapping must agree")
            else:
                retested_service = next(
                    (item for item in retest_cycle.get("service_floor_observations", []) if item.get("service") == objective.get("service")),
                    None,
                )
                if retested_service is None or retested_service.get("observed_percent", -1) < retested_service.get("required_floor_percent", 101):
                    errors.append(f"{action.get('action_id')!r}: passing action retest must pass its linked service floor")
        elif action.get("status") == "open" and action.get("retest_result") == "pass":
            errors.append(f"{action.get('action_id')!r}: open action cannot report a passing retest without reconciliation")

    expected = loop.get("expected_derived_results", {})
    for cycle_key, cycle_id in (("CYCLE_01", "CYCLE-01"), ("CYCLE_02", "CYCLE-02")):
        cycle = cycle_by_id.get(cycle_id)
        if cycle is None:
            errors.append(f"expected derived results require {cycle_id}")
            continue
        expected_cycle = expected.get(cycle_key, {})
        observations = cycle.get("objective_observations", [])
        evidence_count = sum(1 for item in observations if item.get("accepted_observation") is True and item.get("evidence_ids"))
        scheduled_count = len(cycle.get("scheduled_objective_ids", []))
        service_obs = cycle.get("service_floor_observations", [])
        floor_passes = sum(1 for item in service_obs if item.get("observed_percent", -1) >= item.get("required_floor_percent", 101))
        resources = cycle.get("resources", {})
        authority = cycle.get("authority_evidence", {})
        accessibility = cycle.get("accessibility", {})
        derived_checks = (
            (("objective_evidence_coverage", "numerator"), evidence_count),
            (("objective_evidence_coverage", "denominator"), scheduled_count),
            (("service_floors_passed", "numerator"), floor_passes),
            (("service_floors_passed", "denominator"), len(service_obs)),
            (("planning_budget", "committed"), resources.get("planning_budget_committed")),
            (("planning_budget", "available"), resources.get("planning_budget_available")),
            (("responder_hours", "committed"), resources.get("responder_hours_committed")),
            (("responder_hours", "available"), resources.get("responder_hours_available")),
            (("authority_evidence", "numerator"), authority.get("decisions_with_role_basis_and_review_path")),
            (("authority_evidence", "denominator"), authority.get("decisions_sampled")),
            (("accessibility", "numerator"), accessibility.get("requirements_tested")),
            (("accessibility", "denominator"), accessibility.get("requirements_specified")),
        )
        for (group, field), value in derived_checks:
            if expected_cycle.get(group, {}).get(field) != value:
                errors.append(f"{cycle_id}: expected_derived_results mismatch for {group}.{field}")

    if len(cycles) >= 2:
        cycle2 = cycle_by_id.get("CYCLE-02", {})
        cycle2_expected = expected.get("CYCLE_02", {})
        due_actions = [item for item in actions if item.get("due_by_cycle_id") == "CYCLE-02"]
        verified_actions = [item for item in due_actions if item.get("status") == "closed-verified" and item.get("completion_evidence_id") and item.get("independently_verified") is True and item.get("retest_result") == "pass"]
        if cycle2_expected.get("verified_corrective_actions", {}).get("numerator") != len(verified_actions):
            errors.append("CYCLE-02: expected verified corrective-action numerator is inconsistent")
        if cycle2_expected.get("verified_corrective_actions", {}).get("denominator") != len(due_actions):
            errors.append("CYCLE-02: expected verified corrective-action denominator is inconsistent")

        c1 = cycle_by_id.get("CYCLE-01", {})
        previous_failures = {
            item.get("service") for item in c1.get("service_floor_observations", [])
            if item.get("observed_percent", 101) < item.get("required_floor_percent", 0)
        }
        retest_failures = {
            item.get("service") for item in cycle2.get("service_floor_observations", [])
            if item.get("service") in previous_failures and item.get("observed_percent", 101) < item.get("required_floor_percent", 0)
        }
        retested_previous = previous_failures.intersection({
            item.get("service") for item in cycle2.get("service_floor_observations", [])
        })
        if cycle2_expected.get("retested_failure_recurrence", {}).get("numerator") != len(retest_failures):
            errors.append("CYCLE-02: expected recurrence numerator is inconsistent")
        if cycle2_expected.get("retested_failure_recurrence", {}).get("denominator") != len(retested_previous):
            errors.append("CYCLE-02: expected recurrence denominator is inconsistent")

    forbidden_composite_keys = {"overall_resilience_score", "composite_score", "leaderboard_score", "learner_suitability_score"}
    if forbidden_composite_keys.intersection(loop):
        errors.append("capability-loop must not include composite, leaderboard or personal suitability scores")
    guardrails = loop.get("interpretation_guardrails", [])
    if len(guardrails) < 5 or not any("single overall" in item for item in guardrails):
        errors.append("capability-loop must prohibit a single compensating overall score")

    return errors


def self_test_capability_loop(
    loop: dict[str, Any],
    fixture: dict[str, Any],
) -> int:
    cases: list[tuple[str, dict[str, Any], str]] = []

    over_budget = copy.deepcopy(loop)
    over_budget["cycles"][0]["resources"]["planning_budget_committed"] = 1001
    cases.append(("budget overrun", over_budget, "planning budget commitment"))

    floor_drift = copy.deepcopy(loop)
    floor_drift["cycles"][1]["service_floor_observations"][3]["required_floor_percent"] = 70
    cases.append(("service floor drift", floor_drift, "floor must match the frozen NRES-002 fixture"))

    false_closure = copy.deepcopy(loop)
    false_closure["corrective_actions"][2]["status"] = "closed-verified"
    cases.append(("unverified action closure", false_closure, "verified closure requires completion evidence"))

    aggregate = copy.deepcopy(loop)
    aggregate["overall_resilience_score"] = 100
    cases.append(("aggregate score added", aggregate, "capability-loop must prohibit a single compensating overall score"))

    for name, test_loop, expected in cases:
        errors = validate_capability_loop(test_loop, fixture)
        if not any(expected in error for error in errors):
            print(f"SELF-TEST FAIL: {name}: expected rejection containing {expected!r}", file=sys.stderr)
            return 1

    print(f"SELF-TEST PASS: {len(cases)} capability-loop negative controls rejected unsafe measurement mutations")
    return 0


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
    if not FIXTURE_PATH.is_file() or not COURSE_PATH.is_file() or not WORKBOOK_PATH.is_file() or not CAPABILITY_LOOP_PATH.is_file():
        print("ERROR: required course, fixture, workbook or capability-loop file is missing", file=sys.stderr)
        return 2

    try:
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        course = json.loads(COURSE_PATH.read_text(encoding="utf-8"))
        workbook = json.loads(WORKBOOK_PATH.read_text(encoding="utf-8"))
        capability_loop = json.loads(CAPABILITY_LOOP_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"ERROR: cannot read valid JSON inputs: {error}", file=sys.stderr)
        return 2

    errors = validate(fixture, course)
    errors.extend(validate_workbook(workbook, fixture, course))
    errors.extend(validate_capability_loop(capability_loop, fixture))
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print("PASS: course, fixture and workbook internal consistency only; no educational or policy-effect claim")
    if "--self-test" in sys.argv[1:]:
        base_status = self_test(fixture, course)
        if base_status != 0:
            return base_status
        workbook_status = self_test_workbook(workbook, fixture, course)
        if workbook_status != 0:
            return workbook_status
        return self_test_capability_loop(capability_loop, fixture)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
