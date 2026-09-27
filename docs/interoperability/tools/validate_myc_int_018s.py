#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

GENERIC_PROFILE = "myc-int-018s-productive-loop-v1"
H2_PROFILE = "myc-int-018s-h2-hydroponic-productive-loop-v1"
EXPECTED_GENERIC_STATUS = "design-contract"
EXPECTED_H2_STATUS = "synthetic-planned-showcase"
EXPECTED_H1_SUBJECTS = {
    "h1_manifest": "e0b8a5c3e58d937a94eab1ba975fc8ad1b36945d",
    "h1_install_run": "557665f9edbc8d08c2921958bd3b5db91e241c7c",
    "h1_maturation_binding": "3d30fbefc8ad49dcf7d4fa789022aa985b122141",
    "h1_maturation_validator": "bc986eae0a998cf30e23681e20447f8ae118ee94",
}
REQUIRED_SECTIONS = {
    "loop_identity", "useful_output_profile", "production_window",
    "input_material_observations", "work_observations", "process_observation_refs",
    "useful_output_observations", "loss_waste_failure_observations",
    "outcome_feedback", "external_dependency_import_map", "currentness_coverage",
    "conflicts_unknowns", "correction_supersession_lineage",
}
REQUIRED_INPUT = {
    "material_resource_subject", "quantity", "unit_profile", "source_class",
    "event_or_window_time", "origin_provenance", "local_or_external_source",
    "estimated_or_observed_status", "correction_lineage",
}
REQUIRED_WORK = {
    "work_event_subject", "actor_or_role_reference_under_privacy_profile",
    "bounded_time_or_duration_profile", "activity_class", "productive_loop_subject",
    "source_class", "evidence_refs", "correction_lineage",
}
REQUIRED_OUTPUT = {
    "output_subject", "output_profile", "quantity", "unit_profile", "event_or_window_time",
    "source_class", "disposition", "evidence_refs", "correction_lineage",
}
REQUIRED_OUTCOME = {
    "intended_use_or_output", "useful_output_obtained_status", "defects_rejections_losses",
    "post_cycle_inspection", "deviations_unknowns", "recovery_or_adaptation_required",
}
REQUIRED_N1N2 = {
    "at-least-one-real-productive-loop", "work-material-observations", "outcome-feedback",
}
REQUIRED_CONFORMERS = {
    "hydroponic-food-production", "fabrication", "repair", "water-treatment", "energy-service",
}
EXPECTED_STAGES = [
    ("H2a", "crop-lot-input-work-preregistration"),
    ("H2b", "cultivation-process-observation-window"),
    ("H2c", "harvest-useful-output-observation"),
    ("H2d", "losses-outcome-feedback-post-cycle-review"),
]
REQUIRED_H2_EXTERNAL = {
    "seeds-genetics", "nutrients-material-feedstocks", "water-input", "electricity-energy",
    "replacement-parts", "calibration-materials", "external-specialist-labor-or-knowledge",
    "legal-food-safety-or-other-external-services",
}
REQUIRED_GUARDS = {
    "H1-process-telemetry-alone-does-not-establish-productive-loop",
    "planned-input-does-not-equal-consumed-material",
    "work-estimate-does-not-equal-observed-work",
    "work-does-not-issue-credits-or-standing",
    "harvested-output-does-not-imply-edible-or-food-safe",
    "pH-EC-nominal-does-not-imply-useful-output",
    "one-cycle-does-not-imply-node-food-independence",
    "external-inputs-must-remain-visible",
    "unknown-denominator-does-not-permit-yield-share",
    "Symthaea-prediction-does-not-become-output-observation",
    "correction-does-not-rewrite-history",
    "ProductiveLoop-does-not-grant-authority",
    "H2-references-H1-observations-without-duplicating-identity",
    "synthetic-fixture-does-not-become-physical-evidence",
    "completed-loop-does-not-auto-establish-N2",
}


class ValidationError(ValueError):
    pass


def _strict_object(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValidationError(f"duplicate JSON key: {key}")
        obj[key] = value
    return obj


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle, object_pairs_hook=_strict_object)


def _all_false(mapping, prefix, errors):
    for key, value in mapping.items():
        if value is not False:
            errors.append(f"{prefix} {key} must remain false")


def validate(bench, generic, h2):
    errors = []

    if generic.get("profile_id") != GENERIC_PROFILE:
        errors.append("generic profile_id drift")
    if generic.get("profile_version") != "1.0.0":
        errors.append("generic version drift")
    if generic.get("status") != EXPECTED_GENERIC_STATUS:
        errors.append("generic status drift")
    if generic.get("whole_node_extrapolation_prohibited") is not True:
        errors.append("generic whole-node extrapolation must remain prohibited")
    if generic.get("authority") != "None":
        errors.append("generic authority must remain None")

    if not REQUIRED_SECTIONS.issubset(set(generic.get("required_sections", []))):
        errors.append("generic required section set incomplete")
    if not REQUIRED_INPUT.issubset(set(generic.get("input_material_observation_requirements", []))):
        errors.append("input/material requirements incomplete")
    if not REQUIRED_WORK.issubset(set(generic.get("work_observation_requirements", []))):
        errors.append("work requirements incomplete")
    if not REQUIRED_OUTPUT.issubset(set(generic.get("useful_output_observation_requirements", []))):
        errors.append("useful-output requirements incomplete")
    if not REQUIRED_OUTCOME.issubset(set(generic.get("outcome_feedback_requirements", []))):
        errors.append("outcome feedback requirements incomplete")

    _all_false(generic.get("work_nonclaims", {}), "work nonclaim", errors)

    proc = generic.get("process_observation_contract", {})
    if proc.get("domain_owned") is not True:
        errors.append("process evidence must remain domain-owned")
    if proc.get("referenced_not_duplicated") is not True:
        errors.append("process evidence must be referenced, not duplicated")
    if proc.get("analysis_can_replace_source_observation") is not False:
        errors.append("analysis cannot replace source observation")

    _all_false(generic.get("output_nonclaims", {}), "output nonclaim", errors)

    metrics = generic.get("quantitative_metric_rule", {})
    if metrics.get("denominator_required_when_share_or_yield_claimed") is not True:
        errors.append("share/yield denominator must remain required")
    if metrics.get("coverage_window_required") is not True:
        errors.append("metric coverage window must remain required")
    if metrics.get("unknown_denominator_means_metric_unknown") is not True:
        errors.append("unknown denominator must mean unknown metric")

    mat = generic.get("maturation_binding", {})
    if mat.get("transition") != "N1->N2":
        errors.append("generic maturation transition drift")
    if set(mat.get("requirements", [])) != REQUIRED_N1N2:
        errors.append("generic N1->N2 requirement set drift")
    if mat.get("evidence_complete_loop_automatically_establishes_transition") is not False:
        errors.append("ProductiveLoop cannot auto-establish N2")
    if mat.get("transition_record_required") is not True:
        errors.append("maturation transition record must remain required")

    corr = generic.get("correction_policy", {})
    if corr.get("history_rewrite_allowed") is not False:
        errors.append("history rewrite must remain prohibited")
    if corr.get("supersession_required") is not True:
        errors.append("supersession must remain required")

    if not REQUIRED_CONFORMERS.issubset(set(generic.get("supported_conformer_classes", []))):
        errors.append("generic conformer support narrowed too far")

    _all_false(generic.get("claim_ceiling", {}), "generic claim ceiling", errors)

    if h2.get("profile_id") != H2_PROFILE:
        errors.append("H2 profile_id drift")
    if h2.get("profile_version") != "1.0.0":
        errors.append("H2 version drift")
    if h2.get("status") != EXPECTED_H2_STATUS:
        errors.append("H2 status drift")
    if h2.get("conforms_to") != GENERIC_PROFILE:
        errors.append("H2 generic conformance drift")

    if h2.get("parent_h1_subjects") != EXPECTED_H1_SUBJECTS:
        errors.append("H2 exact H1 parent subjects drift")
    if h2.get("whole_node_extrapolation_prohibited") is not True:
        errors.append("H2 whole-node extrapolation must remain prohibited")
    if h2.get("authority") != "None":
        errors.append("H2 authority must remain None")
    if h2.get("synthetic_fixture") is not True:
        errors.append("H2 must remain synthetic fixture")
    if h2.get("physical_cycle_executed") is not False:
        errors.append("H2 physical cycle must remain unexecuted")

    crop = h2.get("crop_profile", {})
    if crop.get("binding_status") != "Unbound":
        errors.append("H2 crop profile must remain explicitly Unbound")
    if crop.get("crop_or_cultivar_subject") is not None or crop.get("agronomic_profile") is not None:
        errors.append("H2 crop/agronomic profile cannot be silently prebound")

    stages = [(s.get("id"), s.get("name")) for s in h2.get("stages", [])]
    if stages != EXPECTED_STAGES:
        errors.append("H2 stage vocabulary/order drift")

    if h2.get("h1_channel_identity_duplicated") is not False:
        errors.append("H2 must not duplicate H1 channel identity")
    bench_channels = {c.get("id") for c in bench.get("channels", [])}
    unknown = sorted(set(h2.get("h1_process_observation_refs", [])) - bench_channels)
    if unknown:
        errors.append(f"H2 references unknown H1 channels: {unknown}")

    if not h2.get("input_material_classes"):
        errors.append("H2 input material classes missing")
    if not h2.get("work_activity_classes"):
        errors.append("H2 work activity classes missing")
    if not h2.get("planned_output_observations"):
        errors.append("H2 planned output observations missing")
    if not h2.get("planned_outcome_feedback"):
        errors.append("H2 outcome feedback missing")
    if not REQUIRED_H2_EXTERNAL.issubset(set(h2.get("external_dependency_categories_required", []))):
        errors.append("H2 external dependency map incomplete")

    out = h2.get("useful_output_profile", {})
    for key in ("edible_claim_allowed", "food_safety_claim_allowed", "marketability_claim_allowed"):
        if out.get(key) is not False:
            errors.append(f"H2 {key} must remain false")

    contrib = h2.get("maturation_contribution", {})
    if contrib.get("transition") != "N1->N2":
        errors.append("H2 maturation transition drift")
    if set(contrib.get("can_contribute_if_physical_evidence_complete", [])) != REQUIRED_N1N2:
        errors.append("H2 maturation requirement set drift")
    if contrib.get("automatically_establishes_N2") is not False:
        errors.append("H2 cannot auto-establish N2")
    if contrib.get("transition_record_required") is not True:
        errors.append("H2 transition record must remain required")

    guards = set(h2.get("negative_semantic_guards", []))
    if not REQUIRED_GUARDS.issubset(guards):
        errors.append("H2 negative semantic guards incomplete")

    _all_false(h2.get("claim_ceiling", {}), "H2 claim ceiling", errors)

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bench_manifest")
    parser.add_argument("productive_loop_contract")
    parser.add_argument("h2_conformer")
    args = parser.parse_args()
    errors = validate(
        load_json(args.bench_manifest),
        load_json(args.productive_loop_contract),
        load_json(args.h2_conformer),
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("MYC-INT-018T: PASS")


if __name__ == "__main__":
    main()
