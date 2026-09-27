#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_PROFILE = "myc-int-018q-h1-maturation-binding-v1"
EXPECTED_STATUS = "design-binding-fixture"
EXPECTED_SUBJECTS = {
    "h1_manifest": "e0b8a5c3e58d937a94eab1ba975fc8ad1b36945d",
    "h1_install_run": "557665f9edbc8d08c2921958bd3b5db91e241c7c",
    "h1_validator": "9b946d7ea627a019532dd908a5093b94cca16147",
    "maturation_trajectory": "6489de3e2268c92e8b80c509ef60751bd28238cd",
    "maturation_checkpoint_contract": "52bdd21e1482272e75e122ebcc4a8fac37b2964d",
    "maturation_checkpoint_validator": "45fbd406a30657f21a8f9e3b8a481ed265ff7dc8",
}
EXPECTED_RELATIONS = {
    "CanSatisfyUnderExactProfile",
    "ContributesButInsufficient",
    "CannotSatisfy",
    "NotApplicable",
}
EXPECTED_BINDINGS = {
    ("N0->N1", "local-acquisition-or-evidence-path"): "CanSatisfyUnderExactProfile",
    ("N0->N1", "dependency-map"): "ContributesButInsufficient",
    ("N1->N2", "at-least-one-real-productive-loop"): "CannotSatisfy",
    ("N1->N2", "work-material-observations"): "CannotSatisfy",
    ("N1->N2", "outcome-feedback"): "CannotSatisfy",
    ("N2->N3", "bounded-outage-continuity-evidence"): "ContributesButInsufficient",
    ("N2->N3", "failure-recovery-evidence"): "ContributesButInsufficient",
    ("N2->N3", "local-safe-stop"): "CanSatisfyUnderExactProfile",
}
EXPECTED_DOMAIN_RELATIONS = {
    "water": "ContributesButInsufficient",
    "energy": "ContributesButInsufficient",
    "food": "CannotSatisfy",
    "repair-fabrication": "ContributesButInsufficient",
    "compute-network": "ContributesButInsufficient",
}
REQUIRED_FAILURES = {"FI-12", "FI-13", "FI-16", "FI-17"}
REQUIRED_LOCAL_OPTIONALITIES = {
    "mycelix_required_for_local_acquisition",
    "holochain_required_for_local_acquisition",
    "symthaea_required_for_local_acquisition",
    "itc_required_for_local_acquisition",
    "fleet_required_for_local_acquisition",
}
REQUIRED_RUN_EVIDENCE = {
    "fault-action-log",
    "manual-interventions",
    "stop-abort-events",
    "post-run-inspection-result",
}
REQUIRED_PRODUCTIVE = {
    "useful-output-subject",
    "production-window",
    "material-input-observations",
    "work-observations",
    "process-observations",
    "useful-output-observations",
    "loss-waste-failure-observations",
    "outcome-feedback",
    "external-dependency-import-map",
    "correction-supersession-lineage",
    "whole-node-extrapolation-prohibited",
}
EXPECTED_H1_STAGES = {
    "H1a": ("dry-instrumentation-power", False, False),
    "H1b": ("clean-water-closed-loop", True, False),
    "H1c": ("nutrient-solution-sensing", True, True),
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


def validate(bench, package, binding):
    errors = []

    if binding.get("profile_id") != EXPECTED_PROFILE:
        errors.append("018Q profile_id drift")
    if binding.get("profile_version") != "1.0.0":
        errors.append("018Q profile_version drift")
    if binding.get("status") != EXPECTED_STATUS:
        errors.append("018Q status drift")

    subjects = binding.get("source_subjects", {})
    for key, expected in EXPECTED_SUBJECTS.items():
        if subjects.get(key, {}).get("exact_subject") != expected:
            errors.append(f"source subject drift: {key}")

    if set(binding.get("relation_vocabulary", [])) != EXPECTED_RELATIONS:
        errors.append("relation vocabulary drift")

    global_inv = binding.get("global_invariants", {})
    if global_inv.get("whole_node_extrapolation_prohibited") is not True:
        errors.append("whole-node extrapolation must remain prohibited")
    for key in (
        "mapping_is_current_evidence",
        "mapping_establishes_transition",
        "mapping_grants_authority",
        "mapping_grants_federation_membership",
        "mapping_grants_governance_standing",
        "symthaea_analysis_is_direct_observation",
    ):
        if global_inv.get(key) is not False:
            errors.append(f"global invariant {key} must remain false")

    bindings = {}
    for item in binding.get("transition_bindings", []):
        key = (item.get("transition"), item.get("requirement"))
        if key in bindings:
            errors.append(f"duplicate transition binding: {key}")
        bindings[key] = item
        if item.get("currentness_window_required") is not True:
            errors.append(f"{key}: currentness/window must be required")
        if not item.get("completeness_ceiling"):
            errors.append(f"{key}: completeness ceiling missing")
        if not item.get("known_gaps"):
            errors.append(f"{key}: known gaps must remain explicit")
        if item.get("authority") != "None":
            errors.append(f"{key}: authority must remain None")
        if item.get("whole_node_extrapolation_prohibited") is not True:
            errors.append(f"{key}: whole-node extrapolation must remain prohibited")

    if set(bindings) != set(EXPECTED_BINDINGS):
        errors.append("transition binding set drift")
    for key, expected_relation in EXPECTED_BINDINGS.items():
        if bindings.get(key, {}).get("relation") != expected_relation:
            errors.append(f"{key}: expected relation {expected_relation}")

    for requirement in (
        "at-least-one-real-productive-loop",
        "work-material-observations",
        "outcome-feedback",
    ):
        item = bindings.get(("N1->N2", requirement), {})
        if item.get("relation") != "CannotSatisfy":
            errors.append(f"{requirement}: current H1 must remain CannotSatisfy")

    stages = {
        s.get("id"): (
            s.get("name"),
            s.get("water_present"),
            s.get("nutrient_solution_present"),
        )
        for s in bench.get("stages", [])
    }
    if stages != EXPECTED_H1_STAGES:
        errors.append("018K H1 stage semantics drift")

    channels = {c.get("id"): c for c in bench.get("channels", [])}
    for c in bench.get("channels", []):
        if c.get("actuation_authority") is not False:
            errors.append(f"018K channel {c.get('id')} gained actuation authority")

    for domain in binding.get("domain_scope_bindings", []):
        for channel_id in domain.get("h1_evidence", []):
            if channel_id in {
                "fault-action-log",
                "manual-interventions",
                "restart-power-cycle",
                "network-partition",
                "local-acquisition-can-continue",
            }:
                continue
            if channel_id not in channels:
                errors.append(f"domain binding references unknown H1 channel: {channel_id}")

    failure_ids = {f.get("id") for f in bench.get("failure_injections", [])}
    if not REQUIRED_FAILURES.issubset(failure_ids):
        errors.append("required H1 fault injections missing")

    optionalities = bench.get("runtime_optionalities", {})
    for key in REQUIRED_LOCAL_OPTIONALITIES:
        if optionalities.get(key) is not False:
            errors.append(f"{key} must remain false for local acquisition")

    h1c_checks = set(package.get("stage_gates", {}).get("H1c", {}).get("required_checks", []))
    if "automatic-dosing-disabled" not in h1c_checks:
        errors.append("H1c automatic dosing must remain disabled")

    stop = set(package.get("stop_abort_conditions", []))
    if "manual-stop-unavailable" not in stop:
        errors.append("manual-stop-unavailable stop condition missing")
    evidence = set(package.get("run_evidence_requirements", []))
    missing_run = sorted(REQUIRED_RUN_EVIDENCE - evidence)
    if missing_run:
        errors.append(f"run evidence requirements missing: {missing_run}")

    for key, value in package.get("claim_ceiling", {}).items():
        if value is not False:
            errors.append(f"018O claim ceiling {key} must remain false")

    for dependency in package.get("local_stop_dependencies", []):
        if dependency in {"Mycelix", "Holochain", "Symthaea", "Fleet", "network"}:
            errors.append(f"local stop cannot depend on {dependency}")

    domains = {d.get("dimension"): d for d in binding.get("domain_scope_bindings", [])}
    if set(domains) != set(EXPECTED_DOMAIN_RELATIONS):
        errors.append("domain scope set drift")
    for dimension, expected_relation in EXPECTED_DOMAIN_RELATIONS.items():
        item = domains.get(dimension, {})
        if item.get("relation") != expected_relation:
            errors.append(f"{dimension}: relation drift")
        if not item.get("prohibited_inference"):
            errors.append(f"{dimension}: prohibited inference missing")
        if not item.get("node_level_gap"):
            errors.append(f"{dimension}: node-level gap missing")

    gap = binding.get("productive_loop_gap", {})
    if gap.get("current_h1_profile_must_not_be_widened_silently") is not True:
        errors.append("silent H1 widening must remain prohibited")
    if gap.get("follow_on_profile_required") is not True:
        errors.append("ProductiveLoop follow-on must remain required")
    if gap.get("candidate_profile_name") != "ProductiveLoopV1":
        errors.append("ProductiveLoop profile name drift")
    if not REQUIRED_PRODUCTIVE.issubset(set(gap.get("required_semantics", []))):
        errors.append("ProductiveLoop semantics incomplete")
    if gap.get("hydroponics_may_be_first_conformer") is not True:
        errors.append("hydroponics first-conformer flag drift")
    if gap.get("food_is_only_valid_productive_loop") is not False:
        errors.append("ProductiveLoop must not be hard-coded to food only")

    for key, value in binding.get("claim_ceiling", {}).items():
        if value is not False:
            errors.append(f"018Q claim ceiling {key} must remain false")

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bench_manifest")
    parser.add_argument("install_run_package")
    parser.add_argument("maturation_binding")
    args = parser.parse_args()
    errors = validate(
        load_json(args.bench_manifest),
        load_json(args.install_run_package),
        load_json(args.maturation_binding),
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("MYC-INT-018R: PASS")


if __name__ == "__main__":
    main()
