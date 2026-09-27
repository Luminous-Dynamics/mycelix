#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_PARENT = "adf94f84adfd73ea94cdcf5fc9a1a96a05e10472"
ALLOWED_STATUS = {"Draft", "ReadyForReview", "ReviewedWithOpenItems", "ApprovedForDryBringUp", "Superseded"}
REQUIRED_STAGE_NAMES = {
    "H1a": "dry-instrumentation-power",
    "H1b": "clean-water-closed-loop",
    "H1c": "nutrient-solution-sensing",
}
REQUIRED_STOP = {
    "leak-detected",
    "unexpected-current-or-overtemperature",
    "manual-stop-unavailable",
    "unintended-actuator-state",
    "loss-of-containment",
    "critical-sensor-or-interface-failure",
    "unknown-wiring-or-device-identity",
    "operator-concern",
}
REQUIRED_RUN_EVIDENCE = {
    "exact-installation-package-generation",
    "exact-run-profile",
    "exact-hardware-install-identities",
    "pre-run-checklist-result",
    "calibration-check-evidence-refs",
    "observation-export",
    "fault-action-log",
    "manual-interventions",
    "stop-abort-events",
    "post-run-inspection-result",
    "deviations-and-unknowns",
    "evidence-commitments",
}
REQUIRED_ELECTRICAL_KEYS = {
    "power_rails", "protection_devices", "switching_elements", "ground_domains",
    "isolation_barriers", "level_shifters_or_buffers", "connector_pin_map",
    "manual_stop_override", "wet_dry_boundary", "power_budget_reviewed",
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


def validate(bench, matrix, package):
    errors = []

    if package.get("parent_018n_subject") != EXPECTED_PARENT:
        errors.append("package does not bind exact 018N subject")

    if package.get("package_status") not in ALLOWED_STATUS:
        errors.append("unknown package_status")

    claim = package.get("claim_ceiling", {})
    for key, value in claim.items():
        if value is not False:
            errors.append(f"claim ceiling {key} must remain false in design fixture")

    bench_stages = {item["id"]: item["name"] for item in bench.get("stages", [])}
    if bench_stages != REQUIRED_STAGE_NAMES:
        errors.append("018K stage vocabulary drift")

    stage_gates = package.get("stage_gates", {})
    if set(stage_gates) != set(REQUIRED_STAGE_NAMES):
        errors.append("018O stage set must be exactly H1a/H1b/H1c")
    for stage, expected_name in REQUIRED_STAGE_NAMES.items():
        if stage_gates.get(stage, {}).get("name") != expected_name:
            errors.append(f"{stage}: stage name drift")

    if "H1a-approved" not in stage_gates.get("H1b", {}).get("requires", []):
        errors.append("H1b must require H1a approval")
    if "H1b-evidence-reviewed" not in stage_gates.get("H1c", {}).get("requires", []):
        errors.append("H1c must require H1b evidence review")
    if package.get("hydraulic_process", {}).get("first_wet_test_medium") != "clean-water-only":
        errors.append("H1b first wet test must remain clean-water-only")
    if "automatic-dosing-disabled" not in stage_gates.get("H1c", {}).get("required_checks", []):
        errors.append("H1c must keep automatic dosing disabled")

    electrical = package.get("electrical_design", {})
    missing_keys = sorted(REQUIRED_ELECTRICAL_KEYS - set(electrical))
    if missing_keys:
        errors.append(f"electrical design missing keys: {missing_keys}")
    if electrical.get("power_budget_reviewed") not in (False, True):
        errors.append("power_budget_reviewed must be boolean")

    stop = set(package.get("stop_abort_conditions", []))
    missing_stop = sorted(REQUIRED_STOP - stop)
    if missing_stop:
        errors.append(f"missing local stop conditions: {missing_stop}")

    evidence = set(package.get("run_evidence_requirements", []))
    missing_evidence = sorted(REQUIRED_RUN_EVIDENCE - evidence)
    if missing_evidence:
        errors.append(f"missing run evidence requirements: {missing_evidence}")

    for channel in bench.get("channels", []):
        if channel.get("actuation_authority") is not False:
            errors.append(f"018K channel {channel.get('id')} gained actuation authority")

    matrix_claim = matrix.get("claim_ceiling", {})
    for key, value in matrix_claim.items():
        if value is not False:
            errors.append(f"018N claim ceiling {key} unexpectedly true")

    candidates = {item.get("candidate_id"): item for item in matrix.get("candidates", [])}
    pump = candidates.get("pump-dfrobot-fit0200")
    if pump:
        if pump.get("acquisition_owner") != "ActuatorProgramDeferred":
            errors.append("pump actuator path must remain deferred/separate from sensor provider")
        if pump.get("provider_profile") == "read-only local digital sensor":
            errors.append("pump cannot masquerade as sensor provider")

    known_candidates = set(candidates)
    for item in package.get("delivered_hardware_inventory", []):
        candidate_id = item.get("candidate_id")
        if candidate_id and candidate_id not in known_candidates:
            errors.append(f"unknown delivered candidate_id: {candidate_id}")
        if item.get("revision_mismatch") is True and not item.get("deviation_from_candidate"):
            errors.append(f"{candidate_id}: revision mismatch must retain explicit deviation")

    forbidden_stop_dependencies = {"Mycelix", "Holochain", "Symthaea", "Fleet", "network"}
    for dependency in package.get("local_stop_dependencies", []):
        if dependency in forbidden_stop_dependencies:
            errors.append(f"local stop cannot depend on {dependency}")

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bench_manifest")
    parser.add_argument("integration_matrix")
    parser.add_argument("install_run_package")
    args = parser.parse_args()
    errors = validate(
        load_json(args.bench_manifest),
        load_json(args.integration_matrix),
        load_json(args.install_run_package),
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("MYC-INT-018P: PASS")


if __name__ == "__main__":
    main()
