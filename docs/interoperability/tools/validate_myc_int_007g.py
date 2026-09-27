#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_PARENT = "006f7da146b4d1c781967c3ee8a6161b411b633c"
EXECUTION_MODES = {"UNBOUND", "DeterministicLocal", "NetworkEmulated", "MixedPhysicalVirtual"}
REQUIRED_EVIDENCE = {
    "run-manifest-exact-bytes",
    "implementation-build-identities",
    "node-logs-with-run-identity",
    "semantic-event-exports",
    "delivery-attempt-and-ack-traces",
    "authority-decision-evidence",
    "schema-and-translation-receipts",
    "conflict-and-reconciliation-records",
    "fault-injector-event-log",
    "final-state-export-commitments",
    "environment-capsule",
}
REQUIRED_SAFETY = {
    "federation-harness-does-not-gain-h1-actuator-authority",
    "edge-local-acquisition-does-not-depend-on-federation",
    "physical-node-stop-does-not-depend-on-network",
    "synthetic-replay-evidence-remains-labeled",
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


def _ids(items, field="id"):
    values = [item[field] for item in items]
    if len(values) != len(set(values)):
        raise ValidationError(f"duplicate {field}: {values}")
    return values


def validate(federation, run):
    errors = []

    if run.get("parent_007f_subject") != EXPECTED_PARENT:
        errors.append("run parent_007f_subject does not bind the exact 007F subject")

    nodes = federation.get("nodes", [])
    node_ids = _ids(nodes)
    node_by_id = {node["id"]: node for node in nodes}

    bindings = run.get("node_bindings", [])
    binding_ids = [binding.get("fixture_node") for binding in bindings]
    if len(binding_ids) != len(set(binding_ids)):
        errors.append("duplicate fixture_node binding")
    if set(binding_ids) != set(node_ids):
        errors.append(f"node binding set drift: expected {node_ids}, got {binding_ids}")

    for binding in bindings:
        node_id = binding.get("fixture_node")
        if node_id not in node_by_id:
            continue
        expected_provenance = node_by_id[node_id].get("provenance")
        if binding.get("provenance") != expected_provenance:
            errors.append(f"{node_id}: provenance drift")
        if node_id == "A" and binding.get("runtime_mode") != "external-physical":
            errors.append("A must remain external-physical in the frozen F0 run profile")
        if node_id == "G" and binding.get("schema_generation") != "from-007f-old-generation":
            errors.append("G must remain bound to the deliberately old schema generation")
        if node_id == "A" and binding.get("actuator_authority") not in (None, False):
            errors.append("A cannot gain actuator authority from the federation run manifest")

    phase_ids = _ids(federation.get("demonstration_phases", []))
    run_phase_ids = run.get("workload", {}).get("phase_ids", [])
    if run_phase_ids != phase_ids:
        errors.append(f"phase set/order drift: expected {phase_ids}, got {run_phase_ids}")

    fault_ids = _ids(federation.get("fault_campaigns", []))
    run_fault_ids = run.get("fault_campaigns", [])
    if run_fault_ids != fault_ids:
        errors.append(f"fault set/order drift: expected {fault_ids}, got {run_fault_ids}")

    execution_mode = run.get("execution_mode")
    if execution_mode not in EXECUTION_MODES:
        errors.append(f"unknown execution_mode: {execution_mode}")

    evidence = set(run.get("evidence_capture", []))
    missing_evidence = sorted(REQUIRED_EVIDENCE - evidence)
    if missing_evidence:
        errors.append(f"missing evidence requirements: {missing_evidence}")

    safety = set(run.get("safety_invariants", []))
    missing_safety = sorted(REQUIRED_SAFETY - safety)
    if missing_safety:
        errors.append(f"missing safety invariants: {missing_safety}")

    claim = run.get("claim_ceiling", {})
    for key in ("executed", "qualified", "scalability_established", "physical_effect_authorized"):
        if claim.get(key) is not False:
            errors.append(f"claim ceiling {key} must remain false in design fixture")

    if execution_mode == "UNBOUND":
        identity = run.get("run_identity", {})
        for key in (
            "orchestrator_tool",
            "orchestrator_version",
            "environment_profile",
            "environment_commitment",
        ):
            if identity.get(key) is not None:
                errors.append(f"UNBOUND run cannot fabricate {key}")
        for binding in bindings:
            if binding.get("implementation_subject") is not None:
                errors.append(f"UNBOUND run cannot pin implementation for {binding.get('fixture_node')}")

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("federation")
    parser.add_argument("run_manifest")
    args = parser.parse_args()
    errors = validate(load_json(args.federation), load_json(args.run_manifest))
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("MYC-INT-007H: PASS")


if __name__ == "__main__":
    main()
