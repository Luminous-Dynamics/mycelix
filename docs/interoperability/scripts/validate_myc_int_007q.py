#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_DIMENSIONS = [
    "food","water","energy","repair-fabrication","compute-network",
    "governance-operations","evidence-provenance","health-safety-support",
    "external-legal-finance","critical-imports","skills-maintainers",
    "recovery-spares","federation-dependency",
]
SOURCE_CLASSES = {
    "DirectObservation","SourceOwnedOperationalFact","ReconstructedState",
    "DerivedAssessment","OperatorDeclaration","ImportedForeignEvidence","Unknown",
}
DEPENDENCY_STATES = {
    "LocallyAvailable","LocallyAvailableWithExternalInputs","ExternallyDependent",
    "DegradedWithoutExternalSupport","UnavailableWithoutExternalSupport","Unknown",
}
DISPOSITIONS = {
    "EvidenceSatisfiedUnderProfile","EvidenceIncomplete","EvidenceConflicting",
    "NotEstablished","Superseded",
}
EXPECTED_HOSTILE_IDS = [f"Q{i:02d}" for i in range(1, 16)]
EXPECTED_REQUIRED_EVIDENCE = ["local-acquisition-or-evidence-path", "dependency-map"]
ALLOWED_HOSTILE_EXPECTED = {
    "EvidenceIncomplete","NotEstablished","PreserveTradeoff",
    "QuantitativeClaimUnknown","Reject","EvidenceConflicting",
}


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def validate(checkpoint, transition, hostiles):
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    require(
        checkpoint.get("profile_id") == "myc-int-007q-observed-maturation-checkpoint-v1",
        "unexpected checkpoint profile",
    )
    require(checkpoint.get("profile_version") == "1.0.0", "checkpoint version drift")
    require(checkpoint.get("status") == "synthetic-validation-fixture", "checkpoint status drift")
    require(checkpoint.get("fixture_provenance") == "SyntheticValidationFixture",
            "checkpoint synthetic provenance drift")
    require(checkpoint.get("real_world_observation_claimed") is False,
            "synthetic checkpoint claims real-world observation")
    require(checkpoint.get("candidate_lifecycle_generation") == "N1",
            "positive checkpoint candidate generation drift")

    window = checkpoint.get("observation_window", {})
    require(bool(window.get("start")) and bool(window.get("end")),
            "checkpoint observation window missing")
    require(window.get("synthetic_time") is True,
            "synthetic checkpoint lost synthetic-time marker")

    planned = checkpoint.get("planned_reference", {})
    require(planned.get("profile") == "myc-int-007o-node-maturation-trajectory-v1",
            "planned profile reference drift")
    require(planned.get("snapshot_id") == "A-N1", "planned snapshot reference drift")
    require(planned.get("copied_as_observation") is False,
            "planned checkpoint copied/promoted as observation")

    entries = checkpoint.get("dimension_entries", [])
    dimensions = [item.get("dimension") for item in entries]
    require(dimensions == EXPECTED_DIMENSIONS, "checkpoint dimension set/order drift")
    require(len(set(dimensions)) == len(EXPECTED_DIMENSIONS), "duplicate checkpoint dimension")
    for item in entries:
        dim = item.get("dimension", "<unknown>")
        require(item.get("dependency_state") in DEPENDENCY_STATES,
                f"{dim}: invalid dependency state")
        require(item.get("source_class") in SOURCE_CLASSES,
                f"{dim}: invalid source class")
        require(isinstance(item.get("evidence_refs"), list),
                f"{dim}: evidence refs missing")
        require(bool(item.get("currentness")), f"{dim}: currentness missing")
        require(isinstance(item.get("missing_evidence"), list),
                f"{dim}: missing-evidence list absent")
        require(isinstance(item.get("conflicting_evidence"), list),
                f"{dim}: conflicting-evidence list absent")
        require(isinstance(item.get("remaining_external_dependencies"), list),
                f"{dim}: external-dependency list absent")
        require(bool(item.get("claim_ceiling")), f"{dim}: claim ceiling missing")

    require(checkpoint.get("authority") == "None", "checkpoint acquired authority")
    require(all(value is False for value in checkpoint.get("claim_ceiling", {}).values()),
            "checkpoint claim ceiling upgraded")
    require(checkpoint.get("quantitative_claims") == [],
            "positive synthetic fixture unexpectedly asserts quantitative claim")

    correction = checkpoint.get("correction_lineage", {})
    require(
        set(correction.keys()) == {"supersedes", "superseded_by", "correction_reason"},
        "correction-lineage shape drift",
    )

    require(
        transition.get("profile_id") == "myc-int-007q-maturation-transition-record-v1",
        "unexpected transition profile",
    )
    require(transition.get("profile_version") == "1.0.0", "transition version drift")
    require(transition.get("status") == "synthetic-validation-fixture",
            "transition status drift")
    require(transition.get("fixture_provenance") == "SyntheticValidationFixture",
            "transition synthetic provenance drift")
    require(
        transition.get("from_generation") == "N0"
        and transition.get("candidate_to_generation") == "N1",
        "positive transition generation drift",
    )
    require(transition.get("checkpoint_ref") == checkpoint.get("checkpoint_id"),
            "transition checkpoint binding mismatch")
    require(
        transition.get("policy_profile") == "myc-int-007o-node-maturation-trajectory-v1",
        "transition policy profile drift",
    )
    require(transition.get("required_evidence") == EXPECTED_REQUIRED_EVIDENCE,
            "N0->N1 required evidence drift")
    require(
        [item.get("requirement") for item in transition.get("satisfied_evidence", [])]
        == EXPECTED_REQUIRED_EVIDENCE,
        "positive transition does not satisfy exact requirement set",
    )
    require(transition.get("missing_evidence") == [], "positive transition gained missing evidence")
    require(transition.get("conflicting_evidence") == [],
            "positive transition gained conflicting evidence")
    require(transition.get("disposition") in DISPOSITIONS,
            "transition disposition outside descriptive vocabulary")
    require(transition.get("disposition") == "EvidenceSatisfiedUnderProfile",
            "positive transition disposition drift")
    require(transition.get("authority") == "None", "transition acquired authority")
    require(all(value is False for value in transition.get("claim_ceiling", {}).values()),
            "transition claim ceiling upgraded")

    require(hostiles.get("profile_id") == "myc-int-007q-hostile-cases-v1",
            "unexpected hostile profile")
    require(hostiles.get("profile_version") == "1.0.0", "hostile version drift")
    require(hostiles.get("status") == "synthetic-validation-fixture",
            "hostile status drift")
    cases = hostiles.get("cases", [])
    require([case.get("id") for case in cases] == EXPECTED_HOSTILE_IDS,
            "hostile case ID/order drift")
    require(len({case.get("id") for case in cases}) == 15,
            "hostile case IDs not unique")
    for case in cases:
        require(bool(case.get("mutation")), f"{case.get('id')}: mutation text missing")
        require(case.get("expected") in ALLOWED_HOSTILE_EXPECTED,
                f"{case.get('id')}: unexpected result vocabulary")

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("transition")
    parser.add_argument("hostiles")
    args = parser.parse_args()
    errors = validate(
        load_json(args.checkpoint),
        load_json(args.transition),
        load_json(args.hostiles),
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("MYC-INT-007Q observed-maturation fixtures: PASS")


if __name__ == "__main__":
    main()
