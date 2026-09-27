#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

EXPECTED_STATES = [
    "LocallyAvailable",
    "LocallyAvailableWithExternalInputs",
    "ExternallyDependent",
    "DegradedWithoutExternalSupport",
    "UnavailableWithoutExternalSupport",
    "Unknown",
]
EXPECTED_DIMENSIONS = [
    "food",
    "water",
    "energy",
    "repair-fabrication",
    "compute-network",
    "governance-operations",
    "evidence-provenance",
    "health-safety-support",
    "external-legal-finance",
    "critical-imports",
    "skills-maintainers",
    "recovery-spares",
    "federation-dependency",
]
EXPECTED_SNAPSHOTS = ["A-N0", "A-N1", "A-N2", "A-N3", "A-N4", "A-N5"]
EXPECTED_GENERATIONS = ["N0", "N1", "N2", "N3", "N4", "N5"]
EXPECTED_TRANSITION_EVIDENCE = {
    "N0->N1": ["local-acquisition-or-evidence-path", "dependency-map"],
    "N1->N2": [
        "at-least-one-real-productive-loop",
        "work-material-observations",
        "outcome-feedback",
    ],
    "N2->N3": [
        "bounded-outage-continuity-evidence",
        "failure-recovery-evidence",
        "local-safe-stop",
    ],
    "N3->N4": [
        "domain-specific-dependency-change-evidence",
        "remaining-critical-imports",
    ],
    "N4->N5": [
        "node-seed-package-generation",
        "secret-free-audit",
        "selective-admission-contract",
    ],
}
ALLOWED_SCORE_KEYS = {
    "scalar_maturity_score_allowed",
    "scalar_self_sufficiency_score_allowed",
    "show_single_overall_score",
}


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def score_like_keys(value, path="$"):
    findings = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_l = key.lower()
            if "score" in key_l and key not in ALLOWED_SCORE_KEYS:
                findings.append(f"{path}.{key}")
            findings.extend(score_like_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            findings.extend(score_like_keys(child, f"{path}[{index}]"))
    return findings


def dependency_map(snapshot):
    return {
        item.get("dimension"): item.get("state")
        for item in snapshot.get("dependency_states", [])
    }


def validate(trajectory):
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    require(
        trajectory.get("profile_id") == "myc-int-007o-node-maturation-trajectory-v1",
        "unexpected trajectory profile_id",
    )
    require(trajectory.get("profile_version") == "1.0.0", "profile version drift")
    require(trajectory.get("status") == "design-fixture", "trajectory status drift")
    require(trajectory.get("node") == "A", "trajectory no longer binds Node A")
    require(
        trajectory.get("provenance_class") == "SyntheticPlannedShowcase",
        "synthetic planned provenance was upgraded or changed",
    )
    require(
        trajectory.get("physical_observation_claimed") is False,
        "planned trajectory now claims physical observation",
    )
    require(
        trajectory.get("scalar_maturity_score_allowed") is False,
        "scalar maturity score enabled",
    )
    require(
        trajectory.get("scalar_self_sufficiency_score_allowed") is False,
        "scalar self-sufficiency score enabled",
    )
    require(
        trajectory.get("dependency_state_vocabulary") == EXPECTED_STATES,
        "dependency-state vocabulary drift",
    )
    require(
        trajectory.get("capability_dimensions") == EXPECTED_DIMENSIONS,
        "capability dimension vocabulary/order drift",
    )
    require(
        not score_like_keys(trajectory),
        f"unapproved aggregate score-like key(s): {score_like_keys(trajectory)}",
    )

    snapshots = trajectory.get("snapshots", [])
    require(
        [item.get("snapshot_id") for item in snapshots] == EXPECTED_SNAPSHOTS,
        "snapshot ID set/order drift",
    )
    require(
        [item.get("lifecycle_generation") for item in snapshots] == EXPECTED_GENERATIONS,
        "lifecycle generation set/order drift",
    )

    state_by_generation = {}
    for index, snapshot in enumerate(snapshots):
        sid = snapshot.get("snapshot_id", f"snapshot-{index}")
        generation = snapshot.get("lifecycle_generation")
        require(
            snapshot.get("evidence_class") == "SyntheticPlannedShowcase",
            f"{sid}: evidence class is no longer synthetic/planned",
        )
        require(
            snapshot.get("transition_claim") == "PlannedNotExecuted",
            f"{sid}: transition claim upgraded",
        )
        require(
            snapshot.get("remaining_external_dependencies_explicit") is True,
            f"{sid}: remaining external dependencies hidden",
        )

        states = snapshot.get("dependency_states", [])
        dimensions = [item.get("dimension") for item in states]
        require(dimensions == EXPECTED_DIMENSIONS, f"{sid}: dimension set/order drift")
        require(len(set(dimensions)) == len(EXPECTED_DIMENSIONS), f"{sid}: duplicate dimension")
        for item in states:
            require(
                item.get("state") in EXPECTED_STATES,
                f"{sid}/{item.get('dimension')}: invalid dependency state",
            )
        state_by_generation[generation] = dependency_map(snapshot)

        expected_seed_state = (
            "PlannedNotExecuted" if generation == "N5" else "PlannedNotApplicable"
        )
        require(
            snapshot.get("seed_export_state") == expected_seed_state,
            f"{sid}: seed export state drift",
        )

    for generation in ("N3", "N4", "N5"):
        states = state_by_generation.get(generation, {})
        for dimension in (
            "health-safety-support",
            "external-legal-finance",
            "critical-imports",
        ):
            require(
                states.get(dimension) == "ExternallyDependent",
                f"{generation}/{dimension}: planned external dependency was hidden",
            )

    require(
        state_by_generation.get("N4") == state_by_generation.get("N5"),
        "N5 dependency vector diverged from N4; seeder capability must not imply fake autarky",
    )

    display = trajectory.get("display_contract", {})
    require(display.get("show_dimensions_independently") is True,
            "UI no longer shows dimensions independently")
    require(display.get("show_remaining_external_dependencies") is True,
            "UI hides remaining external dependencies")
    require(display.get("show_unknowns") is True, "UI hides unknowns")
    require(display.get("show_tradeoffs") is True, "UI hides trade-offs")
    require(display.get("show_single_overall_score") is False,
            "UI enabled a single overall score")

    require(
        trajectory.get("transition_evidence_requirements") == EXPECTED_TRANSITION_EVIDENCE,
        "transition evidence requirements drift",
    )
    require(
        all(value is False for value in trajectory.get("claim_ceiling", {}).values()),
        "trajectory claim ceiling upgraded",
    )

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory")
    args = parser.parse_args()
    errors = validate(load_json(args.trajectory))
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        raise SystemExit(1)
    print("MYC-INT-007O maturation trajectory fixture: PASS")


if __name__ == "__main__":
    main()
