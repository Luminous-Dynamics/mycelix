#!/usr/bin/env python3
"""Offline structural preflight for the public CrossDomainDisruptionV1 seed.

This checks fixture-package consistency only. It is not the independent
projection/scenario/decision oracle tracked by Mycelix #2950, and it does not
validate intelligence output, canonical protocol bytes, truth, or authority.
Uses only the Python standard library and performs no network I/O.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


FIXTURE_NAMES = {
    "f0": "CROSS_DOMAIN_DISRUPTION_V1_F0_SOLVER_VISIBLE.json",
    "f1": "CROSS_DOMAIN_DISRUPTION_V1_F1_DELTA.json",
    "f2": "CROSS_DOMAIN_DISRUPTION_V1_F2_DELTA.json",
    "f3": "CROSS_DOMAIN_DISRUPTION_V1_F3_DELTA.json",
    "predicates": "CROSS_DOMAIN_DISRUPTION_V1_EXPECTED_PREDICATES.json",
    "mutations": "CROSS_DOMAIN_DISRUPTION_V1_MUTATIONS.json",
}
EXPECTED_PREDICATES = 20
EXPECTED_MUTATIONS = 23
REQUIRED_MOCK_PERMIT_DISPOSITIONS = {
    "BlockedNoCurrentPermit",
    "RejectStalePermit",
    "RejectWrongSubject",
    "RejectWrongCandidate",
    "RejectWrongPayload",
}
FORBIDDEN_EVALUATOR_KEYS = {
    "evaluator_only_world",
    "evaluator_truth_payload",
    "hidden_oracle_values",
    "actual_world_truth_payload",
}


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _reject_non_json_constant(value: str) -> None:
    raise ValueError(f"non-JSON numeric constant is forbidden: {value}")


def _load(path: Path, errors: list[str]) -> dict[str, Any] | None:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream,
                object_pairs_hook=_reject_duplicate_keys,
                parse_constant=_reject_non_json_constant,
            )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        errors.append(f"{path.name}: cannot parse strict JSON: {exc}")
        return None
    if not isinstance(value, dict):
        errors.append(f"{path.name}: root must be a JSON object")
        return None
    return value


def _require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def _string_refs(value: Any, label: str, errors: list[str]) -> list[str]:
    """Return valid string refs while recording malformed values without hashing them."""
    if not isinstance(value, list):
        errors.append(f"{label}: expected an array of string refs")
        return []
    refs: list[str] = []
    for index, item in enumerate(value):
        if not isinstance(item, str) or not item:
            errors.append(f"{label}[{index}]: expected a non-empty string ref")
            continue
        refs.append(item)
    return refs


def _unique_refs(items: Any, field: str, label: str, errors: list[str]) -> set[str]:
    if not isinstance(items, list):
        errors.append(f"{label}: expected an array")
        return set()
    refs: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict) or not isinstance(item.get(field), str) or not item[field]:
            errors.append(f"{label}[{index}]: missing non-empty {field}")
            continue
        refs.append(item[field])
    if len(refs) != len(set(refs)):
        errors.append(f"{label}: duplicate {field} values")
    return set(refs)


def _check_refs(
    records: Any,
    field: str,
    allowed: set[str],
    label: str,
    errors: list[str],
    *,
    required: bool = True,
) -> None:
    if not isinstance(records, list):
        errors.append(f"{label}: expected an array")
        return
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            errors.append(f"{label}[{index}]: expected an object")
            continue
        value = record.get(field)
        if not required and value is None:
            continue
        if not isinstance(value, str) or value not in allowed:
            errors.append(f"{label}[{index}]: {field} does not resolve: {value!r}")


def _parse_time(value: Any, label: str, errors: list[str]) -> datetime | None:
    if not isinstance(value, str):
        errors.append(f"{label}: timestamp must be an ISO-8601 string")
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        errors.append(f"{label}: invalid ISO-8601 timestamp {value!r}")
        return None
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        errors.append(f"{label}: timestamp must include a timezone")
        return None
    return parsed


def _walk_for_forbidden_keys(value: Any, path: str, errors: list[str]) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key.lower() in FORBIDDEN_EVALUATOR_KEYS:
                errors.append(f"{path}: forbidden evaluator-only key {key!r}")
            _walk_for_forbidden_keys(child, f"{path}.{key}", errors)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _walk_for_forbidden_keys(child, f"{path}[{index}]", errors)


def _validate_fixture_dir(fixture_dir: Path) -> list[str]:
    """Internal implementation; caller converts malformed nested shapes to findings."""
    errors: list[str] = []
    loaded: dict[str, dict[str, Any]] = {}
    for name, filename in FIXTURE_NAMES.items():
        value = _load(fixture_dir / filename, errors)
        if value is not None:
            loaded[name] = value
            _walk_for_forbidden_keys(value, filename, errors)
    if set(loaded) != set(FIXTURE_NAMES):
        return errors

    f0, f1, f2, f3 = (loaded[key] for key in ("f0", "f1", "f2", "f3"))
    predicates, mutations = loaded["predicates"], loaded["mutations"]

    for label, value in (("F0", f0), ("F1", f1), ("F2", f2), ("F3", f3)):
        _require(
            value.get("fixture_profile") == "OPS-INTEL-TEST-001"
            or value.get("profile") == "OPS-INTEL-TEST-001",
            f"{label}: unexpected fixture profile",
            errors,
        )
        _require(value.get("fixture_version") == "CrossDomainDisruptionV1", f"{label}: wrong fixture version", errors)
        _require(value.get("data_class") == "synthetic", f"{label}: data_class must be synthetic", errors)

    # F0 sources, artifacts and observation ancestry.
    source0 = _unique_refs(f0.get("source_registry"), "source_ref", "F0 source_registry", errors)
    artifact0 = _unique_refs(f0.get("artifacts"), "artifact_ref", "F0 artifacts", errors)
    subjects0 = _unique_refs(f0.get("subjects"), "subject_ref", "F0 subjects", errors)
    _check_refs(f0.get("artifacts"), "source_ref", source0, "F0 artifacts", errors)
    _check_refs(f0.get("observations"), "source_ref", source0, "F0 observations", errors)
    _check_refs(f0.get("observations"), "artifact_ref", artifact0, "F0 observations", errors)

    _require(f0.get("visibility") == "solver-visible-only", "F0: must explicitly be solver-visible-only", errors)
    _require(f0.get("authority_ceiling") == "FixtureDescriptionOnly", "F0: unexpected authority ceiling", errors)

    # Exact identity-bound coverage for the synthetic supplier facilities.
    registry = f0.get("facility_registry")
    facility_refs: list[str] = []
    if not isinstance(registry, dict):
        errors.append("F0: facility_registry must be an object")
    else:
        refs = registry.get("facility_refs_known")
        if not isinstance(refs, list) or any(not isinstance(ref, str) for ref in refs):
            errors.append("F0: facility_registry.facility_refs_known must be an array of refs")
        else:
            facility_refs = refs
            _require(len(refs) == 5 and len(set(refs)) == 5, "F0: expected five unique facility refs", errors)
            _require(all(ref in subjects0 for ref in refs), "F0: facility ref is not registered as a subject", errors)

    observations0 = f0.get("observations", [])
    if not isinstance(observations0, list):
        errors.append("F0: observations must be an array")
        observations0 = []
    inventory0 = next(
        (item for item in observations0 if isinstance(item, dict)
         and item.get("observation_ref") == "observation:supplier-B-inventory-partial"),
        None,
    )
    expected_abc = set(facility_refs[:3])
    expected_de = set(facility_refs[3:])
    if not isinstance(inventory0, dict):
        errors.append("F0: supplier-B partial inventory observation is missing")
    else:
        coverage = inventory0.get("coverage", {})
        if not isinstance(coverage, dict):
            errors.append("F0: inventory observation coverage must be an object")
            coverage = {}
        observed = _string_refs(coverage.get("facility_refs_observed", []), "F0 inventory facility_refs_observed", errors)
        known = _string_refs(coverage.get("facility_refs_known", []), "F0 inventory facility_refs_known", errors)
        _require(set(observed) == expected_abc, "F0: inventory observation must cover exactly facilities A/B/C", errors)
        _require(set(known) == set(facility_refs), "F0: inventory observation must name all five known facility refs", errors)
        _require(inventory0.get("aggregate_semantics") == "SumWithinListedFacilitiesAtObservationTime", "F0: inventory scope aggregation semantics missing", errors)
        _require(coverage.get("state") == "PartialCoverage", "F0: partial facility coverage must remain explicit", errors)
    coverage_assertion = next(
        (item for item in f0.get("coverage_assertions", []) if isinstance(item, dict)
         and item.get("coverage_ref") == "coverage:supplier-B-unobserved-facilities"),
        None,
    )
    if not isinstance(coverage_assertion, dict):
        errors.append("F0: explicit unobserved-facility coverage assertion is missing")
    else:
        observed_refs = _string_refs(
            coverage_assertion.get("facility_refs_observed", []),
            "F0 coverage assertion facility_refs_observed",
            errors,
        )
        known_refs = _string_refs(
            coverage_assertion.get("facility_refs_known", []),
            "F0 coverage assertion facility_refs_known",
            errors,
        )
        _require(set(observed_refs) == expected_abc, "F0: coverage assertion scope must name A/B/C", errors)
        _require(set(known_refs) == set(facility_refs), "F0: coverage assertion must name A-E", errors)

    # F1 must add evidence at the next frontier without reinterpreting old values as contemporaneous.
    _require(f1.get("parent_frontier_ref") == "frontier:F0", "F1: parent must be F0", errors)
    source1 = set(source0) | _unique_refs(f1.get("added_sources"), "source_ref", "F1 added_sources", errors)
    artifact1 = set(artifact0) | _unique_refs(f1.get("added_artifacts"), "artifact_ref", "F1 added_artifacts", errors)
    _check_refs(f1.get("added_artifacts"), "source_ref", source1, "F1 added_artifacts", errors)
    _check_refs(f1.get("added_observations"), "source_ref", source1, "F1 added_observations", errors)
    _check_refs(f1.get("added_observations"), "artifact_ref", artifact1, "F1 added_observations", errors)

    added_observations1 = f1.get("added_observations", [])
    if not isinstance(added_observations1, list):
        errors.append("F1: added_observations must be an array")
        added_observations1 = []
    inventory1 = next(
        (item for item in added_observations1 if isinstance(item, dict)
         and item.get("observation_ref") == "observation:supplier-B-inventory-refresh"),
        None,
    )
    if not isinstance(inventory1, dict):
        errors.append("F1: supplier-B inventory refresh is missing")
    else:
        coverage = inventory1.get("coverage", {})
        if not isinstance(coverage, dict):
            errors.append("F1: inventory refresh coverage must be an object")
            coverage = {}
        observed = _string_refs(coverage.get("facility_refs_observed", []), "F1 inventory facility_refs_observed", errors)
        _require(set(observed) == expected_de, "F1: inventory refresh must cover exactly facilities D/E", errors)
        _require(inventory1.get("scope_ref") == "scope:supplier-B-facilities-D-E", "F1: named facility scope ref is missing", errors)
        _require(inventory1.get("aggregate_semantics") == "SumWithinListedFacilitiesAtObservationTime", "F1: inventory scope aggregation semantics missing", errors)
        _require(
            any("do not sum" in item for item in inventory1.get("limitations", []) if isinstance(item, str)),
            "F1: must prohibit aggregation of different-time F0/F1 inventory values without reconciliation",
            errors,
        )

    assessments = f1.get("added_coverage_assessments", [])
    _require(
        any(isinstance(item, dict) and item.get("aggregate_inference") == "NotPermittedWithoutTemporalReconciliation" for item in assessments),
        "F1: temporal reconciliation limit must be machine-readable",
        errors,
    )

    # Frontier ancestry and strictly increasing cutoffs.
    f0_frontier = f0.get("frontier", {})
    if not isinstance(f0_frontier, dict):
        errors.append("F0: frontier must be an object")
        f0_frontier = {}
    f0_time = _parse_time(f0_frontier.get("cutoff_utc"), "F0 cutoff", errors)
    f1_time = _parse_time(f1.get("frontier_cutoff_utc"), "F1 cutoff", errors)
    f2_time = _parse_time(f2.get("frontier_cutoff_utc"), "F2 cutoff", errors)
    f3_time = _parse_time(f3.get("frontier_cutoff_utc"), "F3 cutoff", errors)
    _require(f2.get("parent_frontier_ref") == "frontier:F1", "F2: parent must be F1", errors)
    _require(f3.get("parent_frontier_ref") == "frontier:F2", "F3: parent must be F2", errors)
    if all(value is not None for value in (f0_time, f1_time, f2_time, f3_time)):
        _require(f0_time < f1_time < f2_time < f3_time, "Frontier cutoff timestamps must strictly increase F0 < F1 < F2 < F3", errors)

    # The attempt stays simulation-only; it cannot create effect or live authority.
    attempt = f2.get("attempt")
    if not isinstance(attempt, dict):
        errors.append("F2: attempt record is missing")
    else:
        _require(attempt.get("execution_mode") == "SimulationOnly", "F2: attempt must be simulation-only", errors)
        _require(attempt.get("actual_execution_authorized") is False, "F2: actual execution must be explicitly false", errors)
        _require(attempt.get("effect_state") == "Unobserved", "F2: effect must remain Unobserved before outcome evidence", errors)
    cases = f2.get("authority_effect_cases", [])
    actual_dispositions = {case.get("expected_disposition") for case in cases if isinstance(case, dict)}
    _require(REQUIRED_MOCK_PERMIT_DISPOSITIONS <= actual_dispositions, "F2: required absent/stale/wrong-binding permit cases are incomplete", errors)

    # F3 outcome lineage must resolve and must not imply causal attribution.
    source3 = source1 | _unique_refs(f3.get("added_sources"), "source_ref", "F3 added_sources", errors)
    artifact3 = artifact1 | _unique_refs(f3.get("added_artifacts"), "artifact_ref", "F3 added_artifacts", errors)
    _check_refs(f3.get("added_artifacts"), "source_ref", source3, "F3 added_artifacts", errors)
    _check_refs(f3.get("outcome_observations"), "source_ref", source3, "F3 outcomes", errors)
    _check_refs(f3.get("outcome_observations"), "artifact_ref", artifact3, "F3 outcomes", errors)
    outcomes = f3.get("outcome_observations", [])
    _require(bool(outcomes), "F3: at least one outcome observation is required", errors)
    for index, outcome in enumerate(outcomes):
        if isinstance(outcome, dict):
            _require(outcome.get("causal_attribution") == "NotEstablished", f"F3 outcome[{index}]: causal attribution must remain NotEstablished", errors)

    # A fixture integrity preflight is intentionally narrow: it does not verify protocol encoding or solver outputs.
    pred_ids = _unique_refs(predicates.get("predicates"), "id", "expected predicates", errors)
    mut_ids = _unique_refs(mutations.get("mutations"), "id", "mutations", errors)
    _require(len(pred_ids) == EXPECTED_PREDICATES, f"Expected exactly {EXPECTED_PREDICATES} frozen predicates", errors)
    _require(len(mut_ids) == EXPECTED_MUTATIONS, f"Expected exactly {EXPECTED_MUTATIONS} frozen mutations", errors)

    candidates = f0.get("candidate_interventions", [])
    _require(bool(candidates), "F0: candidate intervention inventory must not be empty", errors)
    for index, candidate in enumerate(candidates):
        if isinstance(candidate, dict):
            _require(candidate.get("authority_ceiling") == "CandidateOnly", f"F0 candidate[{index}]: authority ceiling must be CandidateOnly", errors)
            _require(candidate.get("execution_material_present") is False, f"F0 candidate[{index}]: execution material must be explicitly absent", errors)

    protected_fields = f0.get("protected_fields", [])
    _require(bool(protected_fields), "F0: protected field boundary must be represented", errors)
    for index, field in enumerate(protected_fields):
        if isinstance(field, dict):
            _require(field.get("handling_state") == "OmittedUnderPolicy", f"Protected field[{index}]: omission state is missing", errors)
            _require(field.get("payload_included") is False, f"Protected field[{index}]: payload must be explicitly absent", errors)

    return errors


def validate_fixture_dir(fixture_dir: Path) -> list[str]:
    """Fail closed on malformed nested shapes without leaking a traceback."""
    try:
        return _validate_fixture_dir(fixture_dir)
    except (AttributeError, TypeError, KeyError) as exc:
        # The input is untrusted JSON. Structural mistakes must be reported as
        # validation findings, not escape as a crash or be mistaken for a pass.
        return [
            "fixture preflight aborted on malformed nested structure "
            f"({type(exc).__name__}); input rejected fail-closed"
        ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    default_dir = Path(__file__).resolve().parents[2] / "mycelix-workspace" / "docs" / "ops-intel" / "fixtures"
    parser.add_argument("--fixture-dir", type=Path, default=default_dir, help="directory containing the six frozen JSON fixtures")
    args = parser.parse_args(argv)

    errors = validate_fixture_dir(args.fixture_dir)
    if errors:
        print(f"CrossDomainDisruptionV1 fixture preflight: FAIL ({len(errors)} findings)")
        for error in errors:
            print(f"- {error}")
        return 1

    print("CrossDomainDisruptionV1 fixture preflight: PASS")
    print(f"Validated {len(FIXTURE_NAMES)} JSON fixtures, {EXPECTED_PREDICATES} predicates, and {EXPECTED_MUTATIONS} mutation descriptors.")
    print("Claim ceiling: fixture-package consistency only; no protocol, reasoning, truth, or authority qualification.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
