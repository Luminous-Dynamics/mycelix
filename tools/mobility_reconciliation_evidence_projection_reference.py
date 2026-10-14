#!/usr/bin/env python3
"""Independent reference evaluator for MOBILITY-COMMONS-015."""

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path


class Classification(str, Enum):
    COEXISTENT = "coexistent"
    CONFLICTING = "conflicting"
    SEQUENTIAL = "sequential"
    INCOMPARABLE = "incomparable"
    INDETERMINATE = "indeterminate"
    SUPERSEDED = "superseded"


class Projection(str, Enum):
    CONFLICT_REFERENCE_ONLY = "conflict_reference_only"
    LIFECYCLE_SUPERSESSION = "lifecycle_supersession"
    INDETERMINATE_REFERENCE = "indeterminate_reference"
    NO_EPISTEMIC_PROMOTION = "no_epistemic_promotion"


@dataclass(frozen=True)
class Witness:
    classification: Classification
    disputed: bool = False


@dataclass(frozen=True)
class State:
    epistemic: str = "supported"
    lifecycle: str = "current"
    conflict: str = "uncontested"
    contradiction_reference: str | None = None
    conflict_reference: str | None = None
    unresolved_dependency_reference: str | None = None


def apply(projection: Projection, witness: Witness, state: State, witness_ref: str,
          dependency_ref: str | None = None) -> State:
    if not witness_ref.strip():
        raise ValueError("witness reference required")

    if projection is Projection.CONFLICT_REFERENCE_ONLY:
        if witness.classification is not Classification.CONFLICTING:
            raise ValueError("conflict projection requires conflicting witness")
        return State(state.epistemic, state.lifecycle,
                     "disputed" if witness.disputed else "uncontested",
                     state.contradiction_reference, witness_ref,
                     state.unresolved_dependency_reference)

    if projection is Projection.LIFECYCLE_SUPERSESSION:
        if witness.classification is not Classification.SUPERSEDED:
            raise ValueError("supersession projection requires superseded witness")
        return State(state.epistemic, "superseded", state.conflict,
                     state.contradiction_reference, state.conflict_reference,
                     state.unresolved_dependency_reference)

    if projection is Projection.INDETERMINATE_REFERENCE:
        if witness.classification is not Classification.INDETERMINATE:
            raise ValueError("indeterminate projection requires indeterminate witness")
        if dependency_ref is not None and not dependency_ref.strip():
            raise ValueError("dependency reference cannot be empty")
        return State("unresolved" if dependency_ref else "indeterminate",
                     state.lifecycle, state.conflict,
                     state.contradiction_reference, state.conflict_reference,
                     dependency_ref)

    if witness.classification not in {
        Classification.COEXISTENT,
        Classification.SEQUENTIAL,
        Classification.INCOMPARABLE,
    }:
        raise ValueError("no-promotion projection has incompatible witness")
    return state


def self_test() -> None:
    contract = json.loads(
        Path(__file__).resolve().parents[1]
        .joinpath("docs/mobility/MOBILITY_RECONCILIATION_EVIDENCE_PROJECTION_V1.json")
        .read_text()
    )
    vector_ids = [v["id"] for v in contract["vectors"]]
    assert vector_ids == [f"REP-{i:03d}" for i in range(1, 10)]
    assert all(
        set(v) == {
            "id",
            "scenario",
            "reconciliation_classification",
            "projection",
            "forbidden_inference",
        }
        for v in contract["vectors"]
    )

    base = State()

    disputed = apply(
        Projection.CONFLICT_REFERENCE_ONLY,
        Witness(Classification.CONFLICTING, disputed=True),
        base, "w-conflict",
    )
    assert disputed.conflict == "disputed"
    assert disputed.epistemic == "supported"
    assert disputed.contradiction_reference is None

    try:
        apply(Projection.LIFECYCLE_SUPERSESSION,
              Witness(Classification.CONFLICTING), base, "bad")
        raise AssertionError("conflicting witness incorrectly accepted as supersession")
    except ValueError:
        pass

    superseded = apply(
        Projection.LIFECYCLE_SUPERSESSION,
        Witness(Classification.SUPERSEDED),
        base, "w-superseded",
    )
    assert superseded.lifecycle == "superseded"
    assert superseded.epistemic == "supported"

    indeterminate = apply(
        Projection.INDETERMINATE_REFERENCE,
        Witness(Classification.INDETERMINATE),
        base, "w-indeterminate",
    )
    assert indeterminate.epistemic == "indeterminate"

    unresolved = apply(
        Projection.INDETERMINATE_REFERENCE,
        Witness(Classification.INDETERMINATE),
        base, "w-unresolved", "dep-1",
    )
    assert unresolved.epistemic == "unresolved"
    assert unresolved.unresolved_dependency_reference == "dep-1"

    assert apply(
        Projection.NO_EPISTEMIC_PROMOTION,
        Witness(Classification.COEXISTENT),
        base, "w-coexistent",
    ) == base
    assert apply(
        Projection.NO_EPISTEMIC_PROMOTION,
        Witness(Classification.SEQUENTIAL),
        base, "w-sequential",
    ) == base
    assert apply(
        Projection.NO_EPISTEMIC_PROMOTION,
        Witness(Classification.INCOMPARABLE),
        base, "w-incomparable",
    ) == base

    external = State(
        epistemic="indeterminate",
        lifecycle="current",
        conflict="uncontested",
        conflict_reference=None,
    )
    projected_external = apply(
        Projection.CONFLICT_REFERENCE_ONLY,
        Witness(Classification.CONFLICTING, disputed=False),
        external, "w-external",
    )
    assert projected_external.epistemic == "indeterminate"


if __name__ == "__main__":
    self_test()
    print("mobility reconciliation evidence projection reference: PASS")
