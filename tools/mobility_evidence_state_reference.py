#!/usr/bin/env python3
"""Independent semantic validator for Mobility Evidence State Tuple V1."""

from dataclasses import dataclass
from enum import Enum


class Epistemic(str, Enum):
    SUPPORTED = "supported"
    CONTRADICTED = "contradicted"
    UNRESOLVED = "unresolved"
    INDETERMINATE = "indeterminate"


class Lifecycle(str, Enum):
    CURRENT = "current"
    SUPERSEDED = "superseded"
    RETIRED = "retired"


class Conflict(str, Enum):
    UNCONTESTED = "uncontested"
    DISPUTED = "disputed"


class Authority(str, Enum):
    COMMONS = "commons"
    EXTERNAL = "external"


class Modality(str, Enum):
    OBSERVATION = "observation"
    MEASUREMENT = "measurement"
    PREDICTION = "prediction"
    SIMULATION = "simulation"
    INTERPRETATION = "interpretation"
    ATTESTATION = "attestation"


@dataclass(frozen=True)
class EvidenceState:
    epistemic: Epistemic
    lifecycle: Lifecycle
    conflict: Conflict
    authority: Authority
    modality: Modality
    contradiction_reference: str | None = None
    unresolved_dependency_reference: str | None = None
    external_authority_reference: str | None = None

    def validate(self) -> None:
        if self.epistemic is Epistemic.CONTRADICTED and not self.contradiction_reference:
            raise ValueError("contradicted requires contradiction reference")
        if self.epistemic is Epistemic.UNRESOLVED and not self.unresolved_dependency_reference:
            raise ValueError("unresolved requires dependency reference")
        if self.conflict is Conflict.DISPUTED and not self.contradiction_reference:
            raise ValueError("disputed requires conflict reference")
        if self.authority is Authority.EXTERNAL and not self.external_authority_reference:
            raise ValueError("external authority requires external-authority reference")
        if self.authority is Authority.COMMONS and self.external_authority_reference:
            raise ValueError("commons cannot silently claim external authority")


def expect_valid(state: EvidenceState) -> None:
    state.validate()


def expect_invalid(state: EvidenceState) -> None:
    try:
        state.validate()
    except ValueError:
        return
    raise AssertionError("invalid state was accepted")


def main() -> None:
    base = EvidenceState(
        Epistemic.SUPPORTED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.COMMONS, Modality.MEASUREMENT,
    )
    expect_valid(base)

    expect_invalid(EvidenceState(
        Epistemic.UNRESOLVED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.COMMONS, Modality.OBSERVATION,
    ))
    expect_valid(EvidenceState(
        Epistemic.UNRESOLVED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.COMMONS, Modality.OBSERVATION,
        unresolved_dependency_reference="dependency-1",
    ))

    expect_invalid(EvidenceState(
        Epistemic.CONTRADICTED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.COMMONS, Modality.MEASUREMENT,
    ))
    expect_valid(EvidenceState(
        Epistemic.CONTRADICTED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.COMMONS, Modality.MEASUREMENT,
        contradiction_reference="evidence-2",
    ))

    # Dispute does not collapse supported evidence into contradiction.
    expect_invalid(EvidenceState(
        Epistemic.SUPPORTED, Lifecycle.CURRENT, Conflict.DISPUTED,
        Authority.COMMONS, Modality.MEASUREMENT,
    ))
    disputed = EvidenceState(
        Epistemic.SUPPORTED, Lifecycle.CURRENT, Conflict.DISPUTED,
        Authority.COMMONS, Modality.MEASUREMENT,
        contradiction_reference="claim-2",
    )
    expect_valid(disputed)
    assert disputed.epistemic is Epistemic.SUPPORTED

    expect_invalid(EvidenceState(
        Epistemic.SUPPORTED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.EXTERNAL, Modality.ATTESTATION,
    ))
    expect_valid(EvidenceState(
        Epistemic.SUPPORTED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.EXTERNAL, Modality.ATTESTATION,
        external_authority_reference="authority-1",
    ))

    # Modality remains independent of epistemic disposition.
    simulation = EvidenceState(
        Epistemic.SUPPORTED, Lifecycle.CURRENT, Conflict.UNCONTESTED,
        Authority.COMMONS, Modality.SIMULATION,
    )
    expect_valid(simulation)
    assert simulation.modality is Modality.SIMULATION

    for lifecycle in (Lifecycle.SUPERSEDED, Lifecycle.RETIRED):
        expect_valid(EvidenceState(
            Epistemic.SUPPORTED, lifecycle, Conflict.UNCONTESTED,
            Authority.COMMONS, Modality.MEASUREMENT,
        ))

    print("evidence-state reference corpus: PASS")


if __name__ == "__main__":
    main()
