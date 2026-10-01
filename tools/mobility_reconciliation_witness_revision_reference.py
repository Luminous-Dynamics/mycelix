#!/usr/bin/env python3
"""Independent semantic reference evaluator for MOBILITY-COMMONS-017."""

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path


class Kind(str, Enum):
    EVIDENCE_RECORD = "evidence_record"
    RECONCILIATION_WITNESS = "reconciliation_witness"


class Relation(str, Enum):
    SUPERSEDES = "supersedes"


@dataclass(frozen=True)
class Identity:
    kind: Kind
    namespace: str
    id: str

    def validate(self) -> None:
        if not self.namespace.strip() or not self.id.strip():
            raise ValueError("identity requires namespace and id")
        if self.namespace == "holochain":
            raise ValueError("protocol namespace is not native engineering identity")
        if self.id.startswith(("uhC0", "uhCE")):
            raise ValueError("Holochain-shaped identifier is not native engineering identity")


@dataclass(frozen=True)
class Witness:
    identity: Identity
    left_claim: Identity
    right_claim: Identity
    left_applicability: tuple[int | None, int | None]
    right_applicability: tuple[int | None, int | None]
    comparability: str
    compatibility: str
    explicitly_superseded: bool
    disputed: bool
    result: str

    def validate(self) -> None:
        self.identity.validate()
        if self.identity.kind is not Kind.RECONCILIATION_WITNESS:
            raise ValueError("witness identity has wrong kind")
        self.left_claim.validate()
        self.right_claim.validate()
        if self.left_claim == self.right_claim:
            raise ValueError("witness requires distinct claims")


def supersedes(successor: Identity, predecessor: Identity) -> bool:
    successor.validate()
    predecessor.validate()
    return (
        successor.kind is Kind.RECONCILIATION_WITNESS
        and predecessor.kind is Kind.RECONCILIATION_WITNESS
        and successor != predecessor
    )


def validate_transition(previous: Witness, current: Witness) -> None:
    previous.validate()
    current.validate()

    if previous.identity == current.identity:
        if previous == current:
            return
        raise ValueError("same witness identity cannot carry changed payload")

    if not supersedes(current.identity, previous.identity):
        raise ValueError("new witness requires explicit Supersedes lineage")


def validate_projection_binding(projection_ref: Identity, witness: Witness) -> None:
    witness.validate()
    projection_ref.validate()
    if projection_ref != witness.identity:
        raise ValueError("projection is bound to a different witness")


def witness(identity: str) -> Witness:
    return Witness(
        Identity(Kind.RECONCILIATION_WITNESS, "synthetic", identity),
        Identity(Kind.EVIDENCE_RECORD, "synthetic", "claim-a"),
        Identity(Kind.EVIDENCE_RECORD, "synthetic", "claim-b"),
        (0, 10),
        (5, 15),
        "comparable",
        "incompatible",
        False,
        False,
        "conflicting",
    )


def self_test() -> None:
    contract = json.loads(
        Path(__file__).resolve().parents[1]
        .joinpath("docs/mobility/MOBILITY_RECONCILIATION_WITNESS_REVISION_V1.json")
        .read_text()
    )
    assert contract["schema"] == "mobility-reconciliation-witness-revision-v1"
    assert [v["id"] for v in contract["vectors"]] == [f"WRV-{i:03}" for i in range(1, 11)]
    assert all(
        set(v) == {"id", "scenario", "expected", "forbidden_inference"}
        for v in contract["vectors"]
    )

    previous = witness("w1")
    validate_transition(previous, previous)

    changed_claim = Witness(
        previous.identity,
        previous.left_claim,
        Identity(Kind.EVIDENCE_RECORD, "synthetic", "claim-c"),
        previous.left_applicability,
        previous.right_applicability,
        previous.comparability,
        previous.compatibility,
        previous.explicitly_superseded,
        previous.disputed,
        previous.result,
    )
    try:
        validate_transition(previous, changed_claim)
        raise AssertionError("changed claim pair reused witness identity")
    except ValueError:
        pass

    successor = witness("w2")
    changed_successor = Witness(
        successor.identity,
        successor.left_claim,
        successor.right_claim,
        successor.left_applicability,
        (5, 20),
        successor.comparability,
        successor.compatibility,
        successor.explicitly_superseded,
        successor.disputed,
        successor.result,
    )
    validate_transition(previous, changed_successor)

    validate_projection_binding(previous.identity, previous)
    try:
        validate_projection_binding(previous.identity, changed_successor)
        raise AssertionError("successor retargeted predecessor projection")
    except ValueError:
        pass

    assert previous.identity != changed_successor.identity
    assert supersedes(changed_successor.identity, previous.identity)

    bad = Identity(Kind.RECONCILIATION_WITNESS, "holochain", "uhC0-invalid")
    try:
        bad.validate()
        raise AssertionError("Holochain-shaped identity accepted")
    except ValueError:
        pass


if __name__ == "__main__":
    self_test()
    print("mobility reconciliation witness revision reference: PASS")
