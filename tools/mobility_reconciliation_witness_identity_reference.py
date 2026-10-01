#!/usr/bin/env python3
"""Independent semantic reference evaluator for MOBILITY-COMMONS-016."""

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path


class Kind(str, Enum):
    EVIDENCE_RECORD = "evidence_record"
    RECONCILIATION_WITNESS = "reconciliation_witness"


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

    def validate(self) -> None:
        self.identity.validate()
        if self.identity.kind is not Kind.RECONCILIATION_WITNESS:
            raise ValueError("witness identity has wrong kind")
        self.left_claim.validate()
        self.right_claim.validate()
        if self.left_claim == self.right_claim:
            raise ValueError("witness requires distinct claims")


def validate_projection(projection_ref: Identity, witness: Witness) -> None:
    witness.validate()
    projection_ref.validate()
    if projection_ref.kind is not Kind.RECONCILIATION_WITNESS:
        raise ValueError("projection reference has wrong kind")
    if projection_ref != witness.identity:
        raise ValueError("projection reference does not identify supplied witness")


def self_test() -> None:
    contract = json.loads(
        Path(__file__).resolve().parents[1]
        .joinpath("docs/mobility/MOBILITY_RECONCILIATION_WITNESS_IDENTITY_V1.json")
        .read_text()
    )
    assert [v["id"] for v in contract["vectors"]] == [f"RWI-{i:03d}" for i in range(1, 9)]
    assert all(set(v) == {"id","scenario","expected","forbidden_inference"} for v in contract["vectors"])

    witness = Witness(
        Identity(Kind.RECONCILIATION_WITNESS, "synthetic", "witness-1"),
        Identity(Kind.EVIDENCE_RECORD, "synthetic", "claim-a"),
        Identity(Kind.EVIDENCE_RECORD, "synthetic", "claim-b"),
    )
    validate_projection(witness.identity, witness)

    for bad in (
        Identity(Kind.RECONCILIATION_WITNESS, "synthetic", "witness-2"),
        witness.left_claim,
        Identity(Kind.EVIDENCE_RECORD, "synthetic", "witness-1"),
        Identity(Kind.RECONCILIATION_WITNESS, "holochain", "uhC0-invalid"),
        Identity(Kind.RECONCILIATION_WITNESS, "synthetic", " "),
    ):
        try:
            validate_projection(bad, witness)
            raise AssertionError("invalid witness reference accepted")
        except ValueError:
            pass

    assert witness.identity != witness.left_claim
    assert witness.identity != witness.right_claim


if __name__ == "__main__":
    self_test()
    print("mobility reconciliation witness identity reference: PASS")
