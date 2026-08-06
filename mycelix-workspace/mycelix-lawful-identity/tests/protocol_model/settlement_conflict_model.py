#!/usr/bin/env python3
"""Exhaustive model checks for renewable authorization lineages.

This is a protocol model, not Holochain conductor evidence. It asserts the
fail-closed invariants that the real conductor lane must reproduce under
challenge conflicts, credential-status refreshes, policy renewals, lineage
forks, lease expiry, and late emergency revocation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import permutations

EXPIRY = 100
SETTLEMENT = 120
MAX_ANCHOR_VALIDITY = 15 * 60
MAX_POLICY_LEASE = 10 * 60
EXPECTED_REQUEST = "request-A"
EXPECTED_RESULT = "result-A"


def require_model(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


@dataclass(frozen=True)
class Anchor:
    predecessor: str | None
    valid_until: int
    valid: bool = True


@dataclass(frozen=True)
class Decision:
    predecessor: str | None
    anchor: str
    not_after: int
    accepted: bool = True


@dataclass
class Model:
    now: int = 0
    visible_proof_links: list[str] = field(default_factory=list)
    receipt: bool = False
    anchors: dict[str, Anchor] = field(default_factory=dict)
    decisions: dict[str, Decision] = field(default_factory=dict)
    revoked_decisions: set[str] = field(default_factory=set)

    @property
    def visible_proofs(self) -> list[str]:
        return list(dict.fromkeys(self.visible_proof_links))

    @staticmethod
    def lineage_tips(records: dict[str, Anchor | Decision]) -> list[str]:
        predecessors = {
            record.predecessor
            for record in records.values()
            if record.predecessor is not None
        }
        return [record_id for record_id in records if record_id not in predecessors]

    @staticmethod
    def lineage_complete(records: dict[str, Anchor | Decision]) -> bool:
        return all(
            record.predecessor is None or record.predecessor in records
            for record in records.values()
        )

    def submit(self, artifact: str) -> None:
        if self.now >= EXPIRY:
            raise ValueError("artifact submitted after expiry")
        self.visible_proof_links.append(artifact)

    def duplicate_link(self, artifact: str) -> None:
        if artifact not in self.visible_proofs:
            raise ValueError("cannot duplicate a missing proof target")
        self.visible_proof_links.append(artifact)

    def settled_unique(self, artifact: str) -> bool:
        return self.now >= SETTLEMENT and self.visible_proofs == [artifact]

    def record_receipt(self, artifact: str, request_hash: str, result_hash: str) -> bool:
        if not self.settled_unique(artifact):
            return False
        if request_hash != EXPECTED_REQUEST or result_hash != EXPECTED_RESULT:
            return False
        self.receipt = True
        return True

    def record_anchor(
        self,
        artifact: str,
        anchor_id: str,
        predecessor: str | None,
        valid_for: int,
        *,
        force_partition_write: bool = False,
    ) -> bool:
        if not self.settled_unique(artifact):
            return False
        if valid_for <= 0 or valid_for > MAX_ANCHOR_VALIDITY:
            return False
        if not force_partition_write:
            tips = self.lineage_tips(self.anchors)
            expected = None if not self.anchors else tips[0] if len(tips) == 1 else "fork"
            if predecessor != expected:
                return False
        self.anchors[anchor_id] = Anchor(predecessor, self.now + valid_for)
        return True

    def record_policy(
        self,
        artifact: str,
        decision_id: str,
        predecessor: str | None,
        anchor_id: str,
        lease: int,
        *,
        force_partition_write: bool = False,
    ) -> bool:
        if not self.settled_unique(artifact) or not self.receipt:
            return False
        anchor = self.anchors.get(anchor_id)
        if anchor is None or not anchor.valid or self.now >= anchor.valid_until:
            return False
        if self.lineage_tips(self.anchors) != [anchor_id]:
            return False
        if self.revoked_decisions:
            return False
        if lease <= 0 or lease > MAX_POLICY_LEASE or self.now + lease > anchor.valid_until:
            return False
        if not force_partition_write:
            tips = self.lineage_tips(self.decisions)
            expected = None if not self.decisions else tips[0] if len(tips) == 1 else "fork"
            if predecessor != expected:
                return False
        self.decisions[decision_id] = Decision(predecessor, anchor_id, self.now + lease)
        return True

    def revoke_policy(self, decision_id: str) -> bool:
        if decision_id not in self.decisions:
            return False
        self.revoked_decisions.add(decision_id)
        return True

    def effective_state(self, artifact: str) -> str:
        proofs = self.visible_proofs
        if len(proofs) != 1 or proofs[0] != artifact:
            return "ChallengeConsumptionConflict"
        if self.now < SETTLEMENT:
            return "AwaitingSettlement"
        if not self.receipt:
            return "Submitted"
        if not self.lineage_complete(self.anchors):
            return "CredentialAnchorLineageIncomplete"
        anchor_tips = self.lineage_tips(self.anchors)
        if not anchor_tips:
            return "CredentialAnchorsPending"
        if len(anchor_tips) != 1:
            return "CredentialAnchorRefreshFork"
        anchor_id = anchor_tips[0]
        anchor = self.anchors[anchor_id]
        if not anchor.valid:
            return "CredentialAnchorsInvalid"
        if self.now >= anchor.valid_until:
            return "CredentialAnchorsExpired"
        if not self.lineage_complete(self.decisions):
            return "PolicyLineageIncomplete"
        decision_tips = self.lineage_tips(self.decisions)
        if not decision_tips:
            return "VerifierAndCredentialsAttestedValid"
        if len(decision_tips) != 1:
            return "PolicyRenewalFork"
        decision_id = decision_tips[0]
        decision = self.decisions[decision_id]
        if self.revoked_decisions:
            return "PolicyLineageRevoked"
        if decision.anchor != anchor_id:
            return "PolicyRenewalRequired"
        if not decision.accepted:
            return "PolicyRejected"
        if self.now >= decision.not_after:
            return "PolicyExpired"
        return "PolicyAccepted"


def accepted_model(anchor_valid_for: int = 300, lease: int = 120) -> Model:
    model = Model(now=99)
    model.submit("A")
    model.now = SETTLEMENT
    require_model(
        model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT),
        "accepted-model verification receipt setup failed",
    )
    require_model(
        model.record_anchor("A", "anchor-1", None, anchor_valid_for),
        "accepted-model anchor setup failed",
    )
    require_model(
        model.record_policy("A", "decision-1", None, "anchor-1", lease),
        "accepted-model policy setup failed",
    )
    return model


def check_pre_settlement_rejection() -> None:
    model = Model(now=99)
    model.submit("A")
    model.now = 119
    require_model(
        not model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT),
        "pre-settlement verification receipt was accepted",
    )
    require_model(
        not model.record_anchor("A", "anchor-1", None, 300),
        "pre-settlement credential anchor was accepted",
    )
    require_model(
        model.effective_state("A") == "AwaitingSettlement",
        "pre-settlement state did not remain AwaitingSettlement",
    )


def check_duplicate_links_are_not_distinct_consumers() -> None:
    model = Model(now=99)
    model.submit("A")
    model.duplicate_link("A")
    model.now = SETTLEMENT
    require_model(model.visible_proofs == ["A"], "duplicate proof target was not deduplicated")
    require_model(
        model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT),
        "deduplicated challenge did not accept its verification receipt",
    )


def check_transcript_substitution_rejection() -> None:
    model = Model(now=99)
    model.submit("A")
    model.now = SETTLEMENT
    require_model(
        not model.record_receipt("A", "request-B", EXPECTED_RESULT),
        "substituted request hash was accepted",
    )
    require_model(
        not model.record_receipt("A", EXPECTED_REQUEST, "result-B"),
        "substituted result hash was accepted",
    )
    require_model(
        model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT),
        "canonical verification transcript was rejected",
    )


def check_all_competing_submission_orders() -> None:
    for order in permutations(("A", "B")):
        model = Model(now=99)
        for artifact in order:
            model.submit(artifact)
        model.now = SETTLEMENT
        require_model(
            not model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT),
            f"competing submission order {order} accepted a receipt",
        )
        require_model(
            model.effective_state("A") == "ChallengeConsumptionConflict",
            f"competing submission order {order} did not fail closed",
        )


def check_anchor_refresh_and_policy_renewal() -> None:
    model = accepted_model(anchor_valid_for=60, lease=30)
    model.now = SETTLEMENT + 31
    require_model(model.effective_state("A") == "PolicyExpired", "policy did not expire")
    require_model(
        model.record_anchor("A", "anchor-2", "anchor-1", 300),
        "anchor refresh was rejected",
    )
    require_model(
        model.effective_state("A") == "PolicyRenewalRequired",
        "refreshed anchor did not suspend the old policy",
    )
    require_model(
        model.record_policy("A", "decision-2", "decision-1", "anchor-2", 120),
        "policy renewal was rejected",
    )
    require_model(
        model.effective_state("A") == "PolicyAccepted",
        "renewed policy did not become active",
    )


def check_anchor_refresh_fork_fails_closed() -> None:
    model = accepted_model()
    require_model(
        model.record_anchor("A", "anchor-2a", "anchor-1", 300),
        "first anchor refresh was rejected",
    )
    require_model(
        model.record_anchor(
            "A", "anchor-2b", "anchor-1", 300, force_partition_write=True
        ),
        "partitioned anchor refresh was not modeled",
    )
    require_model(
        model.effective_state("A") == "CredentialAnchorRefreshFork",
        "anchor refresh fork did not fail closed",
    )


def check_policy_renewal_fork_fails_closed() -> None:
    model = accepted_model(anchor_valid_for=600)
    require_model(
        model.record_policy("A", "decision-2a", "decision-1", "anchor-1", 120),
        "first policy renewal was rejected",
    )
    require_model(
        model.record_policy(
            "A",
            "decision-2b",
            "decision-1",
            "anchor-1",
            120,
            force_partition_write=True,
        ),
        "partitioned policy renewal was not modeled",
    )
    require_model(
        model.effective_state("A") == "PolicyRenewalFork",
        "policy renewal fork did not fail closed",
    )


def check_ancestor_revocation_terminates_descendant() -> None:
    model = accepted_model(anchor_valid_for=600)
    require_model(
        model.record_policy("A", "decision-2", "decision-1", "anchor-1", 120),
        "descendant policy renewal was rejected",
    )
    require_model(
        model.effective_state("A") == "PolicyAccepted",
        "renewed policy was not active before revocation",
    )
    require_model(model.revoke_policy("decision-1"), "ancestor revocation was rejected")
    require_model(
        model.effective_state("A") == "PolicyLineageRevoked",
        "ancestor revocation did not terminate the lineage",
    )
    require_model(
        not model.record_policy("A", "decision-3", "decision-2", "anchor-1", 120),
        "revoked policy lineage accepted another renewal",
    )


def check_late_gossip_supersedes_every_positive_state() -> None:
    model = accepted_model()
    model.visible_proof_links.append("B")
    require_model(
        model.effective_state("A") == "ChallengeConsumptionConflict",
        "late competing proof did not override the positive state",
    )


def main() -> int:
    check_pre_settlement_rejection()
    check_duplicate_links_are_not_distinct_consumers()
    check_transcript_substitution_rejection()
    check_all_competing_submission_orders()
    check_anchor_refresh_and_policy_renewal()
    check_anchor_refresh_fork_fails_closed()
    check_policy_renewal_fork_fails_closed()
    check_ancestor_revocation_terminates_descendant()
    check_late_gossip_supersedes_every_positive_state()
    print("renewable authorization-lineage protocol model: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
