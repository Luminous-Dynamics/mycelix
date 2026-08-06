#!/usr/bin/env python3
"""Deterministic verifier-rollout lifecycle simulation for CI evidence.

This models protocol state, not production cryptography. It exercises the same
fail-closed invariants as runtime protocol v10: zero-mismatch compatibility,
deterministic canary cohorts, monotonic transitions, checkpoint continuity,
and no candidate acceptance during shadow or frozen states.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, replace

TOTAL_BASIS_POINTS = 10_000
MIN_CASES = 128
MAX_WINDOW_SECONDS = 30 * 24 * 60 * 60


@dataclass(frozen=True)
class CompatibilityEvidence:
    predecessor: str
    candidate: str
    corpus_hash: str
    valid_cases: int
    invalid_cases: int
    boundary_cases: int
    mismatches: int

    def digest(self) -> str:
        return digest("mycelix.proof.verifier-compatibility-evidence.v1", self.__dict__)

    def valid(self) -> bool:
        return (
            self.predecessor != self.candidate
            and self.valid_cases > 0
            and self.invalid_cases > 0
            and self.boundary_cases > 0
            and self.valid_cases + self.invalid_cases + self.boundary_cases >= MIN_CASES
            and self.mismatches == 0
        )


@dataclass(frozen=True)
class RolloutPlan:
    predecessor: str
    candidate: str
    evidence_hash: str
    cohort_seed: str
    canary_basis_points: int
    window_start: int
    window_end: int

    def digest(self) -> str:
        return digest("mycelix.proof.verifier-rollout.plan.v1", self.__dict__)

    def valid(self, evidence: CompatibilityEvidence) -> bool:
        return (
            evidence.valid()
            and self.predecessor == evidence.predecessor
            and self.candidate == evidence.candidate
            and self.evidence_hash == evidence.digest()
            and 0 < self.canary_basis_points < TOTAL_BASIS_POINTS
            and 0 < self.window_end - self.window_start <= MAX_WINDOW_SECONDS
        )

    def subject_is_canary(self, subject_hash: str) -> bool:
        material = bytes.fromhex(self.cohort_seed) + bytes.fromhex(subject_hash)
        bucket = int.from_bytes(hashlib.sha256(material).digest()[:8], "little") % TOTAL_BASIS_POINTS
        return bucket < self.canary_basis_points


@dataclass(frozen=True)
class State:
    epoch: int
    stage: str
    effective_at: int
    previous_hash: str | None

    def digest(self, plan_hash: str) -> str:
        return digest(
            "mycelix.proof.verifier-rollout.state.v1",
            {
                "plan_hash": plan_hash,
                "epoch": self.epoch,
                "stage": self.stage,
                "effective_at": self.effective_at,
                "previous_hash": self.previous_hash,
            },
        )


ALLOWED = {
    ("disabled", "shadow"),
    ("shadow", "canary"),
    ("shadow", "rolled_back"),
    ("canary", "general"),
    ("canary", "frozen"),
    ("canary", "rolled_back"),
    ("general", "frozen"),
    ("general", "rolled_back"),
    ("general", "retired"),
    ("frozen", "rolled_back"),
}


def digest(domain: str, value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(domain.encode() + b"\0" + payload).hexdigest()


def valid_successor(plan: RolloutPlan, current: State, successor: State) -> bool:
    if (current.stage, successor.stage) not in ALLOWED:
        return False
    if successor.epoch != current.epoch + 1:
        return False
    if successor.previous_hash != current.digest(plan.digest()):
        return False
    if successor.effective_at <= current.effective_at:
        return False
    if successor.stage == "general" and successor.effective_at < plan.window_start:
        return False
    if successor.stage == "retired" and successor.effective_at < plan.window_end:
        return False
    return True


def decision(plan: RolloutPlan, state: State, subject_hash: str, now: int, common_ready: bool) -> dict[str, bool]:
    canary = plan.subject_is_canary(subject_hash)
    candidate_shadow = common_ready and state.stage in {"shadow", "canary", "general", "retired"}
    candidate_accept = False
    predecessor_accept = False

    if state.stage == "shadow":
        predecessor_accept = True
    elif state.stage == "canary":
        candidate_accept = common_ready and canary and plan.window_start <= now < plan.window_end
        predecessor_accept = not candidate_accept
    elif state.stage == "general":
        candidate_accept = common_ready and now >= plan.window_start
        predecessor_accept = now < plan.window_end
    elif state.stage == "rolled_back":
        predecessor_accept = True
    elif state.stage == "retired":
        candidate_accept = common_ready and now >= plan.window_end

    return {
        "candidate_shadow": candidate_shadow,
        "candidate_accept": candidate_accept,
        "predecessor_accept": predecessor_accept,
    }


def expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def find_subject(plan: RolloutPlan, should_be_canary: bool) -> str:
    for value in range(1, 100_000):
        subject = value.to_bytes(32, "little").hex()
        if plan.subject_is_canary(subject) is should_be_canary:
            return subject
    raise AssertionError("could not find deterministic cohort fixture")


def main() -> None:
    evidence = CompatibilityEvidence(
        predecessor="11" * 32,
        candidate="22" * 32,
        corpus_hash="33" * 32,
        valid_cases=64,
        invalid_cases=48,
        boundary_cases=32,
        mismatches=0,
    )
    plan = RolloutPlan(
        predecessor=evidence.predecessor,
        candidate=evidence.candidate,
        evidence_hash=evidence.digest(),
        cohort_seed="44" * 32,
        canary_basis_points=500,
        window_start=1_000,
        window_end=2_000,
    )
    expect(plan.valid(evidence), "zero-mismatch, category-complete evidence must validate")
    expect(not plan.valid(replace(evidence, mismatches=1)), "one differential mismatch must block rollout")
    expect(not plan.valid(replace(evidence, boundary_cases=0)), "missing boundary corpus must block rollout")
    expect(
        not replace(plan, canary_basis_points=TOTAL_BASIS_POINTS).valid(evidence),
        "a canary may not silently become general rollout",
    )

    plan_hash = plan.digest()
    disabled = State(1, "disabled", 900, None)
    shadow = State(2, "shadow", 950, disabled.digest(plan_hash))
    canary = State(3, "canary", 1_000, shadow.digest(plan_hash))
    general = State(4, "general", 1_200, canary.digest(plan_hash))
    retired = State(5, "retired", 2_000, general.digest(plan_hash))

    expect(valid_successor(plan, disabled, shadow), "disabled must enter shadow first")
    expect(valid_successor(plan, shadow, canary), "shadow may enter canary")
    expect(valid_successor(plan, canary, general), "canary may enter general within the window")
    expect(valid_successor(plan, general, retired), "predecessor may retire only at window end")
    expect(
        not valid_successor(plan, disabled, replace(canary, epoch=2, previous_hash=disabled.digest(plan_hash))),
        "rollout may not skip shadow",
    )
    expect(
        not valid_successor(plan, general, replace(retired, effective_at=plan.window_end - 1)),
        "predecessor may not retire early",
    )
    expect(
        not valid_successor(plan, shadow, replace(canary, previous_hash="ff" * 32)),
        "state chain must match the exact predecessor hash",
    )

    canary_subject = find_subject(plan, True)
    ordinary_subject = find_subject(plan, False)
    expect(
        decision(plan, shadow, canary_subject, 1_000, True)
        == {"candidate_shadow": True, "candidate_accept": False, "predecessor_accept": True},
        "shadow results must never affect acceptance",
    )
    expect(
        decision(plan, canary, canary_subject, 1_100, True)["candidate_accept"],
        "deterministic canary may use candidate inside the compatibility window",
    )
    expect(
        not decision(plan, canary, ordinary_subject, 1_100, True)["candidate_accept"],
        "subjects outside the deterministic cohort must remain on predecessor",
    )
    expect(
        not decision(plan, canary, canary_subject, 1_100, False)["candidate_accept"],
        "missing pins, evidence, authentication, or checkpoint must block candidate",
    )
    frozen = State(4, "frozen", 1_100, canary.digest(plan_hash))
    expect(valid_successor(plan, canary, frozen), "canary may freeze fail-closed")
    expect(
        decision(plan, frozen, canary_subject, 1_100, True)
        == {"candidate_shadow": False, "candidate_accept": False, "predecessor_accept": False},
        "frozen rollout must authorize neither verifier",
    )

    output = {
        "protocol": "mycelix.proof.verifier-rollout.state.v1",
        "plan_hash": plan_hash,
        "evidence_hash": evidence.digest(),
        "canary_basis_points": plan.canary_basis_points,
        "canary_subject_hash": canary_subject,
        "ordinary_subject_hash": ordinary_subject,
        "validated_path": [disabled.stage, shadow.stage, canary.stage, general.stage, retired.stage],
        "negative_cases": [
            "differential_mismatch",
            "missing_boundary_cases",
            "full_canary_allocation",
            "skipped_shadow",
            "early_retirement",
            "unlinked_state",
            "unauthenticated_candidate",
            "frozen_acceptance",
        ],
        "acceptance_enabled": False,
    }
    print(json.dumps(output, sort_keys=True))


if __name__ == "__main__":
    main()
