#!/usr/bin/env python3
"""NCF-ALLOC-6: bounded exhaustive interleaving model for obligation admission.

It demonstrates the gap between a sequential dedup/cap oracle and a durable,
atomic uniqueness+cap transaction. It is a pure model, not a database benchmark.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import hashlib, json
from pathlib import Path

PHASES = ("read_dedup", "read_used", "check", "commit")
CAP = 100

@dataclass(frozen=True)
class Request:
    request_id: str
    work_digest: str
    principal: str = "principal-A"
    instrument: str = "instrument-1"
    amount: int = 60

@dataclass
class Slot:
    seen_snapshot: bool = False
    used_snapshot: int = 0
    eligible: bool = False

@dataclass
class NaiveState:
    seen_work: set[str] = field(default_factory=set)
    seen_request_ids: set[str] = field(default_factory=set)
    used_counter: int = 0
    accepted_events: list[tuple[str, str, int]] = field(default_factory=list)
    slots: dict[str, Slot] = field(default_factory=dict)
    counter_mode: str = "atomic_increment"

    def __post_init__(self):
        if not self.slots:
            self.slots = {"A": Slot(), "B": Slot()}

    def step(self, actor: str, phase: str, request: Request) -> None:
        slot = self.slots[actor]
        if phase == "read_dedup":
            slot.seen_snapshot = request.work_digest in self.seen_work or request.request_id in self.seen_request_ids
        elif phase == "read_used":
            slot.used_snapshot = self.used_counter
        elif phase == "check":
            slot.eligible = (not slot.seen_snapshot) and (slot.used_snapshot + request.amount <= CAP)
        elif phase == "commit":
            if slot.eligible:
                self.accepted_events.append((request.request_id, request.work_digest, request.amount))
                self.seen_work.add(request.work_digest)
                self.seen_request_ids.add(request.request_id)
                if self.counter_mode == "atomic_increment":
                    self.used_counter += request.amount
                elif self.counter_mode == "snapshot_overwrite":
                    self.used_counter = slot.used_snapshot + request.amount
                else:
                    raise ValueError(self.counter_mode)
        else:
            raise ValueError(phase)

def interleavings() -> list[tuple[str, ...]]:
    """All 70 order-preserving interleavings of two four-step client programs."""
    output = []
    def visit(prefix: list[str], used_a: int, used_b: int):
        if used_a == len(PHASES) and used_b == len(PHASES):
            output.append(tuple(prefix)); return
        if used_a < len(PHASES):
            visit(prefix + ["A"], used_a + 1, used_b)
        if used_b < len(PHASES):
            visit(prefix + ["B"], used_a, used_b + 1)
    visit([], 0, 0)
    assert len(output) == 70
    return output

def run_naive(requests: dict[str, Request], schedule: tuple[str, ...], counter_mode: str = "atomic_increment") -> dict:
    state = NaiveState(counter_mode=counter_mode)
    phase_index = {"A": 0, "B": 0}
    for actor in schedule:
        phase = PHASES[phase_index[actor]]
        state.step(actor, phase, requests[actor])
        phase_index[actor] += 1
    actual_accepted_units = sum(amount for _, _, amount in state.accepted_events)
    unique_work_units = sum({digest: amount for _, digest, amount in state.accepted_events}.values())
    return {
        "schedule": "".join(schedule),
        "accepted_event_count": len(state.accepted_events),
        "unique_accepted_work_count": len({digest for _, digest, _ in state.accepted_events}),
        "actual_accepted_units": actual_accepted_units,
        "unique_work_units": unique_work_units,
        "used_counter": state.used_counter,
        "cap": CAP,
        "cap_overshoot_units": max(0, actual_accepted_units - CAP),
        "counter_understates_accepted_units": state.used_counter < actual_accepted_units,
        "duplicate_work_accepted": len({digest for _, digest, _ in state.accepted_events}) < len(state.accepted_events),
        "accepted_events": state.accepted_events,
    }

def run_serializable(requests: dict[str, Request], order: tuple[str, str]) -> dict:
    """Atomic transaction oracle: uniqueness and cap check/debit are one step."""
    seen_request: dict[str, Request] = {}
    seen_work: dict[str, Request] = {}
    used = 0
    accepted: list[tuple[str, str, int]] = []
    decisions = {}
    for actor in order:
        req = requests[actor]
        prior_id = seen_request.get(req.request_id)
        if prior_id is not None:
            same = (prior_id.work_digest, prior_id.principal, prior_id.instrument, prior_id.amount) == (req.work_digest, req.principal, req.instrument, req.amount)
            decisions[actor] = "idempotent_replay" if same else "conflict_rejected"
            continue
        prior_work = seen_work.get(req.work_digest)
        if prior_work is not None:
            if (prior_work.principal, prior_work.instrument, prior_work.amount) != (req.principal, req.instrument, req.amount):
                decisions[actor] = "work_digest_conflict_rejected"
            else:
                seen_request[req.request_id] = prior_work
                decisions[actor] = "idempotent_replay"
            continue
        if used + req.amount > CAP:
            decisions[actor] = "cap_rejected"
            continue
        used += req.amount
        accepted.append((req.request_id, req.work_digest, req.amount))
        seen_request[req.request_id] = req
        seen_work[req.work_digest] = req
        decisions[actor] = "accepted"
    actual = sum(x[2] for x in accepted)
    return {"order": "".join(order), "decisions": decisions,
            "accepted_event_count": len(accepted), "actual_accepted_units": actual,
            "used_counter": used, "cap": CAP, "cap_overshoot_units": max(0, actual-CAP),
            "duplicate_work_accepted": len({x[1] for x in accepted}) < len(accepted)}

def scenario(requests: dict[str, Request], same_underlying_work: bool) -> dict:
    schedules = interleavings()
    modes = {}
    for mode in ("atomic_increment", "snapshot_overwrite"):
        rows = [run_naive(requests, schedule, mode) for schedule in schedules]
        modes[mode] = {
            "schedule_count": len(rows),
            "unsafe_schedules": sum(bool(r["cap_overshoot_units"] or r["duplicate_work_accepted"] or r["counter_understates_accepted_units"]) for r in rows),
            "cap_violation_schedules": sum(r["cap_overshoot_units"] > 0 for r in rows),
            "duplicate_accept_schedules": sum(r["duplicate_work_accepted"] for r in rows),
            "counter_understatement_schedules": sum(r["counter_understates_accepted_units"] for r in rows),
            "worst_overshoot_units": max(r["cap_overshoot_units"] for r in rows),
            "first_unsafe_example": next((r for r in rows if r["cap_overshoot_units"] or r["duplicate_work_accepted"] or r["counter_understates_accepted_units"]), None),
        }
    serial = [run_serializable(requests, order) for order in (("A", "B"), ("B", "A"))]
    return {"same_underlying_work": same_underlying_work, "naive_modes": modes,
            "atomic_serializable_orders": serial,
            "atomic_serializable_orders_with_violation": sum(bool(r["cap_overshoot_units"] or r["duplicate_work_accepted"]) for r in serial),
            "claim_ceiling": "The schedules are a finite abstract two-client model with four admission steps. It reveals a time-of-check/time-of-use hazard; it does not prove a particular database implementation is unsafe or safe."}

def main() -> None:
    same_digest = hashlib.sha256(b"same-canonical-work").hexdigest()
    digest_a = hashlib.sha256(b"work-a").hexdigest()
    digest_b = hashlib.sha256(b"work-b").hexdigest()
    cases = {
        "same_work_different_request_ids": scenario({"A": Request("req-A", same_digest), "B": Request("req-B", same_digest)}, True),
        "distinct_work_contends_for_last_capacity": scenario({"A": Request("req-A", digest_a, amount=60), "B": Request("req-B", digest_b, amount=60)}, False),
        "same_request_id_conflicting_work": scenario({"A": Request("req-shared", digest_a, amount=40), "B": Request("req-shared", digest_b, amount=40)}, False),
    }
    result = {
        "schema": "ncf-alloc6-admission-races-v1",
        "status": "INDEPENDENT_PYTHON_STATE_SPACE_MODEL_NOT_DATABASE_NOT_RUST_NOT_CI_NOT_PRODUCTION",
        "profile": {"clients": 2, "per_client_steps": list(PHASES), "interleavings_per_case": 70, "cap_units": CAP, "request_amount_units": 60},
        "case_results": cases,
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "design_conclusion": "Idempotency, canonical-work uniqueness and cap accounting must linearize in one durable transaction or a correctly proved invariant-confluent protocol. A sequential oracle passing is insufficient.",
        "claim_ceiling": "Finite synthetic scheduler only; does not test threads, disk durability, real databases, network partition behavior, Holochain, signatures or production code."
    }
    print(json.dumps(result, indent=2, sort_keys=True))

if __name__ == "__main__": main()
