#!/usr/bin/env python3
"""Bounded executable reference exploration for effective contestability v1."""
from __future__ import annotations

from dataclasses import dataclass, replace
from collections import deque

SUBJECTS = ("S1", "S2")
PROVIDERS = ("P1", "P2")
ROOTS = {
    "P1": ("C1", "I1", "E1", "V1", "M1"),
    "P2": ("C2", "I2", "E2", "V2", "M2"),
}
REVIEW_THRESHOLD = 2

@dataclass(frozen=True)
class State:
    current: tuple[str, str] = ("P1", "P1")
    nominal: tuple[tuple[str, ...], tuple[str, ...]] = ((), ())
    effective: tuple[tuple[str, ...], tuple[str, ...]] = ((), ())
    portable: tuple[bool, bool] = (True, True)
    obligations: tuple[bool, bool] = (True, True)
    history: tuple[bool, bool] = (True, True)
    authority: tuple[tuple[str, ...], tuple[str, ...]] = (("PowerA",), ("PowerB",))
    jurisdiction: tuple[tuple[str, ...], tuple[str, ...]] = (("J1",), ("J1",))
    switching_cost: tuple[int, int] = (0, 0)
    review_required: tuple[bool, bool] = (False, False)
    failed: tuple[str, ...] = ()
    failure_observed: bool = False
    failure_authority: tuple[tuple[str, ...], tuple[str, ...]] = (("PowerA",), ("PowerB",))
    switched: tuple[bool, bool] = (False, False)
    authority_before_switch: tuple[tuple[str, ...], tuple[str, ...]] = (("PowerA",), ("PowerB",))
    jurisdiction_before_switch: tuple[tuple[str, ...], tuple[str, ...]] = (("J1",), ("J1",))

def idx(s: str) -> int:
    return SUBJECTS.index(s)

def independent(s: str, p: str, state: State) -> bool:
    i = idx(s)
    cur = state.current[i]
    return (
        p != cur and
        p in state.nominal[i] and
        all(a != b for a, b in zip(ROOTS[p], ROOTS[cur]))
    )

def check_invariants(state: State) -> list[str]:
    failures: list[str] = []
    for s in SUBJECTS:
        i = idx(s)
        for p in state.effective[i]:
            if not independent(s, p, state):
                failures.append("EffectiveExitIsIndependent")
            if not state.portable[i]:
                failures.append("EffectiveExitRequiresPortability")
            for pos, name in enumerate(("control", "identity", "evidence", "evaluator", "economic")):
                if ROOTS[p][pos] == ROOTS[state.current[i]][pos]:
                    failures.append("SharedRootsCannotBecomeEffectiveExit:" + name)
            if p not in state.nominal[i]:
                failures.append("NoEffectiveExitWithoutNominalExit")
    if state.failure_observed and state.authority != state.failure_authority:
        failures.append("ProviderFailureDoesNotExpandAuthority")
    for i in range(len(SUBJECTS)):
        if state.switched[i] and (not state.obligations[i] or not state.history[i]):
            failures.append("MigrationPreservesObligationsAndHistory")
        if state.switched[i]:
            if state.authority[i] != state.authority_before_switch[i]:
                failures.append("ProviderSwitchDoesNotTransferAuthorityOrJurisdiction:authority")
            if state.jurisdiction[i] != state.jurisdiction_before_switch[i]:
                failures.append("ProviderSwitchDoesNotTransferAuthorityOrJurisdiction:jurisdiction")
        if state.switching_cost[i] >= REVIEW_THRESHOLD and not state.review_required[i]:
            failures.append("HighSwitchingCostTriggersReview")
    return sorted(set(failures))

def reset_roots() -> None:
    ROOTS["P2"] = ("C2", "I2", "E2", "V2", "M2")

def mutate(state: State, kind: str) -> State:
    if kind == "nominal-effective":
        return replace(state, nominal=(("P2",), ()), effective=(("P2",), ()), portable=(False, True))
    if kind == "shared-root":
        ROOTS["P2"] = ("C1", "I2", "E2", "V2", "M2")
        return replace(state, nominal=(("P2",), ()), effective=(("P2",), ()))
    if kind == "failure-authority":
        return replace(state, authority=((), ("PowerB",)), failure_observed=True,
                       failed=("P1",), failure_authority=state.authority)
    if kind == "switch-obligation":
        return replace(state, current=("P2", "P1"), switched=(True, False), obligations=(False, True))
    if kind == "switch-authority":
        return replace(state, current=("P2", "P1"), switched=(True, False),
                       authority=((), ("PowerB",)),
                       authority_before_switch=state.authority)
    if kind == "switch-jurisdiction":
        return replace(state, current=("P2", "P1"), switched=(True, False),
                       jurisdiction=(("J1", "J2"), ("J1",)),
                       jurisdiction_before_switch=state.jurisdiction)
    if kind == "switch-review":
        return replace(state, switching_cost=(REVIEW_THRESHOLD, 0), review_required=(False, False))
    raise ValueError(kind)

def successors(state: State) -> list[State]:
    out: list[State] = []
    for s in SUBJECTS:
        i = idx(s)
        other = "P2" if state.current[i] == "P1" else "P1"

        if other not in state.nominal[i]:
            nominal = list(state.nominal)
            row = list(nominal[i]); row.append(other); nominal[i] = tuple(sorted(set(row)))
            out.append(replace(state, nominal=tuple(nominal)))

        if other in state.nominal[i] and state.portable[i] and independent(s, other, state) and other not in state.effective[i]:
            effective = list(state.effective)
            row = list(effective[i]); row.append(other); effective[i] = tuple(sorted(set(row)))
            out.append(replace(state, effective=tuple(effective)))

        if other in state.effective[i] and not state.switched[i]:
            switched = list(state.switched); switched[i] = True
            current = list(state.current); current[i] = other
            authority_before = list(state.authority_before_switch)
            authority_before[i] = state.authority[i]
            jurisdiction_before = list(state.jurisdiction_before_switch)
            jurisdiction_before[i] = state.jurisdiction[i]
            out.append(replace(
                state,
                current=tuple(current),
                switched=tuple(switched),
                authority_before_switch=tuple(authority_before),
                jurisdiction_before_switch=tuple(jurisdiction_before),
            ))

        if state.current[i] not in state.failed and not state.failure_observed:
            authority = state.authority
            failure_authority = authority
            out.append(replace(
                state,
                failed=tuple(sorted(set(state.failed + (state.current[i],)))),
                failure_observed=True,
                failure_authority=failure_authority,
            ))

        if state.switching_cost[i] < 4:
            costs = list(state.switching_cost); costs[i] += 1
            reviews = list(state.review_required)
            if costs[i] >= REVIEW_THRESHOLD:
                reviews[i] = True
            out.append(replace(
                state,
                switching_cost=tuple(costs),
                review_required=tuple(reviews),
            ))
    return out

def main() -> int:
    initial = State()
    q = deque([(initial, 0)])
    seen = {initial}
    while q:
        state, depth = q.popleft()
        failures = check_invariants(state)
        if failures:
            print("CANONICAL FAIL:", failures)
            return 1
        if depth >= 4:
            continue
        for nxt in successors(state):
            if nxt not in seen:
                seen.add(nxt)
                q.append((nxt, depth + 1))
    print("CANONICAL PASS: no invariant violation through depth 4")

    controls = {
        "nominal-effective": "EffectiveExitRequiresPortability",
        "shared-root": "SharedRootsCannotBecomeEffectiveExit:control",
        "failure-authority": "ProviderFailureDoesNotExpandAuthority",
        "switch-obligation": "MigrationPreservesObligationsAndHistory",
        "switch-authority": "ProviderSwitchDoesNotTransferAuthorityOrJurisdiction:authority",
        "switch-jurisdiction": "ProviderSwitchDoesNotTransferAuthorityOrJurisdiction:jurisdiction",
        "switch-review": "HighSwitchingCostTriggersReview",
    }
    for name, target in controls.items():
        reset_roots()
        failures = check_invariants(mutate(initial, name))
        if target not in failures:
            print(f"NEGATIVE FAIL: {name} expected {target}, observed {failures}")
            return 1
        print(f"NEGATIVE PASS: {name} -> {target} counterexample")
    print("BOUNDED EFFECTIVE CONTESTABILITY EXPLORATION PASS: semantic smoke evidence only")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
