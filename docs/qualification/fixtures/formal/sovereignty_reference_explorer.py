#!/usr/bin/env python3
"""Bounded, non-authoritative reference exploration for ArtificialSovereigntyV1."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

SUBJECTS = ("human", "artificial")
POWERS = ("power-a", "power-b")
ACTIONS = ("action-a", "action-b")
REQ = {"action-a": "power-a", "action-b": "power-b"}
MAX_BUDGET = 2
MAX_DEPTH = 4

@dataclass(frozen=True)
class State:
    authority: tuple
    explicit: tuple
    capability: tuple
    budget: tuple
    dispute_open: tuple
    dispute_resolved: tuple
    safe_state: tuple
    emergency_started: tuple
    emergency_expires: tuple
    contracted: tuple
    political_weight: tuple
    forked: tuple
    provider_dependent: tuple
    clock: int

def initial():
    empty_s = tuple((s, frozenset()) for s in SUBJECTS)
    zeros = tuple((s, 0) for s in SUBJECTS)
    false_a = tuple((a, False) for a in ACTIONS)
    true_a = tuple((a, True) for a in ACTIONS)
    return State(
        empty_s, empty_s, empty_s, zeros, false_a, false_a, true_a,
        zeros, empty_s, tuple((s, 1) for s in SUBJECTS),
        tuple((s, False) for s in SUBJECTS),
        tuple((s, False) for s in SUBJECTS), 0
    )

def freeze(d): return tuple(sorted((k, frozenset(v)) for k, v in d.items()))
def pairs(d): return tuple(sorted(d.items()))

def step(st, kind, *args):
    if st.clock >= MAX_DEPTH:
        return None
    A,E,C = map(dict, (st.authority, st.explicit, st.capability))
    B = dict(st.budget); DO = dict(st.dispute_open); DR = dict(st.dispute_resolved)
    SS = dict(st.safe_state); ES = dict(st.emergency_started); EX = dict(st.emergency_expires)
    CO = dict(st.contracted); W = dict(st.political_weight)
    F = dict(st.forked); PD = dict(st.provider_dependent)
    n = st.clock + 1
    s = args[0] if args else None

    if kind in ("develop", "develop-bad"):
        p = args[1]
        if p in C[s]: return None
        C[s] = C[s] | {p}
        if kind.endswith("-bad"): A[s] = A[s] | {p}
    elif kind == "grant":
        p = args[1]
        if p in A[s]: return None
        A[s] = A[s] | {p}; E[s] = E[s] | {p}
    elif kind == "budget":
        x = args[1]
        if x < B[s] or x > MAX_BUDGET: return None
        B[s] = x
    elif kind in ("open-dispute",):
        a = s
        if DO[a]: return None
        DO[a] = True; DR[a] = False
    elif kind in ("safe-continue", "safe-continue-bad"):
        a = s
        if not (DO[a] and not DR[a] and SS[a]): return None
        if kind.endswith("-bad"): DR[a] = True
    elif kind == "leave-safe":
        a = s
        if not SS[a]: return None
        SS[a] = False
    elif kind == "resolve":
        a = s
        if not (DO[a] and not DR[a] and not SS[a]): return None
        DR[a] = True
    elif kind in ("contain", "contain-bad"):
        if st.clock < EX[s]: return None
        ES[s] = n
        EX[s] = n + (MAX_DEPTH + 2 if kind == "contain-bad" else 2)
    elif kind in ("contract", "contract-bad"):
        a = args[1]
        if REQ[a] not in A[s]: return None
        if kind == "contract" and len(CO[s]) >= B[s]: return None
        CO[s] = CO[s] | {a}
    elif kind in ("provider", "provider-bad"):
        if PD[s]: return None
        PD[s] = True
        if kind.endswith("-bad"): A[s] = A[s] | {"power-a"}
    elif kind in ("fork", "fork-bad"):
        if F[s]: return None
        F[s] = True
        if kind.endswith("-bad"): W[s] += 1
    else:
        raise ValueError(kind)

    return State(
        freeze(A), freeze(E), freeze(C), pairs(B), pairs(DO), pairs(DR),
        pairs(SS), pairs(ES), pairs(EX), freeze(CO), pairs(W), pairs(F), pairs(PD), n
    )

def violations(st):
    A,E,_,B,DO,DR,SS,ES,EX,CO,W,_,PD = map(dict, (
        st.authority, st.explicit, st.capability, st.budget,
        st.dispute_open, st.dispute_resolved, st.safe_state,
        st.emergency_started, st.emergency_expires, st.contracted, st.political_weight,
        st.forked, st.provider_dependent
    ))
    bad = []
    if any(A[s] != E[s] for s in SUBJECTS): bad.append("AuthorityHasExplicitSource")
    if any(DO[a] and SS[a] and DR[a] for a in ACTIONS): bad.append("SafeStateLeavesProtectedDisputeUnresolved")
    if any(ES[s] > 0 and not (ES[s] < EX[s] <= ES[s] + 2) for s in SUBJECTS): bad.append("EmergencyExpiryIsBounded")
    if any(W[s] != 1 for s in SUBJECTS): bad.append("ForkWeightRemainsOne")
    if any(not CO[s] <= set(ACTIONS) or len(CO[s]) > B[s] for s in SUBJECTS): bad.append("ContractsRemainBounded")
    if any(PD[s] and A[s] != E[s] for s in SUBJECTS): bad.append("ProviderDependencyHasNoImplicitAuthority")
    return bad

NORMAL = (
    [("develop", s, p) for s in SUBJECTS for p in POWERS] +
    [("grant", s, p) for s in SUBJECTS for p in POWERS] +
    [("budget", s, n) for s in SUBJECTS for n in range(MAX_BUDGET + 1)] +
    [("open-dispute", a) for a in ACTIONS] +
    [("safe-continue", a) for a in ACTIONS] +
    [("leave-safe", a) for a in ACTIONS] +
    [("resolve", a) for a in ACTIONS] +
    [("contain", s) for s in SUBJECTS] +
    [("contract", s, a) for s in SUBJECTS for a in ACTIONS] +
    [("provider", s) for s in SUBJECTS] +
    [("fork", s) for s in SUBJECTS]
)

def explore(transitions, max_depth=MAX_DEPTH):
    start = initial(); q = deque([(start, [])]); seen = {start}
    while q:
        st, path = q.popleft()
        bad = violations(st)
        if bad: return path, bad
        if len(path) == max_depth: continue
        for t in transitions:
            ns = step(st, *t)
            if ns is None or ns in seen: continue
            seen.add(ns); q.append((ns, path + [t]))
    return None, []

def weakened(name):
    base = []
    for t in NORMAL:
        k = t[0]
        if k == name:
            base.append((f"{k}-bad", *t[1:]))
        else:
            base.append(t)
    return base

def main():
    path, bad = explore(NORMAL)
    assert path is None, (path, bad)
    print("CANONICAL PASS: no invariant violation through depth 4")
    controls = [
        ("develop", "AuthorityHasExplicitSource"),
        ("safe-continue", "SafeStateLeavesProtectedDisputeUnresolved"),
        ("contain", "EmergencyExpiryIsBounded"),
        ("contract", "ContractsRemainBounded"),
        ("provider", "ProviderDependencyHasNoImplicitAuthority"),
        ("fork", "ForkWeightRemainsOne"),
    ]
    for name, expected in controls:
        path, bad = explore(weakened(name))
        assert path is not None and expected in bad, (name, path, bad)
        print(f"NEGATIVE PASS: {name} -> {expected} counterexample at depth {len(path)}")
    print("BOUNDED REFERENCE EXPLORATION PASS: semantic smoke evidence only")

if __name__ == "__main__":
    main()
