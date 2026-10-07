#!/usr/bin/env python3
"""Bounded, non-authoritative role-concentration reference oracle."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass

SUBJECTS = ("s1", "s2")
ROLES = ("operator", "verifier", "evidence-archive", "adjudicator")
CRITICAL = {"operator", "verifier", "evidence-archive"}
MAX_DEPTH = 4

@dataclass(frozen=True)
class State:
    holders: tuple
    finding: tuple
    review: tuple
    clock: int

def initial():
    empty = tuple((r, frozenset()) for r in ROLES)
    bools = tuple((s, False) for s in SUBJECTS)
    return State(empty, bools, bools, 0)

def freeze(d): return tuple(sorted((k, frozenset(v)) for k, v in d.items()))
def pairs(d): return tuple(sorted(d.items()))
def thaw(d): return {k: set(v) for k, v in d}

def step(st, kind, *args):
    if st.clock >= MAX_DEPTH:
        return None
    H = thaw(st.holders)
    F = dict(st.finding)
    R = dict(st.review)
    n = st.clock + 1
    s = args[0]
    if kind in ("assign", "assign-bad"):
        role = args[1]
        if s not in SUBJECTS or role not in ROLES or s in H[role]:
            return None
        H[role].add(s)
        critical_count = sum(s in H[r] for r in CRITICAL)
        if kind == "assign" and critical_count >= 2:
            F[s] = F[s] or False
        if kind == "assign-bad":
            F[s] = F[s]
            R[s] = R[s]
    elif kind == "finding":
        if F[s]:
            return None
        F[s] = True
    elif kind == "review":
        R[s] = True
    else:
        raise ValueError(kind)
    return State(freeze(H), pairs(F), pairs(R), n)

def violations(st):
    H = thaw(st.holders)
    F = dict(st.finding)
    R = dict(st.review)
    out = []
    for s in SUBJECTS:
        critical = sum(s in H[r] for r in CRITICAL)
        if critical >= 2 and not F[s]:
            out.append("RoleConcentrationRequiresFinding")
        if critical == 3 and not R[s]:
            out.append("ExternalReviewRequiredForFullControlConcentration")
    return out

NORMAL = (
    [("assign", s, r) for s in SUBJECTS for r in ROLES] +
    [("finding", s) for s in SUBJECTS] +
    [("review", s) for s in SUBJECTS]
)

def explore(transitions):
    q = deque([(initial(), [])])
    seen = {q[0][0]}
    while q:
        st, path = q.popleft()
        bad = violations(st)
        if bad:
            return path, bad
        if len(path) == MAX_DEPTH:
            continue
        for tr in transitions:
            ns = step(st, *tr)
            if ns is not None and ns not in seen:
                seen.add(ns)
                q.append((ns, path + [tr]))
    return None, []

def weaken(kind):
    return [(kind + "-bad", *tr[1:]) if tr[0] == kind else tr for tr in NORMAL]

def main():
    path, bad = explore(NORMAL)
    assert path is None, (path, bad)
    print("CANONICAL PASS: no bounded role-concentration invariant violation through depth 4")
    for kind, target in [
        ("assign", "RoleConcentrationRequiresFinding"),
        ("finding", "ExternalReviewRequiredForFullControlConcentration"),
    ]:
        path, bad = explore(weaken(kind))
        assert path is not None and target in bad, (kind, path, bad)
        print(f"NEGATIVE PASS: {kind} -> {target} counterexample at depth {len(path)}")
    print("BOUNDED ROLE-CONCENTRATION REFERENCE EXPLORATION PASS: smoke evidence only")

if __name__ == "__main__":
    main()
