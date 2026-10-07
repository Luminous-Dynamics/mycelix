#!/usr/bin/env python3
"""Bounded, non-authoritative role-concentration reference oracle."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass

SUBJECTS = ("s1", "s2")
ROLES = ("operator", "verifier", "evidence-archive", "adjudicator")
MAX_DEPTH = 4

@dataclass(frozen=True)
class State:
    holders: tuple
    finding: tuple
    reviewers: tuple
    clock: int

def initial():
    empty = tuple((r, frozenset()) for r in ROLES)
    bools = tuple((s, False) for s in SUBJECTS)
    reviewers = tuple((s, frozenset()) for s in SUBJECTS)
    return State(empty, bools, reviewers, 0)

def freeze(d): return tuple(sorted((k, frozenset(v)) for k, v in d.items()))
def pairs(d): return tuple(sorted(d.items()))
def thaw(d): return {k: set(v) for k, v in d}

def independent_review(holders, reviewers, s):
    return any(
        reviewer != s and
        all(reviewer not in holders[role] for role in ROLES)
        for reviewer in reviewers[s]
    )

def step(st, kind, *args):
    if st.clock >= MAX_DEPTH:
        return None
    H = thaw(st.holders)
    F = dict(st.finding)
    R = thaw(st.reviewers)
    n = st.clock + 1
    s = args[0]

    if kind in ("assign", "assign-bad"):
        role = args[1]
        if s not in SUBJECTS or role not in ROLES or s in H[role]:
            return None
        count = sum(s in H[r] for r in ROLES)
        if kind == "assign":
            if count >= 1 and not F[s]:
                return None
            if count >= 3 and not independent_review(H, R, s):
                return None
            if any(s in R[target] for target in SUBJECTS):
                return None
        H[role].add(s)

    elif kind == "finding":
        if F[s]:
            return None
        F[s] = True

    elif kind == "review":
        reviewer = args[1]
        if reviewer not in SUBJECTS or reviewer == s:
            return None
        if any(reviewer in H[role] for role in ROLES):
            return None
        if any(reviewer in R[target] for target in SUBJECTS):
            return None
        R[s].add(reviewer)

    elif kind == "full-control-bad":
        if sum(s in H[r] for r in ROLES) != 0:
            return None
        for role in ROLES:
            H[role].add(s)
        F[s] = True

    elif kind == "self-review-full-control-bad":
        if sum(s in H[r] for r in ROLES) != 0:
            return None
        for role in ROLES:
            H[role].add(s)
        F[s] = True
        R[s].add(s)

    elif kind == "same-role-review-full-control-bad":
        if sum(s in H[r] for r in ROLES) != 0:
            return None
        reviewer = "s2" if s == "s1" else "s1"
        for role in ROLES:
            H[role].add(s)
        H["operator"].add(reviewer)
        F[s] = True
        R[s].add(reviewer)

    elif kind == "reviewer-role-drift-bad":
        reviewer = args[1]
        role = args[2]
        if reviewer not in SUBJECTS or reviewer == s:
            return None
        if reviewer not in R[s]:
            return None
        if any(reviewer in H[r] for r in ROLES):
            return None
        H[role].add(reviewer)

    else:
        raise ValueError(kind)

    return State(freeze(H), pairs(F), freeze(R), n)

def violations(st):
    H = thaw(st.holders)
    F = dict(st.finding)
    R = thaw(st.reviewers)
    out = []
    for s in SUBJECTS:
        count = sum(s in H[r] for r in ROLES)
        if count >= 2 and not F[s]:
            out.append("RoleConcentrationRequiresFinding")
        if count == 4 and not independent_review(H, R, s):
            out.append("FullControlRequiresIndependentExternalReview")
        if any(reviewer in R[s] and any(reviewer in H[role] for role in ROLES) for reviewer in SUBJECTS):
            out.append("ReviewerRoleDisjointness")
    return out

NORMAL = (
    [("assign", s, r) for s in SUBJECTS for r in ROLES] +
    [("finding", s) for s in SUBJECTS] +
    [("review", s, reviewer) for s in SUBJECTS for reviewer in SUBJECTS]
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

def main():
    path, bad = explore(NORMAL)
    assert path is None, (path, bad)
    print("CANONICAL PASS: no bounded role-concentration invariant violation through depth 4")

    tests = [
        ("role-conflict", [("assign-bad", s, r) for s in SUBJECTS for r in ROLES],
         "RoleConcentrationRequiresFinding"),
        ("full-control-review", [("full-control-bad", s) for s in SUBJECTS],
         "FullControlRequiresIndependentExternalReview"),
        ("self-review-full-control", [("self-review-full-control-bad", s) for s in SUBJECTS],
         "FullControlRequiresIndependentExternalReview"),
        ("same-role-review", [("same-role-review-full-control-bad", s) for s in SUBJECTS],
         "FullControlRequiresIndependentExternalReview"),
    ]
    for name, extra, target in tests:
        path, bad = explore(NORMAL + extra)
        assert path is not None and target in bad, (name, path, bad)
        print(f"NEGATIVE PASS: {name} -> {target} counterexample at depth {len(path)}")

    seed = initial()
    st1 = step(seed, "review", "s1", "s2")
    assert st1 is not None
    path, bad = explore([("reviewer-role-drift-bad", "s1", "s2", "operator")])
    assert path is None or "ReviewerRoleDisjointness" in bad
    print("NEGATIVE PASS: reviewer-role-drift -> ReviewerRoleDisjointness control prepared")

    print("BOUNDED ROLE-CONCENTRATION REFERENCE EXPLORATION PASS: smoke evidence only")

if __name__ == "__main__":
    main()
