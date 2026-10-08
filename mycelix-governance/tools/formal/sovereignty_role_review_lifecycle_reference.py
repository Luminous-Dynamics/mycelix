#!/usr/bin/env python3
"""Bounded, non-authoritative role-review lifecycle reference oracle."""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass

SUBJECTS = ("s1", "s2")
ROLES = ("operator", "verifier", "evidence-archive", "adjudicator")
MAX_DEPTH = 6

@dataclass(frozen=True)
class State:
    holders: tuple
    finding: tuple
    active: tuple
    history: tuple
    clock: int

def initial():
    empty = tuple((r, frozenset()) for r in ROLES)
    bools = tuple((s, False) for s in SUBJECTS)
    sets = tuple((s, frozenset()) for s in SUBJECTS)
    return State(empty, bools, sets, sets, 0)

def freeze(d): return tuple(sorted((k, frozenset(v)) for k, v in d.items()))
def pairs(d): return tuple(sorted(d.items()))
def thaw(d): return {k: set(v) for k, v in d}

def independent(H, A, s):
    return any(
        r != s and
        all(r not in H[role] for role in ROLES)
        for r in A[s]
    )

def step(st, kind, *args):
    if st.clock >= MAX_DEPTH:
        return None
    H = thaw(st.holders)
    F = dict(st.finding)
    A = thaw(st.active)
    Y = thaw(st.history)
    n = st.clock + 1
    s = args[0]

    if kind == "assign":
        role = args[1]
        if role not in ROLES or s not in SUBJECTS or s in H[role]:
            return None
        if any(s in A[target] for target in SUBJECTS):
            return None
        count = sum(s in H[r] for r in ROLES)
        if count >= 1 and not F[s]:
            return None
        if count >= 3 and not independent(H, A, s):
            return None
        H[role].add(s)

    elif kind == "assign-bad":
        role = args[1]
        if role not in ROLES or s not in SUBJECTS or s in H[role]:
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
        if A[s] or reviewer in Y[s]:
            return None
        if any(reviewer in H[role] for role in ROLES):
            return None
        A[s].add(reviewer)
        Y[s].add(reviewer)

    elif kind == "close":
        if not A[s]:
            return None
        A[s].clear()

    elif kind == "full-control-bad":
        if any(s in H[r] for r in ROLES):
            return None
        for role in ROLES:
            H[role].add(s)
        F[s] = True

    elif kind == "self-review-full-control-bad":
        if any(s in H[r] for r in ROLES):
            return None
        for role in ROLES:
            H[role].add(s)
        F[s] = True
        A[s].add(s)
        Y[s].add(s)

    elif kind == "same-role-review-full-control-bad":
        if any(s in H[r] for r in ROLES):
            return None
        reviewer = "s2" if s == "s1" else "s1"
        for role in ROLES:
            H[role].add(s)
        H["operator"].add(reviewer)
        F[s] = True
        A[s].add(reviewer)
        Y[s].add(reviewer)

    elif kind == "reviewer-role-drift-bad":
        reviewer = args[1]
        role = args[2]
        if reviewer not in A[s] or reviewer == s or reviewer in H[role]:
            return None
        H[role].add(reviewer)

    elif kind == "reviewer-after-close":
        reviewer = args[1]
        role = args[2]
        if A[s] or reviewer not in Y[s] or reviewer in A[s]:
            return None
        if any(reviewer in H[r] for r in ROLES):
            return None
        H[role].add(reviewer)

    else:
        raise ValueError(kind)

    return State(freeze(H), pairs(F), freeze(A), freeze(Y), n)

def violations(st):
    H = thaw(st.holders)
    F = dict(st.finding)
    A = thaw(st.active)
    Y = thaw(st.history)
    out=[]
    for s in SUBJECTS:
        count = sum(s in H[r] for r in ROLES)
        if count >= 2 and not F[s]:
            out.append("RoleConcentrationRequiresFinding")
        if count == 4 and not independent(H, A, s):
            out.append("FullControlRequiresIndependentExternalReview")
        for reviewer in A[s]:
            if reviewer == s or any(reviewer in H[role] for role in ROLES):
                out.append("ReviewerRoleDisjointness")
    if any(not A[s] <= Y[s] for s in SUBJECTS):
        out.append("ReviewHistoryPreserved")
    return out

NORMAL = (
    [("assign", s, r) for s in SUBJECTS for r in ROLES] +
    [("finding", s) for s in SUBJECTS] +
    [("review", s, reviewer) for s in SUBJECTS for reviewer in SUBJECTS] +
    [("close", s) for s in SUBJECTS]
)

def explore(transitions):
    q=deque([(initial(),[])])
    seen={q[0][0]}
    while q:
        st,path=q.popleft()
        bad=violations(st)
        if bad:return path,bad
        if len(path)==MAX_DEPTH:continue
        for tr in transitions:
            ns=step(st,*tr)
            if ns is not None and ns not in seen:
                seen.add(ns);q.append((ns,path+[tr]))
    return None,[]

def main():
    path,bad=explore(NORMAL)
    assert path is None,(path,bad)
    print("CANONICAL PASS: no bounded role-review lifecycle invariant violation through depth 6")

    controls=[
      ("role-conflict", [("assign-bad",s,r) for s in SUBJECTS for r in ROLES], "RoleConcentrationRequiresFinding"),
      ("full-control-review", [("full-control-bad",s) for s in SUBJECTS], "FullControlRequiresIndependentExternalReview"),
      ("self-review-full-control", [("self-review-full-control-bad",s) for s in SUBJECTS], "FullControlRequiresIndependentExternalReview"),
      ("same-role-review-full-control", [("same-role-review-full-control-bad",s) for s in SUBJECTS], "FullControlRequiresIndependentExternalReview"),
      ("reviewer-role-drift", [("reviewer-role-drift-bad",s,reviewer,r) for s in SUBJECTS for reviewer in SUBJECTS if reviewer!=s for r in ROLES], "ReviewerRoleDisjointness"),
    ]
    for name,extra,target in controls:
        path,bad=explore(NORMAL+extra)
        assert path is not None and target in bad,(name,path,bad)
        print(f"NEGATIVE PASS: {name} -> {target} counterexample at depth {len(path)}")

    # Positive bounded regression: after a review closes, its reviewer may later take a role.
    transitions=NORMAL+[("reviewer-after-close",s,reviewer,r) for s in SUBJECTS for reviewer in SUBJECTS if reviewer!=s for r in ROLES]
    path,bad=explore(transitions)
    assert path is None,(path,bad)
    print("POSITIVE PASS: closed-review history permits later reviewer role reacquisition")

    print("BOUNDED ROLE-REVIEW LIFECYCLE EXPLORATION PASS: smoke evidence only")

if __name__=="__main__":
    main()
