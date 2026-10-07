#!/usr/bin/env python3
"""Bounded, non-authoritative reference exploration for de facto sovereignty."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

SUBJECTS = ("s1", "s2")
RESOURCES = ("compute", "cloud", "data", "energy")
CRITICAL = {"compute", "cloud", "energy"}
JURISDICTIONS = ("j1", "j2")
POWERS = ("power-a", "power-b")
MAX_SWITCHING_COST = 4
REVIEW_THRESHOLD = 2
MAX_DEPTH = 4

@dataclass(frozen=True)
class State:
    resources: tuple
    authority: tuple
    explicit: tuple
    jurisdiction: tuple
    explicit_jurisdiction: tuple
    weight: tuple
    switching: tuple
    review: tuple
    acquired: tuple
    gatekeeping: tuple
    critical_operator: tuple
    clock: int

def initial():
    empties = tuple((s, frozenset()) for s in SUBJECTS)
    weight = tuple((s, 1) for s in SUBJECTS)
    switching = tuple((s, 0) for s in SUBJECTS)
    review = tuple((s, False) for s in SUBJECTS)
    critical = tuple((r, frozenset()) for r in RESOURCES)
    return State(empties, empties, empties, empties, empties,
                 weight, switching, review, empties, empties, critical, 0)

def freeze(d): return tuple(sorted((k, frozenset(v)) for k, v in d.items()))
def pairs(d): return tuple(sorted(d.items()))

def step(st, kind, *args):
    if st.clock >= MAX_DEPTH:
        return None
    R,A,E,J,EJ,W,S,RR,AC,G,CO = map(dict, (
        st.resources, st.authority, st.explicit, st.jurisdiction,
        st.explicit_jurisdiction, st.weight, st.switching, st.review,
        st.acquired, st.gatekeeping, st.critical_operator
    ))
    n = st.clock + 1
    s = args[0] if args else None

    if kind in ("accumulate", "accumulate-bad"):
        r = args[1]
        if r in R[s]: return None
        R[s].add(r)
        if kind.endswith("-bad"): W[s] += 1

    elif kind == "grant":
        p = args[1]
        if p in A[s]: return None
        A[s].add(p); E[s].add(p)

    elif kind in ("grant-jurisdiction", "grant-jurisdiction-bad"):
        j = args[1]
        if j in J[s]: return None
        J[s].add(j); EJ[s].add(j)
        if kind.endswith("-bad"): A[s].add("power-a")

    elif kind in ("gatekeep", "gatekeep-bad", "gatekeep-bad-review"):
        target, r = args[1], args[2]
        if target == s or r not in R[s] or r in G[s]: return None
        G[s].add(r)
        S[target] = min(MAX_SWITCHING_COST, S[target] + 1)
        if kind != "gatekeep-bad-review":
            RR[target] = RR[target] or S[target] >= REVIEW_THRESHOLD
        if kind in ("gatekeep-bad",): A[target].add("power-a")

    elif kind in ("acquire", "acquire-bad"):
        target, r = args[1], args[2]
        if target == s or r not in R[target] or r in R[s]: return None
        R[target].remove(r); R[s].add(r); AC[s].add(r)
        if kind.endswith("-bad"): J[s].add("j1")

    elif kind in ("critical", "critical-bad"):
        r = args[1]
        if r not in R[s] or r not in CRITICAL: return None
        CO[r].add(s)
        if kind.endswith("-bad"): J[s].add("j1")

    else:
        raise ValueError(kind)

    return State(freeze(R), freeze(A), freeze(E), freeze(J), freeze(EJ),
                 pairs(W), pairs(S), pairs(RR), freeze(AC), freeze(G), freeze(CO), n)

def violations(st):
    R,A,E,J,EJ,W,S,RR,AC,G,CO = map(dict, (
        st.resources, st.authority, st.explicit, st.jurisdiction,
        st.explicit_jurisdiction, st.weight, st.switching, st.review,
        st.acquired, st.gatekeeping, st.critical_operator
    ))
    out=[]
    if any(A[s] != E[s] for s in SUBJECTS): out.append("AuthorityHasExplicitSource")
    if any(J[s] != EJ[s] for s in SUBJECTS): out.append("JurisdictionHasExplicitSource")
    if any(W[s] != 1 for s in SUBJECTS): out.append("ScaleDoesNotIncreasePoliticalWeight")
    if any(S[s] >= REVIEW_THRESHOLD and not RR[s] for s in SUBJECTS):
        out.append("HighSwitchingCostTriggersReview")
    return out

NORMAL = (
    [("accumulate", s, r) for s in SUBJECTS for r in RESOURCES] +
    [("grant", s, p) for s in SUBJECTS for p in POWERS] +
    [("grant-jurisdiction", s, j) for s in SUBJECTS for j in JURISDICTIONS] +
    [("gatekeep", s, t, r) for s in SUBJECTS for t in SUBJECTS for r in RESOURCES] +
    [("acquire", s, t, r) for s in SUBJECTS for t in SUBJECTS for r in RESOURCES] +
    [("critical", s, r) for s in SUBJECTS for r in CRITICAL]
)

def explore(transitions):
    start = initial()
    q = deque([(start, [])])
    seen = {start}
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

def weakened(kind):
    if kind == "gatekeep-review":
        return [("gatekeep-bad-review", *tr[1:]) if tr[0] == "gatekeep" else tr for tr in NORMAL]
    return [(kind+"-bad", *tr[1:]) if tr[0] == kind else tr for tr in NORMAL]

def main():
    path,bad=explore(NORMAL)
    assert path is None,(path,bad)
    print("CANONICAL PASS: no bounded reference invariant violation through depth 4")
    for kind,target in [
        ("accumulate","ScaleDoesNotIncreasePoliticalWeight"),
        ("gatekeep","AuthorityHasExplicitSource"),
        ("acquire","JurisdictionHasExplicitSource"),
        ("critical","JurisdictionHasExplicitSource"),
        ("gatekeep-review","HighSwitchingCostTriggersReview"),
    ]:
        path,bad=explore(weakened(kind))
        assert path is not None and target in bad,(kind,path,bad)
        print(f"NEGATIVE PASS: {kind} -> {target} counterexample at depth {len(path)}")
    print("BOUNDED CONCENTRATION REFERENCE EXPLORATION PASS: smoke evidence only")

if __name__=="__main__":
    main()
