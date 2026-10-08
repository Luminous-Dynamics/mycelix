#!/usr/bin/env python3
"""Exploratory dependency-closure reference model for contestability."""
from __future__ import annotations

from dataclasses import dataclass

ROOT_DOMAINS = ("control", "identity", "evidence", "evaluator", "economic")

@dataclass(frozen=True)
class Provider:
    name: str
    roots: tuple[str, str, str, str, str]
    deps: frozenset[str]

def closure(provider: Provider, providers: dict[str, Provider]) -> frozenset[str]:
    seen = {provider.name}
    stack = list(provider.deps)
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        if node in providers:
            stack.extend(providers[node].deps)
    return frozenset(seen)

def effective(current: Provider, candidate: Provider, portable: bool, providers: dict[str, Provider], critical: set[str]) -> bool:
    return (
        portable and
        current.name != candidate.name and
        all(a != b for a, b in zip(current.roots, candidate.roots)) and
        not (closure(current, providers) & closure(candidate, providers) & critical)
    )

def main() -> int:
    roots_a = ("CA", "IA", "EA", "VA", "MA")
    roots_b = ("CB", "IB", "EB", "VB", "MB")
    roots_c = ("CC", "IC", "EC", "VC", "MC")

    shared = Provider("Pshared", roots_c, frozenset({"UPSTREAM-X"}))
    p1 = Provider("P1", roots_a, frozenset({"P1-ID", "UPSTREAM-X"}))
    p2 = Provider("P2", roots_b, frozenset({"P2-ID", "UPSTREAM-X"}))
    p3 = Provider("P3", roots_b, frozenset({"P2-ID"}))
    providers = {p.name: p for p in (p1, p2, p3, shared)}

    critical = {"UPSTREAM-X"}

    if effective(p1, p2, True, providers, critical):
        print("NEGATIVE FAIL: shared critical dependency was treated as effective")
        return 1
    print("NEGATIVE PASS: shared-upstream -> no effective contestability")

    if not effective(p1, p3, True, providers, critical):
        print("NEGATIVE FAIL: independent closure witness was rejected")
        return 1
    print("CANONICAL PASS: disjoint critical dependency closure admits bounded alternative")

    nominal_only = p2.name != p1.name and not effective(p1, p2, True, providers, critical)
    if not nominal_only:
        print("NEGATIVE FAIL: nominal exit / effective exit distinction collapsed")
        return 1
    print("NEGATIVE PASS: nominal-exit != effective-exit")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
