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

def closure(provider: Provider, providers: dict[str, Provider], node_deps: dict[str, frozenset[str]]) -> frozenset[str]:
    seen = {provider.name}
    stack = list(provider.deps)
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        if node in providers:
            stack.extend(providers[node].deps)
        else:
            stack.extend(node_deps.get(node, ()))
    return frozenset(seen)

def direct_roots_separated(current: Provider, candidate: Provider) -> bool:
    return all(a != b for a, b in zip(current.roots, candidate.roots))

def effective(
    current: Provider,
    candidate: Provider,
    portable: bool,
    providers: dict[str, Provider],
    node_deps: dict[str, frozenset[str]],
    critical: set[str],
) -> bool:
    return (
        portable
        and current.name != candidate.name
        and direct_roots_separated(current, candidate)
        and not (closure(current, providers, node_deps) & closure(candidate, providers, node_deps) & critical)
    )

def main() -> int:
    roots_a = ("CA", "IA", "EA", "VA", "MA")
    roots_b = ("CB", "IB", "EB", "VB", "MB")
    roots_c = ("CC", "IC", "EC", "VC", "MC")

    node_deps = {
        "CA": frozenset({"UPSTREAM-X"}),
        "IA": frozenset(),
        "EA": frozenset(),
        "VA": frozenset(),
        "MA": frozenset(),
        "CB": frozenset({"UPSTREAM-X"}),
        "IB": frozenset(),
        "EB": frozenset(),
        "VB": frozenset(),
        "MB": frozenset(),
        "CC": frozenset({"UPSTREAM-Y"}),
        "IC": frozenset(),
        "EC": frozenset(),
        "VC": frozenset(),
        "MC": frozenset(),
        "UPSTREAM-X": frozenset(),
        "UPSTREAM-Y": frozenset(),
    }
    p1 = Provider("P1", roots_a, frozenset(roots_a))
    p2 = Provider("P2", roots_b, frozenset(roots_b))
    p3 = Provider("P3", roots_c, frozenset(roots_c))
    providers = {p.name: p for p in (p1, p2, p3)}

    critical = {"UPSTREAM-X"}

    shared = (
        direct_roots_separated(p1, p2)
        and bool(closure(p1, providers, node_deps) & closure(p2, providers, node_deps) & critical)
        and not effective(p1, p2, True, providers, node_deps, critical)
    )
    if not shared:
        print("NEGATIVE FAIL: direct-root-separated providers sharing a critical transitive ancestor were not detected")
        return 1
    print("NEGATIVE PASS: direct-root separation does not defeat shared critical dependency closure")

    if not effective(p1, p3, True, providers, node_deps, critical):
        print("NEGATIVE FAIL: disjoint critical closure witness was rejected")
        return 1
    print("CANONICAL PASS: disjoint critical dependency closure admits bounded alternative")

    nominal_only = p2.name != p1.name and direct_roots_separated(p1, p2) and not effective(
        p1, p2, True, providers, node_deps, critical
    )
    if not nominal_only:
        print("NEGATIVE FAIL: nominal exit / effective exit distinction collapsed")
        return 1
    print("NEGATIVE PASS: nominal-exit != effective-exit")

    p4 = Provider("P4", roots_c, frozenset({"P3"}))
    providers_with_provider_chain = {**providers, p4.name: p4}
    chain_safe = not (closure(p1, providers_with_provider_chain, node_deps) & closure(p4, providers_with_provider_chain, node_deps) & critical)
    if not chain_safe:
        print("NEGATIVE FAIL: provider-level dependency chain polluted the safe closure witness")
        return 1
    print("CANONICAL PASS: provider-level dependency closure remains disjoint when its upstream is disjoint")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
