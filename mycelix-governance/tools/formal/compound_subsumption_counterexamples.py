#!/usr/bin/env python3
"""Bounded semantic/structural subsumption oracle with replayable counterexamples.

Completeness is claimed only for the finite universe and grammar supplied in a
scenario. Structural witness matching is reported separately from semantic
request-set containment.
"""
from __future__ import annotations
import argparse
import hashlib
import itertools
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCENARIO_SCHEMA = "mycelix.compound-subsumption-counterexample-scenario.v1"
RESULT_SCHEMA = "mycelix.compound-subsumption-counterexample-result.v1"
SUPPORTED_EXTENSION = "none"


class ScenarioError(ValueError):
    pass


@dataclass(frozen=True)
class Request:
    target: str
    purpose: str
    context: str
    amount: int

    def json(self) -> dict[str, Any]:
        return {"target": self.target, "purpose": self.purpose,
                "context": self.context, "amount": self.amount}


@dataclass(frozen=True)
class Universe:
    targets: tuple[str, ...]
    purposes: tuple[str, ...]
    contexts: tuple[str, ...]
    amounts: tuple[int, ...]

    @classmethod
    def parse(cls, raw: dict[str, Any]) -> "Universe":
        def strings(key: str) -> tuple[str, ...]:
            values = raw.get(key)
            if not isinstance(values, list) or not values or any(not isinstance(v, str) or not v for v in values):
                raise ScenarioError(f"universe.{key} must be a non-empty list of strings")
            if len(set(values)) != len(values):
                raise ScenarioError(f"universe.{key} contains duplicates")
            return tuple(values)
        amounts = raw.get("amounts")
        if (not isinstance(amounts, list) or not amounts
                or any(type(v) is not int or v < 0 for v in amounts)
                or len(set(amounts)) != len(amounts)):
            raise ScenarioError("universe.amounts must contain distinct non-negative integers")
        return cls(strings("targets"), strings("purposes"), strings("contexts"), tuple(amounts))

    def requests(self) -> tuple[Request, ...]:
        return tuple(Request(*row) for row in itertools.product(
            self.targets, self.purposes, self.contexts, self.amounts))

    def key(self, request: Request) -> tuple[int, int, int, int]:
        return (self.targets.index(request.target), self.purposes.index(request.purpose),
                self.contexts.index(request.context), self.amounts.index(request.amount))


@dataclass(frozen=True)
class Atom:
    id: str
    target: frozenset[str]
    purpose: frozenset[str]
    context: frozenset[str]
    max_amount: int
    extension: str = SUPPORTED_EXTENSION

    @classmethod
    def parse(cls, raw: dict[str, Any], universe: Universe) -> "Atom":
        atom_id = raw.get("id")
        if not isinstance(atom_id, str) or not atom_id:
            raise ScenarioError("every atom requires a non-empty id")
        def dimension(key: str, allowed: tuple[str, ...]) -> frozenset[str]:
            values = raw.get(key)
            if not isinstance(values, list) or not values:
                raise ScenarioError(f"atom {atom_id}: {key} must be non-empty")
            if any(not isinstance(v, str) or v not in allowed for v in values):
                raise ScenarioError(f"atom {atom_id}: {key} exceeds the declared universe")
            if len(set(values)) != len(values):
                raise ScenarioError(f"atom {atom_id}: {key} contains duplicates")
            return frozenset(values)
        amount = raw.get("max_amount")
        if type(amount) is not int or amount < 0:
            raise ScenarioError(f"atom {atom_id}: max_amount must be a non-negative integer")
        extension = raw.get("extension", SUPPORTED_EXTENSION)
        if not isinstance(extension, str) or not extension:
            raise ScenarioError(f"atom {atom_id}: extension must be a non-empty string")
        return cls(atom_id, dimension("target", universe.targets),
                   dimension("purpose", universe.purposes),
                   dimension("context", universe.contexts), amount, extension)

    def matches(self, request: Request) -> bool:
        return (request.target in self.target and request.purpose in self.purpose
                and request.context in self.context and request.amount <= self.max_amount)

    def mismatch_dimensions(self, request: Request) -> list[str]:
        failures = []
        if request.target not in self.target:
            failures.append("target")
        if request.purpose not in self.purpose:
            failures.append("purpose")
        if request.context not in self.context:
            failures.append("context")
        if request.amount > self.max_amount:
            failures.append("numeric-bound")
        return failures

    def to_json(self) -> dict[str, Any]:
        return {"id": self.id, "target": sorted(self.target), "purpose": sorted(self.purpose),
                "context": sorted(self.context), "max_amount": self.max_amount,
                "extension": self.extension}


@dataclass(frozen=True)
class Compound:
    kind: str
    clauses: tuple[Atom, ...]

    @classmethod
    def parse(cls, raw: dict[str, Any], universe: Universe) -> "Compound":
        kind = raw.get("kind")
        if kind not in {"all", "any"}:
            raise ScenarioError(f"unsupported compound kind: {kind!r}")
        rows = raw.get("clauses")
        if not isinstance(rows, list) or not rows:
            raise ScenarioError(f"{kind} compound must have at least one clause")
        clauses = tuple(Atom.parse(row, universe) for row in rows)
        ids = [atom.id for atom in clauses]
        if len(set(ids)) != len(ids):
            raise ScenarioError(f"{kind} compound contains duplicate clause IDs")
        return cls(kind, clauses)

    def to_json(self) -> dict[str, Any]:
        return {"kind": self.kind, "clauses": [atom.to_json() for atom in self.clauses]}


@dataclass(frozen=True)
class Policy:
    allow: Compound
    deny: tuple[Compound, ...]
    conflict_rule: str

    @classmethod
    def parse(cls, raw: dict[str, Any], universe: Universe) -> "Policy":
        allow = Compound.parse(raw.get("allow", {}), universe)
        rows = raw.get("deny", [])
        if not isinstance(rows, list):
            raise ScenarioError("policy.deny must be a list")
        deny = tuple(Compound.parse(row, universe) for row in rows)
        rule = raw.get("conflict_rule")
        if not isinstance(rule, str) or not rule:
            raise ScenarioError("policy requires explicit conflict_rule")
        return cls(allow, deny, rule)


def denotation(expr: Compound, universe: Universe) -> frozenset[Request]:
    accepted = set()
    for request in universe.requests():
        matches = [atom.matches(request) for atom in expr.clauses]
        admitted = all(matches) if expr.kind == "all" else any(matches)
        if admitted:
            accepted.add(request)
    return frozenset(accepted)


def atom_subsumes(child: Atom, parent: Atom) -> bool:
    if child.extension != SUPPORTED_EXTENSION or parent.extension != SUPPORTED_EXTENSION:
        return False
    return (child.target <= parent.target and child.purpose <= parent.purpose
            and child.context <= parent.context and child.max_amount <= parent.max_amount)


def unsupported_atoms(expressions: list[Compound]) -> list[dict[str, str]]:
    return sorted(
        [{"clause_id": atom.id, "extension": atom.extension}
         for expr in expressions for atom in expr.clauses
         if atom.extension != SUPPORTED_EXTENSION],
        key=lambda item: (item["clause_id"], item["extension"]))


def maximum_injective_matching(parent: Compound, child: Compound) -> dict[str, str]:
    """Deterministic maximum bipartite matching: parent obligation -> child witness."""
    adjacency = {
        pi: [ci for ci, c in enumerate(child.clauses) if atom_subsumes(c, p)]
        for pi, p in enumerate(parent.clauses)
    }
    child_to_parent: dict[int, int] = {}

    def augment(pi: int, seen: set[int]) -> bool:
        for ci in sorted(adjacency[pi], key=lambda index: (child.clauses[index].id, index)):
            if ci in seen:
                continue
            seen.add(ci)
            previous = child_to_parent.get(ci)
            if previous is None or augment(previous, seen):
                child_to_parent[ci] = pi
                return True
        return False

    order = sorted(range(len(parent.clauses)),
                   key=lambda i: (len(adjacency[i]), parent.clauses[i].id, i))
    for pi in order:
        augment(pi, set())
    return dict(sorted({
        parent.clauses[pi].id: child.clauses[ci].id
        for ci, pi in child_to_parent.items()
    }.items()))


def structural_decision(parent: Compound, child: Compound) -> dict[str, Any]:
    if parent.kind != child.kind:
        return {"pass": False, "rule": "unsupported-cross-compound-type",
                "matching": {}, "unmatched_parent_clause_ids": sorted(a.id for a in parent.clauses),
                "unmatched_child_clause_ids": sorted(a.id for a in child.clauses),
                "candidate_edges": []}
    if parent.kind == "all":
        matching = maximum_injective_matching(parent, child)
        matched_parent = set(matching)
        matched_child = set(matching.values())
        edges = [{"parent_clause_id": p.id, "child_clause_id": c.id}
                 for p in sorted(parent.clauses, key=lambda a: a.id)
                 for c in sorted(child.clauses, key=lambda a: a.id) if atom_subsumes(c, p)]
        return {"pass": len(matching) == len(parent.clauses),
                "rule": "injective-parent-obligation-matching", "matching": matching,
                "unmatched_parent_clause_ids": sorted(a.id for a in parent.clauses if a.id not in matched_parent),
                "unmatched_child_clause_ids": sorted(a.id for a in child.clauses if a.id not in matched_child),
                "candidate_edges": edges}
    mapping = {}
    unmatched = []
    for c in sorted(child.clauses, key=lambda a: a.id):
        candidates = sorted((p for p in parent.clauses if atom_subsumes(c, p)), key=lambda a: a.id)
        if candidates:
            mapping[c.id] = candidates[0].id
        else:
            unmatched.append(c.id)
    edges = [{"child_clause_id": c.id, "parent_clause_id": p.id}
             for c in sorted(child.clauses, key=lambda a: a.id)
             for p in sorted(parent.clauses, key=lambda a: a.id) if atom_subsumes(c, p)]
    return {"pass": not unmatched, "rule": "each-derived-disjunct-subsumed-by-parent-disjunct",
            "matching": dict(sorted(mapping.items())), "unmatched_parent_clause_ids": [],
            "unmatched_child_clause_ids": sorted(unmatched), "candidate_edges": edges}


def membership(expr: Compound, request: Request) -> bool:
    results = [atom.matches(request) for atom in expr.clauses]
    return all(results) if expr.kind == "all" else any(results)


def minimize_core(expr: Compound, request: Request, must_admit: bool) -> list[str]:
    clauses = list(sorted(expr.clauses, key=lambda a: a.id))
    changed = True
    while changed and len(clauses) > 1:
        changed = False
        for atom in list(clauses):
            candidate = [item for item in clauses if item.id != atom.id]
            if membership(Compound(expr.kind, tuple(candidate)), request) == must_admit:
                clauses = candidate
                changed = True
                break
    return sorted(atom.id for atom in clauses)


def ordered_difference(left: frozenset[Request], right: frozenset[Request],
                       universe: Universe) -> list[Request]:
    return sorted(left - right, key=universe.key)


def explain_atom(atom: Atom, request: Request) -> dict[str, Any]:
    return {"clause_id": atom.id, "matched": atom.matches(request),
            "failed_dimensions": atom.mismatch_dimensions(request)}


def classify_compounds(parent: Compound, child: Compound, universe: Universe) -> dict[str, Any]:
    unsupported = unsupported_atoms([parent, child])
    if unsupported:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "relation": "bounded-compound", "reason": "unhandled extension has no admitted semantics",
                "unsupported_extensions": unsupported, "finite_universe_size": len(universe.requests()),
                "qualification": "NOT_CLAIMED"}
    if parent.kind != child.kind:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "relation": "bounded-compound", "reason": "cross-type compound pair is not defined",
                "finite_universe_size": len(universe.requests()), "qualification": "NOT_CLAIMED"}

    parent_d = denotation(parent, universe)
    child_d = denotation(child, universe)
    expansion = ordered_difference(child_d, parent_d, universe)
    structural = structural_decision(parent, child)
    result: dict[str, Any] = {
        "schema": RESULT_SCHEMA, "relation": "denotational-containment-plus-structural-subsumption",
        "finite_universe_size": len(universe.requests()), "parent_denotation_size": len(parent_d),
        "child_denotation_size": len(child_d), "denotational_containment": not expansion,
        "structural": structural, "qualification": "NOT_CLAIMED"}

    if expansion:
        request = expansion[0]
        rejected = [explain_atom(a, request) for a in sorted(parent.clauses, key=lambda x: x.id)
                    if not a.matches(request)]
        child_support = sorted(a.id for a in child.clauses if a.matches(request))
        result.update({
            "status": "AUTHORITY_EXPANSION",
            "reason": "child admits a request that parent rejects",
            "counterexample": {
                "request": request.json(), "parent_admits": False, "child_admits": True,
                "first_divergent_parent_constraint": rejected[0] if rejected else None,
                "all_rejected_parent_constraints": rejected,
                "child_supporting_clause_ids": child_support,
                "minimal_witness_core": {
                    "parent_clause_ids_preserving_rejection": minimize_core(parent, request, False),
                    "child_clause_ids_preserving_admission": minimize_core(child, request, True)}}
        })
    elif structural["pass"]:
        result.update({"status": "STRUCTURAL_SUBSUMPTION_PASS",
                       "reason": "bounded denotational containment and structural rule both hold"})
    else:
        result.update({
            "status": "STRUCTURAL_FALSE_NEGATIVE",
            "reason": "semantic containment holds but injective structural matching rejects; no authority expansion exists",
            "counterexample": {"kind": "structural-completeness", "request": None,
                               "unmatched_parent_clause_ids": structural["unmatched_parent_clause_ids"],
                               "candidate_edges": structural.get("candidate_edges", []),
                               "note": "No request exists in child-minus-parent for this finite universe."}})
    return result


def deny_denotation(policy: Policy, universe: Universe) -> frozenset[Request]:
    return frozenset(request for expr in policy.deny for request in denotation(expr, universe))


def effective_denotation(policy: Policy, universe: Universe) -> frozenset[Request]:
    allowed = denotation(policy.allow, universe)
    denied = deny_denotation(policy, universe)
    if policy.conflict_rule == "deny-overrides":
        return frozenset(allowed - denied)
    if policy.conflict_rule == "allow-overrides":
        return allowed
    raise ScenarioError(f"unknown conflict rule: {policy.conflict_rule!r}")


def classify_policies(parent: Policy, child: Policy, universe: Universe) -> dict[str, Any]:
    unsupported = unsupported_atoms([parent.allow, *parent.deny, child.allow, *child.deny])
    if unsupported:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "relation": "effective-policy-denotation", "reason": "unhandled extension has no admitted semantics",
                "unsupported_extensions": unsupported, "finite_universe_size": len(universe.requests()),
                "qualification": "NOT_CLAIMED"}
    if parent.conflict_rule not in {"deny-overrides", "allow-overrides"} or child.conflict_rule not in {
            "deny-overrides", "allow-overrides"}:
        return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
                "relation": "effective-policy-denotation", "reason": "unknown conflict rule",
                "finite_universe_size": len(universe.requests()), "qualification": "NOT_CLAIMED"}

    parent_allow = denotation(parent.allow, universe)
    child_allow = denotation(child.allow, universe)
    parent_deny = deny_denotation(parent, universe)
    child_deny = deny_denotation(child, universe)
    parent_effective = effective_denotation(parent, universe)
    child_effective = effective_denotation(child, universe)
    expansion = ordered_difference(child_effective, parent_effective, universe)
    allow_expansion = ordered_difference(child_allow, parent_allow, universe)
    removed_parent_denies = ordered_difference(parent_deny, child_deny, universe)
    allow_containment = not allow_expansion
    deny_preservation = parent_deny <= child_deny
    conflict_rule_preserved = parent.conflict_rule == child.conflict_rule

    result: dict[str, Any] = {
        "schema": RESULT_SCHEMA, "relation": "effective-policy-denotation-plus-attenuation-components",
        "finite_universe_size": len(universe.requests()),
        "parent_effective_size": len(parent_effective), "child_effective_size": len(child_effective),
        "effective_denotational_containment": not expansion,
        "allow_denotational_containment": allow_containment,
        "deny_preservation": deny_preservation,
        "conflict_rule_preserved": conflict_rule_preserved,
        "parent_conflict_rule": parent.conflict_rule, "child_conflict_rule": child.conflict_rule,
        "allow_denotations_equal": parent_allow == child_allow,
        "parent_deny_size": len(parent_deny), "child_deny_size": len(child_deny),
        "qualification": "NOT_CLAIMED"}

    if expansion:
        request = expansion[0]
        parent_allowed, child_allowed = request in parent_allow, request in child_allow
        parent_denies, child_denies = request in parent_deny, request in child_deny
        if (parent.conflict_rule == "deny-overrides" and parent_denies and not child_denies
                and parent_allowed and child_allowed):
            cause = "effective-deny-removed"
        elif not parent_allowed and child_allowed:
            cause = "allow-denotation-expanded"
        elif parent.conflict_rule != child.conflict_rule:
            cause = "conflict-rule-substitution"
        else:
            cause = "effective-policy-expansion"
        result.update({
            "status": "AUTHORITY_EXPANSION",
            "reason": "child effective policy admits a request denied by parent effective policy",
            "counterexample": {
                "request": request.json(), "parent_effective_admits": False, "child_effective_admits": True,
                "parent_allow_admits": parent_allowed, "child_allow_admits": child_allowed,
                "parent_deny_matches": parent_denies, "child_deny_matches": child_denies, "cause": cause,
                "parent_deny_clause_ids_matching_request": sorted(
                    a.id for expr in parent.deny for a in expr.clauses if a.matches(request)),
                "child_deny_clause_ids_matching_request": sorted(
                    a.id for expr in child.deny for a in expr.clauses if a.matches(request))}})
    elif not allow_containment or not deny_preservation or not conflict_rule_preserved:
        if allow_expansion:
            request = allow_expansion[0]
            cause = "allow-expansion-masked-by-deny" if request not in child_effective else "allow-denotation-expanded"
            evidence = {
                "request": request.json(), "cause": cause,
                "parent_allow_admits": False, "child_allow_admits": True,
                "parent_effective_admits": request in parent_effective,
                "child_effective_admits": request in child_effective,
            }
        elif removed_parent_denies:
            request = removed_parent_denies[0]
            evidence = {
                "request": request.json(),
                "cause": "parent-deny-not-preserved-without-effective-expansion",
                "parent_deny_matches": True, "child_deny_matches": False,
                "parent_effective_admits": request in parent_effective,
                "child_effective_admits": request in child_effective,
            }
        else:
            evidence = {
                "request": None, "cause": "conflict-rule-substitution-without-finite-witness",
                "parent_effective_admits": None, "child_effective_admits": None,
            }
        result.update({
            "status": "POLICY_ATTENUATION_VIOLATION",
            "reason": "effective containment alone does not excuse allow expansion, deny deletion, or conflict-rule substitution",
            "counterexample": evidence,
        })
    else:
        result.update({
            "status": "EFFECTIVE_POLICY_CONTAINMENT_PASS",
            "reason": "effective containment, allow containment, deny preservation, and conflict-rule preservation all hold",
        })
    return result

def evaluate_scenario(raw: dict[str, Any]) -> dict[str, Any]:
    if raw.get("schema") != SCENARIO_SCHEMA:
        raise ScenarioError(f"scenario schema must be {SCENARIO_SCHEMA}")
    universe = Universe.parse(raw.get("universe", {}))
    mode = raw.get("mode", "compound")
    if mode == "compound":
        return classify_compounds(Compound.parse(raw.get("parent", {}), universe),
                                  Compound.parse(raw.get("child", {}), universe), universe)
    if mode == "effective-policy":
        return classify_policies(Policy.parse(raw.get("parent_policy", {}), universe),
                                 Policy.parse(raw.get("child_policy", {}), universe), universe)
    return {"schema": RESULT_SCHEMA, "status": "UNSUPPORTED_OR_UNDECIDABLE",
            "relation": str(mode), "reason": "mode not in admitted grammar", "qualification": "NOT_CLAIMED"}


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        input_bytes = args.scenario.read_bytes()
        raw = json.loads(input_bytes.decode("utf-8"))
        result = evaluate_scenario(raw)
        result["input_sha256"] = hashlib.sha256(input_bytes).hexdigest()
        result["result_sha256"] = hashlib.sha256(canonical_json(result)).hexdigest()
        output = json.dumps(result, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output, encoding="utf-8")
        print(output, end="")
        return 0
    except (OSError, json.JSONDecodeError, ScenarioError, TypeError, KeyError) as error:
        print(f"COUNTEREXAMPLE ORACLE INPUT ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
