#!/usr/bin/env python3
"""Deterministic adversarial controls for the bounded counterexample oracle."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import compound_subsumption_counterexamples as oracle  # noqa: E402

TARGETS = ["alice", "bob"]
PURPOSES = ["business", "refund"]
CONTEXTS = ["trusted", "untrusted"]
AMOUNTS = [0, 1, 2, 3]
UNIVERSE = {"targets": TARGETS, "purposes": PURPOSES, "contexts": CONTEXTS, "amounts": AMOUNTS}
UNIVERSE_SIZE = len(TARGETS) * len(PURPOSES) * len(CONTEXTS) * len(AMOUNTS)


def atom(atom_id: str, target: list[str], purpose: list[str], context: list[str],
         max_amount: int, extension: str = "none") -> dict[str, Any]:
    return {"id": atom_id, "target": target, "purpose": purpose, "context": context,
            "max_amount": max_amount, "extension": extension}


def compound(kind: str, *clauses: dict[str, Any]) -> dict[str, Any]:
    return {"kind": kind, "clauses": list(clauses)}


def scenario(mode: str = "compound", **payload: Any) -> dict[str, Any]:
    return {"schema": oracle.SCENARIO_SCHEMA, "universe": UNIVERSE, "mode": mode, **payload}


BROAD = atom("broad", TARGETS, PURPOSES, CONTEXTS, 3)
NARROW = atom("narrow", ["alice"], ["business"], ["trusted"], 3)
CHILD_NARROW = atom("child-narrow", ["alice"], ["business"], ["trusted"], 2)
CHILD_BROAD = atom("child-broad", TARGETS, PURPOSES, CONTEXTS, 2)
ALLOW_ALL = compound("any", atom("allow-all", TARGETS, PURPOSES, CONTEXTS, 3))
ALLOW_ALICE = compound("any", atom("allow-alice", ["alice"], PURPOSES, CONTEXTS, 3))
DENY_BOB = compound("all", atom("deny-bob", ["bob"], PURPOSES, CONTEXTS, 3))


def controls() -> list[tuple[str, dict[str, Any], str]]:
    expansion = scenario(
        parent=compound("all", atom("parent-alice", ["alice"], PURPOSES, CONTEXTS, 3)),
        child=compound("all", atom("child-alice-bob", TARGETS, PURPOSES, CONTEXTS, 3)),
    )
    structural_false_negative = scenario(
        parent=compound(
            "all",
            atom("parent-context", TARGETS, PURPOSES, ["trusted"], 3),
            atom("parent-purpose", TARGETS, ["business"], CONTEXTS, 3),
        ),
        child=compound("all", atom("child-intersection", ["alice"], ["business"], ["trusted"], 2)),
    )
    ordered_parent = compound("all", BROAD, NARROW)
    ordered_child = compound("all", CHILD_NARROW, CHILD_BROAD)
    order_forward = scenario(parent=ordered_parent, child=ordered_child)
    order_reverse = scenario(parent=compound("all", NARROW, BROAD),
                             child=compound("all", CHILD_BROAD, CHILD_NARROW))
    unsupported = scenario(
        parent=compound("all", NARROW),
        child=compound("all", atom("future-clause", ["alice"], ["business"], ["trusted"], 2, "future-v1")),
    )
    deny_deleted = scenario(
        "effective-policy",
        parent_policy={"allow": ALLOW_ALL, "deny": [DENY_BOB], "conflict_rule": "deny-overrides"},
        child_policy={"allow": ALLOW_ALL, "deny": [], "conflict_rule": "deny-overrides"},
    )
    deny_added = scenario(
        "effective-policy",
        parent_policy={"allow": ALLOW_ALL, "deny": [], "conflict_rule": "deny-overrides"},
        child_policy={"allow": ALLOW_ALL, "deny": [DENY_BOB], "conflict_rule": "deny-overrides"},
    )
    conflict_changed = scenario(
        "effective-policy",
        parent_policy={"allow": ALLOW_ALL, "deny": [DENY_BOB], "conflict_rule": "deny-overrides"},
        child_policy={"allow": ALLOW_ALL, "deny": [DENY_BOB], "conflict_rule": "allow-overrides"},
    )
    deny_removed_without_expansion = scenario(
        "effective-policy",
        parent_policy={"allow": ALLOW_ALICE, "deny": [DENY_BOB], "conflict_rule": "deny-overrides"},
        child_policy={"allow": ALLOW_ALICE, "deny": [], "conflict_rule": "deny-overrides"},
    )
    allow_expansion_masked_by_deny = scenario(
        "effective-policy",
        parent_policy={"allow": ALLOW_ALICE, "deny": [], "conflict_rule": "deny-overrides"},
        child_policy={"allow": ALLOW_ALL, "deny": [DENY_BOB], "conflict_rule": "deny-overrides"},
    )
    unsupported_deny = scenario(
        "effective-policy",
        parent_policy={"allow": ALLOW_ALL, "deny": [], "conflict_rule": "deny-overrides"},
        child_policy={
            "allow": ALLOW_ALL,
            "deny": [compound("all", atom("future-deny", ["bob"], PURPOSES, CONTEXTS, 3, "future-deny-v1"))],
            "conflict_rule": "deny-overrides",
        },
    )
    return [
        ("actual-authority-expansion", expansion, "AUTHORITY_EXPANSION"),
        ("structural-false-negative", structural_false_negative, "STRUCTURAL_FALSE_NEGATIVE"),
        ("clause-order-forward", order_forward, "STRUCTURAL_SUBSUMPTION_PASS"),
        ("clause-order-reverse", order_reverse, "STRUCTURAL_SUBSUMPTION_PASS"),
        ("unsupported-extension", unsupported, "UNSUPPORTED_OR_UNDECIDABLE"),
        ("deny-deletion-expansion", deny_deleted, "AUTHORITY_EXPANSION"),
        ("deny-addition-restriction", deny_added, "EFFECTIVE_POLICY_CONTAINMENT_PASS"),
        ("deny-removal-no-effective-expansion", deny_removed_without_expansion, "POLICY_ATTENUATION_VIOLATION"),
        ("allow-expansion-masked-by-deny", allow_expansion_masked_by_deny, "POLICY_ATTENUATION_VIOLATION"),
        ("conflict-rule-substitution", conflict_changed, "AUTHORITY_EXPANSION"),
        ("unsupported-extension-in-deny", unsupported_deny, "UNSUPPORTED_OR_UNDECIDABLE"),
    ]


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def independent_atom_matches(raw_atom: dict[str, Any], request: dict[str, Any]) -> bool:
    # Deliberately separate from the oracle's Atom.matches implementation.
    return (
        request["target"] in raw_atom["target"]
        and request["purpose"] in raw_atom["purpose"]
        and request["context"] in raw_atom["context"]
        and request["amount"] <= raw_atom["max_amount"]
    )


def independent_compound_matches(raw_expr: dict[str, Any], request: dict[str, Any]) -> bool:
    outcomes = [independent_atom_matches(a, request) for a in raw_expr["clauses"]]
    return all(outcomes) if raw_expr["kind"] == "all" else any(outcomes)


def independent_policy_matches(raw_policy: dict[str, Any], request: dict[str, Any]) -> bool:
    allowed = independent_compound_matches(raw_policy["allow"], request)
    denies = any(independent_compound_matches(expr, request) for expr in raw_policy["deny"])
    if raw_policy["conflict_rule"] == "deny-overrides":
        return allowed and not denies
    if raw_policy["conflict_rule"] == "allow-overrides":
        return allowed
    raise AssertionError("independent replay encountered an unknown conflict rule")


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode()


def replay_and_assert(name: str, raw: dict[str, Any], result: dict[str, Any]) -> None:
    status = result["status"]
    if status == "AUTHORITY_EXPANSION":
        request = result["counterexample"]["request"]
        if raw.get("mode", "compound") == "effective-policy":
            parent = raw["parent_policy"]
            child = raw["child_policy"]
            check(not independent_policy_matches(parent, request),
                  f"{name}: replay says parent already allows witness")
            check(independent_policy_matches(child, request),
                  f"{name}: replay says child does not allow witness")
        else:
            check(not independent_compound_matches(raw["parent"], request),
                  f"{name}: replay says parent already allows witness")
            check(independent_compound_matches(raw["child"], request),
                  f"{name}: replay says child does not allow witness")
    elif status == "STRUCTURAL_FALSE_NEGATIVE":
        # Independently enumerate every one of the 32 admitted requests.
        child = raw["child"]
        parent = raw["parent"]
        for target in TARGETS:
            for purpose in PURPOSES:
                for context in CONTEXTS:
                    for amount in AMOUNTS:
                        request = {"target": target, "purpose": purpose,
                                   "context": context, "amount": amount}
                        check(not (independent_compound_matches(child, request)
                                   and not independent_compound_matches(parent, request)),
                              f"{name}: alleged structural false negative hides an expansion at {request}")
        check(result["counterexample"]["request"] is None,
              f"{name}: structural false negative was mislabeled as a request expansion")
    if status == "POLICY_ATTENUATION_VIOLATION":
        request = result["counterexample"]["request"]
        check(result["effective_denotational_containment"],
              f"{name}: this control must not be an effective-authority expansion")
        if name == "deny-removal-no-effective-expansion":
            parent, child = raw["parent_policy"], raw["child_policy"]
            check(not independent_policy_matches(parent, request)
                  and not independent_policy_matches(child, request),
                  "deny-removal control unexpectedly changed effective authorization")
            check(any(independent_compound_matches(expr, request) for expr in parent["deny"]),
                  "parent deny did not match the minimized policy-boundary witness")
            check(not any(independent_compound_matches(expr, request) for expr in child["deny"]),
                  "child deny unexpectedly retained the deleted restriction")
        if name == "allow-expansion-masked-by-deny":
            parent, child = raw["parent_policy"], raw["child_policy"]
            check(not independent_policy_matches(parent, request)
                  and not independent_policy_matches(child, request),
                  "masked allow-expansion control unexpectedly changed effective authorization")
            check(not independent_compound_matches(parent["allow"], request)
                  and independent_compound_matches(child["allow"], request),
                  "allow denotation expansion was not independently replayed")
            check(any(independent_compound_matches(expr, request) for expr in child["deny"]),
                  "child deny did not mask the newly added allow")
    if name == "deny-deletion-expansion":
        check(result["counterexample"]["cause"] == "effective-deny-removed",
              "deny deletion was not diagnosed as effective deny removal")
        check(result["allow_denotations_equal"], "deny deletion changed the allow denotation unexpectedly")
    if name == "deny-removal-no-effective-expansion":
        check(result["effective_denotational_containment"], "fixture unexpectedly expanded effective authorization")
        check(not result["deny_preservation"], "deleted parent deny was incorrectly treated as preserved")
        check(result["status"] == "POLICY_ATTENUATION_VIOLATION", "deleted deny escaped attenuation gate")
    if name == "allow-expansion-masked-by-deny":
        check(result["effective_denotational_containment"], "deny did not mask the allow expansion as intended")
        check(not result["allow_denotational_containment"], "expanded allow denotation was not detected")
        check(result["counterexample"]["cause"] == "allow-expansion-masked-by-deny",
              "masked allow expansion was misclassified")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--matrix", type=Path, default=HERE.parents[2] / "docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_COMPOUND_SUBSUMPTION_COUNTEREXAMPLE_CONTROL_MATRIX_V1.json")
    args = parser.parse_args()
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
    check(matrix.get("schema") == "mycelix.compound-subsumption-counterexample-control-matrix.v1", "unexpected control matrix schema")
    check(matrix.get("finite_universe", {}).get("request_count") == UNIVERSE_SIZE, "matrix finite-universe count mismatch")
    frozen = {row["id"]: row for row in matrix.get("controls", [])}
    check(len(frozen) == len(matrix.get("controls", [])), "duplicate matrix control IDs")
    built_controls = controls()
    check(set(frozen) == {name for name, _, _ in built_controls}, "matrix and executable control IDs differ")
    check(all(frozen[name]["expected_status"] == expected for name, _, expected in built_controls), "matrix expected statuses differ from executable controls")
    receipt: dict[str, Any] = {
        "schema": "mycelix.compound-subsumption-counterexample-controls.v1",
        "status": "RUNNING",
        "finite_universe_size": UNIVERSE_SIZE,
        "controls": [],
        "qualification": "NOT_CLAIMED",
    }
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], text=True, capture_output=True,
                              check=True, timeout=15).stdout.strip()
        receipt["source_head"] = head
        source_path = HERE / "compound_subsumption_counterexamples.py"
        receipt["oracle_source_sha256"] = hashlib.sha256(source_path.read_bytes()).hexdigest()
        receipt["control_matrix_sha256"] = hashlib.sha256(args.matrix.read_bytes()).hexdigest()
        named_results = {}
        for name, raw, expected_status in built_controls:
            result = oracle.evaluate_scenario(raw)
            check(result["status"] == expected_status,
                  f"{name}: expected {expected_status}, got {result['status']}")
            replay_and_assert(name, raw, result)
            input_bytes = canonical_json(raw)
            result["input_sha256"] = hashlib.sha256(input_bytes).hexdigest()
            result["result_sha256"] = hashlib.sha256(canonical_json(result)).hexdigest()
            (args.evidence_dir / f"{name}.input.json").write_text(
                json.dumps(raw, sort_keys=True, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8")
            (args.evidence_dir / f"{name}.json").write_text(
                json.dumps(result, sort_keys=True, indent=2) + "\n", encoding="utf-8")
            row = {"id": name, "expected_status": expected_status,
                   "observed_status": result["status"], "independent_replay": "PASS",
                   "input_sha256": result["input_sha256"], "result_sha256": result["result_sha256"]}
            if "counterexample" in result:
                row["counterexample"] = result["counterexample"]
            row["marker"] = frozen[name]["marker"]
            receipt["controls"].append(row)
            named_results[name] = result
            print(frozen[name]["marker"])

        forward = named_results["clause-order-forward"]
        reverse = named_results["clause-order-reverse"]
        check(forward["status"] == reverse["status"], "clause permutation changed decision")
        check(forward["structural"]["matching"] == reverse["structural"]["matching"],
              "clause permutation changed deterministic witness assignment")
        expansion_witness = named_results["actual-authority-expansion"]["counterexample"]["request"]
        check(expansion_witness == {"target": "bob", "purpose": "business",
                                    "context": "trusted", "amount": 0},
              f"unexpected minimal request under frozen universe order: {expansion_witness}")
        check(named_results["deny-deletion-expansion"]["counterexample"]["request"]["target"] == "bob",
              "deny deletion witness did not identify the denied target")

        receipt["status"] = "PASS"
        receipt["summary"] = {
            "controls": len(receipt["controls"]),
            "true_authority_expansions": 3,
            "structural_false_negatives": 1,
            "policy_attenuation_violations_without_effective_expansion": 2,
            "unsupported_fail_closed": 2,
            "clause_order_invariant": True,
            "independent_replay": "PASS",
            "qualification": "NOT_CLAIMED",
        }
        (args.evidence_dir / "receipt.json").write_text(
            json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print(f"COUNTEREXAMPLE ORACLE PASS: {len(receipt['controls'])} controls; independent replay passed")
        print("STRUCTURAL FALSE NEGATIVE DISTINCT FROM AUTHORITY EXPANSION: PASS")
        print("DENY DELETION / CONFLICT RULE EXPANSION WITNESSES: PASS")
        print("QUALIFICATION NOT CLAIMED: finite bounded research/specification evidence only")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        (args.evidence_dir / "receipt.json").write_text(
            json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print(f"COUNTEREXAMPLE ORACLE FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
