#!/usr/bin/env python3
"""Exhaustive bounded differential checker for compound attenuation semantics.

The oracle implementation and the reference implementation intentionally do not
share denotation or matching functions. The declared grammar/universe is finite;
no unbounded-policy completeness claim is made.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import compound_subsumption_counterexamples as oracle  # noqa: E402

MATRIX_SCHEMA = "mycelix.compound-subsumption-differential-matrix.v1"
RECEIPT_SCHEMA = "mycelix.compound-subsumption-differential-receipt.v1"


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def declared_requests(universe: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        {"target": target, "purpose": purpose, "context": context, "amount": amount}
        for target, purpose, context, amount in itertools.product(
            universe["targets"], universe["purposes"], universe["contexts"], universe["amounts"]
        )
    ]


def request_key(request: dict[str, Any], universe: dict[str, Any]) -> tuple[int, int, int, int]:
    return (
        universe["targets"].index(request["target"]),
        universe["purposes"].index(request["purpose"]),
        universe["contexts"].index(request["context"]),
        universe["amounts"].index(request["amount"]),
    )


def independent_atom_matches(atom: dict[str, Any], request: dict[str, Any]) -> bool:
    """Independent evaluator over raw JSON; does not invoke oracle.Atom.matches."""
    return (
        request["target"] in atom["target"]
        and request["purpose"] in atom["purpose"]
        and request["context"] in atom["context"]
        and request["amount"] <= atom["max_amount"]
    )


def independent_denotation(expr: dict[str, Any], universe: dict[str, Any]) -> frozenset[tuple[str, str, str, int]]:
    admitted: set[tuple[str, str, str, int]] = set()
    for request in declared_requests(universe):
        atomic_results = [independent_atom_matches(atom, request) for atom in expr["clauses"]]
        accepts = all(atomic_results) if expr["kind"] == "all" else any(atomic_results)
        if accepts:
            admitted.add((request["target"], request["purpose"], request["context"], request["amount"]))
    return frozenset(admitted)


def independent_atom_subsumes(child: dict[str, Any], parent: dict[str, Any]) -> bool:
    """Independent sufficient atom relation used by the structural rule."""
    if child.get("extension", "none") != "none" or parent.get("extension", "none") != "none":
        return False
    return (
        set(child["target"]) <= set(parent["target"])
        and set(child["purpose"]) <= set(parent["purpose"])
        and set(child["context"]) <= set(parent["context"])
        and child["max_amount"] <= parent["max_amount"]
    )


def independent_structural(parent: dict[str, Any], child: dict[str, Any]) -> tuple[bool, dict[str, str]]:
    if parent["kind"] != child["kind"]:
        return False, {}
    if parent["kind"] == "all":
        # Brute-force every injection. This does not use the augmenting-path implementation.
        n_parent, n_child = len(parent["clauses"]), len(child["clauses"])
        if n_child < n_parent:
            return False, {}
        for selected in itertools.permutations(range(n_child), n_parent):
            if all(
                independent_atom_subsumes(child["clauses"][selected[pi]], parent["clauses"][pi])
                for pi in range(n_parent)
            ):
                return True, {
                    parent["clauses"][pi]["id"]: child["clauses"][selected[pi]]["id"]
                    for pi in range(n_parent)
                }
        return False, {}

    # For disjunction, every derived branch must be covered by at least one
    # parent branch. This is conservative with respect to collective coverage.
    mapping: dict[str, str] = {}
    for child_atom in child["clauses"]:
        candidates = sorted(
            parent_atom["id"]
            for parent_atom in parent["clauses"]
            if independent_atom_subsumes(child_atom, parent_atom)
        )
        if not candidates:
            return False, mapping
        mapping[child_atom["id"]] = candidates[0]
    return True, dict(sorted(mapping.items()))


def expression_key(expr: dict[str, Any]) -> str:
    """Canonical identity, intentionally invariant under clause permutation."""
    return expr["kind"] + ":" + ",".join(sorted(atom["id"] for atom in expr["clauses"]))


def expression_ordered_key(expr: dict[str, Any]) -> str:
    """Identity used to ensure every ordered syntax form is actually generated."""
    return expr["kind"] + ":" + ",".join(atom["id"] for atom in expr["clauses"])


def scenario_for(parent: dict[str, Any], child: dict[str, Any], universe: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": oracle.SCENARIO_SCHEMA,
        "universe": universe,
        "mode": "compound",
        "parent": parent,
        "child": child,
    }


def first_expansion_witness(child_denotation: frozenset, parent_denotation: frozenset,
                            universe: dict[str, Any]) -> dict[str, Any] | None:
    difference = child_denotation - parent_denotation
    if not difference:
        return None
    request_rows = declared_requests(universe)
    by_tuple = {
        (req["target"], req["purpose"], req["context"], req["amount"]): req
        for req in request_rows
    }
    ordered = sorted(difference, key=lambda t: request_key(by_tuple[t], universe))
    return by_tuple[ordered[0]]


def mismatch_for(raw: dict[str, Any], result: dict[str, Any] | None = None) -> dict[str, Any] | None:
    """Return a bug/mismatch category, or None when independent relations agree."""
    parent, child = raw["parent"], raw["child"]
    if result is None:
        result = oracle.evaluate_scenario(raw)
    if parent["kind"] != child["kind"]:
        if result.get("status") != "UNSUPPORTED_OR_UNDECIDABLE":
            return {"kind": "cross-type-fail-closed", "expected": "UNSUPPORTED_OR_UNDECIDABLE",
                    "observed": result.get("status")}
        return None

    universe = raw["universe"]
    parent_d = independent_denotation(parent, universe)
    child_d = independent_denotation(child, universe)
    independently_contained = child_d <= parent_d
    independent_structure_pass, independent_mapping = independent_structural(parent, child)
    observed_structure_pass = bool(result.get("structural", {}).get("pass"))

    if observed_structure_pass != independent_structure_pass:
        return {
            "kind": "structural-matcher-disagrees-with-bruteforce",
            "expected_pass": independent_structure_pass,
            "observed_pass": observed_structure_pass,
            "expected_mapping": independent_mapping,
            "observed_mapping": result.get("structural", {}).get("matching", {}),
        }

    emitted_mapping = result.get("structural", {}).get("matching", {})
    parent_by_id = {atom["id"]: atom for atom in parent["clauses"]}
    child_by_id = {atom["id"]: atom for atom in child["clauses"]}
    if parent["kind"] == "all":
        valid_ids = all(pid in parent_by_id and cid in child_by_id
                        for pid, cid in emitted_mapping.items())
        injective = len(set(emitted_mapping.values())) == len(emitted_mapping)
        valid_edges = all(
            independent_atom_subsumes(child_by_id[cid], parent_by_id[pid])
            for pid, cid in emitted_mapping.items()
        )
        complete_when_pass = not observed_structure_pass or set(emitted_mapping) == set(parent_by_id)
        if not (valid_ids and injective and valid_edges and complete_when_pass):
            return {
                "kind": "structural-witness-map-invalid",
                "operator": "all",
                "valid_ids": valid_ids,
                "injective": injective,
                "valid_edges": valid_edges,
                "complete_when_pass": complete_when_pass,
                "observed_mapping": emitted_mapping,
            }
    else:
        valid_ids = all(cid in child_by_id and pid in parent_by_id
                        for cid, pid in emitted_mapping.items())
        valid_edges = all(
            independent_atom_subsumes(child_by_id[cid], parent_by_id[pid])
            for cid, pid in emitted_mapping.items()
        )
        complete_when_pass = not observed_structure_pass or set(emitted_mapping) == set(child_by_id)
        if not (valid_ids and valid_edges and complete_when_pass):
            return {
                "kind": "structural-witness-map-invalid",
                "operator": "any",
                "valid_ids": valid_ids,
                "valid_edges": valid_edges,
                "complete_when_pass": complete_when_pass,
                "observed_mapping": emitted_mapping,
            }

    expected_status = (
        "AUTHORITY_EXPANSION" if not independently_contained
        else "STRUCTURAL_SUBSUMPTION_PASS" if independent_structure_pass
        else "STRUCTURAL_FALSE_NEGATIVE"
    )
    if result.get("status") != expected_status:
        return {"kind": "classification-disagrees-with-independent-denotation",
                "expected_status": expected_status, "observed_status": result.get("status"),
                "independent_containment": independently_contained,
                "independent_structural_pass": independent_structure_pass}

    witness = first_expansion_witness(child_d, parent_d, universe)
    counterexample = result.get("counterexample", {})
    if witness is not None and counterexample.get("request") != witness:
        return {"kind": "counterexample-witness-not-minimal",
                "expected_request": witness, "observed_request": counterexample.get("request")}
    if witness is None and expected_status == "STRUCTURAL_FALSE_NEGATIVE":
        if counterexample.get("request") is not None:
            return {"kind": "structural-false-negative-misreported-as-request-expansion",
                    "observed_request": counterexample.get("request")}
    return None


def control_signature(result: dict[str, Any]) -> dict[str, Any]:
    counterexample = result.get("counterexample", {})
    return {
        "status": result.get("status"),
        "denotational_containment": result.get("denotational_containment"),
        "structural_pass": result.get("structural", {}).get("pass"),
        "matching": result.get("structural", {}).get("matching", {}),
        "request": counterexample.get("request"),
    }


def permutation_mismatch_for(raw: dict[str, Any]) -> dict[str, Any] | None:
    permuted = copy.deepcopy(raw)
    permuted["parent"]["clauses"].reverse()
    permuted["child"]["clauses"].reverse()
    original_result = oracle.evaluate_scenario(raw)
    permuted_result = oracle.evaluate_scenario(permuted)
    original_signature = control_signature(original_result)
    permuted_signature = control_signature(permuted_result)
    if original_signature != permuted_signature:
        return {
            "kind": "clause-order-permutation-changed-result",
            "prior_signature": original_signature,
            "permuted_signature": permuted_signature,
        }
    return None


def minimize_counterexample(raw: dict[str, Any], failure_kind: str) -> dict[str, Any]:
    """Deterministic delta reduction; preserve the same mismatch category."""
    reduced = copy.deepcopy(raw)

    def retains(candidate: dict[str, Any]) -> bool:
        if failure_kind == "clause-order-permutation-changed-result":
            mismatch = permutation_mismatch_for(candidate)
        else:
            mismatch = mismatch_for(candidate)
        return mismatch is not None and mismatch.get("kind") == failure_kind

    for side in ("parent", "child"):
        changed = True
        while changed and len(reduced[side]["clauses"]) > 1:
            changed = False
            for index in range(len(reduced[side]["clauses"])):
                candidate = copy.deepcopy(reduced)
                del candidate[side]["clauses"][index]
                if retains(candidate):
                    reduced = candidate
                    changed = True
                    break

    for side in ("parent", "child"):
        for clause_index in range(len(reduced[side]["clauses"])):
            for dimension in ("target", "purpose", "context"):
                changed = True
                while changed and len(reduced[side]["clauses"][clause_index][dimension]) > 1:
                    changed = False
                    for value_index in range(len(reduced[side]["clauses"][clause_index][dimension])):
                        candidate = copy.deepcopy(reduced)
                        del candidate[side]["clauses"][clause_index][dimension][value_index]
                        if retains(candidate):
                            reduced = candidate
                            changed = True
                            break
            current_max = reduced[side]["clauses"][clause_index]["max_amount"]
            for new_max in range(current_max):
                candidate = copy.deepcopy(reduced)
                candidate[side]["clauses"][clause_index]["max_amount"] = new_max
                if retains(candidate):
                    reduced = candidate
                    break
    return reduced


def control_signature(result: dict[str, Any]) -> dict[str, Any]:
    counterexample = result.get("counterexample", {})
    return {
        "status": result.get("status"),
        "denotational_containment": result.get("denotational_containment"),
        "structural_pass": result.get("structural", {}).get("pass"),
        "matching": result.get("structural", {}).get("matching", {}),
        "request": counterexample.get("request"),
    }


def validate_matrix(matrix: dict[str, Any]) -> None:
    require(matrix.get("schema") == MATRIX_SCHEMA, "unexpected differential matrix schema")
    universe = matrix.get("finite_universe", {})
    require(universe.get("request_count") == 32, "request_count must be exactly 32")
    require(len(declared_requests(universe)) == 32, "declared universe does not contain 32 requests")
    corpus = matrix.get("corpus", {})
    require(corpus.get("atom_count") == 8, "corpus atom count is not 8")
    require(corpus.get("max_clauses_per_compound") == 2, "max clause count is not 2")
    require(corpus.get("expression_count") == 128, "expected 128 generated expressions")
    require(corpus.get("ordered_parent_child_pairs") == 16384, "expected 16,384 ordered pairs")
    require(corpus.get("ordered_pair_request_combinations") == 524288, "expected 524,288 pair-request combinations")
    atoms = matrix.get("atoms", [])
    require(len(atoms) == 8 and len({atom.get("id") for atom in atoms}) == 8,
            "frozen atom catalogue is incomplete or has duplicate IDs")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {"schema": RECEIPT_SCHEMA, "status": "RUNNING",
                               "counts": {}, "qualification": "NOT_CLAIMED"}
    try:
        matrix_bytes = args.matrix.read_bytes()
        matrix = json.loads(matrix_bytes.decode("utf-8"))
        validate_matrix(matrix)
        universe = matrix["finite_universe"]
        atoms = matrix["atoms"]
        requests = declared_requests(universe)
        require(len(requests) == 32, "bounded request enumeration failed")

        expressions = []
        for kind in ("all", "any"):
            for clause_count in (1, 2):
                for selected in itertools.permutations(atoms, clause_count):
                    expressions.append({"kind": kind, "clauses": [copy.deepcopy(atom) for atom in selected]})
        require(len(expressions) == 128, f"generated {len(expressions)} expressions, expected 128")
        require(len({expression_ordered_key(expr) for expr in expressions}) == 128,
                "ordered expression identifiers are not unique")

        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        receipt["matrix_sha256"] = sha256_bytes(matrix_bytes)
        oracle_path = HERE / "compound_subsumption_counterexamples.py"
        receipt["oracle_sha256"] = sha256_bytes(oracle_path.read_bytes())
        receipt["checker_sha256"] = sha256_bytes(Path(__file__).resolve().read_bytes())
        receipt["finite_universe_size"] = len(requests)
        receipt["expressions"] = len(expressions)
        receipt["ordered_pairs_expected"] = len(expressions) ** 2

        counts: Counter[str] = Counter()
        type_pairs: Counter[str] = Counter()
        seen_permutation_signatures: dict[tuple[str, str], str] = {}
        first_samples: dict[str, Any] = {}
        first_mismatch: dict[str, Any] | None = None
        evaluated_pairs = 0

        for parent in expressions:
            for child in expressions:
                raw = scenario_for(parent, child, universe)
                observed = oracle.evaluate_scenario(raw)
                evaluated_pairs += 1
                type_pairs[f"{parent['kind']}->{child['kind']}"] += 1
                mismatch = mismatch_for(raw, observed)
                if mismatch is not None:
                    first_mismatch = {"mismatch": mismatch, "scenario": raw, "observed_result": observed}
                    first_mismatch["minimized_scenario"] = minimize_counterexample(raw, mismatch["kind"])
                    first_mismatch["minimized_mismatch"] = (
                        mismatch_for(first_mismatch["minimized_scenario"])
                        or permutation_mismatch_for(first_mismatch["minimized_scenario"])
                    )
                    break
                counts[observed["status"]] += 1
                if observed["status"] not in first_samples:
                    first_samples[observed["status"]] = {
                        "parent": expression_key(parent), "child": expression_key(child),
                        "status": observed["status"], "counterexample": observed.get("counterexample")}
                pair_key = (expression_key(parent), expression_key(child))
                signature = control_signature(observed)
                encoded = json.dumps(signature, sort_keys=True)
                prior = seen_permutation_signatures.get(pair_key)
                if prior is not None and prior != encoded:
                    first_mismatch = {
                        "mismatch": {"kind": "clause-order-permutation-changed-result",
                                     "parent_expression": pair_key[0], "child_expression": pair_key[1],
                                     "prior_signature": json.loads(prior), "observed_signature": signature},
                        "scenario": raw,
                        "observed_result": observed}
                    first_mismatch["minimized_scenario"] = minimize_counterexample(
                        raw, "clause-order-permutation-changed-result")
                    first_mismatch["minimized_mismatch"] = (
                        mismatch_for(first_mismatch["minimized_scenario"])
                        or permutation_mismatch_for(first_mismatch["minimized_scenario"])
                    )
                    break
                seen_permutation_signatures[pair_key] = encoded
            if first_mismatch is not None:
                break

        receipt["counts"] = dict(sorted(counts.items()))
        receipt["type_pairs"] = dict(sorted(type_pairs.items()))
        receipt["ordered_pairs_evaluated"] = evaluated_pairs
        receipt["permutation_signature_checks"] = len(seen_permutation_signatures)
        receipt["first_samples"] = first_samples

        if first_mismatch is not None:
            (args.evidence_dir / "minimized-counterexample.json").write_text(
                json.dumps(first_mismatch, sort_keys=True, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            receipt["status"] = "FAIL"
            receipt["failure_kind"] = first_mismatch["mismatch"]["kind"]
            receipt["failure_artifact"] = "minimized-counterexample.json"
            (args.evidence_dir / "receipt.json").write_text(
                json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(f"DIFFERENTIAL COMPOUND CHECK FAIL: {receipt['failure_kind']}", file=sys.stderr)
            print(f"MINIMIZED COUNTEREXAMPLE: {args.evidence_dir / 'minimized-counterexample.json'}", file=sys.stderr)
            return 1

        require(evaluated_pairs == 16384, f"only evaluated {evaluated_pairs} of 16,384 pairs")
        require(sum(type_pairs.values()) == 16384, "type-pair accounting mismatch")
        require(counts["UNSUPPORTED_OR_UNDECIDABLE"] == 8192,
                f"expected 8,192 cross-kind fail-closed pairs, got {counts['UNSUPPORTED_OR_UNDECIDABLE']}")
        receipt["status"] = "PASS"
        receipt["summary"] = {
            "ordered_parent_child_pairs": evaluated_pairs,
            "bounded_requests": 32,
            "ordered_pair_request_combinations": evaluated_pairs * 32,
            "cross_type_pairs_fail_closed": counts["UNSUPPORTED_OR_UNDECIDABLE"],
            "independent_denotation": "PASS",
            "brute_force_structural_reference": "PASS",
            "clause_order_metamorphism": "PASS",
            "counterexample_shrinking": "AVAILABLE_ON_FAILURE",
            "qualification": "NOT_CLAIMED"}
        (args.evidence_dir / "receipt.json").write_text(
            json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print("DIFFERENTIAL COMPOUND CHECK PASS: 16,384 ordered policy pairs")
        print("INDEPENDENT DENOTATION / BRUTE-FORCE WITNESS REFERENCE: PASS")
        print("CLAUSE-ORDER METAMORPHIC CHECK: PASS")
        print("CROSS-TYPE FAIL-CLOSED CONTROLS: 8,192")
        print("QUALIFICATION NOT CLAIMED: finite bounded research/specification evidence only")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        (args.evidence_dir / "receipt.json").write_text(
            json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"DIFFERENTIAL COMPOUND CHECK ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
