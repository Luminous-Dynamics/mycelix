#!/usr/bin/env python3
"""Deterministic exact-matrix runner for SOV-AI-AUTH-025.

Negative controls are expected to produce the *named* invariant violation.
Positive controls and the canonical model must complete without violations.
Alloy check commands are checked against an explicit expected-result map.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


PROPERTY_BY_CONTROL = {
    "parent-clause-deletion": "ParentClauseDeletionSafe",
    "duplicate-witness-reuse": "DuplicateWitnessReuseSafe",
    "greedy-dead-end": "GreedyDeadEndSafe",
    "clause-order-permutation": "ClauseOrderPermutationSafe",
    "semantic-equivalent-conjunction": "SemanticEquivalentSafe",
    "semantic-non-equivalent-normalization": "SemanticNonEquivalentNormalizationSafe",
    "cross-type-substitution": "CrossTypeSafe",
    "unsupported-extension-inside-compound": "UnsupportedCompoundSafe",
    "disjunct-expansion": "DisjunctExpansionSafe",
    "additional-restrictive-clause": "AdditionalRestrictiveSafe",
}

ALLOY_EXPECTED = {
    "WitnessSound": "UNSAT",
    "WitnessInjective": "UNSAT",
    "CanonicalAggregate": "UNSAT",
    "ParentClauseDeletionSafe": "SAT",
    "DuplicateWitnessReuseSafe": "SAT",
    "GreedyDeadEndSafe": "SAT",
    "ClauseOrderPermutationSafe": "UNSAT",
    "SemanticEquivalentSafe": "UNSAT",
    "SemanticNonEquivalentNormalizationSafe": "SAT",
    "CrossTypeSafe": "SAT",
    "UnsupportedCompoundSafe": "SAT",
    "DisjunctExpansionSafe": "SAT",
    "AdditionalRestrictiveSafe": "UNSAT",
}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def run(command: list[str], log_path: Path, timeout: int = 600) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False, timeout=timeout,
    )
    log_path.write_text(result.stdout, encoding="utf-8")
    return result


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--tla", type=Path, required=True)
    parser.add_argument("--canonical-cfg", type=Path, required=True)
    parser.add_argument("--negative-cfg-dir", type=Path, required=True)
    parser.add_argument("--alloy", type=Path, required=True)
    parser.add_argument("--runner-class-dir", type=Path, required=True)
    parser.add_argument("--alloy-jar", type=Path, required=True)
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()

    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    evidence: dict[str, Any] = {
        "schema": "mycelix.evidence-attestation-compound-constraint-witness-runtime.v1",
        "status": "RUNNING",
        "controls": [],
        "inputs": {},
    }
    try:
        matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
        require(
            matrix.get("schema") == "mycelix.evidence-attestation-compound-constraint-witness-control-matrix.v1",
            "unexpected control-matrix schema",
        )
        controls = matrix.get("controls")
        require(isinstance(controls, list) and len(controls) == 10, "expected exactly 10 controls")
        ids = [item.get("id") for item in controls]
        require(len(set(ids)) == len(ids), "duplicate control identifiers")
        require(set(ids) == set(PROPERTY_BY_CONTROL), "matrix controls do not match the frozen runner map")
        for item in controls:
            require(item.get("kind") in {"positive", "negative"}, f"invalid kind for {item.get('id')}")
            require(isinstance(item.get("marker"), str) and item["marker"], f"missing marker for {item.get('id')}")

        inputs = [
            args.matrix, args.tla, args.canonical_cfg, args.alloy, args.alloy_jar, args.tla_jar,
        ]
        for path in inputs:
            require(path.is_file(), f"required input missing: {path}")
            evidence["inputs"][str(path)] = sha256_file(path)
        head = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=True, timeout=15,
        ).stdout.strip()
        require(re.fullmatch(r"[0-9a-f]{40}", head) is not None, "could not establish exact source HEAD")
        evidence["source_head"] = head

        # The finite reference oracle must emit each frozen marker exactly at least once.
        reference = args.matrix.parent.parent / "tools" / "formal" / "compound_constraint_witness_reference.py"
        require(reference.is_file(), f"reference oracle missing: {reference}")
        evidence["inputs"][str(reference)] = sha256_file(reference)
        ref_result = run([sys.executable, str(reference)], args.evidence_dir / "reference.stdout.txt")
        require(ref_result.returncode == 0, "reference oracle failed; see reference.stdout.txt")
        for item in controls:
            require(item["marker"] in ref_result.stdout, f"reference omitted marker: {item['marker']}")

        # Canonical TLA+ model is required to hold without an invariant violation.
        canonical_log = args.evidence_dir / "tlc-canonical.stdout.txt"
        canonical_result = run([
            "java", "-cp", str(args.tla_jar), "tlc2.TLC",
            "-config", str(args.canonical_cfg), str(args.tla),
            "-metadir", str(args.evidence_dir / "tlc-canonical-meta"),
        ], canonical_log)
        require(
            canonical_result.returncode == 0
            and "Model checking completed. No error has been found." in canonical_result.stdout,
            "canonical TLA+ configuration did not complete cleanly; see tlc-canonical.stdout.txt",
        )
        evidence["canonical_tla"] = {
            "expected": "NO_INVARIANT_VIOLATION",
            "observed": "NO_INVARIANT_VIOLATION",
            "exit_code": canonical_result.returncode,
            "log_sha256": sha256_file(canonical_log),
        }

        cfg_files = sorted(args.negative_cfg_dir.glob(
            "EvidenceAttestationCompoundConstraintWitnessMatchingV1Negative-*.cfg"
        ))
        expected_cfg_names = {
            "EvidenceAttestationCompoundConstraintWitnessMatchingV1Negative-" + item["id"] + ".cfg"
            for item in controls
        }
        require({path.name for path in cfg_files} == expected_cfg_names,
                "TLA+ control configuration set does not exactly match the frozen matrix")

        for item in controls:
            control_id = item["id"]
            property_name = PROPERTY_BY_CONTROL[control_id]
            cfg = args.negative_cfg_dir / (
                "EvidenceAttestationCompoundConstraintWitnessMatchingV1Negative-" + control_id + ".cfg"
            )
            log_path = args.evidence_dir / ("tlc-" + control_id + ".stdout.txt")
            result = run([
                "java", "-cp", str(args.tla_jar), "tlc2.TLC",
                "-config", str(cfg), str(args.tla),
                "-metadir", str(args.evidence_dir / ("tlc-" + control_id + "-meta")),
            ], log_path)
            has_named_violation = (
                f"Invariant {property_name} is violated" in result.stdout
                or f"Error: Invariant {property_name} is violated" in result.stdout
            )
            if item["kind"] == "negative":
                require(has_named_violation,
                        f"{control_id}: expected the named counterexample for {property_name}; "
                        f"see {log_path.name}")
                observed = "EXPECTED_NAMED_COUNTEREXAMPLE"
            else:
                require(
                    result.returncode == 0
                    and "Model checking completed. No error has been found." in result.stdout,
                    f"{control_id}: positive control failed; see {log_path.name}",
                )
                observed = "NO_INVARIANT_VIOLATION"
            evidence["controls"].append({
                "id": control_id,
                "kind": item["kind"],
                "property": property_name,
                "expected": "NAMED_COUNTEREXAMPLE" if item["kind"] == "negative" else "NO_INVARIANT_VIOLATION",
                "observed": observed,
                "exit_code": result.returncode,
                "log": log_path.name,
                "log_sha256": sha256_file(log_path),
            })

        # Alloy checks must produce precisely the expected SAT/UNSAT result per assertion.
        alloy_log = args.evidence_dir / "alloy.stdout.txt"
        alloy_result = run([
            "java", "-cp", str(args.runner_class_dir) + os.pathsep + str(args.alloy_jar),
            "AgentDelegationAuthorityAlloyRunner", str(args.alloy),
        ], alloy_log)
        require(alloy_result.returncode == 0, "Alloy runner failed; see alloy.stdout.txt")
        records = []
        for line in alloy_result.stdout.splitlines():
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise RuntimeError(f"non-JSON Alloy runner output: {line[:160]}") from error
        require(len(records) == len(ALLOY_EXPECTED), f"expected {len(ALLOY_EXPECTED)} Alloy results, got {len(records)}")
        observed_by_name: dict[str, str] = {}
        for record in records:
            command_text = str(record.get("command", "")) + " " + str(record.get("label", ""))
            matches = [name for name in ALLOY_EXPECTED if re.search(r"\b" + re.escape(name) + r"\b", command_text)]
            require(len(matches) == 1, f"cannot uniquely map Alloy result: {command_text}")
            name = matches[0]
            require(name not in observed_by_name, f"duplicate Alloy result: {name}")
            require(record.get("check") is True, f"expected Alloy check assertion, got non-check command: {name}")
            observed = record.get("actual")
            require(observed in {"SAT", "UNSAT"}, f"invalid Alloy result for {name}: {observed}")
            require(observed == ALLOY_EXPECTED[name],
                    f"Alloy expectation mismatch for {name}: expected {ALLOY_EXPECTED[name]}, got {observed}")
            observed_by_name[name] = observed
        require(set(observed_by_name) == set(ALLOY_EXPECTED), "Alloy assertion set incomplete")
        evidence["alloy"] = {
            "expected_assertions": len(ALLOY_EXPECTED),
            "observed_assertions": len(observed_by_name),
            "results": observed_by_name,
            "log": alloy_log.name,
            "log_sha256": sha256_file(alloy_log),
        }

        evidence["status"] = "PASS"
        evidence["summary"] = {
            "reference": "PASS",
            "tla_canonical": "PASS",
            "tla_controls": len(evidence["controls"]),
            "alloy_assertions": len(observed_by_name),
            "qualification": "NOT_CLAIMED",
        }
        receipt_path = args.evidence_dir / "receipt.json"
        receipt_path.write_text(json.dumps(evidence, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("COMPOUND WITNESS FORMAL MATRIX PASS: reference, TLA+ exact controls, and Alloy expectations match")
        print("QUALIFICATION NOT CLAIMED: bounded research/specification evidence only")
        return 0
    except Exception as error:
        evidence["status"] = "FAIL"
        evidence["error"] = str(error)
        receipt_path = args.evidence_dir / "receipt.json"
        receipt_path.write_text(json.dumps(evidence, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print(f"COMPOUND WITNESS FORMAL MATRIX FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
