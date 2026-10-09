#!/usr/bin/env python3
"""Aggregate exact-head receipts without elevating them to qualification.

This gate ensures all independent bounded checkers emitted PASS receipts for
the identical source commit, and it verifies frozen expected corpus counts.
The aggregate is deliberately named PASS_BOUNDED_RESEARCH_EVIDENCE and always
carries qualification=NOT_CLAIMED.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

REQUIRED_RECEIPTS = (
    ("auth-v20-evidence/receipt.json",
     "mycelix.compound-subsumption-counterexample-controls.v1"),
    ("auth-v20-differential-evidence/receipt.json",
     "mycelix.compound-subsumption-differential-receipt.v1"),
    ("auth-v20-differential-evidence/matrix-mutation-guard.json",
     "mycelix.differential-matrix-mutation-guard-receipt.v1"),
    ("auth-v20-mutation-evidence/oracle-mutation-sensitivity.json",
     "mycelix.oracle-mutation-sensitivity-receipt.v1"),
    ("auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json",
     "mycelix.effective-policy-mutation-sensitivity-receipt.v1"),
    ("auth-v20-chain-evidence/delegation-chain-differential.json",
     "mycelix.delegation-chain-differential-receipt.v1"),
    ("auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json",
     "mycelix.delegation-chain-claims-differential-receipt.v1"),
    ("auth-v20-key-link-evidence/delegation-chain-key-linkage.json",
     "mycelix.delegation-chain-key-linkage-differential-receipt.v1"),
    ("auth-v20-par-hash-evidence/delegation-chain-par-hash.json",
     "mycelix.par-hash-differential-receipt.v1"),
    ("auth-v20-compact-jws-evidence/compact-jws-chain.json",
     "mycelix.compact-jws-aat-chain-differential-receipt.v1"),
    ("auth-v20-capability-evidence/aat-capability-subsumption.json",
     "mycelix.aat-capability-subsumption-differential-receipt.v1"),
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_receipt(data: dict[str, Any], expected_schema: str,
                     expected_head: str, relative_path: str) -> None:
    require(data.get("schema") == expected_schema,
            f"{relative_path}: schema mismatch ({data.get('schema')!r})")
    require(data.get("status") == "PASS",
            f"{relative_path}: status is not PASS ({data.get('status')!r})")
    require(data.get("source_head") == expected_head,
            f"{relative_path}: source_head differs from exact checkout head")
    require(data.get("qualification") == "NOT_CLAIMED",
            f"{relative_path}: receipt must explicitly retain qualification=NOT_CLAIMED")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--expected-head", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    receipt: dict[str, Any] = {
        "schema": "mycelix.auth-v20-exact-head-evidence-aggregate.v1",
        "status": "RUNNING",
        "source_head": args.expected_head,
        "qualification": "NOT_CLAIMED",
        "receipts": [],
        "failures": [],
    }
    try:
        require(len(args.expected_head) == 40 and all(
            char in "0123456789abcdef" for char in args.expected_head
        ), "expected head must be a lowercase 40-character Git SHA")
        if args.expected_head == "":
            raise ValueError("expected head is required")
        for relative_path, schema in REQUIRED_RECEIPTS:
            path = args.evidence_root / relative_path
            require(path.is_file(), f"missing required receipt: {relative_path}")
            raw_bytes = path.read_bytes()
            try:
                data = json.loads(raw_bytes.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise ValueError(f"{relative_path}: invalid UTF-8 JSON: {error}") from error
            require(isinstance(data, dict), f"{relative_path}: receipt must be a JSON object")
            validate_receipt(data, schema, args.expected_head, relative_path)
            row = {
                "path": relative_path,
                "schema": data["schema"],
                "status": data["status"],
                "source_head": data["source_head"],
                "qualification": data["qualification"],
                "sha256": hashlib.sha256(raw_bytes).hexdigest(),
            }
            receipt["receipts"].append(row)

        differential_path = "auth-v20-differential-evidence/receipt.json"
        differential_raw = json.loads((args.evidence_root / differential_path).read_text(encoding="utf-8"))
        require(differential_raw.get("ordered_pairs_expected") == 16384,
                "differential corpus ordered-pair count differs from 16,384")
        require(differential_raw.get("ordered_pairs_evaluated") == 16384,
                "differential corpus did not evaluate all 16,384 ordered pairs")
        require(differential_raw.get("summary", {}).get("ordered_pair_request_combinations") == 524288,
                "differential corpus combination count differs from 524,288")

        matrix_guard = json.loads((args.evidence_root /
            "auth-v20-differential-evidence/matrix-mutation-guard.json").read_text(encoding="utf-8"))
        require(matrix_guard.get("mutation_count") == 14,
                "frozen manifest mutation guard did not reject its 14 required weakening mutations")

        expected_mutants = {
            "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json": 4,
            "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json": 6,
            "auth-v20-chain-evidence/delegation-chain-differential.json": 4,
            "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json": 4,
            "auth-v20-key-link-evidence/delegation-chain-key-linkage.json": 3,
            "auth-v20-par-hash-evidence/delegation-chain-par-hash.json": 3,
            "auth-v20-compact-jws-evidence/compact-jws-chain.json": 3,
            "auth-v20-capability-evidence/aat-capability-subsumption.json": 4,
        }
        for relative_path, count in expected_mutants.items():
            data = json.loads((args.evidence_root / relative_path).read_text(encoding="utf-8"))
            summary = data.get("summary", {})
            observed = summary.get("mutants_detected",
                                   summary.get("checker_mutants_detected"))
            require(observed == count,
                    f"{relative_path}: expected {count} detected mutants, got {observed!r}")

        control_raw = json.loads((args.evidence_root / "auth-v20-evidence/receipt.json").read_text(encoding="utf-8"))
        require(len(control_raw.get("controls", [])) == 11,
                "bounded policy oracle did not execute all 11 frozen controls")
        chain_claims = json.loads((args.evidence_root /
            "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json").read_text(encoding="utf-8"))
        require(chain_claims.get("summary", {}).get("invalid_claim_controls") == 13,
                "chain-claims harness did not execute all 13 invalid/malformed controls")
        key_link = json.loads((args.evidence_root /
            "auth-v20-key-link-evidence/delegation-chain-key-linkage.json").read_text(encoding="utf-8"))
        require(key_link.get("summary", {}).get("adversarial_controls") == 8,
                "key-linkage harness did not execute all 8 adversarial controls")
        par_hash = json.loads((args.evidence_root /
            "auth-v20-par-hash-evidence/delegation-chain-par-hash.json").read_text(encoding="utf-8"))
        require(par_hash.get("summary", {}).get("negative_controls") == 12,
                "par-hash harness did not execute all 12 negative controls")

        compact_jws = json.loads((args.evidence_root /
            "auth-v20-compact-jws-evidence/compact-jws-chain.json").read_text(encoding="utf-8"))
        require(compact_jws.get("summary", {}).get("negative_controls") == 30,
                "compact-JWS harness did not execute all 30 negative controls")
        require(compact_jws.get("summary", {}).get("positive_controls") == 2,
                "compact-JWS harness did not verify both four-token and single-token positive chains")
        require(compact_jws.get("summary", {}).get("signatures_verified") == 5,
                "compact-JWS harness did not verify all five positive-chain signatures")
        capability_result = json.loads((args.evidence_root /
            "auth-v20-capability-evidence/aat-capability-subsumption.json").read_text(encoding="utf-8"))
        capability_summary = capability_result.get("summary", {})
        require(capability_summary.get("constraint_subsumption_controls") == 34,
                "AAT capability harness did not execute all 34 frozen subsumption controls")
        require(capability_summary.get("malformed_or_bound_controls") == 9,
                "AAT capability harness did not execute all 9 malformed/bounded controls")
        require(capability_summary.get("capability_attenuation_controls") == 8,
                "AAT capability harness did not execute all 8 capability attenuation controls")
        require(capability_summary.get("runtime_and_invocation_controls") == 21,
                "AAT capability harness did not execute all 21 runtime/invocation controls")

        receipt["status"] = "PASS_BOUNDED_RESEARCH_EVIDENCE"
        receipt["summary"] = {
            "required_receipts": len(REQUIRED_RECEIPTS),
            "all_receipts_status_pass": True,
            "all_receipts_exact_head_match": True,
            "all_receipts_qualification_not_claimed": True,
            "compound_policy_controls": 11,
            "differential_ordered_pairs": 16384,
            "differential_pair_request_combinations": 524288,
            "manifest_weakening_mutations_rejected": 14,
            "oracle_mutants_detected": 4,
            "effective_policy_mutants_detected": 6,
            "delegation_policy_chain_mutants_detected": 4,
            "delegation_claim_mutants_detected": 4,
            "jwk_linkage_mutants_detected": 3,
            "par_hash_mutants_detected": 3,
            "compact_jws_mutants_detected": 3,
            "aat_capability_mutants_detected": 4,
            "aat_capability_subsumption_controls": 34,
            "aat_capability_runtime_invocation_controls": 21,
            "qualification": "NOT_CLAIMED",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("EXACT-HEAD RECEIPT AGGREGATE PASS: 11 receipts, one source head")
        print("BOUNDED EVIDENCE ONLY: production qualification remains NOT_CLAIMED")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["failures"].append(str(error))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("EXACT-HEAD RECEIPT AGGREGATE FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
