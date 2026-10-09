#!/usr/bin/env python3
"""Mutation tests for the exact-head evidence receipt aggregator."""
from __future__ import annotations

import contextlib
import copy
import io
import json
import sys
import tempfile
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import aggregate_auth_v20_evidence as aggregator  # noqa: E402

FROZEN_REQUIRED_RECEIPTS = (
    ("auth-v20-evidence/receipt.json", "mycelix.compound-subsumption-counterexample-controls.v1"),
    ("auth-v20-differential-evidence/receipt.json", "mycelix.compound-subsumption-differential-receipt.v1"),
    ("auth-v20-differential-evidence/matrix-mutation-guard.json", "mycelix.differential-matrix-mutation-guard-receipt.v1"),
    ("auth-v20-mutation-evidence/oracle-mutation-sensitivity.json", "mycelix.oracle-mutation-sensitivity-receipt.v1"),
    ("auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json", "mycelix.effective-policy-mutation-sensitivity-receipt.v1"),
    ("auth-v20-chain-evidence/delegation-chain-differential.json", "mycelix.delegation-chain-differential-receipt.v1"),
    ("auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json", "mycelix.delegation-chain-claims-differential-receipt.v1"),
    ("auth-v20-key-link-evidence/delegation-chain-key-linkage.json", "mycelix.delegation-chain-key-linkage-differential-receipt.v1"),
    ("auth-v20-par-hash-evidence/delegation-chain-par-hash.json", "mycelix.par-hash-differential-receipt.v1"),
    ("auth-v20-compact-jws-evidence/compact-jws-chain.json", "mycelix.compact-jws-aat-chain-differential-receipt.v1"),
)
HEAD = "a" * 40


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def synthetic_receipts(root: Path) -> dict[str, dict[str, Any]]:
    objects: dict[str, dict[str, Any]] = {}
    for relative, schema in FROZEN_REQUIRED_RECEIPTS:
        data: dict[str, Any] = {
            "schema": schema,
            "status": "PASS",
            "source_head": HEAD,
            "qualification": "NOT_CLAIMED",
            "summary": {},
        }
        if relative == "auth-v20-evidence/receipt.json":
            data["controls"] = [{} for _ in range(11)]
        elif relative == "auth-v20-differential-evidence/receipt.json":
            data.update({"ordered_pairs_expected": 16384, "ordered_pairs_evaluated": 16384})
            data["summary"]["ordered_pair_request_combinations"] = 524288
        elif relative == "auth-v20-differential-evidence/matrix-mutation-guard.json":
            data["mutation_count"] = 14
        elif relative == "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json":
            data["summary"]["mutants_detected"] = 4
        elif relative == "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json":
            data["summary"]["mutants_detected"] = 6
        elif relative == "auth-v20-chain-evidence/delegation-chain-differential.json":
            data["summary"]["mutants_detected"] = 4
        elif relative == "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json":
            data["summary"].update({"checker_mutants_detected": 4, "invalid_claim_controls": 13})
        elif relative == "auth-v20-key-link-evidence/delegation-chain-key-linkage.json":
            data["summary"].update({"checker_mutants_detected": 3, "adversarial_controls": 8})
        elif relative == "auth-v20-par-hash-evidence/delegation-chain-par-hash.json":
            data["summary"].update({"mutants_detected": 3, "negative_controls": 12})
        elif relative == "auth-v20-compact-jws-evidence/compact-jws-chain.json":
            data["summary"].update({"mutants_detected": 3, "negative_controls": 23, "signatures_verified": 4})
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        objects[relative] = data
    return objects


def run_aggregate(root: Path, output: Path, expected_head: str = HEAD) -> tuple[int, str]:
    old_argv = sys.argv
    stdout = io.StringIO()
    stderr = io.StringIO()
    sys.argv = [
        "aggregate_auth_v20_evidence.py",
        "--evidence-root", str(root),
        "--expected-head", expected_head,
        "--output", str(output),
    ]
    try:
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = aggregator.main()
    finally:
        sys.argv = old_argv
    return code, stdout.getvalue() + stderr.getvalue()


def apply_mutation(root: Path, name: str) -> None:
    target = root / FROZEN_REQUIRED_RECEIPTS[0][0]
    diff = root / FROZEN_REQUIRED_RECEIPTS[1][0]
    matrix = root / FROZEN_REQUIRED_RECEIPTS[2][0]
    policy = root / FROZEN_REQUIRED_RECEIPTS[4][0]
    keylink = root / FROZEN_REQUIRED_RECEIPTS[7][0]
    compact_jws = root / FROZEN_REQUIRED_RECEIPTS[9][0]
    if name == "missing-required-receipt":
        (root / FROZEN_REQUIRED_RECEIPTS[5][0]).unlink()
    elif name == "wrong-source-head":
        data = json.loads(policy.read_text(encoding="utf-8"))
        data["source_head"] = "b" * 40
        policy.write_text(json.dumps(data), encoding="utf-8")
    elif name == "qualification-laundered":
        data = json.loads(target.read_text(encoding="utf-8"))
        data["qualification"] = "PRODUCTION_QUALIFIED"
        target.write_text(json.dumps(data), encoding="utf-8")
    elif name == "failed-receipt-hidden":
        data = json.loads(target.read_text(encoding="utf-8"))
        data["status"] = "FAIL"
        target.write_text(json.dumps(data), encoding="utf-8")
    elif name == "schema-downgraded":
        data = json.loads(keylink.read_text(encoding="utf-8"))
        data["schema"] = "future-unknown-schema"
        keylink.write_text(json.dumps(data), encoding="utf-8")
    elif name == "corpus-count-weakened":
        data = json.loads(diff.read_text(encoding="utf-8"))
        data["ordered_pairs_evaluated"] = 16383
        diff.write_text(json.dumps(data), encoding="utf-8")
    elif name == "matrix-mutation-count-weakened":
        data = json.loads(matrix.read_text(encoding="utf-8"))
        data["mutation_count"] = 13
        matrix.write_text(json.dumps(data), encoding="utf-8")
    elif name == "mutant-detection-count-weakened":
        data = json.loads(keylink.read_text(encoding="utf-8"))
        data["summary"]["checker_mutants_detected"] = 2
        keylink.write_text(json.dumps(data), encoding="utf-8")
    elif name == "compact-jws-mutant-count-weakened":
        data = json.loads(compact_jws.read_text(encoding="utf-8"))
        data["summary"]["mutants_detected"] = 2
        compact_jws.write_text(json.dumps(data), encoding="utf-8")
    elif name == "expected-head-malformed":
        # Applied via run_aggregate rather than altering any synthetic receipt.
        return
    else:
        raise KeyError(name)


def main() -> int:
    parser = __import__("argparse").ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    receipt: dict[str, Any] = {
        "schema": "mycelix.auth-v20-aggregate-mutation-test.v1",
        "status": "RUNNING",
        "qualification": "NOT_CLAIMED",
        "mutations": [],
    }
    try:
        require(tuple(aggregator.REQUIRED_RECEIPTS) == FROZEN_REQUIRED_RECEIPTS,
                "aggregator required receipt inventory differs from independently frozen inventory")
        with tempfile.TemporaryDirectory(prefix="mycelix-auth-v20-aggregate-") as temporary:
            root = Path(temporary) / "valid"
            objects = synthetic_receipts(root)
            success_output = Path(temporary) / "success.json"
            code, output = run_aggregate(root, success_output)
            require(code == 0, "valid synthetic evidence rejected: " + output)
            success = json.loads(success_output.read_text(encoding="utf-8"))
            require(success.get("status") == "PASS_BOUNDED_RESEARCH_EVIDENCE",
                    "aggregate does not use the explicit bounded-evidence status")
            require(success.get("qualification") == "NOT_CLAIMED",
                    "aggregate improperly claimed qualification")
            require(len(success.get("receipts", [])) == len(FROZEN_REQUIRED_RECEIPTS),
                    "aggregate receipt inventory is incomplete")

            mutations = (
                "missing-required-receipt",
                "wrong-source-head",
                "qualification-laundered",
                "failed-receipt-hidden",
                "schema-downgraded",
                "corpus-count-weakened",
                "matrix-mutation-count-weakened",
                "mutant-detection-count-weakened",
                "compact-jws-mutant-count-weakened",
                "expected-head-malformed",
            )
            for name in mutations:
                candidate_root = Path(temporary) / name
                synthetic_receipts(candidate_root)
                apply_mutation(candidate_root, name)
                output_path = Path(temporary) / (name + ".json")
                expected_head = "bad-head" if name == "expected-head-malformed" else HEAD
                code, output = run_aggregate(candidate_root, output_path, expected_head)
                require(code != 0, name + ": aggregate accepted weakened or invalid evidence")
                receipt["mutations"].append({
                    "id": name,
                    "rejected": True,
                    "aggregate_failure_reported": bool(output.strip()),
                })

        receipt["status"] = "PASS"
        receipt["summary"] = {
            "valid_synthetic_aggregate_accepted": True,
            "mutations_attempted": len(receipt["mutations"]),
            "mutations_rejected": sum(row["rejected"] for row in receipt["mutations"]),
            "frozen_required_receipts": len(FROZEN_REQUIRED_RECEIPTS),
            "qualification": "NOT_CLAIMED",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("AGGREGATE MUTATION GUARD PASS: valid fixture accepted; 10 weakening mutations rejected")
        print("QUALIFICATION NOT CLAIMED: synthetic receipt checks do not establish semantic correctness")
        return 0
    except Exception as error:
        receipt["status"] = "FAIL"
        receipt["error"] = str(error)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("AGGREGATE MUTATION GUARD FAIL: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
