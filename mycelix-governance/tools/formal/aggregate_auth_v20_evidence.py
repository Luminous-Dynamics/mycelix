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

EXPECTED_MUTATION_IDS = {
    "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json": (
        "mutations", "mutant_detected", (
            "atom-subsumption-opened",
            "denotation-forced-empty",
            "injective-matcher-reuses-child",
            "witness-core-keeps-redundant-clause",
        ),
    ),
    "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json": (
        "mutations", "mutant_detected", (
            "deny-set-erased",
            "deny-overrides-skipped",
            "allow-overrides-misread",
            "masked-allow-expansion-accepted",
            "deny-removal-gate-bypassed",
            "conflict-rule-flag-forged",
        ),
    ),
    "auth-v20-chain-evidence/delegation-chain-differential.json": (
        "mutations", "detected", (
            "adjacent-edge-validation-skipped",
            "root-anchor-validation-skipped",
            "masked-attenuation-violation-accepted",
            "chain-expansion-status-downgraded",
        ),
    ),
    "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json": (
        "checker_mutations", "independent_detection", (
            "expiry-check-omitted",
            "depth-check-omitted",
            "jti-uniqueness-check-omitted",
            "parent-link-check-omitted",
        ),
    ),
    "auth-v20-key-link-evidence/delegation-chain-key-linkage.json": (
        "mutations", "detected", (
            "issuer-link-check-omitted",
            "private-jwk-rejection-omitted",
            "root-issuer-shape-check-omitted",
        ),
    ),
    "auth-v20-par-hash-evidence/delegation-chain-par-hash.json": (
        "mutations", "detected", (
            "par-hash-comparison-omitted",
            "root-par-hash-rejection-omitted",
            "canonical-signing-input-rejection-omitted",
        ),
    ),
    "auth-v20-compact-jws-evidence/compact-jws-chain.json": (
        "mutations", "mutant_acceptance_observed", (
            "signature-check-omitted",
            "issuer-thumbprint-check-omitted",
            "par-hash-check-omitted",
            "jwk-usage-metadata-checks-omitted",
        ),
    ),
    "auth-v20-pop-evidence/delegation-chain-pop-differential.json": (
        "mutations", "mutant_acceptance_observed", (
            "pop-signature-check-omitted",
            "pop-binding-check-omitted",
            "pop-canonical-payload-check-omitted",
            "pop-replay-consumption-omitted",
            "pop-trusted-clock-check-omitted",
        ),
    ),
    "auth-v20-capability-evidence/aat-capability-subsumption.json": (
        "mutations", "mutant_was_observable", (
            "constraint-subsumption-bypassed",
            "tool-set-attenuation-bypassed",
            "runtime-constraint-bypassed",
            "invocation-shape-bypassed",
        ),
    ),
}


EXPECTED_CAPABILITY_VALIDATION_CONTROL_IDS = (
    "unknown-constraint-extension",
    "unexpected-exact-member",
    "exact-object-value",
    "exact-nonfinite-value",
    "range-bool-bound",
    "range-exclusive-empty",
    "any-empty",
    "constraint-depth-overflow",
    "constraint-clause-overflow",
    "constraint-node-overflow",
    "constraint-value-depth-overflow",
    "constraint-value-node-overflow",
    "tool-count-limit-exceeded",
    "argument-key-limit-exceeded",
    "tool-name-limit-exceeded",
)


EXPECTED_CAPABILITY_INVOCATION_CONTROL_IDS = (
    "exact-match",
    "exact-reject",
    "range-match",
    "range-reject-boundary",
    "one-of-match",
    "not-one-of-reject",
    "contains-match",
    "contains-reject",
    "subset-match",
    "subset-reject",
    "wildcard-match",
    "all-match",
    "all-reject",
    "any-match",
    "any-reject",
    "allowed-invocation",
    "value-rejected",
    "extra-argument-rejected",
    "missing-argument-rejected",
    "unknown-tool-rejected",
    "open-world-tool-allows-args",
    "invocation-value-depth-overflow",
    "invocation-value-node-overflow",
)


EXPECTED_POP_CONTROL_IDS = (
    "valid-constrained-invocation",
    "one-time-pop-jti-replay-rejected",
    "fresh-pop-cannot-reuse-old-jti",
    "concurrent-pop-jti-race",
    "replay-store-unavailable-fails-closed",
    "replay-store-unsafe-parent-fails-closed",
    "replay-store-symlink-fails-closed",
    "caller-supplied-clock-cannot-resurrect-expired-chain",
    "audience-optional-when-unconfigured-and-absent",
    "tampered-pop-signature",
    "wrong-aat-id",
    "wrong-tool-claim",
    "wrong-hta",
    "iat-too-old",
    "iat-too-future",
    "missing-audience",
    "wrong-audience",
    "noncanonical-payload",
    "unsupported-extra-claim",
    "empty-pop-jti",
    "wrong-pop-header-alg",
    "wrong-pop-header-type",
    "unauthorized-invocation-tool",
    "leaf-capability-constraint-violation",
    "unverified-aat-chain",
    "audience-policy-unconfigured",
    "restricted-jcs-float",
)


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
    ("auth-v20-pop-evidence/delegation-chain-pop-differential.json",
     "mycelix.aat-invocation-pop-differential-receipt.v1"),
)

EXPECTED_SOURCE_HASH_FIELDS = {
    "auth-v20-evidence/receipt.json": (
        ("oracle_source_sha256", "mycelix-governance/tools/formal/compound_subsumption_counterexamples.py"),
        ("control_matrix_sha256", "docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_COMPOUND_SUBSUMPTION_COUNTEREXAMPLE_CONTROL_MATRIX_V1.json"),
    ),
    "auth-v20-differential-evidence/receipt.json": (
        ("matrix_sha256", "docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_COMPOUND_SUBSUMPTION_DIFFERENTIAL_MATRIX_V1.json"),
        ("oracle_sha256", "mycelix-governance/tools/formal/compound_subsumption_counterexamples.py"),
        ("checker_sha256", "mycelix-governance/tools/formal/differential_compound_subsumption.py"),
    ),
    "auth-v20-differential-evidence/matrix-mutation-guard.json": (
        ("matrix_sha256", "docs/qualification/SOVEREIGNTY_EVIDENCE_ATTESTATION_COMPOUND_SUBSUMPTION_DIFFERENTIAL_MATRIX_V1.json"),
        ("checker_sha256", "mycelix-governance/tools/formal/test_differential_matrix_freeze.py"),
    ),
    "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json": (
        ("oracle_sha256", "mycelix-governance/tools/formal/compound_subsumption_counterexamples.py"),
        ("differential_checker_sha256", "mycelix-governance/tools/formal/differential_compound_subsumption.py"),
        ("mutation_guard_sha256", "mycelix-governance/tools/formal/test_oracle_mutation_sensitivity.py"),
    ),
    "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json": (
        ("oracle_sha256", "mycelix-governance/tools/formal/compound_subsumption_counterexamples.py"),
        ("mutation_guard_sha256", "mycelix-governance/tools/formal/test_effective_policy_mutation_sensitivity.py"),
    ),
    "auth-v20-chain-evidence/delegation-chain-differential.json": (
        ("chain_evaluator_sha256", "mycelix-governance/tools/formal/delegation_chain_counterexamples.py"),
        ("policy_oracle_sha256", "mycelix-governance/tools/formal/compound_subsumption_counterexamples.py"),
        ("test_harness_sha256", "mycelix-governance/tools/formal/test_delegation_chain_counterexamples.py"),
    ),
    "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json": (
        ("checker_sha256", "mycelix-governance/tools/formal/delegation_chain_claims.py"),
        ("test_sha256", "mycelix-governance/tools/formal/test_delegation_chain_claims.py"),
    ),
    "auth-v20-key-link-evidence/delegation-chain-key-linkage.json": (
        ("checker_sha256", "mycelix-governance/tools/formal/delegation_chain_key_linkage.py"),
        ("test_sha256", "mycelix-governance/tools/formal/test_delegation_chain_key_linkage.py"),
    ),
    "auth-v20-par-hash-evidence/delegation-chain-par-hash.json": (
        ("checker_sha256", "mycelix-governance/tools/formal/delegation_chain_par_hash.py"),
        ("test_sha256", "mycelix-governance/tools/formal/test_delegation_chain_par_hash.py"),
    ),
    "auth-v20-compact-jws-evidence/compact-jws-chain.json": (
        ("checker_sha256", "mycelix-governance/tools/formal/delegation_chain_compact_jws.py"),
        ("test_sha256", "mycelix-governance/tools/formal/test_delegation_chain_compact_jws.py"),
    ),
    "auth-v20-capability-evidence/aat-capability-subsumption.json": (
        ("module_sha256", "mycelix-governance/tools/formal/aat_capability_subsumption.py"),
        ("test_sha256", "mycelix-governance/tools/formal/test_aat_capability_subsumption.py"),
    ),
    "auth-v20-pop-evidence/delegation-chain-pop-differential.json": (
        ("checker_sha256", "mycelix-governance/tools/formal/delegation_chain_pop.py"),
        ("test_sha256", "mycelix-governance/tools/formal/test_delegation_chain_pop.py"),
        ("aat_fixture_sha256", "mycelix-governance/tools/formal/test_delegation_chain_compact_jws.py"),
    ),
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def canonical_json_bytes(value: Any) -> bytes:
    """Match the control runner's canonical_json serialization exactly."""
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=False) + "\n").encode("utf-8")


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
        "artifacts": [],
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
            source_hash_rows = []
            for field_name, source_relative_path in EXPECTED_SOURCE_HASH_FIELDS.get(relative_path, ()):
                source_path = Path(__file__).resolve().parents[3] / source_relative_path
                require(source_path.is_file(),
                        f"{relative_path}: referenced source file is missing: {source_relative_path}")
                actual_source_sha256 = hashlib.sha256(source_path.read_bytes()).hexdigest()
                require(data.get(field_name) == actual_source_sha256,
                        f"{relative_path}: {field_name} does not match checked-out source {source_relative_path}")
                source_hash_rows.append({
                    "field": field_name,
                    "path": source_relative_path,
                    "sha256": actual_source_sha256,
                })
            row = {
                "path": relative_path,
                "schema": data["schema"],
                "status": data["status"],
                "source_head": data["source_head"],
                "qualification": data["qualification"],
                "source_hashes": source_hash_rows,
                "sha256": hashlib.sha256(raw_bytes).hexdigest(),
            }
            receipt["receipts"].append(row)

        aggregate_guard_path = "auth-v20-final-evidence/aggregate-mutation-guard.json"
        aggregate_guard_file = args.evidence_root / aggregate_guard_path
        require(aggregate_guard_file.is_file(),
                "missing aggregator self-test receipt: " + aggregate_guard_path)
        aggregate_guard_bytes = aggregate_guard_file.read_bytes()
        try:
            aggregate_guard = json.loads(aggregate_guard_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ValueError(f"{aggregate_guard_path}: invalid UTF-8 JSON: {error}") from error
        require(isinstance(aggregate_guard, dict),
                f"{aggregate_guard_path}: receipt must be a JSON object")
        validate_receipt(aggregate_guard, "mycelix.auth-v20-aggregate-mutation-test.v1",
                         args.expected_head, aggregate_guard_path)
        expected_aggregator_sha256 = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
        aggregate_test_path = Path(__file__).resolve().with_name("test_aggregate_auth_v20_evidence.py")
        expected_aggregate_test_sha256 = hashlib.sha256(aggregate_test_path.read_bytes()).hexdigest()
        require(aggregate_guard.get("aggregator_sha256") == expected_aggregator_sha256,
                "aggregator self-test receipt is not bound to the checked-out aggregator source")
        require(aggregate_guard.get("test_sha256") == expected_aggregate_test_sha256,
                "aggregator self-test receipt is not bound to the checked-out test source")
        expected_aggregate_guard_mutations = (
            "missing-required-receipt", "wrong-source-head", "qualification-laundered",
            "failed-receipt-hidden", "receipt-source-hash-forged", "receipt-source-hash-missing",
            "schema-downgraded", "corpus-count-weakened",
            "matrix-mutation-count-weakened", "matrix-mutation-inventory-substituted",
            "mutant-detection-count-weakened", "mutation-identity-substituted",
            "duplicate-mutation-id", "compact-jws-mutant-count-weakened",
            "missing-capability-receipt", "capability-mutant-count-weakened",
            "missing-pop-receipt", "pop-mutant-count-weakened",
            "pop-control-identity-substituted",
            "expected-head-malformed", "missing-aggregate-mutation-guard",
            "aggregate-guard-head-mismatch", "aggregate-guard-count-weakened",
            "aggregate-guard-identity-substituted", "missing-control-artifact",
            "tampered-control-result", "tampered-control-input",
            "unexpected-control-artifact", "aggregate-guard-aggregator-hash-forged",
            "aggregate-guard-test-hash-forged",
            "capability-bound-control-count-weakened", "capability-resource-limit-weakened",
            "capability-tool-name-limit-weakened",
            "capability-invocation-control-count-weakened",
            "capability-invocation-resource-limit-weakened",
        )
        aggregate_guard_mutations = aggregate_guard.get("mutations")
        require(isinstance(aggregate_guard_mutations, list),
                "aggregator self-test receipt has no mutation records")
        require([row.get("id") for row in aggregate_guard_mutations if isinstance(row, dict)]
                == list(expected_aggregate_guard_mutations),
                "aggregator self-test mutation identities/order differ from frozen inventory")
        require(all(row.get("rejected") is True for row in aggregate_guard_mutations),
                "aggregator self-test did not reject every required evidence weakening")
        aggregate_guard_summary = aggregate_guard.get("summary", {})
        require(aggregate_guard_summary.get("mutations_attempted") == len(expected_aggregate_guard_mutations),
                "aggregator self-test attempted a different mutation count")
        require(aggregate_guard_summary.get("mutations_rejected") == len(expected_aggregate_guard_mutations),
                "aggregator self-test did not reject all expected mutations")
        receipt["receipts"].append({
            "path": aggregate_guard_path,
            "schema": aggregate_guard["schema"],
            "status": aggregate_guard["status"],
            "source_head": aggregate_guard["source_head"],
            "qualification": aggregate_guard["qualification"],
            "aggregator_sha256": aggregate_guard["aggregator_sha256"],
            "test_sha256": aggregate_guard["test_sha256"],
            "sha256": hashlib.sha256(aggregate_guard_bytes).hexdigest(),
        })

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
        expected_matrix_mutations = (
            "remove-request-value", "reorder-request-values", "widen-amount-domain",
            "weaken-atom-target", "widen-atom-numeric-bound", "remove-atom",
            "change-generation-rule", "lower-expression-count", "lower-pair-count",
            "lower-pair-request-count", "remove-compound-operator", "remove-required-invariant",
            "enable-randomized-generation", "duplicate-atom-identifier",
        )
        require(tuple(matrix_guard.get("mutations_rejected", [])) == expected_matrix_mutations,
                "frozen manifest mutation guard rejected a different mutation inventory")

        expected_mutants = {
            "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json": 4,
            "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json": 6,
            "auth-v20-chain-evidence/delegation-chain-differential.json": 4,
            "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json": 4,
            "auth-v20-key-link-evidence/delegation-chain-key-linkage.json": 3,
            "auth-v20-par-hash-evidence/delegation-chain-par-hash.json": 3,
            "auth-v20-compact-jws-evidence/compact-jws-chain.json": 4,
            "auth-v20-capability-evidence/aat-capability-subsumption.json": 4,
            "auth-v20-pop-evidence/delegation-chain-pop-differential.json": 5,
        }
        for relative_path, count in expected_mutants.items():
            data = json.loads((args.evidence_root / relative_path).read_text(encoding="utf-8"))
            summary = data.get("summary", {})
            observed = summary.get("mutants_detected",
                                   summary.get("checker_mutants_detected"))
            require(observed == count,
                    f"{relative_path}: expected {count} detected mutants, got {observed!r}")
            rows_key, detected_key, expected_ids = EXPECTED_MUTATION_IDS[relative_path]
            rows = data.get(rows_key)
            require(isinstance(rows, list),
                    f"{relative_path}: missing mutation records under {rows_key!r}")
            observed_ids = [row.get("id") for row in rows if isinstance(row, dict)]
            require(observed_ids == list(expected_ids),
                    f"{relative_path}: mutation identities/order differ from the frozen inventory")
            require(all(row.get(detected_key) is True for row in rows),
                    f"{relative_path}: at least one expected mutation lacks a positive detection marker")

        control_raw = json.loads((args.evidence_root / "auth-v20-evidence/receipt.json").read_text(encoding="utf-8"))
        expected_policy_control_ids = (
            "actual-authority-expansion", "structural-false-negative",
            "clause-order-forward", "clause-order-reverse", "unsupported-extension",
            "deny-deletion-expansion", "deny-addition-restriction",
            "deny-removal-no-effective-expansion", "allow-expansion-masked-by-deny",
            "conflict-rule-substitution", "unsupported-extension-in-deny",
        )
        observed_policy_control_ids = [
            row.get("id") for row in control_raw.get("controls", []) if isinstance(row, dict)
        ]
        require(observed_policy_control_ids == list(expected_policy_control_ids),
                "bounded policy oracle control identities/order differ from the frozen inventory")
        control_dir = args.evidence_root / "auth-v20-evidence"
        expected_control_files = {"receipt.json"}
        for control_id in expected_policy_control_ids:
            expected_control_files.add(f"{control_id}.input.json")
            expected_control_files.add(f"{control_id}.json")
        actual_control_files = {item.name for item in control_dir.iterdir() if item.is_file()}
        require(actual_control_files == expected_control_files,
                "raw control artifact inventory incomplete or contains unexpected files; "
                f"missing={sorted(expected_control_files - actual_control_files)}, "
                f"extra={sorted(actual_control_files - expected_control_files)}")
        for row in control_raw["controls"]:
            control_id = row["id"]
            input_path = control_dir / f"{control_id}.input.json"
            result_path = control_dir / f"{control_id}.json"
            input_bytes = input_path.read_bytes()
            result_bytes = result_path.read_bytes()
            try:
                input_value = json.loads(input_bytes.decode("utf-8"))
                result_value = json.loads(result_bytes.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise ValueError(f"{control_id}: raw evidence is not valid UTF-8 JSON: {error}") from error
            computed_input_sha256 = hashlib.sha256(canonical_json_bytes(input_value)).hexdigest()
            require(computed_input_sha256 == row.get("input_sha256"),
                    f"{control_id}: raw scenario does not match receipt input_sha256")
            require(result_value.get("input_sha256") == row.get("input_sha256"),
                    f"{control_id}: result input hash differs from receipt")
            stored_result_sha256 = result_value.pop("result_sha256", None)
            require(stored_result_sha256 == row.get("result_sha256"),
                    f"{control_id}: result hash differs from receipt")
            computed_result_sha256 = hashlib.sha256(canonical_json_bytes(result_value)).hexdigest()
            require(computed_result_sha256 == stored_result_sha256,
                    f"{control_id}: result contents do not match result_sha256")
            require(result_value.get("status") == row.get("observed_status"),
                    f"{control_id}: result status differs from control receipt")
            require(row.get("independent_replay") == "PASS",
                    f"{control_id}: independent replay marker is missing")
            for artifact_path, artifact_bytes in (
                (input_path, input_bytes), (result_path, result_bytes)
            ):
                receipt["artifacts"].append({
                    "path": str(artifact_path.relative_to(args.evidence_root)),
                    "sha256": hashlib.sha256(artifact_bytes).hexdigest(),
                    "bytes": len(artifact_bytes),
                })
        chain_policy = json.loads((args.evidence_root /
            "auth-v20-chain-evidence/delegation-chain-differential.json").read_text(encoding="utf-8"))
        chain_policy_summary = chain_policy.get("summary", {})
        require(chain_policy_summary.get("baseline_chains") == 5,
                "delegation-policy harness did not evaluate all five chain fixtures")
        require(chain_policy_summary.get("baseline_relations") == 24,
                "delegation-policy harness did not evaluate all 24 adjacent/root-anchored relations")

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
        require(compact_jws.get("summary", {}).get("negative_controls") == 40,
                "compact-JWS harness did not execute all 40 negative controls")
        require(compact_jws.get("summary", {}).get("positive_controls") == 4,
                "compact-JWS harness did not verify all four positive-chain profiles")
        require(compact_jws.get("summary", {}).get("signatures_verified") == 13,
                "compact-JWS harness did not verify all thirteen positive-chain signatures")
        capability_result = json.loads((args.evidence_root /
            "auth-v20-capability-evidence/aat-capability-subsumption.json").read_text(encoding="utf-8"))
        capability_summary = capability_result.get("summary", {})
        require(capability_summary.get("constraint_subsumption_controls") == 34,
                "AAT capability harness did not execute all 34 frozen subsumption controls")
        require(capability_summary.get("malformed_or_bound_controls") == 15,
                "AAT capability harness did not execute all 15 malformed/bounded controls")
        observed_capability_validation_ids = [
            row.get("id") for row in capability_result.get("validation_controls", [])
            if isinstance(row, dict)
        ]
        require(observed_capability_validation_ids == list(EXPECTED_CAPABILITY_VALIDATION_CONTROL_IDS),
                "AAT malformed/bound control identities/order differ from the frozen inventory")
        require(capability_summary.get("resource_limits") == {
            "max_constraint_depth": 32,
            "max_constraint_nodes": 512,
            "max_composite_clauses": 128,
            "max_tools_per_token": 256,
            "max_constraints_per_tool": 64,
            "max_constraint_value_depth": 32,
            "max_constraint_value_nodes": 512,
            "max_tool_name_bytes": 256,
            "max_invocation_value_depth": 32,
            "max_invocation_value_nodes": 4096,
        }, "AAT capability resource limits differ from the frozen profile")
        require(capability_summary.get("capability_attenuation_controls") == 8,
                "AAT capability harness did not execute all 8 capability attenuation controls")
        require(capability_summary.get("runtime_and_invocation_controls") == 23,
                "AAT capability harness did not execute all 23 runtime/invocation controls")
        observed_capability_invocation_ids = [
            row.get("id") for row in capability_result.get("invocation_controls", [])
            if isinstance(row, dict)
        ]
        require(observed_capability_invocation_ids == list(EXPECTED_CAPABILITY_INVOCATION_CONTROL_IDS),
                "AAT runtime/invocation control identities/order differ from the frozen inventory")

        pop_result = json.loads((args.evidence_root /
            "auth-v20-pop-evidence/delegation-chain-pop-differential.json").read_text(encoding="utf-8"))
        pop_summary = pop_result.get("summary", {})
        require(pop_summary.get("positive_controls") == 2,
                "AAT PoP harness did not execute both positive invocation profiles")
        require(pop_summary.get("negative_controls") == 25,
                "AAT PoP harness did not execute all 25 denial/replay controls")
        require(pop_summary.get("mutants_detected") == 5,
                "AAT PoP harness did not detect all five omitted-check mutants")
        require(pop_summary.get("replay_jti_consumed") is True,
                "AAT PoP harness did not verify one-time replay consumption")
        observed_pop_control_ids = [
            row.get("id") for row in pop_result.get("controls", []) if isinstance(row, dict)
        ]
        require(observed_pop_control_ids == list(EXPECTED_POP_CONTROL_IDS),
                "AAT PoP control identities/order differ from the frozen inventory")

        receipt["status"] = "PASS_BOUNDED_RESEARCH_EVIDENCE"
        receipt["summary"] = {
            "required_specialist_receipts": len(REQUIRED_RECEIPTS),
            "aggregate_self_test_receipt_included": True,
            "aggregate_self_test_source_hashes_match_checkout": True,
            "total_receipts_hashed": len(receipt["receipts"]),
            "raw_control_artifacts_verified": len(receipt["artifacts"]),
            "all_receipts_status_pass": True,
            "all_receipts_exact_head_match": True,
            "all_specialist_source_hashes_match_checkout": True,
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
            "compact_jws_mutants_detected": 4,
            "aat_capability_mutants_detected": 4,
            "aat_capability_subsumption_controls": 34,
            "aat_capability_malformed_bound_controls": 15,
            "aat_capability_runtime_invocation_controls": 23,
            "aat_capability_resource_limits": {
                "max_constraint_depth": 32,
                "max_constraint_nodes": 512,
                "max_composite_clauses": 128,
                "max_tools_per_token": 256,
                "max_constraints_per_tool": 64,
                "max_constraint_value_depth": 32,
                "max_constraint_value_nodes": 512,
                "max_tool_name_bytes": 256,
                "max_invocation_value_depth": 32,
                "max_invocation_value_nodes": 4096,
            },
            "aat_invocation_pop_mutants_detected": 5,
            "aat_invocation_pop_negative_controls": 25,
            "qualification": "NOT_CLAIMED",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(receipt, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        print("EXACT-HEAD RECEIPT AGGREGATE PASS: 13 receipts (12 specialist + aggregator self-test), one source head")
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
