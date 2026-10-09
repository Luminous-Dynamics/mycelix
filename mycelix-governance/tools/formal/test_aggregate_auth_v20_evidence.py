#!/usr/bin/env python3
"""Mutation tests for the exact-head evidence receipt aggregator."""
from __future__ import annotations

import contextlib
import copy
import io
import json
import sys
import tempfile
import subprocess
from pathlib import Path
from typing import Any, Callable

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import aggregate_auth_v20_evidence as aggregator  # noqa: E402

FROZEN_CAPABILITY_VALIDATION_IDS = (
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

FROZEN_CAPABILITY_INVOCATION_IDS = (
    "exact-match", "exact-reject", "range-match", "range-reject-boundary",
    "one-of-match", "not-one-of-reject", "contains-match", "contains-reject",
    "subset-match", "subset-reject", "wildcard-match", "all-match", "all-reject",
    "any-match", "any-reject", "allowed-invocation", "value-rejected",
    "extra-argument-rejected", "missing-argument-rejected", "unknown-tool-rejected",
    "open-world-tool-allows-args", "invocation-value-depth-overflow",
    "invocation-value-node-overflow",
)

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
    ("auth-v20-capability-evidence/aat-capability-subsumption.json", "mycelix.aat-capability-subsumption-differential-receipt.v1"),
    ("auth-v20-pop-evidence/delegation-chain-pop-differential.json", "mycelix.aat-invocation-pop-differential-receipt.v1"),
)
FROZEN_MATRIX_MUTATIONS = (
    "remove-request-value", "reorder-request-values", "widen-amount-domain",
    "weaken-atom-target", "widen-atom-numeric-bound", "remove-atom",
    "change-generation-rule", "lower-expression-count", "lower-pair-count",
    "lower-pair-request-count", "remove-compound-operator", "remove-required-invariant",
    "enable-randomized-generation", "duplicate-atom-identifier",
)
FROZEN_POLICY_CONTROL_IDS = (
    "actual-authority-expansion", "structural-false-negative",
    "clause-order-forward", "clause-order-reverse", "unsupported-extension",
    "deny-deletion-expansion", "deny-addition-restriction",
    "deny-removal-no-effective-expansion", "allow-expansion-masked-by-deny",
    "conflict-rule-substitution", "unsupported-extension-in-deny",
)
FROZEN_POP_CONTROL_IDS = (
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
FROZEN_MUTATIONS = {
    "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json": (
        "mutations", "mutant_detected", (
            "atom-subsumption-opened", "denotation-forced-empty",
            "injective-matcher-reuses-child", "witness-core-keeps-redundant-clause",
        )),
    "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json": (
        "mutations", "mutant_detected", (
            "deny-set-erased", "deny-overrides-skipped", "allow-overrides-misread",
            "masked-allow-expansion-accepted", "deny-removal-gate-bypassed",
            "conflict-rule-flag-forged",
        )),
    "auth-v20-chain-evidence/delegation-chain-differential.json": (
        "mutations", "detected", (
            "adjacent-edge-validation-skipped", "root-anchor-validation-skipped",
            "masked-attenuation-violation-accepted", "chain-expansion-status-downgraded",
        )),
    "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json": (
        "checker_mutations", "independent_detection", (
            "expiry-check-omitted", "depth-check-omitted",
            "jti-uniqueness-check-omitted", "parent-link-check-omitted",
        )),
    "auth-v20-key-link-evidence/delegation-chain-key-linkage.json": (
        "mutations", "detected", (
            "issuer-link-check-omitted", "private-jwk-rejection-omitted",
            "root-issuer-shape-check-omitted",
        )),
    "auth-v20-par-hash-evidence/delegation-chain-par-hash.json": (
        "mutations", "detected", (
            "par-hash-comparison-omitted", "root-par-hash-rejection-omitted",
            "canonical-signing-input-rejection-omitted",
        )),
    "auth-v20-compact-jws-evidence/compact-jws-chain.json": (
        "mutations", "mutant_acceptance_observed", (
            "signature-check-omitted", "issuer-thumbprint-check-omitted",
            "par-hash-check-omitted", "jwk-usage-metadata-checks-omitted",
        )),
    "auth-v20-capability-evidence/aat-capability-subsumption.json": (
        "mutations", "mutant_was_observable", (
            "constraint-subsumption-bypassed", "tool-set-attenuation-bypassed",
            "runtime-constraint-bypassed", "invocation-shape-bypassed",
        )),
    "auth-v20-pop-evidence/delegation-chain-pop-differential.json": (
        "mutations", "mutant_acceptance_observed", (
            "pop-signature-check-omitted", "pop-binding-check-omitted",
            "pop-canonical-payload-check-omitted", "pop-replay-consumption-omitted",
            "pop-trusted-clock-check-omitted",
        )),
}


FROZEN_AGGREGATE_MUTATIONS = (
    "missing-required-receipt", "wrong-source-head", "qualification-laundered",
    "failed-receipt-hidden", "receipt-source-hash-forged", "receipt-source-hash-missing",
    "schema-downgraded", "corpus-count-weakened",
    "matrix-mutation-count-weakened", "matrix-mutation-inventory-substituted",
    "mutant-detection-count-weakened", "mutation-identity-substituted",
    "duplicate-mutation-id", "compact-jws-mutant-count-weakened",
    "missing-capability-receipt", "capability-mutant-count-weakened",
    "missing-pop-receipt", "pop-mutant-count-weakened", "pop-control-identity-substituted",
    "expected-head-malformed", "missing-aggregate-mutation-guard",
    "aggregate-guard-head-mismatch", "aggregate-guard-count-weakened",
    "aggregate-guard-identity-substituted", "missing-control-artifact",
    "tampered-control-result", "tampered-control-input", "unexpected-control-artifact",
    "aggregate-guard-aggregator-hash-forged", "aggregate-guard-test-hash-forged",
    "capability-bound-control-count-weakened", "capability-resource-limit-weakened",
    "capability-tool-name-limit-weakened",
    "capability-invocation-control-count-weakened",
    "capability-invocation-resource-limit-weakened",
)
FROZEN_SOURCE_HASH_FIELDS = {
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
HEAD = "a" * 40


def canonical_json_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=False) + "\n").encode("utf-8")


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
        for field_name, source_relative_path in FROZEN_SOURCE_HASH_FIELDS.get(relative, ()):
            source_path = HERE.parents[2] / source_relative_path
            data[field_name] = hashlib.sha256(source_path.read_bytes()).hexdigest()
        if relative == "auth-v20-evidence/receipt.json":
            control_dir = root / "auth-v20-evidence"
            control_dir.mkdir(parents=True, exist_ok=True)
            controls = []
            for name in FROZEN_POLICY_CONTROL_IDS:
                raw_scenario = {"fixture_id": name, "bounded": True}
                input_bytes = canonical_json_bytes(raw_scenario)
                input_sha = hashlib.sha256(input_bytes).hexdigest()
                result_value = {
                    "fixture_id": name,
                    "status": "CONTROL_PASS",
                    "input_sha256": input_sha,
                }
                result_sha = hashlib.sha256(canonical_json_bytes(result_value)).hexdigest()
                result_value["result_sha256"] = result_sha
                (control_dir / f"{name}.input.json").write_text(
                    json.dumps(raw_scenario, sort_keys=True, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8",
                )
                (control_dir / f"{name}.json").write_text(
                    json.dumps(result_value, sort_keys=True, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8",
                )
                controls.append({
                    "id": name,
                    "expected_status": "CONTROL_PASS",
                    "observed_status": "CONTROL_PASS",
                    "independent_replay": "PASS",
                    "input_sha256": input_sha,
                    "result_sha256": result_sha,
                })
            data["controls"] = controls
        elif relative == "auth-v20-differential-evidence/receipt.json":
            data.update({"ordered_pairs_expected": 16384, "ordered_pairs_evaluated": 16384})
            data["summary"]["ordered_pair_request_combinations"] = 524288
        elif relative == "auth-v20-differential-evidence/matrix-mutation-guard.json":
            data["mutation_count"] = 14
            data["mutations_rejected"] = list(FROZEN_MATRIX_MUTATIONS)
        elif relative == "auth-v20-mutation-evidence/oracle-mutation-sensitivity.json":
            data["summary"]["mutants_detected"] = 4
        elif relative == "auth-v20-policy-mutation-evidence/effective-policy-mutation-sensitivity.json":
            data["summary"]["mutants_detected"] = 6
        elif relative == "auth-v20-chain-evidence/delegation-chain-differential.json":
            data["summary"].update({"mutants_detected": 4, "baseline_chains": 5, "baseline_relations": 24})
        elif relative == "auth-v20-chain-claims-evidence/delegation-chain-claims-differential.json":
            data["summary"].update({"checker_mutants_detected": 4, "invalid_claim_controls": 13})
        elif relative == "auth-v20-key-link-evidence/delegation-chain-key-linkage.json":
            data["summary"].update({"checker_mutants_detected": 3, "adversarial_controls": 8})
        elif relative == "auth-v20-par-hash-evidence/delegation-chain-par-hash.json":
            data["summary"].update({"mutants_detected": 3, "negative_controls": 12})
        elif relative == "auth-v20-compact-jws-evidence/compact-jws-chain.json":
            data["summary"].update({"mutants_detected": 4, "negative_controls": 40, "positive_controls": 4, "signatures_verified": 13})
        elif relative == "auth-v20-capability-evidence/aat-capability-subsumption.json":
            data["summary"].update({
                "constraint_subsumption_controls": 34,
                "malformed_or_bound_controls": 15,
                "capability_attenuation_controls": 8,
                "runtime_and_invocation_controls": 23,
                "mutants_detected": 4,
                "resource_limits": {
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
            })
            data["validation_controls"] = [
                {"id": name, "rejected": True, "finding": "synthetic-expected-finding"}
                for name in FROZEN_CAPABILITY_VALIDATION_IDS
            ]
            data["invocation_controls"] = [
                {"id": name, "expected_accept": False, "observed_accept": False,
                 "independent_replay": "PASS"}
                for name in FROZEN_CAPABILITY_INVOCATION_IDS
            ]
        elif relative == "auth-v20-pop-evidence/delegation-chain-pop-differential.json":
            data["summary"].update({
                "positive_controls": 2,
                "negative_controls": 25,
                "mutants_detected": 5,
                "replay_jti_consumed": True,
            })
            data["controls"] = [{"id": name} for name in FROZEN_POP_CONTROL_IDS]
        if relative in FROZEN_MUTATIONS:
            rows_key, detected_key, names = FROZEN_MUTATIONS[relative]
            data[rows_key] = [{"id": name, detected_key: True} for name in names]
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        objects[relative] = data

    aggregate_guard_path = root / "auth-v20-final-evidence/aggregate-mutation-guard.json"
    aggregate_guard_path.parent.mkdir(parents=True, exist_ok=True)
    guard = {
        "schema": "mycelix.auth-v20-aggregate-mutation-test.v1",
        "status": "PASS",
        "source_head": HEAD,
        "qualification": "NOT_CLAIMED",
        "aggregator_sha256": hashlib.sha256(Path(aggregator.__file__).resolve().read_bytes()).hexdigest(),
        "test_sha256": hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest(),
        "mutations": [{"id": name, "rejected": True} for name in FROZEN_AGGREGATE_MUTATIONS],
        "summary": {
            "mutations_attempted": len(FROZEN_AGGREGATE_MUTATIONS),
            "mutations_rejected": len(FROZEN_AGGREGATE_MUTATIONS),
            "qualification": "NOT_CLAIMED",
        },
    }
    aggregate_guard_path.write_text(json.dumps(guard, sort_keys=True, indent=2) + "\n", encoding="utf-8")
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
    capability_receipt = root / FROZEN_REQUIRED_RECEIPTS[10][0]
    pop_receipt = root / FROZEN_REQUIRED_RECEIPTS[11][0]
    aggregate_guard = root / "auth-v20-final-evidence/aggregate-mutation-guard.json"
    control_dir = root / "auth-v20-evidence"
    control_input = control_dir / "actual-authority-expansion.input.json"
    control_result = control_dir / "actual-authority-expansion.json"
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
    elif name == "receipt-source-hash-forged":
        data = json.loads(policy.read_text(encoding="utf-8"))
        data["oracle_sha256"] = "0" * 64
        policy.write_text(json.dumps(data), encoding="utf-8")
    elif name == "receipt-source-hash-missing":
        data = json.loads(policy.read_text(encoding="utf-8"))
        del data["oracle_sha256"]
        policy.write_text(json.dumps(data), encoding="utf-8")
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
    elif name == "matrix-mutation-inventory-substituted":
        data = json.loads(matrix.read_text(encoding="utf-8"))
        data["mutations_rejected"][0] = "fabricated-mutation"
        matrix.write_text(json.dumps(data), encoding="utf-8")
    elif name == "mutant-detection-count-weakened":
        data = json.loads(keylink.read_text(encoding="utf-8"))
        data["summary"]["checker_mutants_detected"] = 2
        keylink.write_text(json.dumps(data), encoding="utf-8")
    elif name == "mutation-identity-substituted":
        data = json.loads(keylink.read_text(encoding="utf-8"))
        data["mutations"][0]["id"] = "fabricated-mutation-id"
        keylink.write_text(json.dumps(data), encoding="utf-8")
    elif name == "duplicate-mutation-id":
        data = json.loads(keylink.read_text(encoding="utf-8"))
        data["mutations"][1]["id"] = data["mutations"][0]["id"]
        keylink.write_text(json.dumps(data), encoding="utf-8")
    elif name == "compact-jws-mutant-count-weakened":
        data = json.loads(compact_jws.read_text(encoding="utf-8"))
        data["summary"]["mutants_detected"] = 2
        compact_jws.write_text(json.dumps(data), encoding="utf-8")
    elif name == "missing-capability-receipt":
        capability_receipt.unlink()
    elif name == "capability-mutant-count-weakened":
        data = json.loads(capability_receipt.read_text(encoding="utf-8"))
        data["summary"]["mutants_detected"] = 3
        capability_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "missing-pop-receipt":
        pop_receipt.unlink()
    elif name == "pop-mutant-count-weakened":
        data = json.loads(pop_receipt.read_text(encoding="utf-8"))
        data["summary"]["mutants_detected"] = 3
        pop_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "pop-control-identity-substituted":
        data = json.loads(pop_receipt.read_text(encoding="utf-8"))
        data["controls"][0]["id"] = "fabricated-pop-control"
        pop_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "missing-aggregate-mutation-guard":
        aggregate_guard.unlink()
    elif name == "aggregate-guard-head-mismatch":
        data = json.loads(aggregate_guard.read_text(encoding="utf-8"))
        data["source_head"] = "b" * 40
        aggregate_guard.write_text(json.dumps(data), encoding="utf-8")
    elif name == "aggregate-guard-count-weakened":
        data = json.loads(aggregate_guard.read_text(encoding="utf-8"))
        data["summary"]["mutations_rejected"] = len(FROZEN_AGGREGATE_MUTATIONS) - 1
        aggregate_guard.write_text(json.dumps(data), encoding="utf-8")
    elif name == "aggregate-guard-identity-substituted":
        data = json.loads(aggregate_guard.read_text(encoding="utf-8"))
        data["mutations"][0]["id"] = "fabricated-aggregate-mutant"
        aggregate_guard.write_text(json.dumps(data), encoding="utf-8")
    elif name == "aggregate-guard-aggregator-hash-forged":
        data = json.loads(aggregate_guard.read_text(encoding="utf-8"))
        data["aggregator_sha256"] = "0" * 64
        aggregate_guard.write_text(json.dumps(data), encoding="utf-8")
    elif name == "aggregate-guard-test-hash-forged":
        data = json.loads(aggregate_guard.read_text(encoding="utf-8"))
        data["test_sha256"] = "0" * 64
        aggregate_guard.write_text(json.dumps(data), encoding="utf-8")
    elif name == "capability-bound-control-count-weakened":
        data = json.loads(capability_receipt.read_text(encoding="utf-8"))
        data["validation_controls"] = data["validation_controls"][:-1]
        capability_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "capability-resource-limit-weakened":
        data = json.loads(capability_receipt.read_text(encoding="utf-8"))
        data["summary"]["resource_limits"]["max_constraints_per_tool"] = 256
        capability_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "capability-tool-name-limit-weakened":
        data = json.loads(capability_receipt.read_text(encoding="utf-8"))
        data["summary"]["resource_limits"]["max_tool_name_bytes"] = 1024
        capability_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "capability-invocation-control-count-weakened":
        data = json.loads(capability_receipt.read_text(encoding="utf-8"))
        data["invocation_controls"] = data["invocation_controls"][:-1]
        capability_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "capability-invocation-resource-limit-weakened":
        data = json.loads(capability_receipt.read_text(encoding="utf-8"))
        data["summary"]["resource_limits"]["max_invocation_value_nodes"] = 8192
        capability_receipt.write_text(json.dumps(data), encoding="utf-8")
    elif name == "missing-control-artifact":
        control_input.unlink()
    elif name == "tampered-control-result":
        data = json.loads(control_result.read_text(encoding="utf-8"))
        data["status"] = "FAKE_PASS"
        control_result.write_text(json.dumps(data), encoding="utf-8")
    elif name == "tampered-control-input":
        data = json.loads(control_input.read_text(encoding="utf-8"))
        data["bounded"] = False
        control_input.write_text(json.dumps(data), encoding="utf-8")
    elif name == "unexpected-control-artifact":
        (control_dir / "unexpected-artifact.txt").write_text("extra\n", encoding="utf-8")
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
        receipt["source_head"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], text=True, capture_output=True,
            check=True, timeout=15,
        ).stdout.strip()
        receipt["aggregator_sha256"] = hashlib.sha256(
            (HERE / "aggregate_auth_v20_evidence.py").read_bytes()
        ).hexdigest()
        receipt["test_sha256"] = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
        require(tuple(aggregator.REQUIRED_RECEIPTS) == FROZEN_REQUIRED_RECEIPTS,
                "aggregator required receipt inventory differs from independently frozen inventory")
        require(tuple(aggregator.EXPECTED_POP_CONTROL_IDS) == FROZEN_POP_CONTROL_IDS,
                "aggregator PoP control identity inventory differs from independent frozen inventory")
        require(aggregator.EXPECTED_SOURCE_HASH_FIELDS == FROZEN_SOURCE_HASH_FIELDS,
                "aggregator source-hash field/path inventory differs from independent frozen inventory")
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
            require(len(success.get("receipts", [])) == len(FROZEN_REQUIRED_RECEIPTS) + 1,
                    "aggregate receipt inventory is incomplete")
            expected_control_artifact_names = {
                f"auth-v20-evidence/{name}{suffix}"
                for name in FROZEN_POLICY_CONTROL_IDS
                for suffix in (".input.json", ".json")
            }
            artifact_rows = success.get("artifacts", [])
            require({row.get("path") for row in artifact_rows} == expected_control_artifact_names,
                    "aggregate artifact hash inventory does not match all raw/result control files")
            for row in artifact_rows:
                artifact_path = root / row["path"]
                artifact_bytes = artifact_path.read_bytes()
                require(hashlib.sha256(artifact_bytes).hexdigest() == row.get("sha256"),
                        "aggregate recorded an incorrect artifact SHA-256: " + row["path"])
                require(len(artifact_bytes) == row.get("bytes"),
                        "aggregate recorded an incorrect artifact byte count: " + row["path"])

            mutations = (
                "missing-required-receipt",
                "wrong-source-head",
                "qualification-laundered",
                "failed-receipt-hidden",
                "receipt-source-hash-forged",
                "receipt-source-hash-missing",
                "schema-downgraded",
                "corpus-count-weakened",
                "matrix-mutation-count-weakened",
                "matrix-mutation-inventory-substituted",
                "mutant-detection-count-weakened",
                "mutation-identity-substituted",
                "duplicate-mutation-id",
                "compact-jws-mutant-count-weakened",
                "missing-capability-receipt",
                "capability-mutant-count-weakened",
                "missing-pop-receipt",
                "pop-mutant-count-weakened",
                "pop-control-identity-substituted",
                "expected-head-malformed",
                "missing-aggregate-mutation-guard",
                "aggregate-guard-head-mismatch",
                "aggregate-guard-count-weakened",
                "aggregate-guard-identity-substituted",
                "missing-control-artifact",
                "tampered-control-result",
                "tampered-control-input",
                "unexpected-control-artifact",
                "aggregate-guard-aggregator-hash-forged",
                "aggregate-guard-test-hash-forged",
                "capability-bound-control-count-weakened",
                "capability-resource-limit-weakened",
                "capability-tool-name-limit-weakened",
                "capability-invocation-control-count-weakened",
                "capability-invocation-resource-limit-weakened",
            )
            require(tuple(mutations) == FROZEN_AGGREGATE_MUTATIONS,
                    "executed aggregate mutations differ from independently frozen inventory")
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
        print("AGGREGATE MUTATION GUARD PASS: valid fixture accepted; 35 weakening mutations rejected")
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
