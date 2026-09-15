#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from validate_ig007cs0_constitution_bridge_sync_profile import load, validate

AUTHORITY = "MeasurementOnly"
SCHEMA = "mycelix-constitution-bridge-sync-counterexamples-v1"
PROFILE_SHA256 = "60daae86044098561fa8e41bcdf6f695ab41234760be4b7e235d2780271b681e"
EXPECTED_CORPUS_SHA256 = "2ca6d79212cfd0acae1e974c8390d564bab06821050d8eacbc32f437630ef60b"


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def build_corpus(profile: dict) -> dict:
    ref = validate(profile)
    if ref["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("CS1 requires exact CS0 profile")

    surface = profile["bridge_surface"]
    sync = profile["constitution_sync"]
    auth = profile["authorization_boundary"]

    corpus = {
        "schema": SCHEMA,
        "authority": AUTHORITY,
        "profile": {
            "profile_id": ref["profile_id"],
            "content_sha256": ref["profile_content_sha256"],
            "authority_class": ref["authority_class"],
        },
        "issues": [943, 944],
        "counterexamples": [
            {
                "id": "CE-CS-01",
                "revision": 1,
                "kind": "TargetEntrypointCensusObservation",
                "inputs": {
                    "target_zome": "governance_bridge",
                    "target_function": "update_phi_config",
                    "bridge_coordinator_module_count": 8,
                },
                "comparison": {
                    "target_symbol_occurrences_in_bound_census": surface[
                        "target_symbol_occurrences_in_bound_census"
                    ],
                    "visible_runtime_config_updater": surface[
                        "visible_runtime_config_updater"
                    ],
                    "result": "TargetEntrypointAbsentFromObservedBridgeCoordinatorCensus",
                },
                "non_claim": (
                    "Records the exact frozen coordinator census; does not claim every deployed "
                    "bridge build has this surface."
                ),
            },
            {
                "id": "CE-CS-02",
                "revision": 1,
                "kind": "BestEffortSynchronizationObservation",
                "inputs": {
                    "constitution_parameter_write": "SucceededBeforeSyncAttempt",
                    "bridge_sync": "UnavailableOrFailed",
                    "call_semantics": "BestEffort",
                },
                "comparison": {
                    "sync_failure_effect": sync["sync_failure_effect"],
                    "runtime_sync_established": False,
                    "result": "ConstitutionParameterSuccessDoesNotEstablishRuntimeConfigSynchronization",
                },
                "non_claim": (
                    "Models source-visible control flow only; no live constitution/runtime "
                    "divergence is asserted."
                ),
            },
            {
                "id": "CE-CS-03",
                "revision": 1,
                "kind": "CrossMechanismAuthorizationBoundary",
                "inputs": {
                    "observed_target": "update_phi_config",
                    "visible_alternative": "update_consciousness_config",
                    "visible_alternative_issue": 943,
                },
                "comparison": {
                    "rename_target_to_visible_updater_is_sufficient_repair": auth[
                        "rename_target_to_visible_updater_is_sufficient_repair"
                    ],
                    "result": "EntrypointRenameAloneWouldBypassSeparateAuthorizationTheorem",
                },
                "non_claim": (
                    "Does not claim the visible updater is unusable after its independent "
                    "authorization theorem is corrected."
                ),
            },
            {
                "id": "CE-CS-04",
                "revision": 1,
                "kind": "ReconciliationEvidenceObservation",
                "inputs": {
                    "explicit_unsynchronized_state_receipt": sync[
                        "explicit_unsynchronized_state_receipt"
                    ],
                    "explicit_retry_reconciliation_contract": sync[
                        "explicit_retry_reconciliation_contract"
                    ],
                },
                "comparison": {
                    "content_bound_reconciliation_evidence": "NotEstablished",
                    "result": "NoObservedContentBoundReconciliationEvidence",
                },
                "non_claim": (
                    "Records this frozen helper's evidence surface; does not prove operational "
                    "reconciliation never occurs elsewhere."
                ),
            },
        ],
        "non_claims": [
            "no_live_runtime_divergence_claim",
            "no_live_config_mutation_claim",
            "no_deployment_exploit",
            "no_deployment_currentness",
            "no_governance_safety_claim",
        ],
    }
    corpus["corpus_sha256"] = hashlib.sha256(canonical(corpus)).hexdigest()
    return corpus


def self_test(profile: dict) -> dict:
    first = build_corpus(profile)
    second = build_corpus(profile)
    if canonical(first) != canonical(second):
        raise AssertionError("non-deterministic constitution bridge sync corpus")
    if first["corpus_sha256"] != EXPECTED_CORPUS_SHA256:
        raise AssertionError(
            f"constitution bridge sync corpus commitment drift: {first['corpus_sha256']}"
        )

    by_id = {item["id"]: item for item in first["counterexamples"]}
    if set(by_id) != {"CE-CS-01", "CE-CS-02", "CE-CS-03", "CE-CS-04"}:
        raise AssertionError("counterexample roster drift")
    assert by_id["CE-CS-01"]["comparison"]["result"] == (
        "TargetEntrypointAbsentFromObservedBridgeCoordinatorCensus"
    )
    assert by_id["CE-CS-02"]["comparison"]["result"] == (
        "ConstitutionParameterSuccessDoesNotEstablishRuntimeConfigSynchronization"
    )
    assert by_id["CE-CS-03"]["comparison"]["result"] == (
        "EntrypointRenameAloneWouldBypassSeparateAuthorizationTheorem"
    )
    assert by_id["CE-CS-04"]["comparison"]["result"] == (
        "NoObservedContentBoundReconciliationEvidence"
    )

    return {
        "authority": AUTHORITY,
        "schema": SCHEMA,
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": EXPECTED_CORPUS_SHA256,
        "counterexample_count": 4,
        "issues": [943, 944],
        "self_test": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--corpus", action="store_true")
    args = parser.parse_args()
    profile = load(args.profile)
    if args.self_test:
        result = self_test(profile)
    elif args.corpus:
        result = build_corpus(profile)
    else:
        parser.error("choose --self-test or --corpus")
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()
