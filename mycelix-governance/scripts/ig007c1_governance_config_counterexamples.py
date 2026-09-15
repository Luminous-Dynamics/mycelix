#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

PROFILE_SHA256 = "de4435a69356557b1812f8beb46d654c66b9c957be9d18c64bd0431f92546d5a"
CORPUS_SHA256 = "3009e97529934fa8f470769de5615dcfda17bde0e942683e939d2b733430a216"
PROFILE_ID = "mycelix-governance-config-observed-fca2c107-v1"
SCHEMA = "mycelix-governance-config-counterexamples-v1"


def canonical(obj: object) -> bytes:
    return json.dumps(
        obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        value = json.load(f)
    if not isinstance(value, dict):
        raise ValueError("JSON root must be object")
    return value


def load_c0_validator():
    path = Path(__file__).with_name("validate_ig007c0_governance_config_profile.py")
    spec = importlib.util.spec_from_file_location("ig007c0_validator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load C0 validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def shape_valid(gates: dict[str, float]) -> bool:
    order = ["basic", "proposal", "voting", "constitutional"]
    values = [gates[k] for k in order]
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        return False
    if not all(0.0 <= float(v) <= 1.0 for v in values):
        return False
    return all(float(values[i]) <= float(values[i + 1]) for i in range(len(values) - 1))


def require_profile(profile: dict) -> None:
    validator = load_c0_validator()
    result = validator.validate(profile)
    if result["profile_content_sha256"] != PROFILE_SHA256:
        raise ValueError("wrong C0 profile commitment")
    if profile["profile_id"] != PROFILE_ID or profile["profile_revision"] != 1:
        raise ValueError("wrong C0 profile identity")
    if profile["authority_class"] != "ObservedSourceBound":
        raise ValueError("wrong C0 authority")


def build_corpus(profile: dict) -> dict:
    require_profile(profile)

    observed = profile["observed_update_predicates"]
    integrity = profile["integrity_authorization"]

    if observed["proposal_record_requirement"] != "SomeRecord":
        raise ValueError("CE-CFG-01 premise drift")
    if observed["proposal_status_check"] != "NoneObserved":
        raise ValueError("CE-CFG-01 status premise drift")
    if observed["proposal_type_check"] != "NoneObserved":
        raise ValueError("CE-CFG-01 type premise drift")

    defaults = {"basic": 0.2, "proposal": 0.3, "voting": 0.4, "constitutional": 0.6}
    candidate = {"basic": 0.1, "proposal": 0.3, "voting": 0.4, "constitutional": 0.6}
    if not shape_valid(defaults) or not shape_valid(candidate):
        raise ValueError("CE-CFG-02 gate fixture unexpectedly invalid")
    if not candidate["basic"] < defaults["basic"]:
        raise ValueError("CE-CFG-02 no longer lowers the basic gate")

    if integrity["create_update_validator"] != "check_consciousness_config":
        raise ValueError("CE-CFG-03 validator premise drift")
    if integrity["changed_by_proposal_binding"] != "NoneObserved":
        raise ValueError("CE-CFG-03 proposal-binding premise drift")
    if integrity["entry_author_authorization"] != "NoneObserved":
        raise ValueError("CE-CFG-03 author premise drift")

    payload = {
        "schema": SCHEMA,
        "authority": "MeasurementOnly",
        "profile_ref": {
            "id": PROFILE_ID,
            "revision": 1,
            "content_sha256": PROFILE_SHA256,
        },
        "issue": 943,
        "fixtures": [
            {
                "id": "CE-CFG-01",
                "revision": 1,
                "classification": "SourceContractCounterexample",
                "premises": {
                    "proposal_id": "MIP-DRAFT-FIXTURE",
                    "modeled_proposal_state": "Draft",
                    "proposal_lookup_result": "SomeRecord",
                    "proposal_status_inspected": False,
                    "proposal_type_inspected": False,
                },
                "result": "AuthorizationContinuesAfterExistenceOnly",
                "non_claim": "NoLiveMutationExecuted",
            },
            {
                "id": "CE-CFG-02",
                "revision": 1,
                "classification": "PurePolicyEffectCounterexample",
                "premises": {
                    "default_gates": defaults,
                    "candidate_gates": candidate,
                    "required_shape": ["Finite", "UnitInterval", "Nondecreasing"],
                },
                "result": "StructurallyValidLowerRuntimeGate",
                "non_claim": "NoNormativeThresholdVerdict",
            },
            {
                "id": "CE-CFG-03",
                "revision": 1,
                "classification": "IntegrityAuthorityCounterexample",
                "premises": {
                    "changed_by_proposal": "MIP-FIXTURE",
                    "config_shape": "StructurallyValid",
                    "integrity_validator": "check_consciousness_config",
                    "proposal_authority_reconstruction": "NoneObserved",
                    "entry_author_authorization": "NoneObserved",
                },
                "result": "IntegrityAcceptsShapeWithoutObservedProposalAuthorityPredicate",
                "non_claim": "NoLiveDHTMutationExecuted",
            },
        ],
        "non_claims": [
            "no_live_config_mutation",
            "no_exploit_success",
            "no_deployment_currentness",
            "no_normative_threshold_verdict",
        ],
    }
    digest = hashlib.sha256(canonical(payload)).hexdigest()
    if digest != CORPUS_SHA256:
        raise ValueError(f"corpus commitment drift: {digest}")
    return {**payload, "corpus_sha256": digest}


def self_test(profile: dict) -> dict:
    corpus = build_corpus(profile)
    fixtures = {item["id"]: item for item in corpus["fixtures"]}
    assert set(fixtures) == {"CE-CFG-01", "CE-CFG-02", "CE-CFG-03"}
    assert fixtures["CE-CFG-01"]["result"] == "AuthorizationContinuesAfterExistenceOnly"
    assert fixtures["CE-CFG-02"]["result"] == "StructurallyValidLowerRuntimeGate"
    assert fixtures["CE-CFG-03"]["result"] == "IntegrityAcceptsShapeWithoutObservedProposalAuthorityPredicate"
    assert shape_valid(fixtures["CE-CFG-02"]["premises"]["candidate_gates"])
    assert corpus["authority"] == "MeasurementOnly"
    return {
        "self_test": True,
        "authority": corpus["authority"],
        "profile_sha256": PROFILE_SHA256,
        "corpus_sha256": corpus["corpus_sha256"],
        "counterexample_count": len(corpus["fixtures"]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true")
    mode.add_argument("--corpus", action="store_true")
    args = parser.parse_args()
    profile = load_json(args.profile)
    if args.self_test:
        result = self_test(profile)
    else:
        result = build_corpus(profile)
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False))


if __name__ == "__main__":
    main()
