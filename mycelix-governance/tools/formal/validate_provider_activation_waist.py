#!/usr/bin/env python3
"""Validate MYC-CONST-003CR2A provider activation waist using stdlib + Git."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import pathlib
import re
import subprocess
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parents[2]
REPO = ROOT.parent
PROFILE = ROOT / "specs/constitutional-provider-activation-waist.v1.json"
SCHEMA = ROOT / "specs/constitutional-provider-activation-waist.v1.schema.json"

EXPECTED_PARENT = {
    "semantic_head": "f4748da5c60686e9611ce68d60fcd24a4bfa239a",
    "manifest_path": "mycelix-governance/specs/constitutional-refinement-crosswalk.v2.json",
    "manifest_git_blob_sha": "2676271d2578ab58b83bec627bcd7b61454aba6a",
    "verifier_head": "25b4942e1be20ef9ce70f55b2b1d15201ec38ca5",
    "qualification_status": "PreparedNotHostedQualified",
    "execution_pr": 1954,
    "parked_run_id": 35458597274,
    "parked_run_conclusion": "skipped",
}
EXPECTED_INPUTS = {
    "c3": {
        "id": "MYC-CONST-003C3",
        "semantic_head": "b9bb91353788aeb858c4e52422a87e2401d60a0e",
        "verifier_head": "18309d680a5d630bbb7f5443ab103ade9239a557",
        "qualification_run_id": 35314384788,
        "artifact_digest": "sha256:d26e566503fc1223d879d69ecdb4cf6978fc62664576ed04973067393c8feaab",
        "model_path": "mycelix-governance/specs/ConstitutionalLifecycleQuiescence.tla",
        "model_git_blob_sha": "8ca29563bb581cc77958653210e84d55b3aeb364",
    },
    "d1a": {
        "id": "MYC-CONST-003D1A",
        "semantic_head": "15b9c89adf0ac3c6c5a73681614d6bfcd368820a",
        "verifier_head": "70fbe906834fdef1ba69a80066dbbbaef200156f",
        "qualification_run_id": 35320314578,
        "artifact_digest": "sha256:ce9d7176bd1040ef552fa587e44ae8801a13b0363c0013d5428e5f95ff60430c",
        "model_path": "mycelix-governance/specs/ConstitutionalEffectOutbox.tla",
        "model_git_blob_sha": "b75edcee5bfc826786148a0455c5b26a8223c38c",
    },
}
EXPECTED_BLOCKERS = {
    "CR2Q": ("f4748da5c60686e9611ce68d60fcd24a4bfa239a", "25b4942e1be20ef9ce70f55b2b1d15201ec38ca5", "PreparedNotHostedQualified"),
    "C3ConcreteRuntimeRefinement": ("b9bb91353788aeb858c4e52422a87e2401d60a0e", "18309d680a5d630bbb7f5443ab103ade9239a557", "NotEstablished"),
    "D1AConcreteRuntimeRefinement": ("15b9c89adf0ac3c6c5a73681614d6bfcd368820a", "70fbe906834fdef1ba69a80066dbbbaef200156f", "NotEstablished"),
    "D1C-R1": ("90ff00c371d2dd875b9bb7f23f1c5ee4b293f39c", "32a4ff8fac9c9479c76998691912a4febc08d7da", "PreparedNotHostedQualified"),
    "E0-R1": ("ac3f71e37c480a9c6f99578fe2106285fe567a5b", "5ffe02a3ee5712886171d0f3320f096756885e44", "PreparedNotHostedQualified"),
}
C3_CLASSES = {
    "HorizonReached", "IntegrityHalt", "ResolvedQuiescence",
    "AwaitingExternalEvidence", "ActiveResolution", "ProtocolStall",
}
D1A_PHASES = {
    "Absent", "Prepared", "EffectPending", "UnknownOutcome",
    "EffectObserved", "ReceiptCommitted", "Aborted", "IntegrityHalted",
}
OBLIGATIONS = {
    "qualified-evidence-is-not-runtime-authority",
    "effect-dispatch-only-on-effect-resolution",
    "stall-and-halt-block-dispatch",
    "external-wait-is-not-completion",
    "bounded-horizon-is-not-authority",
    "unknown-outcome-blocks-retry",
    "reconcile-no-effect-preserves-identity",
    "provider-identity-is-not-regenerated",
}
NONCLAIMS = {
    "not_cr2_qualified",
    "not_c3_concrete_runtime_refinement",
    "not_d1a_concrete_runtime_refinement",
    "not_d1c_r1_qualified",
    "not_e0_r1_qualified",
    "not_provider_qualified",
    "not_holochain_dht_admission",
    "not_live_effect_wiring",
    "not_physical_exactly_once",
    "not_deployment_currentness",
}


def require(cond: bool, message: str) -> None:
    if not cond:
        raise ValueError(message)


def exact_keys(obj: dict[str, Any], expected: set[str], label: str) -> None:
    require(set(obj) == expected, f"{label} key census drift: {sorted(set(obj) ^ expected)}")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def git_blob(head: str, path: str) -> str:
    return git("rev-parse", f"{head}:{path}")


def git_text(head: str, path: str) -> str:
    return subprocess.check_output(["git", "show", f"{head}:{path}"], cwd=REPO, text=True)


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_tla_string_set(text: str, name: str) -> list[str]:
    match = re.search(rf"(?m)^{re.escape(name)}\s*==\s*\{{(?P<body>.*?)^\}}", text, re.DOTALL)
    if not match:
        raise ValueError(f"TLA set not found: {name}")
    values = re.findall(r'"([^"]+)"', match.group("body"))
    require(bool(values), f"TLA set empty: {name}")
    return values


def tla_operator(text: str, name: str) -> str:
    lines = text.splitlines()
    start = None
    header = re.compile(rf"^{re.escape(name)}(?:\([^)]*\))?\s*==")
    any_header = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(?:\([^)]*\))?\s*==")
    for i, line in enumerate(lines):
        if header.match(line):
            start = i
            break
    if start is None:
        raise ValueError(f"TLA operator not found: {name}")
    out = [lines[start]]
    for line in lines[start + 1:]:
        if any_header.match(line) or line.startswith("===="):
            break
        out.append(line)
    return "\n".join(out)


def require_tokens(body: str, tokens: list[str], label: str) -> None:
    for token in tokens:
        require(token in body, f"{label} missing required token: {token}")


def validate_schema_file() -> None:
    schema = json.loads(SCHEMA.read_text())
    require(schema.get("$schema") == "https://json-schema.org/draft/2020-12/schema", "wrong schema draft")
    require(schema.get("$id") == "https://mycelix.dev/schemas/constitutional-provider-activation-waist.v1.schema.json", "wrong schema id")
    require(schema.get("additionalProperties") is False, "root schema must be closed")


def validate_profile(p: dict[str, Any]) -> dict[str, Any]:
    required_top = {
        "schema", "profile_id", "version", "status", "authority_class", "activation_allowed",
        "parent_cr2", "qualified_inputs", "blocking_prerequisites", "identity_rules",
        "c3_activation_contract", "d1a_activation_contract",
        "provider_activation_obligations", "non_claims",
    }
    exact_keys(p, required_top, "profile")
    require(p["schema"] == "mycelix.constitutional-provider-activation-waist.v1", "schema identity drift")
    require(p["profile_id"] == "mycelix.provider-activation-waist.c3-d1a.v1", "profile id drift")
    require(p["version"] == "0.1.0", "version drift")
    require(p["status"] == "draft", "CR2A cannot self-promote")
    require(p["authority_class"] == "InertCrossLineageActivationContract", "authority class drift")
    require(p["activation_allowed"] is False, "CR2A cannot authorize activation")

    exact_keys(p["parent_cr2"], set(EXPECTED_PARENT), "parent_cr2")
    require(p["parent_cr2"] == EXPECTED_PARENT, "parent CR2 evidence drift")
    require(git_blob(EXPECTED_PARENT["semantic_head"], EXPECTED_PARENT["manifest_path"]) == EXPECTED_PARENT["manifest_git_blob_sha"], "CR2 manifest blob drift")

    exact_keys(p["qualified_inputs"], {"c3", "d1a"}, "qualified_inputs")
    for key, expected in EXPECTED_INPUTS.items():
        actual = p["qualified_inputs"][key]
        exact_keys(actual, set(expected), f"qualified_inputs.{key}")
        require(actual == expected, f"{key} qualification evidence drift")
        require(git_blob(expected["semantic_head"], expected["model_path"]) == expected["model_git_blob_sha"], f"{key} model blob drift")

    blockers = p["blocking_prerequisites"]
    require(len(blockers) == len(EXPECTED_BLOCKERS), "blocking prerequisite census drift")
    by_id = {x["id"]: x for x in blockers}
    require(len(by_id) == len(blockers), "duplicate blocking prerequisite")
    require(set(by_id) == set(EXPECTED_BLOCKERS), "blocking prerequisite IDs drift")
    for bid, (semantic, verifier, status) in EXPECTED_BLOCKERS.items():
        item = by_id[bid]
        exact_keys(item, {"id", "semantic_head", "verifier_head", "status"}, f"blocker:{bid}")
        require((item["semantic_head"], item["verifier_head"], item["status"]) == (semantic, verifier, status), f"blocker drift: {bid}")

    rules = p["identity_rules"]
    require(rules["claim_binding_source"] == "QualifiedUpstreamEvidenceOnly", "ClaimBinding source weakened")
    require(rules["operation_identity_source"] == "QualifiedUpstreamEvidenceOnly", "operation identity source weakened")
    require(rules["action_identity_source"] == "QualifiedUpstreamEvidenceOnly", "action identity source weakened")
    for key in ("provider_key_regeneration_allowed", "action_id_regeneration_allowed", "target_rebinding_allowed", "payload_rebinding_allowed"):
        require(rules[key] is False, f"identity/rebinding rule weakened: {key}")

    c3_info = EXPECTED_INPUTS["c3"]
    c3_text = git_text(c3_info["semantic_head"], c3_info["model_path"])
    parsed_classes = parse_tla_string_set(c3_text, "QuiescenceClasses")
    require(set(parsed_classes) == C3_CLASSES, f"C3 source class census drift: {parsed_classes}")

    c3 = p["c3_activation_contract"]
    require(c3["runtime_refinement_status"] == "NotEstablished", "C3 runtime refinement prematurely promoted")
    require(set(c3["quiescence_classes"]) == C3_CLASSES and len(c3["quiescence_classes"]) == len(C3_CLASSES), "profile C3 class census drift")
    dispatch = c3["effect_dispatch"]
    require(dispatch["required_state_class"] == "ActiveResolution", "dispatch state class weakened")
    require(set(dispatch["required_predicates"]) == {"EffectResolutionRequired", "InternalStepEnabled"}, "dispatch predicate gate drift")
    require(dispatch["required_internal_action"] == "ApplyEffect", "dispatch action gate drift")
    require(set(dispatch["blocked_state_classes"]) == C3_CLASSES - {"ActiveResolution"}, "blocked C3 state census drift")
    require(dispatch["horizon_is_bounded_model_artifact"] is True, "bounded horizon promoted to authority")

    expected_c3_tokens = {
        "InternalStepEnabled": ["ENABLED InternalResolutionStep"],
        "EffectResolutionRequired": ["PendingEffect", "effectReady"],
        "ProtocolStall": ["ResolutionObligation", "~InternalStepEnabled"],
        "AwaitingExternalEvidence": ["~ResolutionObligation"],
        "ActiveResolution": ["ResolutionObligation", "InternalStepEnabled"],
        "ApplyEffect": ["PendingEffect", "effectReady", "effect' = winner"],
    }
    require(c3["source_assertions"] == expected_c3_tokens, "C3 source assertion profile drift")
    for name, tokens in expected_c3_tokens.items():
        require_tokens(tla_operator(c3_text, name), tokens, f"C3 {name}")
    state_class = tla_operator(c3_text, "StateClass")
    for label in C3_CLASSES:
        require(label in state_class, f"C3 StateClass missing {label}")

    d1a_info = EXPECTED_INPUTS["d1a"]
    d1a_text = git_text(d1a_info["semantic_head"], d1a_info["model_path"])
    parsed_phases = parse_tla_string_set(d1a_text, "Phases")
    require(set(parsed_phases) == D1A_PHASES, f"D1A source phase census drift: {parsed_phases}")
    d1a = p["d1a_activation_contract"]
    require(d1a["runtime_refinement_status"] == "NotEstablished", "D1A runtime refinement prematurely promoted")
    require(set(d1a["phases"]) == D1A_PHASES and len(d1a["phases"]) == len(D1A_PHASES), "profile D1A phase census drift")
    require(d1a["dispatch"]["claim_work_phase"] == "EffectPending", "ClaimWork phase drift")
    require(d1a["dispatch"]["start_request_phase"] == "EffectPending", "StartRequest phase drift")
    require(set(d1a["dispatch"]["delivery_identity_fields"]) == {"committedOp", "outbox", "inFlight"}, "delivery identity field drift")
    unknown = d1a["unknown_outcome"]
    require(unknown["blind_retry_allowed"] is False, "blind retry allowed under UnknownOutcome")
    require(unknown["completion_allowed"] is False, "completion allowed under UnknownOutcome")
    require(set(unknown["resolution_actions"]) == {"ReconcileSuccess", "ReconcileNoEffect", "ObserveContradiction"}, "unknown outcome resolution census drift")
    require(unknown["retry_reenable_action"] == "ReconcileNoEffect", "retry re-enable action drift")
    require(unknown["retry_reenable_phase"] == "EffectPending", "retry re-enable phase drift")
    ident = d1a["identity"]
    require(ident["delivery_action"] == "ExternalDeliver", "delivery action drift")
    require(ident["stable_operation_identity_required"] is True, "stable operation identity weakened")
    require(ident["logical_exactly_once_is_provider_assumption_not_physical_claim"] is True, "physical exactly-once overclaim")

    source_requirements = {
        "CommitAndEnqueue": ["committedOp' = op", "outbox' = op", "\"EffectPending\""],
        "ClaimWork": ["committedOp = op", "\"EffectPending\"", "worker' = op"],
        "StartRequest": ["committedOp = op", "outbox = op", "\"EffectPending\"", "worker = op", "inFlight' = op"],
        "ExternalDeliver": ["committedOp = op", "outbox = op", "inFlight = op", "externalEffect' = op"],
        "LoseOrTimeoutAck": ["\"EffectPending\"", "\"UnknownOutcome\"", "inFlight = op"],
        "ReconcileSuccess": ["\"UnknownOutcome\"", "\"EffectObserved\"", "externalEffect = op"],
        "ReconcileNoEffect": ["\"UnknownOutcome\"", "externalEffect = NoOp", "\"EffectPending\""],
        "ObserveContradiction": ["\"UnknownOutcome\"", "\"IntegrityHalted\""],
        "RetryDoesNotChangeOperationIdentity": ["outbox = committedOp", "worker = committedOp", "inFlight = committedOp"],
        "UnknownOutcomeBlocksConflictingOp": ["\"UnknownOutcome\"", "committedOp = op"],
        "OutboxBeforeEffect": ["externalEffect # NoOp", "outbox = externalEffect"],
    }
    for name, tokens in source_requirements.items():
        require_tokens(tla_operator(d1a_text, name), tokens, f"D1A {name}")

    obligations = p["provider_activation_obligations"]
    require({x["id"] for x in obligations} == OBLIGATIONS and len(obligations) == len(OBLIGATIONS), "activation obligation census drift")
    require(all(x.get("required") is True for x in obligations), "activation obligation disabled")
    require(set(p["non_claims"]) == NONCLAIMS and len(p["non_claims"]) == len(NONCLAIMS), "non-claim census drift")

    return {
        "c3_model_blob": git_blob(c3_info["semantic_head"], c3_info["model_path"]),
        "c3_classes": parsed_classes,
        "d1a_model_blob": git_blob(d1a_info["semantic_head"], d1a_info["model_path"]),
        "d1a_phases": parsed_phases,
        "obligations": sorted(OBLIGATIONS),
    }


def expect_rejected(base: dict[str, Any], mutate, label: str) -> None:
    candidate = copy.deepcopy(base)
    mutate(candidate)
    try:
        validate_profile(candidate)
    except (ValueError, subprocess.CalledProcessError):
        return
    raise RuntimeError(f"self-test mutant survived: {label}")


def self_test(base: dict[str, Any]) -> list[str]:
    tests = [
        ("drop-effect-resolution", lambda p: p["c3_activation_contract"]["effect_dispatch"]["required_predicates"].remove("EffectResolutionRequired")),
        ("allow-protocol-stall", lambda p: p["c3_activation_contract"]["effect_dispatch"]["blocked_state_classes"].remove("ProtocolStall")),
        ("allow-integrity-halt", lambda p: p["c3_activation_contract"]["effect_dispatch"]["blocked_state_classes"].remove("IntegrityHalt")),
        ("promote-horizon", lambda p: p["c3_activation_contract"]["effect_dispatch"].__setitem__("horizon_is_bounded_model_artifact", False)),
        ("allow-blind-retry", lambda p: p["d1a_activation_contract"]["unknown_outcome"].__setitem__("blind_retry_allowed", True)),
        ("allow-unknown-completion", lambda p: p["d1a_activation_contract"]["unknown_outcome"].__setitem__("completion_allowed", True)),
        ("activate-profile", lambda p: p.__setitem__("activation_allowed", True)),
        ("fake-cr2-qualified", lambda p: p["parent_cr2"].__setitem__("qualification_status", "Qualified")),
        ("regenerate-provider-key", lambda p: p["identity_rules"].__setitem__("provider_key_regeneration_allowed", True)),
        ("regenerate-action-id", lambda p: p["identity_rules"].__setitem__("action_id_regeneration_allowed", True)),
        ("c3-blob-drift", lambda p: p["qualified_inputs"]["c3"].__setitem__("model_git_blob_sha", "0" * 40)),
        ("d1a-blob-drift", lambda p: p["qualified_inputs"]["d1a"].__setitem__("model_git_blob_sha", "0" * 40)),
        ("drop-nonclaim", lambda p: p["non_claims"].remove("not_physical_exactly_once")),
    ]
    killed: list[str] = []
    for label, mutate in tests:
        expect_rejected(base, mutate, label)
        killed.append(label)
    return killed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=pathlib.Path, default=PROFILE)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--out", type=pathlib.Path)
    args = parser.parse_args()

    validate_schema_file()
    profile = json.loads(args.profile.read_text())
    observations = validate_profile(profile)
    killed: list[str] = self_test(profile) if args.self_test else []
    receipt = {
        "schema": "mycelix.constitutional-provider-activation-waist-validation.v1",
        "passed": True,
        "profile_sha256": sha256_file(args.profile),
        "schema_sha256": sha256_file(SCHEMA),
        "observations": observations,
        "self_test_mutants_killed": killed,
        "self_test_count": len(killed),
    }
    rendered = json.dumps(receipt, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered)
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
