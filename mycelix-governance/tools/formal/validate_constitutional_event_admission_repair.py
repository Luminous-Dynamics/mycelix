#!/usr/bin/env python3
"""Validate MYC-CONST-003D1D-E1AR1 repair-lineage bridge.

This validator does not treat Git commit identities as interchangeable. It binds
qualified E1A to its original D1C/E0 objects, then verifies that each R1 subject
is an exact direct-child repair with the declared file/blob census. Hosted R1
qualifiers must separately prove pinned-rustfmt byte equivalence before a future
successor profile may promote the migration.
"""

from __future__ import annotations

import argparse
import copy
import json
import subprocess
from pathlib import Path
from typing import Any

PROFILE = Path("mycelix-governance/specs/constitutional-event-admission-repair-bridge.v1.json")
OLD_E1A_PROFILE = "mycelix-governance/specs/constitutional-event-admission-crosswalk.v1.json"

EXPECTED_E1A = {
    "semantic_head": "249753017c3781f7a3259061aae16d7e0075e984",
    "verifier_head": "ebe85798557a04c75d0bb9ce6c02349576e12b4f",
    "qualification_run_id": 35360097507,
    "qualification_job_id": 105648954365,
    "artifact_id": 10583687692,
    "artifact_digest": "sha256:4edc41c9172ee221332fe1501cb05291b4f6f9eadaf9d4af84d39ab71810cd2f",
    "conclusion": "success",
    "receipt_inspected": True,
}

EXPECTED_EDGES = {
    "MYC-CONST-003D1C": {
        "old": "47d1d764323dbfaf991b5574cfde83abb7a3e4a4",
        "r1": "90ff00c371d2dd875b9bb7f23f1c5ee4b293f39c",
        "verifier": "a03273ecdc095cd60c5ebfa9a10e9e88bc64dd40",
        "files": {
            "mycelix-governance/crates/constitutional-effect-ledger/src/lib.rs": (
                "bb3b8d6a865bbf514ff1e43a41c520b890a5515d",
                "6f5d6c2a3e8f27cace3b37c23c82b8b4c64c1f0e",
            ),
        },
    },
    "MYC-CONST-003D1D-E0": {
        "old": "36a4fcffb6ca806570ebf439f9c36cf76401e7b2",
        "r1": "ac3f71e37c480a9c6f99578fe2106285fe567a5b",
        "verifier": "42cbfaa5cfb043e124db84f7762e1ca7142a6f08",
        "files": {
            "mycelix-governance/crates/constitutional-event-provider/src/lib.rs": (
                "5d110f95c0d78f494b61a16d5afe5776401820bb",
                "8c57b38e1c304fdfb042485c666bac5dc8d40960",
            ),
            "mycelix-governance/crates/constitutional-event-provider/tests/effect_ledger_identity.rs": (
                "897668782e09fac5ffbc506e1e8b11f653de2950",
                "77dff6764b329c6ba3ce263dbfd34956efb8eac0",
            ),
        },
    },
}

REQUIRED_NONCLAIMS = {
    "not_D1C_R1_qualified",
    "not_E0_R1_qualified",
    "not_Git_SHA_interchangeability",
    "not_retroactive_E1A_mutation",
    "not_E1B_subject_migration",
    "not_Holochain_event_admission_active",
    "not_live_EmitEvent_routing",
    "not_deployment_currentness_qualified",
}

REQUIRED_INVARIANTS = {
    "d1c_to_e0_identity_reused_not_regenerated": True,
    "claim_binding_target_must_bind_exact_event_target": True,
    "claim_binding_payload_must_bind_exact_canonical_payload": True,
    "publisher_must_equal_authenticated_dht_author_did": True,
    "event_entry_must_be_immutable": True,
    "action_key_index_must_be_integrity_validated": True,
    "same_action_same_semantics": "ExistingSame",
    "same_action_conflicting_semantics": "IntegrityConflict",
    "last_write_wins_allowed": False,
    "signal_projection_can_define_constitutional_completion": False,
}

REQUIRED_MIGRATION_RULE = {
    "all_r1_receipts_must_be_exact_head_pass_and_inspected": True,
    "qualification_inheritance_from_old_subjects_allowed": False,
    "git_sha_substitution_allowed": False,
    "e1b_requires_new_revision_for_r1_subjects": True,
    "bridge_can_activate_runtime": False,
}


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    return result.stdout.strip()


def git_show(commit: str, path: str) -> str:
    return git("show", f"{commit}:{path}")


def load_profile(path: Path = PROFILE) -> dict[str, Any]:
    return json.loads(path.read_text())


def edge_map(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    edges = data.get("repair_edges")
    require(isinstance(edges, list), "repair_edges must be an array")
    mapped: dict[str, dict[str, Any]] = {}
    for edge in edges:
        dependency_id = edge.get("dependency_id")
        require(dependency_id not in mapped, f"duplicate repair edge: {dependency_id}")
        mapped[dependency_id] = edge
    return mapped


def validate_shape(data: dict[str, Any]) -> None:
    require(data.get("schema") == "mycelix.constitutional-event-admission-repair-bridge.v1", "wrong schema")
    require(data.get("bridge_id") == "MYC-CONST-003D1D-E1AR1", "wrong bridge_id")
    require(data.get("provider_lane") == "EmitEvent", "wrong provider lane")
    require(data.get("status") == "PendingRepairQualification", "frozen bridge must remain pending")
    require(data.get("activation_allowed") is False, "pending repair bridge cannot activate")

    upstream = data.get("upstream_e1a")
    require(upstream == EXPECTED_E1A, "upstream E1A receipt binding drifted")

    edges = edge_map(data)
    require(set(edges) == set(EXPECTED_EDGES), "repair edge census must be exactly D1C + E0")
    for dependency_id, expected in EXPECTED_EDGES.items():
        edge = edges[dependency_id]
        require(edge.get("old_semantic_head") == expected["old"], f"{dependency_id} old head drift")
        require(edge.get("r1_semantic_head") == expected["r1"], f"{dependency_id} R1 head drift")
        require(edge.get("r1_verifier_head") == expected["verifier"], f"{dependency_id} verifier drift")
        require(edge.get("relation") == "PinnedRustfmtExactBytesRequired", f"{dependency_id} relation weakened")
        require(edge.get("qualification_state") == "PreparedNotHostedQualified", f"{dependency_id} prematurely promoted")
        require(edge.get("qualification_receipt") is None, f"{dependency_id} pending edge must not carry a receipt")

        files = edge.get("files")
        require(isinstance(files, list), f"{dependency_id} files must be an array")
        observed = {item.get("path"): (item.get("old_blob"), item.get("r1_blob")) for item in files}
        require(observed == expected["files"], f"{dependency_id} file/blob census drift")

    require(data.get("carried_forward_invariants") == REQUIRED_INVARIANTS, "carried-forward E1A invariants drifted")
    require(data.get("migration_rule") == REQUIRED_MIGRATION_RULE, "migration rule drifted")
    require(set(data.get("non_claims", [])) == REQUIRED_NONCLAIMS, "non-claim ceiling drifted")


def validate_old_e1a_binding() -> None:
    old = json.loads(git_show(EXPECTED_E1A["semantic_head"], OLD_E1A_PROFILE))
    bindings = {item["id"]: item for item in old["source_bindings"]}
    require(bindings["d1c-effect-ledger"]["commit"] == EXPECTED_EDGES["MYC-CONST-003D1C"]["old"], "E1A D1C head mismatch")
    require(bindings["d1c-effect-ledger"]["git_blob_sha1"] == EXPECTED_EDGES["MYC-CONST-003D1C"]["files"]["mycelix-governance/crates/constitutional-effect-ledger/src/lib.rs"][0], "E1A D1C blob mismatch")
    require(bindings["e0-event-provider"]["commit"] == EXPECTED_EDGES["MYC-CONST-003D1D-E0"]["old"], "E1A E0 head mismatch")
    require(bindings["e0-event-provider"]["git_blob_sha1"] == EXPECTED_EDGES["MYC-CONST-003D1D-E0"]["files"]["mycelix-governance/crates/constitutional-event-provider/src/lib.rs"][0], "E1A E0 provider blob mismatch")
    require(bindings["e0-d1c-identity-test"]["git_blob_sha1"] == EXPECTED_EDGES["MYC-CONST-003D1D-E0"]["files"]["mycelix-governance/crates/constitutional-event-provider/tests/effect_ledger_identity.rs"][0], "E1A E0 mapping-test blob mismatch")
    require(old["identity_mapping"]["status"] == "LocallySourceBound", "E1A identity mapping status drift")
    require(old["identity_mapping"]["qualification_inheritance_allowed"] is False, "E1A must forbid qualification inheritance")


def validate_git_edges(data: dict[str, Any]) -> None:
    edges = edge_map(data)
    for dependency_id, expected in EXPECTED_EDGES.items():
        edge = edges[dependency_id]
        require(git("rev-parse", f"{edge['r1_semantic_head']}^") == edge["old_semantic_head"], f"{dependency_id} R1 is not a direct child of old subject")
        changed = [line for line in git("diff", "--name-only", edge["old_semantic_head"], edge["r1_semantic_head"]).splitlines() if line]
        require(changed == list(expected["files"]), f"{dependency_id} changed-file census mismatch: {changed}")
        require(not any(path.startswith("mycelix-governance/zomes/") for path in changed), f"{dependency_id} repair changed a Holochain zome")
        for path, (old_blob, r1_blob) in expected["files"].items():
            require(git("rev-parse", f"{edge['old_semantic_head']}:{path}") == old_blob, f"{dependency_id} old blob mismatch for {path}")
            require(git("rev-parse", f"{edge['r1_semantic_head']}:{path}") == r1_blob, f"{dependency_id} R1 blob mismatch for {path}")


def validate_source_visible_identity() -> None:
    d1c = git_show(EXPECTED_EDGES["MYC-CONST-003D1C"]["r1"], "mycelix-governance/crates/constitutional-effect-ledger/src/lib.rs")
    for needle in [
        "pub struct ConstitutionalOperation",
        "pub operation_id: String",
        "pub proposal_id: String",
        "pub claim_binding: String",
        "pub struct ActionIntent",
        "pub action_id: String",
        "pub action_commitment: String",
    ]:
        require(needle in d1c, f"D1C-R1 missing source-visible identity field: {needle}")

    e0 = git_show(EXPECTED_EDGES["MYC-CONST-003D1D-E0"]["r1"], "mycelix-governance/crates/constitutional-event-provider/src/lib.rs")
    for needle in [
        "pub struct EventAuthorityBinding",
        "pub operation_id: String",
        "pub action_id: String",
        "pub proposal_id: String",
        "pub claim_binding_commitment: String",
        "pub action_commitment: String",
        "pub fn provider_key(&self) -> &str",
        "&self.authority.action_id",
        "signal projection is not bound to the durable constitutional event",
    ]:
        require(needle in e0, f"E0-R1 missing carried-forward event invariant: {needle}")

    mapping = git_show(EXPECTED_EDGES["MYC-CONST-003D1D-E0"]["r1"], "mycelix-governance/crates/constitutional-event-provider/tests/effect_ledger_identity.rs")
    for needle in [
        "operation_id: operation.operation_id.clone()",
        "action_id: intent.action_id.clone()",
        "proposal_id: operation.proposal_id.clone()",
        "claim_binding_commitment: operation.claim_binding.clone()",
        "action_commitment: intent.action_commitment.clone()",
        "assert_eq!(first.provider_key(), intent.action_id)",
    ]:
        require(needle in mapping, f"E0-R1 mapping test lost identity continuity: {needle}")


def validate(data: dict[str, Any]) -> None:
    validate_shape(data)
    validate_old_e1a_binding()
    validate_git_edges(data)
    validate_source_visible_identity()


def expect_rejected(data: dict[str, Any], label: str) -> None:
    try:
        validate_shape(data)
    except ValidationError:
        return
    raise ValidationError(f"mutation survived: {label}")


def self_test(data: dict[str, Any]) -> None:
    mutant = copy.deepcopy(data)
    mutant["activation_allowed"] = True
    expect_rejected(mutant, "premature activation")

    mutant = copy.deepcopy(data)
    mutant["repair_edges"][0]["r1_semantic_head"] = "0" * 40
    expect_rejected(mutant, "D1C R1 head drift")

    mutant = copy.deepcopy(data)
    mutant["repair_edges"][1]["files"] = mutant["repair_edges"][1]["files"][:1]
    expect_rejected(mutant, "E0 changed-file omission")

    mutant = copy.deepcopy(data)
    mutant["repair_edges"][0]["qualification_state"] = "Qualified"
    mutant["repair_edges"][0]["qualification_receipt"] = "fake-receipt"
    expect_rejected(mutant, "premature qualification promotion")

    mutant = copy.deepcopy(data)
    mutant["upstream_e1a"]["artifact_digest"] = "sha256:" + "0" * 64
    expect_rejected(mutant, "upstream E1A receipt drift")

    mutant = copy.deepcopy(data)
    mutant["migration_rule"]["git_sha_substitution_allowed"] = True
    expect_rejected(mutant, "SHA substitution weakening")

    mutant = copy.deepcopy(data)
    mutant["non_claims"].remove("not_Git_SHA_interchangeability")
    expect_rejected(mutant, "SHA non-claim removal")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    data = load_profile(args.profile)
    validate(data)
    if args.self_test:
        self_test(data)
    print("constitutional event admission repair bridge: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
