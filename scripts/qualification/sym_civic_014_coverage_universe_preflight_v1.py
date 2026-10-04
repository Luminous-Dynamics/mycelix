#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import json
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_014_coverage_universe_preflight_v1.json"

PROGRAM = "SYM-CIVIC-014"
SCHEMA = "mycelix.sym-civic.coverage-universe-preflight.v1"
REJECT = "REJECT_COVERAGE_UNIVERSE_PROVENANCE"
SUFFICIENT = "COVERAGE_UNIVERSE_SUFFICIENT"
INSUFFICIENT = "COVERAGE_UNIVERSE_INSUFFICIENT"
UNRESOLVED = "COVERAGE_UNIVERSE_UNRESOLVED"

PARENT_SUBJECT = "2e1c7ef4e2d0ced913834681e11318104d02de75"
QUALIFICATION_CUT = "2026-10-04T00:00:00Z"
RENDERED_AT = "2026-10-03T01:00:00Z"
APPEAL_DEADLINE = "2026-10-04T00:00:00Z"

EXPECTED_UNIVERSE = {
    "id": "appeal-vds/a",
    "semantic_version": "1.0",
    "namespace": "decision-appeals",
    "decision_id": "decision-a",
    "scope_digest": "sha256:scope-a",
    "canonicalization_version": "canon-v1",
    "closure_basis": "FINITE_MANIFEST",
    "manifest_digest": "sha256:manifest-a",
    "member_count": 3,
}

EXPECTED_SCOPE = {
    "decision_id": "decision-a",
    "decision_identity_digest": "sha256:decision-a",
    "universe_id": "appeal-vds/a",
    "universe_semantic_version": "1.0",
    "relevant_classes": ["appeal"],
    "scope_digest": "sha256:scope-a",
}

EXPECTED_VDS = {
    "producer": "producer-a",
    "log_id": "appeal-vds/a",
}

EXPECTED_EXTENSIONS = {
    "schema_version": "ext-1",
    "critical_namespaces": {
        "coverage-v1": {"mode": "FINITE_MANIFEST"}
    },
}


def canon(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def digest(value):
    return "sha256:" + hashlib.sha256(canon(value).encode("utf-8")).hexdigest()


def ts(value):
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def clone(value):
    return copy.deepcopy(value)


def identity_material(value):
    return {
        key: value[key]
        for key in value
        if key not in {"identity_material", "identity_digest"}
    }


def seal(value):
    value["identity_material"] = identity_material(value)
    value["identity_digest"] = digest(value["identity_material"])


def extension_identity_material(extensions):
    return {
        "schema_version": extensions.get("schema_version"),
        "critical_namespaces": extensions.get("critical_namespaces"),
    }


def extension_identity_digest(extensions):
    return digest(extension_identity_material(extensions))


def make_checkpoint(checkpoint_id, size, state_digest, predecessor_id, issued_at):
    checkpoint = {
        "id": checkpoint_id,
        "size": size,
        "state_digest": state_digest,
        "predecessor_id": predecessor_id,
        "issued_at": issued_at,
        "universe_id": EXPECTED_UNIVERSE["id"],
        "scope_digest": EXPECTED_SCOPE["scope_digest"],
        "vds_producer": EXPECTED_VDS["producer"],
        "vds_log_id": EXPECTED_VDS["log_id"],
    }
    seal(checkpoint)
    return checkpoint


def base_candidate():
    decision = {
        "id": "decision-a",
        "identity_digest": EXPECTED_SCOPE["decision_identity_digest"],
        "rendered_at": RENDERED_AT,
        "appeal_deadline": APPEAL_DEADLINE,
    }

    universe = clone(EXPECTED_UNIVERSE)
    seal(universe)

    scope = clone(EXPECTED_SCOPE)
    scope["universe_identity_digest"] = universe["identity_digest"]
    seal(scope)

    genesis = make_checkpoint(
        "checkpoint-genesis",
        0,
        "sha256:state-0",
        None,
        "2026-10-03T01:00:00Z",
    )
    terminal = make_checkpoint(
        "checkpoint-terminal",
        3,
        "sha256:state-3",
        genesis["id"],
        "2026-10-04T00:00:00Z",
    )
    terminal["predecessor_digest"] = genesis["identity_digest"]
    seal(terminal)

    frontier = {
        "universe_identity_digest": universe["identity_digest"],
        "scope_identity_digest": scope["identity_digest"],
        "vds_producer": EXPECTED_VDS["producer"],
        "vds_log_id": EXPECTED_VDS["log_id"],
        "anchor_checkpoint_id": genesis["id"],
        "terminal_checkpoint_id": terminal["id"],
        "covered_start": RENDERED_AT,
        "covered_cut": QUALIFICATION_CUT,
        "fresh_until": "2026-10-04T01:00:00Z",
        "coverage_mode": "CLOSED_WORLD_MANIFEST",
        "closure_basis": "FINITE_MANIFEST",
        "manifest_digest": EXPECTED_UNIVERSE["manifest_digest"],
        "enumerated_count": 3,
        "non_inclusion_proof": {
            "supported": True,
            "proof_type": "non-inclusion",
            "proof_cut": QUALIFICATION_CUT,
            "coverage_cut": QUALIFICATION_CUT,
            "universe_identity_digest": universe["identity_digest"],
            "scope_identity_digest": scope["identity_digest"],
            "terminal_checkpoint_id": terminal["id"],
            "terminal_checkpoint_digest": terminal["identity_digest"],
            "manifest_digest": EXPECTED_UNIVERSE["manifest_digest"],
            "covers_declared_universe": True,
            "proof_digest": "sha256:proof-a",
            "fresh_until": "2026-10-04T01:00:00Z",
        },
        "continuity_proven": True,
        "consistency_proven": True,
    }
    seal(frontier)

    witness = {
        "witness_id": "witness-a",
        "vds_producer": EXPECTED_VDS["producer"],
        "vds_log_id": EXPECTED_VDS["log_id"],
        "stateful": True,
        "fork_free": True,
        "accepted_anchor_checkpoint_id": genesis["id"],
        "accepted_terminal_checkpoint_id": terminal["id"],
        "last_accepted_checkpoint_id": terminal["id"],
        "countersignature_digest": "sha256:witness-a",
        "witnessed_at": "2026-10-04T00:00:01Z",
    }
    seal(witness)

    extensions = {
        "schema_version": EXPECTED_EXTENSIONS["schema_version"],
        "critical_namespaces": clone(EXPECTED_EXTENSIONS["critical_namespaces"]),
        "noncritical": {"trace": "synthetic"},
    }
    extensions["identity_digest"] = extension_identity_digest(extensions)
    frontier["extension_identity_digest"] = extensions["identity_digest"]
    seal(frontier)

    candidate = {
        "decision": decision,
        "universe": universe,
        "scope": scope,
        "checkpoints": [genesis, terminal],
        "frontier": frontier,
        "witness": witness,
        "extensions": extensions,
        "observed_decisive_items": [],
        "qualification_time": QUALIFICATION_CUT,
    }
    return candidate


def resolve_parent(value, parts):
    node = value
    for part in parts[:-1]:
        if isinstance(node, list):
            node = node[int(part)]
        else:
            node = node[part]
    return node, parts[-1]


def apply_mutation(candidate, mutation):
    op = mutation["op"]
    path = mutation["path"]
    parts = path.split(".")
    parent, leaf = resolve_parent(candidate, parts)

    if op == "replace":
        if isinstance(parent, list):
            parent[int(leaf)] = mutation["value"]
        else:
            parent[leaf] = mutation["value"]
    elif op == "remove":
        if isinstance(parent, list):
            del parent[int(leaf)]
        else:
            parent.pop(leaf, None)
    elif op == "append":
        parent[leaf].append(mutation["value"])
    else:
        raise ValueError(f"unknown mutation op: {op}")


def reseal(candidate):
    extensions = candidate["extensions"]
    extensions["identity_digest"] = extension_identity_digest(extensions)

    seal(candidate["universe"])
    candidate["scope"]["universe_identity_digest"] = candidate["universe"]["identity_digest"]
    seal(candidate["scope"])

    by_id = {checkpoint["id"]: checkpoint for checkpoint in candidate["checkpoints"]}
    for checkpoint in candidate["checkpoints"]:
        if checkpoint.get("predecessor_id") in by_id:
            checkpoint["predecessor_digest"] = by_id[checkpoint["predecessor_id"]]["identity_digest"]
        else:
            checkpoint.pop("predecessor_digest", None)
        seal(checkpoint)

    frontier = candidate["frontier"]
    frontier["universe_identity_digest"] = candidate["universe"]["identity_digest"]
    frontier["scope_identity_digest"] = candidate["scope"]["identity_digest"]
    frontier["extension_identity_digest"] = extensions["identity_digest"]
    proof = frontier.get("non_inclusion_proof")
    if proof is not None:
        terminal_id = frontier.get("terminal_checkpoint_id")
        terminal = by_id.get(terminal_id)
        if terminal is not None:
            proof["terminal_checkpoint_digest"] = terminal["identity_digest"]
        if "covered_cut" in frontier:
            proof["coverage_cut"] = frontier["covered_cut"]
    seal(frontier)

    witness = candidate["witness"]
    seal(witness)


def candidate_for(spec):
    candidate = base_candidate()
    needs_reseal = False
    for mutation in spec.get("mutations", []):
        apply_mutation(candidate, mutation)
        needs_reseal = needs_reseal or mutation.get("reseal", False)
    if needs_reseal:
        reseal(candidate)
    return candidate


def provenance(candidate):
    universe = candidate.get("universe", {})
    scope = candidate.get("scope", {})
    frontier = candidate.get("frontier", {})
    witness = candidate.get("witness", {})
    extensions = candidate.get("extensions", {})

    if universe.get("identity_material") != identity_material(universe):
        return False
    if universe.get("identity_digest") != digest(universe["identity_material"]):
        return False

    exact_universe = all(
        universe.get(key) == EXPECTED_UNIVERSE[key]
        for key in EXPECTED_UNIVERSE
    )
    if not exact_universe:
        return False

    if scope.get("identity_material") != identity_material(scope):
        return False
    if scope.get("identity_digest") != digest(scope["identity_material"]):
        return False
    if any(
        scope.get(key) != EXPECTED_SCOPE[key]
        for key in EXPECTED_SCOPE
    ):
        return False
    if scope.get("universe_identity_digest") != universe.get("identity_digest"):
        return False

    if extensions.get("schema_version") != EXPECTED_EXTENSIONS["schema_version"]:
        return False
    critical = extensions.get("critical_namespaces", {})
    if set(critical) != set(EXPECTED_EXTENSIONS["critical_namespaces"]):
        return False
    if critical != EXPECTED_EXTENSIONS["critical_namespaces"]:
        return False
    if extensions.get("identity_digest") != extension_identity_digest(extensions):
        return False
    if extensions.get("identity_digest") != extension_identity_digest(EXPECTED_EXTENSIONS):
        return False
    if frontier.get("extension_identity_digest") != extensions.get("identity_digest"):
        return False

    checkpoints = candidate.get("checkpoints", [])
    ids = [checkpoint.get("id") for checkpoint in checkpoints]
    if len(ids) != len(set(ids)):
        return False

    for checkpoint in checkpoints:
        if checkpoint.get("identity_material") != identity_material(checkpoint):
            return False
        if checkpoint.get("identity_digest") != digest(checkpoint["identity_material"]):
            return False
        if checkpoint.get("universe_id") != universe["id"]:
            return False
        if checkpoint.get("scope_digest") != scope["scope_digest"]:
            return False
        if checkpoint.get("vds_producer") != EXPECTED_VDS["producer"]:
            return False
        if checkpoint.get("vds_log_id") != EXPECTED_VDS["log_id"]:
            return False

    if frontier.get("identity_material") != identity_material(frontier):
        return False
    if frontier.get("identity_digest") != digest(frontier["identity_material"]):
        return False
    if frontier.get("universe_identity_digest") != universe.get("identity_digest"):
        return False
    if frontier.get("scope_identity_digest") != scope.get("identity_digest"):
        return False
    if frontier.get("vds_producer") != EXPECTED_VDS["producer"]:
        return False
    if frontier.get("vds_log_id") != EXPECTED_VDS["log_id"]:
        return False

    if witness.get("identity_material") != identity_material(witness):
        return False
    if witness.get("identity_digest") != digest(witness["identity_material"]):
        return False
    if witness.get("vds_producer") != EXPECTED_VDS["producer"]:
        return False
    if witness.get("vds_log_id") != EXPECTED_VDS["log_id"]:
        return False

    return True


def checkpoint_map(candidate):
    return {checkpoint["id"]: checkpoint for checkpoint in candidate.get("checkpoints", [])}


def independent_qualify(candidate):
    if not provenance(candidate):
        return REJECT

    universe = candidate["universe"]
    scope = candidate["scope"]
    frontier = candidate["frontier"]
    witness = candidate["witness"]
    checkpoints = checkpoint_map(candidate)
    proof = frontier["non_inclusion_proof"]

    try:
        q = ts(candidate["qualification_time"])
        rendered = ts(candidate["decision"]["rendered_at"])
        deadline = ts(candidate["decision"]["appeal_deadline"])
        covered_cut = ts(frontier["covered_cut"])
        fresh_until = ts(frontier["fresh_until"])
        proof_cut = ts(proof["proof_cut"])
        proof_fresh = ts(proof["fresh_until"])
    except (KeyError, TypeError, ValueError):
        return REJECT

    if q < rendered or q < deadline:
        return INSUFFICIENT

    anchor_id = frontier.get("anchor_checkpoint_id")
    terminal_id = frontier.get("terminal_checkpoint_id")
    if anchor_id not in checkpoints or terminal_id not in checkpoints:
        return INSUFFICIENT

    anchor = checkpoints[anchor_id]
    terminal = checkpoints[terminal_id]

    if proof.get("universe_identity_digest") != universe.get("identity_digest"):
        return REJECT
    if proof.get("scope_identity_digest") != scope.get("identity_digest"):
        return REJECT
    if proof.get("terminal_checkpoint_id") != terminal_id:
        return REJECT
    if proof.get("terminal_checkpoint_digest") != terminal.get("identity_digest"):
        return REJECT
    if proof.get("coverage_cut") != frontier.get("covered_cut"):
        return REJECT
    if proof.get("manifest_digest") != universe.get("manifest_digest"):
        return REJECT

    if candidate.get("observed_decisive_items") != []:
        return INSUFFICIENT

    if anchor.get("predecessor_id") is not None:
        return INSUFFICIENT
    if terminal.get("predecessor_id") != anchor_id:
        return INSUFFICIENT
    if terminal.get("predecessor_digest") != anchor.get("identity_digest"):
        return INSUFFICIENT
    if frontier.get("continuity_proven") is not True:
        return INSUFFICIENT
    if frontier.get("consistency_proven") is not True:
        return INSUFFICIENT

    if witness.get("stateful") is not True:
        return INSUFFICIENT
    if witness.get("fork_free") is not True:
        return INSUFFICIENT
    if witness.get("accepted_anchor_checkpoint_id") != anchor_id:
        return INSUFFICIENT
    if witness.get("accepted_terminal_checkpoint_id") != terminal_id:
        return INSUFFICIENT
    if witness.get("last_accepted_checkpoint_id") != terminal_id:
        return INSUFFICIENT
    if ts(witness["witnessed_at"]) < covered_cut:
        return INSUFFICIENT

    if covered_cut < q:
        return INSUFFICIENT
    if ts(frontier["covered_start"]) > rendered:
        return INSUFFICIENT
    if fresh_until < q or proof_fresh < q or proof_cut < q:
        return INSUFFICIENT

    if proof.get("supported") is not True:
        return INSUFFICIENT
    if proof.get("proof_type") != "non-inclusion":
        return INSUFFICIENT
    if proof.get("covers_declared_universe") is not True:
        return INSUFFICIENT

    mode = frontier.get("coverage_mode")
    closure_basis = frontier.get("closure_basis")

    if mode == "CLOSED_WORLD_MANIFEST" and closure_basis == "FINITE_MANIFEST":
        if universe.get("manifest_digest") != EXPECTED_UNIVERSE["manifest_digest"]:
            return REJECT
        if frontier.get("manifest_digest") != universe.get("manifest_digest"):
            return REJECT
        if frontier.get("enumerated_count") != universe.get("member_count"):
            return INSUFFICIENT
        return SUFFICIENT

    if mode == "CLOSED_WORLD_ENUMERATION" and closure_basis == "CANONICAL_ENUMERATOR":
        if frontier.get("enumerator_digest") != "sha256:enumerator-a":
            return REJECT
        closure = frontier.get("closure_attestation", {})
        if closure.get("independent_witness") is not True:
            return INSUFFICIENT
        if closure.get("output_digest") != "sha256:enumerator-output-a":
            return REJECT
        if closure.get("enumerated_count") != universe.get("member_count"):
            return INSUFFICIENT
        return SUFFICIENT

    if mode == "VDS_NATIVE_NON_INCLUSION" and closure_basis == "VDS_NATIVE":
        return INSUFFICIENT

    return INSUFFICIENT

def primary_qualify(candidate):
    if provenance(candidate) is False:
        return REJECT

    decision = candidate["decision"]
    universe = candidate["universe"]
    scope = candidate["scope"]
    frontier = candidate["frontier"]
    proof = frontier["non_inclusion_proof"]
    witness = candidate["witness"]
    checkpoints = checkpoint_map(candidate)

    try:
        qualification = ts(candidate["qualification_time"])
        rendered = ts(decision["rendered_at"])
        deadline = ts(decision["appeal_deadline"])
        covered_cut = ts(frontier["covered_cut"])
        frontier_fresh = ts(frontier["fresh_until"])
        proof_cut = ts(proof["proof_cut"])
        proof_fresh = ts(proof["fresh_until"])
        witnessed_at = ts(witness["witnessed_at"])
    except (KeyError, TypeError, ValueError):
        return REJECT

    terminal_id = frontier.get("terminal_checkpoint_id")
    anchor_id = frontier.get("anchor_checkpoint_id")
    terminal = checkpoints.get(terminal_id)
    anchor = checkpoints.get(anchor_id)

    # Identity-critical proof binding.
    required_bindings = (
        proof.get("universe_identity_digest") == universe.get("identity_digest"),
        proof.get("scope_identity_digest") == scope.get("identity_digest"),
        proof.get("terminal_checkpoint_id") == terminal_id,
        terminal is not None and proof.get("terminal_checkpoint_digest") == terminal.get("identity_digest"),
        proof.get("coverage_cut") == frontier.get("covered_cut"),
        proof.get("manifest_digest") == universe.get("manifest_digest"),
    )
    if not all(required_bindings):
        return REJECT

    # Coverage must describe the same decision window.
    if qualification < rendered or qualification < deadline:
        return INSUFFICIENT
    if covered_cut < qualification:
        return INSUFFICIENT
    if ts(frontier["covered_start"]) > rendered:
        return INSUFFICIENT
    if qualification > frontier_fresh or qualification > proof_fresh or proof_cut < qualification:
        return INSUFFICIENT

    # The complete path must be anchored.
    if anchor is None or terminal is None:
        return INSUFFICIENT
    if anchor.get("predecessor_id") is not None:
        return INSUFFICIENT
    if terminal.get("predecessor_id") != anchor_id:
        return INSUFFICIENT
    if terminal.get("predecessor_digest") != anchor.get("identity_digest"):
        return INSUFFICIENT
    if frontier.get("continuity_proven") is not True:
        return INSUFFICIENT
    if frontier.get("consistency_proven") is not True:
        return INSUFFICIENT

    # A stateless receipt/consistency proof is not enough to close fork history.
    if witness.get("stateful") is not True:
        return INSUFFICIENT
    if witness.get("fork_free") is not True:
        return INSUFFICIENT
    if witness.get("accepted_anchor_checkpoint_id") != anchor_id:
        return INSUFFICIENT
    if witness.get("accepted_terminal_checkpoint_id") != terminal_id:
        return INSUFFICIENT
    if witness.get("last_accepted_checkpoint_id") != terminal_id:
        return INSUFFICIENT
    if witnessed_at < covered_cut:
        return INSUFFICIENT

    # Non-inclusion must be for this exact closed-world claim.
    if proof.get("supported") is not True or proof.get("proof_type") != "non-inclusion":
        return INSUFFICIENT
    if proof.get("covers_declared_universe") is not True:
        return INSUFFICIENT
    if candidate.get("observed_decisive_items") != []:
        return INSUFFICIENT

    basis = frontier.get("closure_basis")
    mode = frontier.get("coverage_mode")
    if basis == "FINITE_MANIFEST" and mode == "CLOSED_WORLD_MANIFEST":
        return (
            SUFFICIENT
            if frontier.get("enumerated_count") == universe.get("member_count")
            and frontier.get("manifest_digest") == universe.get("manifest_digest")
            else INSUFFICIENT
        )

    if basis == "CANONICAL_ENUMERATOR" and mode == "CLOSED_WORLD_ENUMERATION":
        closure = frontier.get("closure_attestation") or {}
        if frontier.get("enumerator_digest") != "sha256:enumerator-a":
            return REJECT
        if closure.get("output_digest") != "sha256:enumerator-output-a":
            return REJECT
        if closure.get("independent_witness") is not True:
            return INSUFFICIENT
        if closure.get("enumerated_count") != universe.get("member_count"):
            return INSUFFICIENT
        return SUFFICIENT

    return INSUFFICIENT
def main():
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))

    assert document["schema"] == SCHEMA
    assert document["program"] == PROGRAM
    assert document["analysis_role"] == "research_only"
    assert document["parent_subject"] == PARENT_SUBJECT

    cases = document["cases"]
    assert [case["id"] for case in cases] == [f"U-{i:02d}" for i in range(1, 33)]

    for case in cases:
        assert set(case) == {"id", "family", "mutations"}
        lowered = canon(case).lower()
        assert not any(token in lowered for token in (
            "expected_disposition",
            "expected_result",
            "oracle_verdict",
            "candidate_verdict",
        ))

    candidates = {case["id"]: candidate_for(case) for case in cases}
    primary = {case_id: primary_qualify(candidates[case_id]) for case_id in candidates}
    independent = {case_id: independent_qualify(candidates[case_id]) for case_id in candidates}
    assert primary == independent, "independent qualifier disagreement"

    census = {
        REJECT: sum(value == REJECT for value in primary.values()),
        SUFFICIENT: sum(value == SUFFICIENT for value in primary.values()),
        INSUFFICIENT: sum(value == INSUFFICIENT for value in primary.values()),
        UNRESOLVED: sum(value == UNRESOLVED for value in primary.values()),
    }

    assert census == {
        REJECT: 15,
        SUFFICIENT: 1,
        INSUFFICIENT: 16,
        UNRESOLVED: 0,
    }, census

    seed = clone(candidates["U-01"])
    probes = [
        ("noncritical_extension_insertion", lambda x: x["extensions"]["noncritical"].update({"trace2": "synthetic"}), SUFFICIENT, False),
        ("noncritical_extension_removal", lambda x: x["extensions"]["noncritical"].pop("trace"), SUFFICIENT, True),
        ("critical_extension_tamper", lambda x: x["extensions"]["critical_namespaces"]["coverage-v1"].update({"mode": "OPEN_WORLD"}), REJECT, True),
        ("critical_extension_removal", lambda x: x["extensions"]["critical_namespaces"].pop("coverage-v1"), REJECT, True),
        ("unknown_critical_namespace", lambda x: x["extensions"]["critical_namespaces"].update({"future-v2": {"mode": "OTHER"}}), REJECT, True),
        ("extension_schema_version_change", lambda x: x["extensions"].update({"schema_version": "ext-2"}), REJECT, True),
        ("extension_commitment_tamper", lambda x: x["extensions"].update({"identity_digest": "sha256:tampered"}), REJECT, False),
        ("missing_anchor", lambda x: x["frontier"].update({"anchor_checkpoint_id": "missing"}), INSUFFICIENT, True),
        ("witness_fork_signal", lambda x: x["witness"].update({"fork_free": False}), INSUFFICIENT, True),
        ("witness_statefulness_removal", lambda x: x["witness"].update({"stateful": False}), INSUFFICIENT, True),
        ("stale_frontier", lambda x: x["frontier"].update({"covered_cut": "2026-10-03T23:00:00Z"}), INSUFFICIENT, True),
        ("stale_proof", lambda x: x["frontier"]["non_inclusion_proof"].update({"fresh_until": "2026-10-03T23:00:00Z"}), INSUFFICIENT, True),
        ("empty_result_without_manifest_closure", lambda x: x["frontier"].update({"enumerated_count": 0}), INSUFFICIENT, True),
        ("proof_universe_binding_mutation", lambda x: x["frontier"]["non_inclusion_proof"].update({"universe_identity_digest": "sha256:wrong"}), REJECT, True),
        ("proof_scope_binding_mutation", lambda x: x["frontier"]["non_inclusion_proof"].update({"scope_identity_digest": "sha256:wrong"}), REJECT, True),
        ("proof_terminal_binding_mutation", lambda x: x["frontier"]["non_inclusion_proof"].update({"terminal_checkpoint_id": "other"}), REJECT, True),
        ("proof_manifest_binding_mutation", lambda x: x["frontier"]["non_inclusion_proof"].update({"manifest_digest": "sha256:wrong"}), REJECT, True),
        ("proof_cut_mismatch", lambda x: x["frontier"]["non_inclusion_proof"].update({"coverage_cut": "2026-10-03T23:00:00Z"}), REJECT, False),
        ("vds_open_world_mode", lambda x: (
            x["frontier"].update({
                "coverage_mode": "VDS_NATIVE_NON_INCLUSION",
                "closure_basis": "VDS_NATIVE"
            }),
            x["frontier"]["non_inclusion_proof"].update({"covers_declared_universe": True})
        ), INSUFFICIENT, True),
    ]

    for name, mutate, want, needs_reseal in probes:
        candidate = clone(seed)
        mutate(candidate)
        if needs_reseal:
            reseal(candidate)
        got = primary_qualify(candidate)
        assert got == want, (name, got, want)

    enumerator = clone(seed)
    enumerator["frontier"].update({
        "coverage_mode": "CLOSED_WORLD_ENUMERATION",
        "closure_basis": "CANONICAL_ENUMERATOR",
        "enumerator_digest": "sha256:enumerator-a",
        "closure_attestation": {
            "independent_witness": True,
            "output_digest": "sha256:enumerator-output-a",
            "enumerated_count": 3,
        },
    })
    reseal(enumerator)
    assert primary_qualify(enumerator) == SUFFICIENT

    no_soundness = clone(enumerator)
    no_soundness["frontier"]["closure_attestation"]["independent_witness"] = False
    reseal(no_soundness)
    assert primary_qualify(no_soundness) == INSUFFICIENT

    truncated = clone(seed)
    truncated["checkpoints"] = [truncated["checkpoints"][1]]
    assert primary_qualify(truncated) == INSUFFICIENT

    split_frontier = clone(seed)
    split_frontier["frontier"]["terminal_checkpoint_id"] = "checkpoint-terminal"
    split_frontier["witness"]["accepted_terminal_checkpoint_id"] = "checkpoint-other"
    split_frontier["witness"]["last_accepted_checkpoint_id"] = "checkpoint-other"
    reseal(split_frontier)
    assert primary_qualify(split_frontier) == INSUFFICIENT

    receipt = {
        "program": PROGRAM,
        "schema": SCHEMA,
        "census": census,
        "cases": [
            {
                "id": case_id,
                "disposition": primary[case_id],
                "universe_identity": candidates[case_id]["universe"]["identity_digest"],
                "frontier_identity": candidates[case_id]["frontier"]["identity_digest"],
                "witness_identity": candidates[case_id]["witness"]["identity_digest"],
            }
            for case_id in sorted(candidates)
        ],
    }

    print("SYM-CIVIC-014 DERIVED=" + canon(census))
    print("SYM-CIVIC-014 METAMORPHIC=PASS")
    print("SYM-CIVIC-014 GENERATED_RECEIPT=" + digest(receipt))
    print("SYM-CIVIC-014 PASS")


if __name__ == "__main__":
    main()
