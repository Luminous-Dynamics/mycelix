#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_015_witness_set_preflight_v1.json"

PROGRAM = "SYM-CIVIC-015"
SCHEMA = "mycelix.sym-civic.witness-set-preflight.v1"
REJECT = "REJECT_WITNESS_SET_PROVENANCE"
SUFFICIENT = "WITNESS_SET_SUFFICIENT"
INSUFFICIENT = "WITNESS_SET_INSUFFICIENT"
UNRESOLVED = "WITNESS_SET_UNRESOLVED"

PARENT_SUBJECT = "38218c2cdebb520b32cf908166d3f4177a7976a5"
CUT = "2026-10-04T00:00:00Z"

UNIVERSE = {"id": "appeal-vds/a", "version": "1.0", "scope_digest": "sha256:scope-a"}
SCOPE = {"decision_id": "decision-a", "universe_id": "appeal-vds/a", "scope_digest": "sha256:scope-a"}
POLICY = {
    "policy_id": "witness-policy/a",
    "policy_version": "1.0",
    "required_witness_count": 2,
    "required_distinct_operating_parties": 2,
    "required_distinct_failure_domains": 2,
    "required_independence_groups": 2,
}
EXTENSIONS = {
    "schema_version": "witness-ext-1",
    "critical_namespaces": {"witness-set-v1": {"mode": "K_OF_N_INDEPENDENT_GROUPS"}},
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
        k: v for k, v in value.items()
        if k not in {"identity_material", "identity_digest"}
    }


def seal(value):
    value["identity_material"] = identity_material(value)
    value["identity_digest"] = digest(value["identity_material"])


def witness_set_material(ws):
    return {
        "policy": ws["policy"],
        "witnesses": ws["witnesses"],
        "extensions": {
            "schema_version": ws["extensions"].get("schema_version"),
            "critical_namespaces": ws["extensions"].get("critical_namespaces"),
        },
    }


def seal_witness_set(ws):
    ws["identity_material"] = witness_set_material(ws)
    ws["identity_digest"] = digest(ws["identity_material"])


def checkpoint(cid, state, predecessor):
    value = {
        "id": cid,
        "state_digest": state,
        "predecessor_id": predecessor,
        "universe_id": UNIVERSE["id"],
        "scope_digest": SCOPE["scope_digest"],
        "log_id": UNIVERSE["id"],
    }
    seal(value)
    return value


def make_witness(wid, key, verification_ref, operating_party, failure_domain, group, terminal, universe, scope):
    value = {
        "witness_id": wid,
        "public_key_digest": key,
        "verification_method_ref": verification_ref,
        "operating_party_id": operating_party,
        "failure_domain": failure_domain,
        "independence_group": group,
        "universe_identity_digest": universe["identity_digest"],
        "scope_identity_digest": scope["identity_digest"],
        "log_id": UNIVERSE["id"],
        "accepted_terminal_checkpoint_id": terminal["id"],
        "accepted_terminal_checkpoint_digest": terminal["identity_digest"],
        "last_accepted_checkpoint_id": terminal["id"],
        "stateful": True,
        "fork_free": True,
        "witnessed_at": CUT,
    }
    seal(value)
    return value


def extension_digest(ext):
    return digest({
        "schema_version": ext.get("schema_version"),
        "critical_namespaces": ext.get("critical_namespaces"),
    })


def base_candidate():
    universe = clone(UNIVERSE)
    seal(universe)

    scope = clone(SCOPE)
    scope["universe_identity_digest"] = universe["identity_digest"]
    seal(scope)

    genesis = checkpoint("checkpoint-genesis", "sha256:state-0", None)
    terminal = checkpoint("checkpoint-terminal", "sha256:state-3", genesis["id"])
    terminal["predecessor_digest"] = genesis["identity_digest"]
    seal(terminal)
    future = checkpoint("checkpoint-future", "sha256:state-4", terminal["id"])
    future["predecessor_digest"] = terminal["identity_digest"]
    seal(future)

    policy = clone(POLICY)
    seal(policy)
    extensions = clone(EXTENSIONS)
    extensions["noncritical"] = {"trace": "synthetic"}
    extensions["identity_digest"] = extension_digest(extensions)

    ws = {
        "policy": policy,
        "witnesses": [
            make_witness("w1", "sha256:key-1", "vm-key-1", "party-a", "failure-domain-a", "group-1", terminal, universe, scope),
            make_witness("w2", "sha256:key-2", "vm-key-2", "party-b", "failure-domain-b", "group-2", terminal, universe, scope),
            make_witness("w3", "sha256:key-3", "vm-key-3", "party-c", "failure-domain-c", "group-3", terminal, universe, scope),
        ],
        "extensions": extensions,
    }
    seal_witness_set(ws)

    frontier = {
        "terminal_checkpoint_id": terminal["id"],
        "terminal_checkpoint_digest": terminal["identity_digest"],
        "covered_cut": CUT,
        "witness_set_identity_digest": ws["identity_digest"],
    }
    seal(frontier)

    return {
        "decision": {
            "id": "decision-a",
            "rendered_at": "2026-10-03T01:00:00Z",
            "appeal_deadline": CUT,
        },
        "universe": universe,
        "scope": scope,
        "checkpoints": [genesis, terminal, future],
        "frontier": frontier,
        "witness_set": ws,
        "qualification_time": CUT,
    }


def apply_mutation(candidate, mutation):
    node = candidate
    parts = mutation["path"].split(".")
    for part in parts[:-1]:
        node = node[int(part)] if isinstance(node, list) else node[part]
    leaf = parts[-1]
    if mutation["op"] == "replace":
        node[int(leaf) if isinstance(node, list) else leaf] = mutation["value"]
    elif mutation["op"] == "remove":
        if isinstance(node, list):
            del node[int(leaf)]
        else:
            node.pop(leaf, None)
    else:
        raise ValueError(mutation["op"])


def reseal(candidate):
    universe = candidate["universe"]
    scope = candidate["scope"]
    seal(universe)
    scope["universe_identity_digest"] = universe["identity_digest"]
    seal(scope)

    by_id = {c["id"]: c for c in candidate["checkpoints"]}
    for c in candidate["checkpoints"]:
        predecessor = by_id.get(c.get("predecessor_id"))
        if predecessor is not None:
            c["predecessor_digest"] = predecessor["identity_digest"]
        else:
            c.pop("predecessor_digest", None)
        c["universe_id"] = universe["id"]
        c["scope_digest"] = scope["scope_digest"]
        seal(c)

    ws = candidate["witness_set"]
    ws["policy"]["identity_material"] = identity_material(ws["policy"])
    ws["policy"]["identity_digest"] = digest(ws["policy"]["identity_material"])
    for w in ws["witnesses"]:
        w["universe_identity_digest"] = universe["identity_digest"]
        w["scope_identity_digest"] = scope["identity_digest"]
        accepted = by_id.get(w.get("accepted_terminal_checkpoint_id"))
        if accepted is not None:
            w["accepted_terminal_checkpoint_digest"] = accepted["identity_digest"]
        seal(w)

    ws["extensions"]["identity_digest"] = extension_digest(ws["extensions"])
    seal_witness_set(ws)

    frontier = candidate["frontier"]
    frontier["witness_set_identity_digest"] = ws["identity_digest"]
    terminal = by_id.get(frontier.get("terminal_checkpoint_id"))
    if terminal is not None:
        frontier["terminal_checkpoint_digest"] = terminal["identity_digest"]
    seal(frontier)


def candidate_for(spec):
    candidate = base_candidate()
    reseal_needed = False
    for mutation in spec.get("mutations", []):
        apply_mutation(candidate, mutation)
        reseal_needed = reseal_needed or mutation.get("reseal", False)
    if reseal_needed:
        reseal(candidate)
    return candidate


def provenance(candidate):
    universe = candidate["universe"]
    scope = candidate["scope"]
    frontier = candidate["frontier"]
    ws = candidate["witness_set"]
    policy = ws["policy"]
    ext = ws["extensions"]
    witnesses = ws["witnesses"]

    if universe.get("identity_digest") != digest(identity_material(universe)):
        return False
    if any(universe.get(k) != UNIVERSE[k] for k in UNIVERSE):
        return False
    if scope.get("identity_digest") != digest(identity_material(scope)):
        return False
    if any(scope.get(k) != SCOPE[k] for k in SCOPE):
        return False
    if scope.get("universe_identity_digest") != universe.get("identity_digest"):
        return False

    if policy.get("identity_digest") != digest(identity_material(policy)):
        return False
    if any(policy.get(k) != POLICY[k] for k in POLICY):
        return False

    if ext.get("identity_digest") != extension_digest(ext):
        return False
    if ext.get("schema_version") != EXTENSIONS["schema_version"]:
        return False
    if ext.get("critical_namespaces") != EXTENSIONS["critical_namespaces"]:
        return False

    if ws.get("identity_digest") != digest(witness_set_material(ws)):
        return False
    if frontier.get("identity_digest") != digest(identity_material(frontier)):
        return False
    if frontier.get("witness_set_identity_digest") != ws.get("identity_digest"):
        return False

    ids = [w.get("witness_id") for w in witnesses]
    keys = [w.get("public_key_digest") for w in witnesses]
    methods = [w.get("verification_method_ref") for w in witnesses]
    if any(not w.get("verification_method_ref") or not w.get("operating_party_id") for w in witnesses):
        return False
    if len(ids) != len(set(ids)) or len(keys) != len(set(keys)) or len(methods) != len(set(methods)):
        return False

    for w in witnesses:
        if w.get("identity_digest") != digest(identity_material(w)):
            return False
        if w.get("universe_identity_digest") != universe.get("identity_digest"):
            return False
        if w.get("scope_identity_digest") != scope.get("identity_digest"):
            return False
        if w.get("log_id") != universe.get("id"):
            return False

    return True


def primary_qualify(candidate):
    if not provenance(candidate):
        return REJECT

    policy = candidate["witness_set"]["policy"]
    frontier = candidate["frontier"]
    q = ts(candidate["qualification_time"])
    if q < ts(candidate["decision"]["rendered_at"]) or q < ts(candidate["decision"]["appeal_deadline"]):
        return INSUFFICIENT
    if ts(frontier["covered_cut"]) < q:
        return INSUFFICIENT

    terminal_id = frontier["terminal_checkpoint_id"]
    terminal_digest = frontier["terminal_checkpoint_digest"]
    matching = []

    for w in candidate["witness_set"]["witnesses"]:
        if w.get("operating_party_id") == "producer-a":
            continue
        if w.get("stateful") is not True or w.get("fork_free") is not True:
            continue
        if ts(w["witnessed_at"]) < ts(frontier["covered_cut"]):
            continue
        if w.get("last_accepted_checkpoint_id") != w.get("accepted_terminal_checkpoint_id"):
            continue
        if w.get("accepted_terminal_checkpoint_id") == terminal_id and w.get("accepted_terminal_checkpoint_digest") == terminal_digest:
            matching.append(w)

    parties = {w["operating_party_id"] for w in matching}
    groups = {w["independence_group"] for w in matching}
    if (
        len(matching) >= policy["required_witness_count"]
        and len(parties) >= policy["required_distinct_operating_parties"]
        and len({w["failure_domain"] for w in matching}) >= policy["required_distinct_failure_domains"]
        and len(groups) >= policy["required_independence_groups"]
    ):
        return SUFFICIENT
    return INSUFFICIENT


def independent_qualify(candidate):
    if not provenance(candidate):
        return REJECT

    q = ts(candidate["qualification_time"])
    frontier = candidate["frontier"]
    policy = candidate["witness_set"]["policy"]
    if q < ts(candidate["decision"]["rendered_at"]) or q < ts(candidate["decision"]["appeal_deadline"]):
        return INSUFFICIENT
    if ts(frontier["covered_cut"]) < q:
        return INSUFFICIENT

    terminal_id = frontier["terminal_checkpoint_id"]
    terminal_digest = frontier["terminal_checkpoint_digest"]
    matching = []
    for w in candidate["witness_set"]["witnesses"]:
        if w.get("operating_party_id") == "producer-a":
            continue
        if w.get("stateful") is not True or w.get("fork_free") is not True:
            continue
        if w.get("last_accepted_checkpoint_id") != w.get("accepted_terminal_checkpoint_id"):
            continue
        if ts(w["witnessed_at"]) < ts(frontier["covered_cut"]):
            continue
        if w.get("accepted_terminal_checkpoint_id") != terminal_id:
            continue
        if w.get("accepted_terminal_checkpoint_digest") != terminal_digest:
            continue
        matching.append(w)

    return (
        SUFFICIENT
        if len(matching) >= policy["required_witness_count"]
        and len({w["operating_party_id"] for w in matching}) >= policy["required_distinct_operating_parties"]
        and len({w["failure_domain"] for w in matching}) >= policy["required_distinct_failure_domains"]
        and len({w["independence_group"] for w in matching}) >= policy["required_independence_groups"]
        else INSUFFICIENT
    )


def main():
    doc = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert doc["schema"] == SCHEMA
    assert doc["program"] == PROGRAM
    assert doc["analysis_role"] == "research_only"
    assert doc["parent_subject"] == PARENT_SUBJECT

    cases = doc["cases"]
    assert [c["id"] for c in cases] == [f"W-{i:02d}" for i in range(1, 20)]
    for c in cases:
        assert set(c) == {"id", "family", "mutations"}
        lowered = canon(c).lower()
        assert not any(t in lowered for t in ("expected_disposition", "expected_result", "oracle_verdict", "candidate_verdict"))

    candidates = {c["id"]: candidate_for(c) for c in cases}
    primary = {i: primary_qualify(c) for i, c in candidates.items()}
    independent = {i: independent_qualify(c) for i, c in candidates.items()}
    assert primary == independent, "independent qualifier disagreement"

    census = {
        REJECT: sum(v == REJECT for v in primary.values()),
        SUFFICIENT: sum(v == SUFFICIENT for v in primary.values()),
        INSUFFICIENT: sum(v == INSUFFICIENT for v in primary.values()),
        UNRESOLVED: sum(v == UNRESOLVED for v in primary.values()),
    }
    assert census == {REJECT: 8, SUFFICIENT: 3, INSUFFICIENT: 8, UNRESOLVED: 0}, census

    seed = clone(candidates["W-01"])
    probes = [
        ("quorum_population_one", lambda x: x["witness_set"]["witnesses"].__delitem__(slice(1, 3)), INSUFFICIENT, True),
        ("independence_group_collapse", lambda x: [w.update({"independence_group": "group-1"}) for w in x["witness_set"]["witnesses"][1:]], INSUFFICIENT, True),
        ("stale_quorum", lambda x: [w.update({"witnessed_at": "2026-10-03T23:00:00Z"}) for w in x["witness_set"]["witnesses"][:2]], INSUFFICIENT, True),
        ("producer_quorum", lambda x: [w.update({"operating_party_id": "producer-a"}) for w in x["witness_set"]["witnesses"][:2]], INSUFFICIENT, True),
        ("fork_free_failure", lambda x: [w.update({"fork_free": False}) for w in x["witness_set"]["witnesses"][:2]], INSUFFICIENT, True),
        ("noncritical_extension", lambda x: x["witness_set"]["extensions"]["noncritical"].update({"trace2": "synthetic"}), SUFFICIENT, False),
        ("last_checkpoint_regression", lambda x: [w.update({"last_accepted_checkpoint_id": "checkpoint-genesis"}) for w in x["witness_set"]["witnesses"][:2]], INSUFFICIENT, True),
        ("same_operating_party_quorum", lambda x: [x["witness_set"]["witnesses"][1].update({"operating_party_id": "party-a"}), x["witness_set"]["witnesses"][2].update({"accepted_terminal_checkpoint_id": "checkpoint-other", "accepted_terminal_checkpoint_digest": "sha256:other"})], INSUFFICIENT, True),
        ("duplicate_verification_method", lambda x: x["witness_set"]["witnesses"][1].update({"verification_method_ref": "vm-key-1"}), REJECT, True),
        ("same_failure_domain_quorum", lambda x: [x["witness_set"]["witnesses"][1].update({"failure_domain": "failure-domain-a"}), x["witness_set"]["witnesses"][2].update({"accepted_terminal_checkpoint_id": "checkpoint-other", "accepted_terminal_checkpoint_digest": "sha256:other"})], INSUFFICIENT, True),
    ]
    for name, fn, want, do_reseal in probes:
        c = clone(seed)
        fn(c)
        if do_reseal:
            reseal(c)
        got = primary_qualify(c)
        assert got == want, (name, got, want)

    preserve = clone(seed)
    preserve["witness_set"]["witnesses"][2].update({
        "accepted_terminal_checkpoint_id": "checkpoint-other",
        "accepted_terminal_checkpoint_digest": "sha256:other",
    })
    reseal(preserve)
    assert primary_qualify(preserve) == SUFFICIENT

    breaks = clone(seed)
    for idx in (1, 2):
        breaks["witness_set"]["witnesses"][idx].update({
            "accepted_terminal_checkpoint_id": "checkpoint-other",
            "accepted_terminal_checkpoint_digest": "sha256:other",
        })
    reseal(breaks)
    assert primary_qualify(breaks) == INSUFFICIENT

    same = clone(seed)
    duplicate = clone(same["witness_set"]["witnesses"][0])
    duplicate["accepted_terminal_checkpoint_id"] = "checkpoint-other"
    duplicate["accepted_terminal_checkpoint_digest"] = "sha256:other"
    same["witness_set"]["witnesses"].append(duplicate)
    assert primary_qualify(same) == REJECT

    beyond = clone(seed)
    beyond["frontier"]["terminal_checkpoint_id"] = "checkpoint-future"
    reseal(beyond)
    assert primary_qualify(beyond) == INSUFFICIENT

    print("SYM-CIVIC-015 DERIVED=" + canon(census))
    print("SYM-CIVIC-015 METAMORPHIC=PASS")
    print("SYM-CIVIC-015 PASS")


if __name__ == "__main__":
    main()
