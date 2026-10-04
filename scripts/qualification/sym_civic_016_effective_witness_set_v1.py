#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/civic-resilience/sym_civic_016_effective_witness_set_v1.json"

PROGRAM = "SYM-CIVIC-016"
SCHEMA = "mycelix.sym-civic.effective-witness-set-preflight.v1"
REJECT = "REJECT_EFFECTIVE_WITNESS_PROVENANCE"
SUFFICIENT = "EFFECTIVE_WITNESS_SET_SUFFICIENT"
UNSAT = "EFFECTIVE_WITNESS_SET_UNSATISFIABLE"
UNRESOLVED = "EFFECTIVE_WITNESS_SET_UNRESOLVED"

PARENT_SUBJECT = "ed66bf43ac498316da53386752e9db7efb07a0ab"
CRITICAL = {"effective-witness-v1": {"mode": "INTERSECTION_MAX_QUORUM"}}

def canon(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)

def digest(value):
    return "sha256:" + hashlib.sha256(canon(value).encode("utf-8")).hexdigest()

def clone(value):
    return copy.deepcopy(value)

def normalize_origin(value):
    parsed = urlsplit(value)
    if parsed.scheme.lower() != "https" or not parsed.hostname or parsed.query or parsed.fragment:
        raise ValueError("invalid operating-party origin")
    host = parsed.hostname.lower()
    port = parsed.port
    if port in (None, 443):
        netloc = host
    else:
        netloc = f"{host}:{port}"
    return urlunsplit(("https", netloc, "", "", ""))

def identity_material(value):
    return {k: v for k, v in value.items() if k not in {"identity_material", "identity_digest"}}

def seal(value):
    value["identity_material"] = identity_material(value)
    value["identity_digest"] = digest(value["identity_material"])

def seal_extension(ext):
    material = {
        "schema_version": ext.get("schema_version"),
        "critical_namespaces": ext.get("critical_namespaces"),
    }
    ext["identity_material"] = material
    ext["identity_digest"] = digest(material)

def base_witness(member, vm, party, key):
    w = {
        "audience_member_id": member,
        "verification_method_ref": vm,
        "operating_party_id": party,
        "public_key_digest": key,
    }
    seal(w)
    return w

def base_candidate():
    a = [
        base_witness("member-alpha", "vm-key-1", "https://alpha.example", "sha256:key-1"),
        base_witness("member-beta", "vm-key-2", "https://beta.example", "sha256:key-2"),
        base_witness("member-gamma", "vm-key-3", "https://gamma.example", "sha256:key-3"),
    ]
    b = [
        base_witness("member-alpha-b", "vm-key-1", "https://alpha.example", "sha256:key-1"),
        base_witness("member-beta-b", "vm-key-2", "https://beta.example", "sha256:key-2"),
        base_witness("member-gamma-b", "vm-key-3", "https://gamma.example", "sha256:key-3"),
    ]
    agreements = [
        {
            "agreement_hash": "sha256:a-agreement",
            "quorum": 2,
            "witnesses": a,
            "extensions": {
                "schema_version": "effective-witness-1",
                "critical_namespaces": clone(CRITICAL),
                "noncritical": {"trace": "synthetic"},
            },
        },
        {
            "agreement_hash": "sha256:b-agreement",
            "quorum": 2,
            "witnesses": b,
            "extensions": {
                "schema_version": "effective-witness-1",
                "critical_namespaces": clone(CRITICAL),
                "noncritical": {"trace": "synthetic"},
            },
        },
    ]
    for agreement in agreements:
        seal_extension(agreement["extensions"])
        seal(agreement)
    return {"agreements": agreements, "agreement_order": [0, 1]}

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
    for agreement in candidate["agreements"]:
        for witness in agreement["witnesses"]:
            seal(witness)
        seal_extension(agreement["extensions"])
        seal(agreement)

def ordered_agreements(candidate):
    return [candidate["agreements"][i] for i in candidate["agreement_order"]]

def ordered_witnesses(agreement):
    order = agreement.get("witness_order")
    if order is None:
        return list(agreement["witnesses"])
    return [agreement["witnesses"][i] for i in order]

def source_provenance(agreement):
    witnesses = agreement.get("witnesses", [])
    quorum = agreement.get("quorum")
    if not isinstance(quorum, int) or quorum < 0 or quorum > len(witnesses):
        return False
    extensions = agreement.get("extensions", {})
    if extensions.get("schema_version") != "effective-witness-1":
        return False
    if extensions.get("critical_namespaces") != CRITICAL:
        return False
    if extensions.get("identity_digest") != digest({
        "schema_version": extensions.get("schema_version"),
        "critical_namespaces": extensions.get("critical_namespaces"),
    }):
        return False
    audience_ids = []
    methods = []
    keys = []
    parties = set()
    for witness in witnesses:
        if any(not witness.get(k) for k in (
            "audience_member_id",
            "verification_method_ref",
            "operating_party_id",
            "public_key_digest",
        )):
            return False
        try:
            party = normalize_origin(witness["operating_party_id"])
        except Exception:
            return False
        if witness.get("identity_digest") != digest(identity_material(witness)):
            return False
        audience_ids.append(witness["audience_member_id"])
        methods.append(witness["verification_method_ref"])
        keys.append(witness["public_key_digest"])
        parties.add(party)
    if len(audience_ids) != len(set(audience_ids)):
        return False
    if len(methods) != len(set(methods)):
        return False
    if len(keys) != len(set(keys)):
        return False
    if len(parties) < quorum:
        return False
    return True

def effective(candidate):
    agreements = ordered_agreements(candidate)
    if not agreements or any(not source_provenance(a) for a in agreements):
        return REJECT, None
    maps = []
    for agreement in agreements:
        mapping = {}
        for witness in ordered_witnesses(agreement):
            key = (
                normalize_origin(witness["operating_party_id"]),
                witness["verification_method_ref"],
            )
            mapping[key] = witness
        maps.append(mapping)
    common = set(maps[0])
    for mapping in maps[1:]:
        common &= set(mapping)
    selected = []
    for key in sorted(common):
        chosen = min(
            (
                (agreement["agreement_hash"], mapping[key])
                for agreement, mapping in zip(agreements, maps)
            ),
            key=lambda pair: pair[0],
        )[1]
        selected.append(chosen)
    selected.sort(
        key=lambda witness: (
            normalize_origin(witness["operating_party_id"]),
            witness["verification_method_ref"],
        )
    )
    quorum = max(agreement["quorum"] for agreement in agreements)
    if (
        len(selected) < quorum
        or len({normalize_origin(w["operating_party_id"]) for w in selected}) < quorum
        or len({w["verification_method_ref"] for w in selected}) < quorum
    ):
        return UNSAT, {"quorum": quorum, "witnesses": selected}
    return SUFFICIENT, {"quorum": quorum, "witnesses": selected}

def reference_effective(candidate):
    agreements = [candidate["agreements"][i] for i in candidate["agreement_order"]]
    if not agreements or any(not source_provenance(a) for a in agreements):
        return REJECT, None
    key_maps = []
    for agreement in agreements:
        current = {}
        for witness in ordered_witnesses(agreement):
            current[(
                normalize_origin(witness["operating_party_id"]),
                witness["verification_method_ref"],
            )] = witness
        key_maps.append(current)
    common = set(key_maps[0].keys())
    for current in key_maps[1:]:
        common = common.intersection(current.keys())
    result = []
    for key in sorted(common):
        candidates = []
        for agreement, current in zip(agreements, key_maps):
            if key in current:
                candidates.append((agreement["agreement_hash"], current[key]))
        result.append(min(candidates, key=lambda pair: pair[0])[1])
    quorum = max(agreement["quorum"] for agreement in agreements)
    if (
        len(result) < quorum
        or len({normalize_origin(w["operating_party_id"]) for w in result}) < quorum
        or len({w["verification_method_ref"] for w in result}) < quorum
    ):
        return UNSAT, {"quorum": quorum, "witnesses": result}
    return SUFFICIENT, {"quorum": quorum, "witnesses": result}

def candidate_for(spec):
    candidate = base_candidate()
    for mutation in spec.get("mutations", []):
        apply_mutation(candidate, mutation)
    if any(mutation.get("reseal") for mutation in spec.get("mutations", [])):
        reseal(candidate)
    return candidate

def main():
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert document["schema"] == SCHEMA
    assert document["program"] == PROGRAM
    assert document["analysis_role"] == "research_only"
    assert document["parent_subject"] == PARENT_SUBJECT
    cases = document["cases"]
    assert [c["id"] for c in cases] == [f"E-{i:02d}" for i in range(1, 19)]
    for case in cases:
        assert set(case) == {"id", "family", "mutations"}
        lowered = canon(case).lower()
        assert not any(token in lowered for token in (
            "expected_verdict", "expected_disposition", "oracle_verdict", "candidate_verdict"
        ))
    candidates = {case["id"]: candidate_for(case) for case in cases}
    primary = {case_id: effective(candidate) for case_id, candidate in candidates.items()}
    reference = {case_id: reference_effective(candidate) for case_id, candidate in candidates.items()}
    assert primary == reference, "reference disagreement"
    census = {
        REJECT: sum(value[0] == REJECT for value in primary.values()),
        SUFFICIENT: sum(value[0] == SUFFICIENT for value in primary.values()),
        UNSAT: sum(value[0] == UNSAT for value in primary.values()),
        UNRESOLVED: sum(value[0] == UNRESOLVED for value in primary.values()),
    }
    assert census == {
        REJECT: 5,
        SUFFICIENT: 11,
        UNSAT: 2,
        UNRESOLVED: 0,
    }, census

    seed = clone(candidates["E-01"])
    baseline = effective(seed)
    assert baseline[0] == SUFFICIENT

    reordered = clone(seed)
    reordered["agreement_order"] = [1, 0]
    assert effective(reordered) == baseline

    witness_reordered = clone(seed)
    witness_reordered["agreements"][0]["witness_order"] = [2, 0, 1]
    witness_reordered["agreements"][1]["witness_order"] = [1, 2, 0]
    assert effective(witness_reordered) == baseline

    selected_source_changes = clone(seed)
    selected_source_changes["agreements"][0]["agreement_hash"] = "sha256:z-agreement"
    selected_source_changes["agreements"][1]["agreement_hash"] = "sha256:a-agreement"
    reseal(selected_source_changes)
    changed = effective(selected_source_changes)
    assert changed[0] == SUFFICIENT
    assert changed[1]["witnesses"][0]["audience_member_id"] == "member-alpha-b"

    print("SYM-CIVIC-016 DERIVED=" + canon(census))
    print("SYM-CIVIC-016 METAMORPHIC=PASS")
    print("SYM-CIVIC-016 PASS")

if __name__ == "__main__":
    main()
