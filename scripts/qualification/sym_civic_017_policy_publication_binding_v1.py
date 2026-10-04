#!/usr/bin/env python3
from __future__ import annotations
import copy
import hashlib
import json
from datetime import datetime
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

MANIFEST = Path(__file__).resolve().parents[2] / "mycelix-workspace/docs/civic-resilience/sym_civic_017_policy_publication_binding_v1.json"
SCHEMA = "mycelix.sym-civic.policy-publication-binding-preflight.v1"
PROGRAM = "SYM-CIVIC-017"
PARENT_SUBJECT = "0a0b5bbddbac1e611ee9664896c18387a14373f5"
REJECT = "REJECT_POLICY_PUBLICATION_PROVENANCE"
SUFFICIENT = "POLICY_PUBLICATION_SUFFICIENT"
STALE = "POLICY_PUBLICATION_STALE"
UNRESOLVED = "POLICY_PUBLICATION_UNRESOLVED"
CRITICAL = {"policy-publication-v1": {"mode": "EFFECTIVE_WITNESS_BINDING"}}

def canon(v):
    return json.dumps(v, sort_keys=True, separators=(",", ":"), ensure_ascii=True)

def digest(v):
    return "sha256:" + hashlib.sha256(canon(v).encode("utf-8")).hexdigest()

def clone(v):
    return copy.deepcopy(v)

def origin(v):
    parsed = urlsplit(v)
    if (
        parsed.scheme.lower() != "https"
        or not parsed.hostname
        or parsed.username is not None
        or parsed.password is not None
        or parsed.path not in ("", "/")
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError("invalid operating-party origin")
    host = parsed.hostname.lower()
    if ":" in host and not host.startswith("["):
        host = "[" + host + "]"
    port = parsed.port
    netloc = host if port in (None, 443) else f"{host}:{port}"
    return urlunsplit(("https", netloc, "", "", ""))

def witness_material(w):
    return {
        "audience_member_id": w["audience_member_id"],
        "verification_method_ref": w["verification_method_ref"],
        "operating_party_id": origin(w["operating_party_id"]),
        "public_key_digest": w["public_key_digest"],
    }

def seal_witness(w):
    w["identity_material"] = witness_material(w)
    w["identity_digest"] = digest(w["identity_material"])

def make_witness(member, vm, party, key):
    w = {
        "audience_member_id": member,
        "verification_method_ref": vm,
        "operating_party_id": party,
        "public_key_digest": key,
    }
    seal_witness(w)
    return w

def agreement_material(a):
    return {
        "agreement_hash": a["agreement_hash"],
        "quorum": a["quorum"],
        "witnesses": sorted(
            (witness_material(w) for w in a["witnesses"]),
            key=canon,
        ),
        "extensions": {
            "schema_version": a["extensions"]["schema_version"],
            "critical_namespaces": a["extensions"]["critical_namespaces"],
        },
    }

def seal_extension(e):
    e["identity_digest"] = digest({
        "schema_version": e["schema_version"],
        "critical_namespaces": e["critical_namespaces"],
    })

def seal_agreement(a):
    for w in a["witnesses"]:
        seal_witness(w)
    seal_extension(a["extensions"])
    a["content_digest"] = digest(agreement_material(a))

def source_ok(a):
    witnesses = a.get("witnesses", [])
    q = a.get("quorum")
    if a.get("agreement_hash") in (None, ""):
        return False

    if not isinstance(q, int) or q < 0 or q > len(witnesses):
        return False
    ext = a.get("extensions", {})
    if ext.get("schema_version") != "policy-publication-1":
        return False
    if ext.get("critical_namespaces") != CRITICAL:
        return False
    if ext.get("identity_digest") != digest({
        "schema_version": ext.get("schema_version"),
        "critical_namespaces": ext.get("critical_namespaces"),
    }):
        return False
    if a.get("content_digest") != digest(agreement_material(a)):
        return False
    ids, vms, keys, parties = [], [], [], []
    for w in witnesses:
        if any(
            not w.get(k)
            for k in (
                "audience_member_id",
                "verification_method_ref",
                "operating_party_id",
                "public_key_digest",
            )
        ):
            return False
        try:
            party = origin(w["operating_party_id"])
        except Exception:
            return False
        if w.get("identity_digest") != digest(witness_material(w)):
            return False
        ids.append(w["audience_member_id"])
        vms.append(w["verification_method_ref"])
        keys.append(w["public_key_digest"])
        parties.append(party)
    return (
        len(ids) == len(set(ids))
        and len(vms) == len(set(vms))
        and len(keys) == len(set(keys))
        and len(set(parties)) >= q
    )

def effective(agreements, order):
    ordered = [agreements[i] for i in order]
    if not ordered or any(not source_ok(a) for a in ordered):
        return REJECT, None
    agreement_hashes = [a["agreement_hash"] for a in ordered]
    if len(agreement_hashes) != len(set(agreement_hashes)):
        return REJECT, None
    maps = []
    for a in ordered:
        current = {}
        for w in a["witnesses"]:
            current[(origin(w["operating_party_id"]), w["verification_method_ref"])] = w
        maps.append(current)
    common = set(maps[0])
    for current in maps[1:]:
        common &= set(current)
    selected = []
    for key in sorted(common):
        selected.append(
            min(
                ((a["agreement_hash"], m[key]) for a, m in zip(ordered, maps)),
                key=lambda pair: pair[0],
            )[1]
        )
    selected.sort(key=lambda w: (origin(w["operating_party_id"]), w["verification_method_ref"]))
    quorum = max(a["quorum"] for a in ordered)
    if (
        len(selected) < quorum
        or len({origin(w["operating_party_id"]) for w in selected}) < quorum
        or len({w["verification_method_ref"] for w in selected}) < quorum
    ):
        return STALE, {"quorum": quorum, "witnesses": selected}
    return SUFFICIENT, {"quorum": quorum, "witnesses": selected}

def reference_effective(agreements, order):
    ordered = [agreements[i] for i in order]
    if not ordered or any(not source_ok(a) for a in ordered):
        return REJECT, None
    agreement_hashes = [a["agreement_hash"] for a in ordered]
    if len(agreement_hashes) != len(set(agreement_hashes)):
        return REJECT, None
    relations = []
    for a in ordered:
        relation = {
            (origin(w["operating_party_id"]), w["verification_method_ref"]): w
            for w in a["witnesses"]
        }
        relations.append(relation)
    common = set(relations[0].keys())
    for relation in relations[1:]:
        common = common.intersection(relation.keys())
    selected = []
    for key in sorted(common):
        candidates = []
        for a, relation in zip(ordered, relations):
            candidates.append((a["agreement_hash"], relation[key]))
        selected.append(min(candidates, key=lambda pair: pair[0])[1])
    quorum = max(a["quorum"] for a in ordered)
    selected.sort(key=lambda w: (origin(w["operating_party_id"]), w["verification_method_ref"]))
    if (
        len(selected) < quorum
        or len({origin(w["operating_party_id"]) for w in selected}) < quorum
        or len({w["verification_method_ref"] for w in selected}) < quorum
    ):
        return STALE, {"quorum": quorum, "witnesses": selected}
    return SUFFICIENT, {"quorum": quorum, "witnesses": selected}

def policy_wire(inputs, result, ts):
    witnesses = [
        [
            w["audience_member_id"],
            w["verification_method_ref"],
            origin(w["operating_party_id"]),
        ]
        for w in result["witnesses"]
    ]
    witnesses.sort(key=canon)
    return [
        ts,
        sorted(inputs["permitted_signature_algorithms"]),
        sorted(inputs["per_predicate"], key=canon),
        sorted(inputs["transitions"], key=canon),
        [witnesses, result["quorum"]],
        [
            inputs["response_freshness_tolerance_seconds"],
            inputs["ledger_head_notarisation_interval_seconds"],
        ],
    ]

def dependency_binding_material(agreements):
    ordered = sorted(agreements, key=lambda a: a["agreement_hash"])
    return {
        "agreement_hashes": [a["agreement_hash"] for a in ordered],
        "agreement_content_digests": [a["content_digest"] for a in ordered],
    }

def dependency_binding_digest(agreements):
    return digest(dependency_binding_material(agreements))

def signature_material(signature):
    return {
        "algorithm": signature["algorithm"],
        "key_ref": signature["key_ref"],
        "signed_payload_digest": signature["signed_payload_digest"],
        "dependency_binding_digest": signature["dependency_binding_digest"],
    }

def make_publication(agreements, inputs, ts="2026-10-01T12:00:00Z",
                     prev="2026-09-30T12:00:00Z", key="policy-seal-1"):
    status, result = effective(agreements, [0, 1])
    assert status == SUFFICIENT
    payload = policy_wire(inputs, result, ts)
    publication = {
        "schema_version": "policy-publication-1",
        "publication_timestamp": ts,
        "previous_publication_timestamp": prev,
        "sealing_key_ref": key,
        "wire_payload": payload,
        "source_agreement_hashes": sorted(a["agreement_hash"] for a in agreements),
        "source_agreement_content_digests": [
            a["content_digest"] for a in sorted(agreements, key=lambda a: a["agreement_hash"])
        ],
        "dependency_binding_digest": dependency_binding_digest(agreements),
        "payload_digest": digest(payload),
        "extensions": {
            "critical_namespaces": clone(CRITICAL),
            "noncritical": {"trace": "synthetic"},
        },
    }
    publication["extensions"]["identity_digest"] = digest(CRITICAL)
    publication["signature"] = {
        "algorithm": "synthetic-binding-v2",
        "key_ref": key,
        "signed_payload_digest": publication["payload_digest"],
        "dependency_binding_digest": publication["dependency_binding_digest"],
    }
    publication["signature"]["signature_digest"] = digest(
        signature_material(publication["signature"])
    )
    return publication

def mutate(candidate, mutation):
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

def candidate(spec):
    candidate = base()
    for mutation in spec.get("mutations", []):
        mutate(candidate, mutation)
    if any(m.get("reseal") for m in spec.get("mutations", [])):
        for agreement in candidate["agreements"]:
            seal_agreement(agreement)
    if spec["id"] == "P-14":
        candidate["publication"]["payload_digest"] = digest(candidate["publication"]["wire_payload"])
        candidate["publication"]["signature"]["signed_payload_digest"] = candidate["publication"]["payload_digest"]
        candidate["publication"]["signature"]["signature_digest"] = digest(
            signature_material(candidate["publication"]["signature"])
        )
    return candidate

def base():
    def trio(suffix):
        return [
            make_witness(f"member-{name}{suffix}", f"vm-key-{i}",
                         f"https://{name}.example", f"sha256:key-{i}")
            for i, name in enumerate(("alpha", "beta", "gamma"), 1)
        ]
    agreements = [
        {
            "agreement_hash": "sha256:a-agreement",
            "quorum": 2,
            "witnesses": trio(""),
            "extensions": {
                "schema_version": "policy-publication-1",
                "critical_namespaces": clone(CRITICAL),
                "noncritical": {"trace": "synthetic"},
            },
        },
        {
            "agreement_hash": "sha256:b-agreement",
            "quorum": 2,
            "witnesses": trio("-b"),
            "extensions": {
                "schema_version": "policy-publication-1",
                "critical_namespaces": clone(CRITICAL),
                "noncritical": {"trace": "synthetic"},
            },
        },
    ]
    for agreement in agreements:
        seal_agreement(agreement)
    inputs = {
        "permitted_signature_algorithms": ["ES256", "EdDSA"],
        "per_predicate": [
            ["civic-empty-result", ["regime-a", "regime-b"], ["MAJORITY", {"threshold": 2}], [0, 3600]]
        ],
        "transitions": [
            ["pattern-library-v1", "2026-09-20T00:00:00Z"],
            ["policy-version-v1", "2026-09-25T00:00:00Z"],
        ],
        "response_freshness_tolerance_seconds": 300,
        "ledger_head_notarisation_interval_seconds": 60,
    }
    return {
        "agreements": agreements,
        "policy_inputs": inputs,
        "publication": make_publication(agreements, inputs),
        "agreement_order": [0, 1],
    }

def validate(candidate):
    publication = candidate["publication"]
    agreements = candidate["agreements"]
    inputs = candidate["policy_inputs"]
    primary = effective(agreements, candidate["agreement_order"])
    reference = reference_effective(agreements, candidate["agreement_order"])
    if primary != reference:
        return UNRESOLVED, None
    if primary[0] != SUFFICIENT:
        return REJECT, None
    try:
        if datetime.fromisoformat(publication["publication_timestamp"].replace("Z", "+00:00")) < datetime.fromisoformat(
            publication["previous_publication_timestamp"].replace("Z", "+00:00")
        ):
            return STALE, None
    except Exception:
        return REJECT, None

    if publication.get("schema_version") != "policy-publication-1":
        return REJECT, None
    ext = publication.get("extensions", {})
    if ext.get("critical_namespaces") != CRITICAL or ext.get("identity_digest") != digest(CRITICAL):
        return REJECT, None

    expected_hashes = sorted(a["agreement_hash"] for a in agreements)
    expected_digests = [
        a["content_digest"] for a in sorted(agreements, key=lambda a: a["agreement_hash"])
    ]
    expected_dependency_binding = dependency_binding_digest(agreements)
    if publication.get("source_agreement_hashes") != expected_hashes:
        return REJECT, None
    if publication.get("source_agreement_content_digests") != expected_digests:
        return STALE, None
    if publication.get("dependency_binding_digest") != expected_dependency_binding:
        return REJECT, None

    expected_payload = policy_wire(inputs, primary[1], publication["publication_timestamp"])
    if publication.get("wire_payload") != expected_payload:
        return REJECT, None
    if publication.get("payload_digest") != digest(expected_payload):
        return REJECT, None

    signature = publication.get("signature", {})
    if signature.get("key_ref") != publication.get("sealing_key_ref"):
        return REJECT, None
    if signature.get("signed_payload_digest") != publication.get("payload_digest"):
        return REJECT, None
    if signature.get("signature_digest") != digest(signature_material(signature)):
        return REJECT, None

    return SUFFICIENT, {
        "payload_digest": publication["payload_digest"],
        "source_agreement_hashes": expected_hashes,
        "effective_quorum": primary[1]["quorum"],
        "effective_witness_count": len(primary[1]["witnesses"]),
    }

def main():
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert document["schema"] == SCHEMA
    assert document["program"] == PROGRAM
    assert document["analysis_role"] == "research_only"
    assert document["parent_subject"] == PARENT_SUBJECT
    cases = document["cases"]
    assert [c["id"] for c in cases] == [f"P-{i:02d}" for i in range(1, 21)]
    for case in cases:
        assert set(case) == {"id", "family", "mutations"}
        lowered = canon(case).lower()
        assert not any(
            token in lowered
            for token in (
                "expected_verdict",
                "expected_disposition",
                "oracle_verdict",
                "candidate_verdict",
            )
        )
    results = {case["id"]: validate(candidate(case)) for case in cases}
    census = {
        REJECT: sum(v[0] == REJECT for v in results.values()),
        SUFFICIENT: sum(v[0] == SUFFICIENT for v in results.values()),
        STALE: sum(v[0] == STALE for v in results.values()),
        UNRESOLVED: sum(v[0] == UNRESOLVED for v in results.values()),
    }
    assert census == {
        REJECT: 16,
        SUFFICIENT: 3,
        STALE: 1,
        UNRESOLVED: 0,
    }, census

    seed = candidate(cases[0])
    baseline = validate(seed)
    assert baseline[0] == SUFFICIENT

    reordered = clone(seed)
    reordered["agreement_order"] = [1, 0]
    assert validate(reordered) == baseline

    witness_reordered = clone(seed)
    for agreement in witness_reordered["agreements"]:
        agreement["witnesses"].reverse()
        seal_agreement(agreement)
    witness_reordered["publication"] = make_publication(
        witness_reordered["agreements"], witness_reordered["policy_inputs"]
    )
    assert validate(witness_reordered)[0] == SUFFICIENT

    changed = clone(seed)
    changed["agreements"][0]["witnesses"][0]["audience_member_id"] = "member-alpha-new"
    assert validate(changed)[0] == REJECT

    dependency_tampered = clone(seed)
    dependency_tampered["publication"]["dependency_binding_digest"] = "sha256:tampered"
    assert validate(dependency_tampered)[0] == REJECT

    print("SYM-CIVIC-017 DERIVED=" + canon(census))
    print("SYM-CIVIC-017 METAMORPHIC=PASS")
    print("SYM-CIVIC-017 PASS")

if __name__ == "__main__":
    main()
