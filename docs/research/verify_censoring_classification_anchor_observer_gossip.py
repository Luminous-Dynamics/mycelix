#!/usr/bin/env python3
"""Research-only cross-observer gossip / split-view verifier.

The subject tree heads are authenticated by the earlier witness layer.
This layer authenticates the observer's report about what head it saw.
"""
from __future__ import annotations
import copy, json, sys
from pathlib import Path
from verify_censoring_classification_anchor_witness_crypto import canonical, digest, b64d, ed25519_verify

GOSSIP_REGISTRY_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-registry.v1"
GOSSIP_OBSERVATION_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-observation.v1"
GOSSIP_DOMAIN = "mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip.v1"
HEAD_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1"
WITNESS_REGISTRY_ID = "mycelix.research.anchor-witness-registry.v2"
WITNESS_ALGORITHM = "Ed25519"
VDS_ID = "mycelix.research.anchor-statement-sequence.v1"
GOSSIP_REGISTRY_ID = "mycelix.research.anchor-observer-gossip-registry.v1"
EXPECTED_GOSSIP_REGISTRY_SHA = "sha256:df41b280e6250f28425f2944f20e417bb69fa522eed9c52ca6b45d6b7931dd8e"

def key_for(reg, witness, key_id, version):
    wi = reg.get("witnesses", {}).get(witness)
    key = wi and wi.get("keys", {}).get(key_id)
    if not key:
        return None, "unknown-witness-or-key"
    if key.get("algorithm") != WITNESS_ALGORITHM:
        return None, "key-algorithm"
    if version < key.get("valid_from_version", 10**9):
        return None, "key-not-yet-valid"
    if key.get("valid_until_version") is not None and version > key["valid_until_version"]:
        return None, "key-expired-for-version"
    if key.get("revoked_at_version") is not None and version >= key["revoked_at_version"]:
        return None, "key-revoked"
    if key.get("status") == "revoked":
        return None, "key-revoked"
    try:
        return b64d(key["public_key"], 32), None
    except Exception:
        return None, "key-public-key"

def verify_subject_head(head, witness_reg):
    fields = {"schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"}
    if not isinstance(head, dict) or set(head) != fields:
        return False, "head-schema"
    if head["schema"] != HEAD_SCHEMA or head["domain"] != HEAD_SCHEMA:
        return False, "head-schema-or-domain"
    if head["algorithm"] != WITNESS_ALGORITHM:
        return False, "head-algorithm"
    if head["registry_id"] != WITNESS_REGISTRY_ID or head["registry_version"] != 2:
        return False, "head-registry-binding"
    if head["vds_id"] != VDS_ID:
        return False, "head-vds-binding"
    witness = head["observer_id"]
    wi = witness_reg.get("witnesses", {}).get(witness)
    if not wi:
        return False, "unknown-witness"
    pub, err = key_for(witness_reg, witness, head["key_id"], head["manifest_version"])
    if err:
        return False, err
    payload = {
        "schema": HEAD_SCHEMA,
        "domain": HEAD_SCHEMA,
        "algorithm": WITNESS_ALGORITHM,
        "observer_id": witness,
        "key_id": head["key_id"],
        "witness_identity_commitment": wi["identity_commitment"],
        "registry_id": head["registry_id"],
        "registry_version": head["registry_version"],
        "vds_id": head["vds_id"],
        "manifest_version": head["manifest_version"],
        "tree_size": head["tree_size"],
        "root_hash": head["root_hash"],
    }
    try:
        sig = b64d(head["signature"], 64)
    except Exception:
        return False, "head-signature-encoding"
    return (ed25519_verify(pub, sig, canonical(payload)), None) if ed25519_verify(pub, sig, canonical(payload)) else (False, "head-signature-invalid")

def verify_gossip_registry(reg):
    if reg.get("schema") != GOSSIP_REGISTRY_SCHEMA:
        return "registry-schema"
    if reg.get("status") != "research-fixture-only":
        return "registry-status"
    if reg.get("registry_id") != GOSSIP_REGISTRY_ID or reg.get("registry_version") != 1:
        return "registry-identity"
    if reg.get("algorithm") != WITNESS_ALGORITHM or reg.get("domain") != GOSSIP_DOMAIN:
        return "registry-profile"
    monitors = reg.get("monitors")
    if not isinstance(monitors, dict) or not monitors:
        return "registry-monitors"
    seen = set()
    for mid, m in monitors.items():
        if m.get("status") != "active" or m.get("key_id") in seen:
            return "monitor-schema"
        seen.add(m.get("key_id"))
        try:
            b64d(m.get("public_key", ""), 32)
        except Exception:
            return "monitor-public-key"
    return None

def verify_gossip(obs, registry):
    fields = {"schema","domain","algorithm","monitor_id","key_id","registry_id","registry_version","vds_id","subject_observer_id","subject_head_sha256","observed_tree_size","observed_root_hash","observation_sequence","signature"}
    if not isinstance(obs, dict) or set(obs) != fields:
        return None, "gossip-envelope"
    if obs["schema"] != GOSSIP_OBSERVATION_SCHEMA:
        return None, "gossip-schema"
    if obs["domain"] != GOSSIP_DOMAIN:
        return None, "gossip-domain"
    if obs["algorithm"] != WITNESS_ALGORITHM:
        return None, "gossip-algorithm"
    if obs["registry_id"] != GOSSIP_REGISTRY_ID or obs["registry_version"] != 1:
        return None, "gossip-registry-binding"
    if obs["vds_id"] != VDS_ID:
        return None, "gossip-vds-binding"
    mon = registry.get("monitors", {}).get(obs["monitor_id"])
    if not mon or mon.get("status") != "active" or mon.get("key_id") != obs["key_id"]:
        return None, "monitor-key"
    try:
        pub = b64d(mon["public_key"], 32)
        sig = b64d(obs["signature"], 64)
    except Exception:
        return None, "gossip-signature-encoding"
    payload = {k: obs[k] for k in fields if k != "signature"}
    if not ed25519_verify(pub, sig, canonical(payload)):
        return None, "gossip-signature-invalid"
    return obs, None

def consistency_4_to_7(first_hash: bytes, second_hash: bytes, path: bytes) -> bool:
    if not path:
        return False
    # RFC 9162 verification with first tree size 4 (an exact power of two).
    fr = sr = first_hash
    # After prepending first_hash, fn=3, sn=6; shift while LSB(fn) is set.
    fn, sn = 3, 6
    while fn & 1:
        fn >>= 1
        sn >>= 1
    # The remaining consistency path for this fixture has one node.
    if sn == 0:
        return False
    sr = sha256_node(sr, path)
    fn >>= 1
    sn >>= 1
    return sn == 0 and fr == first_hash and sr == second_hash

def sha256_node(a: bytes, b: bytes) -> bytes:
    import hashlib
    return hashlib.sha256(b"\x01" + a + b).digest()

def lookup_head(fixture, observation):
    for head in fixture.get("subject_heads", {}).values():
        if digest(head) == observation["subject_head_sha256"]:
            return head
    return None

def evaluate(case, fixture, gossip_reg, witness_reg):
    obs = copy.deepcopy(fixture["observations"])
    if case.get("mutate"):
        target = case.get("subject") or case.get("left")
        o = obs[target]
        m = case["mutate"]
        if m == "root":
            o["observed_root_hash"] = "sha256:" + "a" * 64
        elif m == "domain":
            o["domain"] = "mycelix.attacker.v1"
        elif m == "key_id":
            o["key_id"] = "m99-k1"
        elif m == "monitor_id":
            o["monitor_id"] = "m99"
        elif m == "subject_observer_id":
            o["subject_observer_id"] = "w02"
        elif m == "subject_head_sha256":
            o["subject_head_sha256"] = "sha256:" + "b" * 64
        elif m == "vds_id":
            o["vds_id"] = "mycelix.attacker.vds"
    if case.get("add_field"):
        obs[case["subject"]]["evil"] = True
    if case.get("same_monitor"):
        obs[case["right"]]["monitor_id"] = obs[case["left"]]["monitor_id"]
        obs[case["right"]]["key_id"] = obs[case["left"]]["key_id"]
    if case.get("same_observation"):
        obs[case["right"]] = copy.deepcopy(obs[case["left"]])

    def authenticated(o):
        _, err = verify_gossip(o, gossip_reg)
        if err:
            return None, err
        h = lookup_head(fixture, o)
        if not h:
            return None, "unknown-subject-head"
        if h.get("observer_id") != o["subject_observer_id"] or h.get("tree_size") != o["observed_tree_size"] or h.get("root_hash") != o["observed_root_hash"]:
            return None, "subject-claim-binding"
        ok, herr = verify_subject_head(h, witness_reg)
        if not ok:
            return None, herr
        return h, None

    if case["kind"] == "single":
        _, err = authenticated(obs[case["subject"]])
        return ("qualified", "authenticated-observation") if not err else ("unresolved", err)

    left, right = obs[case["left"]], obs[case["right"]]
    if left["monitor_id"] == right["monitor_id"]:
        return "unresolved", "monitor-independence"
    lh, le = authenticated(left)
    rh, re = authenticated(right)
    if le:
        return "unresolved", le
    if re:
        return "unresolved", re
    ls, rs = left["observed_tree_size"], right["observed_tree_size"]
    if ls == rs:
        if left["observed_root_hash"] == right["observed_root_hash"]:
            return "qualified", "same-head-cross-observer"
        return "unresolved", "split-view"
    if rs < ls:
        return "unresolved", "rollback"
    if case.get("drop_proof"):
        return "unresolved", "missing-consistency-proof"
    proof = b64d(fixture["consistency_proof_4_to_7"][0], 32)
    if case.get("mutate_proof") == "replace-first":
        import hashlib
        proof = hashlib.sha256(proof).digest()
    if ls != 4 or rs != 7:
        return "unresolved", "unsupported-consistency-pair"
    if not consistency_4_to_7(bytes.fromhex(left["observed_root_hash"][7:]), bytes.fromhex(right["observed_root_hash"][7:]), proof):
        return "unresolved", "consistency-proof-invalid"
    return "qualified", "cross-observer-consistency"

def main():
    if len(sys.argv) != 6:
        print("usage: verifier GOSSIP_REGISTRY FIXTURE CAMPAIGN WITNESS_REGISTRY REPORT", file=sys.stderr)
        return 2
    gp, fp, cp, wp, outp = sys.argv[1:]
    gossip_reg = json.loads(Path(gp).read_text())
    fixture = json.loads(Path(fp).read_text())
    campaign = json.loads(Path(cp).read_text())
    witness_reg = json.loads(Path(wp).read_text())
    err = verify_gossip_registry(gossip_reg)
    if err or digest(gossip_reg) != EXPECTED_GOSSIP_REGISTRY_SHA:
        return 1
    if fixture.get("schema") != "mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-fixture.v1":
        return 1
    if campaign.get("schema") != "mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-campaign.v1":
        return 1
    if campaign.get("case_count") != 16 or len(campaign.get("cases", [])) != 16:
        return 1
    if campaign.get("expected_gossip_registry_sha256") != EXPECTED_GOSSIP_REGISTRY_SHA:
        return 1
    if campaign.get("witness_registry_id") != WITNESS_REGISTRY_ID:
        return 1
    ids = [c.get("case_id") for c in campaign["cases"]]
    if len(ids) != len(set(ids)):
        return 1
    rows, failures = [], []
    for c in campaign["cases"]:
        verdict, reason = evaluate(c, fixture, gossip_reg, witness_reg)
        row = {"case_id": c["case_id"], "expected_verdict": c["expected_verdict"], "actual_verdict": verdict, "reason": reason}
        rows.append(row)
        if verdict != c["expected_verdict"]:
            failures.append([c["case_id"], c["expected_verdict"], verdict, reason])
    out = {"schema":"mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-report.v1","status":"research-evidence-only","case_count":len(rows),"cases":rows,"failures":failures}
    Path(outp).write_bytes(canonical(out) + b"\n")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0

if __name__ == "__main__":
    raise SystemExit(main())
