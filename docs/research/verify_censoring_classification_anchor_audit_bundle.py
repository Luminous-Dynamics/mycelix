#!/usr/bin/env python3
"""Research-only replayable evidence-binding verifier.

This verifies that all upstream research artifacts are exactly the ones named
by the audit bundle and that their declared cross-links remain consistent.
It does not turn the bundle into a hosted PASS.
"""
from __future__ import annotations
import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path

BUNDLE_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle.v1"
CAMPAIGN_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-campaign.v1"
WREG_ID = "mycelix.research.anchor-witness-registry.v2"
VDS_ID = "mycelix.research.anchor-statement-sequence.v1"
EXPECTED_STACK_HEAD = "deb940fea8bb83ce2882a72459b2d9e3af7a9d52"
EXPECTED_BUNDLE_ID = f"mycelix.audit-bundle.v1@{EXPECTED_STACK_HEAD}"

def canonical(v):
    return json.dumps(v, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")

def digest(v):
    return "sha256:" + hashlib.sha256(canonical(v)).hexdigest()

def git_blob_sha(path: Path) -> str:
    return subprocess.check_output(["git", "hash-object", str(path)], text=True).strip()

def contains_secret(value):
    secret_terms = {"private_key", "private_keys", "secret", "seed", "secret_key"}
    if isinstance(value, dict):
        return any(k.lower() in secret_terms or contains_secret(v) for k, v in value.items())
    if isinstance(value, list):
        return any(contains_secret(v) for v in value)
    return False

def validate_bundle(bundle, repo_root: Path):
    if bundle.get("schema") != BUNDLE_SCHEMA:
        return "bundle-schema"
    if bundle.get("status") != "research-evidence-only" or bundle.get("evidence_ceiling") != "replayable-binding-only":
        return "bundle-status"
    if bundle.get("bundle_id") != EXPECTED_BUNDLE_ID:
        return "bundle-id"
    if bundle.get("hosted_status") != "unclaimed":
        return "hosted-claim-injection"
    topo = {(x.get("pr"), x.get("head")) for x in bundle.get("topology", [])}
    expected_topo = {
        (4848, "5af2d8a2a4e42ccdcabc70160ad639d810642c86"),
        (4870, "2af796188bd7981fac111178141ae82165c2fc4e"),
        (4873, "e3a175b905a041a17630b92356e123ccba0d9175"),
        (4874, "b55af8c135a858abfb0663a762e7fd510e9ea6d2"),
        (4875, "4c34a77adacd533cbbcafc076846ccf09c9b0749"),
        (4876, "267e43350f9311a5515762311ec37677b1f4b8da"),
        (4877, EXPECTED_STACK_HEAD),
    }
    if topo != expected_topo:
        return "topology"
    req = bundle.get("decision_requires")
    required = {"witness_crypto_verifier","vds_verifier","tree_head_verifier","receipt_verifier","observer_gossip_verifier"}
    if not req or req.get("hosted_pass") is not False or any(req.get(k) is not True for k in required):
        return "decision-requirement"
    arts = bundle.get("artifacts", {})
    for name, pair in arts.items():
        if not isinstance(pair, list) or len(pair) != 2:
            return f"artifact-record:{name}"
        path, expected_sha = pair
        p = repo_root / path
        if not p.is_file():
            return f"artifact-missing:{name}"
        if git_blob_sha(p) != expected_sha:
            return f"artifact-sha:{name}"
        try:
            obj = json.loads(p.read_text())
        except Exception:
            return f"artifact-json:{name}"
        if contains_secret(obj):
            return f"secret-material:{name}"

    reg = json.loads((repo_root / arts["witness_crypto_registry"][0]).read_text())
    root = json.loads((repo_root / arts["witness_crypto_root"][0]).read_text())
    baseline = json.loads((repo_root / arts["witness_crypto_baseline"][0]).read_text())
    forward = json.loads((repo_root / arts["witness_crypto_forward"][0]).read_text())
    vds = json.loads((repo_root / arts["vds_fixture"][0]).read_text())
    treeheads = json.loads((repo_root / arts["vds_tree_heads"][0]).read_text())
    receipt_ts = json.loads((repo_root / arts["receipt_ts_registry"][0]).read_text())
    receipt = json.loads((repo_root / arts["receipt_fixture"][0]).read_text())
    gossip = json.loads((repo_root / arts["gossip_fixture"][0]).read_text())
    gossip_reg = json.loads((repo_root / arts["gossip_registry"][0]).read_text())

    if reg.get("registry_id") != WREG_ID or reg.get("registry_version") != 2:
        return "witness-registry-binding"
    if root.get("registry_id") != WREG_ID or root.get("registry_version") != reg.get("registry_version"):
        return "witness-root-binding"
    if root.get("registry_sha256") != digest(reg):
        return "witness-root-digest"
    if baseline.get("registry_id") != WREG_ID or forward.get("registry_id") != WREG_ID:
        return "witness-checkpoint-binding"
    if vds.get("vds_id") != VDS_ID:
        return "vds-binding"
    if treeheads.get("vds_id") != VDS_ID:
        return "tree-head-binding"
    if receipt_ts.get("ts_id") != receipt.get("ts_registry_sha256") and receipt_ts.get("registry_id") != receipt.get("ts_registry_sha256"):
        # Receipt fixture stores the registry digest, not the registry identifier.
        if receipt.get("ts_registry_sha256") != digest(receipt_ts):
            return "receipt-ts-registry-binding"
    if receipt.get("vds_id") != VDS_ID:
        return "receipt-vds-binding"
    if gossip.get("vds_id") != VDS_ID or gossip_reg.get("registry_id") != bundle["bindings"]["gossip_registry_id"]:
        return "gossip-binding"
    if bundle["bindings"]["witness_registry_id"] != WREG_ID or bundle["bindings"]["vds_id"] != VDS_ID:
        return "bundle-binding"

    return None

def apply_mutation(bundle, mutation):
    out = copy.deepcopy(bundle)
    m = mutation
    if m == "witness_crypto_registry_sha":
        out["artifacts"]["witness_crypto_registry"][1] = "0" * 40
    elif m == "witness_root_sha":
        out["artifacts"]["witness_crypto_root"][1] = "0" * 40
    elif m == "vds_fixture_sha":
        out["artifacts"]["vds_fixture"][1] = "0" * 40
    elif m == "tree_head_sha":
        out["artifacts"]["vds_tree_heads"][1] = "0" * 40
    elif m == "receipt_fixture_sha":
        out["artifacts"]["receipt_fixture"][1] = "0" * 40
    elif m == "gossip_fixture_sha":
        out["artifacts"]["gossip_fixture"][1] = "0" * 40
    elif m == "witness_registry_id":
        out["bindings"]["witness_registry_id"] = "attacker.registry"
    elif m == "vds_id":
        out["bindings"]["vds_id"] = "attacker.vds"
    elif m == "hosted_status":
        out["hosted_status"] = "success"
    elif m == "topology_head":
        out["topology"][-1]["head"] = "0" * 40
    elif m == "decision_requirement":
        out["decision_requires"]["hosted_pass"] = True
    elif m == "witness_crypto_verifier":
        out["decision_requires"]["witness_crypto_verifier"] = False
    elif m == "vds_verifier":
        out["decision_requires"]["vds_verifier"] = False
    elif m == "tree_head_verifier":
        out["decision_requires"]["tree_head_verifier"] = False
    elif m == "receipt_verifier":
        out["decision_requires"]["receipt_verifier"] = False
    elif m == "gossip_verifier":
        out["decision_requires"]["observer_gossip_verifier"] = False
    elif m == "bundle_id":
        out["bundle_id"] = "mycelix.audit-bundle.v1@attacker"
    return out

def main():
    if len(sys.argv) != 5:
        print("usage: verifier REPO_ROOT BUNDLE CAMPAIGN REPORT", file=sys.stderr)
        return 2
    root, bundle_path, campaign_path, report_path = map(Path, sys.argv[1:])
    bundle = json.loads(bundle_path.read_text())
    campaign = json.loads(campaign_path.read_text())
    if campaign.get("schema") != CAMPAIGN_SCHEMA or campaign.get("case_count") != 18 or len(campaign.get("cases", [])) != 18:
        return 1
    rows, failures = [], []
    for case in campaign["cases"]:
        candidate = apply_mutation(bundle, case.get("mutate"))
        err = validate_bundle(candidate, root)
        verdict = "evidence-ready" if err is None else "unresolved"
        row = {"case_id": case["case_id"], "expected_verdict": case["expected_verdict"], "actual_verdict": verdict, "reason": err or "bindings-and-git-object-identities-match"}
        rows.append(row)
        if verdict != case["expected_verdict"]:
            failures.append([case["case_id"], case["expected_verdict"], verdict, err])
    report = {"schema":"mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-report.v1","status":"research-evidence-only","case_count":len(rows),"cases":rows,"failures":failures}
    Path(report_path).write_bytes(canonical(report) + b"\n")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0

if __name__ == "__main__":
    raise SystemExit(main())
