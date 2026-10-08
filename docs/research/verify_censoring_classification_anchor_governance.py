#!/usr/bin/env python3
"""Research-only verifier for independently pinned anchor-manifest governance."""
from __future__ import annotations
import copy, hashlib, json, sys
from pathlib import Path

ROOT_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-trust-root.v1"
MANIFEST_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-manifest.v1"
CAMPAIGN_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-governance-campaign.v1"
OBSERVED_STATE_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-observed-state.v1"
AUTHORITY_ID = "mycelix.research.anchor-authority.v1"
ROOT_ID = "mycelix.research.anchor-root.v1"
ROOT_EPOCH = "t1"
EPOCHS = ("t0", "t1", "t2")
IDENTITY_FIELDS = ("attempt_id","censoring_reason","classification_epoch","frozen_epoch","policy_blob_sha","basis_id","revision","claim_scope_anchor")

def canonical(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical(value)).hexdigest()
def git_blob_sha_bytes(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()
def git_blob_sha(path: Path) -> str:
    return git_blob_sha_bytes(path.read_bytes())

def node_index(graph: dict) -> dict[str, dict] | None:
    if not isinstance(graph.get("nodes"), list): return None
    out: dict[str, dict] = {}
    for n in graph["nodes"]:
        if not isinstance(n, dict) or not isinstance(n.get("id"), str) or n["id"] in out: return None
        out[n["id"]] = n
    return out

def apply_mutations(base: dict, mutations: list[list[object]]) -> dict:
    out = copy.deepcopy(base)
    for op in mutations:
        if op[0] == "set_node":
            n = next((x for x in out["nodes"] if x["id"] == op[1]), None)
            if n is None: raise ValueError(f"unknown node: {op[1]}")
            n[op[2]] = op[3]
        elif op[0] == "add_node": out["nodes"].append(copy.deepcopy(op[1]))
        elif op[0] == "add_edge": out["edges"].append(copy.deepcopy(op[1]))
        elif op[0] == "remove_edge": out["edges"].remove(op[1])
        elif op[0] == "remove_node":
            out["nodes"] = [n for n in out["nodes"] if n["id"] != op[1]]
            out["edges"] = [e for e in out["edges"] if e[0] != op[1] and e[1] != op[1]]
        elif op[0] == "reverse_collection": out[op[1]].reverse()
        else: raise ValueError(f"unknown mutation: {op[0]}")
    return out

def set_path(obj: dict, dotted: str, value: object) -> None:
    t = obj
    p = dotted.split(".")
    for x in p[:-1]: t = t[x]
    t[p[-1]] = value

def epoch_index(v: object) -> int | None:
    return EPOCHS.index(v) if v in EPOCHS else None

def validate_observed_state(observed: dict, expected: str) -> str | None:
    if digest(observed) != expected: return "observed-pin"
    if observed.get("schema") != OBSERVED_STATE_SCHEMA or observed.get("status") != "research-anchor-observed-state-only": return "observed-schema"
    if observed.get("trust_root_id") != ROOT_ID or observed.get("authority_id") != AUTHORITY_ID: return "observed-authority"
    if not isinstance(observed.get("highest_manifest_version"), int) or observed.get("highest_manifest_version") < 1: return "observed-version"
    if not isinstance(observed.get("highest_manifest_sha256"), str): return "observed-commitment"
    if not isinstance(observed.get("observed_subject_fixture_git_blob_sha"), str): return "observed-subject"
    return None

def validate_root(root: dict, manifest: dict, expected: str) -> str | None:
    if digest(root) != expected: return "root-pin"
    if root.get("schema") != ROOT_SCHEMA or root.get("status") != "research-anchor-trust-root-only": return "root-schema"
    if root.get("trust_root_id") != ROOT_ID or root.get("authority_id") != AUTHORITY_ID: return "root-authority"
    if root.get("current_manifest_sha256") != digest(manifest): return "manifest-root-binding"
    if root.get("current_manifest_version") != manifest.get("manifest_version"): return "manifest-version"
    lo, hi, cur = root.get("minimum_manifest_version"), root.get("maximum_manifest_version"), root.get("current_manifest_version")
    if not all(isinstance(x, int) for x in (lo, hi, cur)) or not lo <= cur <= hi: return "manifest-version-range"
    if epoch_index(root.get("current_epoch")) is None: return "root-epoch"
    return None

def validate_manifest(manifest: dict, previous: dict, subject_bytes: bytes, policy_sha: str, graph: dict, current_epoch_name: str) -> str | None:
    if manifest.get("schema") != MANIFEST_SCHEMA or manifest.get("status") != "research-anchor-manifest-only": return "manifest-schema"
    if manifest.get("authority_id") != AUTHORITY_ID: return "manifest-authority"
    version = manifest.get("manifest_version")
    if not isinstance(version, int) or version < 1: return "manifest-version"
    if version > 1:
        if manifest.get("previous_manifest_sha256") != digest(previous): return "previous-manifest-link"
        if previous.get("manifest_version") != version - 1: return "previous-manifest-version"
    issued, expiry, current = epoch_index(manifest.get("issued_epoch")), epoch_index(manifest.get("expires_after_epoch")), epoch_index(current_epoch_name)
    if issued is None or expiry is None or issued > expiry or current > expiry: return "manifest-epoch"
    if manifest.get("subject_fixture_git_blob_sha") != git_blob_sha_bytes(subject_bytes): return "subject-fixture-binding"
    if manifest.get("policy_blob_sha") != policy_sha: return "policy-binding"
    nodes = node_index(graph)
    if nodes is None or nodes.get("claim", {}).get("type") != "Claim": return "claim-root"
    if manifest.get("claim_scope_anchor") != nodes["claim"].get("claim_scope_anchor"): return "claim-scope-binding"
    entries = manifest.get("entries")
    if not isinstance(entries, dict): return "entries-structure"
    for e in entries.values():
        if not isinstance(e, dict) or e.get("type") != "CensoringClassification" or not isinstance(e.get("commitment"), str): return "entry-structure"
        r = epoch_index(e.get("registration_epoch"))
        if r is None or r < issued or r > expiry: return "registration-epoch"
    return None

def manifest_for_case(c: dict, current: dict, previous: dict) -> dict:
    m = copy.deepcopy(previous if c.get("manifest_source") == "previous" else current)
    for op in c.get("manifest_mutations", []): set_path(m, op[1], op[2])
    return m

def root_for_case(c: dict, base: dict, manifest: dict) -> dict:
    r = copy.deepcopy(base)
    if c.get("derive_manifest_hash"): r["current_manifest_sha256"] = digest(manifest)
    for op in c.get("root_mutations", []): set_path(r, op[1], op[2])
    return r

def evaluate(c: dict, fixture: dict, current: dict, previous: dict, base_root: dict, observed: dict, policy_sha: str, fixture_bytes: bytes, expected_root: str, expected_observed: str) -> str:
    source = {x["case_id"]: x for x in fixture["cases"]}.get(c.get("source_case_id"))
    if source is None: return "unresolved"
    subject = apply_mutations(fixture["base_graph"], source["mutation"] + c.get("subject_mutations", []))
    manifest = manifest_for_case(c, current, previous)
    root = root_for_case(c, base_root, manifest)
    expected = expected_root if c.get("mode") == "fixed-pin" else digest(root)
    if c.get("mode") not in {"fixed-pin","semantic-liveness"}: return "unresolved"
    if validate_observed_state(observed, expected_observed): return "unresolved"
    if validate_root(root, manifest, expected): return "unresolved"
    subject_bytes = fixture_bytes + c.get("fixture_suffix","").encode("utf-8")
    if validate_manifest(manifest, previous, subject_bytes, policy_sha, subject, root.get("current_epoch")): return "unresolved"
    nodes = node_index(subject)
    assert nodes is not None
    observed_version = observed["highest_manifest_version"]
    candidate_version = manifest["manifest_version"]
    candidate_manifest_sha = digest(manifest)
    if candidate_version < observed_version: return "unresolved"
    if candidate_version == observed_version:
        if candidate_manifest_sha != observed["highest_manifest_sha256"]: return "unresolved"
    elif candidate_version == observed_version + 1:
        if manifest.get("previous_manifest_sha256") != observed["highest_manifest_sha256"]: return "unresolved"
    else:
        return "unresolved"
    if manifest.get("subject_fixture_git_blob_sha") != observed.get("observed_subject_fixture_git_blob_sha") and candidate_version == observed_version:
        return "unresolved"
    for node_id, entry in manifest["entries"].items():
        node = nodes.get(node_id)
        if node is None: continue
        if not isinstance(node.get("commitment"), str): return "unresolved"
        ni = digest({"id":node_id,"type":node.get("type"),"commitment":node["commitment"]})
        mi = digest({"id":node_id,"type":entry["type"],"commitment":entry["commitment"]})
        if ni != mi: return "unqualified"
        if entry.get("status") != "active": return "unresolved"
        rec, reg = epoch_index(node.get("classification_epoch")), epoch_index(entry.get("registration_epoch"))
        if rec is None or reg is None or reg > rec: return "unresolved"
    return "qualified"

def main() -> int:
    if len(sys.argv) != 9:
        print("usage: verify_anchor_governance.py EXPECTED_ROOT_SHA EXPECTED_OBSERVED_SHA OBSERVED.json ROOT.json CURRENT_MANIFEST.json PREVIOUS_MANIFEST.json POLICY.json CAMPAIGN.json REPORT.json", file=sys.stderr); return 2
    expected_root, expected_observed, observed_path, root_path, current_path, previous_path, policy_path, campaign_path, report_path = map(Path, sys.argv[1:10])
    expected = str(expected_root)
    root = json.loads(root_path.read_text(encoding="utf-8"))
    observed = json.loads(observed_path.read_text(encoding="utf-8"))
    current = json.loads(current_path.read_text(encoding="utf-8"))
    previous = json.loads(previous_path.read_text(encoding="utf-8"))
    campaign = json.loads(campaign_path.read_text(encoding="utf-8"))
    fixture_path = policy_path.parent / "CONTINUAL_ADAPTATION_CENSORING_CLASSIFICATION_FIXTURES.json"
    fixture_bytes = fixture_path.read_bytes()
    fixture = json.loads(fixture_bytes)
    if campaign.get("schema") != CAMPAIGN_SCHEMA or campaign.get("expected_trust_root_sha256") != expected or campaign.get("expected_observed_state_sha256") != str(expected_observed):
        print("campaign root/schema/observed pin mismatch", file=sys.stderr); return 1
    ids = [c.get("case_id") for c in campaign.get("cases", [])]
    fixed_ids = [c.get("case_id") for c in fixture.get("cases", [])]
    if len(ids) != len(set(ids)) or any(not isinstance(x, str) for x in ids) or len(fixed_ids) != len(set(fixed_ids)) or any(not isinstance(x, str) for x in fixed_ids):
        print("duplicate or invalid case IDs", file=sys.stderr); return 1
    policy_sha = git_blob_sha(policy_path)
    failures, rows = [], []
    for c in campaign["cases"]:
        verdict = evaluate(c, fixture, current, previous, root, observed, policy_sha, fixture_bytes, expected, str(expected_observed))
        rows.append({"actual_verdict":verdict,"case_id":c["case_id"],"expected_verdict":c["expected_verdict"],"mode":c["mode"]})
        if verdict != c["expected_verdict"]: failures.append([c["case_id"],c["expected_verdict"],verdict])
    report = {"cases":rows,"failures":failures,"external_trust_root_sha256":expected,"external_observed_state_sha256":str(expected_observed),"current_manifest_version":current.get("manifest_version"),"schema":"mycelix.continual-adaptation.censoring-classification-anchor-governance-report.v1","status":"research-evidence-only"}
    report_path.write_text(canonical(report).decode("utf-8")+"\n",encoding="utf-8")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0

if __name__ == "__main__": raise SystemExit(main())
