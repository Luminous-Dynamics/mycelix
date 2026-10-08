#!/usr/bin/env python3
"""Exact input + verifier inventory auditor for the layered Mycelix anchor transparency stack."""
from __future__ import annotations
import copy, hashlib, json, subprocess, sys
from pathlib import Path

BUNDLE_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle.v2"
CAMPAIGN_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-campaign.v2"
STACK_HEAD="62a12a46d450bc6f9fe8a4994c5fa05b44204011"
EXPECTED_BUNDLE_ID="mycelix.audit-bundle.v2@"+STACK_HEAD
WREG_ID="mycelix.research.anchor-witness-registry.v2"
VDS_ID="mycelix.research.anchor-statement-sequence.v1"
TS_ID="mycelix.research.anchor-cose-receipt-ts-registry.v1"
GOSSIP_ID="mycelix.research.anchor-observer-gossip-registry.v1"
COSE_REGISTRY_SHA="sha256:21772a0a1dbb88358c80e5297d4ec4dd82c2ba2bca474a334bae45535ec8b331"
LEGACY_TS_REGISTRY_SHA="sha256:efb549a023660010a07d70b7d78afbfd81664200fe4ecf1d0d542dc945f0bb54"
EXPECTED_TREEHEAD_KEYS={"size_4","size_7","fork_size_4","wrong_key","revoked_key","rollback_key","noncanonical"}
EXPECTED_TOPOLOGY={
 (4848,"5af2d8a2a4e42ccdcabc70160ad639d810642c86"),
 (4870,"2af796188bd7981fac111178141ae82165c2fc4e"),
 (4873,"e3a175b905a041a17630b92356e123ccba0d9175"),
 (4874,"b55af8c135a858abfb0663a762e7fd510e9ea6d2"),
 (4875,"7648801649b30bde5233ac380924a539b0cdbf35"),
 (4876,"1e352f442a4bc7bc537d0223cb39c5fe37944c56"),
 (4877,"61c46f16d32ee4d0bc42ffae76743caf19a76dd1"),
 (4879,"1c8f5a167c12ebed1e0bb4c09f1f7f8ee5a7e2c8"),
 (4889,"c6e69d0c9969b7b6d7feece65c60ae7c913e980d"),
 (4890,STACK_HEAD),
}
REQUIRED_VERIFIERS={
 "audit_bundle_python","audit_bundle_node","witness_crypto_python","witness_crypto_node",
 "vds_python","vds_node","rotation_python","rotation_node","tree_head_python","tree_head_node",
 "legacy_receipt_python","legacy_receipt_node","static_gossip_python","static_gossip_node",
 "cose_receipt_python","cose_receipt_node","gossip_simulation_python","gossip_simulation_node",
}

def canonical(v):
    return json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(",",":")).encode("utf-8")
def digest_obj(v):
    return "sha256:"+hashlib.sha256(canonical(v)).hexdigest()
def git_blob_sha(path: Path, repo_root: Path) -> str:
    return subprocess.check_output(["git","hash-object",str(path)],cwd=str(repo_root),text=True).strip()
def has_private_key_field(value):
    prohibited={"private_key","private_keys","secret_key","seed","private_seed","secret"}
    if isinstance(value,dict):
        return any(k.lower() in prohibited or has_private_key_field(v) for k,v in value.items())
    if isinstance(value,list):
        return any(has_private_key_field(x) for x in value)
    return False

def sha256(b): return hashlib.sha256(b).digest()
def leaf_hash(b): return sha256(b"\x00"+b)
def node_hash(a,b): return sha256(b"\x01"+a+b)
def mth(entries):
    n=len(entries)
    if n==0:return sha256(b"")
    if n==1:return leaf_hash(canonical(entries[0]))
    k=1<<((n-1).bit_length()-1)
    return node_hash(mth(entries[:k]),mth(entries[k:]))

def validate(bundle,repo_root:Path):
    if bundle.get("schema")!=BUNDLE_SCHEMA:return "bundle-schema"
    if bundle.get("status")!="research-evidence-only" or bundle.get("evidence_ceiling")!="exact-input-and-verifier-binding-only":return "bundle-status"
    if bundle.get("bundle_id")!=EXPECTED_BUNDLE_ID:return "bundle-id"
    if bundle.get("hosted_status")!="unclaimed":return "hosted-claim-injection"
    topology={(x.get("pr"),x.get("head")) for x in bundle.get("topology",[])}
    if topology!=EXPECTED_TOPOLOGY:return "topology"
    if set(bundle.get("verifiers",{}))!=REQUIRED_VERIFIERS:return "verifier-inventory"
    req=bundle.get("bindings",{}).get("required_verifiers",{})
    if set(req)!=(REQUIRED_VERIFIERS|{"hosted_pass"}):return "verifier-requirement-inventory"
    if any(req.get(k) is not True for k in REQUIRED_VERIFIERS) or req.get("hosted_pass") is not False:return "verifier-requirement-weakening"
    claims=bundle.get("bindings",{}).get("security_claims",{})
    expected_claims={"complete_scitt_interoperability","live_network_convergence","organizational_independence","private_key_custody_proven","hosted_pass"}
    if set(claims)!=expected_claims or any(claims[k] is not False for k in expected_claims):return "claim-ceiling-injection"
    arts=bundle.get("artifacts",{})
    verifiers=bundle.get("verifiers",{})
    for group,items in (("artifact",arts),("verifier",verifiers)):
        for name,pair in items.items():
            if not isinstance(pair,list) or len(pair)!=2:return f"{group}-record:{name}"
            rel,expected_sha=pair
            path=repo_root/rel
            if not path.is_file():return f"{group}-missing:{name}"
            if git_blob_sha(path,repo_root)!=expected_sha:return f"{group}-sha:{name}"
            if group=="artifact":
                try:obj=json.loads(path.read_text(encoding="utf-8"))
                except Exception:return f"artifact-json:{name}"
                if has_private_key_field(obj):return f"private-key-material:{name}"
    # Cross-layer identity and structural checks: these do not replace the upstream signature verifiers.
    def read(name):return json.loads((repo_root/arts[name][0]).read_text(encoding="utf-8"))
    v1=read("audit_bundle_v1")
    if v1.get("schema")!="mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle.v1":return "v1-bundle-schema"
    if v1.get("bundle_id")!="mycelix.audit-bundle.v1@61c46f16d32ee4d0bc42ffae76743caf19a76dd1":return "v1-bundle-id"
    if v1.get("artifacts",{}).get("vds_tree_heads")!=arts.get("tree_heads"):return "v1-tree-head-pin"
    reg=read("witness_registry");root=read("witness_root")
    if reg.get("registry_id")!=WREG_ID or reg.get("registry_version")!=2:return "witness-registry-binding"
    if root.get("registry_id")!=WREG_ID or root.get("registry_version")!=2 or root.get("registry_sha256")!=digest_obj(reg):return "witness-root-binding"
    if read("witness_baseline").get("registry_id")!=WREG_ID or read("witness_forward").get("registry_id")!=WREG_ID:return "witness-checkpoint-binding"
    tree=read("tree_heads")
    if tree.get("schema")!="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head-fixture.v1":return "tree-head-schema"
    if tree.get("vds_id")!=VDS_ID or set(tree.get("heads",{}))!=EXPECTED_TREEHEAD_KEYS:return "tree-head-topology"
    entries=read("vds_fixture").get("entries")
    if not isinstance(entries,list) or len(entries)<7:return "vds-entries"
    for name,size in (("size_4",4),("size_7",7)):
        attestations=tree["heads"][name].get("attestations",{})
        if len(attestations)!=4:return f"tree-head-quorum:{name}"
        roots={x.get("root_hash") for x in attestations.values()}
        versions={x.get("tree_size") for x in attestations.values()}
        if len(roots)!=1 or versions!={size}:return f"tree-head-agreement:{name}"
        if roots.pop()!="sha256:"+mth(entries[:size]).hex():return f"tree-head-merkle-root:{name}"
    legacy_ts=read("legacy_receipt_registry");legacy_receipt=read("legacy_receipt_fixture")
    if digest_obj(legacy_ts)!=LEGACY_TS_REGISTRY_SHA or legacy_receipt.get("ts_registry_sha256")!=digest_obj(legacy_ts):return "legacy-receipt-registry-binding"
    if legacy_receipt.get("vds_id")!=VDS_ID:return "legacy-receipt-vds-binding"
    cose_ts=read("cose_registry");cose=read("cose_fixture");cose_campaign=read("cose_campaign")
    if digest_obj(cose_ts)!=COSE_REGISTRY_SHA or cose.get("ts_registry_sha256")!=digest_obj(cose_ts) or cose.get("ts_registry_id")!=TS_ID:return "cose-registry-binding"
    if cose.get("vds_id")!=VDS_ID or cose.get("vds_algorithm_id")!=1 or cose.get("root_hash")!="sha256:"+mth(entries[:cose.get("tree_size",0)]).hex():return "cose-vds-binding"
    if cose_campaign.get("case_count")!=22 or len(cose_campaign.get("cases",[]))!=22:return "cose-campaign-shape"
    gossip_reg=read("gossip_registry");gossip=read("gossip_fixture");gossip_campaign=read("gossip_campaign")
    if gossip_reg.get("registry_id")!=GOSSIP_ID or gossip.get("vds_id")!=VDS_ID:return "gossip-binding"
    if gossip_campaign.get("case_count")!=16 or len(gossip_campaign.get("cases",[]))!=16:return "gossip-campaign-shape"
    sim=read("gossip_simulation");sim_campaign=read("gossip_simulation_campaign")
    if sim.get("schema")!="mycelix.continual-adaptation.censoring-classification-anchor-observer-gossip-simulation.v1":return "simulation-schema"
    if sim_campaign.get("case_count")!=8 or len(sim_campaign.get("cases",[]))!=8:return "simulation-campaign-shape"
    if bundle.get("bindings",{}).get("witness_registry_id")!=WREG_ID or bundle["bindings"].get("vds_id")!=VDS_ID:return "bundle-bindings"
    cose_profile=bundle["bindings"].get("cose",{})
    if cose_profile!={"cose_tag":18,"sign1":True,"detached_payload":True,"protected_labels":{"alg":1,"kid":4,"vds":395},"vds_algorithm":1,"vdp_label":396,"inclusion_proof_label":-1,"consistency_proof_label":-2,"cose_algorithm":-8}:return "cose-profile-substitution"
    return None

def mutate(bundle,mutation):
    b=copy.deepcopy(bundle)
    typ,_,name=mutation.partition(":")
    if typ=="artifact" and name in b["artifacts"]:b["artifacts"][name][1]="0"*40
    elif typ=="verifier" and name in b["verifiers"]:b["verifiers"][name][1]="0"*40
    elif typ=="binding" and name=="vds_id":b["bindings"]["vds_id"]="attacker.vds"
    elif typ=="topology" and name=="4890":
        next(x for x in b["topology"] if x["pr"]==4890)["head"]="0"*40
    elif typ=="disable_verifier" and name in b["bindings"]["required_verifiers"]:b["bindings"]["required_verifiers"][name]=False
    elif typ=="claim" and name in b["bindings"]["security_claims"]:b["bindings"]["security_claims"][name]=True
    elif mutation=="hosted_status":b["hosted_status"]="success"
    return b

def main():
    if len(sys.argv)!=5:
        print("usage: verifier REPO_ROOT BUNDLE CAMPAIGN REPORT",file=sys.stderr);return 2
    repo_root,bundle_path,campaign_path,report_path=map(Path,sys.argv[1:])
    bundle=json.loads(bundle_path.read_text());campaign=json.loads(campaign_path.read_text())
    if campaign.get("schema")!=CAMPAIGN_SCHEMA or campaign.get("case_count")!=25 or len(campaign.get("cases",[]))!=25:return 1
    case_ids=[x.get("case_id") for x in campaign["cases"]]
    if len(case_ids)!=len(set(case_ids)):return 1
    rows=[];failures=[]
    for case in campaign["cases"]:
        err=validate(mutate(bundle,case.get("mutation","")),repo_root)
        verdict="evidence-ready" if err is None else "unresolved"
        row={"case_id":case["case_id"],"expected_verdict":case["expected_verdict"],"actual_verdict":verdict,"reason":err or "inputs-verifiers-topology-and-claim-ceiling-match"}
        rows.append(row)
        if verdict!=case["expected_verdict"]:failures.append([case["case_id"],case["expected_verdict"],verdict,err])
    report={"schema":"mycelix.continual-adaptation.censoring-classification-anchor-audit-bundle-report.v2","status":"research-evidence-only","case_count":len(rows),"cases":rows,"failures":failures}
    Path(report_path).write_bytes(canonical(report)+b"\n")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0
if __name__=="__main__":raise SystemExit(main())
