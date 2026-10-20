#!/usr/bin/env python3
from __future__ import annotations

import argparse, hashlib, json, re, subprocess
from pathlib import Path

def run(cmd):
    return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)

def sha(b): return hashlib.sha256(b).hexdigest()
def file_sha(p): return sha(p.read_bytes())

def alloy_rows(out):
    rows={}
    for line in out.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            x=json.loads(line); rows[x["label"]]=x["actual"]
    return rows

def remove_fact(src,name):
    marker=f"fact {name} {{"
    start=src.find(marker)
    if start<0: raise RuntimeError("mutation fact not found")
    brace=src.find("{",start); depth=0
    for i in range(brace,len(src)):
        if src[i]=="{": depth+=1
        elif src[i]=="}":
            depth-=1
            if depth==0: return src[:start]+src[i+1:]
    raise RuntimeError("unterminated fact")

p=argparse.ArgumentParser()
for n in ("matrix","tla","canonical-cfg","negative-cfg","negative-tla","alloy","runner-class-dir","alloy-jar","tla-jar","reference","runtime","evidence-dir"):
    p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True)

m=json.loads(a.matrix.read_text())
c=m["control"]
assert m["schema"]=="mycelix.evidence-attestation-claim-authorization-control-matrix.v1"
assert c["id"]=="unauthorized-claim"

head=run(["git","rev-parse","HEAD"]); tree=run(["git","rev-parse","HEAD^{tree}"])
if head.returncode or tree.returncode: raise RuntimeError("exact head/tree unavailable")
runtime=json.loads(a.runtime.read_text())
assert runtime["schema"]=="mycelix.evidence-attestation-claim-formal-runtime.v1"
receipt={"receipt_schema":"mycelix.evidence-attestation-claim-formal-receipt.v1","result":"ExecutedFail",
         "repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},
         "runtime":runtime,
         "input_sha256":{str(x):file_sha(x) for x in (a.matrix,a.tla,a.canonical_cfg,a.negative_cfg,a.negative_tla,a.alloy,a.reference,a.runtime)}}

ref=run(["python3",str(a.reference)])
(a.evidence_dir/"reference.log").write_text(ref.stdout)
if ref.returncode: raise RuntimeError("reference failed")
for marker in [
"CANONICAL PASS: authorized trusted subject-bound claim may accompany a grant-backed authority transition",
"ISOLATION PASS: unauthorized claim leaves steady-state grant provenance valid",
"NEGATIVE PASS: unauthorized claim authority delta detected",
]:
    if marker not in ref.stdout: raise RuntimeError("missing reference marker: "+marker)

def tlc(cfg,label):
    r=run(["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-config",str(cfg),str(a.tla)])
    (a.evidence_dir/(label+".log")).write_text(r.stdout)
    return set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.",r.stdout))

if tlc(a.canonical_cfg,"tla-canonical"):
    raise RuntimeError("canonical TLA violated")
viol=tlc(a.negative_cfg,"tla-negative")
if viol!={"UnauthorizedClaimCannotChangeAuthority"}:
    raise RuntimeError("TLA negative not isolated: "+repr(sorted(viol)))
receipt["reference"]="PASS"; receipt["tla"]={"canonical":"PASS","negative":sorted(viol)}

cp=f"{a.runner_class_dir}:{a.alloy_jar}"
can=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(a.alloy)])
(a.evidence_dir/"alloy-canonical.log").write_text(can.stdout)
if can.returncode: raise RuntimeError("canonical Alloy failed")
rows=alloy_rows(can.stdout)
expected={"ValidAuthorizedAuthorityDeltaWitness":"SAT","UnauthorizedClaimGrantBackedDeltaWitness":"UNSAT","UnauthorizedClaimCannotMintAuthority":"UNSAT"}
if {k:rows.get(k) for k in expected}!=expected:
    raise RuntimeError("canonical Alloy mismatch: "+repr(rows))
src=a.alloy.read_text()
mutant_path=a.evidence_dir/"alloy-negative-unauthorized-claim.als"
mutant_path.write_text(remove_fact(src,"UnauthorizedClaimNoAuthorityDelta"))
mut=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(mutant_path)])
(a.evidence_dir/"alloy-negative.log").write_text(mut.stdout)
if mut.returncode: raise RuntimeError("Alloy mutation runner failed")
mrows=alloy_rows(mut.stdout)
changed={k for k in set(rows)|set(mrows) if rows.get(k)!=mrows.get(k)}
allowed={"UnauthorizedClaimGrantBackedDeltaWitness","UnauthorizedClaimCannotMintAuthority"}
if mrows.get("UnauthorizedClaimGrantBackedDeltaWitness")!="SAT" or mrows.get("UnauthorizedClaimCannotMintAuthority")!="SAT":
    raise RuntimeError("target Alloy mutation did not become SAT")
if changed!=allowed: raise RuntimeError("unrelated Alloy outcome changed: "+repr(sorted(changed)))
receipt["alloy"]={"canonical":rows,"negative":mrows,"changed_outcomes":sorted(changed)}
receipt["result"]="ExecutedPass"
(a.evidence_dir/"evidence-attestation-claim-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
