#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re, subprocess
from pathlib import Path

CONTROL_IDS=[
"decision-identity","authority-epoch","request-commitment","target","policy-epoch",
"adapter-profile","invocation-identity","capability-expiry","decision-horizon"
]
BOUND={x:"" for x in CONTROL_IDS}

def run(cmd):
    return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)

def sha(b): return hashlib.sha256(b).hexdigest()
def fsha(p): return sha(p.read_bytes())

def alloy_rows(out):
    rows={}
    for line in out.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            o=json.loads(line); rows[o["label"]]=o["actual"]
    return rows

def remove_fact(source,name):
    marker="fact "+name+" {"; start=source.find(marker)
    if start<0: raise RuntimeError("mutation fact not found: "+name)
    brace=source.find("{",start); depth=0
    for i in range(brace,len(source)):
        if source[i]=="{": depth+=1
        elif source[i]=="}":
            depth-=1
            if depth==0: return source[:start]+source[i+1:]
    raise RuntimeError("unterminated fact: "+name)

p=argparse.ArgumentParser()
for n in ("matrix","tla","canonical-cfg","negative-tla","negative-cfg-dir","alloy",
          "runner-class-dir","alloy-jar","tla-jar","reference","runtime","evidence-dir"):
    p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True)

m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-decision-effect-binding-control-matrix.v1"
controls=m["controls"]
assert [c["id"] for c in controls]==CONTROL_IDS

head=run(["git","rev-parse","HEAD"]); tree=run(["git","rev-parse","HEAD^{tree}"])
if head.returncode or tree.returncode: raise RuntimeError("exact head/tree resolution failed")
runtime=json.loads(a.runtime.read_text())
assert runtime["schema"]=="mycelix.evidence-attestation-capability-formal-runtime.v1"

receipt={
"receipt_schema":"mycelix.evidence-attestation-decision-effect-binding-formal-receipt.v1",
"result":"ExecutedFail",
"repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},
"control_matrix_sha256":fsha(a.matrix),
"runtime":runtime,
"inputs":{str(p):fsha(p) for p in [a.matrix,a.tla,a.canonical_cfg,a.negative_tla,a.alloy,a.reference,a.runtime]},
"controls":[c["id"] for c in controls],
}

ref=run(["python3",str(a.reference)])
(a.evidence_dir/"reference.log").write_text(ref.stdout)
if ref.returncode: raise RuntimeError("reference model failed")
for c in controls:
    if c["reference_marker"] not in ref.stdout: raise RuntimeError("reference marker missing: "+c["reference_marker"])
receipt["reference"]={"returncode":ref.returncode,"stdout_sha256":sha(ref.stdout.encode())}

def tlc(cfg,module,label):
    metadir=a.evidence_dir/(label+"-metadir"); metadir.mkdir(parents=True,exist_ok=True)
    r=run(["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-metadir",str(metadir),"-config",str(cfg),str(module)])
    (a.evidence_dir/(label+".log")).write_text(r.stdout)
    if r.returncode==0 and "Model checking completed. No error has been found." in r.stdout: return set()
    violations=set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",r.stdout))
    if violations: return violations
    raise RuntimeError("TLC failed without classified invariant violation: "+" | ".join(r.stdout.splitlines()[-10:]))

cv=tlc(a.canonical_cfg,a.tla,"tla-canonical")
if cv: raise RuntimeError("canonical TLA violated: "+repr(sorted(cv)))
receipt["tla"]={"canonical":"PASS","negatives":{}}

for c in controls:
    cfg=a.negative_cfg_dir/("EvidenceAttestationDecisionEffectBindingV1Negative-"+c["id"]+".cfg")
    violations=tlc(cfg,a.negative_tla,"tla-negative-"+c["id"])
    expected=c["tla_invariant"]
    if violations != {expected}: raise RuntimeError(c["id"]+" TLA isolation mismatch: "+repr(sorted(violations)))
    receipt["tla"]["negatives"][c["id"]]=sorted(violations)

cp=f"{a.runner_class_dir}:{a.alloy_jar}"
canon=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(a.alloy)])
(a.evidence_dir/"alloy-canonical.log").write_text(canon.stdout)
if canon.returncode: raise RuntimeError("canonical Alloy runner failed")
rows=alloy_rows(canon.stdout)
required={"ValidEffectAdmissionWitness":"SAT","DecisionToEffectAuthorityStillBound":"UNSAT"}
for c in controls:
    required[c["alloy_witness"]]="UNSAT"
for c in controls:
    required[c["alloy_assertion"]]="UNSAT"
if {k:rows.get(k) for k in required}!=required: raise RuntimeError("canonical Alloy mismatch: "+repr(rows))
receipt["alloy"]={"canonical":rows,"mutations":{}}

source=a.alloy.read_text()
for c in controls:
    mutant_path=a.evidence_dir/("alloy-negative-"+c["id"]+".als")
    mutant_path.write_text(remove_fact(source,c["alloy_mutant_fact"]))
    r=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(mutant_path)])
    (a.evidence_dir/("alloy-negative-"+c["id"]+".log")).write_text(r.stdout)
    if r.returncode: raise RuntimeError("Alloy mutant failed: "+c["id"])
    mr=alloy_rows(r.stdout)
    if mr.get(c["alloy_witness"])!="SAT" or mr.get(c["alloy_assertion"])!="SAT":
        raise RuntimeError("Alloy negative mismatch "+c["id"]+": "+repr(mr))
    changed={k for k in set(rows)|set(mr) if rows.get(k)!=mr.get(k)}
    allowed={c["alloy_witness"],c["alloy_assertion"],"DecisionToEffectAuthorityStillBound"}
    if changed!=allowed: raise RuntimeError("unrelated Alloy outcomes changed for "+c["id"]+": "+repr(sorted(changed)))
    receipt["alloy"]["mutations"][c["id"]]={"removed_fact":c["alloy_mutant_fact"],"changed_outcomes":sorted(changed),"outcomes":mr}

receipt["result"]="ExecutedPass"
(a.evidence_dir/"evidence-attestation-decision-effect-binding-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
