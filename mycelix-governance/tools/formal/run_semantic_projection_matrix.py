#!/usr/bin/env python3
from __future__ import annotations
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

IDS=["uncovered-field","unknown-field","value-substitution","source-substitution","implicit-default","schema-profile","required-omission"]

def run(cmd):
    return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)

def fsha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def rows(out):
    result={}
    for line in out.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            obj=json.loads(line)
            result[obj["label"]]=obj["actual"]
    return result

def remove_fact(source,name):
    marker="fact "+name+" {"
    start=source.find(marker)
    if start<0:
        raise RuntimeError("fact not found: "+name)
    brace=source.find("{",start)
    depth=0
    for i in range(brace,len(source)):
        if source[i]=="{":
            depth+=1
        elif source[i]=="}":
            depth-=1
            if depth==0:
                return source[:start]+source[i+1:]
    raise RuntimeError("unterminated fact: "+name)

p=argparse.ArgumentParser()
for n in ("matrix","tla","canonical-cfg","negative-tla","negative-cfg-dir","alloy","runner-class-dir","alloy-jar","tla-jar","reference","runtime","evidence-dir"):
    p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args()
a.evidence_dir.mkdir(parents=True,exist_ok=True)

matrix=json.loads(a.matrix.read_text())
assert matrix["schema"]=="mycelix.evidence-attestation-semantic-projection-control-matrix.v1"
controls=matrix["controls"]
assert [c["id"] for c in controls]==IDS

head=run(["git","rev-parse","HEAD"])
tree=run(["git","rev-parse","HEAD^{tree}"])
if head.returncode or tree.returncode:
    raise RuntimeError("head/tree resolution failed")
runtime=json.loads(a.runtime.read_text())
assert runtime["schema"]=="mycelix.evidence-attestation-capability-formal-runtime.v1"

negative_cfgs=[a.negative_cfg_dir/("EvidenceAttestationSemanticProjectionV1Negative-"+c["id"]+".cfg") for c in controls]
if any(not path.is_file() for path in negative_cfgs):
    raise RuntimeError("negative CFG missing")

inputs=[a.matrix,a.tla,a.canonical_cfg,a.negative_tla,a.alloy,a.reference,a.runtime]+negative_cfgs
receipt={
    "receipt_schema":"mycelix.evidence-attestation-semantic-projection-formal-receipt.v1",
    "result":"ExecutedFail",
    "repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},
    "runtime":runtime,
    "inputs":{str(path):fsha(path) for path in inputs},
    "controls":IDS,
}

reference=run(["python3",str(a.reference)])
(a.evidence_dir/"reference.log").write_text(reference.stdout)
if reference.returncode:
    raise RuntimeError("reference failed")
if "NEGATIVE PASS" not in reference.stdout:
    raise RuntimeError("reference negative marker missing")
receipt["reference"]={"returncode":0,"stdout_sha256":hashlib.sha256(reference.stdout.encode()).hexdigest()}

def tlc(cfg,module,label):
    metadir=a.evidence_dir/(label+"-metadir")
    metadir.mkdir(parents=True,exist_ok=True)
    result=run(["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-metadir",str(metadir),"-config",str(cfg),str(module)])
    (a.evidence_dir/(label+".log")).write_text(result.stdout)
    if result.returncode==0 and "Model checking completed. No error has been found." in result.stdout:
        return set()
    violations=set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",result.stdout))
    if violations:
        return violations
    raise RuntimeError("TLC execution failed without a classified invariant violation: "+" | ".join(result.stdout.splitlines()[-12:]))

canonical=tlc(a.canonical_cfg,a.tla,"tla-canonical")
if canonical:
    raise RuntimeError("canonical TLA violations: "+repr(sorted(canonical)))
receipt["tla"]={"canonical":"PASS","negatives":{}}

for c in controls:
    violations=tlc(a.negative_cfg_dir/("EvidenceAttestationSemanticProjectionV1Negative-"+c["id"]+".cfg"),a.negative_tla,"tla-negative-"+c["id"])
    expected={c["tla_invariant"]}
    if violations!=expected:
        raise RuntimeError("TLA isolation mismatch "+c["id"]+": "+repr(sorted(violations)))
    receipt["tla"]["negatives"][c["id"]]=sorted(violations)

cp=f"{a.runner_class_dir}:{a.alloy_jar}"
canonical_run=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(a.alloy)])
(a.evidence_dir/"alloy-canonical.log").write_text(canonical_run.stdout)
if canonical_run.returncode:
    raise RuntimeError("canonical Alloy runner failed")
canonical_rows=rows(canonical_run.stdout)
if canonical_rows.get("CanonicalProjectionWitness")!="SAT":
    raise RuntimeError("canonical Alloy witness mismatch: "+repr(canonical_rows))
if canonical_rows.get("SemanticProjectionExact")!="UNSAT":
    raise RuntimeError("canonical Alloy assertion mismatch: "+repr(canonical_rows))
for c in controls:
    if canonical_rows.get(c["alloy_witness"])!="UNSAT":
        raise RuntimeError("canonical witness not UNSAT "+c["id"])

receipt["alloy"]={"canonical":canonical_rows,"mutations":{}}
source=a.alloy.read_text()

for c in controls:
    mutant=a.evidence_dir/("alloy-negative-"+c["id"]+".als")
    mutant.write_text(remove_fact(source,c["alloy_mutant_fact"]))
    result=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(mutant)])
    (a.evidence_dir/("alloy-negative-"+c["id"]+".log")).write_text(result.stdout)
    if result.returncode:
        raise RuntimeError("Alloy mutant failed "+c["id"])
    actual=rows(result.stdout)
    if actual.get(c["alloy_witness"])!="SAT":
        raise RuntimeError("target Alloy witness not SAT "+c["id"]+": "+repr(actual))
    if actual.get("SemanticProjectionExact")!="SAT":
        raise RuntimeError("aggregate Alloy assertion not SAT "+c["id"]+": "+repr(actual))
    for other in controls:
        if other["id"]!=c["id"] and actual.get(other["alloy_witness"])!="UNSAT":
            raise RuntimeError("unrelated Alloy witness became SAT for "+c["id"]+": "+repr(actual))
    changed={k for k in set(canonical_rows)|set(actual) if canonical_rows.get(k)!=actual.get(k)}
    if changed!={c["alloy_witness"],"SemanticProjectionExact"}:
        raise RuntimeError("unrelated Alloy outcomes "+c["id"]+": "+repr(sorted(changed)))
    receipt["alloy"]["mutations"][c["id"]]={"removed_fact":c["alloy_mutant_fact"],"changed_outcomes":sorted(changed),"outcomes":actual}

receipt["result"]="ExecutedPass"
(a.evidence_dir/"evidence-attestation-semantic-projection-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
