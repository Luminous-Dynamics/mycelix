#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re, subprocess
from pathlib import Path

IDS=["decision-identity","intent-identity","operation-commitment","use-commitment","target","adapter","invocation-identity"]

def run(cmd): return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
def fsha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(out):
 r={}
 for line in out.splitlines():
  if line.startswith("{") and '"label"' in line and '"actual"' in line:
   o=json.loads(line); r[o["label"]]=o["actual"]
 return r
def remove_fact(source,name):
 marker="fact "+name+" {"; start=source.find(marker)
 if start<0: raise RuntimeError("fact not found: "+name)
 brace=source.find("{",start); depth=0
 for i in range(brace,len(source)):
  if source[i]=="{": depth+=1
  elif source[i]=="}":
   depth-=1
   if depth==0: return source[:start]+source[i+1:]
 raise RuntimeError("unterminated fact: "+name)

p=argparse.ArgumentParser()
for n in ("matrix","tla","canonical-cfg","negative-tla","negative-cfg-dir","alloy","runner-class-dir","alloy-jar","tla-jar","reference","runtime","evidence-dir"):
 p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True)
m=json.loads(a.matrix.read_text()); assert m["schema"]=="mycelix.evidence-attestation-effect-handoff-identity-control-matrix.v1"
controls=m["controls"]; assert [c["id"] for c in controls]==IDS
head=run(["git","rev-parse","HEAD"]); tree=run(["git","rev-parse","HEAD^{tree}"])
if head.returncode or tree.returncode: raise RuntimeError("head/tree resolution failed")
runtime=json.loads(a.runtime.read_text()); assert runtime["schema"]=="mycelix.evidence-attestation-capability-formal-runtime.v1"
negative_cfgs=[a.negative_cfg_dir/("EvidenceAttestationEffectHandoffIdentityV1Negative-"+c["id"]+".cfg") for c in controls]
if any(not p.is_file() for p in negative_cfgs): raise RuntimeError("negative CFG missing")
inputs=[a.matrix,a.tla,a.canonical_cfg,a.negative_tla,a.alloy,a.reference,a.runtime]+negative_cfgs
receipt={"receipt_schema":"mycelix.evidence-attestation-effect-handoff-identity-formal-receipt.v1","result":"ExecutedFail","repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},"runtime":runtime,"inputs":{str(pth):fsha(pth) for pth in inputs},"controls":IDS}
ref=run(["python3",str(a.reference)]); (a.evidence_dir/"reference.log").write_text(ref.stdout)
if ref.returncode: raise RuntimeError("reference failed")
if "NEGATIVE PASS" not in ref.stdout: raise RuntimeError("reference negative marker missing")
receipt["reference"]={"returncode":0,"stdout_sha256":hashlib.sha256(ref.stdout.encode()).hexdigest()}

def tlc(cfg,module,label):
 md=a.evidence_dir/(label+"-metadir"); md.mkdir(parents=True,exist_ok=True)
 r=run(["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-metadir",str(md),"-config",str(cfg),str(module)])
 (a.evidence_dir/(label+".log")).write_text(r.stdout)
 if r.returncode==0 and "Model checking completed. No error has been found." in r.stdout: return set()
 v=set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",r.stdout))
 if v: return v
 raise RuntimeError("TLC failed without classified invariant violation: "+" | ".join(r.stdout.splitlines()[-10:]))

cv=tlc(a.canonical_cfg,a.tla,"tla-canonical")
if cv: raise RuntimeError("canonical TLA violations: "+repr(sorted(cv)))
receipt["tla"]={"canonical":"PASS","negatives":{}}
for c in controls:
 v=tlc(a.negative_cfg_dir/("EvidenceAttestationEffectHandoffIdentityV1Negative-"+c["id"]+".cfg"),a.negative_tla,"tla-negative-"+c["id"])
 if v != {c["tla_invariant"]}: raise RuntimeError("TLA isolation mismatch "+c["id"]+": "+repr(sorted(v)))
 receipt["tla"]["negatives"][c["id"]]=sorted(v)

cp=f"{a.runner_class_dir}:{a.alloy_jar}"
canon=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(a.alloy)])
(a.evidence_dir/"alloy-canonical.log").write_text(canon.stdout)
if canon.returncode: raise RuntimeError("canonical Alloy failed")
cr=rows(canon.stdout)
if cr.get("ValidHandoffWitness")!="SAT" or cr.get("IdentityConservation")!="UNSAT": raise RuntimeError("canonical Alloy mismatch: "+repr(cr))
for c in controls:
 if cr.get(c["alloy_witness"])!="UNSAT": raise RuntimeError("canonical witness not UNSAT "+c["id"])
receipt["alloy"]={"canonical":cr,"mutations":{}}
src=a.alloy.read_text()
for c in controls:
 mp=a.evidence_dir/("alloy-negative-"+c["id"]+".als")
 mp.write_text(remove_fact(src,c["alloy_mutant_fact"]))
 mr=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(mp)])
 (a.evidence_dir/("alloy-negative-"+c["id"]+".log")).write_text(mr.stdout)
 if mr.returncode: raise RuntimeError("Alloy mutant failed "+c["id"])
 rr=rows(mr.stdout)
 if rr.get(c["alloy_witness"])!="SAT" or rr.get(c["alloy_assertion"])!="SAT":
  raise RuntimeError("Alloy negative mismatch "+c["id"]+": "+repr(rr))
 other_witnesses=[x["alloy_witness"] for x in controls if x["id"]!=c["id"]]
 if any(rr.get(w)!="UNSAT" for w in other_witnesses):
  raise RuntimeError("other Alloy witnesses became SAT for "+c["id"]+": "+repr({w:rr.get(w) for w in other_witnesses}))
 changed={k for k in set(cr)|set(rr) if cr.get(k)!=rr.get(k)}
 if changed != {c["alloy_witness"],"IdentityConservation"}:
  raise RuntimeError("unrelated Alloy outcomes "+c["id"]+": "+repr(sorted(changed)))
 receipt["alloy"]["mutations"][c["id"]]={"removed_fact":c["alloy_mutant_fact"],"changed_outcomes":sorted(changed),"outcomes":rr}
receipt["result"]="ExecutedPass"
(a.evidence_dir/"evidence-attestation-effect-handoff-identity-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
