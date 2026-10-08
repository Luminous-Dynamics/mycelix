from __future__ import annotations
import argparse,hashlib,json,re,subprocess
from pathlib import Path
IDS=["cartesian-recombination","target-currency-correlation","operation-argument-correlation","audience-target-correlation","wildcard-expansion","capability-set-identity-substitution","tuple-normalization-substitution","profile-widening-unchanged-marginals","effect-outside-relation-inside-marginals","downstream-relational-subset-bypass"]
def run(c): return subprocess.run(c,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
def rows(out):
 r={}
 for line in out.splitlines():
  if line.startswith("{") and '"label"' in line and '"actual"' in line:
   x=json.loads(line); r[x["label"]]=x["actual"]
 return r
p=argparse.ArgumentParser()
for n in ("matrix","tla","canonical-cfg","negative-tla","negative-cfg-dir","alloy","runner-class-dir","alloy-jar","tla-jar","reference","runtime","evidence-dir"): p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True); m=json.loads(a.matrix.read_text()); controls=m["controls"]
assert [c["id"] for c in controls]==IDS
head=run(["git","rev-parse","HEAD"]); tree=run(["git","rev-parse","HEAD^{tree}"])
runtime=json.loads(a.runtime.read_text())
cfgs=[a.negative_cfg_dir/f"EvidenceAttestationRelationalCapabilityContainmentV1Negative-{c['id']}.cfg" for c in controls]
if any(not x.is_file() for x in cfgs): raise RuntimeError("missing negative CFG")
inputs=[a.matrix,a.tla,a.canonical_cfg,a.negative_tla,a.alloy,a.reference,a.runtime]+cfgs
receipt={"receipt_schema":"mycelix.evidence-attestation-relational-capability-containment-formal-receipt.v1","result":"ExecutedFail","repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},"runtime":runtime,"inputs":{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in inputs},"controls":IDS}
rr=run(["python3",str(a.reference)]); (a.evidence_dir/"reference.log").write_text(rr.stdout)
if rr.returncode or "CANONICAL PASS" not in rr.stdout: raise RuntimeError("reference failed")
for c in controls:
 if rr.stdout.count(c["reference_marker"])!=1: raise RuntimeError("reference marker mismatch "+c["id"])
receipt["reference"]={"returncode":0,"stdout_sha256":hashlib.sha256(rr.stdout.encode()).hexdigest()}
def tlc(cfg,module,label):
 md=a.evidence_dir/(label+"-metadir"); md.mkdir(parents=True,exist_ok=True)
 r=run(["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-metadir",str(md),"-config",str(cfg),str(module)])
 (a.evidence_dir/(label+".log")).write_text(r.stdout)
 if r.returncode==0 and "Model checking completed. No error has been found." in r.stdout:return set()
 return set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",r.stdout))
v=tlc(a.canonical_cfg,a.tla,"tla-canonical")
if v: raise RuntimeError("canonical TLA violation "+repr(sorted(v)))
receipt["tla"]={"canonical":"PASS","negatives":{}}
for c in controls:
 v=tlc(a.negative_cfg_dir/f"EvidenceAttestationRelationalCapabilityContainmentV1Negative-{c['id']}.cfg",a.negative_tla,"tla-negative-"+c["id"])
 if v!={c["tla_invariant"]}: raise RuntimeError("TLA isolation mismatch "+c["id"]+": "+repr(sorted(v)))
 receipt["tla"]["negatives"][c["id"]]=sorted(v)
cp=f"{a.runner_class_dir}:{a.alloy_jar}"; ar=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(a.alloy)])
(a.evidence_dir/"alloy.log").write_text(ar.stdout)
if ar.returncode: raise RuntimeError("Alloy runner failed")
rows0=rows(ar.stdout)
for k in ("CapabilitySetOrderReflexive","CapabilitySetOrderTransitive","CapabilitySetOrderAntisymmetricModuloEquivalence","CanonicalAggregate"):
 if rows0.get(k)!="UNSAT": raise RuntimeError("canonical Alloy result mismatch "+k+":"+repr(rows0.get(k)))
for c in controls:
 if rows0.get(c["alloy_witness"]) is not None: raise RuntimeError("canonical witness label unexpectedly present as result: "+c["id"])
receipt["alloy"]={"canonical":{k:rows0.get(k) for k in ("CapabilitySetOrderReflexive","CapabilitySetOrderTransitive","CapabilitySetOrderAntisymmetricModuloEquivalence","CanonicalAggregate")}}
receipt["result"]="ExecutedPass"
(a.evidence_dir/"evidence-attestation-relational-capability-containment-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
