from __future__ import annotations
import argparse,hashlib,json,re,subprocess
from pathlib import Path
IDS=["interval-widening","interval-hole-filling","wildcard-expansion","temporal-window-widening","context-weakening","normalization-equivalence","non-equivalent-normalization","deny-set-deletion","conflict-rule-substitution","unknown-extension-treated-as-subsumed","unsupported-compound-extension"]
INVS={"interval-widening":"IntervalWideningRejected","interval-hole-filling":"IntervalHoleFillingRejected","wildcard-expansion":"WildcardExpansionRejected","temporal-window-widening":"TemporalWideningRejected","context-weakening":"ContextWeakeningRejected","normalization-equivalence":"NormalizationEquivalenceAccepted","non-equivalent-normalization":"NormalizationNonEquivalentRejected","deny-set-deletion":"DenyDeletionRejected","conflict-rule-substitution":"ConflictRuleSubstitutionRejected","unknown-extension-treated-as-subsumed":"UnknownExtensionRejected","unsupported-compound-extension":"CompoundExtensionRejected"}
def run(cmd): return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
def violations(out): return set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",out))
def alloy_rows(out):
    r={}
    for line in out.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            x=json.loads(line); r[x["label"]]=x["actual"]
    return r
p=argparse.ArgumentParser()
for n in ("matrix","reference","tla","canonical-cfg","negative-cfg-dir","alloy","runner-class-dir","alloy-jar","tla-jar","runtime","evidence-dir"):
    p.add_argument("--"+n,type=Path,required=True)
a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True)
m=json.loads(a.matrix.read_text()); controls=m["controls"]; assert [c["id"] for c in controls]==IDS
head=run(["git","rev-parse","HEAD"]); tree=run(["git","rev-parse","HEAD^{tree}"])
if head.returncode or tree.returncode: raise RuntimeError("head/tree resolution failed")
inputs=[a.matrix,a.reference,a.tla,a.canonical_cfg,a.alloy,a.runtime]+[a.negative_cfg_dir/f"EvidenceAttestationCapabilityConstraintAlgebraV1Negative-{x}.cfg" for x in IDS]
receipt={"receipt_schema":"mycelix.evidence-attestation-capability-constraint-algebra-formal-receipt.v1","result":"ExecutedFail","repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},"runtime":json.loads(a.runtime.read_text()),"inputs":{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},"controls":IDS}
rr=run(["python3",str(a.reference)]); (a.evidence_dir/"reference.log").write_text(rr.stdout)
if rr.returncode: raise RuntimeError("reference oracle failed")
receipt["reference"]={"returncode":0,"stdout_sha256":hashlib.sha256(rr.stdout.encode()).hexdigest()}
def tlc(cfg,label):
    md=a.evidence_dir/(label+"-metadir"); md.mkdir(parents=True,exist_ok=True)
    r=run(["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-metadir",str(md),"-config",str(cfg),str(a.tla)])
    (a.evidence_dir/(label+".log")).write_text(r.stdout)
    if r.returncode==0 and "Model checking completed. No error has been found." in r.stdout:return set()
    v=violations(r.stdout)
    if v:return v
    raise RuntimeError("TLC failed without classified invariant: "+label)
v=tlc(a.canonical_cfg,"tla-canonical")
if v: raise RuntimeError("canonical TLA violation: "+repr(sorted(v)))
receipt["tla"]={"canonical":"PASS","negatives":{}}
for c in controls:
    v=tlc(a.negative_cfg_dir/f"EvidenceAttestationCapabilityConstraintAlgebraV1Negative-{c['id']}.cfg","tla-negative-"+c["id"])
    expected=set() if c["id"]=="normalization-equivalence" else {INVS[c["id"]]}
    if v!=expected: raise RuntimeError("TLA control mismatch "+c["id"]+": expected "+repr(sorted(expected))+" got "+repr(sorted(v)))
    receipt["tla"]["negatives"][c["id"]]=sorted(v)
cp=f"{a.runner_class_dir}:{a.alloy_jar}"
ar=run(["java","-cp",cp,"AgentDelegationAuthorityAlloyRunner",str(a.alloy)])
(a.evidence_dir/"alloy.log").write_text(ar.stdout)
if ar.returncode: raise RuntimeError("Alloy runner failed")
rows=alloy_rows(ar.stdout)
required=["PolicyOrderReflexive","PolicyOrderTransitive","PolicyOrderAntisymmetricModuloSemanticEquivalence","CanonicalAggregate","IntervalWideningRejected","HoleFillingRejected","WildcardExpansionRejected","TemporalWideningRejected","ContextWeakeningRejected","NormalizationEquivalentAccepted","NormalizationNonEquivalentRejected","DenyDeletionRejected","ConflictSubstitutionRejected","UnknownExtensionRejected","CompoundExtensionRejected"]
for k in required:
    if rows.get(k)!="UNSAT": raise RuntimeError("Alloy assertion not UNSAT "+k+": "+repr(rows.get(k)))
receipt["alloy"]={k:rows[k] for k in required}
receipt["result"]="ExecutedPass"
(a.evidence_dir/"evidence-attestation-capability-constraint-algebra-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
