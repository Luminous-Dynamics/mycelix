from __future__ import annotations
import argparse,hashlib,json,re,subprocess
from pathlib import Path

IDS=["monotonicity-sampled-only","monotonicity-violation-outside-sample","undeclared-adapter-domain",
"schema-semantic-type-substitution","unit-conversion-expansion","lossy-redaction-reconstitution",
"adapter-implementation-substitution","external-dependency-substitution","partial-adapter-success",
"domain-precondition-bypass"]

def run(cmd):
    return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)

def rows(output):
    result={}
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            item=json.loads(line); result[item["label"]]=item["actual"]
    return result

def tlc(jar,cfg,module,evidence,label):
    meta=evidence/(label+"-metadir"); meta.mkdir(parents=True,exist_ok=True)
    result=run(["java","-cp",str(jar),"tlc2.TLC","-workers","1","-metadir",str(meta),"-config",str(cfg),str(module)])
    (evidence/(label+".log")).write_text(result.stdout)
    if result.returncode==0 and "Model checking completed. No error has been found." in result.stdout:
        return set()
    violations=set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?",result.stdout))
    if violations: return violations
    raise RuntimeError("TLC failed without classified invariant: "+label)

parser=argparse.ArgumentParser()
for name in ("matrix","tla","canonical-cfg","negative-tla","negative-cfg-dir","alloy","runner-class-dir","alloy-jar","tla-jar","reference","runtime","evidence-dir"):
    parser.add_argument("--"+name,type=Path,required=True)
args=parser.parse_args()
args.evidence_dir.mkdir(parents=True,exist_ok=True)
matrix=json.loads(args.matrix.read_text()); controls=matrix["controls"]
assert [c["id"] for c in controls]==IDS

head=run(["git","rev-parse","HEAD"]); tree=run(["git","rev-parse","HEAD^{tree}"])
if head.returncode or tree.returncode: raise RuntimeError("repository identity resolution failed")
runtime=json.loads(args.runtime.read_text())
inputs=[args.matrix,args.tla,args.canonical_cfg,args.negative_tla,args.alloy,args.reference,args.runtime]+[
    args.negative_cfg_dir/f"EvidenceAttestationUniversalSemanticAdapterSoundnessV1Negative-{c['id']}.cfg" for c in controls]
receipt={"receipt_schema":"mycelix.evidence-attestation-universal-semantic-adapter-soundness-formal-receipt.v1",
"result":"ExecutedFail","repository":{"head":head.stdout.strip(),"tree":tree.stdout.strip()},"runtime":runtime,
"inputs":{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},"controls":IDS}

ref=run(["python3",str(args.reference)]); (args.evidence_dir/"reference.log").write_text(ref.stdout)
if ref.returncode or "CANONICAL PASS" not in ref.stdout: raise RuntimeError("reference oracle failed")
receipt["reference"]={"returncode":ref.returncode,"stdout_sha256":hashlib.sha256(ref.stdout.encode()).hexdigest()}

canonical=tlc(args.tla_jar,args.canonical_cfg,args.tla,args.evidence_dir,"tla-canonical")
if canonical: raise RuntimeError("canonical TLA violations: "+repr(sorted(canonical)))
receipt["tla"]={"canonical":"PASS","negatives":{}}
for c in controls:
    got=tlc(args.tla_jar,args.negative_cfg_dir/f"EvidenceAttestationUniversalSemanticAdapterSoundnessV1Negative-{c['id']}.cfg",args.negative_tla,args.evidence_dir,"tla-negative-"+c["id"])
    expected={c["tla_invariant"]}
    if got!=expected: raise RuntimeError("TLA isolation mismatch "+c["id"]+": "+repr(sorted(got)))
    receipt["tla"]["negatives"][c["id"]]=sorted(got)

classpath=f"{args.runner_class_dir}:{args.alloy_jar}"
ar=run(["java","-cp",classpath,"AgentDelegationAuthorityAlloyRunner",str(args.alloy)])
(args.evidence_dir/"alloy.log").write_text(ar.stdout)
if ar.returncode: raise RuntimeError("Alloy runner failed")
alloy=rows(ar.stdout)
if alloy.get("CanonicalAggregate")!="UNSAT": raise RuntimeError("canonical Alloy aggregate not UNSAT")
if alloy.get("DomainOrderReflexive")!="UNSAT": raise RuntimeError("order assertion not UNSAT")
receipt["alloy"]={"canonical":{"CanonicalAggregate":alloy.get("CanonicalAggregate"),"DomainOrderReflexive":alloy.get("DomainOrderReflexive")},"mutations":{}}
for c in controls:
    key=c["id"].replace("-","_")
    aggregate="MutantAggregate_"+key
    witness=c["alloy_witness"]
    if alloy.get(aggregate)!="SAT": raise RuntimeError("mutant aggregate not SAT "+c["id"])
    if alloy.get(witness)!="SAT": raise RuntimeError("mutant witness not SAT "+c["id"])
    receipt["alloy"]["mutations"][c["id"]]={"aggregate":"SAT","witness":"SAT"}
receipt["result"]="ExecutedPass"
(args.evidence_dir/"evidence-attestation-universal-semantic-adapter-soundness-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n")
print(json.dumps(receipt,sort_keys=True))
