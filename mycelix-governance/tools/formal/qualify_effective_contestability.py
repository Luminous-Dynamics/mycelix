#!/usr/bin/env python3
"""Candidate formal classifier for effective contestability v1."""
from __future__ import annotations
import argparse, hashlib, json, re, subprocess, sys
from pathlib import Path

TLA_OK = "Model checking completed. No error has been found."
TLA_INVARIANTS = ["TypeOK","EffectiveExitRequiresNominalExit","EffectiveExitContextIsFresh","EffectiveExitRequiresPortability","SharedRootsCannotBecomeEffectiveExit","ProviderFailureDoesNotExpandAuthority","MigrationPreservesObligationsAndHistory","ProviderSwitchDoesNotTransferAuthority","ProviderSwitchDoesNotTransferJurisdiction","HighSwitchingCostTriggersReview","NoEffectiveExitWithoutNominalExit"];
TLA_CONTROLS = {"switch-stale-exit":"EffectiveExitContextIsFresh","missing-nominal":"NoEffectiveExitWithoutNominalExit","nonportable-effective":"EffectiveExitRequiresPortability","shared-root":"SharedRootsCannotBecomeEffectiveExit","failure-authority":"ProviderFailureDoesNotExpandAuthority","switch-obligation":"MigrationPreservesObligationsAndHistory","switch-authority":"ProviderSwitchDoesNotTransferAuthority","switch-jurisdiction":"ProviderSwitchDoesNotTransferJurisdiction","switch-review":"HighSwitchingCostTriggersReview"};
ALLOY_SAT = ["NominalExitWithoutEffective","EffectiveIndependentAlternative","MigrationContinuityWitness","ProviderFailureNoAuthorityChangeWitness","HighSwitchingCostReviewWitness"];
ALLOY_UNSAT = ["EffectiveAlternativesHaveIndependentRoots","MigrationsPreserveObligationsHistoryAuthorityAndJurisdiction","ProviderFailuresPreserveAuthority","ReviewIsRequiredAtHighSwitchingCost"];
ALLOY_NEGATIVE_FACTS = {"EffectiveExitRequiresNominal":"EffectiveExitWithoutNominalWitness","EffectiveExitRequiresPortability":"NonPortableEffectiveExitWitness","EffectiveExitExcludesCurrentProvider":"CurrentProviderAlsoEffectiveWitness","EffectiveExitRequiresIndependentRoots":"SharedControlRootEffectiveWitness","MigrationPreservesContinuity":"MigrationContinuityBreakWitness","MigrationPreservesAuthority":"MigrationAuthorityTransferWitness","MigrationPreservesJurisdiction":"MigrationJurisdictionTransferWitness","ProviderFailureDoesNotTransferAuthority":"ProviderFailureAuthorityTransferWitness","ReviewAtThreshold":"HighSwitchingCostWithoutReviewWitness"};
ALLOY_UNSAT_WITNESSES = ["EffectiveExitWithoutNominalWitness","NonPortableEffectiveExitWitness","SharedControlRootEffectiveWitness","CurrentProviderAlsoEffectiveWitness","MigrationContinuityBreakWitness","MigrationAuthorityTransferWitness","MigrationJurisdictionTransferWitness","ProviderFailureAuthorityTransferWitness","HighSwitchingCostWithoutReviewWitness"];

def fail(message: str) -> "NoReturn": raise RuntimeError("CONTESTABILITY_FORMAL_FAIL: " + message)
def sha256_file(path: Path) -> str:
    h=hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda:f.read(1024*1024),b""): h.update(chunk)
    return h.hexdigest()
def git_blob_sha1(data: bytes) -> str: return hashlib.sha1(f"blob {len(data)}\0".encode()+data).hexdigest()
def load_json(path: Path): 
    raw=path.read_bytes(); return json.loads(raw.decode("utf-8")), raw
def run(cmd): return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
def tla_violations(output): return set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.",output))
def alloy_rows(output):
    return [json.loads(line) for line in output.splitlines() if line.startswith("{") and '"label"' in line and '"actual"' in line]
def remove_named_fact(source,name):
    marker=f"fact {name} {{"
    start=source.find(marker)
    if start<0: fail(f"Alloy fact not found: {name}")
    brace=source.find("{",start); depth=0
    for i in range(brace,len(source)):
        if source[i]=="{": depth+=1
        elif source[i]=="}":
            depth-=1
            if depth==0: return source[:start]+source[i+1:]
    fail(f"unterminated Alloy fact: {name}")

def main() -> int:
    p=argparse.ArgumentParser()
    for n in ("profile","pins","control-matrix","tla","cfg","negative-tla","alloy","alloy-runner-java","alloy-runner-class","alloy-runner-class-dir","tla-jar","alloy-jar","evidence-dir","workflow","crosswalk","reference-explorer","runtime-metadata"):
        p.add_argument("--"+n,type=Path,required=True)
    a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True)
    profile,pb=load_json(a.profile); pins,_=load_json(a.pins); matrix,mb=load_json(a.control_matrix); crosswalk,cwb=load_json(a.crosswalk); runtime,rmb=load_json(a.runtime_metadata)

    subject=profile.get("subject",{})
    if profile.get("authority")!="non-authoritative" or profile.get("status")!="candidate-verification-profile": fail("profile authority/status mismatch")
    if not re.fullmatch(r"[0-9a-f]{40}",str(subject.get("head_sha",""))): fail("subject head is not exact")
    if not isinstance(subject.get("source_branch"),str) or not subject.get("source_branch"): fail("subject source branch is missing")
    tree=subject.get("tree_sha")
    if not re.fullmatch(r"[0-9a-f]{40}",str(tree or "")):
        receipt={"receipt_schema":"effective-contestability-formal-receipt-v1","result":"BlockedMissingExactSubjectTree","subject_head":subject.get("head_sha"),"subject_tree":tree,"failure":"exact subject tree binding is required and unresolved","nonclaims":profile.get("nonclaims",[])}
        (a.evidence_dir/"effective-contestability-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        print(json.dumps(receipt,sort_keys=True)); return 2

    if pins.get("schema")!="mycelix.effective-contestability-formal-tool-pins.v1": fail("pin schema mismatch")
    if matrix.get("schema")!="mycelix.effective-contestability-control-matrix.v1": fail("control matrix schema mismatch")
    matrix_ids={x.get("id") for x in matrix.get("controls",[])}
    if matrix_ids != set(TLA_CONTROLS): fail("control matrix coverage mismatch")
    if any(x.get("tla_invariant") not in TLA_INVARIANTS or x.get("tla_control") not in TLA_CONTROLS or x.get("alloy_witness") not in ALLOY_UNSAT_WITNESSES for x in matrix.get("controls",[])): fail("control matrix contains unknown target")
    if profile["verifier"].get("control_matrix_path")!=a.control_matrix.as_posix(): fail("control matrix path mismatch")
    if git_blob_sha1(a.control_matrix.read_bytes())!=profile["verifier"]["control_matrix_git_blob_sha"]: fail("control matrix blob mismatch")
    head=subject["head_sha"]; branch_name=subject["source_branch"]
    fetched=run(["git","fetch","--no-tags","--depth","1","origin",head])
    if fetched.returncode!=0: fail("unable to fetch exact subject commit")
    remote=run(["git","ls-remote","origin",f"refs/heads/{branch_name}"])
    remote_head=remote.stdout.split()[0] if remote.returncode==0 and remote.stdout.strip() else ""
    if remote_head!=head: fail("subject branch does not point at frozen head")
    actual_tree=run(["git","rev-parse",f"{head}^{{tree}}"])
    if actual_tree.returncode!=0 or actual_tree.stdout.strip()!=tree: fail("frozen subject tree does not match commit")
    if crosswalk.get("subject_head")!=head or crosswalk.get("subject_tree")!=tree: fail("crosswalk subject binding mismatch")
    expected_paths=profile["verifier"]["fixture_paths"]
    actual_paths={
        "tla":a.tla.as_posix(),"cfg":a.cfg.as_posix(),"negative_tla":a.negative_tla.as_posix(),
        "alloy":a.alloy.as_posix(),"reference":a.reference_explorer.as_posix(),
        "alloy_runner":a.alloy_runner_java.as_posix(),
    }
    if actual_paths != expected_paths: fail("detached fixture paths do not match frozen profile")
    if runtime.get("schema")!="mycelix.effective-contestability-formal-runtime.v1": fail("runtime schema mismatch")
    if runtime.get("nixpkgs_rev")!=pins["nixpkgs_rev"] or runtime.get("jdk_package")!="jdk17_headless" or runtime.get("java_major")!=17: fail("runtime pin mismatch")
    for path,expected in [
        (a.tla,profile["models"]["tla"]["git_blob_sha"]),
        (a.cfg,profile["models"]["tla"]["config_git_blob_sha"]),
        (a.negative_tla,profile["models"]["negative_tla"]["git_blob_sha"]),
        (a.alloy,profile["models"]["alloy"]["git_blob_sha"]),
        (a.reference_explorer,profile["models"]["reference"]["git_blob_sha"]),
        (a.alloy_runner_java,profile["models"]["alloy"]["runner_git_blob_sha"]),
        (a.crosswalk,profile["verifier"]["crosswalk_git_blob_sha"])]:
        if git_blob_sha1(path.read_bytes())!=expected: fail("frozen byte mismatch: "+str(path))
    if sha256_file(a.tla_jar)!=pins["tools"]["tla2tools"]["sha256"]: fail("TLA+ tool hash mismatch")
    if sha256_file(a.alloy_jar)!=pins["tools"]["alloy"]["sha256"]: fail("Alloy tool hash mismatch")
    baseline={str(x):sha256_file(x) for x in (a.profile,a.pins,a.control_matrix,a.tla,a.cfg,a.negative_tla,a.alloy,a.alloy_runner_java,a.crosswalk,a.reference_explorer,a.runtime_metadata,a.workflow)}
    if a.workflow.as_posix()!=".github/workflows/sovereignty-contestability-formal-candidate.yml": fail("workflow path mismatch")
    receipt={"receipt_schema":"effective-contestability-formal-receipt-v1","result":"ExecutedFail","claim_scope":"candidate bounded formal qualification only","subject":subject,
             "verifier":{"profile_sha256":hashlib.sha256(pb).hexdigest(),"crosswalk_sha256":hashlib.sha256(cwb).hexdigest(),"implementation_sha256":sha256_file(Path(__file__)),"alloy_runner_java_sha256":sha256_file(a.alloy_runner_java),"alloy_runner_class_sha256":sha256_file(a.alloy_runner_class),"runtime_metadata_sha256":hashlib.sha256(rmb).hexdigest()},
             "runtime":runtime,"tools":pins["tools"],"tla":{"canonical":{},"negative_controls":{}},"alloy":{"canonical":{},"negative_controls":{}},"reference":{},"nonclaims":profile.get("nonclaims",[])}
    try:
        ref=run([sys.executable,str(a.reference_explorer)]); (a.evidence_dir/"reference.log").write_text(ref.stdout,encoding="utf-8")
        receipt["reference"]={"returncode":ref.returncode,"log_sha256":sha256_file(a.evidence_dir/"reference.log")}
        if ref.returncode!=0 or "CANONICAL PASS: no invariant violation through depth 4" not in ref.stdout: fail("reference explorer canonical result missing")
        for c,t in TLA_CONTROLS.items():
            if f"NEGATIVE PASS: {c} -> {t} counterexample" not in ref.stdout: fail("reference control missing: "+c)

        tla_cmd=["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-config",str(a.cfg),str(a.tla)]
        tla=run(tla_cmd); (a.evidence_dir/"tla-canonical.log").write_text(tla.stdout,encoding="utf-8")
        if tla.returncode!=0 or TLA_OK not in tla.stdout or tla_violations(tla.stdout): fail("canonical TLA result invalid")
        receipt["tla"]["canonical"]={"command":tla_cmd,"returncode":tla.returncode,"log_sha256":sha256_file(a.evidence_dir/"tla-canonical.log")}

        negative_source=a.negative_tla.read_text(encoding="utf-8"); canonical=a.tla.read_text(encoding="utf-8")
        for c,t in TLA_CONTROLS.items():
            work=a.evidence_dir/"tla-negative"/c; work.mkdir(parents=True,exist_ok=True)
            (work/a.negative_tla.name).write_text(negative_source,encoding="utf-8")
            (work/a.tla.name).write_text(canonical,encoding="utf-8")
            c2="C1" if c=="shared-root" else "C2"
            cfg="CONSTANTS\n"+f"S1 = S1\nS2 = S2\nP1 = P1\nP2 = P2\nC1 = C1\nC2 = {c2}\nI1 = I1\nI2 = I2\nE1 = E1\nE2 = E2\nV1 = V1\nV2 = V2\nM1 = M1\nM2 = M2\nPowerA = PowerA\nPowerB = PowerB\nJ1 = J1\nJ2 = J2\nMaxSwitchingCost = 4\nReviewThreshold = 2\nControl = \\"{c}\\"\n\nINIT Init\nNEXT NegativeNext\nINVARIANTS\n"+"\n".join(TLA_INVARIANTS)+"\nCHECK_DEADLOCK FALSE\n"
            cfgp=work/"negative.cfg"; cfgp.write_text(cfg,encoding="utf-8")
            cmd=["java","-cp",str(a.tla_jar),"tlc2.TLC","-workers","1","-config",str(cfgp),str(work/a.negative_tla.name)]
            res=run(cmd); violations=tla_violations(res.stdout)
            if res.returncode==0 or violations!={t}: fail(f"TLA negative control not isolated: {c} -> {sorted(violations)}")
            receipt["tla"]["negative_controls"][c]={"target_invariant":t,"violated_invariants":sorted(violations),"returncode":res.returncode,"log_sha256":hashlib.sha256(res.stdout.encode()).hexdigest()}

        cp=f"{a.alloy_runner_class_dir}:{a.alloy_jar}"; cmd=["java","-cp",cp,"EffectiveContestabilityAlloyRunner",str(a.alloy)]
        res=run(cmd); (a.evidence_dir/"alloy.log").write_text(res.stdout,encoding="utf-8")
        if res.returncode!=0: fail("Alloy runner failed")
        rows=alloy_rows(res.stdout); by={r["label"]:r for r in rows}
        if set(by)!=set(ALLOY_SAT)|set(ALLOY_UNSAT): fail("Alloy label set mismatch")
        scope=profile["models"]["alloy"]["scope"]
        for r in rows:
            if not re.fullmatch(r"[0-9a-f]{64}",str(r.get("solution_sha256",""))): fail("Alloy solution digest missing")
            command=r["command"]
            for name,count in scope.items():
                token=f"{count} {name}" if name!="int" else f"{count} int"
                if name!="overall" and token not in command: fail("Alloy exact scope missing: "+token+" -> "+r["label"])
            if f"for {scope['overall']}" not in command: fail("Alloy overall scope missing: "+r["label"])
        for label in ALLOY_SAT:
            if by[label]["actual"]!="SAT" or by[label]["check"] or by[label]["expects"]!=1: fail("Alloy SAT mismatch: "+label)
        for label in ALLOY_UNSAT:
            if by[label]["actual"]!="UNSAT" or not by[label]["check"] or by[label]["expects"]!=0: fail("Alloy UNSAT mismatch: "+label)
        for label in ALLOY_UNSAT_WITNESSES:
            if label not in by or by[label]["actual"]!="UNSAT" or by[label]["check"]: fail("Alloy negative witness not UNSAT canonically: "+label)
        canonical_alloy={r["label"]:r["actual"] for r in rows}; receipt["alloy"]["canonical"]={"commands":rows}
        source=a.alloy.read_text(encoding="utf-8")
        for fact,target in ALLOY_NEGATIVE_FACTS.items():
            work=a.evidence_dir/"alloy-negative"/fact; work.mkdir(parents=True,exist_ok=True)
            mutated=work/a.alloy.name; mutated.write_text(remove_named_fact(source,fact),encoding="utf-8")
            res=run(["java","-cp",cp,"EffectiveContestabilityAlloyRunner",str(mutated)]); rows=alloy_rows(res.stdout); by={r["label"]:r for r in rows}
            if res.returncode!=0 or by.get(target,{}).get("actual")!="SAT": fail("Alloy negative failed: "+fact)
            if set(by)!=set(canonical_alloy): fail("Alloy negative label set changed: "+fact)
            for label,actual in canonical_alloy.items():
                if label!=target and by[label]["actual"]!=actual: fail("Alloy negative changed unrelated outcome: "+fact+" -> "+label)

        if any(sha256_file(Path(p))!=d for p,d in baseline.items()): fail("qualification input changed")
        if subprocess.run(["git","status","--porcelain"],stdout=subprocess.PIPE,text=True).stdout.strip(): fail("repository state changed")
        receipt["result"]="QualifiedExactHead"; receipt["postflight"]={"inputs_unchanged":True,"git_status_clean":True}
    except Exception as exc:
        receipt["failure"]=str(exc)
        (a.evidence_dir/"effective-contestability-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        print(json.dumps(receipt,sort_keys=True)); return 1
    (a.evidence_dir/"effective-contestability-formal-receipt-v1.json").write_text(json.dumps(receipt,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    print(json.dumps(receipt,sort_keys=True)); return 0

if __name__=="__main__": raise SystemExit(main())
