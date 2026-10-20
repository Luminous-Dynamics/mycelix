#!/usr/bin/env python3
"""Offline coherence verifier for QREC GitHub qualification provenance.

PASS means preserved metadata/archive bytes are internally coherent with an exact
local policy. It does NOT authenticate provider retrieval, GitHub/Sigstore,
Xenia, issuer authorization, qualification truth, or runtime/provider authority.
"""
from __future__ import annotations
import argparse, copy, hashlib, importlib.util, io, json, re, shutil, stat, sys, tempfile, zipfile
from pathlib import Path
from typing import Mapping, Sequence

POLICY_SCHEMA="mycelix.qrec.github-offline-policy.v1"
RESULT_SCHEMA="mycelix.qrec.github-offline-provenance.v1"
RESULT="QREC_GITHUB_METADATA_AND_ARCHIVE_COHERENT"
AUTH_MEMBER="qualification-auth-capsule-v1.tar"
QREC_BLOB="8036bb43064fdac50349bcc71b3ee5086100aa25"
QREC_SHA256="acc7a19a7d5da3b4442a9e1e03eac8e307f3c0fb5aa8b4e70e781bd62ab07949"
MAX_JSON=4*1024*1024
MAX_ZIP=1024*1024*1024
HEX40=re.compile(r"^[0-9a-f]{40}$")
HEX64=re.compile(r"^[0-9a-f]{64}$")
PORTABLE=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")

class ProvenanceError(ValueError): pass
def fail(m): raise ProvenanceError(m)
def pos(v,n):
    if not isinstance(v,int) or isinstance(v,bool) or v<=0: fail(f"{n} must be positive integer")
    return v
def text(v,n,limit=512):
    if not isinstance(v,str) or not v or v.strip()!=v or len(v.encode())>limit: fail(f"{n} invalid")
    return v
def head(v,n):
    v=text(v,n,40)
    if HEX40.fullmatch(v) is None: fail(f"{n} must be 40 lowercase hex")
    return v
def digest(v,n):
    v=text(v,n,71)
    if not v.startswith("sha256:") or HEX64.fullmatch(v[7:]) is None: fail(f"{n} invalid")
    return v
def exact_obj(v,n,keys):
    if not isinstance(v,dict) or set(v)!=set(keys): fail(f"{n} field census mismatch")
    return v
def nodup(pairs):
    out={}
    for k,v in pairs:
        if k in out: fail(f"duplicate JSON key: {k}")
        out[k]=v
    return out
def read_regular(path,limit,label):
    try: st=path.lstat()
    except OSError as e: fail(f"cannot stat {label}: {e}")
    if not stat.S_ISREG(st.st_mode): fail(f"{label} must be ordinary non-symlink file")
    if st.st_size<0 or st.st_size>limit: fail(f"{label} too large")
    try:
        with path.open("rb") as f: data=f.read(limit+1)
    except OSError as e: fail(f"cannot read {label}: {e}")
    if len(data)>limit: fail(f"{label} too large")
    return data
def load_json(path,label):
    try: v=json.loads(read_regular(path,MAX_JSON,label).decode(),object_pairs_hook=nodup)
    except ProvenanceError: raise
    except (UnicodeDecodeError,json.JSONDecodeError) as e: fail(f"cannot parse {label}: {e}")
    if not isinstance(v,dict): fail(f"{label} root must be object")
    return v
def sha(data): return hashlib.sha256(data).hexdigest()
def git_blob(data): return hashlib.sha1(b"blob "+str(len(data)).encode()+b"\0"+data).hexdigest()

def policy(path):
    p=load_json(path,"policy")
    keys={"schema","repository","issuer_id","workflow","event","semantic_head","verifier_head","qualifier_job_name","artifact_name_prefix"}
    if set(p)!=keys or p["schema"]!=POLICY_SCHEMA: fail("policy schema/field census mismatch")
    repo=exact_obj(p["repository"],"policy.repository",{"id","full_name"})
    rid=pos(repo["id"],"policy.repository.id"); rname=text(repo["full_name"],"policy.repository.full_name",256)
    if "/" not in rname: fail("repository full_name must be owner/name")
    if text(p["issuer_id"],"policy.issuer_id",256)!=f"github-repository:{rid}": fail("issuer_id/repository mismatch")
    wf=exact_obj(p["workflow"],"policy.workflow",{"id","path"}); pos(wf["id"],"policy.workflow.id")
    wfp=text(wf["path"],"policy.workflow.path")
    if not wfp.startswith(".github/workflows/") or not wfp.endswith((".yml",".yaml")) or ".." in wfp or "\\" in wfp: fail("workflow path invalid")
    if text(p["event"],"policy.event",64) not in {"pull_request","workflow_dispatch"}: fail("event invalid")
    head(p["semantic_head"],"policy.semantic_head"); head(p["verifier_head"],"policy.verifier_head")
    text(p["qualifier_job_name"],"policy.qualifier_job_name",256)
    if PORTABLE.fullmatch(text(p["artifact_name_prefix"],"policy.artifact_name_prefix",128)) is None: fail("artifact prefix invalid")
    return p

def qrec_module(path):
    data=read_regular(path,2*1024*1024,"QREC-002A tool")
    if git_blob(data)!=QREC_BLOB or sha(data)!=QREC_SHA256: fail("QREC-002A source identity mismatch")
    spec=importlib.util.spec_from_file_location("qrec_capsule_exact",path)
    if spec is None or spec.loader is None: fail("cannot load QREC-002A")
    m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    for n in ("verify_auth_bytes","read_tar_exact","parse_receipt","AUTH_MEMBER_NAMES","RECEIPT_NAME","CapsuleError"):
        if not hasattr(m,n): fail(f"QREC-002A missing API {n}")
    return m

def verify_run(r,p):
    rid=pos(r.get("id"),"run.id"); attempt=pos(r.get("run_attempt"),"run.run_attempt"); wf=p["workflow"]
    for k,v in {"workflow_id":wf["id"],"path":wf["path"],"event":p["event"],"status":"completed","conclusion":"success","head_sha":p["verifier_head"]}.items():
        if r.get(k)!=v: fail(f"run.{k} mismatch")
    repo=p["repository"]
    for k in ("repository","head_repository"):
        a=r.get(k)
        if not isinstance(a,dict) or a.get("id")!=repo["id"] or a.get("full_name")!=repo["full_name"]: fail(f"run.{k} mismatch")
    return rid,attempt

def verify_jobs(d,rid,attempt,p):
    jobs=d.get("jobs")
    if not isinstance(jobs,list) or d.get("total_count")!=len(jobs) or len(jobs)!=1: fail("exact one-job qualifier workflow required")
    j=jobs[0]
    if not isinstance(j,dict) or j.get("name")!=p["qualifier_job_name"]: fail("qualifier job mismatch")
    jid=pos(j.get("id"),"job.id")
    if j.get("run_id")!=rid or j.get("run_attempt") not in (None,attempt): fail("job run identity mismatch")
    if j.get("status")!="completed" or j.get("conclusion")!="success": fail("qualifier job not successful")
    steps=j.get("steps")
    if not isinstance(steps,list) or not steps: fail("qualifier job has no executed steps")
    success=0
    for i,s in enumerate(steps):
        if not isinstance(s,dict) or s.get("status")!="completed" or s.get("conclusion") not in {"success","skipped"}: fail(f"job step {i} invalid")
        success+=s.get("conclusion")=="success"
    if success==0: fail("qualifier job has no successful steps")
    return jid

def verify_artifacts(d,rid,attempt,p):
    arts=d.get("artifacts")
    if d.get("total_count")!=1 or not isinstance(arts,list) or len(arts)!=1 or not isinstance(arts[0],dict): fail("exactly one QREC artifact required")
    a=arts[0]; aid=pos(a.get("id"),"artifact.id"); name=f"{p['artifact_name_prefix']}{rid}-{attempt}"
    if a.get("name")!=name or a.get("expired") is not False: fail("artifact name/expiry mismatch")
    pos(a.get("size_in_bytes"),"artifact.size_in_bytes"); dig=digest(a.get("digest"),"artifact.digest")
    wr=a.get("workflow_run"); repo=p["repository"]
    if not isinstance(wr,dict): fail("artifact.workflow_run missing")
    for k,v in {"id":rid,"repository_id":repo["id"],"head_repository_id":repo["id"],"head_sha":p["verifier_head"]}.items():
        if wr.get(k)!=v: fail(f"artifact.workflow_run.{k} mismatch")
    return aid,name,dig

def auth_from_zip(path,expected_digest):
    z=read_regular(path,MAX_ZIP,"artifact ZIP")
    if "sha256:"+sha(z)!=expected_digest: fail("artifact ZIP digest mismatch")
    try:
        with zipfile.ZipFile(io.BytesIO(z),"r") as a:
            infos=a.infolist()
            if len(infos)!=1: fail("artifact ZIP member census mismatch")
            i=infos[0]
            if i.filename!=AUTH_MEMBER or i.is_dir() or i.filename.startswith("/") or ".." in i.filename.split("/"): fail("artifact ZIP member invalid")
            if ((i.external_attr>>16)&0o170000)==stat.S_IFLNK or i.flag_bits&1: fail("artifact ZIP symlink/encryption forbidden")
            if i.file_size<=0 or i.file_size>MAX_ZIP: fail("A size invalid")
            body=a.read(i)
            if len(body)!=i.file_size: fail("A ZIP size mismatch")
            return body
    except ProvenanceError: raise
    except (zipfile.BadZipFile,RuntimeError,OSError) as e: fail(f"artifact ZIP invalid: {e}")

def verify_receipt(r,p,rid,attempt):
    for k,v in {"issuer_id":p["issuer_id"],"semantic_head":p["semantic_head"],"verifier_head":p["verifier_head"],"run_id":rid,"run_attempt":attempt}.items():
        if r.get(k)!=v: fail(f"QREC receipt {k} mismatch")

def verify(policy_path,run_path,jobs_path,arts_path,zip_path,qrec_path):
    p=policy(policy_path); run=load_json(run_path,"run JSON"); jobs=load_json(jobs_path,"jobs JSON"); arts=load_json(arts_path,"artifacts JSON"); q=qrec_module(qrec_path)
    rid,attempt=verify_run(run,p); jid=verify_jobs(jobs,rid,attempt,p); aid,aname,adig=verify_artifacts(arts,rid,attempt,p)
    A=auth_from_zip(zip_path,adig)
    try:
        asum=q.verify_auth_bytes(A); members=dict(q.read_tar_exact(A,expected_names=q.AUTH_MEMBER_NAMES)); r=q.parse_receipt(members[q.RECEIPT_NAME])
    except q.CapsuleError as e: fail(f"QREC-002A capsule verification failed: {e}")
    verify_receipt(r,p,rid,attempt)
    return {
      "schema":RESULT_SCHEMA,"result":RESULT,"repository":p["repository"],"workflow":p["workflow"],
      "semantic_head":p["semantic_head"],"verifier_head":p["verifier_head"],"github_run_id":rid,"github_run_attempt":attempt,
      "qualifier_job_id":jid,"artifact_id":aid,"artifact_name":aname,"artifact_digest":adig,
      "authentication_capsule_sha256":asum["sha256"],"receipt_commitment":asum["receipt_commitment"],
      "provider_metadata_snapshot_structurally_verified":True,"qualifier_job_execution_observed":True,
      "archive_digest_matches_metadata":True,"authentication_capsule_structurally_verified":True,
      "provider_metadata_retrieval_authenticated":False,"provider_artifact_retrievability_authenticated":False,
      "github_sigstore_attestation_verified":False,"xenia_attestation_verified":False,"issuer_authorization_verified":False,
      "verified_receipt_minted":False,"hosted_qualification_authenticity_established":False,"provider_runtime_authority_granted":False,
    }

def write_json(path,v): path.write_text(json.dumps(v,indent=2,sort_keys=True)+"\n")
def fixture(root,q):
    repo_id=1176351975; repo="Luminous-Dynamics/mycelix"; wfid=777001; wf=".github/workflows/example-qrec-qualifier.yml"; sem="1"*40; ver="2"*40
    rid=424242; attempt=3; jid=515151; aid=989898; prefix="qrec-evidence-"
    p={"schema":POLICY_SCHEMA,"repository":{"id":repo_id,"full_name":repo},"issuer_id":f"github-repository:{repo_id}","workflow":{"id":wfid,"path":wf},"event":"pull_request","semantic_head":sem,"verifier_head":ver,"qualifier_job_name":"Exact qualifier","artifact_name_prefix":prefix}
    E=q.pack_evidence_bytes([("logs/result.txt",b"PASS\n"),("source/head.txt",sem.encode()+b"\n")])
    R=q.canonical_json_bytes({"schema_version":1,"dependency_id":"fixture:qrec-002b","issuer_id":f"github-repository:{repo_id}","semantic_head":sem,"verifier_head":ver,"run_id":rid,"run_attempt":attempt,"artifact_digest":q.sha256_tagged(E),"receipt_commitment":"blake3-256:"+"3"*64,"navigation":{"job_id":jid,"artifact_id":aid}})
    A=q.pack_auth_bytes(E,R); zb=io.BytesIO()
    with zipfile.ZipFile(zb,"w",compression=zipfile.ZIP_DEFLATED) as z:
        i=zipfile.ZipInfo(AUTH_MEMBER); i.external_attr=(0o100644<<16); z.writestr(i,A)
    zbytes=zb.getvalue()
    run={"id":rid,"run_attempt":attempt,"workflow_id":wfid,"path":wf,"event":"pull_request","status":"completed","conclusion":"success","head_sha":ver,"repository":{"id":repo_id,"full_name":repo},"head_repository":{"id":repo_id,"full_name":repo}}
    jobs={"total_count":1,"jobs":[{"id":jid,"name":"Exact qualifier","run_id":rid,"run_attempt":attempt,"status":"completed","conclusion":"success","steps":[{"name":"Checkout","status":"completed","conclusion":"success"},{"name":"Qualify","status":"completed","conclusion":"success"}]}]}
    arts={"total_count":1,"artifacts":[{"id":aid,"name":f"{prefix}{rid}-{attempt}","size_in_bytes":len(zbytes),"expired":False,"digest":"sha256:"+sha(zbytes),"workflow_run":{"id":rid,"repository_id":repo_id,"head_repository_id":repo_id,"head_sha":ver}}]}
    paths=[root/"policy.json",root/"run.json",root/"jobs.json",root/"artifacts.json",root/"artifact.zip"]
    for path,value in zip(paths[:4],[p,run,jobs,arts]): write_json(path,value)
    paths[4].write_bytes(zbytes); return tuple(paths)

def mutate(path,fn):
    d=json.loads(path.read_text()); fn(d); write_json(path,d)
def reject(base,qpath,label,fn):
    c=base.parent/f"case-{label}"; shutil.copytree(base,c); fn(c)
    try: verify(c/"policy.json",c/"run.json",c/"jobs.json",c/"artifacts.json",c/"artifact.zip",qpath)
    except ProvenanceError: return
    fail("self-test accepted "+label)

def rebuild_zip(case,q,receipt_change=None,auth_mutate=False,extra=False,symlink=False):
    zp=case/"artifact.zip"
    with zipfile.ZipFile(zp,"r") as z: A=z.read(AUTH_MEMBER)
    if receipt_change:
        parts=dict(q.read_tar_exact(A,expected_names=q.AUTH_MEMBER_NAMES)); r=json.loads(parts[q.RECEIPT_NAME]); receipt_change(r)
        A=q.pack_auth_bytes(parts[q.EVIDENCE_NAME],q.canonical_json_bytes(r))
    if auth_mutate:
        b=bytearray(A); b[0]^=1; A=bytes(b)
    with zipfile.ZipFile(zp,"w",compression=zipfile.ZIP_DEFLATED) as z:
        i=zipfile.ZipInfo(AUTH_MEMBER); i.create_system=3; i.external_attr=((0o120777 if symlink else 0o100644)<<16); z.writestr(i,A)
        if extra: z.writestr("unexpected.txt",b"x")
    mutate(case/"artifacts.json",lambda d:d["artifacts"][0].__setitem__("digest","sha256:"+sha(zp.read_bytes())))

def self_test(qpath):
    q=qrec_module(qpath)
    with tempfile.TemporaryDirectory() as td:
        root=Path(td); base=root/"valid"; base.mkdir(); paths=fixture(base,q)
        out=verify(*paths,qpath); assert out["result"]==RESULT and out["provider_metadata_retrieval_authenticated"] is False and out["verified_receipt_minted"] is False
        reject(base,qpath,"repo",lambda c:mutate(c/"run.json",lambda d:d["repository"].__setitem__("id",1)))
        reject(base,qpath,"workflow",lambda c:mutate(c/"run.json",lambda d:d.__setitem__("workflow_id",1)))
        reject(base,qpath,"verifier",lambda c:mutate(c/"run.json",lambda d:d.__setitem__("head_sha","f"*40)))
        reject(base,qpath,"failed",lambda c:mutate(c/"run.json",lambda d:d.__setitem__("conclusion","failure")))
        reject(base,qpath,"empty-steps",lambda c:mutate(c/"jobs.json",lambda d:d["jobs"][0].__setitem__("steps",[])))
        reject(base,qpath,"dup-job",lambda c:mutate(c/"jobs.json",lambda d:(d["jobs"].append(copy.deepcopy(d["jobs"][0])),d.__setitem__("total_count",2))))
        reject(base,qpath,"attempt",lambda c:mutate(c/"run.json",lambda d:d.__setitem__("run_attempt",4)))
        reject(base,qpath,"expired",lambda c:mutate(c/"artifacts.json",lambda d:d["artifacts"][0].__setitem__("expired",True)))
        reject(base,qpath,"artifact-head",lambda c:mutate(c/"artifacts.json",lambda d:d["artifacts"][0]["workflow_run"].__setitem__("head_sha","f"*40)))
        reject(base,qpath,"zip-digest",lambda c:(c/"artifact.zip").write_bytes((c/"artifact.zip").read_bytes()+b"x"))
        reject(base,qpath,"A-mutation",lambda c:rebuild_zip(c,q,auth_mutate=True))
        reject(base,qpath,"R-attempt",lambda c:rebuild_zip(c,q,receipt_change=lambda r:r.__setitem__("run_attempt",r["run_attempt"]+1)))
        reject(base,qpath,"R-issuer",lambda c:rebuild_zip(c,q,receipt_change=lambda r:r.__setitem__("issuer_id","github-repository:1")))
        reject(base,qpath,"R-semantic",lambda c:rebuild_zip(c,q,receipt_change=lambda r:r.__setitem__("semantic_head","f"*40)))
        reject(base,qpath,"zip-extra",lambda c:rebuild_zip(c,q,extra=True))
        reject(base,qpath,"zip-symlink",lambda c:rebuild_zip(c,q,symlink=True))
        bad=root/"bad-qrec.py"; bad.write_bytes(qpath.read_bytes()+b"\n#modified\n")
        try: verify(*paths,bad)
        except ProvenanceError: pass
        else: raise AssertionError("modified QREC-002A tool accepted")
    print("QREC-002B offline GitHub provenance self-test: PASS")

def main(argv:Sequence[str]|None=None):
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("--qrec-tool",type=Path,required=True); p.add_argument("--self-test",action="store_true"); p.add_argument("--json-output",type=Path)
    for n in ("policy","run_json","jobs_json","artifacts_json","artifact_zip"): p.add_argument(n,type=Path,nargs="?")
    a=p.parse_args(argv)
    try:
        if a.self_test: self_test(a.qrec_tool); return 0
        vals=[a.policy,a.run_json,a.jobs_json,a.artifacts_json,a.artifact_zip]
        if any(v is None for v in vals): fail("five positional evidence inputs required")
        out=verify(*vals,a.qrec_tool); t=json.dumps(out,indent=2,sort_keys=True)+"\n"
        if a.json_output: a.json_output.write_text(t)
        else: print(t,end="")
        return 0
    except ProvenanceError as e:
        print(f"QREC GitHub provenance verification failed: {e}",file=sys.stderr); return 2
if __name__=="__main__": raise SystemExit(main())
