#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json,os,re,shutil,subprocess,sys,tempfile
from pathlib import Path,PurePosixPath
CD=b"MYCELIX_QUALIFICATION_CAPSULE_V1\0"; RD=b"MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V1\0"
H40=re.compile(r"^[0-9a-f]{40}$"); H64=re.compile(r"^[0-9a-f]{64}$"); ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")
TOP={"capsule_format_revision","theorem_id","theorem_revision","repository_identity","product_subject_sha","predecessor_sha","expected_changed_paths","expected_object_blobs","toolchain_profile_ref","environment_profile_ref","gates","verdict","claim","nonclaims"}
CTX={"execution_context_format_revision","runner_profile_ref","toolchain_profile_ref","environment_profile_ref","resolved_toolchain_commitment","resolved_environment_commitment"}
class E(Exception):pass
def cj(v):
 def e(x):
  if x is None:return"null"
  if x is True:return"true"
  if x is False:return"false"
  if isinstance(x,int)and not isinstance(x,bool):return str(x)
  if isinstance(x,float):raise E("floats forbidden")
  if isinstance(x,str):return json.dumps(x,ensure_ascii=False,separators=(",",":"))
  if isinstance(x,list):return"["+",".join(e(i)for i in x)+"]"
  if isinstance(x,dict):return"{"+",".join(json.dumps(k,ensure_ascii=False)+":"+e(x[k])for k in sorted(x,key=lambda k:k.encode()))+"}"
  raise E("unsupported type")
 return e(v).encode()
def com(d,v):b=cj(v);return hashlib.sha256(d+len(b).to_bytes(8,"big")+b).hexdigest()
def cc(m):return com(CD,m)
def rc(r):return com(RD,r)
def keys(v,k,w):
 if not isinstance(v,dict)or set(v)!=k:raise E(w+" keys mismatch")
def hx(v,p,w):
 if not isinstance(v,str)or not p.fullmatch(v):raise E(w+" invalid")
def prof(v,w):
 keys(v,{"id","revision","digest"},w); r=v["revision"]
 if not isinstance(v["id"],str)or not ID.fullmatch(v["id"]):raise E(w+".id invalid")
 if not isinstance(r,int)or isinstance(r,bool)or r<1:raise E(w+".revision invalid")
 hx(v["digest"],H64,w+".digest")
def rel(v,w):
 if not isinstance(v,str)or not v:raise E(w+" invalid")
 p=PurePosixPath(v)
 if p.is_absolute()or".."in p.parts or"\\"in v or str(p)!=v:raise E(w+" unsafe")
def su(v,w):
 if not isinstance(v,list)or not all(isinstance(x,str)for x in v)or v!=sorted(set(v),key=lambda x:x.encode()):raise E(w+" not sorted unique")
def vm(m,root=None):
 keys(m,TOP,"manifest")
 if m["capsule_format_revision"]!=1:raise E("unsupported capsule format")
 if not isinstance(m["theorem_id"],str)or not ID.fullmatch(m["theorem_id"]):raise E("theorem id invalid")
 if not isinstance(m["theorem_revision"],int)or isinstance(m["theorem_revision"],bool)or m["theorem_revision"]<1:raise E("theorem revision invalid")
 if not isinstance(m["repository_identity"],str)or"/"not in m["repository_identity"]:raise E("repository identity invalid")
 hx(m["product_subject_sha"],H40,"subject");hx(m["predecessor_sha"],H40,"predecessor")
 if m["product_subject_sha"]==m["predecessor_sha"]:raise E("subject equals predecessor")
 su(m["expected_changed_paths"],"changed paths");[rel(x,"changed path")for x in m["expected_changed_paths"]]
 if not isinstance(m["expected_object_blobs"],dict)or not m["expected_object_blobs"]:raise E("blob map invalid")
 for p,d in m["expected_object_blobs"].items():rel(p,"blob path");hx(d,H40,"blob sha")
 prof(m["toolchain_profile_ref"],"toolchain profile");prof(m["environment_profile_ref"],"environment profile")
 if not isinstance(m["gates"],list)or not m["gates"]:raise E("gates empty")
 ids=[];scripts=[]
 for g in m["gates"]:
  keys(g,{"id","class","script","sha256","args","timeout_seconds"},"gate")
  if not isinstance(g["id"],str)or not ID.fullmatch(g["id"]):raise E("gate id invalid")
  if g["class"]not in{"theorem","oracle","lineage"}:raise E("gate class invalid")
  rel(g["script"],"gate script");hx(g["sha256"],H64,"gate sha256")
  if not isinstance(g["args"],list)or not all(isinstance(a,str)for a in g["args"]):raise E("gate args invalid")
  t=g["timeout_seconds"]
  if not isinstance(t,int)or isinstance(t,bool)or not 1<=t<=7200:raise E("gate timeout invalid")
  ids.append(g["id"]);scripts.append((g["script"],g["sha256"]))
 if ids!=sorted(set(ids),key=lambda x:x.encode()):raise E("gates not sorted unique")
 keys(m["verdict"],{"kind","required_gate_ids"},"verdict");su(m["verdict"]["required_gate_ids"],"required gates")
 if m["verdict"]["kind"]!="all"or m["verdict"]["required_gate_ids"]!=ids:raise E("verdict invalid")
 if not isinstance(m["claim"],str)or not m["claim"].strip():raise E("claim empty")
 su(m["nonclaims"],"nonclaims");cj(m)
 if root:
  root=Path(root).resolve()
  for p,d in scripts:
   f=(root/p).resolve()
   try:f.relative_to(root)
   except ValueError:raise E("script escapes root")
   if not f.is_file()or hashlib.sha256(f.read_bytes()).hexdigest()!=d:raise E("script digest mismatch "+p)
def vc(c,m):
 keys(c,CTX,"execution context")
 if c["execution_context_format_revision"]!=1:raise E("unsupported execution context")
 prof(c["runner_profile_ref"],"runner profile");prof(c["toolchain_profile_ref"],"execution toolchain");prof(c["environment_profile_ref"],"execution environment")
 if c["toolchain_profile_ref"]!=m["toolchain_profile_ref"]:raise E("toolchain profile mismatch")
 if c["environment_profile_ref"]!=m["environment_profile_ref"]:raise E("environment profile mismatch")
 hx(c["resolved_toolchain_commitment"],H64,"resolved toolchain");hx(c["resolved_environment_commitment"],H64,"resolved environment");cj(c)
def git(r,*a,check=True):
 try:return subprocess.run(["git","-C",str(r),*a],check=check,capture_output=True,text=True)
 except(OSError,subprocess.CalledProcessError)as x:raise E("git "+" ".join(a)+" failed")from x
def pre(m,r,rid):
 if rid!=m["repository_identity"]:raise E("repository identity mismatch")
 if git(r,"cat-file","-e",m["product_subject_sha"]+"^{commit}",check=False).returncode:raise E("subject unavailable")
def state(w):return git(w,"rev-parse","HEAD").stdout.strip(),git(w,"status","--porcelain=v1","--untracked-files=all").stdout.strip()
def addwt(r,s):
 p=Path(tempfile.mkdtemp(prefix="qcap-"));w=p/"subject"
 if git(r,"worktree","add","--detach","--force",str(w),s,check=False).returncode:shutil.rmtree(p,ignore_errors=True);raise E("worktree materialization failed")
 return p,w
def rmwt(r,p,w):
 x=git(r,"worktree","remove","--force",str(w),check=False);shutil.rmtree(p,ignore_errors=True)
 if x.returncode:raise E("worktree cleanup failed")
def gs(c):return"GatePass"if c==0 else("GateFail"if c==10 else"RunnerInfrastructureFailure")
def nr(x):
 keys(x,{"id","status","output_sha256","exit_code"},"gate result");hx(x["output_sha256"],H64,"output digest")
 if not isinstance(x["exit_code"],int)or isinstance(x["exit_code"],bool)or x["status"]!=gs(x["exit_code"]):raise E("gate status/exit mismatch")
def verdict(rs):
 s={x["status"]for x in rs}
 return"CompletedConjunctivePass"if s=={"GatePass"}else("RunnerInfrastructureFailure"if"RunnerInfrastructureFailure"in s else"CompletedConjunctiveFail")
def receipt(m,a,c,rs):
 if not isinstance(a,str)or not ID.fullmatch(a):raise E("attempt id invalid")
 vc(c,m);ids=[g["id"]for g in m["gates"]]
 if[r.get("id")for r in rs]!=ids:raise E("gate order mismatch")
 [nr(r)for r in rs]
 o={"receipt_format_revision":1,"capsule_commitment":cc(m),"theorem_id":m["theorem_id"],"theorem_revision":m["theorem_revision"],"repository_identity":m["repository_identity"],"product_subject_sha":m["product_subject_sha"],"attempt_id":a,"execution_context":c,"gate_results":rs,"verdict":verdict(rs),"claim":m["claim"],"nonclaims":m["nonclaims"]};o["receipt_commitment"]=rc(o);return o
def vr(r,m):
 k={"receipt_format_revision","capsule_commitment","theorem_id","theorem_revision","repository_identity","product_subject_sha","attempt_id","execution_context","gate_results","verdict","claim","nonclaims","receipt_commitment"};keys(r,k,"receipt")
 if r["receipt_format_revision"]!=1:raise E("unsupported receipt")
 z=dict(r);sup=z.pop("receipt_commitment");hx(sup,H64,"receipt commitment")
 if rc(z)!=sup:raise E("receipt commitment mismatch")
 for a,b,n in[(r["capsule_commitment"],cc(m),"capsule"),(r["theorem_id"],m["theorem_id"],"theorem"),(r["theorem_revision"],m["theorem_revision"],"theorem revision"),(r["repository_identity"],m["repository_identity"],"repository"),(r["product_subject_sha"],m["product_subject_sha"],"subject"),(r["claim"],m["claim"],"claim"),(r["nonclaims"],m["nonclaims"],"nonclaims")]:
  if a!=b:raise E("receipt "+n+" mismatch")
 vc(r["execution_context"],m);ids=[g["id"]for g in m["gates"]]
 if not isinstance(r["gate_results"],list)or[x.get("id")for x in r["gate_results"]]!=ids:raise E("receipt gate set mismatch")
 [nr(x)for x in r["gate_results"]]
 if r["verdict"]!=verdict(r["gate_results"]):raise E("receipt verdict mismatch")
 return True
def gate(m,root,repo,g,a,c):
 p=w=None;o=b"";code=20
 try:
  p,w=addwt(repo,m["product_subject_sha"]);h,d=state(w)
  if h!=m["product_subject_sha"]or d:raise E("fresh worktree not exact")
  env=os.environ.copy();env.update(QCAP_CAPSULE_COMMITMENT=cc(m),QCAP_PRODUCT_SUBJECT_SHA=m["product_subject_sha"],QCAP_PREDECESSOR_SHA=m["predecessor_sha"],QCAP_EXPECTED_CHANGED_PATHS_JSON=cj(m["expected_changed_paths"]).decode(),QCAP_EXPECTED_OBJECT_BLOBS_JSON=cj(m["expected_object_blobs"]).decode(),QCAP_TOOLCHAIN_PROFILE_ID=m["toolchain_profile_ref"]["id"],QCAP_TOOLCHAIN_PROFILE_REVISION=str(m["toolchain_profile_ref"]["revision"]),QCAP_TOOLCHAIN_PROFILE_DIGEST=m["toolchain_profile_ref"]["digest"],QCAP_ENVIRONMENT_PROFILE_ID=m["environment_profile_ref"]["id"],QCAP_ENVIRONMENT_PROFILE_REVISION=str(m["environment_profile_ref"]["revision"]),QCAP_ENVIRONMENT_PROFILE_DIGEST=m["environment_profile_ref"]["digest"],QCAP_RESOLVED_TOOLCHAIN_COMMITMENT=c["resolved_toolchain_commitment"],QCAP_RESOLVED_ENVIRONMENT_COMMITMENT=c["resolved_environment_commitment"],QCAP_ATTEMPT_ID=a,QCAP_SUBJECT_DIR=str(w))
  try:x=subprocess.run([str((Path(root)/g["script"]).resolve()),*g["args"]],cwd=w,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=g["timeout_seconds"]);o=x.stdout;code=x.returncode
  except subprocess.TimeoutExpired as x:o=(x.stdout or b"")+f"\nqcap_gate_timeout=RUNNER_FAILURE seconds={g['timeout_seconds']}\n".encode();code=20
  except OSError as x:o=f"qcap_gate_start=RUNNER_FAILURE {x}\n".encode();code=20
  if gs(code)!="RunnerInfrastructureFailure":
   h,d=state(w)
   if h!=m["product_subject_sha"]or d:o+=(f"\nqcap_subject_integrity=FAIL expected_head={m['product_subject_sha']} actual_head={h} dirty={bool(d)}\n").encode();code=10
 except E as x:o+=("qcap_worktree=RUNNER_FAILURE "+str(x)+"\n").encode();code=20
 finally:
  if p is not None:
   try:rmwt(repo,p,w)
   except E as x:o+=("qcap_worktree_cleanup=RUNNER_FAILURE "+str(x)+"\n").encode();code=20
 if o:sys.stderr.buffer.write(o+(b""if o.endswith(b"\n")else b"\n"))
 return{"id":g["id"],"status":gs(code),"output_sha256":hashlib.sha256(o).hexdigest(),"exit_code":code}
def run(m,root,repo,rid,a,c):
 vm(m,root);vc(c,m);pre(m,repo,rid);rs=[]
 for g in m["gates"]:
  x=gate(m,root,repo,g,a,c);rs.append(x)
  if x["status"]=="RunnerInfrastructureFailure":break
 done={x["id"]for x in rs};empty=hashlib.sha256(b"").hexdigest()
 for g in m["gates"]:
  if g["id"]not in done:rs.append({"id":g["id"],"status":"RunnerInfrastructureFailure","output_sha256":empty,"exit_code":20})
 rs.sort(key=lambda x:x["id"].encode());return receipt(m,a,c,rs)

CapsuleError=E
canonical_json=cj
capsule_commitment=cc
receipt_commitment=rc
validate_manifest=vm
validate_execution_context=vc
preflight=pre
compose_receipt=receipt
verify_receipt=vr
run_capsule=run

def load(p):return json.loads(Path(p).read_text())
def main(v=None):
 p=argparse.ArgumentParser();s=p.add_subparsers(dest="cmd",required=True);c=s.add_parser("commit");c.add_argument("manifest");x=s.add_parser("verify");x.add_argument("manifest");x.add_argument("--root",required=True);x=s.add_parser("verify-receipt");x.add_argument("manifest");x.add_argument("receipt");x=s.add_parser("run");x.add_argument("manifest");x.add_argument("--root",required=True);x.add_argument("--subject-repo",required=True);x.add_argument("--repository-identity",required=True);x.add_argument("--attempt-id",required=True);x.add_argument("--execution-context",required=True);x.add_argument("--receipt-out");a=p.parse_args(v)
 try:
  m=load(a.manifest)
  if a.cmd=="commit":vm(m);print("capsule_commitment="+cc(m));return 0
  if a.cmd=="verify":vm(m,a.root);print("capsule_commitment="+cc(m));print("capsule_integrity=PASS");return 0
  if a.cmd=="verify-receipt":vm(m);vr(load(a.receipt),m);print("receipt_integrity=PASS");return 0
  r=run(m,a.root,a.subject_repo,a.repository_identity,a.attempt_id,load(a.execution_context));z=json.dumps(r,sort_keys=True,indent=2)+"\n";(Path(a.receipt_out).write_text(z)if a.receipt_out else sys.stdout.write(z));return 0 if r["verdict"]=="CompletedConjunctivePass"else(10 if r["verdict"]=="CompletedConjunctiveFail"else 20)
 except(E,ValueError,json.JSONDecodeError)as x:print("qcap_error="+str(x),file=sys.stderr);return 20
if __name__=="__main__":raise SystemExit(main())