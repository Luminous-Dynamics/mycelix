from __future__ import annotations
import hashlib,os,sys
from pathlib import Path
from qcap_canon import CapsuleError,canonical_json,capsule_commitment
from qcap_manifest import validate_manifest
from qcap3_limits import account_manifest_resources,limits_ref,validate_execution_context_v3,validate_limits
from qcap3_receipt import compose_receipt_v3,gate_result,not_run_result
from qcap3_isolation import *
def verified_gate_bytes(root,g):
 root=Path(root).resolve();p=root
 for part in Path(g["script"]).parts:
  p=p/part
  if p.is_symlink():raise CapsuleError("gate script symlink forbidden")
 rp=p.resolve()
 try:rp.relative_to(root)
 except ValueError as e:raise CapsuleError("gate script escapes capsule root")from e
 if not rp.is_file():raise CapsuleError("gate script unavailable")
 b=rp.read_bytes()
 if hashlib.sha256(b).hexdigest()!=g["sha256"]:raise CapsuleError("gate script digest changed before execution")
 return b
def snapshot_gate(parent,g,b):
 d=parent/"gate-snapshots";d.mkdir(mode=0o700)
 token=hashlib.sha256(g["id"].encode()+b"\0"+g["sha256"].encode()).hexdigest();p=d/(token+".gate")
 try:fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o700)
 except OSError as e:raise CapsuleError("gate snapshot creation failed")from e
 try:
  with os.fdopen(fd,"wb")as f:f.write(b);f.flush();os.fsync(f.fileno())
  os.chmod(p,0o700)
 except OSError as e:
  try:p.unlink()
  except OSError:pass
  raise CapsuleError("gate snapshot write failed")from e
 if p.is_symlink()or hashlib.sha256(p.read_bytes()).hexdigest()!=g["sha256"]:raise CapsuleError("gate snapshot integrity failure")
 return p
def controlled_env(m,c,l,a,w,scratch):
 (scratch/"home").mkdir();(scratch/"tmp").mkdir()
 return{"PATH":os.environ.get("PATH","/usr/bin:/bin"),"HOME":str(scratch/"home"),"TMPDIR":str(scratch/"tmp"),"LANG":"C","LC_ALL":"C","TZ":"UTC","QCAP_CAPSULE_COMMITMENT":capsule_commitment(m),"QCAP_PRODUCT_SUBJECT_SHA":m["product_subject_sha"],"QCAP_PREDECESSOR_SHA":m["predecessor_sha"],"QCAP_EXPECTED_CHANGED_PATHS_JSON":canonical_json(m["expected_changed_paths"]).decode(),"QCAP_EXPECTED_OBJECT_BLOBS_JSON":canonical_json(m["expected_object_blobs"]).decode(),"QCAP_RESOLVED_RUNNER_COMMITMENT":c["resolved_runner_commitment"],"QCAP_RESOLVED_TOOLCHAIN_COMMITMENT":c["resolved_toolchain_commitment"],"QCAP_RESOLVED_ENVIRONMENT_COMMITMENT":c["resolved_environment_commitment"],"QCAP_EXECUTION_LIMITS_DIGEST":limits_ref(l)["digest"],"QCAP_ATTEMPT_ID":a,"QCAP_SUBJECT_DIR":str(w)}
def execute_gate(m,root,repo,g,a,c,l):
 parent=w=None;captured=b"";code=20;reason="RunnerInternalFailure";truncated=False
 try:
  gate_bytes=verified_gate_bytes(root,g);parent,w=add_worktree(repo,m["product_subject_sha"]);h,d=state(w)
  if h!=m["product_subject_sha"]or d:raise CapsuleError("fresh worktree not exact")
  scratch=parent/"scratch";scratch.mkdir();script=snapshot_gate(parent,g,gate_bytes);env=controlled_env(m,c,l,a,w,scratch);captured,code,reason,truncated=execute_bounded([str(script),*g["args"]],w,env,g["timeout_seconds"],l["max_gate_output_bytes"])
  if code!=20:
   h,d=state(w)
   if h!=m["product_subject_sha"]or d:
    code=10;reason=None;truncated=False
 except CapsuleError as e:
  captured=("qcap3_runner=RUNNER_FAILURE "+str(e)+"\n").encode()[:l["max_gate_output_bytes"]];code=20;reason="ArtifactIntegrityFailure";truncated=False
 finally:
  if parent is not None:
   try:remove_worktree(repo,parent,w)
   except CapsuleError:
    code=20;reason="WorktreeCleanupFailure";truncated=False
 if captured:sys.stderr.buffer.write(captured+(b""if captured.endswith(b"\n")else b"\n"))
 status="GatePass"if code==0 else("GateFail"if code==10 else"RunnerInfrastructureFailure")
 return gate_result(g["id"],status,code,captured,truncated,reason)
def run_capsule_v3(m,root,repo,rid,a,c,l,expected_runner_commitment):
 enable_linux_subreaper();validate_limits(l);validate_manifest(m,root);account_manifest_resources(m,root,l);validate_execution_context_v3(c,m,l,expected_runner_commitment);preflight(m,repo,rid);rs=[];infra=False
 for g in m["gates"]:
  if infra:rs.append(not_run_result(g["id"]));continue
  r=execute_gate(m,root,repo,g,a,c,l);rs.append(r);infra=r["status"]=="RunnerInfrastructureFailure"
 return compose_receipt_v3(m,a,c,l,rs)
