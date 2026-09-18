from __future__ import annotations
import hashlib,os,sys
from pathlib import Path
from qcap_canon import CapsuleError,canonical_json,capsule_commitment
from qcap_manifest import validate_execution_context,validate_manifest
from qcap_receipt import compose_receipt,gate_status,not_run_result
from qcap_isolation import *
def verify_gate_artifact(root,g):
 root=Path(root).resolve();p=(root/g["script"]).resolve()
 try:p.relative_to(root)
 except ValueError as e:raise CapsuleError("gate script escapes capsule root")from e
 if not p.is_file()or hashlib.sha256(p.read_bytes()).hexdigest()!=g["sha256"]:raise CapsuleError("gate script digest changed before execution")
 return p
def controlled_env(m,c,a,w,scratch):
 (scratch/"home").mkdir();(scratch/"tmp").mkdir()
 return{"PATH":os.environ.get("PATH","/usr/bin:/bin"),"HOME":str(scratch/"home"),"TMPDIR":str(scratch/"tmp"),"LANG":"C","LC_ALL":"C","TZ":"UTC","QCAP_CAPSULE_COMMITMENT":capsule_commitment(m),"QCAP_PRODUCT_SUBJECT_SHA":m["product_subject_sha"],"QCAP_PREDECESSOR_SHA":m["predecessor_sha"],"QCAP_EXPECTED_CHANGED_PATHS_JSON":canonical_json(m["expected_changed_paths"]).decode(),"QCAP_EXPECTED_OBJECT_BLOBS_JSON":canonical_json(m["expected_object_blobs"]).decode(),"QCAP_TOOLCHAIN_PROFILE_ID":m["toolchain_profile_ref"]["id"],"QCAP_TOOLCHAIN_PROFILE_REVISION":str(m["toolchain_profile_ref"]["revision"]),"QCAP_TOOLCHAIN_PROFILE_DIGEST":m["toolchain_profile_ref"]["digest"],"QCAP_ENVIRONMENT_PROFILE_ID":m["environment_profile_ref"]["id"],"QCAP_ENVIRONMENT_PROFILE_REVISION":str(m["environment_profile_ref"]["revision"]),"QCAP_ENVIRONMENT_PROFILE_DIGEST":m["environment_profile_ref"]["digest"],"QCAP_RESOLVED_RUNNER_COMMITMENT":c["resolved_runner_commitment"],"QCAP_RESOLVED_TOOLCHAIN_COMMITMENT":c["resolved_toolchain_commitment"],"QCAP_RESOLVED_ENVIRONMENT_COMMITMENT":c["resolved_environment_commitment"],"QCAP_ATTEMPT_ID":a,"QCAP_SUBJECT_DIR":str(w)}
def execute_gate(m,root,repo,g,a,c):
 parent=w=None;o=b"";code=20
 try:
  script=verify_gate_artifact(root,g);parent,w=add_worktree(repo,m["product_subject_sha"]);h,d=state(w)
  if h!=m["product_subject_sha"]or d:raise CapsuleError("fresh worktree not exact")
  scratch=parent/"scratch";scratch.mkdir();env=controlled_env(m,c,a,w,scratch);o,code=execute([str(script),*g["args"]],w,env,g["timeout_seconds"])
  if gate_status(code)!="RunnerInfrastructureFailure":
   h,d=state(w)
   if h!=m["product_subject_sha"]or d:o+=(f"\nqcap_subject_integrity=FAIL expected_head={m['product_subject_sha']} actual_head={h} dirty={bool(d)}\n").encode();code=10
 except CapsuleError as e:o+=f"qcap_runner=RUNNER_FAILURE {e}\n".encode();code=20
 finally:
  if parent is not None:
   try:remove_worktree(repo,parent,w)
   except CapsuleError as e:o+=f"qcap_worktree_cleanup=RUNNER_FAILURE {e}\n".encode();code=20
 if o:sys.stderr.buffer.write(o+(b""if o.endswith(b"\n")else b"\n"))
 return{"id":g["id"],"status":gate_status(code),"output_sha256":hashlib.sha256(o).hexdigest(),"exit_code":code}
def run_capsule(m,root,repo,rid,a,c,expected_runner_commitment):
 enable_linux_subreaper();validate_manifest(m,root);validate_execution_context(c,m,expected_runner_commitment);preflight(m,repo,rid);rs=[];infra=False
 for g in m["gates"]:
  if infra:rs.append(not_run_result(g["id"]));continue
  r=execute_gate(m,root,repo,g,a,c);rs.append(r);infra=r["status"]=="RunnerInfrastructureFailure"
 return compose_receipt(m,a,c,rs)
