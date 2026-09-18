from pathlib import Path
import hashlib
from qcap_canon import *
TOP={"capsule_format_revision","theorem_id","theorem_revision","repository_identity","product_subject_sha","predecessor_sha","expected_changed_paths","expected_object_blobs","toolchain_profile_ref","environment_profile_ref","gates","verdict","claim","nonclaims"}
CTX={"execution_context_format_revision","runner_profile_ref","toolchain_profile_ref","environment_profile_ref","resolved_runner_commitment","resolved_toolchain_commitment","resolved_environment_commitment"}
def validate_manifest(m,root=None):
 exact_keys(m,TOP,"manifest")
 if m["capsule_format_revision"]!=1:raise CapsuleError("unsupported capsule format")
 if not isinstance(m["theorem_id"],str)or not IDENT.fullmatch(m["theorem_id"]):raise CapsuleError("theorem id invalid")
 r=m["theorem_revision"]
 if not isinstance(r,int)or isinstance(r,bool)or r<1:raise CapsuleError("theorem revision invalid")
 if not isinstance(m["repository_identity"],str)or"/"not in m["repository_identity"]:raise CapsuleError("repository identity invalid")
 hex_value(m["product_subject_sha"],H40,"subject");hex_value(m["predecessor_sha"],H40,"predecessor")
 if m["product_subject_sha"]==m["predecessor_sha"]:raise CapsuleError("subject equals predecessor")
 sorted_unique_strings(m["expected_changed_paths"],"changed paths")
 for p in m["expected_changed_paths"]:validate_relpath(p,"changed path")
 blobs=m["expected_object_blobs"]
 if not isinstance(blobs,dict)or not blobs:raise CapsuleError("blob map invalid")
 for p,d in blobs.items():validate_relpath(p,"blob path");hex_value(d,H40,"blob sha")
 validate_profile(m["toolchain_profile_ref"],"toolchain profile");validate_profile(m["environment_profile_ref"],"environment profile")
 gates=m["gates"]
 if not isinstance(gates,list)or not gates:raise CapsuleError("gates empty")
 ids=[];scripts=[]
 for g in gates:
  exact_keys(g,{"id","class","script","sha256","args","timeout_seconds"},"gate")
  if not isinstance(g["id"],str)or not IDENT.fullmatch(g["id"]):raise CapsuleError("gate id invalid")
  if g["class"]not in{"theorem","oracle","lineage"}:raise CapsuleError("gate class invalid")
  validate_relpath(g["script"],"gate script");hex_value(g["sha256"],H64,"gate sha256")
  if not isinstance(g["args"],list)or not all(isinstance(a,str)for a in g["args"]):raise CapsuleError("gate args invalid")
  t=g["timeout_seconds"]
  if not isinstance(t,int)or isinstance(t,bool)or not 1<=t<=7200:raise CapsuleError("gate timeout invalid")
  ids.append(g["id"]);scripts.append((g["script"],g["sha256"]))
 if ids!=sorted(set(ids),key=lambda x:x.encode("utf-8")):raise CapsuleError("gates not sorted unique")
 exact_keys(m["verdict"],{"kind","required_gate_ids"},"verdict");sorted_unique_strings(m["verdict"]["required_gate_ids"],"required gates")
 if m["verdict"]["kind"]!="all"or m["verdict"]["required_gate_ids"]!=ids:raise CapsuleError("verdict invalid")
 if not isinstance(m["claim"],str)or not m["claim"].strip():raise CapsuleError("claim empty")
 sorted_unique_strings(m["nonclaims"],"nonclaims");canonical_json(m)
 if root is not None:
  root=Path(root).resolve()
  for rel,expected in scripts:
   p=(root/rel).resolve()
   try:p.relative_to(root)
   except ValueError as e:raise CapsuleError("script escapes root")from e
   if not p.is_file()or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise CapsuleError(f"script digest mismatch {rel}")
def validate_execution_context(c,m,expected_runner_commitment=None):
 exact_keys(c,CTX,"execution context")
 if c["execution_context_format_revision"]!=2:raise CapsuleError("unsupported execution context")
 validate_profile(c["runner_profile_ref"],"runner profile");validate_profile(c["toolchain_profile_ref"],"execution toolchain");validate_profile(c["environment_profile_ref"],"execution environment")
 if c["toolchain_profile_ref"]!=m["toolchain_profile_ref"]:raise CapsuleError("toolchain profile mismatch")
 if c["environment_profile_ref"]!=m["environment_profile_ref"]:raise CapsuleError("environment profile mismatch")
 for k in("resolved_runner_commitment","resolved_toolchain_commitment","resolved_environment_commitment"):hex_value(c[k],H64,k)
 if expected_runner_commitment is not None and c["resolved_runner_commitment"]!=expected_runner_commitment:raise CapsuleError("resolved runner commitment mismatch")
 canonical_json(c)
