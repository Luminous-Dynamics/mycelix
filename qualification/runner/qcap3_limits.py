from __future__ import annotations
from pathlib import Path
from qcap_canon import CapsuleError,canonical_json,commitment,exact_keys,validate_profile,H64,hex_value
from qcap_manifest import validate_manifest
LIMITS_DOMAIN=b"MYCELIX_QCAP_EXECUTION_LIMITS_V1\0"
U64_MAX=(1<<64)-1
LIMIT_KEYS={
 "execution_limits_format_revision","profile_id","profile_revision","max_gate_output_bytes","max_gate_count",
 "max_gate_script_bytes","max_total_gate_script_bytes","max_args_per_gate","max_total_arg_bytes",
 "max_manifest_canonical_bytes","max_claim_nonclaim_utf8_bytes"
}
CTX3_KEYS={
 "execution_context_format_revision","runner_profile_ref","toolchain_profile_ref","environment_profile_ref",
 "execution_limits_profile_ref","resolved_runner_commitment","resolved_toolchain_commitment","resolved_environment_commitment"
}

def u64(v,w,minimum=0):
 if not isinstance(v,int)or isinstance(v,bool)or v<minimum or v>U64_MAX:raise CapsuleError(f"{w} invalid")
 return v

def checked_add(a,b,w):
 if a>U64_MAX-b:raise CapsuleError(f"{w} accounting overflow")
 return a+b

def validate_limits(l):
 exact_keys(l,LIMIT_KEYS,"execution limits")
 if l["execution_limits_format_revision"]!=1:raise CapsuleError("unsupported execution limits format")
 if not isinstance(l["profile_id"],str)or not l["profile_id"]:raise CapsuleError("limits profile id invalid")
 u64(l["profile_revision"],"limits profile revision",1)
 for k in LIMIT_KEYS-{"execution_limits_format_revision","profile_id","profile_revision"}:u64(l[k],k,1)
 canonical_json(l)
 return l

def limits_digest(l):validate_limits(l);return commitment(LIMITS_DOMAIN,l)
def limits_ref(l):
 validate_limits(l);return{"id":l["profile_id"],"revision":l["profile_revision"],"digest":limits_digest(l)}

def validate_execution_context_v3(c,m,l,expected_runner_commitment=None):
 exact_keys(c,CTX3_KEYS,"execution context v3")
 if c["execution_context_format_revision"]!=3:raise CapsuleError("unsupported execution context")
 validate_profile(c["runner_profile_ref"],"runner profile");validate_profile(c["toolchain_profile_ref"],"execution toolchain");validate_profile(c["environment_profile_ref"],"execution environment");validate_profile(c["execution_limits_profile_ref"],"execution limits profile")
 if c["toolchain_profile_ref"]!=m["toolchain_profile_ref"]:raise CapsuleError("toolchain profile mismatch")
 if c["environment_profile_ref"]!=m["environment_profile_ref"]:raise CapsuleError("environment profile mismatch")
 if c["execution_limits_profile_ref"]!=limits_ref(l):raise CapsuleError("execution limits profile mismatch")
 for k in("resolved_runner_commitment","resolved_toolchain_commitment","resolved_environment_commitment"):hex_value(c[k],H64,k)
 if expected_runner_commitment is not None and c["resolved_runner_commitment"]!=expected_runner_commitment:raise CapsuleError("resolved runner commitment mismatch")
 canonical_json(c)
 return c

def account_manifest_resources(m,root,l):
 validate_limits(l);validate_manifest(m,root);root=Path(root).resolve()
 if len(m["gates"])>l["max_gate_count"]:raise CapsuleError("gate count exceeds execution limit")
 mb=len(canonical_json(m))
 if mb>l["max_manifest_canonical_bytes"]:raise CapsuleError("manifest canonical bytes exceed execution limit")
 text=len(m["claim"].encode())
 for n in m["nonclaims"]:text=checked_add(text,len(n.encode()),"claim/nonclaim bytes")
 if text>l["max_claim_nonclaim_utf8_bytes"]:raise CapsuleError("claim/nonclaim bytes exceed execution limit")
 total_scripts=0;total_args=0
 for g in m["gates"]:
  p=(root/g["script"]).resolve();sz=p.stat().st_size
  if sz>l["max_gate_script_bytes"]:raise CapsuleError("gate script exceeds execution limit")
  total_scripts=checked_add(total_scripts,sz,"gate script bytes")
  if len(g["args"])>l["max_args_per_gate"]:raise CapsuleError("gate arg count exceeds execution limit")
  for a in g["args"]:total_args=checked_add(total_args,len(a.encode()),"gate arg bytes")
 if total_scripts>l["max_total_gate_script_bytes"]:raise CapsuleError("total gate script bytes exceed execution limit")
 if total_args>l["max_total_arg_bytes"]:raise CapsuleError("total gate arg bytes exceed execution limit")
 return{"gate_count":len(m["gates"]),"manifest_canonical_bytes":mb,"claim_nonclaim_utf8_bytes":text,"total_gate_script_bytes":total_scripts,"total_arg_bytes":total_args}
