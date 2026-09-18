from qcap_canon import *
from qcap_manifest import validate_execution_context
def gate_status(c):return"GatePass"if c==0 else("GateFail"if c==10 else"RunnerInfrastructureFailure")
def not_run_result(i):return{"id":i,"status":"GateNotRun","output_sha256":EMPTY_SHA256,"exit_code":None}
def validate_gate_result(r):
 exact_keys(r,{"id","status","output_sha256","exit_code"},"gate result")
 if not isinstance(r["id"],str)or not IDENT.fullmatch(r["id"]):raise CapsuleError("gate result id invalid")
 hex_value(r["output_sha256"],H64,"output digest")
 if r["status"]=="GateNotRun":
  if r["exit_code"]is not None or r["output_sha256"]!=EMPTY_SHA256:raise CapsuleError("GateNotRun must have null exit and empty output digest")
  return
 c=r["exit_code"]
 if not isinstance(c,int)or isinstance(c,bool)or r["status"]!=gate_status(c):raise CapsuleError("gate status/exit mismatch")
def attempt_verdict(rs):
 infra=False;fail=False
 for r in rs:
  validate_gate_result(r);s=r["status"]
  if infra:
   if s!="GateNotRun":raise CapsuleError("only GateNotRun may follow runner failure")
   continue
  if s=="GateNotRun":raise CapsuleError("GateNotRun before runner failure")
  if s=="RunnerInfrastructureFailure":infra=True
  elif s=="GateFail":fail=True
 return"RunnerInfrastructureFailure"if infra else("CompletedConjunctiveFail"if fail else"CompletedConjunctivePass")
def compose_receipt(m,a,c,rs):
 if not isinstance(a,str)or not IDENT.fullmatch(a):raise CapsuleError("attempt id invalid")
 validate_execution_context(c,m);ids=[g["id"]for g in m["gates"]]
 if[r.get("id")for r in rs]!=ids:raise CapsuleError("gate order mismatch")
 b={"receipt_format_revision":2,"capsule_commitment":capsule_commitment(m),"theorem_id":m["theorem_id"],"theorem_revision":m["theorem_revision"],"repository_identity":m["repository_identity"],"product_subject_sha":m["product_subject_sha"],"attempt_id":a,"execution_context":c,"gate_results":rs,"verdict":attempt_verdict(rs),"claim":m["claim"],"nonclaims":m["nonclaims"]};b["receipt_commitment"]=receipt_commitment(b);return b
def verify_receipt(r,m):
 ks={"receipt_format_revision","capsule_commitment","theorem_id","theorem_revision","repository_identity","product_subject_sha","attempt_id","execution_context","gate_results","verdict","claim","nonclaims","receipt_commitment"};exact_keys(r,ks,"receipt")
 if r["receipt_format_revision"]!=2:raise CapsuleError("unsupported receipt")
 supplied=hex_value(r["receipt_commitment"],H64,"receipt commitment");b=dict(r);b.pop("receipt_commitment")
 if receipt_commitment(b)!=supplied:raise CapsuleError("receipt commitment mismatch")
 for k,e in(("capsule_commitment",capsule_commitment(m)),("theorem_id",m["theorem_id"]),("theorem_revision",m["theorem_revision"]),("repository_identity",m["repository_identity"]),("product_subject_sha",m["product_subject_sha"]),("claim",m["claim"]),("nonclaims",m["nonclaims"])):
  if r[k]!=e:raise CapsuleError(f"receipt {k} mismatch")
 if not isinstance(r["attempt_id"],str)or not IDENT.fullmatch(r["attempt_id"]):raise CapsuleError("receipt attempt id invalid")
 validate_execution_context(r["execution_context"],m);ids=[g["id"]for g in m["gates"]]
 if not isinstance(r["gate_results"],list)or[x.get("id")for x in r["gate_results"]]!=ids:raise CapsuleError("receipt gate set mismatch")
 if r["verdict"]!=attempt_verdict(r["gate_results"]):raise CapsuleError("receipt verdict mismatch")
 return True
