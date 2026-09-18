from __future__ import annotations
from qcap_canon import *
from qcap3_limits import validate_execution_context_v3
RECEIPT3_DOMAIN=b"MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V3\0"
RUNNER_FAILURE_REASONS={"OutputLimitExceeded","Timeout","ProcessStartFailure","OutputDrainTimeout","WorktreeMaterializationFailure","WorktreeCleanupFailure","ArtifactIntegrityFailure","GateReportedRunnerFailure","UnexpectedExitCode","RunnerInternalFailure"}
RESULT_KEYS={"id","status","effective_exit_code","captured_output_sha256","captured_output_bytes","output_truncated","runner_failure_reason"}

def receipt_commitment_v3(r):return commitment(RECEIPT3_DOMAIN,r)
def not_run_result(i):return{"id":i,"status":"GateNotRun","effective_exit_code":None,"captured_output_sha256":EMPTY_SHA256,"captured_output_bytes":0,"output_truncated":False,"runner_failure_reason":None}
def gate_result(i,status,code,captured,output_truncated=False,reason=None):
 return{"id":i,"status":status,"effective_exit_code":code,"captured_output_sha256":__import__('hashlib').sha256(captured).hexdigest(),"captured_output_bytes":len(captured),"output_truncated":bool(output_truncated),"runner_failure_reason":reason}
def validate_gate_result(r):
 exact_keys(r,RESULT_KEYS,"gate result")
 if not isinstance(r["id"],str)or not IDENT.fullmatch(r["id"]):raise CapsuleError("gate result id invalid")
 hex_value(r["captured_output_sha256"],H64,"captured output digest")
 n=r["captured_output_bytes"]
 if not isinstance(n,int)or isinstance(n,bool)or n<0:raise CapsuleError("captured output bytes invalid")
 if not isinstance(r["output_truncated"],bool):raise CapsuleError("output_truncated invalid")
 s=r["status"];c=r["effective_exit_code"];reason=r["runner_failure_reason"]
 if s=="GateNotRun":
  if c is not None or n!=0 or r["captured_output_sha256"]!=EMPTY_SHA256 or r["output_truncated"] or reason is not None:raise CapsuleError("GateNotRun fields invalid")
  return
 if s=="GatePass":
  if c!=0 or reason is not None or r["output_truncated"]:raise CapsuleError("GatePass fields invalid")
  return
 if s=="GateFail":
  if c!=10 or reason is not None or r["output_truncated"]:raise CapsuleError("GateFail fields invalid")
  return
 if s=="RunnerInfrastructureFailure":
  if c!=20 or reason not in RUNNER_FAILURE_REASONS:raise CapsuleError("runner failure fields invalid")
  if (reason=="OutputLimitExceeded")!=r["output_truncated"]:raise CapsuleError("output truncation/reason mismatch")
  return
 raise CapsuleError("gate status invalid")
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
def compose_receipt_v3(m,a,c,l,rs):
 if not isinstance(a,str)or not IDENT.fullmatch(a):raise CapsuleError("attempt id invalid")
 validate_execution_context_v3(c,m,l);ids=[g["id"]for g in m["gates"]]
 if[r.get("id")for r in rs]!=ids:raise CapsuleError("gate order mismatch")
 b={"receipt_format_revision":3,"capsule_commitment":capsule_commitment(m),"theorem_id":m["theorem_id"],"theorem_revision":m["theorem_revision"],"repository_identity":m["repository_identity"],"product_subject_sha":m["product_subject_sha"],"attempt_id":a,"execution_context":c,"gate_results":rs,"verdict":attempt_verdict(rs),"claim":m["claim"],"nonclaims":m["nonclaims"]};b["receipt_commitment"]=receipt_commitment_v3(b);return b
def verify_receipt_v3(r,m,l):
 ks={"receipt_format_revision","capsule_commitment","theorem_id","theorem_revision","repository_identity","product_subject_sha","attempt_id","execution_context","gate_results","verdict","claim","nonclaims","receipt_commitment"};exact_keys(r,ks,"receipt")
 if r["receipt_format_revision"]!=3:raise CapsuleError("unsupported receipt")
 supplied=hex_value(r["receipt_commitment"],H64,"receipt commitment");b=dict(r);b.pop("receipt_commitment")
 if receipt_commitment_v3(b)!=supplied:raise CapsuleError("receipt commitment mismatch")
 for k,e in(("capsule_commitment",capsule_commitment(m)),("theorem_id",m["theorem_id"]),("theorem_revision",m["theorem_revision"]),("repository_identity",m["repository_identity"]),("product_subject_sha",m["product_subject_sha"]),("claim",m["claim"]),("nonclaims",m["nonclaims"])):
  if r[k]!=e:raise CapsuleError(f"receipt {k} mismatch")
 validate_execution_context_v3(r["execution_context"],m,l);ids=[g["id"]for g in m["gates"]]
 if not isinstance(r["gate_results"],list)or[x.get("id")for x in r["gate_results"]]!=ids:raise CapsuleError("receipt gate set mismatch")
 if r["verdict"]!=attempt_verdict(r["gate_results"]):raise CapsuleError("receipt verdict mismatch")
 return True
