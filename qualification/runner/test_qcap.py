import hashlib,importlib.util,json,subprocess,tempfile,unittest
from pathlib import Path
H=Path(__file__).resolve().parent;S=importlib.util.spec_from_file_location("qcap",H/"qcap.py");q=importlib.util.module_from_spec(S);S.loader.exec_module(q)
def pf(n):return{"id":n,"revision":1,"digest":hashlib.sha256(n.encode()).hexdigest()}
class Core(unittest.TestCase):
 def m(self,r,ids=("a",)):
  gs=[]
  for i in ids:
   p=r/(i+".sh");p.write_text("#!/bin/sh\nexit 0\n");p.chmod(0o755);gs.append({"id":i,"class":"theorem"if i=="a"else"oracle","script":p.name,"sha256":hashlib.sha256(p.read_bytes()).hexdigest(),"args":[],"timeout_seconds":5})
  return{"capsule_format_revision":1,"theorem_id":"TEST-001","theorem_revision":1,"repository_identity":"Luminous-Dynamics/mycelix","product_subject_sha":"1"*40,"predecessor_sha":"2"*40,"expected_changed_paths":["a.txt"],"expected_object_blobs":{"a.txt":"3"*40},"toolchain_profile_ref":pf("tool"),"environment_profile_ref":pf("env"),"gates":gs,"verdict":{"kind":"all","required_gate_ids":list(ids)},"claim":"claim","nonclaims":["a","b"]}
 def c(self,m):return{"execution_context_format_revision":2,"runner_profile_ref":pf("runner"),"toolchain_profile_ref":m["toolchain_profile_ref"],"environment_profile_ref":m["environment_profile_ref"],"resolved_runner_commitment":q.runner_commitment(),"resolved_toolchain_commitment":"4"*64,"resolved_environment_commitment":"5"*64}
 def r(self,i,s,c,o=b""):return{"id":i,"status":s,"output_sha256":hashlib.sha256(o).hexdigest(),"exit_code":c}
 def test_commitment_and_types(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d));self.assertEqual(q.capsule_commitment(m),q.capsule_commitment(dict(reversed(list(m.items())))));n=json.loads(json.dumps(m));n["gates"][0]["timeout_seconds"]+=1;self.assertNotEqual(q.capsule_commitment(m),q.capsule_commitment(n))
  self.assertRaises(q.CapsuleError,q.canonical_json,{"x":1.5});self.assertRaises(q.CapsuleError,q.canonical_json,{1:"x"})
 def test_state_machine(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d),("a","b","c"));c=self.c(m);good=[self.r("a","RunnerInfrastructureFailure",20),q.not_run_result("b"),q.not_run_result("c")];x=q.compose_receipt(m,"a1",c,good);self.assertEqual(x["verdict"],"RunnerInfrastructureFailure");self.assertTrue(q.verify_receipt(x,m))
   self.assertRaises(q.CapsuleError,q.compose_receipt,m,"a1",c,[q.not_run_result("a"),self.r("b","RunnerInfrastructureFailure",20),q.not_run_result("c")]);self.assertRaises(q.CapsuleError,q.compose_receipt,m,"a1",c,[self.r("a","RunnerInfrastructureFailure",20),q.not_run_result("b"),self.r("c","GatePass",0)])
 def test_theorem_fail_receipt(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d),("a","b"));x=q.compose_receipt(m,"a1",self.c(m),[self.r("a","GateFail",10),self.r("b","GatePass",0)]);self.assertEqual(x["verdict"],"CompletedConjunctiveFail");self.assertTrue(q.verify_receipt(x,m))
 def test_context_and_receipt_binding(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d));c=self.c(m);q.validate_execution_context(c,m,q.runner_commitment());bad=json.loads(json.dumps(c));bad["environment_profile_ref"]=pf("wrong");self.assertRaises(q.CapsuleError,q.validate_execution_context,bad,m)
   r=q.compose_receipt(m,"a1",c,[self.r("a","GatePass",0)]);other=json.loads(json.dumps(c));other["resolved_runner_commitment"]="6"*64;r2=q.compose_receipt(m,"a1",other,[self.r("a","GatePass",0)]);self.assertNotEqual(r["receipt_commitment"],r2["receipt_commitment"])
 def test_tamper_rejected_even_if_recommitted(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d),("a","b"));r=q.compose_receipt(m,"a1",self.c(m),[self.r("a","RunnerInfrastructureFailure",20),q.not_run_result("b")]);r["gate_results"][1].update(status="GatePass",exit_code=0);body=dict(r);body.pop("receipt_commitment");r["receipt_commitment"]=q.receipt_commitment(body);self.assertRaises(q.CapsuleError,q.verify_receipt,r,m)
 def test_v2_schemas_and_independent_vectors(self):
  sd=H.parent/"schema"
  for n,k in(("qualification-execution-context-v2.schema.json","execution_context_format_revision"),("qualification-attempt-receipt-v2.schema.json","receipt_format_revision")):
   x=json.loads((sd/n).read_text());self.assertFalse(x["additionalProperties"]);self.assertEqual(x["properties"][k]["const"],2)
  o=H.parent/"vectors"/"verify_qcap_v2_vectors.py";r=subprocess.run([str(o)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stdout+r.stderr);self.assertIn("qcap_v2_independent_vectors=PASS",r.stdout)
if __name__=="__main__":unittest.main()
