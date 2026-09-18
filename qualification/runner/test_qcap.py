import hashlib,importlib.util,json,os,subprocess,tempfile,unittest
from pathlib import Path
H=Path(__file__).resolve().parent;S=importlib.util.spec_from_file_location("qcap",H/"qcap.py");q=importlib.util.module_from_spec(S);S.loader.exec_module(q)
def pf(n):return{"id":n,"revision":1,"digest":hashlib.sha256(n.encode()).hexdigest()}
def repo(root):
 r=root/"r";r.mkdir();subprocess.run(["git","init","-q",str(r)],check=True);subprocess.run(["git","-C",str(r),"config","user.email","q@invalid"],check=True);subprocess.run(["git","-C",str(r),"config","user.name","Q"],check=True)
 (r/"base").write_text("b\n");subprocess.run(["git","-C",str(r),"add","base"],check=True);subprocess.run(["git","-C",str(r),"commit","-q","-m","b"],check=True);p=subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip()
 (r/"a.txt").write_text("subject\n");subprocess.run(["git","-C",str(r),"add","a.txt"],check=True);subprocess.run(["git","-C",str(r),"commit","-q","-m","s"],check=True);s=subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip();b=subprocess.check_output(["git","-C",str(r),"rev-parse",f"{s}:a.txt"],text=True).strip();return r,p,s,b
class T(unittest.TestCase):
 def m(self,root,ids=("a",),timeout=30):
  gs=[]
  for i in ids:
   f=root/(i+".sh");f.write_text("#!/bin/sh\necho "+i+"\nexit 0\n");f.chmod(0o755);gs.append({"id":i,"class":"theorem"if i=="a"else"oracle","script":f.name,"sha256":hashlib.sha256(f.read_bytes()).hexdigest(),"args":[],"timeout_seconds":timeout})
  return{"capsule_format_revision":1,"theorem_id":"TEST-1","theorem_revision":1,"repository_identity":"Luminous-Dynamics/mycelix","product_subject_sha":"1"*40,"predecessor_sha":"2"*40,"expected_changed_paths":["a.txt"],"expected_object_blobs":{"a.txt":"3"*40},"toolchain_profile_ref":pf("tool"),"environment_profile_ref":pf("env"),"gates":gs,"verdict":{"kind":"all","required_gate_ids":list(ids)},"claim":"claim","nonclaims":["a","b"]}
 def c(self,m):return{"execution_context_format_revision":1,"runner_profile_ref":pf("runner"),"toolchain_profile_ref":m["toolchain_profile_ref"],"environment_profile_ref":m["environment_profile_ref"],"resolved_toolchain_commitment":hashlib.sha256(b"tool").hexdigest(),"resolved_environment_commitment":hashlib.sha256(b"env").hexdigest()}
 def bind(self,m,p,s,b):m.update(product_subject_sha=s,predecessor_sha=p,expected_object_blobs={"a.txt":b});return m
 def test_commitment_and_script_integrity(self):
  with tempfile.TemporaryDirectory()as d:
   r=Path(d);m=self.m(r);self.assertEqual(q.cc(m),q.cc(dict(reversed(list(m.items())))));n=json.loads(json.dumps(m));n["claim"]="x";self.assertNotEqual(q.cc(m),q.cc(n));n=json.loads(json.dumps(m));n["gates"][0]["timeout_seconds"]+=1;self.assertNotEqual(q.cc(m),q.cc(n));q.vm(m,r);(r/"a.sh").write_text("#!/bin/sh\nexit 0\n");self.assertRaises(q.E,q.vm,m,r)
 def test_execution_context_profile_binding(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d));c=self.c(m);q.vc(c,m);c["environment_profile_ref"]=pf("wrong");self.assertRaises(q.E,q.vc,c,m)
 def test_receipt_binding_and_semantics(self):
  with tempfile.TemporaryDirectory()as d:
   m=self.m(Path(d));c=self.c(m);e=hashlib.sha256(b"").hexdigest();rs=[{"id":"a","status":"GatePass","output_sha256":e,"exit_code":0}];r=q.receipt(m,"a1",c,rs);self.assertTrue(q.vr(r,m));c2=json.loads(json.dumps(c));c2["resolved_toolchain_commitment"]="4"*64;r2=q.receipt(m,"a1",c2,rs);self.assertNotEqual(r["receipt_commitment"],r2["receipt_commitment"]);r["gate_results"][0]["exit_code"]=10;z=dict(r);z.pop("receipt_commitment");r["receipt_commitment"]=q.rc(z);self.assertRaises(q.E,q.vr,r,m)
 def test_exact_subject_materialization_does_not_touch_caller_checkout(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);(r/"later").write_text("x");subprocess.run(["git","-C",str(r),"add","later"],check=True);subprocess.run(["git","-C",str(r),"commit","-q","-m","later"],check=True);later=subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip();cr=z/"c";cr.mkdir();m=self.bind(self.m(cr),p,s,b);x=q.run(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual(x["verdict"],"CompletedConjunctivePass");self.assertEqual(subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip(),later)
 def test_gate_planes_are_isolated_and_mutation_is_red(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.bind(self.m(cr,("a","b")),p,s,b);a=cr/"a.sh";a.write_text("#!/bin/sh\necho leak > unexpected.txt\nexit 10\n");a.chmod(0o755);bb=cr/"b.sh";bb.write_text("#!/bin/sh\ntest ! -e unexpected.txt\nexit 0\n");bb.chmod(0o755)
   for g in m["gates"]:g["sha256"]=hashlib.sha256((cr/g["script"]).read_bytes()).hexdigest()
   x=q.run(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual([y["status"]for y in x["gate_results"]],["GateFail","GatePass"]);self.assertFalse((r/"unexpected.txt").exists())
 def test_tracked_or_untracked_mutation_is_gate_fail(self):
  for body in("echo changed > a.txt","echo leak > unexpected.txt"):
   with self.subTest(body=body),tempfile.TemporaryDirectory()as d:
    z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.bind(self.m(cr),p,s,b);f=cr/"a.sh";f.write_text("#!/bin/sh\n"+body+"\nexit 0\n");f.chmod(0o755);m["gates"][0]["sha256"]=hashlib.sha256(f.read_bytes()).hexdigest();x=q.run(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual(x["gate_results"][0]["status"],"GateFail");self.assertFalse((r/"unexpected.txt").exists());self.assertEqual((r/"a.txt").read_text(),"subject\n")
 def test_timeout_is_runner_failure(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.bind(self.m(cr,timeout=1),p,s,b);f=cr/"a.sh";f.write_text("#!/bin/sh\nsleep 5\n");f.chmod(0o755);m["gates"][0]["sha256"]=hashlib.sha256(f.read_bytes()).hexdigest();x=q.run(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual(x["verdict"],"RunnerInfrastructureFailure")
 def test_lineage_gate_failures_are_theorem_red(self):
  gate=(H.parent/"gates"/"subject_identity.py").resolve()
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,_=repo(z);base=os.environ.copy();base.update(QCAP_SUBJECT_DIR=str(r),QCAP_PRODUCT_SUBJECT_SHA=s,QCAP_EXPECTED_CHANGED_PATHS_JSON='["a.txt"]')
   for par,blobs in(("0"*40,"{}"),(p,'{"missing":"0000000000000000000000000000000000000000"}')):
    e=base.copy();e.update(QCAP_PREDECESSOR_SHA=par,QCAP_EXPECTED_OBJECT_BLOBS_JSON=blobs);self.assertEqual(subprocess.run([str(gate)],env=e).returncode,10)
 def test_preflight_repository_identity(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.bind(self.m(cr),p,s,b);self.assertRaises(q.E,q.pre,m,r,"wrong/repo")
 def test_closed_schemas(self):
  sd=H.parent/"schema"
  for n,k in(("qualification-capsule-v1.schema.json","capsule_format_revision"),("qualification-execution-context-v1.schema.json","execution_context_format_revision"),("qualification-attempt-receipt-v1.schema.json","receipt_format_revision")):
   x=json.loads((sd/n).read_text());self.assertFalse(x["additionalProperties"]);self.assertEqual(x["properties"][k]["const"],1)
if __name__=="__main__":unittest.main()