import hashlib,importlib.util,os,subprocess,tempfile,time,unittest
from pathlib import Path
H=Path(__file__).resolve().parent;S=importlib.util.spec_from_file_location("qcap",H/"qcap.py");q=importlib.util.module_from_spec(S);S.loader.exec_module(q)
def pf(n):return{"id":n,"revision":1,"digest":hashlib.sha256(n.encode()).hexdigest()}
def repo(z):
 r=z/"r";r.mkdir();subprocess.run(["git","init","-q",str(r)],check=True);subprocess.run(["git","-C",str(r),"config","user.email","q@invalid"],check=True);subprocess.run(["git","-C",str(r),"config","user.name","Q"],check=True);(r/"base").write_text("b\n");subprocess.run(["git","-C",str(r),"add","base"],check=True);subprocess.run(["git","-C",str(r),"commit","-q","-m","b"],check=True);p=subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip();(r/"a.txt").write_text("subject\n");subprocess.run(["git","-C",str(r),"add","a.txt"],check=True);subprocess.run(["git","-C",str(r),"commit","-q","-m","s"],check=True);s=subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip();b=subprocess.check_output(["git","-C",str(r),"rev-parse",f"{s}:a.txt"],text=True).strip();return r,p,s,b
class Exec(unittest.TestCase):
 def m(self,cr,p,s,b,defs):
  gs=[]
  for i,cl,body,code in defs:
   f=cr/(i+".sh");f.write_text(f"#!/bin/sh\n{body}\nexit {code}\n");f.chmod(0o755);gs.append({"id":i,"class":cl,"script":f.name,"sha256":hashlib.sha256(f.read_bytes()).hexdigest(),"args":[],"timeout_seconds":5})
  gs.sort(key=lambda g:g["id"]);return{"capsule_format_revision":1,"theorem_id":"TEST-001","theorem_revision":1,"repository_identity":"Luminous-Dynamics/mycelix","product_subject_sha":s,"predecessor_sha":p,"expected_changed_paths":["a.txt"],"expected_object_blobs":{"a.txt":b},"toolchain_profile_ref":pf("tool"),"environment_profile_ref":pf("env"),"gates":gs,"verdict":{"kind":"all","required_gate_ids":[g["id"]for g in gs]},"claim":"claim","nonclaims":[]}
 def c(self,m):return{"execution_context_format_revision":2,"runner_profile_ref":pf("runner"),"toolchain_profile_ref":m["toolchain_profile_ref"],"environment_profile_ref":m["environment_profile_ref"],"resolved_runner_commitment":q.runner_commitment(),"resolved_toolchain_commitment":"4"*64,"resolved_environment_commitment":"5"*64}
 def test_fail_continues_and_isolation(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.m(cr,p,s,b,(("a","theorem","echo leak > unexpected.txt",10),("b","oracle","test ! -e unexpected.txt",0)));x=q.run_capsule(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual([y["status"]for y in x["gate_results"]],["GateFail","GatePass"]);self.assertEqual(x["verdict"],"CompletedConjunctiveFail");self.assertFalse((r/"unexpected.txt").exists())
 def test_success_mutation_becomes_fail(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.m(cr,p,s,b,(("a","theorem","echo changed > a.txt",0),));x=q.run_capsule(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual(x["gate_results"][0]["status"],"GateFail");self.assertEqual((r/"a.txt").read_text(),"subject\n")
 def test_script_revalidation_and_not_run_suffix(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();marker=z/"marker";m=self.m(cr,p,s,b,(("a","theorem","echo x",0),("b","oracle",f"echo ran > {marker}",0),("c","oracle","echo c",0)));bs=cr/"b.sh";a=cr/"a.sh";a.write_text(f"#!/bin/sh\nprintf '#!/bin/sh\\nexit 0\\n' > {bs}\nexit 0\n");a.chmod(0o755);m["gates"][0]["sha256"]=hashlib.sha256(a.read_bytes()).hexdigest();x=q.run_capsule(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual([y["status"]for y in x["gate_results"]],["GatePass","RunnerInfrastructureFailure","GateNotRun"]);self.assertFalse(marker.exists())
 def test_runner_and_profile_mismatch_preflight(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();marker=z/"marker";m=self.m(cr,p,s,b,(("a","theorem",f"echo ran > {marker}",0),));c=self.c(m);bad=dict(c);bad["resolved_runner_commitment"]="6"*64;self.assertRaises(q.CapsuleError,q.run_capsule,m,cr,r,m["repository_identity"],"a1",bad);bad=self.c(m);bad["environment_profile_ref"]=pf("wrong");self.assertRaises(q.CapsuleError,q.run_capsule,m,cr,r,m["repository_identity"],"a1",bad);self.assertFalse(marker.exists())
 def test_repository_identity_and_origin(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.m(cr,p,s,b,(("a","theorem","true",0),));self.assertRaises(q.CapsuleError,q.preflight,m,r,"Wrong/Repo");subprocess.run(["git","-C",str(r),"remote","add","origin","https://github.com/Wrong/Repo.git"],check=True);self.assertRaises(q.CapsuleError,q.preflight,m,r,m["repository_identity"])
 def test_ambient_secret_closed(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();m=self.m(cr,p,s,b,(("a","theorem",'test -z "${QCAP_TEST_SECRET+x}" && test "$LANG" = C && test -n "$HOME" && test -n "$TMPDIR"',0),));old=os.environ.get("QCAP_TEST_SECRET");os.environ["QCAP_TEST_SECRET"]="secret"
   try:x=q.run_capsule(m,cr,r,m["repository_identity"],"a1",self.c(m))
   finally:
    if old is None:os.environ.pop("QCAP_TEST_SECRET",None)
    else:os.environ["QCAP_TEST_SECRET"]=old
   self.assertEqual(x["verdict"],"CompletedConjunctivePass")
 @unittest.skipUnless(os.name=="posix","POSIX")
 def test_timeout_kills_and_reaps_descendant(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/"c";cr.mkdir();pidf=z/"pid";m=self.m(cr,p,s,b,(("a","theorem",f"sleep 60 &\necho $! > {pidf}\nsleep 60",0),("b","oracle","true",0)));m["gates"][0]["timeout_seconds"]=1;x=q.run_capsule(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual([y["status"]for y in x["gate_results"]],["RunnerInfrastructureFailure","GateNotRun"]);pid=int(pidf.read_text());time.sleep(.1)
   with self.assertRaises(ProcessLookupError):os.kill(pid,0)
 def test_caller_checkout_untouched(self):
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);(r/"later").write_text("x");subprocess.run(["git","-C",str(r),"add","later"],check=True);subprocess.run(["git","-C",str(r),"commit","-q","-m","later"],check=True);later=subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip();cr=z/"c";cr.mkdir();m=self.m(cr,p,s,b,(("a","theorem","true",0),));q.run_capsule(m,cr,r,m["repository_identity"],"a1",self.c(m));self.assertEqual(subprocess.check_output(["git","-C",str(r),"rev-parse","HEAD"],text=True).strip(),later)
 def test_lineage_gate_is_theorem_red(self):
  gate=(H.parent/"gates"/"subject_identity.py").resolve()
  with tempfile.TemporaryDirectory()as d:
   z=Path(d);r,p,s,b=repo(z);e={"PATH":os.environ.get("PATH","/usr/bin:/bin"),"QCAP_SUBJECT_DIR":str(r),"QCAP_PRODUCT_SUBJECT_SHA":s,"QCAP_PREDECESSOR_SHA":"0"*40,"QCAP_EXPECTED_CHANGED_PATHS_JSON":'["a.txt"]',"QCAP_EXPECTED_OBJECT_BLOBS_JSON":"{}"};self.assertEqual(subprocess.run([str(gate)],env=e).returncode,10)
if __name__=="__main__":unittest.main()
