import pathlib,subprocess,sys,tempfile,unittest
sys.path.insert(0,str(pathlib.Path(__file__).parent));import spec,guard,model,execute
class F:isolated=1;ignore_environment=1;no_user_site=1;no_site=1;safe_path=True
class T(unittest.TestCase):
 def ml(self,bad=False):
  return guard.can({"authority_ceiling":{"crash_recovery_measured":False,"distributed_consistency_measured":False,"enumeration_resistance_established":False,"production_admitted":False,"runtime_atomicity_measured":False,"sybil_resistance_established":False},"parent":{"psi_002a_commit":"2be72da2acfd9903bfca168035c9ee087059f46f","psi_002b_commit":spec.PP,"psi_002b_runtime_contract_blob":"128821eb0d7cfc2715560c6310f91d9bac1fee2e"},"profile":"psi-admission-reference-model-v1","schema":"mycelix.psi.002c0.model-lock.v0.1","semantics":{"failed_transition_state_unchanged":True,"release_idempotent":True,"released_commitment_remains_consumed":True,"successful_admission_consumes_budget_and_commitment_together":True},"runtime_capabilities":{"database":bad,"filesystem":False,"network":False,"randomness":False,"threads":False,"wall_clock":False}})
 def test_canon(self):self.assertEqual(guard.can({"b":1,"a":2}),b'{"a":2,"b":1}\n')
 def test_git_env(self):
  with self.assertRaises(guard.E):guard.reject_env({"GIT_DIR":"x"})
 def test_rust_env(self):
  with self.assertRaises(guard.E):guard.reject_env({"RUSTC_WRAPPER":"x"})
 def test_cargo_env(self):
  with self.assertRaises(guard.E):guard.reject_env({"CARGO_PROFILE_RELEASE_LTO":"1"})
 def test_python(self):guard.python_runtime(F(),True)
 def test_python_bad(self):
  with self.assertRaises(guard.E):guard.python_runtime(F(),False)
 def test_paths(self):
  with tempfile.TemporaryDirectory() as z:
   r=pathlib.Path(z);(r/spec.QX).mkdir(parents=True)
   with self.assertRaises(guard.E):guard.execution_paths(r,{"qualify.py":r/"x"})
 def test_model_lock(self):model.model_lock(self.ml())
 def test_model_lock_widen(self):
  with self.assertRaises(guard.E):model.model_lock(self.ml(True))
 def test_tool(self):self.assertEqual(execute.tool(f"release: {spec.RT}\ncommit-hash: {spec.RC}\n",f"release: {spec.CT}\ncommit-hash: {spec.CC}\n")["rustc_release"],spec.RT)
 def test_tool_bad(self):
  with self.assertRaises(guard.E):execute.tool(f"release: {spec.RT}\ncommit-hash: bad\n",f"release: {spec.CT}\ncommit-hash: {spec.CC}\n")
 def test_lock_claim(self):self.assertFalse(spec.lock_obj({})["authority"]["durable_atomicity_measured"])
 def test_component_mapping(self):self.assertIn(("psi-abuse-control-profiles",spec.X+"psi-abuse-control-profile"),spec.C)
 def test_product_blob_count(self):self.assertEqual(len(spec.PB),4)
 def test_launcher(self):
  z=subprocess.run([sys.executable,"-I","-S","-B",str(pathlib.Path(__file__).parent/"qualify.py"),"--help"],capture_output=True,text=True);self.assertEqual(z.returncode,0,z.stderr)
if __name__=="__main__":unittest.main()
