import pathlib,subprocess,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).parent));import spec,guard,product,execute
class F:isolated=1;ignore_environment=1;no_user_site=1;no_site=1;safe_path=True
class T(unittest.TestCase):
 def test_canon(self):self.assertEqual(guard.can({'b':1,'a':2}),b'{"a":2,"b":1}\n')
 def test_git_env(self):
  with self.assertRaises(guard.E):guard.reject_env({'GIT_DIR':'x'})
 def test_rust_env(self):
  with self.assertRaises(guard.E):guard.reject_env({'RUSTFLAGS':'-Cfoo'})
 def test_cargo_env(self):
  with self.assertRaises(guard.E):guard.reject_env({'CARGO_PROFILE_RELEASE_LTO':'1'})
 def test_python(self):guard.python_runtime(F(),True)
 def test_python_bad(self):
  with self.assertRaises(guard.E):guard.python_runtime(F(),False)
 def test_paths(self):
  with self.assertRaises(guard.E):guard.execution_paths(pathlib.Path('/tmp/r'),{'qualify.py':'/tmp/x'})
 def test_tool(self):self.assertEqual(execute.tool(f'release: {spec.RT}\ncommit-hash: {spec.RC}\n',f'release: {spec.CT}\ncommit-hash: {spec.CC}\n',spec,guard)['cargo_release'],spec.CT)
 def test_tool_bad(self):
  with self.assertRaises(guard.E):execute.tool(f'release: {spec.RT}\ncommit-hash: bad\n',f'release: {spec.CT}\ncommit-hash: {spec.CC}\n',spec,guard)
 def test_lock_authority(self):
  o=spec.lock_obj({});self.assertFalse(o['authority']['durable_atomic_accounting_established']);self.assertTrue(o['authority']['runtime_contract_text_qualified_only'])
 def test_product_paths(self):self.assertEqual(len(spec.PB),5)
 def test_contract_blob_bound(self):self.assertEqual(spec.lock_obj({})['runtime_contract_blob'],'128821eb0d7cfc2715560c6310f91d9bac1fee2e')
 def test_test_count(self):self.assertEqual(spec.TESTS,15)
 def test_qualifier_test_count(self):self.assertEqual(spec.QTESTS,18)
 def test_dependency_manifest(self):
  lock=b'[[package]]\nname="a"\nversion="1.0.0"\nsource="registry+x"\nchecksum="abc"\ndependencies=["b 2.0.0"]\n[[package]]\nname="b"\nversion="2.0.0"\n'
  meta='{"packages":[{"id":"a 1.0.0 (registry+x)","name":"a","version":"1.0.0","source":"registry+x"},{"id":"b 2.0.0 (path)","name":"b","version":"2.0.0","source":null}],"resolve":{"nodes":[{"id":"b 2.0.0 (path)","features":[]},{"id":"a 1.0.0 (registry+x)","features":["z","a"]}]}}'
  m=execute.dependency_manifest(lock,meta,guard);self.assertEqual(m['lock_packages'][0]['checksum'],'abc');self.assertEqual(m['resolved_features'][0]['features'],['a','z'])
 def test_dependency_manifest_rejects_unknown_node(self):
  with self.assertRaises(guard.E):execute.dependency_manifest(b'[[package]]\nname="a"\nversion="1"\n','{"packages":[],"resolve":{"nodes":[{"id":"x","features":[]}]}}',guard)
 def test_component_mapping(self):self.assertIn(('psi-abuse-control-profile','mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile'),spec.COMP)
 def test_launcher(self):
  q=subprocess.run([sys.executable,'-I','-S','-B',str(pathlib.Path(__file__).parent/'qualify.py'),'--help'],capture_output=True,text=True);self.assertEqual(q.returncode,0,q.stderr)
if __name__=='__main__':unittest.main()
