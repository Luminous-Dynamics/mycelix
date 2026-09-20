import copy, importlib.util, pathlib, sys, unittest
HERE=pathlib.Path('/tmp')
S=importlib.util.spec_from_file_location('e',str(HERE/'evidence_chain.py'));e=importlib.util.module_from_spec(S);sys.modules[S.name]=e;S.loader.exec_module(e)

def receipt(name):
 x=e.COMPONENTS[name]
 r={'schema':x['schema'],'subject':x['subject'],'subject_tree':x['subject_tree'],'qualifier_commit':x['qualifier_commit'],'qualifier_tree':x['qualifier_tree'],
    'source_lock_sha256':x['source_lock_sha256'],'tests':{'returncode':0,'stdout_sha256':'2'*64,'stderr_sha256':'3'*64},'proposition':x['proposition'],'nonclaims':list(x['nonclaims'])}
 r.update(x['extra_ids'])
 for f in x['false_grants']:r[f]=False
 if 'test_command' in x:r['tests']['command']=x['test_command']
 r['receipt_commitment']=e.receipt_commitment(r);return r

def chain():return [receipt(n) for n in 'ABCD']
class T(unittest.TestCase):
 def test_valid(self):self.assertEqual(len(e.verify_chain(chain())['components']),4)
 def test_order_independent_inputs_canonical_output(self):self.assertEqual(e.verify_chain(chain())['evidence_root_commitment'],e.verify_chain(list(reversed(chain())))['evidence_root_commitment'])
 def test_missing(self):
  with self.assertRaises(e.ChainError):e.verify_chain(chain()[:3])
 def test_extra(self):
  with self.assertRaises(e.ChainError):e.verify_chain(chain()+[receipt('A')])
 def test_duplicate_schema(self):
  c=chain();c[-1]=receipt('A')
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_unknown_schema(self):
  c=chain();c[0]['schema']='x'
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_extra_field(self):
  c=chain();c[0]['x']=1;c[0]['receipt_commitment']=e.receipt_commitment(c[0])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_missing_field(self):
  c=chain();del c[0]['nonclaims'];c[0]['receipt_commitment']=e.receipt_commitment(c[0])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_bad_commitment(self):
  c=chain();c[0]['receipt_commitment']='0'*64
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_subject_drift(self):
  c=chain();c[0]['subject']='0'*40;c[0]['receipt_commitment']=e.receipt_commitment(c[0])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_qualifier_drift(self):
  c=chain();c[1]['qualifier_commit']='0'*40;c[1]['receipt_commitment']=e.receipt_commitment(c[1])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_tree_drift(self):
  c=chain();c[2]['qualifier_tree']='0'*40;c[2]['receipt_commitment']=e.receipt_commitment(c[2])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_proposition_drift(self):
  c=chain();c[3]['proposition']='better';c[3]['receipt_commitment']=e.receipt_commitment(c[3])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_authority_widening(self):
  c=chain();c[2]['grants_actions_mutation']=True;c[2]['receipt_commitment']=e.receipt_commitment(c[2])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_failed_tests(self):
  c=chain();c[1]['tests']['returncode']=1;c[1]['receipt_commitment']=e.receipt_commitment(c[1])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_bad_test_digest(self):
  c=chain();c[1]['tests']['stdout_sha256']='x';c[1]['receipt_commitment']=e.receipt_commitment(c[1])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_test_extra_field(self):
  c=chain();c[1]['tests']['command']=[];c[1]['receipt_commitment']=e.receipt_commitment(c[1])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_a_command_drift(self):
  c=chain();c[0]['tests']['command']=['python'];c[0]['receipt_commitment']=e.receipt_commitment(c[0])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_bad_source_lock_digest(self):
  c=chain();c[0]['source_lock_sha256']='x';c[0]['receipt_commitment']=e.receipt_commitment(c[0])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_lineage_b(self):
  c=chain();c[1]['parent']='0'*40;c[1]['receipt_commitment']=e.receipt_commitment(c[1])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_lineage_c_parent(self):
  c=chain();c[2]['parent']='0'*40;c[2]['receipt_commitment']=e.receipt_commitment(c[2])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_lineage_c_policy(self):
  c=chain();c[2]['policy_source']='0'*40;c[2]['receipt_commitment']=e.receipt_commitment(c[2])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_lineage_d(self):
  c=chain();c[3]['parent']='0'*40;c[3]['receipt_commitment']=e.receipt_commitment(c[3])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_fixture_digest_drift(self):
  c=chain();c[3]['fixture_sha256']='0'*64;c[3]['receipt_commitment']=e.receipt_commitment(c[3])
  with self.assertRaises(e.ChainError):e.verify_chain(c)

 def test_lock_digest_drift(self):
  c=chain();c[0]['source_lock_sha256']='0'*64;c[0]['receipt_commitment']=e.receipt_commitment(c[0])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_nonclaims_drift(self):
  c=chain();c[1]['nonclaims']=['different'];c[1]['receipt_commitment']=e.receipt_commitment(c[1])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_bool_returncode_rejected(self):
  c=chain();c[2]['tests']['returncode']=False;c[2]['receipt_commitment']=e.receipt_commitment(c[2])
  with self.assertRaises(e.ChainError):e.verify_chain(c)
 def test_duplicate_json_key_rejected(self):
  with self.assertRaises(e.ChainError):e.loads_strict(b'{"schema":"a","schema":"b"}')
 def test_nonfinite_json_rejected(self):
  with self.assertRaises(e.ChainError):e.loads_strict(b'{"x":NaN}')
 def test_oversized_receipt_rejected(self):
  with self.assertRaises(e.ChainError):e.loads_strict(b'x'*(e.MAX_RECEIPT_BYTES+1))
 def test_non_utf8_rejected(self):
  with self.assertRaises(e.ChainError):e.loads_strict(b'\xff')
 def test_file_hashes_bound(self):
  fs={n:str(i+1)*64 for i,n in enumerate('ABCD')}
  r=e.verify_chain(chain(),fs);self.assertEqual([x['receipt_file_sha256'] for x in r['components']],[fs[n] for n in 'ABCD'])
 def test_root_authorities_false(self):
  r=e.verify_chain(chain());self.assertFalse(any(r[k] for k in r if k.startswith('grants_')))
 def test_root_commitment_sensitive(self):
  a=e.verify_chain(chain());c=chain();c[0]['tests']['stdout_sha256']='4'*64;c[0]['receipt_commitment']=e.receipt_commitment(c[0]);b=e.verify_chain(c);self.assertNotEqual(a['evidence_root_commitment'],b['evidence_root_commitment'])
if __name__=='__main__':unittest.main()
