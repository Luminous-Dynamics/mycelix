import importlib.util,pathlib,unittest
P=pathlib.Path(__file__).with_name('liveness_receipt.py')
S=importlib.util.spec_from_file_location('lr',P);lr=importlib.util.module_from_spec(S);S.loader.exec_module(lr)
H='4fa9a8597152f404ec922afdabd113fd6d305e49';W='.github/workflows/amsap-004a-subject-topology.yml'
def m(j=None):return {'schema':lr.MANIFEST_SCHEMA,'repository':'Luminous-Dynamics/mycelix','theorem_id':'AMSAP-004A','qualification_head':H,'workflow_path':W,'required_jobs':j or [{'name':'qualify','required_gate_names':['gate'],'depends_on':[]}]}
def r(st='queued',co=None):return {'id':35443692878,'head_sha':H,'path':W,'status':st,'conclusion':co}
def q(st='queued',co=None,steps=None,rid=0,rn=''):return {'id':105898874637,'run_id':35443692878,'head_sha':H,'name':'qualify','status':st,'conclusion':co,'steps':[] if steps is None else steps,'runner_id':rid,'runner_name':rn}
def step(name='gate',st='completed',co='success',at='x'):return {'name':name,'status':st,'conclusion':co,'started_at':at}
def c(R=None,J=None,M=None):return lr.classify_liveness(R or r(),{'jobs':[J or q()]},M or m(),'2026-09-20T12:43:50Z')
class T(unittest.TestCase):
 def test_amsap_terminal_no_start(self):
  x=c(r('completed','cancelled'),q('completed','cancelled'));self.assertEqual(x['classification'],'TerminalCancelledNoStart');self.assertFalse(x['runner_assignment_observed']);self.assertIsNone(x['qualification_result'])
 def test_queued_no_start(self):self.assertEqual(c()['classification'],'AllRequiredJobsNoStart')
 def test_cancel_after_runner_before_theorem(self):self.assertEqual(c(r('completed','cancelled'),q('completed','cancelled',[],7,'runner'))['classification'],'TerminalCancelledBeforeTheorem')
 def test_unknown_theorem_start(self):self.assertEqual(c(r('completed','cancelled'),q('completed','cancelled',[step('setup')],7,'runner'))['classification'],'TerminalCancelledTheoremStartUnknown')
 def test_completed_pass_fail(self):
  self.assertEqual(c(r('completed','success'),q('completed','success',[step()],7,'runner'))['classification'],'CompletedConjunctivePass')
  self.assertEqual(c(r('completed','failure'),q('completed','failure',[step(co='failure')],7,'runner'))['classification'],'CompletedConjunctiveFail')
 def test_identity_mismatch(self):
  j=q();j['run_id']=1
  with self.assertRaises(lr.LivenessError):c(J=j)
 def test_manifest_cycle(self):
  M=m([{'name':'a','required_gate_names':['g'],'depends_on':['b']},{'name':'b','required_gate_names':['g'],'depends_on':['a']}])
  with self.assertRaises(lr.LivenessError):lr.manifest(M)
 def test_authority_ceiling(self):
  x=c();self.assertFalse(x['qualification_authority']);self.assertFalse(x['merge_authority']);self.assertFalse(x['deployment_authority']);self.assertIsNone(x['theorem_result'])
if __name__=='__main__':unittest.main()
