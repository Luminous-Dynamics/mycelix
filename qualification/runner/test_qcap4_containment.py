import os,subprocess,sys,tempfile,unittest
from pathlib import Path
from unittest import mock
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H))
import qcap4_containment as c

PROFILE={
 "containment_profile_format_revision":1,"profile_id":"containment:linux-cgroup-v2-kill-v1","profile_revision":1,
 "platform":"linux","primitive":"cgroup-v2","membership_start":"before-gate-exec",
 "termination_policy":"cgroup-kill","empty_postcondition":"cgroup-events-populated-zero",
 "descendant_scope":"cgroup-membership-preserving","session_process_group_escape_covered":True,
 "cleanup_ceiling_ms":1000,"hostile_code_sandbox":False,
}
EXPECTED_DIGEST="ae8d5f03e79c2838ba5483102f9997e0c583dc7f2ada75176e9edfe451cd3b23"

class Qcap4Containment(unittest.TestCase):
 def test_profile_digest_matches_frozen_a3d1_r2(self):
  self.assertEqual(c.containment_digest(PROFILE),EXPECTED_DIGEST)
  self.assertEqual(c.containment_ref(PROFILE)["digest"],EXPECTED_DIGEST)
 def test_profile_rejects_hostile_sandbox_overclaim(self):
  p=dict(PROFILE);p["hostile_code_sandbox"]=True
  with self.assertRaises(c.CapsuleError):c.validate_containment_profile(p)
 def test_events_parser_is_closed_enough_for_populated_law(self):
  self.assertEqual(c.parse_cgroup_events("populated 0\nfrozen 1\n")["populated"],0)
  for bad in ("","populated 2\n","populated x\n","populated 0\npopulated 1\n"):
   with self.subTest(bad=bad):
    with self.assertRaises(c.ContainmentError):c.parse_cgroup_events(bad)
 def test_gate_name_is_deterministic_and_domain_separates_fields(self):
  a=c._gate_name("attempt-a","gate")
  self.assertEqual(a,c._gate_name("attempt-a","gate"));self.assertNotEqual(a,c._gate_name("attempt","a-gate"))
  self.assertTrue(a.startswith("qcap-"));self.assertEqual(len(a),37)
 def test_resolved_commitment_binds_root_identity(self):
  with tempfile.TemporaryDirectory() as d:
   a=Path(d)/"a";b=Path(d)/"b";a.mkdir();b.mkdir()
   self.assertNotEqual(c.resolved_containment_commitment(PROFILE,a),c.resolved_containment_commitment(PROFILE,b))
 def test_symlink_root_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);real=z/"real";real.mkdir();link=z/"link";link.symlink_to(real,target_is_directory=True)
   with self.assertRaises(c.ContainmentError):c.resolved_containment_commitment(PROFILE,link)
 def test_wait_empty_obeys_bounded_transition(self):
  with mock.patch.object(c,"cgroup_populated",side_effect=[True,True,False])as p,mock.patch.object(c.time,"sleep"):
   c.wait_empty("/unused",1000);self.assertEqual(p.call_count,3)
 def test_wait_empty_timeout_is_containment_failure(self):
  ticks=iter([0.0,0.02,0.06])
  with mock.patch.object(c,"cgroup_populated",return_value=True),mock.patch.object(c.time,"monotonic",side_effect=lambda:next(ticks)),mock.patch.object(c.time,"sleep"):
   with self.assertRaisesRegex(c.ContainmentError,"remained populated"):c.wait_empty("/unused",50)
 def test_terminate_order_is_kill_empty_cleanup(self):
  order=[]
  with mock.patch.object(c,"kill_cgroup",side_effect=lambda p:order.append("kill")),mock.patch.object(c,"wait_empty",side_effect=lambda p,m:order.append("empty")),mock.patch.object(c,"remove_empty_tree",side_effect=lambda p:order.append("remove")):
   c.terminate_and_cleanup("/unused",1000)
  self.assertEqual(order,["kill","empty","remove"])
 def test_wrapper_establishes_membership_before_exec(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);cg=z/"cg";cg.mkdir();procs=cg/"cgroup.procs";procs.write_text("")
   gate=z/"gate.py";marker=z/"marker"
   gate.write_text("#!/usr/bin/env python3\nfrom pathlib import Path\nimport os,sys\nms={int(x) for x in Path(sys.argv[2]).read_text().split() if x.isdigit()}\nPath(sys.argv[1]).write_text('member' if os.getpid() in ms else 'missing')\n")
   gate.chmod(0o755)
   r=subprocess.run([sys.executable,str(H/"qcap4_gate_wrapper.py"),str(cg),str(gate),str(marker),str(procs)],capture_output=True,text=True)
   self.assertEqual(r.returncode,0,r.stdout+r.stderr);self.assertEqual(marker.read_text(),"member")
 def test_wrapper_failure_never_executes_gate(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);cg=z/"cg";cg.mkdir();marker=z/"ran";gate=z/"gate.sh"
   gate.write_text("#!/bin/sh\nprintf ran > \"$1\"\n");gate.chmod(0o755)
   r=subprocess.run([sys.executable,str(H/"qcap4_gate_wrapper.py"),str(cg),str(gate),str(marker)],capture_output=True,text=True)
   self.assertEqual(r.returncode,20);self.assertFalse(marker.exists());self.assertIn("containment primitive missing cgroup.procs",r.stderr)

if __name__=="__main__":unittest.main()
