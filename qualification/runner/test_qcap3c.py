import os,tempfile,unittest
from pathlib import Path
from unittest import mock
import qcap3_exec as ex
import qcap3_isolation as iso
from test_qcap3 import CapsuleError,context,limits,manifest,q,repo

class Qcap3C(unittest.TestCase):
 def test_controlled_environment_is_closed_and_path_is_fixed(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits();m=manifest(cr,p,s,b,(("a","true",()),));c=context(m,l);parent,w=iso.add_worktree(r,s)
   try:
    scratch=parent/'scratch';scratch.mkdir();env=ex.controlled_env(m,c,l,'attempt-env',w,scratch);self.assertEqual(env['PATH'],os.pathsep.join([str(w/'qualification'/'runner'/'bin'),'/usr/bin','/bin']));self.assertEqual(env['LANG'],'C.UTF-8');self.assertEqual(env['LC_ALL'],'C.UTF-8');self.assertEqual(env['TZ'],'UTC');self.assertEqual(env['PYTHONDONTWRITEBYTECODE'],'1');self.assertEqual(env['PYTHONHASHSEED'],'0');self.assertNotIn('HOME',env);self.assertNotIn('TMPDIR',env);self.assertNotIn('CI',env);self.assertFalse(any(k.startswith('GITHUB_')for k in env));self.assertTrue(all(k.startswith('QCAP_')or k in{'PATH','LANG','LC_ALL','TZ','PYTHONDONTWRITEBYTECODE','PYTHONHASHSEED'}for k in env))
   finally:iso.remove_worktree(r,parent,w)
 def test_ambient_path_only_executable_is_not_resolved_and_repo_unchanged(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();bad=z/'bad';bad.mkdir();marker=z/'ambient-ran';helper=bad/'ambient-only';helper.write_text('#!/bin/sh\nprintf ran > "$1"\n');helper.chmod(0o755);l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a",'ambient-only "$1"',(str(marker),)),));before=iso.state(r)
   with mock.patch.dict(os.environ,{'PATH':str(bad)+os.pathsep+os.environ.get('PATH','')},clear=False):x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-path-miss',context(m,l),l)
   g=x['gate_results'][0];self.assertEqual(x['verdict'],'RunnerInfrastructureFailure');self.assertEqual(g['runner_failure_reason'],'UnexpectedExitCode');self.assertFalse(marker.exists());self.assertEqual(iso.state(r),before)
 def test_ambient_fake_git_cannot_preempt_fixed_system_git(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();bad=z/'bad';bad.mkdir();marker=z/'fake-git-ran';fake=bad/'git';fake.write_text(f'#!/bin/sh\nprintf fake > "{marker}"\nexit 99\n');fake.chmod(0o755);l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a",'git --version >/dev/null',()),))
   with mock.patch.dict(os.environ,{'PATH':str(bad)+os.pathsep+os.environ.get('PATH','')},clear=False):x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-path-git',context(m,l),l)
   self.assertEqual(x['verdict'],'CompletedConjunctivePass');self.assertFalse(marker.exists())
 def test_timeout_and_output_limit_leave_repository_state_unchanged(self):
  cases=(("timeout","sleep 5",256,"Timeout"),("output","while :; do printf 1234567890abcdef; done",32,"OutputLimitExceeded"))
  for name,body,cap,reason in cases:
   with self.subTest(name=name),tempfile.TemporaryDirectory() as d:
    z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=cap);m=manifest(cr,p,s,b,(("a",body,()),));m['gates'][0]['timeout_seconds']=1;before=iso.state(r);x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-'+name,context(m,l),l);self.assertEqual(x['gate_results'][0]['runner_failure_reason'],reason);self.assertEqual(iso.state(r),before)
 def test_cleanup_failure_suppresses_otherwise_passing_gate(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits();m=manifest(cr,p,s,b,(("a","true",()),));original=ex.remove_worktree
   def fail_after_cleanup(repo_path,parent,worktree):original(repo_path,parent,worktree);raise CapsuleError('simulated cleanup postcondition failure')
   with mock.patch.object(ex,'remove_worktree',fail_after_cleanup):x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-cleanup',context(m,l),l)
   g=x['gate_results'][0];self.assertEqual(x['verdict'],'RunnerInfrastructureFailure');self.assertEqual(g['runner_failure_reason'],'WorktreeCleanupFailure');self.assertEqual(g['effective_exit_code'],20)
 def test_cleanup_noop_or_race_fails_closed(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);parent,w=iso.add_worktree(r,s)
   try:
    with mock.patch.object(iso.shutil,'rmtree',return_value=None):
     with self.assertRaisesRegex(CapsuleError,'worktree cleanup failed'):iso.remove_worktree(r,parent,w)
   finally:
    if os.path.lexists(parent):iso.shutil.rmtree(parent,ignore_errors=True)
 def test_undeclared_verdict_gate_is_rejected_before_execution(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();marker=z/'ran';l=limits();m=manifest(cr,p,s,b,(("a",f'touch {marker}',()),));m['verdict']['required_gate_ids']=['undeclared']
   with self.assertRaises(CapsuleError):q.run_capsule(m,cr,r,m['repository_identity'],'attempt-undeclared',context(m,l),l)
   self.assertFalse(marker.exists())

if __name__=='__main__':unittest.main()
