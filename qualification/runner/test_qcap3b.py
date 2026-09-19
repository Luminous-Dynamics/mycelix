import hashlib,shutil,subprocess,sys,tempfile,unittest
from pathlib import Path
from test_qcap3 import H,CapsuleError,context,limits,manifest,q,repo

class Qcap3B(unittest.TestCase):
 def copy_runner(self,d):
  rd=Path(d)/'runner';rd.mkdir()
  for name in q.RUNNER_FILES:shutil.copyfile(H/name,rd/name)
  return rd
 def write_limits(self,rd):
  import json
  p=rd/'limits.json';p.write_text(json.dumps(limits()));return p
 def test_gate_executes_private_snapshot_path(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();marker=z/'argv0';l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a",'printf %s "$0" > "$1"',(str(marker),)),));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-snapshot',context(m,l),l);self.assertEqual(x['verdict'],'CompletedConjunctivePass');observed=marker.read_text();self.assertNotEqual(Path(observed),cr/'a.sh');self.assertIn('gate-snapshots',observed)
 def test_current_gate_canonical_mutation_cannot_change_snapshot_execution(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();marker=z/'done';target=cr/'a.sh';body='printf "#!/bin/sh\\nexit 99\\n" > "$1"; printf done > "$2"';l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a",body,(str(target),str(marker))),));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-self-mutate',context(m,l),l);self.assertEqual(x['verdict'],'CompletedConjunctivePass');self.assertEqual(marker.read_text(),'done');self.assertIn('exit 99',target.read_text())
 def test_future_gate_mutation_fails_before_future_execution(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();marker=z/'b-ran';l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a",'printf "#!/bin/sh\\nexit 0\\n" > "$1"',(str(cr/'b.sh'),)),("b",'touch "$1"',(str(marker),))));x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-future-mutate',context(m,l),l);self.assertEqual(x['gate_results'][0]['status'],'GatePass');self.assertEqual(x['gate_results'][1]['status'],'RunnerInfrastructureFailure');self.assertEqual(x['gate_results'][1]['runner_failure_reason'],'ArtifactIntegrityFailure');self.assertFalse(marker.exists())
 def test_symlink_gate_artifact_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a","true",()),));a=cr/'a.sh';data=a.read_bytes();a.unlink();target=cr/'target.sh';target.write_bytes(data);target.chmod(0o755);a.symlink_to(target.name);x=q.run_capsule(m,cr,r,m['repository_identity'],'attempt-symlink',context(m,l),l);g=x['gate_results'][0];self.assertEqual(g['status'],'RunnerInfrastructureFailure');self.assertEqual(g['runner_failure_reason'],'ArtifactIntegrityFailure')
 def test_reference_cli_requires_isolated_no_bytecode_mode(self):
  with tempfile.TemporaryDirectory() as d:
   rd=self.copy_runner(d);lp=self.write_limits(rd);bad=subprocess.run([sys.executable,str(rd/'qcap3.py'),'limits',str(lp)],capture_output=True,text=True);self.assertEqual(bad.returncode,20);self.assertIn('requires python -I -B',bad.stderr);good=subprocess.run([sys.executable,'-I','-B',str(rd/'qcap3.py'),'limits',str(lp)],capture_output=True,text=True);self.assertEqual(good.returncode,0,good.stdout+good.stderr);self.assertIn('execution_limits_digest=',good.stdout)
 def test_reference_cli_rejects_stdlib_shadow(self):
  with tempfile.TemporaryDirectory() as d:
   rd=self.copy_runner(d);lp=self.write_limits(rd);(rd/'hashlib.py').write_text('raise RuntimeError("shadow")\n');x=subprocess.run([sys.executable,'-I','-B',str(rd/'qcap3.py'),'limits',str(lp)],capture_output=True,text=True);self.assertEqual(x.returncode,20);self.assertIn('stdlib shadow forbidden: hashlib.py',x.stderr)
 def test_reference_cli_rejects_bytecode_cache(self):
  with tempfile.TemporaryDirectory() as d:
   rd=self.copy_runner(d);lp=self.write_limits(rd);pc=rd/'__pycache__';pc.mkdir();(pc/'qcap3_exec.cpython-999.pyc').write_bytes(b'not-bytecode');x=subprocess.run([sys.executable,'-I','-B',str(rd/'qcap3.py'),'limits',str(lp)],capture_output=True,text=True);self.assertEqual(x.returncode,20);self.assertIn('bytecode cache forbidden:',x.stderr)
 def test_runner_source_mutation_suppresses_receipt(self):
  original=(H/'qcap3_exec.py').read_bytes()
  try:
   with tempfile.TemporaryDirectory() as d:
    z=Path(d);r,p,s,b=repo(z);cr=z/'c';cr.mkdir();l=limits(max_gate_output_bytes=256);m=manifest(cr,p,s,b,(("a",'printf "# mutated during attempt\\n" > "$1"',(str(H/'qcap3_exec.py'),)),));c=context(m,l)
    with self.assertRaisesRegex(CapsuleError,'runner self-integrity changed during attempt'):q.run_capsule(m,cr,r,m['repository_identity'],'attempt-runner-mutate',c,l)
  finally:(H/'qcap3_exec.py').write_bytes(original)

if __name__=='__main__':unittest.main()
