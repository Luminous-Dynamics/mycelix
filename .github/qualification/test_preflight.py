#!/usr/bin/env python3
from __future__ import annotations
import contextlib, importlib.util, json, os, shutil, subprocess, sys, tempfile, unittest
from pathlib import Path

S=importlib.util.spec_from_file_location('preflight',Path(__file__).with_name('preflight.py')); assert S and S.loader
p=importlib.util.module_from_spec(S); S.loader.exec_module(p)

def g(r,*a): return subprocess.run(['git','-C',str(r),*a],check=True,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE).stdout.strip()

class T(unittest.TestCase):
 def repo(self):
  t=tempfile.TemporaryDirectory(); r=Path(t.name)/'r'; r.mkdir(); g(r,'init','-q'); g(r,'config','user.email','t@e'); g(r,'config','user.name','T'); g(r,'remote','add','origin','https://github.com/Luminous-Dynamics/mycelix.git')
  (r/'c/src').mkdir(parents=True); (r/'c/Cargo.toml').write_text("[package]\nname='x'\nversion='0.1.0'\n"); (r/'c/src/lib.rs').write_text('pub fn a() {}\n'); (r/'.gitignore').write_text('*.tmp\n'); g(r,'add','.'); g(r,'commit','-qm','base'); parent=g(r,'rev-parse','HEAD')
  (r/'c/src/lib.rs').write_text('pub fn b() {}\n'); g(r,'add','.'); g(r,'commit','-qm','subject'); return t,r,parent,g(r,'rev-parse','HEAD')
 def profile(self,r,parent,**kw):
  d={'schema':p.SCHEMA,'profile_id':'t-v1','repository':'Luminous-Dynamics/mycelix','required_parent_sha':parent,'expected_changed_paths':['c/src/lib.rs'],'toolchain':'1.98.1','manifest_path':'c/Cargo.toml','probe_timeout_seconds':10,'rustfmt_timeout_seconds':10}; d.update(kw); q=r.parent/'p.json'; q.write_text(json.dumps(d)); return q
 @contextlib.contextmanager
 def tools(self,r,mode='pass',present=True):
  b=r.parent/'bin'; b.mkdir(exist_ok=True); git=subprocess.run(['which','git'],check=True,text=True,stdout=subprocess.PIPE).stdout.strip(); gl=b/'git'; gl.exists() or gl.symlink_to(git); ru=b/'rustup'
  if present:
   ru.write_text(f'''#!{sys.executable}\nimport os,sys\nfrom pathlib import Path\nM={mode!r}; a=sys.argv[1:]\nif any(k in os.environ for k in ('CARGO','RUSTFMT','CARGO_ALIAS_FMT','RUSTUP_TOOLCHAIN','GIT_CONFIG_COUNT')): raise SystemExit(70)\nif a==['toolchain','list']: print('stable-x86_64-unknown-linux-gnu' if M=='missing' else '1.98.1-x86_64-unknown-linux-gnu'); raise SystemExit(0)\nif a==['run','1.98.1','cargo','--version']: print('cargo 1.97.0' if M=='wrongcargo' else 'cargo 1.98.1'); raise SystemExit(9 if M=='cargoerr' else 0)\nif a==['run','1.98.1','rustfmt','--version']: print('rustfmt fake'); raise SystemExit(8 if M=='fmterr' else 0)\nif len(a)==8 and a[:5]==['run','1.98.1','cargo','fmt','--manifest-path'] and a[6:]==['--','--check']:\n m=Path(a[5]); s=m.parent\n if not m.is_absolute() or Path.cwd()==s or s in Path.cwd().parents or os.environ.get('CARGO_NET_OFFLINE')!='true': raise SystemExit(71)\n if M=='fail': raise SystemExit(7)\n if M=='mut': (s/'src/lib.rs').write_text('x\\n')\n if M=='ignored': (s/'x.tmp').write_text('x\\n')\n raise SystemExit(0)\nraise SystemExit(64)\n'''); ru.chmod(0o755)
  elif ru.exists(): ru.unlink()
  old=os.environ.copy(); os.environ.update(PATH=str(b),CARGO='/tmp/evil-cargo',RUSTFMT='evil',CARGO_ALIAS_FMT='evil',RUSTUP_TOOLCHAIN='nightly',GIT_CONFIG_COUNT='0')
  try: yield
  finally: os.environ.clear(); os.environ.update(old)
 def test_eligible_and_preserves_caller(self):
  t,r,parent,s=self.repo(); (r/'dirty').write_text('x'); h=g(r,'rev-parse','HEAD'); st=subprocess.run(['git','-C',str(r),'status','--porcelain=v1','-z','--untracked-files=all','--ignored=matching'],check=True,stdout=subprocess.PIPE).stdout
  with self.tools(r): z=p.evaluate(r,self.profile(r,parent),s)
  self.assertEqual(z['classification'],'ELIGIBLE'); self.assertIsNone(z['qualification_result']); self.assertFalse(z['qualification_authority']); self.assertRegex(z['preflight_implementation_commitment'],r'^[0-9a-f]{64}$'); self.assertEqual(h,g(r,'rev-parse','HEAD')); self.assertEqual(st,subprocess.run(['git','-C',str(r),'status','--porcelain=v1','-z','--untracked-files=all','--ignored=matching'],check=True,stdout=subprocess.PIPE).stdout); t.cleanup()
 def test_failure_and_mutation(self):
  for mode in ('fail','mut','ignored'):
   with self.subTest(mode=mode):
    t,r,parent,s=self.repo()
    with self.tools(r,mode): z=p.evaluate(r,self.profile(r,parent),s)
    self.assertEqual(z['classification'],'NOT_ELIGIBLE'); t.cleanup()
 def test_unavailable_states(self):
  for mode,present in [('missing',True),('cargoerr',True),('wrongcargo',True),('fmterr',True),('pass',False)]:
   with self.subTest(mode=mode):
    t,r,parent,s=self.repo()
    with self.tools(r,mode,present): z=p.evaluate(r,self.profile(r,parent),s)
    self.assertEqual(z['classification'],'UNAVAILABLE'); t.cleanup()
 def test_wrong_parent_and_paths_invalid(self):
  t,r,parent,s=self.repo()
  with self.tools(r), self.assertRaises(p.PreflightError): p.evaluate(r,self.profile(r,'0'*40),s)
  with self.tools(r), self.assertRaises(p.PreflightError): p.evaluate(r,self.profile(r,parent,expected_changed_paths=['x']),s)
  t.cleanup()
 def test_branch_movement_keeps_subject(self):
  t,r,parent,s=self.repo(); q=self.profile(r,parent); (r/'later').write_text('x'); g(r,'add','.'); g(r,'commit','-qm','later'); later=g(r,'rev-parse','HEAD')
  with self.tools(r): z=p.evaluate(r,q,s)
  self.assertEqual(z['classification'],'ELIGIBLE'); self.assertEqual(z['subject_sha'],s); self.assertEqual(g(r,'rev-parse','HEAD'),later); t.cleanup()
 def test_profile_commitment_canonical(self):
  t,r,parent,s=self.repo(); q=self.profile(r,parent); d=json.loads(q.read_text()); _,a=p._load_profile(q); q.write_text(json.dumps(dict(reversed(list(d.items()))))); _,b=p._load_profile(q); self.assertEqual(a,b); t.cleanup()
 def test_profile_is_closed(self):
  t,r,parent,s=self.repo()
  for extra in ({'qualification_pass':True},{'probe_argv':['/bin/sh']}, {'toolchain':'+nightly'}, {'manifest_path':'../Cargo.toml'}, {'expected_changed_paths':['z','a']}, {'expected_changed_paths':['c/src/lib.rs','c/src/lib.rs']}):
   with self.subTest(extra=extra):
    with self.assertRaises(p.PreflightError): p._load_profile(self.profile(r,parent,**extra))
  t.cleanup()
 def test_duplicate_json_key_rejected(self):
  t,r,parent,s=self.repo(); q=self.profile(r,parent); x=q.read_text().replace('"profile_id": "t-v1"','"profile_id": "t-v1", "profile_id": "shadow"',1); q.write_text(x)
  with self.assertRaises(p.PreflightError): p._load_profile(q)
  t.cleanup()
 def test_clone_has_no_alternates(self):
  t,r,parent,s=self.repo()
  with tempfile.TemporaryDirectory() as d:
   dst=Path(d)/'x'; p.clone_isolated(r,dst,s); self.assertFalse((dst/'.git/objects/info/alternates').exists()); self.assertEqual(g(dst,'rev-parse','HEAD'),s)
  t.cleanup()
 def test_implementation_commitment_is_frozen_at_import(self):
  import hashlib
  with tempfile.TemporaryDirectory() as d:
   src=Path(p.__file__); dst=Path(d)/'preflight_copy.py'; shutil.copy2(src,dst)
   spec=importlib.util.spec_from_file_location('preflight_copy',dst); assert spec and spec.loader
   mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); before=mod.implementation_commitment()
   dst.write_text(dst.read_text()+'\n# changed after import\n')
   self.assertEqual(before,mod.implementation_commitment())
   self.assertNotEqual(before,hashlib.sha256(mod.IMPL_DOMAIN+dst.read_bytes()).hexdigest())

if __name__=='__main__': unittest.main()
