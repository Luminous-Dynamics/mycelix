#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, os, pathlib, subprocess, sys, tempfile
from typing import Any
SUBJECT='8ee69ad386e4189fb4b1d549dc95ea649c282c1d'; SUBJECT_TREE='d44049d174976b2450e090a702f036a090acdb48'
PARENT='d3de3a4d24c7459b80f939fee5c3c2bb4843ad71'; PARENT_TREE='3ef12424861acf5a1ee63d216b71805561bed977'
POLICY='bf07e3fa9701e947cb7ac5c03e43052661e8a155'; POLICY_TREE='e486d2d74d696b92577c52ab5d4e534ed2c36890'
SCHEMA='mycelix.ci-gov.001k.admission-adviser-qualification.receipt.v0.1'
PATHS={
 '.github/scripts/CI_GOV_001K_ADMISSION_ADVISER.lock.json':'397e88b241c99e5cd56ecf191f46637b866a01fb',
 '.github/scripts/CI_GOV_001K_ADMISSION_ADVISER.md':'35dbe2df25f4ac25d1a8c1fe65a7df73437b9853',
 '.github/scripts/ci_qualification_admission_adviser.py':'5efc958db919147607f6fdda05fdbaf30ddd3ba4',
 '.github/scripts/test_ci_qualification_admission_adviser.py':'337bdab4bd8cc39afaf08fb9a28d393960e0fb93'}
INHERITED={
 '.github/scripts/ci_qualification_capacity_oracle.py':'1b2b2f4862c910a2c013432172d74b5213c42c41',
 '.github/scripts/CI_GOV_001K.lock.json':'44caaaa35ed882c5f6c4d411e6ffc7a3f65777a6',
 '.github/scripts/ci_qualification_capacity_observer.py':'7af073d89f1b9db5be87b3c84fa07341a5adb722',
 '.github/scripts/CI_GOV_001K_OBSERVER.lock.json':'04323683ae394748836ec75e2692c73063259085'}
EXEC_PATHS=(
 '.github/scripts/ci_qualification_capacity_oracle.py',
 '.github/scripts/ci_qualification_capacity_observer.py',
 '.github/scripts/ci_qualification_admission_adviser.py',
 '.github/scripts/test_ci_qualification_admission_adviser.py')
QUALIFIER_PATHS={'.github/scripts/CI_GOV_001K_ADMISSION_ADVISER_QUALIFICATION.md','.github/scripts/ci_gov_001k_adviser_exact_qualify.py','.github/scripts/test_ci_gov_001k_adviser_exact_qualify.py'}
EXPECTED_LOCK={
 'schema':'mycelix.ci-gov.001k.admission-adviser-lock.v0.1','parent_observer_commit':PARENT,'parent_observer_tree':PARENT_TREE,
 'policy_source':{'commit':POLICY,'tree':POLICY_TREE,'oracle_blob':INHERITED['.github/scripts/ci_qualification_capacity_oracle.py'],'policy_lock_blob':INHERITED['.github/scripts/CI_GOV_001K.lock.json']},
 'observer_source':{'commit':PARENT,'tree':PARENT_TREE,'observer_blob':INHERITED['.github/scripts/ci_qualification_capacity_observer.py'],'observer_lock_blob':INHERITED['.github/scripts/CI_GOV_001K_OBSERVER.lock.json']},
 'review_payload':{k:v for k,v in PATHS.items() if not k.endswith('lock.json')},
 'read_contract':{'max_reads':3,'pr_endpoint_template':'https://api.github.com/repos/Luminous-Dynamics/mycelix/pulls/{pr_number}','capacity_endpoint':'https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/concurrency_groups/mycelix-heavy-qualification-v1','api_version':'2026-03-10','max_body_bytes':1048576,'max_timeout_seconds':10},
 'decision_contract':{'explicit_label':'ci:qualify','max_observation_age_seconds':30,'soft_pending_budget':8,'pre_post_pr_snapshot_required':True,'live_scheduler_qualification_required':True},
 'authority_ceiling':{'github_read':True,'label_mutation':False,'actions_mutation':False,'cancellation_authority':False,'merge_authority':False,'product_pass_authority':False,'scientific_pass_authority':False},
 'local_logic_preflight_cases':28}
class QualificationError(RuntimeError):pass
def canonical(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode()
def sha256(b:bytes)->str:return hashlib.sha256(b).hexdigest()
def git_env():
 for k in ('GIT_DIR','GIT_WORK_TREE','GIT_OBJECT_DIRECTORY','GIT_ALTERNATE_OBJECT_DIRECTORIES','GIT_COMMON_DIR','GIT_INDEX_FILE'):
  if os.environ.get(k):raise QualificationError(f'forbidden git redirect env: {k}')
 e=os.environ.copy();e['GIT_NO_REPLACE_OBJECTS']='1';return e
def git(*args,text=True):
 cp=subprocess.run(['git',*args],stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=git_env(),check=False)
 if cp.returncode:raise QualificationError(cp.stderr.decode(errors='replace').strip())
 return cp.stdout.decode().strip() if text else cp.stdout
def reject_rewrite():
 root=pathlib.Path(git('rev-parse','--show-toplevel'));gd=pathlib.Path(git('rev-parse','--git-dir'))
 if not gd.is_absolute():gd=(root/gd).resolve()
 if (gd/'info'/'grafts').exists():raise QualificationError('grafts file present')
 if git('for-each-ref','--format=%(refname)','refs/replace').strip():raise QualificationError('replace refs present')
def verify_subject():
 reject_rewrite()
 if git('rev-parse',f'{SUBJECT}^{{tree}}')!=SUBJECT_TREE:raise QualificationError('subject tree mismatch')
 if git('rev-parse',f'{SUBJECT}^')!=PARENT:raise QualificationError('subject parent mismatch')
 if git('rev-parse',f'{PARENT}^{{tree}}')!=PARENT_TREE:raise QualificationError('parent tree mismatch')
 if git('rev-parse',f'{POLICY}^{{tree}}')!=POLICY_TREE:raise QualificationError('policy tree mismatch')
 changed=[x for x in git('diff-tree','--no-commit-id','--name-only','-r',SUBJECT).splitlines() if x]
 if len(changed)!=4 or set(changed)!=set(PATHS):raise QualificationError('subject path set mismatch')
 for p,oid in {**PATHS,**INHERITED}.items():
  if git('rev-parse',f'{SUBJECT}:{p}')!=oid:raise QualificationError(f'blob mismatch: {p}')
def verify_lock():
 raw=git('show',f'{SUBJECT}:.github/scripts/CI_GOV_001K_ADMISSION_ADVISER.lock.json',text=False)
 try:v=json.loads(raw)
 except json.JSONDecodeError as e:raise QualificationError('invalid lock JSON') from e
 if v!=EXPECTED_LOCK:raise QualificationError('lock mismatch')
 return sha256(raw)
def verify_checkout():
 head=git('rev-parse','HEAD')
 if git('rev-parse','HEAD^')!=SUBJECT:raise QualificationError('qualifier parent mismatch')
 changed=[x for x in git('diff-tree','--no-commit-id','--name-only','-r','HEAD').splitlines() if x]
 if len(changed)!=3 or set(changed)!=QUALIFIER_PATHS:raise QualificationError('qualifier path set mismatch')
 if git('status','--porcelain').strip():raise QualificationError('dirty qualifier')
 root=pathlib.Path(git('rev-parse','--show-toplevel'))
 for p in QUALIFIER_PATHS:
  if (root/p).read_bytes()!=git('show',f'HEAD:{p}',text=False):raise QualificationError(f'working byte mismatch: {p}')
 return head,git('rev-parse','HEAD^{tree}')
def run_tests():
 with tempfile.TemporaryDirectory() as td:
  root=pathlib.Path(td)
  for p in EXEC_PATHS:(root/pathlib.Path(p).name).write_bytes(git('show',f'{SUBJECT}:{p}',text=False))
  cp=subprocess.run([sys.executable,'-E','-s','-S','-B','test_ci_qualification_admission_adviser.py'],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env={'PATH':os.environ.get('PATH',''),'PYTHONHASHSEED':'0'},check=False)
  if cp.returncode:raise QualificationError('adviser tests failed: '+cp.stderr.decode(errors='replace'))
  return {'returncode':0,'stdout_sha256':sha256(cp.stdout),'stderr_sha256':sha256(cp.stderr)}
def outside(path):
 if path is None:return
 root=pathlib.Path(git('rev-parse','--show-toplevel')).resolve();target=path.resolve()
 if target==root or root in target.parents:raise QualificationError('receipt must be outside checkout')
def qualify(path=None):
 qc,qt=verify_checkout();verify_subject();lock=verify_lock();tests=run_tests();verify_checkout()
 b={'schema':SCHEMA,'subject':SUBJECT,'subject_tree':SUBJECT_TREE,'parent':PARENT,'policy_source':POLICY,'qualifier_commit':qc,'qualifier_tree':qt,'source_lock_sha256':lock,'tests':tests,'proposition':'Exact CI-GOV-001K-C v0.1 read-only admission adviser identity and committed 28-case corpus passed against canonical inherited A/B source bytes.','grants_live_scheduler_qualification':False,'grants_operational_label_mutation':False,'grants_actions_mutation':False,'grants_cancellation_authority':False,'grants_product_pass':False,'grants_scientific_pass':False,'nonclaims':['No live API observation was performed by this source qualifier.','No PR label was mutated.','No live queue:max scheduler behavior is established.']};b['receipt_commitment']=sha256(canonical(b))
 if path is not None:outside(path);path.write_text(json.dumps(b,indent=2,sort_keys=True)+'\n')
 return b
def main():
 p=argparse.ArgumentParser();p.add_argument('--receipt-output',type=pathlib.Path);a=p.parse_args()
 try:r=qualify(a.receipt_output)
 except QualificationError as e:print(f'QUALIFICATION ERROR: {e}',file=sys.stderr);return 2
 print(json.dumps(r,indent=2,sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
