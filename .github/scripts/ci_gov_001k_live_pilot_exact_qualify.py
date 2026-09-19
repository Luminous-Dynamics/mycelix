#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, os, pathlib, subprocess, sys, tempfile
from typing import Any
SUBJECT='55b6de9f8e39ea9c639b890730f4f48b9889267c'
SUBJECT_TREE='d8fc9c0f8f03840a1114d02c24cd3881820b2d6e'
PARENT='bf07e3fa9701e947cb7ac5c03e43052661e8a155'
PARENT_TREE='e486d2d74d696b92577c52ab5d4e534ed2c36890'
SCHEMA='mycelix.ci-gov.001k.live-pilot-fixture-qualification.receipt.v0.1'
FIXTURE_SHA256='c1479745b168901f67931ff392b7c3e70ec4959a3841328943380719e9117cfe'
PATHS={
 '.github/scripts/CI_GOV_001K_LIVE_PILOT.lock.json':'067f8c056deb9e09a312cb1bf4b9cb6da6c5c44a',
 '.github/scripts/CI_GOV_001K_LIVE_PILOT.md':'a974e4efd1f47e4ecd109ee048448caa7161bd88',
 '.github/scripts/ci_gov_001k_live_pilot.yml.fixture':'f3b27460f724d4a2e16b2859e7d79d44a8084158',
 '.github/scripts/ci_gov_001k_pilot_fixture_verify.py':'024cde0be25f2f9e2ad4ad841f8c1595af05063d',
 '.github/scripts/test_ci_gov_001k_pilot_fixture_verify.py':'dfafbffbc8910ddca8fdbaa38764f1ffc727cb7b'}
EXEC_PATHS=(
 '.github/scripts/ci_gov_001k_live_pilot.yml.fixture',
 '.github/scripts/ci_gov_001k_pilot_fixture_verify.py',
 '.github/scripts/test_ci_gov_001k_pilot_fixture_verify.py')
QUALIFIER_PATHS={'.github/scripts/CI_GOV_001K_LIVE_PILOT_QUALIFICATION.md','.github/scripts/ci_gov_001k_live_pilot_exact_qualify.py','.github/scripts/test_ci_gov_001k_live_pilot_exact_qualify.py'}
EXPECTED_LOCK={
 'schema':'mycelix.ci-gov.001k.live-pilot-fixture-lock.v0.1',
 'parent_policy_commit':PARENT,'parent_policy_tree':PARENT_TREE,'tracking_issue':2067,
 'review_payload':{k:v for k,v in PATHS.items() if not k.endswith('lock.json')},
 'fixture_sha256':FIXTURE_SHA256,
 'future_workflow_path':'.github/workflows/ci-gov-001k-live-pilot.yml',
 'pilot_contract':{'trigger':'pull_request:labeled','label':'ci:qualify-pilot','draft_only':True,'group':'mycelix-heavy-qualification-v1','queue':'max','cancel_in_progress':False,'jobs':1,'runner':'ubuntu-24.04','timeout_minutes':5,'hold_seconds':90,'third_party_actions':0,'repository_mutation':False},
 'bounded_protocol':{'subjects':3,'admit_canary_first':True,'admit_followers_only_after_canary_in_progress':True,'max_new_runs_when_runner_unavailable':1,'natural_completion_required':True,'pilot_cleanup_cancellation':False,'fairness_claim':False},
 'evidence_states':['PASS','RUNNER_UNAVAILABLE','INCONCLUSIVE','FAIL'],
 'authority_ceiling':{'workflow_activation':False,'label_mutation':False,'actions_mutation':False,'cancellation_authority':False,'merge_authority':False,'product_pass_authority':False,'scientific_pass_authority':False},
 'local_verifier_preflight_cases':25}
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
 changed=[x for x in git('diff-tree','--no-commit-id','--name-only','-r',SUBJECT).splitlines() if x]
 if len(changed)!=5 or set(changed)!=set(PATHS):raise QualificationError('subject path set mismatch')
 for p,oid in PATHS.items():
  if git('rev-parse',f'{SUBJECT}:{p}')!=oid:raise QualificationError(f'blob mismatch: {p}')
 fixture=git('show',f'{SUBJECT}:.github/scripts/ci_gov_001k_live_pilot.yml.fixture',text=False)
 if sha256(fixture)!=FIXTURE_SHA256:raise QualificationError('fixture sha mismatch')
def verify_lock():
 raw=git('show',f'{SUBJECT}:.github/scripts/CI_GOV_001K_LIVE_PILOT.lock.json',text=False)
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
  cp=subprocess.run([sys.executable,'-E','-s','-S','-B','test_ci_gov_001k_pilot_fixture_verify.py'],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env={'PATH':os.environ.get('PATH',''),'PYTHONHASHSEED':'0'},check=False)
  if cp.returncode:raise QualificationError('fixture tests failed: '+cp.stderr.decode(errors='replace'))
  return {'returncode':0,'stdout_sha256':sha256(cp.stdout),'stderr_sha256':sha256(cp.stderr)}
def outside(path):
 if path is None:return
 root=pathlib.Path(git('rev-parse','--show-toplevel')).resolve();target=path.resolve()
 if target==root or root in target.parents:raise QualificationError('receipt must be outside checkout')
def qualify(path=None):
 qc,qt=verify_checkout();verify_subject();lock=verify_lock();tests=run_tests();verify_checkout()
 b={'schema':SCHEMA,'subject':SUBJECT,'subject_tree':SUBJECT_TREE,'parent':PARENT,'qualifier_commit':qc,'qualifier_tree':qt,'fixture_sha256':FIXTURE_SHA256,'source_lock_sha256':lock,'tests':tests,'proposition':'Exact CI-GOV-001K-D-A v0.1 inert live-pilot fixture identity and committed 25-case static verifier corpus passed.','grants_workflow_activation':False,'grants_live_scheduler_qualification':False,'grants_label_mutation':False,'grants_actions_mutation':False,'grants_cancellation_authority':False,'grants_product_pass':False,'grants_scientific_pass':False,'nonclaims':['No workflow was installed under .github/workflows.','No pilot run was created.','No live queue:max scheduler behavior is established.']};b['receipt_commitment']=sha256(canonical(b))
 if path is not None:outside(path);path.write_text(json.dumps(b,indent=2,sort_keys=True)+'\n')
 return b
def main():
 p=argparse.ArgumentParser();p.add_argument('--receipt-output',type=pathlib.Path);a=p.parse_args()
 try:r=qualify(a.receipt_output)
 except QualificationError as e:print(f'QUALIFICATION ERROR: {e}',file=sys.stderr);return 2
 print(json.dumps(r,indent=2,sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
