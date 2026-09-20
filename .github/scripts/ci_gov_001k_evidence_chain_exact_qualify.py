#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, os, pathlib, subprocess, sys, tempfile
from typing import Any

SUBJECT='1b0d00342faea04fa09bae05931af7ced21f69dd'
SUBJECT_TREE='36d962547c61056600dba1e2e3f0a76b8d5a2a41'
PARENT='bf07e3fa9701e947cb7ac5c03e43052661e8a155'
PARENT_TREE='e486d2d74d696b92577c52ab5d4e534ed2c36890'
SCHEMA='mycelix.ci-gov.001k.evidence-chain-qualification.receipt.v0.1'
PATHS={
 '.github/scripts/CI_GOV_001K_EVIDENCE_CHAIN.lock.json':'4878a3e661fe8292a28bef7ede3c56881cc9014d',
 '.github/scripts/CI_GOV_001K_EVIDENCE_CHAIN.md':'77db0b2a7330d22fbaa95970885b93f9b7e7893f',
 '.github/scripts/ci_gov_001k_evidence_chain_verify.py':'a64c4d66aa227ec639a3b3332f34694ccc877c22',
 '.github/scripts/test_ci_gov_001k_evidence_chain_verify.py':'ff008bbd7259a069ba969734c5395d4e23c50676',
}
EXEC_PATHS=(
 '.github/scripts/ci_gov_001k_evidence_chain_verify.py',
 '.github/scripts/test_ci_gov_001k_evidence_chain_verify.py',
)
QUALIFIER_PATHS={
 '.github/scripts/CI_GOV_001K_EVIDENCE_CHAIN_QUALIFICATION.md',
 '.github/scripts/ci_gov_001k_evidence_chain_exact_qualify.py',
 '.github/scripts/test_ci_gov_001k_evidence_chain_exact_qualify.py',
}
EXPECTED_LOCK={
 'schema':'mycelix.ci-gov.001k.evidence-chain-lock.v0.1',
 'parent_policy_commit':PARENT,
 'parent_policy_tree':PARENT_TREE,
 'tracking_issue':2117,
 'review_payload':{
  '.github/scripts/CI_GOV_001K_EVIDENCE_CHAIN.md':PATHS['.github/scripts/CI_GOV_001K_EVIDENCE_CHAIN.md'],
  '.github/scripts/ci_gov_001k_evidence_chain_verify.py':PATHS['.github/scripts/ci_gov_001k_evidence_chain_verify.py'],
  '.github/scripts/test_ci_gov_001k_evidence_chain_verify.py':PATHS['.github/scripts/test_ci_gov_001k_evidence_chain_verify.py'],
 },
 'components':{
  'A':{'schema':'mycelix.ci-gov.001k.source-qualification.receipt.v0.2','subject':'bf07e3fa9701e947cb7ac5c03e43052661e8a155','subject_tree':'e486d2d74d696b92577c52ab5d4e534ed2c36890','qualifier_commit':'2b0d268ad40ab46357a4d256c727cd3ae432cf76','qualifier_tree':'eb7d914c9df2a3217b28fd70d008e454d1c7585d','source_lock_sha256':'55b83c77e91bbab9cdbcf7ee4089bbec0116c1a4cf7f37a3fec40bb6599f23e9'},
  'B':{'schema':'mycelix.ci-gov.001k.capacity-observer-qualification.receipt.v0.1','subject':'d3de3a4d24c7459b80f939fee5c3c2bb4843ad71','subject_tree':'3ef12424861acf5a1ee63d216b71805561bed977','qualifier_commit':'936633584dc99bc917d341788dbc9b7aabd01722','qualifier_tree':'26f071f4922b6a592bf277e19a0e02ea138b8488','source_lock_sha256':'7c29e73c525947d91025d80b9b1c37bc062b8f2bb1e236323c39ac5563d40bfc'},
  'C':{'schema':'mycelix.ci-gov.001k.admission-adviser-qualification.receipt.v0.1','subject':'8ee69ad386e4189fb4b1d549dc95ea649c282c1d','subject_tree':'d44049d174976b2450e090a702f036a090acdb48','qualifier_commit':'ba1e01ab3bb51f3b40aa2d8c88399f4d227eedec','qualifier_tree':'0e38c46cd331adc81beeb67fa979864b782699ce','source_lock_sha256':'52f8498909aa02ed7f27d123b6854ee0b3597232496f7f77d002df8450f7ab22'},
  'D':{'schema':'mycelix.ci-gov.001k.live-pilot-fixture-qualification.receipt.v0.1','subject':'55b6de9f8e39ea9c639b890730f4f48b9889267c','subject_tree':'d8fc9c0f8f03840a1114d02c24cd3881820b2d6e','qualifier_commit':'498fa8d487b9a7b8b65be4dcff9ab76cb24e2910','qualifier_tree':'bbcbd63187c3926c43208aabd73d6f9343268329','source_lock_sha256':'dea22ebd5e38002adba1da3ffbb25dfb0559b9ab8adc55d2d19d6d6bad86ec7d','fixture_sha256':'c1479745b168901f67931ff392b7c3e70ec4959a3841328943380719e9117cfe'},
 },
 'parser_contract':{'max_receipt_bytes':1048576,'utf8_only':True,'reject_duplicate_json_keys':True,'reject_nonfinite_numbers':True,'reject_unknown_or_extra_fields':True},
 'aggregate_contract':{'schema':'mycelix.ci-gov.001k.evidence-root.receipt.v0.1','canonical_component_order':['A','B','C','D'],'bind_component_receipt_commitments':True,'bind_raw_receipt_file_sha256':True},
 'authority_ceiling':{'workflow_activation':False,'label_mutation':False,'actions_mutation':False,'live_scheduler_qualification':False,'cancellation_authority':False,'merge_authority':False,'product_pass_authority':False,'scientific_pass_authority':False},
 'local_verifier_preflight_cases':34,
}

class QualificationError(RuntimeError): pass

def canonical(v:Any)->bytes:
 return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode()
def sha256(b:bytes)->str: return hashlib.sha256(b).hexdigest()

def git_env()->dict[str,str]:
 for k in ('GIT_DIR','GIT_WORK_TREE','GIT_OBJECT_DIRECTORY','GIT_ALTERNATE_OBJECT_DIRECTORIES','GIT_COMMON_DIR','GIT_INDEX_FILE'):
  if os.environ.get(k): raise QualificationError(f'forbidden git redirect env: {k}')
 e=os.environ.copy();e['GIT_NO_REPLACE_OBJECTS']='1';return e

def git(*args:str,text:bool=True):
 cp=subprocess.run(['git',*args],stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=git_env(),check=False)
 if cp.returncode: raise QualificationError(cp.stderr.decode(errors='replace').strip())
 return cp.stdout.decode().strip() if text else cp.stdout

def reject_rewrite()->None:
 root=pathlib.Path(git('rev-parse','--show-toplevel'));gd=pathlib.Path(git('rev-parse','--git-dir'))
 if not gd.is_absolute(): gd=(root/gd).resolve()
 if (gd/'info'/'grafts').exists(): raise QualificationError('grafts file present')
 if git('for-each-ref','--format=%(refname)','refs/replace').strip(): raise QualificationError('replace refs present')

def verify_subject()->None:
 reject_rewrite()
 if git('rev-parse',f'{SUBJECT}^{{tree}}')!=SUBJECT_TREE: raise QualificationError('subject tree mismatch')
 if git('rev-parse',f'{SUBJECT}^')!=PARENT: raise QualificationError('subject parent mismatch')
 if git('rev-parse',f'{PARENT}^{{tree}}')!=PARENT_TREE: raise QualificationError('parent tree mismatch')
 changed=[x for x in git('diff-tree','--no-commit-id','--name-only','-r',SUBJECT).splitlines() if x]
 if len(changed)!=4 or set(changed)!=set(PATHS): raise QualificationError('subject path set mismatch')
 for p,oid in PATHS.items():
  if git('rev-parse',f'{SUBJECT}:{p}')!=oid: raise QualificationError(f'blob mismatch: {p}')

def verify_lock()->str:
 raw=git('show',f'{SUBJECT}:.github/scripts/CI_GOV_001K_EVIDENCE_CHAIN.lock.json',text=False)
 try:v=json.loads(raw)
 except json.JSONDecodeError as e: raise QualificationError('invalid lock JSON') from e
 if v!=EXPECTED_LOCK: raise QualificationError('lock mismatch')
 return sha256(raw)

def verify_checkout()->tuple[str,str]:
 head=git('rev-parse','HEAD')
 if git('rev-parse','HEAD^')!=SUBJECT: raise QualificationError('qualifier parent mismatch')
 changed=[x for x in git('diff-tree','--no-commit-id','--name-only','-r','HEAD').splitlines() if x]
 if len(changed)!=3 or set(changed)!=QUALIFIER_PATHS: raise QualificationError('qualifier path set mismatch')
 if git('status','--porcelain').strip(): raise QualificationError('dirty qualifier')
 root=pathlib.Path(git('rev-parse','--show-toplevel'))
 for p in QUALIFIER_PATHS:
  if (root/p).read_bytes()!=git('show',f'HEAD:{p}',text=False): raise QualificationError(f'working byte mismatch: {p}')
 return head,git('rev-parse','HEAD^{tree}')

def source_audit()->None:
 text=git('show',f'{SUBJECT}:.github/scripts/ci_gov_001k_evidence_chain_verify.py')
 for forbidden in ('urllib.request','requests.','urlopen(','api.github.com','cancel_run(','rerun_workflow','merge_pull_request','label_pr('):
  if forbidden in text: raise QualificationError(f'forbidden source capability: {forbidden}')

def run_tests()->dict[str,Any]:
 with tempfile.TemporaryDirectory() as td:
  root=pathlib.Path(td)
  for p in EXEC_PATHS:(root/pathlib.Path(p).name).write_bytes(git('show',f'{SUBJECT}:{p}',text=False))
  cp=subprocess.run([sys.executable,'-E','-s','-S','-B','test_ci_gov_001k_evidence_chain_verify.py'],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env={'PATH':os.environ.get('PATH',''),'PYTHONHASHSEED':'0'},check=False)
  if cp.returncode: raise QualificationError('evidence-chain tests failed: '+cp.stderr.decode(errors='replace'))
  return {'command':['python','-E','-s','-S','-B','test_ci_gov_001k_evidence_chain_verify.py'],'returncode':0,'stdout_sha256':sha256(cp.stdout),'stderr_sha256':sha256(cp.stderr)}

def outside(path:pathlib.Path|None)->None:
 if path is None:return
 root=pathlib.Path(git('rev-parse','--show-toplevel')).resolve();target=path.resolve()
 if target==root or root in target.parents: raise QualificationError('receipt must be outside checkout')

def qualify(path:pathlib.Path|None=None)->dict[str,Any]:
 qc,qt=verify_checkout();verify_subject();lock=verify_lock();source_audit();tests=run_tests();verify_checkout()
 body={'schema':SCHEMA,'subject':SUBJECT,'subject_tree':SUBJECT_TREE,'parent':PARENT,'qualifier_commit':qc,'qualifier_tree':qt,'source_lock_sha256':lock,'tests':tests,
       'proposition':'Exact CI-GOV-001K-E v0.1 aggregate evidence-chain verifier identity and committed 34-case fail-closed corpus passed.',
       'grants_component_execution':False,'grants_workflow_activation':False,'grants_label_mutation':False,'grants_actions_mutation':False,'grants_live_scheduler_qualification':False,'grants_cancellation_authority':False,'grants_merge_authority':False,'grants_product_pass':False,'grants_scientific_pass':False,
       'nonclaims':['This source qualifier does not execute A/B/C/D qualifiers.','Synthetic component receipts used by unit tests are not canonical qualification evidence.','No live GitHub scheduler behavior is established.']}
 body['receipt_commitment']=sha256(canonical(body))
 if path is not None: outside(path);path.write_text(json.dumps(body,indent=2,sort_keys=True)+'\n')
 return body

def main()->int:
 p=argparse.ArgumentParser();p.add_argument('--receipt-output',type=pathlib.Path);a=p.parse_args()
 try:r=qualify(a.receipt_output)
 except QualificationError as e:print(f'QUALIFICATION ERROR: {e}',file=sys.stderr);return 2
 print(json.dumps(r,indent=2,sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
