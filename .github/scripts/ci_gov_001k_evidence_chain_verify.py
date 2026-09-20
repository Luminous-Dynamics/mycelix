#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, pathlib, re, sys
from typing import Any

SCHEMA='mycelix.ci-gov.001k.evidence-root.receipt.v0.1'
HEX40=re.compile(r'^[0-9a-f]{40}$')
HEX64=re.compile(r'^[0-9a-f]{64}$')

COMPONENTS={
 'A':{
  'schema':'mycelix.ci-gov.001k.source-qualification.receipt.v0.2',
  'subject':'bf07e3fa9701e947cb7ac5c03e43052661e8a155','subject_tree':'e486d2d74d696b92577c52ab5d4e534ed2c36890',
  'qualifier_commit':'2b0d268ad40ab46357a4d256c727cd3ae432cf76','qualifier_tree':'eb7d914c9df2a3217b28fd70d008e454d1c7585d',
  'extra_ids':{'base':'884a14e14758a91d3c1d370d49648946dc8b89ef','base_tree':'1a3190fef7ce9fc4ea9fba05aa20f3c277ec030f'},
  'proposition':'Exact CI-GOV-001K v0.2 source capsule identity and committed 28-case offline admission/budget oracle suite passed.',
  'source_lock_sha256':'55b83c77e91bbab9cdbcf7ee4089bbec0116c1a4cf7f37a3fec40bb6599f23e9',
  'nonclaims':['No GitHub concurrency behavior was exercised.','No capacity-observer correctness was established.','No Actions run was mutated.','No cancellation authority is granted.','No product or scientific PASS is granted.','No fairness/FIFO property is established.'],
  'false_grants':{'grants_live_scheduler_qualification','grants_observer_qualification','grants_cancellation_authority','grants_product_pass','grants_scientific_pass'},
  'fields':{'schema','subject','subject_tree','base','base_tree','qualifier_commit','qualifier_tree','source_lock_sha256','tests','proposition','nonclaims','grants_live_scheduler_qualification','grants_observer_qualification','grants_cancellation_authority','grants_product_pass','grants_scientific_pass','receipt_commitment'},
  'test_fields':{'command','returncode','stdout_sha256','stderr_sha256'},
  'test_command':['python','-E','-s','-S','-B','test_ci_qualification_capacity_oracle.py'],
 },
 'B':{
  'schema':'mycelix.ci-gov.001k.capacity-observer-qualification.receipt.v0.1',
  'subject':'d3de3a4d24c7459b80f939fee5c3c2bb4843ad71','subject_tree':'3ef12424861acf5a1ee63d216b71805561bed977',
  'qualifier_commit':'936633584dc99bc917d341788dbc9b7aabd01722','qualifier_tree':'26f071f4922b6a592bf277e19a0e02ea138b8488',
  'extra_ids':{'parent':'bf07e3fa9701e947cb7ac5c03e43052661e8a155'},
  'proposition':'Exact CI-GOV-001K-B v0.1 read-only capacity observer identity and committed 26-case corpus passed.',
  'source_lock_sha256':'7c29e73c525947d91025d80b9b1c37bc062b8f2bb1e236323c39ac5563d40bfc',
  'nonclaims':['No live API observation was qualified by this source test.','No queue admission authority is granted.','No scheduler behavior is established.'],
  'false_grants':{'grants_live_observation','grants_queue_admission','grants_cancellation_authority','grants_product_pass','grants_scientific_pass'},
  'fields':{'schema','subject','subject_tree','parent','qualifier_commit','qualifier_tree','source_lock_sha256','tests','proposition','grants_live_observation','grants_queue_admission','grants_cancellation_authority','grants_product_pass','grants_scientific_pass','nonclaims','receipt_commitment'},
  'test_fields':{'returncode','stdout_sha256','stderr_sha256'},
 },
 'C':{
  'schema':'mycelix.ci-gov.001k.admission-adviser-qualification.receipt.v0.1',
  'subject':'8ee69ad386e4189fb4b1d549dc95ea649c282c1d','subject_tree':'d44049d174976b2450e090a702f036a090acdb48',
  'qualifier_commit':'ba1e01ab3bb51f3b40aa2d8c88399f4d227eedec','qualifier_tree':'0e38c46cd331adc81beeb67fa979864b782699ce',
  'extra_ids':{'parent':'d3de3a4d24c7459b80f939fee5c3c2bb4843ad71','policy_source':'bf07e3fa9701e947cb7ac5c03e43052661e8a155'},
  'proposition':'Exact CI-GOV-001K-C v0.1 read-only admission adviser identity and committed 28-case corpus passed against canonical inherited A/B source bytes.',
  'source_lock_sha256':'52f8498909aa02ed7f27d123b6854ee0b3597232496f7f77d002df8450f7ab22',
  'nonclaims':['No live API observation was performed by this source qualifier.','No PR label was mutated.','No live queue:max scheduler behavior is established.'],
  'false_grants':{'grants_live_scheduler_qualification','grants_operational_label_mutation','grants_actions_mutation','grants_cancellation_authority','grants_product_pass','grants_scientific_pass'},
  'fields':{'schema','subject','subject_tree','parent','policy_source','qualifier_commit','qualifier_tree','source_lock_sha256','tests','proposition','grants_live_scheduler_qualification','grants_operational_label_mutation','grants_actions_mutation','grants_cancellation_authority','grants_product_pass','grants_scientific_pass','nonclaims','receipt_commitment'},
  'test_fields':{'returncode','stdout_sha256','stderr_sha256'},
 },
 'D':{
  'schema':'mycelix.ci-gov.001k.live-pilot-fixture-qualification.receipt.v0.1',
  'subject':'55b6de9f8e39ea9c639b890730f4f48b9889267c','subject_tree':'d8fc9c0f8f03840a1114d02c24cd3881820b2d6e',
  'qualifier_commit':'498fa8d487b9a7b8b65be4dcff9ab76cb24e2910','qualifier_tree':'bbcbd63187c3926c43208aabd73d6f9343268329',
  'extra_ids':{'parent':'bf07e3fa9701e947cb7ac5c03e43052661e8a155','fixture_sha256':'c1479745b168901f67931ff392b7c3e70ec4959a3841328943380719e9117cfe'},
  'proposition':'Exact CI-GOV-001K-D-A v0.1 inert live-pilot fixture identity and committed 25-case static verifier corpus passed.',
  'source_lock_sha256':'dea22ebd5e38002adba1da3ffbb25dfb0559b9ab8adc55d2d19d6d6bad86ec7d',
  'nonclaims':['No workflow was installed under .github/workflows.','No pilot run was created.','No live queue:max scheduler behavior is established.'],
  'false_grants':{'grants_workflow_activation','grants_live_scheduler_qualification','grants_label_mutation','grants_actions_mutation','grants_cancellation_authority','grants_product_pass','grants_scientific_pass'},
  'fields':{'schema','subject','subject_tree','parent','qualifier_commit','qualifier_tree','fixture_sha256','source_lock_sha256','tests','proposition','grants_workflow_activation','grants_live_scheduler_qualification','grants_label_mutation','grants_actions_mutation','grants_cancellation_authority','grants_product_pass','grants_scientific_pass','nonclaims','receipt_commitment'},
  'test_fields':{'returncode','stdout_sha256','stderr_sha256'},
 },
}
SCHEMA_TO_NAME={v['schema']:k for k,v in COMPONENTS.items()}

class ChainError(ValueError): pass

def canonical(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=True).encode()
def sha256(b:bytes)->str:return hashlib.sha256(b).hexdigest()
def _hex40(v:Any)->bool:return isinstance(v,str) and bool(HEX40.fullmatch(v))
def _hex64(v:Any)->bool:return isinstance(v,str) and bool(HEX64.fullmatch(v))

MAX_RECEIPT_BYTES=1_048_576

def _no_duplicate_pairs(pairs):
 out={}
 for k,v in pairs:
  if k in out:raise ChainError(f'duplicate JSON key: {k}')
  out[k]=v
 return out

def loads_strict(raw:bytes)->dict[str,Any]:
 if len(raw)>MAX_RECEIPT_BYTES:raise ChainError('receipt too large')
 try:text=raw.decode('utf-8')
 except UnicodeDecodeError as e:raise ChainError('receipt not UTF-8') from e
 try:v=json.loads(text,object_pairs_hook=_no_duplicate_pairs,parse_constant=lambda x:(_ for _ in ()).throw(ChainError(f'non-finite JSON constant: {x}')))
 except json.JSONDecodeError as e:raise ChainError('invalid JSON') from e
 if not isinstance(v,dict):raise ChainError('receipt JSON must be object')
 return v

def receipt_commitment(receipt:dict[str,Any])->str:
 x=dict(receipt);x.pop('receipt_commitment',None);return sha256(canonical(x))

def verify_component(name:str,r:dict[str,Any])->dict[str,Any]:
 e=COMPONENTS[name]
 if not isinstance(r,dict):raise ChainError(f'{name}: receipt must be object')
 if set(r)!=e['fields']:raise ChainError(f'{name}: top-level fields mismatch')
 if r['schema']!=e['schema']:raise ChainError(f'{name}: schema mismatch')
 for k in ('subject','subject_tree','qualifier_commit','qualifier_tree'):
  if r[k]!=e[k]:raise ChainError(f'{name}: {k} mismatch')
 for k,v in e['extra_ids'].items():
  if r.get(k)!=v:raise ChainError(f'{name}: {k} mismatch')
 if r['source_lock_sha256']!=e['source_lock_sha256']:raise ChainError(f'{name}: source lock digest mismatch')
 if r['proposition']!=e['proposition']:raise ChainError(f'{name}: proposition drift')
 if r['nonclaims']!=e['nonclaims']:raise ChainError(f'{name}: nonclaims drift')
 for field in e['false_grants']:
  if r[field] is not False:raise ChainError(f'{name}: authority widened: {field}')
 t=r['tests']
 if not isinstance(t,dict) or set(t)!=e['test_fields']:raise ChainError(f'{name}: test fields mismatch')
 if type(t['returncode']) is not int or t['returncode']!=0:raise ChainError(f'{name}: tests did not pass')
 if not _hex64(t['stdout_sha256']) or not _hex64(t['stderr_sha256']):raise ChainError(f'{name}: invalid test digest')
 if 'test_command' in e and t['command']!=e['test_command']:raise ChainError(f'{name}: test command drift')
 if not _hex64(r['receipt_commitment']) or r['receipt_commitment']!=receipt_commitment(r):raise ChainError(f'{name}: receipt commitment mismatch')
 return {'schema':r['schema'],'subject':r['subject'],'subject_tree':r['subject_tree'],'qualifier_commit':r['qualifier_commit'],'qualifier_tree':r['qualifier_tree'],'source_lock_sha256':r['source_lock_sha256'],'receipt_commitment':r['receipt_commitment'],'tests':{'stdout_sha256':t['stdout_sha256'],'stderr_sha256':t['stderr_sha256']}}

def verify_chain(receipts:list[dict[str,Any]],file_sha256s:dict[str,str]|None=None)->dict[str,Any]:
 if len(receipts)!=4:raise ChainError('exactly four receipts required')
 by_name={}
 for r in receipts:
  if not isinstance(r,dict) or r.get('schema') not in SCHEMA_TO_NAME:raise ChainError('unknown receipt schema')
  n=SCHEMA_TO_NAME[r['schema']]
  if n in by_name:raise ChainError(f'duplicate component {n}')
  by_name[n]=r
 if set(by_name)!=set(COMPONENTS):raise ChainError('A/B/C/D receipt set incomplete')
 verified={n:verify_component(n,by_name[n]) for n in 'ABCD'}
 if file_sha256s is not None:
  if set(file_sha256s)!=set('ABCD') or any(not _hex64(x) for x in file_sha256s.values()):raise ChainError('receipt file digest map invalid')
  for n in 'ABCD':verified[n]['receipt_file_sha256']=file_sha256s[n]
 # Explicit lineage checks, even though component constants already bind them.
 if by_name['B']['parent']!=by_name['A']['subject']:raise ChainError('B parent != A subject')
 if by_name['C']['parent']!=by_name['B']['subject']:raise ChainError('C parent != B subject')
 if by_name['C']['policy_source']!=by_name['A']['subject']:raise ChainError('C policy source != A subject')
 if by_name['D']['parent']!=by_name['A']['subject']:raise ChainError('D parent != A subject')
 root={'schema':SCHEMA,'components':[{'name':n,**verified[n]} for n in 'ABCD'],
       'component_receipt_commitments':[verified[n]['receipt_commitment'] for n in 'ABCD'],
       'proposition':'Exact CI-GOV-001K A/B/C/D qualification receipts form one canonical fail-closed evidence chain.',
       'grants_workflow_activation':False,'grants_label_mutation':False,'grants_actions_mutation':False,
       'grants_live_scheduler_qualification':False,'grants_cancellation_authority':False,'grants_merge_authority':False,
       'grants_product_pass':False,'grants_scientific_pass':False,
       'nonclaims':['Component receipts must come from real qualifier executions; this verifier does not execute A/B/C/D qualifiers.','No live GitHub scheduler behavior is established.','No operational mutation authority is granted.']}
 root['evidence_root_commitment']=sha256(canonical(root))
 return root

def main()->int:
 p=argparse.ArgumentParser();p.add_argument('receipts',nargs=4,type=pathlib.Path);p.add_argument('--output',type=pathlib.Path);a=p.parse_args()
 try:
  raws=[x.read_bytes() for x in a.receipts]
  receipts=[loads_strict(x) for x in raws]
  names=[]
  for r in receipts:
   if r.get('schema') not in SCHEMA_TO_NAME:raise ChainError('unknown receipt schema')
   names.append(SCHEMA_TO_NAME[r['schema']])
  digests={n:sha256(raw) for n,raw in zip(names,raws)}
  if len(digests)!=4:raise ChainError('duplicate component input')
  out=verify_chain(receipts,digests)
 except (OSError,json.JSONDecodeError,ChainError) as e:
  print(f'EVIDENCE CHAIN ERROR: {e}',file=sys.stderr);return 2
 text=json.dumps(out,indent=2,sort_keys=True)+'\n'
 if a.output:a.output.write_text(text)
 print(text,end='');return 0
if __name__=='__main__':raise SystemExit(main())
