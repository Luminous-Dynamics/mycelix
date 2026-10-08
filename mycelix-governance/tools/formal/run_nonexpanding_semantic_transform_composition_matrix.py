from __future__ import annotations
import argparse,hashlib,json,re,subprocess
from pathlib import Path
IDS=["upstream-decision-identity-substitution","derivation-rule-substitution","source-amount-substitution","audience-widening","second-hop-profile-ceiling-widening","second-hop-effect-widening","chain-link-substitution","missing-monotonicity-declaration","monotonicity-violation","downstream-implementation-substitution"]
def run(cmd): return subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
def fsha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def alloy_rows(out):
 r={}
 for line in out.splitlines():
  if line.startswith('{') and '"label"' in line and '"actual"' in line:
   o=json.loads(line); r[o['label']]=o['actual']
 return r
def remove_fact(source,name):
 marker='fact '+name+' {'; start=source.find(marker)
 if start<0: raise RuntimeError('fact not found: '+name)
 brace=source.find('{',start); depth=0
 for i in range(brace,len(source)):
  if source[i]=='{': depth+=1
  elif source[i]=='}':
   depth-=1
   if depth==0: return source[:start]+source[i+1:]
 raise RuntimeError('unterminated fact: '+name)
p=argparse.ArgumentParser()
for n in ('matrix','tla','canonical-cfg','negative-tla','negative-cfg-dir','alloy','runner-class-dir','alloy-jar','tla-jar','reference','runtime','evidence-dir'): p.add_argument('--'+n,type=Path,required=True)
a=p.parse_args(); a.evidence_dir.mkdir(parents=True,exist_ok=True)
m=json.loads(a.matrix.read_text()); assert m['schema']=='mycelix.evidence-attestation-nonexpanding-semantic-transform-composition-control-matrix.v1'
controls=m['controls']; assert [c['id'] for c in controls]==IDS
head=run(['git','rev-parse','HEAD']); tree=run(['git','rev-parse','HEAD^{tree}'])
if head.returncode or tree.returncode: raise RuntimeError('head/tree resolution failed')
runtime=json.loads(a.runtime.read_text()); assert runtime['schema']=='mycelix.evidence-attestation-capability-formal-runtime.v1'
negative_cfgs=[a.negative_cfg_dir/('EvidenceAttestationNonExpandingSemanticTransformCompositionV1Negative-'+c['id']+'.cfg') for c in controls]
if any(not x.is_file() for x in negative_cfgs): raise RuntimeError('negative CFG missing')
inputs=[a.matrix,a.tla,a.canonical_cfg,a.negative_tla,a.alloy,a.reference,a.runtime]+negative_cfgs
receipt={'receipt_schema':'mycelix.evidence-attestation-nonexpanding-semantic-transform-composition-formal-receipt.v1','result':'ExecutedFail','repository':{'head':head.stdout.strip(),'tree':tree.stdout.strip()},'runtime':runtime,'inputs':{str(x):fsha(x) for x in inputs},'controls':IDS}
ref_run=run(['python3',str(a.reference)]); (a.evidence_dir/'reference.log').write_text(ref_run.stdout)
if ref_run.returncode or 'COMPOSITION PASS' not in ref_run.stdout: raise RuntimeError('reference oracle failed')
for c in controls:
 if ref_run.stdout.count(c['reference_marker']) != 1: raise RuntimeError('reference marker mismatch '+c['id'])
receipt['reference']={'returncode':0,'stdout_sha256':hashlib.sha256(ref_run.stdout.encode()).hexdigest(),'markers':{c['id']:c['reference_marker'] for c in controls}}
def tlc(cfg,module,label):
 md=a.evidence_dir/(label+'-metadir'); md.mkdir(parents=True,exist_ok=True)
 r=run(['java','-cp',str(a.tla_jar),'tlc2.TLC','-workers','1','-metadir',str(md),'-config',str(cfg),str(module)])
 (a.evidence_dir/(label+'.log')).write_text(r.stdout)
 if r.returncode==0 and 'Model checking completed. No error has been found.' in r.stdout: return set()
 v=set(re.findall(r'Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated(?: by the initial state)?',r.stdout))
 if v: return v
 raise RuntimeError('TLC execution failed without classified invariant violation: '+' | '.join(r.stdout.splitlines()[-12:]))
canon=tlc(a.canonical_cfg,a.tla,'tla-canonical')
if canon: raise RuntimeError('canonical TLA violations: '+repr(sorted(canon)))
receipt['tla']={'canonical':'PASS','negatives':{}}
for c in controls:
 v=tlc(a.negative_cfg_dir/('EvidenceAttestationNonExpandingSemanticTransformCompositionV1Negative-'+c['id']+'.cfg'),a.negative_tla,'tla-negative-'+c['id'])
 exp={c['tla_invariant']}
 if v!=exp: raise RuntimeError('TLA isolation mismatch '+c['id']+': '+repr(sorted(v)))
 receipt['tla']['negatives'][c['id']]=sorted(v)
cp=f'{a.runner_class_dir}:{a.alloy_jar}'
canon_run=run(['java','-cp',cp,'AgentDelegationAuthorityAlloyRunner',str(a.alloy)])
(a.evidence_dir/'alloy-canonical.log').write_text(canon_run.stdout)
if canon_run.returncode: raise RuntimeError('canonical Alloy runner failed')
cr=alloy_rows(canon_run.stdout)
for label in ('ScopeOrderReflexive','ScopeOrderTransitive','ScopeOrderAntisymmetric'):
 if cr.get(label)!='UNSAT': raise RuntimeError('order property failed '+label+': '+repr(cr))
if cr.get('CanonicalTwoHopTransformWitness')!='SAT' or cr.get('NonExpandingSemanticTransformCompositionExact')!='UNSAT': raise RuntimeError('canonical Alloy mismatch: '+repr(cr))
for c in controls:
 if cr.get(c['alloy_witness'])!='UNSAT': raise RuntimeError('canonical negative witness not UNSAT '+c['id'])
receipt['alloy']={'canonical':cr,'mutations':{}}; source=a.alloy.read_text()
for c in controls:
 mutant=a.evidence_dir/('alloy-negative-'+c['id']+'.als'); mutant.write_text(remove_fact(source,c['alloy_mutant_fact']))
 r=run(['java','-cp',cp,'AgentDelegationAuthorityAlloyRunner',str(mutant)]); (a.evidence_dir/('alloy-negative-'+c['id']+'.log')).write_text(r.stdout)
 if r.returncode: raise RuntimeError('Alloy mutant failed '+c['id'])
 rows=alloy_rows(r.stdout)
 if rows.get(c['alloy_witness'])!='SAT' or rows.get('NonExpandingSemanticTransformCompositionExact')!='SAT': raise RuntimeError('Alloy target mismatch '+c['id']+': '+repr(rows))
 for stable in ('ScopeOrderReflexive','ScopeOrderTransitive','ScopeOrderAntisymmetric'):
  if rows.get(stable)!='UNSAT': raise RuntimeError('order outcome changed '+c['id']+': '+stable)
 for other in controls:
  if other['id']!=c['id'] and rows.get(other['alloy_witness'])!='UNSAT': raise RuntimeError('unrelated Alloy witness became SAT for '+c['id']+': '+repr(rows))
 changed={k for k in set(cr)|set(rows) if cr.get(k)!=rows.get(k)}
 if changed!={c['alloy_witness'],'NonExpandingSemanticTransformCompositionExact'}: raise RuntimeError('unrelated Alloy outcomes '+c['id']+': '+repr(sorted(changed)))
 receipt['alloy']['mutations'][c['id']]={'removed_fact':c['alloy_mutant_fact'],'changed_outcomes':sorted(changed),'outcomes':rows}
receipt['result']='ExecutedPass'
(a.evidence_dir/'evidence-attestation-nonexpanding-semantic-transform-composition-formal-receipt-v1.json').write_text(json.dumps(receipt,indent=2,sort_keys=True)+'\n')
print(json.dumps(receipt,sort_keys=True))
