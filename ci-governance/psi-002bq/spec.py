import json
PRODUCT='10ea945145066a6f79075a888840dd8266b53be1'
TREE='fbd3ad7dd534c8a68124d0670d1186b32a71f566'
PARENT='2be72da2acfd9903bfca168035c9ee087059f46f'
PARENT_TREE='d5f793cfb99e08ab3412ea7d3e144babbf52479e'
QX='ci-governance/psi-002bq'
PB={
'mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile/Cargo.toml':'a48992deed68aa0bfda8b2dc3c8a1f3cd6787429',
'mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile/README.md':'ac8b5380d13dda587d25d0a738330211835cb9f1',
'mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile/COMPOSITION.lock.json':'5c92fedbafa4c3d80eb6ce925875aa3d7c81834f',
'mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile/RUNTIME-CONTRACT.md':'128821eb0d7cfc2715560c6310f91d9bac1fee2e',
'mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile/src/lib.rs':'ee63b75be6e48cce77de5fd4838fff0012c796e6'}
COMP=[('privacy-computation-core','mycelix-workspace/mycelix-core/libs/privacy-computation-core'),('privacy-protocol-profiles','mycelix-workspace/mycelix-core/libs/privacy-protocol-profiles'),('psi-abuse-control-profile','mycelix-workspace/mycelix-core/libs/psi-abuse-control-profile')]
RT='1.96.0'; RC='ac68faa20c58cbccd01ee7208bf3b6e93a7d7f96'; CT='1.96.0'; CC='30a34c6821b57de0aaec83a901aca39f88f6778c'
TESTS=15; QTESTS=18
COMMANDS=[['cargo','fmt','--check','--all'],['cargo','generate-lockfile','--offline'],['cargo','test','--offline','--locked','--workspace'],['cargo','clippy','--offline','--locked','--workspace','--all-targets','--','-D','warnings'],['cargo','tree','--offline','--locked','-e','normal,dev'],['cargo','metadata','--offline','--locked','--format-version','1']]
FILES=['README.md','spec.py','guard.py','product.py','execute.py','lock.json','qualify.py','test_qualify.py']
PATHS=[f'{QX}/{x}' for x in FILES]
def lock_obj(blobs):
 return {'schema':'mycelix.psi.002bq.lock.v0.1','product':{'commit':PRODUCT,'tree':TREE,'parent':PARENT,'parent_tree':PARENT_TREE},'product_blobs':PB,'runtime_contract_blob':PB[next(k for k in PB if k.endswith('RUNTIME-CONTRACT.md'))],'components':[{'dest':d,'prefix':p} for d,p in COMP],'qualifier':{'paths':PATHS,'source_blobs':blobs,'execution':{'python':['python3','-I','-S','-B'],'launcher':f'{QX}/qualify.py','modules':[f'{QX}/{x}' for x in ['spec.py','guard.py','product.py','execute.py']]}},'toolchain':{'rustc_release':RT,'rustc_commit':RC,'cargo_release':CT,'cargo_commit':CC},'product_tests':TESTS,'qualifier_self_tests':QTESTS,'commands':COMMANDS,'authority':{'structural_profile_only':True,'runtime_contract_text_qualified_only':True,'durable_atomic_accounting_established':False,'revocation_race_free_established':False,'distributed_consistency_established':False,'enumeration_resistance_established':False,'sybil_resistance_established':False,'client_anonymity_established':False,'privacy_preserving_accounting_established':False,'production_admitted':False,'application_authority_granted':False}}
