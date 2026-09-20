import pathlib,tomllib

def verify(repo,spec,g):
 if g.git(repo,'rev-parse',f'{spec.PRODUCT}^{{tree}}')!=spec.TREE:raise g.E('product tree')
 if g.git(repo,'rev-parse',f'{spec.PRODUCT}^')!=spec.PARENT:raise g.E('product parent')
 if g.git(repo,'rev-parse',f'{spec.PARENT}^{{tree}}')!=spec.PARENT_TREE:raise g.E('parent tree')
 paths=tuple(sorted(g.git(repo,'diff','--name-only',spec.PARENT,spec.PRODUCT).splitlines()))
 if paths!=tuple(sorted(spec.PB)):raise g.E('product path set')
 for p,o in spec.PB.items():
  if g.git(repo,'rev-parse',f'{spec.PRODUCT}:{p}')!=o:raise g.E(f'product blob {p}')
def lock(repo,spec,g):
 p=next(x for x in spec.PB if x.endswith('COMPOSITION.lock.json')); raw=g.gb(repo,'show',f'{spec.PRODUCT}:{p}');o=g.strict_json(raw)
 if raw!=g.can(o):raise g.E('composition lock noncanonical')
 req={'schema':'mycelix.psi.002b.composition-lock.v0.2','profile':'psi-abuse-control-v1','raw_identifier_policy':'ForbiddenAtAdmissionBoundary','synthetic_only':True}
 for k,v in req.items():
  if o.get(k)!=v:raise g.E(f'composition lock {k}')
 r=o.get('runtime_contract',{})
 for k,v in {'atomic_compare_and_consume_required':True,'distributed_consistency_required_if_multi_node':True,'failed_admission_partial_state_forbidden':True,'revocation_check_inside_atomic_admission':True,'runtime_contract_path':'RUNTIME-CONTRACT.md','request_commitment_scope':'PsiSubjectServiceCapabilityEpoch','epoch_rollover':'FreshServerAuthorizedLedgerNoClientReset'}.items():
  if r.get(k)!=v:raise g.E(f'runtime lock {k}')
 for k,v in o.get('authority_ceiling',{}).items():
  if v is not False:raise g.E(f'authority widened {k}')
 return g.sha(raw)
def contract(repo,spec,g):
 p=next(x for x in spec.PB if x.endswith('RUNTIME-CONTRACT.md')); s=g.gb(repo,'show',f'{spec.PRODUCT}:{p}').decode()
 for t in ['one serializable admission transaction','Revocation status must be read inside the same serializable transaction','none of those state changes may become visible','A check-then-update sequence split across independent transactions does not satisfy this contract','fresh ledger namespace','Local per-process counters do not satisfy this contract','must not receive raw queried identifiers']:
  if t not in s:raise g.E(f'contract invariant missing: {t}')
 return g.sha(s.encode())
def source(repo,spec,g):
 sp=next(x for x in spec.PB if x.endswith('src/lib.rs')); cp=next(x for x in spec.PB if x.endswith('Cargo.toml'))
 s=g.gb(repo,'show',f'{spec.PRODUCT}:{sp}').decode(); c=g.gb(repo,'show',f'{spec.PRODUCT}:{cp}').decode();m=tomllib.loads(c)
 deps=set(m.get('dependencies',{}))
 if deps!={'privacy-computation-core','privacy-protocol-profiles','serde'}:raise g.E(f'dependency surface widened: {sorted(deps)}')
 for t in ['PSI_002A_SUBJECT','ForbiddenAtAdmissionBoundary','UniquePerCapabilityEpoch','CheckBeforeEveryAdmission','online_enumeration_resistance_established','sybil_resistance_established','privacy_preserving_accounting_established','production_admission_granted','application_authority_granted']:
  if t not in s:raise g.E(f'source invariant missing: {t}')
 for t in ['std::fs','std::net','std::process','tokio','rusqlite','sqlx','unsafe {','unsafe fn','extern "C"','VoprfClient','VoprfServer']:
  if t in s or t in c:raise g.E(f'runtime/backend capability present: {t}')
 if s.count('#[test]')!=spec.TESTS:raise g.E('product test count')
 return {'tests':spec.TESTS,'composition_lock_sha256':lock(repo,spec,g),'runtime_contract_sha256':contract(repo,spec,g)}
