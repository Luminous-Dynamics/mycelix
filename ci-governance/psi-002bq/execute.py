import json,pathlib,re,tempfile,tomllib

def tool(r,c,spec,g):
 rm=re.search(r'^release:\s*(\S+)$',r,re.M); rh=re.search(r'^commit-hash:\s*([0-9a-f]+)$',r,re.M); cm=re.search(r'^release:\s*(\S+)$',c,re.M); ch=re.search(r'^commit-hash:\s*([0-9a-f]+)$',c,re.M)
 if not rm or (rm.group(1),rh.group(1) if rh else '')!=(spec.RT,spec.RC):raise g.E('rustc identity')
 if not cm or (cm.group(1),ch.group(1) if ch else '')!=(spec.CT,spec.CC):raise g.E('cargo identity')
 return {'rustc_release':spec.RT,'rustc_commit':spec.RC,'cargo_release':spec.CT,'cargo_commit':spec.CC}
def materialize(repo,root,spec,g):
 w=root/'workspace';w.mkdir();(w/'Cargo.toml').write_text('[workspace]\nmembers=["privacy-computation-core","privacy-protocol-profiles","psi-abuse-control-profile"]\nresolver="2"\n')
 for dest,prefix in spec.COMP:
  paths=g.git(repo,'ls-tree','-r','--name-only',spec.PRODUCT,'--',prefix).splitlines()
  if not paths:raise g.E(f'missing component {prefix}')
  for src in paths:
   rel=src[len(prefix)+1:];d=w/dest/rel;d.parent.mkdir(parents=True,exist_ok=True);d.write_bytes(g.gb(repo,'show',f'{spec.PRODUCT}:{src}'))
 return w
def cargo_home(root):
 h=root/'cargo-home';h.mkdir(); orig=pathlib.Path.home()/'.cargo'
 for n in ('registry','git'):
  p=orig/n
  if p.exists():(h/n).symlink_to(p,target_is_directory=True)
 return h
def run(a,cwd,g,ch=None):
 q=g.sh(a,cwd,check=False,cargo_home=ch)
 if q.returncode:raise g.E(f'command failed: {a}: {q.stderr[-1000:]}')
 return {'argv':a,'returncode':0,'stdout_sha256':g.sha(q.stdout.encode()),'stderr_sha256':g.sha(q.stderr.encode())},q.stdout
def dependency_manifest(lock_raw,metadata_raw,g):
 try: lock=tomllib.loads(lock_raw.decode()); meta=json.loads(metadata_raw)
 except Exception as x: raise g.E(f'dependency evidence parse failed: {x}')
 lp=[]
 for p in lock.get('package',[]):
  lp.append({'name':p.get('name'),'version':p.get('version'),'source':p.get('source'),'checksum':p.get('checksum'),'dependencies':sorted(p.get('dependencies',[]))})
 lp.sort(key=lambda x:(x['name'] or '',x['version'] or '',x['source'] or ''))
 byid={p['id']:(p.get('name'),p.get('version'),p.get('source')) for p in meta.get('packages',[]) if 'id' in p}
 rf=[]
 resolve=meta.get('resolve') or {}
 for n in resolve.get('nodes',[]):
  if n.get('id') not in byid: raise g.E('metadata node missing package')
  name,version,source=byid[n['id']]
  rf.append({'name':name,'version':version,'source':source,'features':sorted(n.get('features',[]))})
 rf.sort(key=lambda x:(x['name'] or '',x['version'] or '',x['source'] or ''))
 if not lp or not rf: raise g.E('empty dependency evidence')
 return {'lock_packages':lp,'resolved_features':rf}
def qualify(repo,out,spec,g,prod,files):
 g.reject_env();g.python_runtime();repo=repo.resolve();out=out.resolve()
 try:out.relative_to(repo);raise g.E('receipt must be outside checkout')
 except ValueError:pass
 g.repo_guard(repo);g.execution_paths(repo,files)
 if g.git(repo,'status','--porcelain','--untracked-files=all'):raise g.E('dirty checkout')
 head=g.git(repo,'rev-parse','HEAD');tree=g.git(repo,'rev-parse','HEAD^{tree}')
 if g.git(repo,'rev-parse','HEAD^')!=spec.PRODUCT:raise g.E('qualifier parent')
 if tuple(sorted(g.git(repo,'diff','--name-only',spec.PRODUCT,head).splitlines()))!=tuple(sorted(spec.PATHS)):raise g.E('qualifier path set')
 initial=g.working(repo,spec);prod.verify(repo,spec,g); evidence=prod.source(repo,spec,g)
 blobs={pathlib.Path(x).name:g.git(repo,'rev-parse',f'HEAD:{x}') for x in spec.PATHS if not x.endswith('/lock.json')}
 raw=g.gb(repo,'show',f'HEAD:{spec.QX}/lock.json');actual=g.strict_json(raw);expected=spec.lock_obj(blobs)
 if actual!=expected or raw!=g.can(expected):raise g.E('qualifier lock')
 with tempfile.TemporaryDirectory(prefix='psi-002bq-') as z:
  root=pathlib.Path(z);w=materialize(repo,root,spec,g);ch=cargo_home(root)
  rr=g.sh(['rustc','-Vv'],w,check=False,cargo_home=ch);cc=g.sh(['cargo','-Vv'],w,check=False,cargo_home=ch)
  if rr.returncode or cc.returncode:raise g.E('toolchain unavailable')
  ti=tool(rr.stdout,cc.stdout,spec,g);runs=[];metadata_raw=None
  for a in spec.COMMANDS:
   rec,stdout=run(a,w,g,ch);runs.append(rec)
   if a[:2]==['cargo','metadata']:metadata_raw=stdout
  lock_raw=(w/'Cargo.lock').read_bytes();cargo_lock_sha256=g.sha(lock_raw)
  if metadata_raw is None:raise g.E('cargo metadata evidence missing')
  dependency_resolution=dependency_manifest(lock_raw,metadata_raw,g)
 g.repo_guard(repo);g.execution_paths(repo,files)
 if g.git(repo,'status','--porcelain','--untracked-files=all') or g.git(repo,'rev-parse','HEAD')!=head or g.git(repo,'rev-parse','HEAD^{tree}')!=tree:raise g.E('postflight checkout drift')
 if g.working(repo,spec)!=initial:raise g.E('postflight qualifier byte drift')
 prod.verify(repo,spec,g); final=prod.source(repo,spec,g)
 if final!=evidence:raise g.E('postflight product probe drift')
 receipt={'schema':'mycelix.psi.002bq.receipt.v0.1','product':spec.PRODUCT,'qualifier_commit':head,'qualifier_tree':tree,'toolchain':ti,'commands':runs,'cargo_lock_sha256':cargo_lock_sha256,'dependency_resolution':dependency_resolution,'evidence':evidence,'claims':{'exact_source_qualified':True,'structural_abuse_profile_qualified':True,'runtime_contract_text_qualified':True,'durable_atomic_accounting_established':False,'revocation_race_free_established':False,'distributed_consistency_established':False,'enumeration_resistance_established':False,'sybil_resistance_established':False,'production_admitted':False,'application_authority_granted':False}}
 receipt['receipt_commitment_sha256']=g.sha(g.can(receipt));out.write_bytes(g.can(receipt));return receipt
