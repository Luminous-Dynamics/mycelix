import hashlib,json,os,pathlib,subprocess,sys
class E(RuntimeError):pass
def can(o):return (json.dumps(o,sort_keys=True,separators=(',',':'),ensure_ascii=False)+'\n').encode()
def sha(b):return hashlib.sha256(b).hexdigest()
BAD_GIT={'GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE','GIT_OBJECT_DIRECTORY','GIT_ALTERNATE_OBJECT_DIRECTORIES','GIT_COMMON_DIR','GIT_REPLACE_REF_BASE','GIT_CONFIG_GLOBAL','GIT_CONFIG_SYSTEM','GIT_CONFIG_NOSYSTEM','GIT_CONFIG_COUNT','GIT_CONFIG_PARAMETERS','GIT_NAMESPACE','GIT_SHALLOW_FILE'}
BAD_RUST={'RUSTC_WRAPPER','RUSTC_WORKSPACE_WRAPPER','RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTDOCFLAGS','RUSTDOC','RUSTC'}
def reject_env(env=None):
 env=os.environ if env is None else env; bad=[]
 for k,v in env.items():
  if not v:continue
  if k in BAD_GIT or k in BAD_RUST or k.startswith(('GIT_CONFIG_KEY_','GIT_CONFIG_VALUE_','CARGO_PROFILE_','CARGO_TARGET_','CARGO_REGISTRIES_','CARGO_SOURCE_')):bad.append(k)
 if bad:raise E(f'forbidden environment override(s): {sorted(bad)}')
def python_runtime(flags=None,dwb=None):
 flags=sys.flags if flags is None else flags; dwb=sys.dont_write_bytecode if dwb is None else dwb
 req={'isolated':1,'ignore_environment':1,'no_user_site':1,'no_site':1,'safe_path':True}
 for k,v in req.items():
  if getattr(flags,k,None)!=v:raise E(f'python isolation mismatch: {k}')
 if not dwb:raise E('python bytecode writes must be disabled')
def env(cargo_home=None):
 e={k:v for k,v in os.environ.items() if not k.startswith(('GIT_','CARGO_','RUST'))}
 e.update(GIT_NO_REPLACE_OBJECTS='1',GIT_CONFIG_NOSYSTEM='1',GIT_CONFIG_GLOBAL=os.devnull,CARGO_NET_OFFLINE='true')
 if cargo_home:e['CARGO_HOME']=str(cargo_home)
 return e
def sh(a,cwd,check=True,text=True,cargo_home=None):return subprocess.run(a,cwd=cwd,env=env(cargo_home),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=text,check=check)
def git(repo,*a):return sh(['git',*a],repo).stdout.strip()
def gb(repo,*a):return sh(['git',*a],repo,text=False).stdout
def repo_guard(repo):
 if pathlib.Path(git(repo,'rev-parse','--show-toplevel')).resolve()!=repo.resolve():raise E('repo root mismatch')
 gd=pathlib.Path(git(repo,'rev-parse','--git-dir'));gd=gd if gd.is_absolute() else (repo/gd).resolve()
 for r in ('info/grafts','objects/info/alternates'):
  p=gd/r
  if p.exists() and p.read_bytes().strip():raise E(f'git indirection: {r}')
 if git(repo,'for-each-ref','--format=%(refname)','refs/replace'):raise E('replace refs present')
 cfg=gd/'config'
 if cfg.exists():
  q=sh(['git','config','--file',str(cfg),'--no-includes','--name-only','--list'],repo,check=False)
  if q.returncode:raise E('cannot inspect git config')
  for k in q.stdout.lower().splitlines():
   if k.strip() in {'core.worktree','extensions.worktreeconfig','core.attributesfile','core.hookspath','core.fsmonitor','core.alternaterefscommand','include.path'} or (k.startswith('includeif.') and k.endswith('.path')):raise E(f'dangerous git config: {k}')
def execution_paths(repo,files):
 root=(repo/files['qualify.py']).parent.resolve()
 for n,p in files.items():
  exp=(root/n).resolve()
  if pathlib.Path(p).resolve()!=exp:raise E(f'executing path mismatch: {n}')
def working(repo,spec):
 out={}
 for rel in spec.PATHS:
  p=repo/rel
  if p.is_symlink() or not p.is_file():raise E(f'bad qualifier file: {rel}')
  b=p.read_bytes(); c=gb(repo,'show',f'HEAD:{rel}')
  if b!=c:raise E(f'working bytes differ: {rel}')
  out[rel]={'blob':git(repo,'rev-parse',f'HEAD:{rel}'),'sha256':sha(b)}
 return out
def strict_json(raw):
 if len(raw)>1048576:raise E('json too large')
 def pairs(ps):
  d={}
  for k,v in ps:
   if k in d:raise E(f'duplicate json key: {k}')
   d[k]=v
  return d
 try:return json.loads(raw.decode(),object_pairs_hook=pairs,parse_constant=lambda x:(_ for _ in()).throw(E('nonfinite json')))
 except (UnicodeDecodeError,json.JSONDecodeError) as x:raise E(str(x))
