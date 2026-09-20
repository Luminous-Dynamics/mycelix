import hashlib,json,os,pathlib,subprocess
import spec
class E(RuntimeError):pass
def h(b):return hashlib.sha256(b).hexdigest()
def can(x):return (json.dumps(x,sort_keys=True,separators=(",",":"))+"\n").encode()
def env():
 d={k:v for k,v in os.environ.items() if not k.startswith("GIT_")};d.update(GIT_NO_REPLACE_OBJECTS="1",GIT_CONFIG_NOSYSTEM="1",GIT_CONFIG_GLOBAL=os.devnull);return d
def sh(a,r,e=None,ok=True,text=True):return subprocess.run(a,cwd=r,env=e or env(),check=ok,text=text,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
def g(r,*a):return sh(["git",*a],r).stdout.strip()
def gb(r,*a):return sh(["git",*a],r,text=False).stdout
def reject_env(s=None):
 s=os.environ if s is None else s;b=[k for k,v in s.items() if v and (k.startswith(("GIT_CONFIG_","CARGO_PROFILE_","CARGO_TARGET_","CARGO_REGISTRIES_","CARGO_SOURCE_")) or k in {"GIT_DIR","GIT_WORK_TREE","GIT_INDEX_FILE","GIT_OBJECT_DIRECTORY","GIT_ALTERNATE_OBJECT_DIRECTORIES","GIT_COMMON_DIR","GIT_REPLACE_REF_BASE","GIT_NAMESPACE","GIT_SHALLOW_FILE","RUSTC","RUSTC_WRAPPER","RUSTC_WORKSPACE_WRAPPER","RUSTFLAGS","CARGO_ENCODED_RUSTFLAGS","RUSTDOCFLAGS","CARGO_BUILD_TARGET","CARGO_BUILD_RUSTC","CARGO_BUILD_RUSTC_WRAPPER","CARGO_BUILD_RUSTFLAGS"})]
 if b:raise E(f"forbidden environment override(s): {sorted(b)}")
def python_runtime(flags,dwb):
 for k,v in {"isolated":1,"ignore_environment":1,"no_user_site":1,"no_site":1,"safe_path":True}.items():
  if getattr(flags,k,None)!=v:raise E(f"python isolation mismatch: {k}")
 if not dwb:raise E("bytecode writes enabled")
def execution_paths(r,files):
 for name,path in files.items():
  if pathlib.Path(path).resolve()!=(r/spec.QX/name).resolve():raise E(f"executing module path mismatch: {name}")
def indirection(r):
 if pathlib.Path(g(r,"rev-parse","--show-toplevel")).resolve()!=r.resolve():raise E("repo root mismatch")
 gd=pathlib.Path(g(r,"rev-parse","--git-dir"));gd=gd if gd.is_absolute() else (r/gd).resolve()
 for z in ("info/grafts","objects/info/alternates"):
  p=gd/z
  if p.exists() and p.read_bytes().strip():raise E(f"git indirection: {z}")
 if g(r,"for-each-ref","--format=%(refname)","refs/replace"):raise E("replace refs")
 cf=gd/"config"
 if cf.exists():
  p=sh(["git","config","--file",str(cf),"--no-includes","--name-only","--list"],r,ok=False)
  if p.returncode:raise E("git config unreadable")
  bad={"core.worktree","extensions.worktreeconfig","core.attributesfile","core.hookspath","core.fsmonitor","core.alternaterefscommand","include.path"}
  for k in p.stdout.lower().splitlines():
   if k in bad or (k.startswith("includeif.") and k.endswith(".path")):raise E(f"dangerous git config: {k}")
def checkout(r):
 if g(r,"status","--porcelain","--untracked-files=all"):raise E("dirty checkout")
 if g(r,"rev-parse","HEAD^")!=spec.P or tuple(sorted(g(r,"diff","--name-only",spec.P,"HEAD").splitlines()))!=tuple(sorted(spec.Q)):raise E("qualifier topology mismatch")
 return g(r,"rev-parse","HEAD"),g(r,"rev-parse","HEAD^{tree}")
def working(r):
 o={}
 for n,p in zip(spec.N,spec.Q,strict=True):
  f=r/p;b=f.read_bytes()
  if f.is_symlink() or not f.is_file() or b!=gb(r,"show",f"HEAD:{p}"):raise E(f"qualifier byte mismatch: {p}")
  o[n]={"blob":g(r,"rev-parse",f"HEAD:{p}"),"sha256":h(b)}
 return o
def verify_lock(r):
 names=("README.md","spec.py","guard.py","model.py","execute.py","qualify.py","test_qualify.py");s={n:g(r,"rev-parse",f"HEAD:{spec.QX}/{n}") for n in names};raw=gb(r,"show",f"HEAD:{spec.QX}/lock.json")
 if raw!=can(spec.lock_obj(s)):raise E("qualifier lock mismatch")
 return h(raw)
