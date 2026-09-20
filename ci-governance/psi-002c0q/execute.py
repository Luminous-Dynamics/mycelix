import os,pathlib,re,tempfile
import spec,guard,model
def tool(a,b):
 def f(x,n):
  m=re.search(rf"^{n}:\s*(\S+)\s*$",x,re.M);return m.group(1) if m else ""
 x=(f(a,"release"),f(a,"commit-hash"),f(b,"release"),f(b,"commit-hash"))
 if x!=(spec.RT,spec.RC,spec.CT,spec.CC):raise guard.E("toolchain identity mismatch")
 return {"rustc_release":x[0],"rustc_commit":x[1],"cargo_release":x[2],"cargo_commit":x[3]}
def copy_component(r,w,prefix,dest):
 rows=guard.g(r,"ls-tree","-r",spec.P,"--",prefix).splitlines()
 if not rows:raise guard.E(f"component missing: {prefix}")
 for row in rows:
  meta,path=row.split("\t",1);mode,typ,_=meta.split()
  if typ!="blob" or mode not in {"100644","100755"}:raise guard.E(f"unsupported component entry: {path}")
  rel=path.split(prefix+"/",1)[1];dst=w/dest/rel;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(guard.gb(r,"show",f"{spec.P}:{path}"))
def materialize(r,root):
 w=root/"w";w.mkdir();(w/"Cargo.toml").write_text('[workspace]\nmembers=["privacy-computation-core","privacy-protocol-profiles","psi-abuse-control-profiles","psi-admission-reference-model"]\nresolver="2"\n')
 for d,p in spec.C:copy_component(r,w,p,d)
 return w
def cenv(root):
 guard.reject_env();e=guard.ev();old=pathlib.Path(os.environ.get("CARGO_HOME",pathlib.Path.home()/".cargo")).expanduser();new=root/"cargo";new.mkdir()
 for n in ("registry","git"):
  if (old/n).exists():os.symlink(old/n,new/n,target_is_directory=True)
 e.update(CARGO_HOME=str(new),CARGO_NET_OFFLINE="true");return e
def qualify(r,out,files):
 import sys
 guard.reject_env();guard.python_runtime(sys.flags,sys.dont_write_bytecode);r=r.resolve();out=out.resolve()
 try:out.relative_to(r);raise guard.E("receipt inside checkout")
 except ValueError:pass
 guard.indirection(r);guard.execution_paths(r,files);qh,qt=guard.checkout(r);w0=guard.working(r);model.product(r);ls=guard.verify_lock(r);ps=model.probe(r)
 with tempfile.TemporaryDirectory(prefix="c0q-") as td:
  root=pathlib.Path(td);w=materialize(r,root);e=cenv(root);rv=guard.sh(["rustc","-Vv"],w,e);cv=guard.sh(["cargo","-Vv"],w,e);tc=tool(rv.stdout,cv.stdout);rs=[]
  for c in spec.CM:
   z=guard.sh(list(c),w,e,ok=False)
   if z.returncode:raise guard.E(f"command failed: {c}")
   rs.append({"argv":list(c),"stdout_sha256":guard.h(z.stdout.encode()),"stderr_sha256":guard.h(z.stderr.encode())})
  cl=guard.h((w/"Cargo.lock").read_bytes())
 guard.indirection(r);guard.execution_paths(r,files)
 if guard.g(r,"status","--porcelain","--untracked-files=all") or guard.g(r,"rev-parse","HEAD")!=qh or guard.g(r,"rev-parse","HEAD^{tree}")!=qt or guard.working(r)!=w0:raise guard.E("postflight checkout drift")
 model.product(r)
 if guard.verify_lock(r)!=ls or model.probe(r)!=ps:raise guard.E("postflight evidence drift")
 x={"schema":"mycelix.psi.002c0q.receipt.v0.2","product":{"commit":spec.P,"tree":spec.PT,"parent":spec.PP},"qualifier":{"commit":qh,"tree":qt,"lock_sha256":ls},"toolchain":tc,"static":ps,"commands":rs,"cargo_lock_sha256":cl,"claim":{"exact_source_qualified":True,"pure_reference_model_qualified":True,"durable_atomicity_measured":False,"crash_recovery_measured":False,"distributed_consistency_measured":False,"enumeration_resistance_established":False,"production_admitted":False,"application_authority_granted":False}}
 x["receipt_commitment_sha256"]=guard.h(guard.can(x));out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(guard.can(x));return x
