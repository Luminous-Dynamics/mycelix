from __future__ import annotations
import ctypes,os,shutil,signal,subprocess,sys,tempfile
from pathlib import Path
from urllib.parse import urlparse
from qcap_canon import CapsuleError

def git(repo,*args,check=True):
 try:return subprocess.run(["git","-C",str(repo),*args],check=check,capture_output=True,text=True)
 except(OSError,subprocess.CalledProcessError)as e:raise CapsuleError(f"git {' '.join(args)} failed")from e
def normalize_github_identity(url):
 if not url:return None
 v=url.strip()
 if v.startswith("git@github.com:"):p=v[len("git@github.com:"):]
 else:
  u=urlparse(v)
  if u.hostname not in{"github.com","www.github.com"}:return None
  p=u.path.lstrip("/")
 if p.endswith(".git"):p=p[:-4]
 parts=[x for x in p.split("/")if x];return"/".join(parts[:2])if len(parts)>=2 else None
def preflight(m,repo,rid):
 if rid!=m["repository_identity"]:raise CapsuleError("repository identity mismatch")
 if git(repo,"cat-file","-e",m["product_subject_sha"]+"^{commit}",check=False).returncode:raise CapsuleError("subject unavailable")
 origin=git(repo,"remote","get-url","origin",check=False)
 if origin.returncode==0:
  observed=normalize_github_identity(origin.stdout.strip())
  if observed is not None and observed!=m["repository_identity"]:raise CapsuleError("configured repository origin mismatch")
def state(w):return git(w,"rev-parse","HEAD").stdout.strip(),git(w,"status","--porcelain=v1","--untracked-files=all").stdout.strip()
def add_worktree(repo,subject):
 p=Path(tempfile.mkdtemp(prefix="qcap-"));w=p/"subject"
 if git(repo,"worktree","add","--detach","--force",str(w),subject,check=False).returncode:shutil.rmtree(p,ignore_errors=True);raise CapsuleError("worktree materialization failed")
 return p,w
def remove_worktree(repo,p,w):
 r=git(repo,"worktree","remove","--force",str(w),check=False);shutil.rmtree(p,ignore_errors=True)
 if r.returncode:raise CapsuleError("worktree cleanup failed")
def enable_linux_subreaper():
 if not sys.platform.startswith("linux"):return
 try:
  libc=ctypes.CDLL(None,use_errno=True);prctl=libc.prctl;prctl.argtypes=[ctypes.c_int,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_ulong];prctl.restype=ctypes.c_int
  if prctl(36,1,0,0,0)!=0:raise OSError(ctypes.get_errno(),"prctl(PR_SET_CHILD_SUBREAPER)")
 except(AttributeError,OSError)as e:raise CapsuleError(f"linux subreaper setup failed: {e}")from e
def reap_group(pgid):
 if not sys.platform.startswith("linux"):return
 import time;deadline=time.monotonic()+1.0
 while time.monotonic()<deadline:
  any_reaped=False
  while True:
   try:pid,_=os.waitpid(-pgid,os.WNOHANG)
   except ChildProcessError:return
   if pid==0:break
   any_reaped=True
  if not any_reaped:time.sleep(.01)
 while True:
  try:pid,_=os.waitpid(-pgid,os.WNOHANG)
  except ChildProcessError:return
  if pid==0:return
def terminate_group(p):
 if os.name!="posix":p.kill();p.communicate();return
 pgid=p.pid
 try:os.killpg(pgid,signal.SIGTERM)
 except ProcessLookupError:pass
 try:p.communicate(timeout=1)
 except subprocess.TimeoutExpired:
  try:os.killpg(pgid,signal.SIGKILL)
  except ProcessLookupError:pass
  p.communicate()
 reap_group(pgid)
def execute(argv,cwd,env,timeout):
 try:p=subprocess.Popen(argv,cwd=cwd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=(os.name=="posix"))
 except OSError as e:return f"qcap_gate_start=RUNNER_FAILURE {e}\n".encode(),20
 try:o,_=p.communicate(timeout=timeout);return o,p.returncode
 except subprocess.TimeoutExpired as e:
  partial=e.output or b"";terminate_group(p);return partial+f"\nqcap_gate_timeout=RUNNER_FAILURE seconds={timeout}\n".encode(),20
