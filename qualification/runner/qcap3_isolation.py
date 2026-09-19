from __future__ import annotations
import ctypes,os,selectors,shutil,signal,subprocess,sys,tempfile,time
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
 p=Path(tempfile.mkdtemp(prefix="qcap3-"));w=p/"subject"
 if git(repo,"worktree","add","--detach","--force",str(w),subject,check=False).returncode:shutil.rmtree(p,ignore_errors=True);raise CapsuleError("worktree materialization failed")
 return p,w
def remove_worktree(repo,p,w):
 r=git(repo,"worktree","remove","--force",str(w),check=False);cleanup_error=False
 try:
  if os.path.lexists(p):shutil.rmtree(p)
 except OSError:cleanup_error=True
 if r.returncode or cleanup_error or os.path.lexists(p):raise CapsuleError("worktree cleanup failed")
def enable_linux_subreaper():
 if not sys.platform.startswith("linux"):return
 try:
  libc=ctypes.CDLL(None,use_errno=True);prctl=libc.prctl;prctl.argtypes=[ctypes.c_int,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_ulong];prctl.restype=ctypes.c_int
  if prctl(36,1,0,0,0)!=0:raise OSError(ctypes.get_errno(),"prctl(PR_SET_CHILD_SUBREAPER)")
 except(AttributeError,OSError)as e:raise CapsuleError(f"linux subreaper setup failed: {e}")from e
def reap_group(pgid,ceiling=.5):
 if not sys.platform.startswith("linux"):return
 deadline=time.monotonic()+ceiling
 while time.monotonic()<deadline:
  try:pid,_=os.waitpid(-pgid,os.WNOHANG)
  except ChildProcessError:return
  if pid==0:time.sleep(.01)
def terminate_group_bounded(p,ceiling=.75):
 deadline=time.monotonic()+ceiling
 if os.name!="posix":
  if p.poll()is None:p.kill()
  try:p.wait(timeout=max(0.01,deadline-time.monotonic()))
  except subprocess.TimeoutExpired:pass
  return
 pgid=p.pid
 try:os.killpg(pgid,signal.SIGTERM)
 except ProcessLookupError:pass
 try:p.wait(timeout=min(.2,max(.01,deadline-time.monotonic())))
 except subprocess.TimeoutExpired:pass
 try:os.killpg(pgid,signal.SIGKILL)
 except ProcessLookupError:pass
 if p.poll()is None:
  try:p.wait(timeout=max(.01,deadline-time.monotonic()))
  except subprocess.TimeoutExpired:pass
 reap_group(pgid,max(0.0,deadline-time.monotonic()))
def execute_bounded(argv,cwd,env,timeout,max_output):
 try:p=subprocess.Popen(argv,cwd=cwd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=(os.name=="posix"),bufsize=0)
 except OSError:return b"",20,"ProcessStartFailure",False
 assert p.stdout is not None
 fd=p.stdout.fileno();os.set_blocking(fd,False);sel=selectors.DefaultSelector();sel.register(fd,selectors.EVENT_READ)
 captured=bytearray();deadline=time.monotonic()+timeout;eof=False;reason=None;exit_deadline=None
 try:
  while True:
   now=time.monotonic()
   if p.poll()is None and now>=deadline:
    reason="Timeout";terminate_group_bounded(p);break
   wait=.05 if p.poll()is None else .01
   events=sel.select(wait)
   for key,_ in events:
    remaining=max_output-len(captured);want=max(1,min(65536,remaining+1))
    try:chunk=os.read(key.fd,want)
    except BlockingIOError:continue
    if not chunk:
     eof=True
     try:sel.unregister(fd)
     except Exception:pass
     break
    if len(chunk)>remaining:
     if remaining:captured.extend(chunk[:remaining])
     reason="OutputLimitExceeded";terminate_group_bounded(p);break
    captured.extend(chunk)
   if reason:break
   if p.poll()is not None:
    if eof:break
    # Bound post-exit drain independently of the original theorem timeout.
    if exit_deadline is None:exit_deadline=time.monotonic()+.25
    if time.monotonic()>=exit_deadline:
     reason="OutputDrainTimeout";terminate_group_bounded(p);break
  if reason:
   return bytes(captured),20,reason,reason=="OutputLimitExceeded"
  rc=p.returncode if p.returncode is not None else p.wait(timeout=.05)
  if rc==0:return bytes(captured),0,None,False
  if rc==10:return bytes(captured),10,None,False
  if rc==20:return bytes(captured),20,"GateReportedRunnerFailure",False
  return bytes(captured),20,"UnexpectedExitCode",False
 finally:
  try:sel.close()
  except Exception:pass
  try:p.stdout.close()
  except Exception:pass
