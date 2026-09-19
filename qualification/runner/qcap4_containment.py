from __future__ import annotations
import hashlib,os,sys,time
from pathlib import Path
from qcap_canon import CapsuleError,canonical_json,commitment,exact_keys

CONTAINMENT_DOMAIN=b"MYCELIX_QCAP_CONTAINMENT_PROFILE_V1\0"
RESOLVED_CONTAINMENT_DOMAIN=b"MYCELIX_QCAP_RESOLVED_CONTAINMENT_V1\0"
PROFILE_KEYS={
 "containment_profile_format_revision","profile_id","profile_revision","platform","primitive",
 "membership_start","termination_policy","empty_postcondition","descendant_scope",
 "session_process_group_escape_covered","cleanup_ceiling_ms","hostile_code_sandbox"
}
class ContainmentError(CapsuleError):pass

def validate_containment_profile(p):
 exact_keys(p,PROFILE_KEYS,"containment profile")
 expected={
  "containment_profile_format_revision":1,"platform":"linux","primitive":"cgroup-v2",
  "membership_start":"before-gate-exec","termination_policy":"cgroup-kill",
  "empty_postcondition":"cgroup-events-populated-zero",
  "descendant_scope":"cgroup-membership-preserving",
  "session_process_group_escape_covered":True,"hostile_code_sandbox":False,
 }
 for k,v in expected.items():
  if p[k]!=v:raise CapsuleError(f"unsupported containment profile {k}")
 if not isinstance(p["profile_id"],str)or not p["profile_id"]:raise CapsuleError("containment profile id invalid")
 r=p["profile_revision"]
 if not isinstance(r,int)or isinstance(r,bool)or r<1:raise CapsuleError("containment profile revision invalid")
 c=p["cleanup_ceiling_ms"]
 if not isinstance(c,int)or isinstance(c,bool)or not 1<=c<=60000:raise CapsuleError("containment cleanup ceiling invalid")
 canonical_json(p);return p

def containment_digest(p):validate_containment_profile(p);return commitment(CONTAINMENT_DOMAIN,p)
def containment_ref(p):
 validate_containment_profile(p);return{"id":p["profile_id"],"revision":p["profile_revision"],"digest":containment_digest(p)}

def _root_identity(root):
 p=Path(root)
 if not p.is_absolute():raise ContainmentError("containment root must be absolute")
 if p.is_symlink():raise ContainmentError("containment root symlink forbidden")
 try:r=p.resolve(strict=True)
 except OSError as e:raise ContainmentError("containment root unavailable")from e
 return r

def resolved_containment_document(profile,root):
 r=_root_identity(root)
 return{"resolved_containment_format_revision":1,"containment_profile_ref":containment_ref(profile),"containment_root":str(r)}

def resolved_containment_commitment(profile,root):
 return commitment(RESOLVED_CONTAINMENT_DOMAIN,resolved_containment_document(profile,root))

def parse_cgroup_events(text):
 out={}
 for line in text.splitlines():
  parts=line.split()
  if len(parts)!=2 or not parts[1].isdigit():raise ContainmentError("malformed cgroup.events")
  out[parts[0]]=int(parts[1])
 if "populated"not in out or out["populated"]not in(0,1):raise ContainmentError("cgroup.events missing populated")
 return out

def cgroup_populated(cgroup):
 try:return parse_cgroup_events((Path(cgroup)/"cgroup.events").read_text())["populated"]==1
 except(OSError,UnicodeError)as e:raise ContainmentError("cannot read cgroup.events")from e

def _gate_name(attempt_id,gate_id):
 raw=(attempt_id+"\0"+gate_id).encode("utf-8");return"qcap-"+hashlib.sha256(raw).hexdigest()[:32]

def _validate_child_files(p):
 for name in("cgroup.procs","cgroup.events","cgroup.kill"):
  q=p/name
  if not q.exists()or q.is_symlink():raise ContainmentError(f"containment primitive missing {name}")

def create_gate_cgroup(root,attempt_id,gate_id):
 r=_root_identity(root);p=r/_gate_name(attempt_id,gate_id)
 try:p.mkdir(mode=0o700)
 except OSError as e:raise ContainmentError("cannot create gate cgroup")from e
 try:_validate_child_files(p);return p
 except Exception:
  try:p.rmdir()
  except OSError:pass
  raise

def _write_control(path,data):
 fd=None
 try:
  fd=os.open(path,os.O_WRONLY|os.O_CLOEXEC);os.write(fd,data)
 except OSError as e:raise ContainmentError(f"cannot write {Path(path).name}")from e
 finally:
  if fd is not None:
   try:os.close(fd)
   except OSError:pass

def probe_containment_root(root,profile):
 validate_containment_profile(profile)
 if not sys.platform.startswith("linux"):raise ContainmentError("cgroup-v2 containment requires linux")
 r=_root_identity(root)
 for name in("cgroup.procs","cgroup.controllers"):
  q=r/name
  if not q.exists()or q.is_symlink():raise ContainmentError(f"containment root missing {name}")
 token=_gate_name("probe-"+str(os.getpid()),str(time.monotonic_ns()));p=r/token
 try:
  p.mkdir(mode=0o700);_validate_child_files(p)
  if cgroup_populated(p):raise ContainmentError("new containment probe unexpectedly populated")
  fd=os.open(p/"cgroup.procs",os.O_WRONLY|os.O_CLOEXEC);os.close(fd)
  _write_control(p/"cgroup.kill",b"1")
 finally:
  try:
   if p.exists():p.rmdir()
  except OSError as e:raise ContainmentError("containment probe cleanup failed")from e
 return resolved_containment_commitment(profile,r)

def kill_cgroup(cgroup):
 try:_write_control(Path(cgroup)/"cgroup.kill",b"1")
 except ContainmentError as e:raise ContainmentError("cgroup.kill failed")from e

def wait_empty(cgroup,ceiling_ms):
 deadline=time.monotonic()+ceiling_ms/1000
 while True:
  if not cgroup_populated(cgroup):return
  if time.monotonic()>=deadline:raise ContainmentError("cgroup remained populated")
  time.sleep(.01)

def remove_empty_tree(cgroup):
 p=Path(cgroup)
 try:
  dirs=[x for x in p.rglob("*")if x.is_dir()]
  for d in sorted(dirs,key=lambda x:len(x.parts),reverse=True):d.rmdir()
  p.rmdir()
 except OSError as e:raise ContainmentError("cgroup cleanup failed")from e

def terminate_and_cleanup(cgroup,ceiling_ms):
 kill_cgroup(cgroup);wait_empty(cgroup,ceiling_ms);remove_empty_tree(cgroup)

def cleanup_empty(cgroup):
 if cgroup_populated(cgroup):raise ContainmentError("residual descendant process")
 remove_empty_tree(cgroup)
