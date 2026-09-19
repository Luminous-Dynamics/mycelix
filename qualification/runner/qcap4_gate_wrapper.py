#!/usr/bin/env python3
from __future__ import annotations
import os,sys
from pathlib import Path

def fail(message):
 print("qcap4_gate_wrapper="+message,file=sys.stderr);raise SystemExit(20)

def control(path,name):
 q=Path(path)/name
 if q.is_symlink()or not q.is_file():fail("containment primitive missing "+name)
 return q

def join_before_exec(cgroup):
 q=control(cgroup,"cgroup.procs");fd=None
 try:
  fd=os.open(q,os.O_WRONLY|os.O_CLOEXEC)
  data=(str(os.getpid())+"\n").encode()
  if os.write(fd,data)!=len(data):fail("short cgroup.procs write")
 except OSError:fail("cannot join containment cgroup")
 finally:
  if fd is not None:
   try:os.close(fd)
   except OSError:pass
 try:members={int(x)for x in q.read_text().split()if x.isdigit()}
 except(OSError,UnicodeError):fail("cannot verify containment membership")
 if os.getpid()not in members:fail("containment membership not established")

def main(argv=None):
 a=list(sys.argv[1:]if argv is None else argv)
 if len(a)<2:fail("usage: qcap4_gate_wrapper.py CGROUP EXEC [ARGS...]")
 cgroup=Path(a[0]);exe=a[1]
 if not cgroup.is_absolute():fail("cgroup path must be absolute")
 if not os.path.isabs(exe):fail("gate executable must be absolute")
 join_before_exec(cgroup)
 try:os.execve(exe,a[1:],dict(os.environ))
 except OSError:fail("gate exec failed")

if __name__=="__main__":main()
