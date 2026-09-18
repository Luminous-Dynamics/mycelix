#!/usr/bin/env python3
"""QCAP built-in reusable theorem gate: exact predecessor/scope/blob identity."""
import json, os, subprocess, sys
from pathlib import Path

def git(repo,*args):
    try:
        p=subprocess.run(["git","-C",str(repo),*args],check=True,capture_output=True,text=True)
    except (OSError,subprocess.CalledProcessError) as exc:
        print(f"runner_git_failure={exc}",file=sys.stderr)
        raise SystemExit(20)
    return p.stdout.strip()

repo=Path(os.environ["QCAP_SUBJECT_DIR"]).resolve()
head=os.environ["QCAP_PRODUCT_SUBJECT_SHA"]
expected_parent=os.environ["QCAP_PREDECESSOR_SHA"]
expected_paths=json.loads(os.environ["QCAP_EXPECTED_CHANGED_PATHS_JSON"])
expected_blobs=json.loads(os.environ["QCAP_EXPECTED_OBJECT_BLOBS_JSON"])

parent=git(repo,"rev-parse","HEAD^")
if parent!=expected_parent:
    print(f"lineage_predecessor=FAIL expected={expected_parent} actual={parent}",file=sys.stderr)
    raise SystemExit(10)

actual_paths=git(repo,"diff","--name-only",parent,head).splitlines()
actual_paths=sorted(actual_paths,key=lambda x:x.encode())
if actual_paths!=expected_paths:
    print(f"lineage_scope=FAIL expected={expected_paths!r} actual={actual_paths!r}",file=sys.stderr)
    raise SystemExit(10)

for path,expected in sorted(expected_blobs.items(),key=lambda kv:kv[0].encode()):
    try:
        p=subprocess.run(
            ["git","-C",str(repo),"rev-parse",f"{head}:{path}"],
            capture_output=True,text=True
        )
    except OSError as exc:
        print(f"runner_git_failure={exc}",file=sys.stderr)
        raise SystemExit(20)
    if p.returncode!=0:
        print(f"object_identity=FAIL path={path} expected={expected} actual=<missing>",file=sys.stderr)
        raise SystemExit(10)
    actual=p.stdout.strip()
    if actual!=expected:
        print(f"object_identity=FAIL path={path} expected={expected} actual={actual}",file=sys.stderr)
        raise SystemExit(10)

print("qcap_subject_lineage=PASS")
