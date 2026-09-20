#!/usr/bin/env python3
import argparse,pathlib,sys
H=pathlib.Path(__file__).resolve().parent;sys.path.insert(0,str(H))
import spec,guard,model,execute
p=argparse.ArgumentParser();p.add_argument("--repo",default=".");p.add_argument("--receipt",required=True);a=p.parse_args()
files={"qualify.py":pathlib.Path(__file__),"spec.py":pathlib.Path(spec.__file__),"guard.py":pathlib.Path(guard.__file__),"model.py":pathlib.Path(model.__file__),"execute.py":pathlib.Path(execute.__file__)}
try:execute.qualify(pathlib.Path(a.repo),pathlib.Path(a.receipt),files)
except guard.E as e:print(f"QUALIFICATION-FAIL: {e}",file=sys.stderr);raise SystemExit(1)
print("QUALIFICATION-PASS")
