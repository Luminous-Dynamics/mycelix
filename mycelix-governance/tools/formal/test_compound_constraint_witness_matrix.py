import argparse,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser(); p.add_argument("--matrix",type=Path,required=True); p.add_argument("--reference",type=Path,required=True)
a=p.parse_args(); m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-compound-constraint-witness-control-matrix.v1"
assert len(m["controls"])==10
ref=a.reference.read_text()
for c in m["controls"]: assert c["marker"] in ref
r=subprocess.run(["python3",str(a.reference)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
assert r.returncode==0,r.stdout
print("COMPOUND CONSTRAINT MATRIX PASS: 10 controls executable")
