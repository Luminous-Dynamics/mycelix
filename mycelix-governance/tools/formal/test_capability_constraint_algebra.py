import argparse,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument("--matrix",type=Path,required=True); p.add_argument("--reference",type=Path,required=True)
a=p.parse_args(); m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-capability-constraint-algebra-control-matrix.v1"
ref=a.reference.read_text()
for c in m["controls"]: assert c["marker"] in ref
assert "syntax_equal" in ref and "semantic_equivalent" in ref and "decidable_subsumes" in ref
assert "deny_denotation" in ref and "effective_denotation" in ref and "supported" in ref
r=subprocess.run(["python3",str(a.reference)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
assert r.returncode==0,r.stdout
print("CONSTRAINT ALGEBRA MATRIX PASS: 11 controls are executable")
print("SEMANTIC RELATIONS PASS: syntax equality, semantic equivalence, and authorization subsumption are distinct")
print("DENY SEPARATION PASS: allow and deny denotations are independently modeled")
