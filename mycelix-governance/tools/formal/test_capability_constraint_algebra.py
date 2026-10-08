import argparse,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument("--matrix",type=Path,required=True); p.add_argument("--reference",type=Path,required=True)
p.add_argument("--tla",type=Path,required=True); p.add_argument("--alloy",type=Path,required=True)
a=p.parse_args(); m=json.loads(a.matrix.read_text())
assert m["schema"]=="mycelix.evidence-attestation-capability-constraint-algebra-control-matrix.v1"
assert len(m["controls"])==11
ref=a.reference.read_text(); tla=a.tla.read_text(); alloy=a.alloy.read_text()
for c in m["controls"]:
    assert c["marker"] in ref and c["tla_invariant"] in tla and c["alloy_assertion"] in alloy
assert "syntax_equal" in ref and "semantic_equivalent" in ref and "decidable_subsumes" in ref
assert "AllowSubsumption" in tla and "DenyPreservation" in tla
assert "DenyOverrides" in alloy and "attenuation" in alloy
r=subprocess.run(["python3",str(a.reference)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
assert r.returncode==0,r.stdout
print("CONSTRAINT ALGEBRA FORMAL MATRIX PASS: reference/TLA+/Alloy surfaces are structurally aligned")
