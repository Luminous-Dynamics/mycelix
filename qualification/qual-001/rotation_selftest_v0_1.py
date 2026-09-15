#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
import verifier_v0_2 as v

CURRENT_BUNDLE_REL = "qualification/qual-001/verifier-bundle-v0.2.json"
CURRENT_POINTER_REL = "qualification/qual-001/current-verifier.json"

def run(root: Path, *args: str) -> str:
    return subprocess.check_output(["git","-C",str(root),*args],text=True).strip()

def commit(root: Path, message: str) -> str:
    subprocess.run(["git","-C",str(root),"add","-A"],check=True)
    subprocess.run(["git","-C",str(root),"commit","-m",message],check=True,stdout=subprocess.DEVNULL)
    return run(root,"rev-parse","HEAD")

def init_repo(path: Path) -> None:
    subprocess.run(["git","init","-q",str(path)],check=True)
    subprocess.run(["git","-C",str(path),"config","user.email","qual@example.invalid"],check=True)
    subprocess.run(["git","-C",str(path),"config","user.name","QUAL self-test"],check=True)

def copy_current(dst: Path) -> None:
    for rel in [
        ".github/workflows/qual-001-authoritative.yml",
        "qualification/qual-001/dispatcher_v0_1.py",
        "qualification/qual-001/current-verifier.json",
        "qualification/qual-001/gate-manifest-v0.2.json",
        "qualification/qual-001/verifier-bundle-v0.2.json",
        "qualification/qual-001/rotation-policy-v0.1.json",
        "qualification/qual-001/verifier_v0_2.py",
    ]:
        out = dst / rel
        out.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(REPO / rel, out)
    (dst/"README.md").write_text("base\n")

def make_repo(path: Path) -> str:
    path.mkdir()
    copy_current(path)
    init_repo(path)
    return commit(path,"base")

def verify(verifier_root: Path, subject_root: Path, base: str, head: str) -> dict:
    return v.verify(
        verifier_root,
        subject_root,
        verifier_root/"qualification/qual-001/gate-manifest-v0.2.json",
        verifier_root/CURRENT_BUNDLE_REL,
        verifier_root/CURRENT_POINTER_REL,
        run(verifier_root,"rev-parse","HEAD"),
        base,
        head,
    )

def expect_fail(fn, label: str) -> None:
    try:
        fn()
    except v.VerificationError:
        return
    raise AssertionError(f"expected fail-closed rejection: {label}")

def write_successor(subject: Path, *, wrong_predecessor=False, tamper=False, mixed=False) -> tuple[str,str]:
    gate_rel="qualification/qual-001/gate-manifest-v0.3.json"
    verifier_rel="qualification/qual-001/verifier_v0_3.py"
    policy_rel="qualification/qual-001/rotation-policy-v0.2.json"
    bundle_rel="qualification/qual-001/verifier-bundle-v0.3.json"

    gate=json.loads((REPO/"qualification/qual-001/gate-manifest-v0.2.json").read_text())
    gate["profile"]="mycelix.qual.static-subject-independence.v0.3"
    (subject/gate_rel).write_text(json.dumps(gate,indent=2,sort_keys=True)+"\n")
    (subject/verifier_rel).write_text("# future verifier fixture\nprint('fixture only; never executed')\n")
    policy=json.loads((REPO/"qualification/qual-001/rotation-policy-v0.1.json").read_text())
    policy["profile"]="mycelix.qual.verifier-rotation.v0.2"
    (subject/policy_rel).write_text(json.dumps(policy,indent=2,sort_keys=True)+"\n")

    def digest(rel: str) -> str:
        return hashlib.sha256((subject/rel).read_bytes()).hexdigest()

    current=json.loads((REPO/CURRENT_BUNDLE_REL).read_text())
    successor={
        "schema":"mycelix.qual.verifier-bundle.v0.2",
        "profile":"mycelix.qual.static-subject-independence.v0.3",
        "predecessor_bundle_sha256":"0"*64 if wrong_predecessor else hashlib.sha256((REPO/CURRENT_BUNDLE_REL).read_bytes()).hexdigest(),
        "launcher":current["launcher"],
        "components":{
            "gate_manifest":{"path":gate_rel,"sha256":digest(gate_rel)},
            "verifier":{"path":verifier_rel,"sha256":digest(verifier_rel)},
            "rotation_policy":{"path":policy_rel,"sha256":digest(policy_rel)},
        },
    }
    (subject/bundle_rel).write_text(json.dumps(successor,indent=2,sort_keys=True)+"\n")
    bundle_sha=digest(bundle_rel)
    pointer={"schema":"mycelix.qual.current-verifier.v0.1","profile":successor["profile"],"bundle_path":bundle_rel,"bundle_sha256":bundle_sha}
    (subject/CURRENT_POINTER_REL).write_text(json.dumps(pointer,indent=2,sort_keys=True)+"\n")
    if tamper:
        (subject/verifier_rel).write_text("# tampered after bundle commitment\n")
    if mixed:
        p=subject/"crates/product/src/lib.rs"; p.parent.mkdir(parents=True,exist_ok=True); p.write_text("pub fn mixed() {}\n")
    return bundle_rel,bundle_sha

def main() -> None:
    with tempfile.TemporaryDirectory() as td:
        td=Path(td)
        verifier=td/"verifier"; verifier.mkdir(); copy_current(verifier); init_repo(verifier); commit(verifier,"verifier")

        ordinary=td/"ordinary"; base=make_repo(ordinary); p=ordinary/"docs/example.txt"; p.parent.mkdir(parents=True); p.write_text("ok\n"); head=commit(ordinary,"ordinary")
        r=verify(verifier,ordinary,base,head)
        assert r["mode"]=="ordinary"

        control=td/"control-path"; base=make_repo(control)
        p=control/("docs/evil\nname.txt"); p.parent.mkdir(parents=True,exist_ok=True); p.write_text("ambiguous path\n")
        head=commit(control,"control path")
        expect_fail(lambda: verify(verifier,control,base,head),"control character in git path")

        shadow=td/"shadow"; base=make_repo(shadow); p=shadow/"qualification/qual-001/verifier_v0_2.py"; p.write_text(p.read_text()+"\n# shadow\n"); head=commit(shadow,"shadow")
        expect_fail(lambda: verify(verifier,shadow,base,head),"verifier shadow without rotation pointer")

        valid=td/"valid"; base=make_repo(valid); _,bundle_sha=write_successor(valid); head=commit(valid,"valid rotation")
        r=verify(verifier,valid,base,head)
        assert r["mode"]=="rotation_authorization"
        assert r["proposed_bundle_sha256"]==bundle_sha
        assert r["candidate_verifier_code_executed"] is False
        assert r["admin_bypass_qualifies"] is False

        wrong=td/"wrong"; base=make_repo(wrong); write_successor(wrong,wrong_predecessor=True); head=commit(wrong,"wrong predecessor")
        expect_fail(lambda: verify(verifier,wrong,base,head),"wrong predecessor")

        mixed=td/"mixed"; base=make_repo(mixed); write_successor(mixed,mixed=True); head=commit(mixed,"mixed product")
        expect_fail(lambda: verify(verifier,mixed,base,head),"mixed product path")

        tampered=td/"tampered"; base=make_repo(tampered); write_successor(tampered,tamper=True); head=commit(tampered,"tampered component")
        expect_fail(lambda: verify(verifier,tampered,base,head),"tampered successor component")

        launcher=td/"launcher"; base=make_repo(launcher)
        wf=launcher/".github/workflows/qual-001-authoritative.yml"; wf.write_text(wf.read_text()+"\n# mutate launcher\n")
        ptr=launcher/CURRENT_POINTER_REL; ptr.write_text(ptr.read_text()+" ")
        head=commit(launcher,"launcher mutation")
        expect_fail(lambda: verify(verifier,launcher,base,head),"launcher mutation")

    print("QUAL-001R rotation self-test: PASS")

if __name__ == "__main__":
    main()
