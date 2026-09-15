#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import shutil
import tempfile
from pathlib import Path

import rotation_selftest_v0_1 as base
import verifier_monotonic_v0_2 as verifier

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
WRAPPER_REL = "qualification/qual-001/verifier_monotonic_v0_2.py"

base.v = verifier
_original_copy_current = base.copy_current


def copy_current(dst: Path) -> None:
    _original_copy_current(dst)
    out = dst / WRAPPER_REL
    out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(REPO / WRAPPER_REL, out)


base.copy_current = copy_current


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mutate_successor(subject: Path, *, gate_mut=None, policy_mut=None, bundle_mut=None) -> None:
    bundle_rel, _ = base.write_successor(subject)
    bundle_path = subject / bundle_rel
    bundle = json.loads(bundle_path.read_text())

    gate_path = subject / bundle["components"]["gate_manifest"]["path"]
    gate = json.loads(gate_path.read_text())
    current_gate = json.loads(
        (REPO / "qualification/qual-001/gate-manifest-v0.2.json").read_text()
    )
    gate["profile"] = current_gate["profile"]
    if gate_mut:
        gate_mut(gate)
    gate_path.write_text(json.dumps(gate, indent=2, sort_keys=True) + "\n")
    bundle["components"]["gate_manifest"]["sha256"] = digest(gate_path)

    policy_path = subject / bundle["components"]["rotation_policy"]["path"]
    policy = json.loads(policy_path.read_text())
    current_policy = json.loads(
        (REPO / "qualification/qual-001/rotation-policy-v0.1.json").read_text()
    )
    policy["profile"] = current_policy["profile"]
    if policy_mut:
        policy_mut(policy)
    policy_path.write_text(json.dumps(policy, indent=2, sort_keys=True) + "\n")
    bundle["components"]["rotation_policy"]["sha256"] = digest(policy_path)

    if bundle_mut:
        bundle_mut(bundle)
    bundle_path.write_text(json.dumps(bundle, indent=2, sort_keys=True) + "\n")

    pointer_path = subject / base.CURRENT_POINTER_REL
    pointer = json.loads(pointer_path.read_text())
    pointer["bundle_sha256"] = digest(bundle_path)
    pointer["profile"] = bundle["profile"]
    pointer_path.write_text(json.dumps(pointer, indent=2, sort_keys=True) + "\n")


def expect_fail(fn, label: str) -> None:
    try:
        fn()
    except RuntimeError:
        return
    raise AssertionError(f"expected fail-closed rejection: {label}")


def run_case(td: Path, name: str, *, gate_mut=None, policy_mut=None, bundle_mut=None, ok=False) -> None:
    subject = td / name
    start = base.make_repo(subject)
    mutate_successor(
        subject,
        gate_mut=gate_mut,
        policy_mut=policy_mut,
        bundle_mut=bundle_mut,
    )
    head = base.commit(subject, name)
    if ok:
        receipt = base.verify(td / "verifier", subject, start, head)
        assert receipt["mode"] == "rotation_authorization"
        assert receipt["proposed_profile"] == verifier.NEXT_BUNDLE_PROFILE
        assert receipt[
            "successor_candidate_verifier_code_executed_during_rotation_authorization"
        ] is False
        assert receipt["admin_bypass_qualifies"] is False
    else:
        expect_fail(lambda: base.verify(td / "verifier", subject, start, head), name)


def main() -> None:
    with tempfile.TemporaryDirectory() as temp:
        td = Path(temp)
        vr = td / "verifier"
        vr.mkdir()
        copy_current(vr)
        base.init_repo(vr)
        base.commit(vr, "verifier")

        run_case(td, "valid-fixed-contract-rotation", ok=True)

        cases = [
            ("gate-profile-substitution", "gate", lambda x: x.__setitem__("profile", x["profile"] + ".substitution")),
            ("policy-profile-substitution", "policy", lambda x: x.__setitem__("profile", x["profile"] + ".substitution")),
            ("bundle-profile-substitution", "bundle", lambda x: x.__setitem__("profile", "mycelix.qual.static-subject-independence.v9.9-unregistered")),
            ("gate-widen-paths", "gate", lambda x: x.__setitem__("max_changed_paths", x["max_changed_paths"] + 1)),
            ("gate-shrink-paths", "gate", lambda x: x.__setitem__("max_changed_paths", x["max_changed_paths"] - 1)),
            ("gate-widen-bytes", "gate", lambda x: x.__setitem__("max_changed_bytes", x["max_changed_bytes"] + 1)),
            ("gate-shrink-bytes", "gate", lambda x: x.__setitem__("max_changed_bytes", x["max_changed_bytes"] - 1)),
            ("gate-drop-base-receipt", "gate", lambda x: x["required_receipt_fields"].pop()),
            ("gate-add-base-receipt", "gate", lambda x: x["required_receipt_fields"].append("impossible_receipt_field")),
            ("gate-drop-rotation-receipt", "gate", lambda x: x["required_rotation_receipt_fields"].pop()),
            ("gate-add-rotation-receipt", "gate", lambda x: x["required_rotation_receipt_fields"].append("impossible_rotation_receipt_field")),
            ("gate-weaken-forbidden", "gate", lambda x: x["ordinary_forbidden_prefixes"].clear()),
            ("gate-overrestrict-forbidden", "gate", lambda x: x["ordinary_forbidden_prefixes"].append("docs/")),
            ("policy-widen-paths", "policy", lambda x: x.__setitem__("max_changed_paths", x["max_changed_paths"] + 1)),
            ("policy-shrink-paths", "policy", lambda x: x.__setitem__("max_changed_paths", x["max_changed_paths"] - 1)),
            ("policy-widen-bytes", "policy", lambda x: x.__setitem__("max_changed_bytes", x["max_changed_bytes"] + 1)),
            ("policy-shrink-bytes", "policy", lambda x: x.__setitem__("max_changed_bytes", x["max_changed_bytes"] - 1)),
            ("policy-broaden-prefix", "policy", lambda x: x["allowed_prefixes"].append("crates/")),
            ("policy-narrow-prefix", "policy", lambda x: x["allowed_prefixes"].pop()),
            ("policy-broaden-exact", "policy", lambda x: x["allowed_exact_paths"].append("README.md")),
            ("policy-remove-exact", "policy", lambda x: x["allowed_exact_paths"].clear()),
            ("policy-remove-immutability", "policy", lambda x: x["immutable_launcher_paths"].pop()),
            ("policy-self-brick-immutability", "policy", lambda x: x["immutable_launcher_paths"].append(x["current_pointer_path"])),
            ("policy-add-component", "policy", lambda x: x["required_successor_components"].append("impossible_component")),
            ("policy-remove-component", "policy", lambda x: x["required_successor_components"].pop()),
        ]

        for name, plane, mutation in cases:
            kwargs = {f"{plane}_mut": mutation}
            run_case(td, name, **kwargs)

    print("QUAL-001R fixed-contract successor self-test: PASS")


if __name__ == "__main__":
    main()
