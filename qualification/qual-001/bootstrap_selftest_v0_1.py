#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUAL = ROOT / "qualification" / "qual-001"
sys.path.insert(0, str(QUAL))

import verifier_v0_1 as verifier  # noqa: E402

EXPECTED_GATE_SHA = "60478ceec4b5e5c542a8a66d6e34f13c94eb2048a4f77773100347ba0e859b61"
EXPECTED_VERIFIER_SHA = "ef1cf59d7d96e5689fcfe1f4f873a7cfeefa3c365cd813bd019d7c26952f44d2"
EXPECTED_AUTH_WORKFLOW_SHA = "884f8ea58b5a21d21e9626d463ceabdd2ef01a2df83629a5922ec7945f619ac3"
EXPECTED_BUNDLE_SHA = "b0281549127d0aa892a347ebffd32c5d9e59f8b6c26b3a478db852dc4366a964"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def expect_reject(fn, message: str) -> None:
    try:
        fn()
    except verifier.VerificationError:
        return
    raise SystemExit(message)


def main() -> None:
    manifest_path = QUAL / "gate-manifest-v0.1.json"
    bundle_path = QUAL / "verifier-bundle-v0.1.json"
    verifier_path = QUAL / "verifier_v0_1.py"
    workflow_path = ROOT / ".github" / "workflows" / "qual-001-authoritative.yml"

    manifest, manifest_sha = verifier.load_gate_manifest(manifest_path)
    require(manifest_sha == EXPECTED_GATE_SHA, "gate-manifest digest drift")
    require(sha(verifier_path) == EXPECTED_VERIFIER_SHA, "verifier source digest drift")
    require(sha(workflow_path) == EXPECTED_AUTH_WORKFLOW_SHA, "authoritative workflow digest drift")

    bundle, bundle_sha, bundle_files = verifier.load_and_verify_bundle(
        ROOT,
        bundle_path,
        manifest["profile"],
    )
    require(bundle_sha == EXPECTED_BUNDLE_SHA, "verifier-bundle digest drift")
    require(bundle["schema"] == verifier.BUNDLE_SCHEMA, "verifier-bundle schema drift")
    require(
        bundle_files["qualification/qual-001/gate-manifest-v0.1.json"] == EXPECTED_GATE_SHA,
        "bundle gate-manifest commitment drift",
    )
    require(
        bundle_files["qualification/qual-001/verifier_v0_1.py"] == EXPECTED_VERIFIER_SHA,
        "bundle verifier commitment drift",
    )
    require(
        bundle_files[".github/workflows/qual-001-authoritative.yml"] == EXPECTED_AUTH_WORKFLOW_SHA,
        "bundle workflow commitment drift",
    )

    verifier.validate_changed_paths(
        [
            "qualification/qual-001-demo/subject.txt",
            "qualification/qual-001-demo/candidate_local_verifier.py",
            ".github/workflows/candidate-local-weakened.yml",
        ],
        manifest,
    )

    expect_reject(
        lambda: verifier.validate_changed_paths(
            ["qualification/qual-001/verifier_v0_1.py"], manifest
        ),
        "verifier-root shadowing was accepted",
    )
    expect_reject(
        lambda: verifier.validate_changed_paths(
            [".github/workflows/qual-001-authoritative.yml"], manifest
        ),
        "authoritative workflow shadowing was accepted",
    )
    expect_reject(
        lambda: verifier.validate_changed_paths(["../escape"], manifest),
        "path traversal was accepted",
    )
    expect_reject(
        lambda: verifier.validate_changed_paths(
            [f"subject/{index}.txt" for index in range(manifest["max_changed_paths"] + 1)],
            manifest,
        ),
        "changed-path exhaustion was accepted",
    )

    with tempfile.TemporaryDirectory() as tmp:
        subject = Path(tmp)
        (subject / "safe.txt").write_text("safe")
        verifier.changed_file_bytes(subject, ["safe.txt"], manifest["max_changed_bytes"])

        link = subject / "link.txt"
        try:
            link.symlink_to("/etc/passwd")
        except OSError:
            pass
        else:
            expect_reject(
                lambda: verifier.changed_file_bytes(
                    subject, ["link.txt"], manifest["max_changed_bytes"]
                ),
                "subject symlink was accepted",
            )

    with tempfile.TemporaryDirectory() as tmp:
        hostile_root = Path(tmp)
        for relative in (
            ".github/workflows/qual-001-authoritative.yml",
            "qualification/qual-001/gate-manifest-v0.1.json",
            "qualification/qual-001/verifier_v0_1.py",
            "qualification/qual-001/verifier-bundle-v0.1.json",
        ):
            source = ROOT / relative
            target = hostile_root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        tampered = hostile_root / "qualification/qual-001/verifier_v0_1.py"
        tampered.write_text(tampered.read_text() + "\n# hostile drift\n")
        expect_reject(
            lambda: verifier.load_and_verify_bundle(
                hostile_root,
                hostile_root / "qualification/qual-001/verifier-bundle-v0.1.json",
                manifest["profile"],
            ),
            "verifier-bundle accepted tampered verifier bytes",
        )

    sample = {
        "z": 1,
        "a": False,
        "nested": {"b": 2, "a": 1},
    }
    require(
        verifier.canonical_json(sample)
        == '{"a":false,"nested":{"a":1,"b":2},"z":1}',
        "canonical JSON drift",
    )

    source = verifier_path.read_text()
    for forbidden in ("shell=True", "os.system(", "eval(", "exec(", "importlib", "runpy"):
        require(forbidden not in source, f"dangerous verifier primitive present: {forbidden}")

    workflow = workflow_path.read_text()
    require("pull_request_target:" in workflow, "authoritative lane is not pull_request_target")
    require("permissions:\n  contents: read" in workflow, "authoritative token is not read-only")
    require("secrets." not in workflow, "authoritative lane references secrets")
    require(workflow.count("persist-credentials: false") >= 2, "checkout credentials may persist")
    require("path: verifier" in workflow and "path: subject" in workflow, "dual checkouts missing")
    require("ref: ${{ github.sha }}" in workflow, "verifier checkout is not base/default exact SHA")
    require(
        "ref: ${{ github.event.pull_request.head.sha }}" in workflow,
        "subject checkout is not exact head SHA",
    )
    require(
        "python3 -B verifier/qualification/qual-001/verifier_v0_1.py" in workflow,
        "workflow does not invoke verifier-owned implementation",
    )
    require(
        "--bundle-manifest verifier/qualification/qual-001/verifier-bundle-v0.1.json" in workflow,
        "workflow does not require verifier-bundle manifest",
    )
    for forbidden in ("cd subject", "bash subject", "python subject", "./subject"):
        require(forbidden not in workflow, f"authoritative lane executes subject code: {forbidden}")
    require(
        "Emit successful independent-verifier receipt" in workflow,
        "postflight receipt step missing",
    )

    adoption = json.loads(
        (ROOT / "mycelix-workspace" / "docs" / "agents" / "qual-001-bootstrap-v0.1.json").read_text()
    )
    require(adoption.get("candidate_owned_bootstrap") is True, "bootstrap ownership claim drift")
    require(
        adoption.get("default_branch_adoption_required") is True,
        "default-branch adoption requirement missing",
    )
    require(
        adoption.get("repository_protection_required") is True,
        "repository-protection requirement missing",
    )
    require(
        adoption.get("repository_protection_issue") == 938,
        "repository-protection issue binding drift",
    )
    require(
        adoption.get("independent_topology_active") is False,
        "bootstrap must not claim active independent topology",
    )

    print("QUAL-001A verifier bootstrap self-test: PASS")


if __name__ == "__main__":
    main()
