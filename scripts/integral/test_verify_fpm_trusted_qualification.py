#!/usr/bin/env python3
"""Black-box adversarial corpus for the FPM reference verifier."""

from __future__ import annotations

import base64
import copy
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT = Path(__file__).with_name("verify_fpm_trusted_qualification.py")
REPO = "Luminous-Dynamics/mycelix"
REPO_ID = 1176351975
CW_ID = 377461322
CW_PATH = ".github/workflows/fpm-wasm-artifact-identity.yml"
TW_NAME = "FPM trusted qualification policy"
TW_PATH = ".github/workflows/fpm-trusted-qualification.yml"
IW_PATH = ".github/workflows/fpm-trusted-qualification-independent-verify.yml"
WORKFLOW_PATH = Path(__file__).parents[2] / ".github/workflows/fpm-trusted-qualification-independent-verify.yml"
COLLECTOR = Path(__file__).with_name("collect_fpm_trusted_artifacts.py")
COLLECTOR_MODULE = "collect_fpm_trusted_artifacts"
MANIFEST = "crates/fpm-wasm-artifact-identity/Cargo.toml"
MANIFEST_SHA = "c94b53f61ed8a9bfb6249b1b339550dddd074d6c"

SUBJECT, TREE, BASE = "1" * 40, "2" * 40, "3" * 40
POLICY, POLICY_BLOB = "4" * 40, "5" * 40
IVERIFY, IVERIFY_BLOB = "6" * 40, "7" * 40
LOCK_SHA, ARTIFACT_SHA = "8" * 64, "9" * 64
CANDIDATE_RUN, TRUSTED_RUN = 1001, 2002
RECEIPT_ARTIFACT, INDEX_ARTIFACT = 3003, 4004
PR = 123


def cjson(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()


def write(path: Path, obj: object) -> None:
    path.write_bytes(cjson(obj) + b"\n")


def run(root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(SCRIPT), str(root)],
                          text=True, capture_output=True, check=False)


def snapshot(root: Path) -> None:
    receipt = {
        "schema": "mycelix.fpm.trusted-qualification-receipt.v1",
        "qualification": "FPM-WASM-ARTIFACT-IDENTITY-V1",
        "repository": REPO, "repository_id": REPO_ID, "pr_number": str(PR),
        "subject_sha": SUBJECT, "subject_tree_sha": TREE,
        "observed_postflight_head_sha": SUBJECT,
        "observed_postflight_tree_sha": TREE, "base_sha": BASE,
        "trusted_policy_sha": POLICY, "trusted_policy_blob_sha": POLICY_BLOB,
        "trusted_policy_ref": "refs/heads/main",
        "trusted_workflow_run_id": TRUSTED_RUN, "trusted_workflow_run_attempt": 1,
        "upstream_workflow_run_id": CANDIDATE_RUN, "upstream_workflow_run_attempt": 1,
        "upstream_workflow_id": CW_ID, "upstream_workflow_path": CW_PATH,
        "upstream_workflow_conclusion": "success", "manifest_blob_sha": MANIFEST_SHA,
        "lock_mode": "generated_for_run", "lock_sha256": LOCK_SHA,
        "rustc_version": "rustc 1.96.1",
        "rustc_commit": "31fca3adb283cc9dfd56b49cdee9a96eb9c96ffd",
        "cargo_version": "cargo 1.96.1", "candidate_uid": 10001,
        "candidate_gid": 10001,
        "candidate_execution_profile": "fpm-docker-offline-v1",
        "sandbox_image_digest": "sha256:f610ab94648195aa356059f5b41d6085c9d4d903c072430cdd1af7bdb646106b",
        "sandbox_probe": "passed",
        "dependency_cache_sha256": "a" * 64,
        "steps": {k: "success" for k in
                  ("preflight", "checkout", "source", "toolchain", "lock", "dependencies", "sandbox_image", "fmt", "sandbox_probe", "tests", "postflight")},
        "execution_pass": True,
        "procedure_trust": "trusted_default_branch_snapshot",
        "promotion_authority": "pending_repository_governance_evidence",
    }
    receipt_digest = hashlib.sha256(cjson(receipt)).hexdigest()
    index = {
        "schema": "mycelix.fpm.trusted-qualification-artifact-index.v1",
        "receipt_sha256": receipt_digest,
        "artifact": {
            "id": RECEIPT_ARTIFACT, "sha256_hex": ARTIFACT_SHA,
            "url": f"https://github.com/{REPO}/actions/runs/{TRUSTED_RUN}/artifacts/{RECEIPT_ARTIFACT}",
            "retention_days": 90, "immutable_after_upload": True,
            "deletion_by_repository_writer_possible": True,
        },
        "subject_sha": SUBJECT, "subject_tree_sha": TREE,
        "trusted_policy_sha": POLICY, "trusted_policy_blob_sha": POLICY_BLOB,
        "trusted_workflow_run_id": TRUSTED_RUN,
    }
    artifacts = {"artifacts": [
        {"id": RECEIPT_ARTIFACT, "name": f"fpm-trusted-qualification-{SUBJECT}",
         "expired": False, "created_at": "2026-10-07T20:00:00Z", "expires_at": "2027-01-05T20:00:00Z", "size_in_bytes": 1, "digest": f"sha256:{ARTIFACT_SHA}",
         "workflow_run": {"id": TRUSTED_RUN, "repository_id": REPO_ID, "head_repository_id": REPO_ID}},
        {"id": INDEX_ARTIFACT, "name": f"fpm-trusted-qualification-index-{SUBJECT}",
         "expired": False, "created_at": "2026-10-07T20:00:01Z", "expires_at": "2027-01-05T20:00:01Z", "size_in_bytes": 1, "digest": f"sha256:{ARTIFACT_SHA}",
         "workflow_run": {"id": TRUSTED_RUN, "repository_id": REPO_ID, "head_repository_id": REPO_ID}},
    ]}
    trusted = {"id": TRUSTED_RUN, "name": TW_NAME, "path": TW_PATH, "event": "workflow_run",
               "status": "completed", "conclusion": "success", "run_attempt": 1,
               "head_sha": POLICY, "head_branch": "main", "repository": {"id": REPO_ID, "full_name": REPO}}
    candidate = {"id": CANDIDATE_RUN, "workflow_id": CW_ID, "path": CW_PATH, "event": "pull_request",
                 "conclusion": "success", "run_attempt": 1,
                 "head_repository": {"id": REPO_ID, "full_name": REPO}, "head_sha": SUBJECT}
    pr = {"number": PR, "state": "closed", "draft": False,
          "base": {"ref": "main", "sha": BASE, "repo": {"id": REPO_ID}},
          "head": {"sha": SUBJECT, "repo": {"id": REPO_ID, "full_name": REPO}}}
    commit = {"sha": SUBJECT, "commit": {"tree": {"sha": TREE}}}
    policy_text = "FPM_SANDBOX_IMAGE: ubuntu@sha256:f610ab94648195aa356059f5b41d6085c9d4d903c072430cdd1af7bdb646106b\n--network none\n--read-only\n--cap-drop ALL\n--security-opt no-new-privileges\n--pids-limit 512\n--memory 6g\n--memory-swap 6g\n--cpus 2\n--mount type=bind,src=\"${GITHUB_WORKSPACE}/candidate\",dst=/candidate,readonly\n--mount type=bind,src=\"${FPM_TOOLCHAIN_ROOT}\",dst=/opt/fpm-rust,readonly\n--mount type=bind,src=\"${FPM_CARGO_HOME}\",dst=/cargo-ro,readonly\n--mount type=bind,src=\"${FPM_TARGET_DIR}\",dst=/target\n--user \"${CANDIDATE_UID}:${CANDIDATE_GID}\"\nCARGO_NET_OFFLINE=true\ncargo test --locked --offline --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml\ncargo fmt --check --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml\n"
    policy = {
        "path": TW_PATH,
        "sha": POLICY_BLOB,
        "encoding": "base64",
        "content": base64.b64encode(policy_text.encode()).decode(),
    }
    manifest = {"path": MANIFEST, "sha": MANIFEST_SHA}
    control = {"repository": REPO, "repository_id": REPO_ID, "path": IW_PATH,
               "ref": "refs/heads/main",
               "workflow_ref": f"{REPO}/{IW_PATH}@refs/heads/main",
               "workflow_sha": IVERIFY, "workflow_blob_sha": IVERIFY_BLOB,
               "reference_verifier_path": "scripts/integral/verify_fpm_trusted_qualification.py",
               "reference_verifier_blob_sha": "8" * 40,
               "artifact_collector_path": "scripts/integral/collect_fpm_trusted_artifacts.py",
               "artifact_collector_blob_sha": "9" * 40}
    files = {
        "qualification-receipt.json": receipt, "artifact-binding-index.json": index,
        "artifacts.json": artifacts, "trusted-run.json": trusted,
        "candidate-run.json": candidate, "pull-request.json": pr,
        "subject-commit.json": commit, "manifest.json": manifest,
        "policy-file.json": policy, "verifier-control.json": control,
        "candidate-lock.json": {"mode": "generated_for_run"},
        "main-ref.json": {"ref": "refs/heads/main", "object": {"sha": BASE}},
    }
    identities = [{"id": item["id"], "name": item["name"]} for item in artifacts["artifacts"]]
    enumeration = {
        "schema": "mycelix.fpm.trusted-qualification-artifact-enumeration.v1",
        "page_size": 100,
        "max_pages": 4,
        "max_artifacts": 256,
        "total_count_reported": 2,
        "enumerated_count": 2,
        "page_counts": [2],
        "terminal_page": 1,
        "artifact_identity_sha256": hashlib.sha256(cjson(identities)).hexdigest(),
        "complete": True,
        "repeat_enumeration_verified": True,
        "repeat_total_count_reported": 2,
        "repeat_page_counts": [2],
        "repeat_artifact_identity_sha256": hashlib.sha256(cjson(identities)).hexdigest(),
    }
    write(root / "artifact-enumeration.json", enumeration)

    for name, value in files.items():
        write(root / name, value)


def mutated_case(base: Path, target: str, mutator) -> Path:
    td = Path(tempfile.mkdtemp(prefix="fpm-ref-negative-"))
    for src in base.iterdir():
        (td / src.name).write_bytes(src.read_bytes())
    value = json.loads((td / target).read_text(encoding="utf-8"))
    mutator(value)
    write(td / target, value)
    return td


def expect_failure(base: Path, target: str, label: str, mutator) -> None:
    root = mutated_case(base, target, mutator)
    try:
        result = run(root)
        assert result.returncode != 0, f"mutation unexpectedly verified: {label}"
    finally:
        shutil.rmtree(root)


def assert_evidence_normalization_contract() -> None:
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    marker = "      - name: Normalize downloaded evidence\n"
    next_marker = "      - name: Extract evidence targets with strict JSON parser\n"
    assert marker in workflow and next_marker in workflow
    block = workflow.split(marker, 1)[1].split(next_marker, 1)[0]
    assert "root.rglob(\"*\")" in block
    assert "members != [expected_name]" in block
    assert "source.is_file()" in block
    assert "source.is_symlink()" in block
    assert "find snapshot/download" not in block
    assert "-print -quit" not in block


def assert_artifact_collector_http_contract() -> None:
    collector = COLLECTOR.read_text(encoding="utf-8")
    assert "?per_page={PAGE_SIZE}&page={page}&direction=asc" in collector
    assert '"-f"' not in collector
    assert "enumerate_consistent_artifacts(get_page)" in collector


def assert_workflow_target_extractor_dependencies() -> None:
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    marker = "      - name: Extract evidence targets with strict JSON parser\n"
    next_marker = "      - name: Resolve evidence-linked GitHub objects\n"
    assert marker in workflow and next_marker in workflow
    block = workflow.split(marker, 1)[1].split(next_marker, 1)[0]
    assert "import os" in block, "strict target extractor must import os for GITHUB_OUTPUT"
    assert "with open(os.environ[\"GITHUB_OUTPUT\"]" in block


def main() -> None:
    assert_artifact_collector_http_contract()
    assert_artifact_collector_http_contract()
    assert_evidence_normalization_contract()
    assert_workflow_target_extractor_dependencies()
    with tempfile.TemporaryDirectory(prefix="fpm-ref-corpus-") as td:
        root = Path(td)
        snapshot(root)
        baseline = run(root)
        assert baseline.returncode == 0, baseline.stderr + baseline.stdout

        receipt = [
            ("receipt.repository_id", lambda x: x.__setitem__("repository_id", REPO_ID + 1)),
            ("receipt.subject_sha", lambda x: x.__setitem__("subject_sha", "a" * 40)),
            ("receipt.trusted_policy_sha", lambda x: x.__setitem__("trusted_policy_sha", "b" * 40)),
            ("receipt.upstream_workflow_run_id", lambda x: x.__setitem__("upstream_workflow_run_id", CANDIDATE_RUN + 1)),
            ("receipt.manifest_blob_sha", lambda x: x.__setitem__("manifest_blob_sha", "c" * 40)),
            ("receipt.lock_sha256", lambda x: x.__setitem__("lock_sha256", "d" * 64)),
            ("receipt.candidate_uid", lambda x: x.__setitem__("candidate_uid", 10002)),
            ("receipt.candidate_gid", lambda x: x.__setitem__("candidate_gid", 10002)),
            ("receipt.sandbox_image_digest", lambda x: x.__setitem__("sandbox_image_digest", "sha256:" + "b" * 64)),
            ("receipt.sandbox_probe", lambda x: x.__setitem__("sandbox_probe", "failed")),
            ("receipt.dependency_cache_sha256", lambda x: x.__setitem__("dependency_cache_sha256", "b" * 64)),
            ("receipt.execution_pass", lambda x: x.__setitem__("execution_pass", False)),
            ("receipt.promotion_authority", lambda x: x.__setitem__("promotion_authority", "authorized")),
        ]
        for label, fn in receipt:
            expect_failure(root, "qualification-receipt.json", label, fn)

        expect_failure(
            root, "qualification-receipt.json", "receipt.steps.tests",
            lambda x: x["steps"].__setitem__("tests", "failure"),
        )

        external = [
            ("candidate-run.json", "candidate-run.head_sha", lambda x: x.__setitem__("head_sha", "a" * 40)),
            ("candidate-run.json", "candidate-run.workflow_id", lambda x: x.__setitem__("workflow_id", CW_ID + 1)),
            ("trusted-run.json", "trusted-run.head_sha", lambda x: x.__setitem__("head_sha", "a" * 40)),
            ("trusted-run.json", "trusted-run.conclusion", lambda x: x.__setitem__("conclusion", "failure")),
            ("pull-request.json", "pull-request.head.sha",
             lambda x: x["head"].__setitem__("sha", "a" * 40)),
            ("subject-commit.json", "subject-commit.tree",
             lambda x: x["commit"]["tree"].__setitem__("sha", "a" * 40)),
            ("manifest.json", "manifest.sha", lambda x: x.__setitem__("sha", "a" * 40)),
            ("policy-file.json", "policy-file.sha", lambda x: x.__setitem__("sha", "a" * 40)),
        ]
        for file_name, label, fn in external:
            expect_failure(root, file_name, label, fn)

        policy_cases = [
            ("--network none", "--network host", "policy.network"),
            ("--read-only", "--security-opt no-new-privileges", "policy.read-only"),
            ("--cap-drop ALL", "--cap-drop NET_RAW", "policy.cap-drop"),
            ("--security-opt no-new-privileges", "--privileged", "policy.no-new-privileges"),
            ("--pids-limit 512", "--pids-limit 4096", "policy.pids-limit"),
            ("--memory 6g", "--memory 64g", "policy.memory"),
            ("--cpus 2", "--cpus 64", "policy.cpus"),
            ("CARGO_NET_OFFLINE=true", "CARGO_NET_OFFLINE=false", "policy.offline"),
            ("cargo test --locked --offline --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "cargo test --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "policy.cargo-offline"),
            ("cargo fmt --check --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "cargo fmt --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "policy.rustfmt"),
        ]
        for old, new, label in policy_cases:
            def mutate_policy(value, old=old, new=new):
                decoded = base64.b64decode(value["content"], validate=True).decode()
                assert old in decoded
                value["content"] = base64.b64encode(decoded.replace(old, new, 1).encode()).decode()
            expect_failure(root, "policy-file.json", label, mutate_policy)

        expect_failure(
            root, "verifier-control.json", "control.reference-verifier-blob",
            lambda x: x.__setitem__("reference_verifier_blob_sha", "a" * 40),
        )
        expect_failure(
            root, "verifier-control.json", "control.collector-blob",
            lambda x: x.__setitem__("artifact_collector_blob_sha", "b" * 40),
        )
        expect_failure(
            root, "verifier-control.json", "control.extra-field",
            lambda x: x.__setitem__("unexpected", True),
        )

        expect_failure(
            root, "artifacts.json", "artifact-set-extra",
            lambda x: x["artifacts"].append(copy.deepcopy(x["artifacts"][0])),
        )
        expect_failure(
            root, "artifact-binding-index.json", "index.receipt_sha256",
            lambda x: x.__setitem__("receipt_sha256", "0" * 64),
        )
        expect_failure(
            root, "artifact-binding-index.json", "index.artifact.id",
            lambda x: x["artifact"].__setitem__("id", RECEIPT_ARTIFACT + 1),
        )
        expect_failure(
            root, "artifacts.json", "artifact-expiry-order",
            lambda x: x["artifacts"][0].__setitem__("expires_at", "2026-10-07T19:59:59Z"),
        )

        raw = (root / "qualification-receipt.json").read_bytes()
        duplicate = raw[:-1].replace(
            b',"subject_sha":"' + SUBJECT.encode() + b'"',
            b',"subject_sha":"' + SUBJECT.encode() + b'","subject_sha":"' + SUBJECT.encode() + b'"',
            1,
        ) + b"\n"
        (root / "qualification-receipt.json").write_bytes(duplicate)
        assert run(root).returncode != 0, "duplicate JSON key accepted"

        def expect_nonstandard_constant(value: str) -> None:
            raw = (root / "qualification-receipt.json").read_bytes()
            marker = b'"repository_id":1176351975'
            assert marker in raw
            mutated = raw.replace(marker, b'"repository_id":' + value.encode(), 1)
            fresh_root = Path(tempfile.mkdtemp(prefix="fpm-ref-constant-"))
            try:
                for src in root.iterdir():
                    (fresh_root / src.name).write_bytes(src.read_bytes())
                (fresh_root / "qualification-receipt.json").write_bytes(mutated)
                result = run(fresh_root)
                assert result.returncode != 0, f"{value} was accepted"
                assert "non-standard JSON constant" in result.stderr
            finally:
                shutil.rmtree(fresh_root)

        expect_nonstandard_constant("NaN")
        expect_nonstandard_constant("Infinity")
        expect_nonstandard_constant("-Infinity")

        fresh = mutated_case(
            root,
            "pull-request.json",
            lambda x: (x.__setitem__("state", "open"), x.__setitem__("draft", False)),
        )
        try:
            (fresh / "main-ref.json").write_bytes(
                cjson({"ref": "refs/heads/main", "object": {"sha": BASE}}) + b"\n"
            )
            result = run(fresh)
            assert result.returncode == 0, result.stderr + result.stdout
            verified = json.loads(result.stdout)
            assert verified["historical_qualification_valid"] is True
            assert verified["current_promotion_eligible"] is False, (
                "stale trusted-policy revision must not make a current open PR eligible"
            )
        finally:
            shutil.rmtree(fresh)

        snapshot(root)
        raw = (root / "qualification-receipt.json").read_bytes()
        (root / "qualification-receipt.json").write_bytes(b" " + raw)
        assert run(root).returncode != 0, "noncanonical receipt accepted"

    with tempfile.TemporaryDirectory(prefix="fpm-artifact-enum-corpus-") as td:
        fixtures = Path(td) / "fixtures"
        fixtures.mkdir()
        all_items = []
        for artifact_id in range(1, 102):
            all_items.append({
                "id": artifact_id,
                "name": (
                    f"fpm-trusted-qualification-{SUBJECT}" if artifact_id == 100
                    else f"unexpected-artifact-{artifact_id}"
                ),
            })
        all_items[100]["name"] = f"fpm-trusted-qualification-index-{SUBJECT}"
        write(fixtures / "page-1.json", {"total_count": 101, "artifacts": all_items[:100]})
        write(fixtures / "page-2.json", {"total_count": 101, "artifacts": all_items[100:]})
        output = Path(td) / "artifacts.json"
        enumeration = Path(td) / "enumeration.json"
        result = subprocess.run(
            [sys.executable, str(COLLECTOR), "--fixtures-dir", str(fixtures),
             "--artifacts-out", str(output), "--enumeration-out", str(enumeration)],
            text=True, capture_output=True, check=False,
        )
        assert result.returncode == 0, result.stderr + result.stdout
        collected = json.loads(output.read_text())["artifacts"]
        meta = json.loads(enumeration.read_text())
        assert len(collected) == 101
        assert meta["page_counts"] == [100, 1]
        assert meta["terminal_page"] == 2
        assert meta["total_count_reported"] == 101
        assert any(item["id"] == 101 for item in collected), "artifact beyond first page was lost"

        duplicate_fixtures = Path(td) / "duplicate-fixtures"
        duplicate_fixtures.mkdir()
        write(duplicate_fixtures / "page-1.json", {"total_count": 101, "artifacts": all_items[:100]})
        duplicate = dict(all_items[0])
        write(duplicate_fixtures / "page-2.json", {"total_count": 101, "artifacts": [duplicate]})
        result = subprocess.run(
            [sys.executable, str(COLLECTOR), "--fixtures-dir", str(duplicate_fixtures),
             "--artifacts-out", str(output), "--enumeration-out", str(enumeration)],
            text=True, capture_output=True, check=False,
        )
        assert result.returncode != 0, "duplicate artifact identity crossed pages without rejection"

    import importlib.util

    spec = importlib.util.spec_from_file_location(COLLECTOR_MODULE, COLLECTOR)
    assert spec and spec.loader
    collector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(collector)

    class Counter:
        value = 0

    counter = Counter()
    def divergent_page(page):
        counter.value += 1
        second_pass = counter.value > 1
        item_id = 2 if not second_pass else 3
        if page == 1:
            return 2, [
                {"id": 1, "name": "artifact-a"},
                {"id": item_id, "name": "artifact-b"},
            ]
        return 2, []

    try:
        collector.enumerate_consistent_artifacts(divergent_page)
    except SystemExit as exc:
        assert "identity sequence changed" in str(exc)
    else:
        raise AssertionError("divergent repeat enumeration was accepted")


    print("FPM reference-verifier adversarial corpus: PASS")


if __name__ == "__main__":
    main()
