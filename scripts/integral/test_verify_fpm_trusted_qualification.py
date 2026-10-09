#!/usr/bin/env python3
"""Black-box adversarial corpus for the FPM reference verifier."""

from __future__ import annotations

import base64
import copy
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
import stat
import tempfile
import zipfile
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
TRUSTED_WORKFLOW_PATH = Path(__file__).parents[2] / ".github/workflows/fpm-trusted-qualification.yml"
TRUSTED_POLICY_WORKFLOW_PATH = Path(__file__).parents[2] / ".github/workflows/fpm-trusted-qualification.yml"
COLLECTOR = Path(__file__).with_name("collect_fpm_trusted_artifacts.py")
COLLECTOR_MODULE = "collect_fpm_trusted_artifacts"
MANIFEST = "crates/fpm-wasm-artifact-identity/Cargo.toml"
MANIFEST_SHA = "c94b53f61ed8a9bfb6249b1b339550dddd074d6c"

SUBJECT, TREE, BASE = "1" * 40, "2" * 40, "3" * 40
POLICY = "4" * 40
IVERIFY, IVERIFY_BLOB = "6" * 40, "7" * 40
LOCK_SHA = "8" * 64
CANDIDATE_RUN, TRUSTED_RUN = 1001, 2002
RECEIPT_ARTIFACT, INDEX_ARTIFACT = 3003, 4004
PR = 123
SYSTEM_CLOSURE_COMMANDS = ["bash", "env", "grep", "tr", "timeout", "cargo", "rustc", "rustfmt", "cc", "ld", "as", "ldd", "realpath", "readelf", "sha256sum", "sed", "uname", "cat", "find", "sort", "mkdir", "chmod", "rm", "ln"]


def cjson(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()


def write(path: Path, obj: object) -> None:
    path.write_bytes(cjson(obj) + b"\n")


def run(root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(SCRIPT), str(root)],
                          text=True, capture_output=True, check=False)


def write_artifact_zip(path: Path, member_name: str, data: bytes, duplicate: bool = False) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(member_name, data)
        if duplicate:
            archive.writestr(member_name, data)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(root: Path) -> None:
    policy_bytes = TRUSTED_POLICY_WORKFLOW_PATH.read_bytes()
    policy_text = policy_bytes.decode("utf-8")
    policy_blob_sha = hashlib.sha1(
        b"blob " + str(len(policy_bytes)).encode("ascii") + b"\0" + policy_bytes
    ).hexdigest()
    receipt = {
        "schema": "mycelix.fpm.trusted-qualification-receipt.v1",
        "qualification": "FPM-WASM-ARTIFACT-IDENTITY-V1",
        "repository": REPO, "repository_id": REPO_ID, "pr_number": str(PR),
        "subject_sha": SUBJECT, "subject_tree_sha": TREE,
        "observed_postflight_head_sha": SUBJECT,
        "observed_postflight_tree_sha": TREE, "base_sha": BASE,
        "trusted_policy_sha": POLICY, "trusted_policy_blob_sha": policy_blob_sha,
        "trusted_policy_ref": "refs/heads/main",
        "trusted_workflow_run_id": TRUSTED_RUN, "trusted_workflow_run_attempt": 1,
        # The production workflow passes these two upstream values through
        # environment variables, so their receipt wire types are strings.
        "upstream_workflow_run_id": str(CANDIDATE_RUN),
        "upstream_workflow_run_attempt": "1",
        "upstream_workflow_id": CW_ID, "upstream_workflow_path": CW_PATH,
        "upstream_workflow_conclusion": "success", "manifest_blob_sha": MANIFEST_SHA,
        "lock_mode": "generated_for_run", "lock_sha256": LOCK_SHA,
        "rustc_version": "rustc 1.96.1",
        "rustc_commit": "31fca3adb283cc9dfd56b49cdee9a96eb9c96ffd",
        "cargo_version": "cargo 1.96.1", "candidate_uid": 10001,
        "candidate_gid": 10001,
        "candidate_execution_profile": "fpm-docker-offline-v1",
        "sandbox_image_digest": "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5",
        "sandbox_probe": "passed",
        "sandbox_system_closure": {
            "profile": "fpm-debian12-bookworm-gcc13.4-amd64-rust-1.96.1-v1",
            "image_digest": "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5",
            "architecture": "linux/amd64",
            "os_release_sha256": "b" * 64,
            "libc_version": "ldd (Debian GLIBC 2.36) 2.36",
            "commands": [
                {"name": name, "path": ("/opt/fpm-rust/bin/" + name if name in {"cargo", "rustc", "rustfmt"} else "/usr/bin/" + name), "sha256": "c" * 64}
                for name in sorted(SYSTEM_CLOSURE_COMMANDS)
            ],
            "libraries": [
                {"path": "/lib/x86_64-linux-gnu/libc.so.6", "sha256": "d" * 64},
                {"path": "/lib64/ld-linux-x86-64.so.2", "sha256": "e" * 64},
            ],
        },
        "sandbox_system_closure_sha256": hashlib.sha256(
            cjson({
                "profile": "fpm-debian12-bookworm-gcc13.4-amd64-rust-1.96.1-v1",
                "image_digest": "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5",
                "architecture": "linux/amd64",
                "os_release_sha256": "b" * 64,
                "libc_version": "ldd (Debian GLIBC 2.36) 2.36",
                "commands": [
                    {"name": name, "path": ("/opt/fpm-rust/bin/" + name if name in {"cargo", "rustc", "rustfmt"} else "/usr/bin/" + name), "sha256": "c" * 64}
                    for name in sorted(SYSTEM_CLOSURE_COMMANDS)
                ],
                "libraries": [
                    {"path": "/lib/x86_64-linux-gnu/libc.so.6", "sha256": "d" * 64},
                    {"path": "/lib64/ld-linux-x86-64.so.2", "sha256": "e" * 64},
                ],
            })
        ).hexdigest(),
        "sandbox_target_closure": {
            "executables": [
                {"path": "/target/debug/deps/fpm_wasm_artifact_identity-1111111111111111", "sha256": "f" * 64},
                {"path": "/target/debug/deps/fpm_wasm_artifact_identity-2222222222222222", "sha256": "0" * 64},
            ],
            "libraries": [
                {"path": "/lib/x86_64-linux-gnu/libc.so.6", "sha256": "1" * 64},
                {"path": "/lib64/ld-linux-x86-64.so.2", "sha256": "2" * 64},
            ],
        },
        "sandbox_target_closure_sha256": hashlib.sha256(
            cjson({
                "executables": [
                    {"path": "/target/debug/deps/fpm_wasm_artifact_identity-1111111111111111", "sha256": "f" * 64},
                {"path": "/target/debug/deps/fpm_wasm_artifact_identity-2222222222222222", "sha256": "0" * 64},
                ],
                "libraries": [
                    {"path": "/lib/x86_64-linux-gnu/libc.so.6", "sha256": "1" * 64},
                    {"path": "/lib64/ld-linux-x86-64.so.2", "sha256": "2" * 64},
                ],
            })
        ).hexdigest(),
        "dependency_cache_sha256": "a" * 64,        "dependency_source_policy": "crates-io-registry-only-v1",
        "steps": {k: "success" for k in
                  ("preflight", "checkout", "source", "toolchain", "lock", "dependencies", "sandbox_image", "fmt", "sandbox_probe", "compile", "tests", "postflight")},
        "execution_pass": True,
        "procedure_trust": "trusted_default_branch_snapshot",
        "promotion_authority": "pending_repository_governance_evidence",
    }
    receipt_bytes = cjson(receipt) + b"\n"
    receipt_archive_digest = write_artifact_zip(
        root / "raw/receipt.zip",
        "qualification-receipt.json",
        receipt_bytes,
    )
    receipt_digest = hashlib.sha256(cjson(receipt)).hexdigest()
    index = {
        "schema": "mycelix.fpm.trusted-qualification-artifact-index.v1",
        "receipt_sha256": receipt_digest,
        "artifact": {
            "id": RECEIPT_ARTIFACT, "sha256_hex": receipt_archive_digest,
            "url": f"https://github.com/{REPO}/actions/runs/{TRUSTED_RUN}/artifacts/{RECEIPT_ARTIFACT}",
            "retention_days": 90, "immutable_after_upload": True,
            "deletion_by_repository_writer_possible": True,
        },
        "subject_sha": SUBJECT, "subject_tree_sha": TREE,
        "trusted_policy_sha": POLICY, "trusted_policy_blob_sha": policy_blob_sha,
        "trusted_workflow_run_id": TRUSTED_RUN,
    }
    index_bytes = cjson(index) + b"\n"
    index_archive_digest = write_artifact_zip(
        root / "raw/index.zip",
        "artifact-binding-index.json",
        index_bytes,
    )
    artifacts = {"artifacts": [
        {"id": RECEIPT_ARTIFACT, "name": f"fpm-trusted-qualification-{SUBJECT}",
         "expired": False, "created_at": "2026-10-07T20:00:00Z", "expires_at": "2027-01-05T20:00:00Z", "size_in_bytes": (root / "raw/receipt.zip").stat().st_size, "digest": f"sha256:{receipt_archive_digest}",
         "workflow_run": {"id": TRUSTED_RUN, "repository_id": REPO_ID, "head_repository_id": REPO_ID, "head_sha": POLICY, "head_branch": "main"}},
        {"id": INDEX_ARTIFACT, "name": f"fpm-trusted-qualification-index-{SUBJECT}",
         "expired": False, "created_at": "2026-10-07T20:00:01Z", "expires_at": "2027-01-05T20:00:01Z", "size_in_bytes": (root / "raw/index.zip").stat().st_size, "digest": f"sha256:{index_archive_digest}",
         "workflow_run": {"id": TRUSTED_RUN, "repository_id": REPO_ID, "head_repository_id": REPO_ID, "head_sha": POLICY, "head_branch": "main"}},
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
    policy = {
        "path": TW_PATH,
        "sha": policy_blob_sha,
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
        if src.is_dir():
            shutil.copytree(src, td / src.name)
        else:
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


def load_reference_verifier():
    spec = importlib.util.spec_from_file_location("fpm_reference_verifier_under_test", SCRIPT)
    if spec is None or spec.loader is None:
        raise AssertionError("unable to load reference verifier module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assert_policy_mutation_rejected(verifier, base: Path, label: str, mutator, expected_fragment=None) -> None:
    policy = json.loads((base / "policy-file.json").read_text(encoding="utf-8"))
    original = base64.b64decode(policy["content"], validate=True).decode("utf-8")
    changed = mutator(original)
    raw = changed.encode("utf-8")
    policy["content"] = base64.b64encode(raw).decode("ascii")
    policy["sha"] = hashlib.sha1(
        f"blob {len(raw)}".encode("ascii") + bytes([0]) + raw
    ).hexdigest()
    try:
        verifier.verify_sandbox_policy(policy, "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5")
    except SystemExit as exc:
        message = str(exc)
        assert "trusted policy content does not match its Git blob SHA" not in message, (
            f"policy mutation did not reach semantic validation: {label}: {message}"
        )
        if expected_fragment is not None:
            assert expected_fragment in message, (
                f"policy mutation failed for the wrong reason: {label}: {message}"
            )
    else:
        raise AssertionError(f"policy mutation unexpectedly passed semantic validation: {label}")


def assert_evidence_archive_contract() -> None:
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    trusted_workflow = TRUSTED_WORKFLOW_PATH.read_text(encoding="utf-8")
    marker = "      - name: Normalize downloaded raw evidence archives\n"
    next_marker = "      - name: Extract evidence targets with strict JSON parser\n"
    assert workflow.count(next_marker) == 1
    assert marker in workflow
    block = workflow.split(marker, 1)[1].split(next_marker, 1)[0]
    assert "root.iterdir()" in block
    assert "len(members) != 1" in block
    assert "source.is_file()" in block
    assert "source.is_symlink()" in block
    assert "skip-decompress: true" in workflow
    assert "Validate and materialize raw evidence members" in workflow
    assert "verify_raw_artifact_archive" in workflow
    verifier = Path(__file__).with_name("verify_fpm_trusted_qualification.py").read_text(encoding="utf-8")
    assert "actual_steps = {step for step, _ in invocations}" in verifier
    assert "Cargo fmt inside immutable sandbox" in verifier
    assert "Execute precompiled test harness inside immutable sandbox" in verifier
    assert "Compile test artifacts inside immutable offline sandbox" in verifier
    assert "dst=/target,readonly" in verifier or "dst=/target,readonly" in workflow
    assert "selection[\"receipt_artifact_id\"]" in workflow
    assert "selection[\"index_artifact_id\"]" in workflow
    assert "snapshot/raw/receipt.zip" in workflow
    assert block.index("verify_raw_artifact_archive") < block.index("Path(\"snapshot/qualification-receipt.json\")")
    assert "find snapshot/download" not in workflow
    assert "-print -quit" not in workflow
    assert "sandbox-system-closure.tsv" in trusted_workflow
    assert "FPM_SYSTEM_CLOSURE_PROFILE" in trusted_workflow
    assert trusted_workflow.count("export PATH=/opt/fpm-rust/bin:/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin") == 3
    selftest_workflow = (Path(__file__).parents[2] / ".github/workflows/fpm-reference-verifier-selftest.yml").read_text(encoding="utf-8")
    assert "Parse every embedded sandbox program" in selftest_workflow
    assert "expected exactly four embedded sandbox programs" in selftest_workflow
    assert 'bash -n "$script"' in selftest_workflow
    assert 'shellcheck --severity=error "$script"' in selftest_workflow
    assert "FPM_SANDBOX_IMAGE: gcc@sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5" in trusted_workflow
    assert trusted_workflow.count('-ceu "$(cat <<\'FPM_SANDBOX_SCRIPT\'') == 4
    assert trusted_workflow.count("\n          FPM_SANDBOX_SCRIPT\n") == 4
    assert "-ceu '" not in trusted_workflow
    assert '|| ""' not in trusted_workflow
    assert '"sandbox_system_closure_sha256": hashlib.sha256(' in trusted_workflow
    assert '"sandbox_target_closure_sha256": hashlib.sha256(' in trusted_workflow
    assert "FPM_SYSTEM_CLOSURE_PROFILE: fpm-debian12-bookworm-gcc13.4-amd64-rust-1.96.1-v1" in trusted_workflow
    assert "rustc --crate-name fpm_toolchain_probe --edition 2024 -C linker=cc" in trusted_workflow
    assert "FPM_SANDBOX_IMAGE: gcc@{expected_image_digest}" in verifier
    assert "/usr/local/lib64/" in verifier
    assert "required_commands=(bash env grep tr timeout cargo rustc rustfmt cc ld as ldd realpath readelf sha256sum sed uname cat find sort mkdir chmod rm ln)" in trusted_workflow
    assert "/usr/local/bin/*|/usr/local/sbin/*|/usr/bin/*" in trusted_workflow
    assert "FPM_SANDBOX_IMAGE: gcc@sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5" in trusted_workflow
    assert "queue=()" in trusted_workflow
    assert 'while [ "$queue_index" -lt "${#queue[@]}" ]; do' in trusted_workflow
    assert "native library closure exceeds 256 unique libraries" in trusted_workflow
    assert "done < /tmp/fpm-ldd-queue" not in trusted_workflow
    assert "sandbox_system_closure" in verifier
    assert "def require_canonical_absolute_path" in verifier
    assert "posixpath.normpath(value) != value" in verifier
    assert '[A-Za-z0-9._/+:-]+' in verifier
    assert "sandbox_system_closure.command-path-traversal" in Path(__file__).read_text(encoding="utf-8")
    assert "sandbox_target_closure.executable-control-char" in Path(__file__).read_text(encoding="utf-8")
    assert "noncanonical system closure line: {line!r}" in trusted_workflow
    assert "noncanonical target closure line: {line!r}" in trusted_workflow
    assert "compiled test executable is absent from closure" in trusted_workflow
    assert "native dependency is absent from closure" in trusted_workflow
    assert '[[ "$path" =~ ^/[A-Za-z0-9._/+:-]+$ ]]' in trusted_workflow
    assert 'test "$actual_executable_count" -eq "$declared_executable_count"' in trusted_workflow
    assert "rustc --crate-name fpm_toolchain_probe --edition 2024 -C linker=cc" in verifier
    assert "/target/.fpm-toolchain-probe" in verifier
    assert "if len(invocations) != len(expected_steps) or actual_steps != expected_steps:" in verifier
    assert "cargo test --locked --offline --no-run --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml" in trusted_workflow
    assert "readelf -l \"$resolved\"" in trusted_workflow
    assert "test native library closure exceeds 512 unique libraries" in trusted_workflow
    assert 'target_closure_path = os.environ["TARGET_CLOSURE_FILE"]' in trusted_workflow
    assert "if os.path.isfile(target_closure_path)" in trusted_workflow
    assert "chmod -R a-w /target" not in trusted_workflow
    assert "--mount type=bind,src=\"${TARGET_CLOSURE_FILE}\",dst=/tmp/fpm-target-closure.tsv,readonly" in trusted_workflow
    assert workflow.count("- name: Extract evidence targets with strict JSON parser") == 1
    assert "root_config_hits=\"$(git ls-files --stage -- .cargo/config .cargo/config.toml || true)\"" in trusted_workflow
    assert 'config_ancestor="$(pwd -P)"' in trusted_workflow
    assert 'LOCK_HOME="$RUNNER_TEMP/fpm-lock-home"' in trusted_workflow
    assert 'GIT_CONFIG_GLOBAL=/dev/null' in trusted_workflow
    assert 'env -i \\\n              PATH="$PATH" \\\n              HOME="$HOME" \\\n              CARGO_HOME="$CARGO_HOME"' in trusted_workflow
    assert "cargo generate-lockfile --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml" in trusted_workflow
    assert 'test ! -L "${CARGO_HOME}/config.toml"' in trusted_workflow
    assert 'while :; do' in trusted_workflow
    assert 'cargo_config_dir="${config_ancestor}/.cargo"' in trusted_workflow
    assert 'test ! -L "$cargo_config_dir"' in trusted_workflow
    assert 'config_path="${cargo_config_dir}/${config_name}"' in trusted_workflow
    assert '[ -e "$config_path" ] || [ -L "$config_path" ]' in trusted_workflow
    assert 'config_ancestor="$(dirname -- "$config_ancestor")"' in trusted_workflow
    assert trusted_workflow.count("test ! -e /.cargo/config\n") == 4
    assert trusted_workflow.count("test ! -e /.cargo/config.toml") == 4
    assert 'if "patch" in lock or "replace" in lock:' in trusted_workflow
    assert 'test ! -e "$CARGO_HOME/config"' in trusted_workflow
    assert 'test ! -L "$CARGO_HOME/config.toml"' in trusted_workflow
    assert "config_hit=\"$(git ls-files | grep -E" not in trusted_workflow
    


def assert_artifact_collector_http_contract() -> None:
    collector = COLLECTOR.read_text(encoding="utf-8")
    assert "?per_page={PAGE_SIZE}&page={page}&direction=asc" in collector
    assert '"-f"' not in collector
    assert "enumerate_consistent_artifacts(get_page)" in collector


def assert_trusted_workflow_context_scope() -> None:
    workflow = Path(__file__).parents[2].joinpath(".github/workflows/fpm-trusted-qualification.yml").read_text(encoding="utf-8")
    env_start = workflow.index("env:\n")
    jobs_start = workflow.index("jobs:\n", env_start)
    top_env = workflow[env_start:jobs_start]
    assert "${{ runner." not in top_env
    assert "printf 'FPM_CARGO_HOME=%s/fpm-cargo\\n' \"$RUNNER_TEMP\" >> \"$GITHUB_ENV\"" in workflow
    assert "printf 'FPM_EVIDENCE_DIR=%s/fpm-trusted-evidence-%s\\n' \"$RUNNER_TEMP\" \"$GITHUB_RUN_ID\" >> \"$GITHUB_ENV\"" in workflow
    lock_start = workflow.index("      - name: Generate run-local locked dependency snapshot\n")
    dependency_start = workflow.index("      - name: Materialize dependency cache before hostile execution\n", lock_start)
    lock_step = workflow[lock_start:dependency_start]
    assert 'export HOME="$RUNNER_TEMP/fpm-lock-home"' in lock_step
    assert "export GIT_CONFIG_GLOBAL=/dev/null" in lock_step
    assert "export GIT_CONFIG_NOSYSTEM=true" in lock_step
    assert "export GIT_TERMINAL_PROMPT=0" in lock_step
    assert "export CARGO_REGISTRIES_CRATES_IO_PROTOCOL=sparse" in lock_step
    assert "export CARGO_NET_OFFLINE=false" in lock_step
    assert "export CARGO_NET_GIT_FETCH_WITH_CLI=false" in lock_step
    assert lock_step.index('export GIT_CONFIG_GLOBAL=/dev/null') < lock_step.index("cargo generate-lockfile")
    assert 'env -i \\\n' in lock_step
    assert 'CARGO_HOME="$CARGO_HOME" \\\n' in lock_step
    assert "CARGO_REGISTRIES_CRATES_IO_PROTOCOL=sparse" in lock_step
    assert "CARGO_NET_OFFLINE=false" in lock_step
    assert "CARGO_NET_GIT_FETCH_WITH_CLI=false" in lock_step
    assert "GIT_CONFIG_GLOBAL=/dev/null" in lock_step
    assert "GIT_CONFIG_NOSYSTEM=true" in lock_step
    assert "GIT_TERMINAL_PROMPT=0" in lock_step
    assert lock_step.index("env -i") < lock_step.index("cargo generate-lockfile")
    assert 'test ! -e "$HOME/.gitconfig"' in lock_step
    assert 'test ! -e "${CARGO_HOME}/config.toml"' in lock_step
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
    assert_evidence_archive_contract()
    assert_workflow_target_extractor_dependencies()
    assert_trusted_workflow_context_scope()
    with tempfile.TemporaryDirectory(prefix="fpm-ref-corpus-") as td:
        root = Path(td)
        snapshot(root)
        baseline = run(root)
        assert baseline.returncode == 0, baseline.stderr + baseline.stdout

        import importlib.util

        verifier_spec = importlib.util.spec_from_file_location("fpm_reference_verifier", SCRIPT)
        assert verifier_spec and verifier_spec.loader
        verifier = importlib.util.module_from_spec(verifier_spec)
        verifier_spec.loader.exec_module(verifier)

        promotion_pr = {
            "state": "open",
            "draft": False,
            "head": {"sha": SUBJECT},
            "base": {"sha": BASE},
        }
        promotion_receipt = {"subject_sha": SUBJECT, "lock_mode": "generated_for_run"}
        promotion_trusted_run = {"head_sha": BASE}
        assert verifier.is_current_promotion_eligible(
            promotion_pr, promotion_receipt, promotion_trusted_run, BASE
        ) is False
        promotion_receipt["lock_mode"] = "tracked"
        assert verifier.is_current_promotion_eligible(
            promotion_pr, promotion_receipt, promotion_trusted_run, BASE
        ) is True

        lock_base = b"""version = 4

[[package]]
name = "fpm-wasm-artifact-identity"
version = "0.1.0"
dependencies = ["holo_hash"]

[[package]]
name = "holo_hash"
version = "0.7.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"

[[package]]
name = "serde"
version = "1.0.0"
source = "registry+https://github.com/rust-lang/crates.io-index"
checksum = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
"""
        verifier.verify_tracked_lock_source_policy(lock_base)
        lock_mutations = [
            ("git-source", lock_base.replace(
                b'registry+https://github.com/rust-lang/crates.io-index',
                b'git+https://example.invalid/repository#abcdef',
                1,
            )),
            ("alternate-registry", lock_base.replace(
                b'registry+https://github.com/rust-lang/crates.io-index',
                b'registry+https://registry.example.invalid/index',
                1,
            )),
            ("missing-checksum", lock_base.replace(
                b'checksum = "' + b'b' * 64 + b'"',
                b'checksum = "not-a-checksum"',
                1,
            )),
            ("extra-source-free", lock_base + b'''
[[package]]
name = "local-helper"
version = "1.0.0"
'''),
        ]
        for label, lock_bytes in lock_mutations:
            try:
                verifier.verify_tracked_lock_source_policy(lock_bytes)
            except SystemExit:
                pass
            else:
                raise AssertionError(f"tracked Cargo.lock mutation was accepted: {label}")

        with tempfile.TemporaryDirectory(prefix="fpm-zip-corpus-") as zip_td:
            zroot = Path(zip_td)
            good_data = b'{"qualification":"ok"}\n'
            good_zip = zroot / "good.zip"
            good_digest = write_artifact_zip(good_zip, "qualification-receipt.json", good_data)
            extracted = zroot / "qualification-receipt.json"
            extracted.write_bytes(good_data)
            verified_archive = verifier.verify_raw_artifact_archive(
                good_zip, "qualification-receipt.json", f"sha256:{good_digest}", good_zip.stat().st_size, extracted, "receipt"
            )
            assert verified_archive["member_count"] == 1
            assert verified_archive["member_names"] == ["qualification-receipt.json"]
            assert verified_archive["member_sha256"] == hashlib.sha256(good_data).hexdigest()
            assert verified_archive["member_set_sha256"] == hashlib.sha256(
                b'["qualification-receipt.json"]'
            ).hexdigest()

            bad_cases = [
                ("duplicate-member", lambda p: write_artifact_zip(p, "qualification-receipt.json", good_data, duplicate=True)),
            ]

            unexpected = zroot / "unexpected.zip"
            with zipfile.ZipFile(unexpected, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                archive.writestr("qualification-receipt.json", good_data)
                archive.writestr("unexpected.json", b"{}")
            bad_cases.append(("unexpected-member", lambda p: None))

            traversal = zroot / "traversal.zip"
            traversal.parent.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(traversal, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                archive.writestr("../qualification-receipt.json", good_data)
            bad_cases.append(("traversal-member", lambda p: None))

            symlink = zroot / "symlink.zip"
            info = zipfile.ZipInfo("qualification-receipt.json")
            info.external_attr = (stat.S_IFLNK | 0o777) << 16
            with zipfile.ZipFile(symlink, "w") as archive:
                archive.writestr(info, good_data)
            bad_cases.append(("symlink-member", lambda p: None))

            for label, make_case in bad_cases:
                if label == "duplicate-member":
                    archive_path = zroot / "duplicate.zip"
                    digest = make_case(archive_path)  # type: ignore[misc]
                elif label == "unexpected-member":
                    archive_path = unexpected
                    digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()
                else:
                    archive_path = traversal if label == "traversal-member" else symlink
                    digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()
                try:
                    verifier.verify_raw_artifact_archive(
                        archive_path, "qualification-receipt.json", f"sha256:{digest}", archive_path.stat().st_size, extracted, "receipt"
                    )
                except SystemExit:
                    pass
                else:
                    raise AssertionError(f"raw archive mutation was accepted: {label}")

            extracted.write_bytes(b"tampered\n")
            try:
                verifier.verify_raw_artifact_archive(
                    good_zip, "qualification-receipt.json", f"sha256:{good_digest}", good_zip.stat().st_size, extracted, "receipt"
                )
            except SystemExit:
                pass
            else:
                raise AssertionError("materialized evidence mismatch was accepted")

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
            ("receipt.dependency_source_policy", lambda x: x.__setitem__("dependency_source_policy", "unrestricted")),
            ("receipt.execution_pass", lambda x: x.__setitem__("execution_pass", False)),
            ("receipt.promotion_authority", lambda x: x.__setitem__("promotion_authority", "authorized")),
        ]
        for label, fn in receipt:
            expect_failure(root, "qualification-receipt.json", label, fn)

        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.commitment",
            lambda x: x.__setitem__("sandbox_system_closure_sha256", "0" * 64),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.commitment",
            lambda x: x.__setitem__("sandbox_target_closure_sha256", "0" * 64),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.executable-order",
            lambda x: x["sandbox_target_closure"]["executables"].reverse(),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.library-order",
            lambda x: x["sandbox_target_closure"]["libraries"].reverse(),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.executable-escape",
            lambda x: x["sandbox_target_closure"]["executables"][0].__setitem__("path", "/tmp/evil"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.library-escape",
            lambda x: x["sandbox_target_closure"]["libraries"][0].__setitem__("path", "/tmp/libevil.so"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.profile",
            lambda x: x["sandbox_system_closure"].__setitem__("profile", "untrusted"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.image_digest",
            lambda x: x["sandbox_system_closure"].__setitem__("image_digest", "sha256:" + "f" * 64),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.os_release_sha256",
            lambda x: x["sandbox_system_closure"].__setitem__("os_release_sha256", "not-a-sha"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.command-missing",
            lambda x: x["sandbox_system_closure"]["commands"].pop(),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.command-digest",
            lambda x: x["sandbox_system_closure"]["commands"][0].__setitem__("sha256", "bad"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.library-order",
            lambda x: x["sandbox_system_closure"]["libraries"].reverse(),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.library-duplicate",
            lambda x: x["sandbox_system_closure"]["libraries"].append(copy.deepcopy(x["sandbox_system_closure"]["libraries"][0])),
        )

        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.command-path-escape",
            lambda x: x["sandbox_system_closure"]["commands"][0].__setitem__("path", "/tmp/escape"),
        )

        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.command-path-traversal",
            lambda x: x["sandbox_system_closure"]["commands"][0].__setitem__("path", "/usr/bin/../bin/as"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.library-path-traversal",
            lambda x: x["sandbox_system_closure"]["libraries"][0].__setitem__("path", "/lib/../lib/x86_64-linux-gnu/libc.so.6"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.executable-path-traversal",
            lambda x: x["sandbox_target_closure"]["executables"][0].__setitem__("path", "/target/debug/deps/../deps/fpm_wasm_artifact_identity-1111111111111111"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.library-path-traversal",
            lambda x: x["sandbox_target_closure"]["libraries"][0].__setitem__("path", "/lib/../lib/x86_64-linux-gnu/libc.so.6"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.executable-shell-metacharacter",
            lambda x: x["sandbox_target_closure"]["executables"][0].__setitem__("path", "/target/debug/deps/$(id)"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.executable-backslash",
            lambda x: x["sandbox_target_closure"]["executables"][0].__setitem__("path", "/target/debug/deps/fpm\\evil"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_target_closure.executable-control-char",
            lambda x: x["sandbox_target_closure"]["executables"][0].__setitem__("path", "/target/debug/deps/fpm\nattack"),
        )

        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.command-path-duplicate",
            lambda x: x["sandbox_system_closure"]["commands"][1].__setitem__(
                "path", x["sandbox_system_closure"]["commands"][0]["path"]
            ),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.library-path-escape",
            lambda x: x["sandbox_system_closure"]["libraries"][0].__setitem__("path", "/tmp/libevil.so"),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.missing-libc",
            lambda x: x["sandbox_system_closure"]["libraries"].pop(0),
        )
        expect_failure(
            root, "qualification-receipt.json", "receipt.sandbox_system_closure.missing-loader",
            lambda x: x["sandbox_system_closure"]["libraries"].pop(),
        )

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

        verifier_module = load_reference_verifier()

        def mutate_only_fmt_network(text: str) -> str:
            start = text.index("      - name: Cargo fmt inside immutable sandbox")
            end = text.index("      - name: Probe hostile-code sandbox boundary", start)
            block = text[start:end]
            assert "--network none" in block
            return text[:start] + block.replace("--network none", "--network host", 1) + text[end:]

        assert_policy_mutation_rejected(
            verifier_module, root, "policy.fmt-network-only", mutate_only_fmt_network,
            "exact allowlisted profile",
        )

        def mutate_policy_content_without_sha(value):
            decoded = base64.b64decode(value["content"], validate=True)
            value["content"] = base64.b64encode(decoded + b" ").decode("ascii")

        expect_failure(
            root,
            "policy-file.json",
            "policy.blob-content-mismatch",
            mutate_policy_content_without_sha,
        )

        policy_cases = [
            ("--network none", "--network host", "policy.network", "exact allowlisted profile"),
            ("--read-only", "--security-opt no-new-privileges", "policy.read-only", "exact allowlisted profile"),
            ("--cap-drop ALL", "--cap-drop NET_RAW", "policy.cap-drop", "exact allowlisted profile"),
            ("--security-opt no-new-privileges", "--privileged", "policy.no-new-privileges", "exact allowlisted profile"),
            ("--pids-limit 512", "--pids-limit 4096", "policy.pids-limit", "exact allowlisted profile"),
            ("--memory 6g", "--memory 64g", "policy.memory", "exact allowlisted profile"),
            ("--cpus 2", "--cpus 64", "policy.cpus", "exact allowlisted profile"),
            ("CARGO_NET_OFFLINE=true", "CARGO_NET_OFFLINE=false", "policy.offline", "enables Cargo network access"),
            ("cargo test --locked --offline --no-run --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "cargo test --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "policy.cargo-offline", "missing required invariant"),
            ("cargo fmt --check --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "cargo fmt --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml", "policy.rustfmt", "missing required invariant"),
        ]
        for old, new, label, expected_fragment in policy_cases:
            def mutate_policy(text: str, old=old, new=new) -> str:
                assert old in text
                return text.replace(old, new, 1)
            assert_policy_mutation_rejected(
                verifier_module, root, label, mutate_policy, expected_fragment
            )

        def mutate_extra_host_mount(text: str) -> str:
            lines = text.splitlines()
            for index, line in enumerate(lines):
                if 'src="${FPM_TOOLCHAIN_ROOT}"' in line and "dst=/opt/fpm-rust,readonly" in line:
                    assert line.rstrip().endswith(chr(92))
                    lines.insert(index + 1, '            --mount type=bind,src="/",dst=/host,readonly ' + chr(92))
                    return "\n".join(lines) + "\n"
            raise AssertionError("could not locate trusted toolchain mount")

        assert_policy_mutation_rejected(
            verifier_module, root, "policy.extra-host-mount", mutate_extra_host_mount,
            "exact allowlisted profile",
        )

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

        # JSON numbers must match the schema's integer type exactly.
        # Python equality alone accepts 100.0 == 100 and True == 1.
        expect_failure(
            root, "artifact-enumeration.json", "enumeration.page-size-float",
            lambda x: x.__setitem__("page_size", 100.0),
        )
        expect_failure(
            root, "artifact-enumeration.json", "enumeration.terminal-page-bool",
            lambda x: x.__setitem__("terminal_page", True),
        )
        expect_failure(
            root, "artifact-enumeration.json", "enumeration.total-count-float",
            lambda x: x.__setitem__("total_count_reported", 2.0),
        )
        expect_failure(
            root, "artifact-enumeration.json", "enumeration.repeat-count-float",
            lambda x: x.__setitem__("repeat_total_count_reported", 2.0),
        )

        # Receipt and GitHub API run identities must use their documented
        # wire types; equality through int()/bool coercion is not sufficient.
        strict_identity_mutations = [
            ("qualification-receipt.json", "receipt.trusted-run-id-float",
             lambda x: x.__setitem__("trusted_workflow_run_id", float(TRUSTED_RUN))),
            ("qualification-receipt.json", "receipt.trusted-run-attempt-bool",
             lambda x: x.__setitem__("trusted_workflow_run_attempt", True)),
            ("qualification-receipt.json", "receipt.upstream-run-id-int",
             lambda x: x.__setitem__("upstream_workflow_run_id", CANDIDATE_RUN)),
            ("qualification-receipt.json", "receipt.upstream-run-attempt-float",
             lambda x: x.__setitem__("upstream_workflow_run_attempt", 1.0)),
            ("qualification-receipt.json", "receipt.candidate-uid-bool",
             lambda x: x.__setitem__("candidate_uid", True)),
            ("candidate-run.json", "candidate.run-id-float",
             lambda x: x.__setitem__("id", float(CANDIDATE_RUN))),
            ("candidate-run.json", "candidate.run-attempt-bool",
             lambda x: x.__setitem__("run_attempt", True)),
            ("trusted-run.json", "trusted.run-attempt-bool",
             lambda x: x.__setitem__("run_attempt", True)),
            ("artifacts.json", "artifact.id-float",
             lambda x: x["artifacts"][0].__setitem__("id", float(RECEIPT_ARTIFACT))),
            ("artifacts.json", "artifact.size-float",
             lambda x: x["artifacts"][0].__setitem__("size_in_bytes", float(x["artifacts"][0]["size_in_bytes"]))),
            ("artifacts.json", "artifact.run-id-float",
             lambda x: x["artifacts"][0]["workflow_run"].__setitem__("id", float(TRUSTED_RUN))),
            ("artifacts.json", "artifact.repository-id-bool",
             lambda x: x["artifacts"][0]["workflow_run"].__setitem__("repository_id", True)),
        ]
        for file_name, label, mutator in strict_identity_mutations:
            expect_failure(root, file_name, label, mutator)

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

        expect_failure(
            root, "artifacts.json", "artifact-size-mismatch",
            lambda x: x["artifacts"][0].__setitem__(
                "size_in_bytes", x["artifacts"][0]["size_in_bytes"] + 1
            ),
        )
        expect_failure(
            root, "artifacts.json", "artifact-run-head-sha",
            lambda x: x["artifacts"][0]["workflow_run"].__setitem__("head_sha", "a" * 40),
        )
        expect_failure(
            root, "artifacts.json", "artifact-run-branch",
            lambda x: x["artifacts"][0]["workflow_run"].__setitem__("head_branch", "feature"),
        )
        def expect_duplicate_snapshot_key(file_name: str, marker: bytes, duplicate: bytes) -> None:
            original = (root / file_name).read_bytes()
            assert marker in original, f"duplicate-key fixture marker absent in {file_name}"
            mutated = original.replace(marker, duplicate, 1)
            fresh_root = Path(tempfile.mkdtemp(prefix="fpm-ref-snapshot-duplicate-"))
            try:
                for src in root.iterdir():
                    if src.is_file():
                        (fresh_root / src.name).write_bytes(src.read_bytes())
                (fresh_root / file_name).write_bytes(mutated)
                result = run(fresh_root)
                assert result.returncode != 0, f"duplicate key accepted in {file_name}"
                assert "duplicate JSON key" in result.stderr, (
                    f"duplicate key in {file_name} failed for an unrelated reason: {result.stderr}"
                )
            finally:
                shutil.rmtree(fresh_root)

        expect_duplicate_snapshot_key(
            "trusted-run.json", b'"id":2002', b'"id":2002,"id":2002'
        )
        expect_duplicate_snapshot_key(
            "candidate-run.json", b'"run_attempt":1',
            b'"run_attempt":1,"run_attempt":1'
        )
        expect_duplicate_snapshot_key(
            "artifacts.json", b'"id":3003', b'"id":3003,"id":3003'
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
