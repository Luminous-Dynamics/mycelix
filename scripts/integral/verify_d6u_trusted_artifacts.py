#!/usr/bin/env python3
"""Verify D6U evidence as inert data before trusted attestation."""

import base64
import hashlib
import json
import os
import re
import sys
import urllib.parse
import urllib.request
from pathlib import Path


ROOT = Path(__file__).parents[2]
POLICY = ROOT / "docs/integral/d6u-trusted-builder-policy.json"


def github_get(repo: str, api_path: str, token: str) -> dict:
    url = f"https://api.github.com/repos/{repo}{api_path}"
    request = urllib.request.Request(
        url,
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": "2022-11-28",
            "User-Agent": "mycelix-d6u-trusted-builder",
        },
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_tree_from_api(repo: str, ref: str, token: str) -> dict:
    return github_get(
        repo,
        f"/git/trees/{urllib.parse.quote(ref, safe='')}?recursive=1",
        token,
    )


def contents_bytes_from_api(repo: str, path: str, ref: str, token: str) -> bytes:
    payload = github_get(
        repo,
        f"/contents/{urllib.parse.quote(path, safe='/')}?ref={urllib.parse.quote(ref, safe='')}",
        token,
    )
    assert payload.get("encoding") == "base64", f"unexpected encoding for {path!r}"
    return base64.b64decode(payload["content"], validate=True)


def verify_required_tracked_blobs(tree_payload: dict, required: dict[str, str]) -> None:
    assert tree_payload.get("truncated") is False, (
        "GitHub Git tree response was truncated; refusing incomplete source identity"
    )

    observed: dict[str, str] = {}
    for entry in tree_payload.get("tree", []):
        path = entry.get("path")
        if path not in required:
            continue
        assert path not in observed, f"duplicate Git tree path: {path!r}"
        assert entry.get("type") == "blob", (
            f"trusted source path is not a regular Git blob: {path!r}"
        )
        assert entry.get("mode") in {"100644", "100755"}, (
            f"trusted source path has unexpected Git mode: {path!r}: {entry.get('mode')!r}"
        )
        sha = entry.get("sha")
        assert re.fullmatch(r"[0-9a-f]{40}", sha or ""), (
            f"Git tree returned no valid blob SHA for {path!r}"
        )
        observed[path] = sha

    assert set(observed) == set(required), (
        f"trusted source tree coverage mismatch: "
        f"missing={sorted(set(required) - set(observed))!r}"
    )
    mismatches = {
        path: {"expected": expected, "observed": observed[path]}
        for path, expected in required.items()
        if observed[path] != expected
    }
    assert not mismatches, f"trusted source blob mismatch: {mismatches!r}"


def verify_forbidden_cargo_config_paths(tree_payload: dict, forbidden_paths: list[str]) -> None:
    assert tree_payload.get("truncated") is False, (
        "GitHub Git tree response was truncated; refusing incomplete Cargo-config identity"
    )
    observed = {entry.get("path") for entry in tree_payload.get("tree", [])}
    forbidden = sorted(path for path in forbidden_paths if path in observed)
    assert not forbidden, f"source tree contains forbidden Cargo config: {forbidden!r}"


def verify_artifact_layout(artifact_dir: Path, expected_files: set[str]) -> None:
    assert artifact_dir.is_dir(), f"missing trusted artifact directory: {artifact_dir}"
    entries = list(artifact_dir.rglob("*"))
    symlinks = [p for p in entries if p.is_symlink()]
    assert not symlinks, f"trusted artifact contains symlink(s): {symlinks!r}"
    nested = [p for p in entries if p.is_dir()]
    assert not nested, f"trusted artifact contains nested directories: {nested!r}"
    special = [p for p in entries if not p.is_dir() and not p.is_file()]
    assert not special, f"trusted artifact contains special file(s): {special!r}"
    files = {
        p.relative_to(artifact_dir).as_posix()
        for p in entries
        if p.is_file()
    }
    assert files == expected_files, (
        f"unexpected trusted-input files: {sorted(files)!r}"
    )


def load_record(path: Path) -> dict[str, str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines and lines[0] == "D6U HOLOCHAIN 0.7 RUNTIME EVIDENCE"
    record: dict[str, str] = {}
    for line in lines[1:]:
        assert "=" in line, f"malformed evidence record line: {line!r}"
        key, value = line.split("=", 1)
        assert key and key not in record, f"duplicate evidence key: {key!r}"
        record[key] = value
    return record


def verify_executor_workflow_record(
    record: dict[str, str],
    policy: dict,
    repo: str,
) -> None:
    cfg = policy["executor_workflow"]
    assert record["executor_workflow_file_path"] == cfg["path"]
    assert record["executor_workflow_repository"] == repo
    assert re.fullmatch(r"[0-9a-f]{40}", record["executor_workflow_commit_sha"])
    assert record["executor_workflow_ref"] == (
        f"{repo}/{cfg['path']}@refs/heads/main"
    )


def verify_executor_run_record(
    executor_run: dict,
    record: dict[str, str],
    policy: dict,
    repo: str,
) -> None:
    cfg = policy["executor_workflow"]
    assert executor_run["name"] == cfg["name"]
    assert executor_run["path"] == cfg["path"]
    assert executor_run["event"] == "workflow_run"
    assert executor_run["conclusion"] == "success"
    assert executor_run["repository"]["full_name"] == repo
    assert executor_run["head_repository"]["full_name"] == repo
    assert executor_run["head_branch"] == "main"
    assert executor_run["id"] == int(record["executor_run_id"])
    assert executor_run["run_attempt"] == int(record["executor_run_attempt"])


def verify_executor_workflow_identity(
    record: dict[str, str],
    policy: dict,
    repo: str,
    token: str,
) -> None:
    verify_executor_workflow_record(record, policy, repo)
    tree = git_tree_from_api(repo, record["executor_workflow_commit_sha"], token)
    verify_required_tracked_blobs(
        tree,
        {policy["executor_workflow"]["path"]: policy["executor_workflow"]["blob_sha"]},
    )


def verify_trigger_run_record(
    record: dict[str, str],
    trigger: dict,
    policy: dict,
    repo: str,
) -> None:
    cfg = policy["trigger_workflow"]
    assert trigger["name"] == cfg["name"]
    assert trigger["path"] == cfg["path"]
    assert trigger["event"] == "pull_request"
    assert trigger["conclusion"] == "success"
    assert trigger["head_repository"]["full_name"] == repo
    assert trigger["head_branch"] == policy["source_branch"]
    assert trigger["run_attempt"] == int(record["trigger_workflow_run_attempt"])
    assert trigger["head_sha"] == record["source_commit"]
    assert record["trigger_workflow_name"] == trigger["name"]
    assert record["trigger_workflow_path"] == trigger["path"]
    assert record["source_repository"] == trigger["head_repository"]["full_name"]
    assert record["source_branch"] == trigger["head_branch"]
    assert record["trigger_workflow_blob_sha"] == (
        policy["required_source_blobs"][cfg["path"]]
    )


def verify_trigger_run(
    record: dict[str, str],
    policy: dict,
    repo: str,
    token: str,
) -> dict:
    trigger = github_get(
        repo,
        f"/actions/runs/{urllib.parse.quote(record['trigger_workflow_run_id'], safe='')}",
        token,
    )
    verify_trigger_run_record(record, trigger, policy, repo)
    source_tree = git_tree_from_api(repo, record["source_commit"], token)
    verify_forbidden_cargo_config_paths(
        source_tree,
        policy["forbidden_cargo_config_paths"],
    )
    verify_required_tracked_blobs(
        source_tree,
        policy["required_source_blobs"],
    )
    return trigger


def verify_cases(log: str, policy: dict) -> None:
    observed = {}
    for line in log.splitlines():
        if not line.startswith("D6U_CASE\t"):
            continue
        parts = line.split("\t")
        assert len(parts) == 5 and parts[4] == "PASS", f"malformed D6U_CASE: {line!r}"
        case_id, outcome, reachability = parts[1], parts[2], parts[3]
        assert case_id not in observed, f"duplicate D6U_CASE: {case_id!r}"
        assert reachability in {"zome-reached=true", "zome-reached=false"}
        observed[case_id] = {
            "outcome": outcome,
            "zome_reached": reachability == "zome-reached=true",
        }

    assert observed == policy["cases"], (
        f"D6U_CASE mismatch: observed={observed!r}"
    )

    expected_supplemental = policy["supplemental_substrate"]
    witnesses = {}
    checks = {}
    for line in log.splitlines():
        if line.startswith("D6U_RUNTIME_WITNESS\t"):
            parts = line.split("\t", 2)
            assert len(parts) == 3
            witness_id, witness = parts[1], parts[2]
            assert witness_id in expected_supplemental
            assert witness_id not in witnesses
            assert expected_supplemental[witness_id] in witness
            witnesses[witness_id] = witness
        elif line.startswith("D6U_SUBSTRATE_CHECK\t"):
            parts = line.split("\t")
            assert len(parts) == 4 and parts[3] == "PASS"
            check_id, reason = parts[1], parts[2]
            assert check_id in expected_supplemental
            assert check_id not in checks
            assert expected_supplemental[check_id] in reason
            checks[check_id] = reason

    assert set(witnesses) == set(expected_supplemental)
    assert checks == witnesses

    app = {}
    fragment = policy["application_check"]["fragment"]
    for line in log.splitlines():
        if line.startswith("D6U_APPLICATION_CHECK\t"):
            parts = line.split("\t")
            assert len(parts) == 4 and parts[3] == "PASS"
            check_id, evidence = parts[1], parts[2]
            assert check_id == policy["application_check"]["id"]
            assert check_id not in app
            assert fragment in evidence
            app[check_id] = evidence
    assert len(app) == 1


def verify_lock(path: Path, policy: dict) -> None:
    import tomllib

    lock = tomllib.loads(path.read_text(encoding="utf-8"))
    packages = lock.get("package", [])
    for name, version in policy["lock_packages"].items():
        matches = [p for p in packages if p.get("name") == name]
        assert matches, f"trusted lock missing {name!r}"
        assert {p.get("version") for p in matches} == {version}
        for package in matches:
            assert package.get("source") == policy["lock_source"]
            assert re.fullmatch(r"[0-9a-f]{64}", package.get("checksum", ""))


def main() -> None:
    assert len(sys.argv) == 2, "usage: verify_d6u_trusted_artifacts.py ARTIFACT_DIR"
    artifact_dir = Path(sys.argv[1]).resolve()
    event = json.loads(Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8"))
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    repo = os.environ["GITHUB_REPOSITORY"]
    token = os.environ["GITHUB_TOKEN"]
    executor_run = event["workflow_run"]

    assert event["repository"]["full_name"] == repo
    verify_executor_run_record(executor_run, {
        "executor_run_id": str(executor_run["id"]),
        "executor_run_attempt": str(executor_run["run_attempt"]),
    }, policy, repo)

    expected_files = {
        "d6u-runtime-evidence.txt",
        "d6u-runtime-test.log",
        "Cargo.lock",
    }
    verify_artifact_layout(artifact_dir, expected_files)

    evidence = artifact_dir / "d6u-runtime-evidence.txt"
    test_log = artifact_dir / "d6u-runtime-test.log"
    lockfile = artifact_dir / "Cargo.lock"
    record = load_record(evidence)

    assert set(record) == set(policy["record_fields"])
    assert record["status"] == "runtime-reference-evidence"
    assert record["workflow_run_id"] == str(executor_run["id"])
    assert record["workflow_run_attempt"] == str(executor_run["run_attempt"])
    assert record["executor_run_id"] == str(executor_run["id"])
    assert record["executor_run_attempt"] == str(executor_run["run_attempt"])
    assert record["attestation_status"] == "deferred-to-trusted-builder"
    assert record["claim_ceiling"] == policy["claim_ceiling"]

    verify_executor_workflow_identity(record, policy, repo, token)
    trigger = verify_trigger_run(record, policy, repo, token)

    assert record["source_commit"] == trigger["head_sha"]
    assert record["source_repository"] == repo

    assert record["test_log_sha256"] == sha256(test_log)
    assert record["cargo_lock_sha256"] == sha256(lockfile)

    verify_cases(test_log.read_text(encoding="utf-8"), policy)
    verify_lock(lockfile, policy)

    d6s1 = contents_bytes_from_api(
        repo,
        "docs/integral/d6s-canon-1-golden-vectors.json",
        record["source_commit"],
        token,
    )
    assert hashlib.sha256(d6s1).hexdigest() == policy["d6s1_corpus_sha256"]

    assert record["d6s2_manifest_git_blob_sha"] == policy["d6s2_manifest_git_blob_sha"]
    assert record["d6s2_fixture_git_blob_sha"] == policy["d6s2_fixture_git_blob_sha"]
    assert record["d6s1_corpus_sha256"] == policy["d6s1_corpus_sha256"]

    print(
        "verified D6U trusted-builder input: "
        f"executor_run={executor_run['id']}, "
        f"trigger_run={record['trigger_workflow_run_id']}, "
        f"source={record['source_commit']}"
    )


if __name__ == "__main__":
    main()
