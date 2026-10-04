#!/usr/bin/env python3
"""Verify D6U artifacts as untrusted data before trusted attestation.

This verifier is intentionally maintained on the default branch. It never imports
or executes code from the pull-request source tree.
"""

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


def api_get(repo: str, path: str, ref: str, token: str) -> dict:
    url = (
        f"https://api.github.com/repos/{repo}/contents/"
        f"{urllib.parse.quote(path, safe='/')}?ref={urllib.parse.quote(ref, safe='')}"
    )
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


def git_blob_from_api(repo: str, path: str, ref: str, token: str) -> str:
    payload = api_get(repo, path, ref, token)
    observed = payload.get("sha")
    assert re.fullmatch(r"[0-9a-f]{40}", observed or ""), (
        f"GitHub contents API returned no valid blob SHA for {path!r}"
    )
    return observed


def file_bytes_from_api(repo: str, path: str, ref: str, token: str) -> bytes:
    payload = api_get(repo, path, ref, token)
    assert payload.get("encoding") == "base64", f"unexpected encoding for {path!r}"
    return base64.b64decode(payload["content"], validate=True)


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

    expected = policy["cases"]
    assert observed == expected, f"D6U_CASE mismatch: observed={observed!r}"

    expected_supplemental = policy["supplemental_substrate"]
    witnesses = {}
    checks = {}
    for line in log.splitlines():
        if line.startswith("D6U_RUNTIME_WITNESS\t"):
            parts = line.split("\t", 2)
            assert len(parts) == 3, f"malformed runtime witness: {line!r}"
            witness_id, witness = parts[1], parts[2]
            assert witness_id in expected_supplemental
            assert witness_id not in witnesses
            assert expected_supplemental[witness_id] in witness, (
                f"runtime witness mismatch for {witness_id!r}"
            )
            witnesses[witness_id] = witness
        elif line.startswith("D6U_SUBSTRATE_CHECK\t"):
            parts = line.split("\t")
            assert len(parts) == 4 and parts[3] == "PASS", (
                f"malformed substrate check: {line!r}"
            )
            check_id, reason = parts[1], parts[2]
            assert check_id in expected_supplemental
            assert check_id not in checks
            assert expected_supplemental[check_id] in reason
            checks[check_id] = reason

    assert set(witnesses) == set(expected_supplemental)
    assert checks == witnesses

    application = {}
    fragment = policy["application_check"]["fragment"]
    for line in log.splitlines():
        if not line.startswith("D6U_APPLICATION_CHECK\t"):
            continue
        parts = line.split("\t")
        assert len(parts) == 4 and parts[3] == "PASS"
        check_id, evidence = parts[1], parts[2]
        assert check_id == policy["application_check"]["id"]
        assert check_id not in application
        assert fragment in evidence
        application[check_id] = evidence

    assert len(application) == 1


def verify_lock(path: Path, policy: dict) -> None:
    import tomllib

    lock = tomllib.loads(path.read_text(encoding="utf-8"))
    packages = lock.get("package", [])
    expected_packages = policy["lock_packages"]
    expected_source = policy["lock_source"]

    for name, version in expected_packages.items():
        matches = [p for p in packages if p.get("name") == name]
        assert matches, f"trusted lock missing {name!r}"
        assert {p.get("version") for p in matches} == {version}, (
            f"trusted lock version mismatch for {name!r}"
        )
        for package in matches:
            assert package.get("source") == expected_source
            assert re.fullmatch(r"[0-9a-f]{64}", package.get("checksum", "")), (
                f"trusted lock checksum malformed for {name!r}"
            )


def main() -> None:
    assert len(sys.argv) == 2, "usage: verify_d6u_trusted_artifacts.py ARTIFACT_DIR"
    artifact_dir = Path(sys.argv[1]).resolve()
    event = json.loads(Path(os.environ["GITHUB_EVENT_PATH"]).read_text(encoding="utf-8"))
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    assert policy["origin_policy"] == "same-repository-only"

    repo = os.environ["GITHUB_REPOSITORY"]
    token = os.environ["GITHUB_TOKEN"]
    run = event["workflow_run"]

    assert run["event"] == "pull_request"
    assert run["conclusion"] == "success"
    assert run["name"] == policy["workflow_name"]
    assert run["path"] == policy["workflow_path"]
    assert run["head_repository"]["full_name"] == repo
    assert event["repository"]["full_name"] == repo

    run_id = str(run["id"])
    run_attempt = str(run["run_attempt"])
    head_sha = run["head_sha"]
    assert re.fullmatch(r"[0-9a-f]{40}", head_sha)

    expected_files = {
        "d6u-runtime-evidence.txt",
        "d6u-runtime-test.log",
        "Cargo.lock",
    }
    files = {
        p.relative_to(artifact_dir).as_posix()
        for p in artifact_dir.rglob("*")
        if p.is_file() and not p.is_symlink()
    }
    assert files == expected_files, f"unexpected trusted-input files: {sorted(files)!r}"

    evidence = artifact_dir / "d6u-runtime-evidence.txt"
    test_log = artifact_dir / "d6u-runtime-test.log"
    lockfile = artifact_dir / "Cargo.lock"
    record = load_record(evidence)

    assert set(record) == set(policy["record_fields"])
    assert record["status"] == "runtime-reference-evidence"
    assert record["source_commit"] == head_sha
    assert record["workflow_run_id"] == run_id
    assert record["workflow_run_attempt"] == run_attempt
    assert record["workflow_execution_ref"].startswith(
        f"{repo}/{policy['workflow_path']}@"
    )
    assert record["attestation_status"] == "deferred-to-trusted-builder"
    assert record["claim_ceiling"] == policy["claim_ceiling"]
    assert record["manifest_version"] == str(policy["manifest_version"])
    assert record["d6s2_authority_ledger_schema"] == policy["d6s2_authority_ledger_schema"]
    assert record["case_coverage"] == policy["expected_case_coverage"]
    assert record["supplemental_coverage"] == policy["expected_supplemental_coverage"]
    assert record["application_check_coverage"] == policy["expected_application_check_coverage"]
    assert record["case_outcome_classes"] == ",".join(policy["expected_case_outcome_classes"])
    assert record["runtime"] == f"holochain-{policy['runtime']['holochain']}"
    assert record["hdk"] == policy["runtime"]["hdk"]
    assert record["hdi"] == policy["runtime"]["hdi"]
    assert record["test"] == "d6u_authority_boundary:passed"
    assert record["supported_cases"] == str(len(policy["cases"]))
    assert record["unsupported_cases"] == ",".join(policy["unsupported_reference_cases"])

    assert record["test_log_sha256"] == sha256(test_log)
    assert record["cargo_lock_sha256"] == sha256(lockfile)

    verify_cases(test_log.read_text(encoding="utf-8"), policy)
    verify_lock(lockfile, policy)

    for path, expected_blob in policy["required_tracked_blobs"].items():
        observed_blob = git_blob_from_api(repo, path, head_sha, token)
        assert observed_blob == expected_blob, (
            f"trusted source blob mismatch for {path!r}: "
            f"expected={expected_blob}, observed={observed_blob}"
        )

    d6s1_bytes = file_bytes_from_api(
        repo,
        "docs/integral/d6s-canon-1-golden-vectors.json",
        head_sha,
        token,
    )
    assert hashlib.sha256(d6s1_bytes).hexdigest() == policy["d6s1_corpus_sha256"]

    assert record["d6s2_manifest_git_blob_sha"] == policy["d6s2_manifest_git_blob_sha"]
    assert record["d6s2_fixture_git_blob_sha"] == policy["d6s2_fixture_git_blob_sha"]
    assert record["d6s1_corpus_sha256"] == policy["d6s1_corpus_sha256"]

    print(
        "verified D6U trusted-builder input: "
        f"run={run_id}, attempt={run_attempt}, source={head_sha}, "
        f"cases={len(policy['cases'])}/{len(policy['cases'])}"
    )


if __name__ == "__main__":
    main()
