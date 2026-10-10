#!/usr/bin/env python3
"""Verify D6U evidence as inert data before trusted attestation."""

import base64
import hashlib
import json
import os
import re
import sys
import tomllib
import urllib.parse
import urllib.request
from pathlib import Path



if not __debug__:
    raise RuntimeError("trusted D6U program must not run with Python optimization enabled")

ROOT = Path(__file__).parents[2]
POLICY = ROOT / "docs/integral/d6u-trusted-builder-policy.json"

MAX_GITHUB_JSON_BYTES = 8 * 1024 * 1024
LOCK_PACKAGE_VERSION_PATTERN = re.compile(
    r"^[0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.-]+)?(?:\+[0-9A-Za-z.-]+)?$"
)


class NoAuthorizationRedirectHandler(urllib.request.HTTPRedirectHandler):
    """Never forward the GitHub Actions bearer token across a redirect."""

    def redirect_request(self, req, fp, code, msg, hdrs, newurl):
        redirected = super().redirect_request(req, fp, code, msg, hdrs, newurl)
        if redirected is not None:
            parsed = urllib.parse.urlsplit(newurl)
            assert parsed.scheme == "https", (
                "trusted GitHub API redirect must remain on HTTPS"
            )
            assert parsed.username is None and parsed.password is None, (
                "trusted GitHub API redirect must not introduce URL credentials"
            )
            redirected.remove_header("Authorization")
        return redirected


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
    opener = urllib.request.build_opener(NoAuthorizationRedirectHandler())
    with opener.open(request, timeout=30) as response:
        final_url = urllib.parse.urlsplit(response.geturl())
        assert final_url.scheme == "https"
        assert final_url.hostname == "api.github.com"
        assert final_url.username is None and final_url.password is None
        assert final_url.port in (None, 443)
        payload = response.read(MAX_GITHUB_JSON_BYTES + 1)
        if len(payload) > MAX_GITHUB_JSON_BYTES:
            raise RuntimeError(
                f"GitHub API response exceeded {MAX_GITHUB_JSON_BYTES} bytes"
            )
        return json.loads(payload)


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


def verify_exact_harness_file_set(tree_payload: dict, expected_files: list[str]) -> None:
    assert tree_payload.get("truncated") is False, (
        "GitHub Git tree response was truncated; refusing incomplete D6U harness identity"
    )
    prefix = "d6u-runtime-harness/"
    entries = [entry for entry in tree_payload.get("tree", []) if entry.get("path", "").startswith(prefix)]
    nonstandard = [
        entry for entry in entries
        if entry.get("type") not in {"blob", "tree"}
    ]
    assert not nonstandard, f"D6U harness contains unsupported Git entries: {nonstandard!r}"

    observed_files = sorted(
        entry["path"] for entry in entries if entry.get("type") == "blob"
    )
    assert observed_files == sorted(expected_files), (
        f"D6U harness file-set mismatch: "
        f"expected={sorted(expected_files)!r}, observed={observed_files!r}"
    )
    for entry in entries:
        if entry.get("type") != "blob":
            continue
        assert entry.get("mode") in {"100644", "100755"}, (
            f"D6U harness file has unexpected Git mode: "
            f"{entry.get('path')!r}: {entry.get('mode')!r}"
        )


def verify_artifact_layout(
    artifact_dir: Path,
    expected_files: set[str],
    max_entries: int,
) -> None:
    assert artifact_dir.is_dir(), f"missing trusted artifact directory: {artifact_dir}"

    stack = [artifact_dir]
    files: set[str] = set()
    entry_count = 0
    while stack:
        directory = stack.pop()
        with os.scandir(directory) as entries:
            for entry in entries:
                entry_count += 1
                assert entry_count <= max_entries, (
                    f"trusted artifact contains too many filesystem entries: "
                    f"{entry_count} > {max_entries}"
                )

                relative = Path(entry.path).relative_to(artifact_dir).as_posix()
                assert not entry.is_symlink(), (
                    f"trusted artifact contains symlink: {relative!r}"
                )
                if entry.is_dir(follow_symlinks=False):
                    raise AssertionError(
                        f"trusted artifact contains nested directory: {relative!r}"
                    )
                if entry.is_file(follow_symlinks=False):
                    files.add(relative)
                    continue

                raise AssertionError(
                    f"trusted artifact contains special file: {relative!r}"
                )

    assert files == expected_files, (
        f"unexpected trusted-input files: {sorted(files)!r}"
    )


def verify_artifact_size_limits(
    artifact_dir: Path,
    maximums: dict[str, int],
    total_maximum: int,
) -> None:
    total = 0
    for relative_path, maximum in maximums.items():
        path = artifact_dir / relative_path
        size = path.stat().st_size
        assert size <= maximum, (
            f"trusted artifact file is too large: {relative_path!r}: "
            f"size={size}, maximum={maximum}"
        )
        total += size
    assert total <= total_maximum, (
        f"trusted artifact total size is too large: "
        f"size={total}, maximum={total_maximum}"
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


def verify_trusted_workflow_identity(
    policy: dict,
    repo: str,
    workflow_sha: str,
    token: str,
) -> None:
    cfg = policy["trusted_workflow"]
    tree = git_tree_from_api(repo, workflow_sha, token)
    verify_required_tracked_blobs(
        tree,
        {cfg["path"]: cfg["blob_sha"]},
    )


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


def verify_executor_run_event(
    executor_run: dict,
    policy: dict,
    repo: str,
) -> None:
    cfg = policy["executor_workflow"]
    expected_repository_id = int(policy["repository_identity"]["repository_id"])
    assert executor_run["name"] == cfg["name"], "executor workflow name mismatch"
    assert executor_run["path"] == cfg["path"], "executor workflow path mismatch"
    assert executor_run["event"] == "workflow_run", "executor event type mismatch"
    assert executor_run["conclusion"] == "success", "executor run did not succeed"
    assert executor_run["repository"]["full_name"] == repo, "executor repository name mismatch"
    assert int(executor_run["repository"]["id"]) == expected_repository_id, (
        "executor repository ID mismatch"
    )
    assert executor_run["head_repository"]["full_name"] == repo, (
        "executor head repository name mismatch"
    )
    assert int(executor_run["head_repository"]["id"]) == expected_repository_id, (
        "executor head repository ID mismatch"
    )
    assert executor_run["head_branch"] == "main", "executor workflow ref is not main"
    head_sha = executor_run["head_sha"]
    assert isinstance(head_sha, str) and re.fullmatch(r"[0-9a-f]{40}", head_sha), (
        "executor workflow commit SHA is not canonical"
    )
    for key in ("id", "run_attempt"):
        value = executor_run[key]
        assert isinstance(value, int) and not isinstance(value, bool) and value > 0, (
            f"executor {key} is not a positive integer"
        )


def verify_executor_run_record(
    executor_run: dict,
    record: dict[str, str],
    policy: dict,
    repo: str,
) -> None:
    verify_executor_run_event(executor_run, policy, repo)
    head_sha = executor_run["head_sha"]
    assert record["executor_workflow_commit_sha"] == head_sha, (
        "executor workflow commit does not match evidence record"
    )
    assert executor_run["id"] == int(record["executor_run_id"]), (
        "executor run ID does not match evidence record"
    )
    assert executor_run["run_attempt"] == int(record["executor_run_attempt"]), (
        "executor run attempt does not match evidence record"
    )


def verify_executor_workflow_against_run_head(
    executor_run: dict,
    policy: dict,
    repo: str,
    token: str,
) -> None:
    expected_repository_id = int(policy["repository_identity"]["repository_id"])
    assert executor_run["repository"]["full_name"] == repo
    assert int(executor_run["repository"]["id"]) == expected_repository_id
    assert executor_run["head_repository"]["full_name"] == repo
    assert int(executor_run["head_repository"]["id"]) == expected_repository_id
    assert executor_run["head_branch"] == "main"
    head_sha = executor_run["head_sha"]
    assert re.fullmatch(r"[0-9a-f]{40}", head_sha)
    tree = git_tree_from_api(repo, head_sha, token)
    verify_required_tracked_blobs(
        tree,
        {policy["executor_workflow"]["path"]: policy["executor_workflow"]["blob_sha"]},
    )


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
    expected_repository_id = int(policy["repository_identity"]["repository_id"])
    assert trigger["name"] == cfg["name"]
    assert trigger["path"] == cfg["path"]
    assert int(trigger["workflow_id"]) == int(cfg["workflow_id"])
    assert trigger["event"] == "pull_request"
    assert trigger["conclusion"] == "success"
    assert trigger["head_repository"]["full_name"] == repo
    assert int(trigger["head_repository"]["id"]) == expected_repository_id
    assert trigger["repository"]["full_name"] == repo
    assert int(trigger["repository"]["id"]) == expected_repository_id
    assert trigger["head_branch"] == policy["source_branch"]
    assert trigger["id"] == int(record["trigger_workflow_run_id"])
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
    verify_exact_harness_file_set(
        source_tree,
        policy["d6u_harness_tracked_files"],
    )
    verify_required_tracked_blobs(
        source_tree,
        policy["required_source_blobs"],
    )
    return trigger


def verify_record_metadata(record: dict[str, str], policy: dict) -> None:
    expected_record_fields = set(policy["record_fields"])
    assert len(expected_record_fields) == len(policy["record_fields"])
    assert set(record) == expected_record_fields, (
        f"runtime evidence record schema mismatch: "
        f"expected={sorted(expected_record_fields)!r}, observed={sorted(record)!r}"
    )
    for key in (
        "workflow_run_id",
        "workflow_run_attempt",
        "trigger_workflow_run_id",
        "trigger_workflow_run_attempt",
        "executor_run_id",
        "executor_run_attempt",
    ):
        assert re.fullmatch(r"[1-9][0-9]*", record[key]), (
            f"runtime evidence run identity is not canonical: {key}={record[key]!r}"
        )
    assert record["workflow_run_id"] == record["executor_run_id"]
    assert record["workflow_run_attempt"] == record["executor_run_attempt"]
    assert record["d6s2_authority_ledger_schema"] == policy["d6s2_authority_ledger_schema"]
    assert record["d6s1_corpus_sha256"] == policy["d6s1_corpus_sha256"]

    required = policy["required_source_blobs"]
    assert record["manifest_version"] == str(policy["manifest_version"])
    assert record["manifest_git_blob_sha"] == required[
        "docs/integral/d6u-runtime-manifest.json"
    ]
    assert record["evidence_verifier_git_blob_sha"] == required[
        "scripts/integral/verify_d6u_runtime_evidence.py"
    ]
    assert record["lock_verifier_git_blob_sha"] == required[
        "scripts/integral/verify_d6u_runtime_lock.py"
    ]

    assert record["case_coverage"] == policy["expected_case_coverage"]
    assert record["supplemental_coverage"] == policy["expected_supplemental_coverage"]
    assert record["application_check_coverage"] == policy[
        "expected_application_check_coverage"
    ]
    assert record["case_outcome_classes"] == ",".join(
        policy["expected_case_outcome_classes"]
    )
    assert record["runtime"] == f"holochain-{policy['runtime']['holochain']}"
    assert record["hdk"] == policy["runtime"]["hdk"]
    assert record["hdi"] == policy["runtime"]["hdi"]
    assert record["test"] == "d6u_authority_boundary:passed"
    assert record["supported_cases"] == str(len(policy["cases"]))
    assert record["unsupported_cases"] == ",".join(
        policy["unsupported_reference_cases"]
    )
    assert record["claim_ceiling"] == policy["claim_ceiling"]



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


def _git_blob_sha1(content: bytes) -> str:
    header = f"blob {len(content)}\0".encode("utf-8")
    return hashlib.sha1(header + content).hexdigest()


def _manifest_dependency_names(manifest: dict) -> set[str]:
    names: set[str] = set()

    def consume(table: object) -> None:
        assert isinstance(table, dict)
        for alias, specification in table.items():
            assert isinstance(alias, str) and alias
            if isinstance(specification, dict):
                package_name = specification.get("package", alias)
                assert isinstance(package_name, str) and package_name
            else:
                package_name = alias
            names.add(package_name)

    for section in ("dependencies", "dev-dependencies", "build-dependencies"):
        if section in manifest:
            consume(manifest[section])

    targets = manifest.get("target", {})
    if targets:
        assert isinstance(targets, dict)
        for target in targets.values():
            assert isinstance(target, dict)
            for section in ("dependencies", "dev-dependencies", "build-dependencies"):
                if section in target:
                    consume(target[section])

    return names


def _parse_lock_dependency(reference: str) -> tuple[str, str | None, str | None]:
    assert isinstance(reference, str) and reference
    value = reference.strip()
    source: str | None = None
    if value.endswith(")") and " (" in value:
        value, source_with_paren = value.rsplit(" (", 1)
        source = source_with_paren[:-1]
        assert source
    parts = value.rsplit(" ", 1)
    if len(parts) == 2 and LOCK_PACKAGE_VERSION_PATTERN.fullmatch(parts[1]):
        return parts[0], parts[1], source
    assert " " not in value, f"malformed Cargo.lock dependency reference: {reference!r}"
    return value, None, source


def _resolve_lock_dependency(
    reference: str,
    packages_by_name: dict[str, list[tuple[str, str, str | None]]],
) -> tuple[str, str, str | None]:
    name, version, source = _parse_lock_dependency(reference)
    candidates = packages_by_name.get(name, [])
    if version is not None:
        candidates = [candidate for candidate in candidates if candidate[1] == version]
    if source is not None:
        candidates = [candidate for candidate in candidates if candidate[2] == source]
    assert len(candidates) == 1, (
        f"Cargo.lock dependency reference is missing or ambiguous: "
        f"reference={reference!r}, candidates={candidates!r}"
    )
    return candidates[0]


def verify_lock_graph_against_manifest(
    lock: dict,
    manifest: dict,
    policy: dict,
) -> None:
    graph_policy = policy["lock_graph"]
    packages = lock["package"]
    local_packages = set(graph_policy["allowed_local_packages"])
    local_nodes = [
        package
        for package in packages
        if package.get("source") is None
    ]
    assert local_nodes == [
        package
        for package in packages
        if package.get("name") in local_packages
    ], "Cargo.lock local-package surface does not match policy"
    assert len(local_nodes) == 1, (
        f"Cargo.lock must contain exactly one local root package, observed {len(local_nodes)}"
    )

    manifest_package = manifest.get("package")
    assert isinstance(manifest_package, dict), "trusted Cargo.toml must define [package]"
    root = local_nodes[0]
    assert root.get("name") == manifest_package.get("name"), (
        "Cargo.lock root package name does not match the trusted Cargo.toml package name"
    )
    assert root.get("version") == manifest_package.get("version"), (
        "Cargo.lock root package version does not match the trusted Cargo.toml package version"
    )

    expected_direct = _manifest_dependency_names(manifest)
    root_dependencies = root.get("dependencies", [])
    assert isinstance(root_dependencies, list)
    observed_direct: set[str] = set()
    seen_direct: set[str] = set()
    packages_by_name: dict[str, list[tuple[str, str, str | None]]] = {}
    identity_set: set[tuple[str, str, str | None]] = set()

    for package in packages:
        identity = (
            package["name"],
            package["version"],
            package.get("source"),
        )
        assert identity not in identity_set, f"duplicate Cargo.lock package identity: {identity!r}"
        identity_set.add(identity)
        packages_by_name.setdefault(identity[0], []).append(identity)

    for reference in root_dependencies:
        name, _, _ = _parse_lock_dependency(reference)
        assert name not in seen_direct, (
            f"duplicate direct dependency reference in Cargo.lock root: {reference!r}"
        )
        seen_direct.add(name)
        observed_direct.add(name)
        _resolve_lock_dependency(reference, packages_by_name)

    assert observed_direct == expected_direct, (
        f"Cargo.lock root dependencies do not match trusted Cargo.toml: "
        f"expected={sorted(expected_direct)!r}, observed={sorted(observed_direct)!r}"
    )

    adjacency: dict[tuple[str, str, str | None], set[tuple[str, str, str | None]]] = {}
    for package in packages:
        identity = (package["name"], package["version"], package.get("source"))
        references = package.get("dependencies", [])
        assert isinstance(references, list)
        edges: set[tuple[str, str, str | None]] = set()
        for reference in references:
            resolved = _resolve_lock_dependency(reference, packages_by_name)
            assert resolved not in edges, (
                f"duplicate Cargo.lock dependency edge: "
                f"package={identity!r}, reference={reference!r}"
            )
            edges.add(resolved)
        adjacency[identity] = edges

    root_identity = (
        root["name"],
        root["version"],
        root.get("source"),
    )
    reachable: set[tuple[str, str, str | None]] = set()
    pending = [root_identity]
    while pending:
        current = pending.pop()
        if current in reachable:
            continue
        reachable.add(current)
        pending.extend(sorted(adjacency[current] - reachable))

    assert reachable == identity_set, (
        f"Cargo.lock contains unreachable package nodes: "
        f"{sorted(identity_set - reachable)!r}"
    )


def verify_lock_graph_integrity(lock: dict, policy: dict) -> None:
    packages = lock.get("package", [])
    assert isinstance(packages, list) and packages
    graph_policy = policy["lock_graph"]
    local_packages = set(graph_policy["allowed_local_packages"])
    assert local_packages
    observed_local = []
    seen = set()
    for package in packages:
        assert isinstance(package, dict)
        name = package.get("name")
        version = package.get("version")
        assert isinstance(name, str) and name
        assert isinstance(version, str) and version
        identity = (name, version, package.get("source"))
        assert identity not in seen, f"duplicate Cargo.lock package identity: {identity!r}"
        seen.add(identity)
        if name in local_packages:
            assert package.get("source") is None
            assert package.get("checksum") is None
            observed_local.append(name)
            continue
        assert package.get("source") == graph_policy["required_registry_source"], (
            f"Cargo.lock package {name!r} resolves from an untrusted source: "
            f"{package.get('source')!r}"
        )
        checksum = package.get("checksum", "")
        assert re.fullmatch(r"[0-9a-f]{64}", checksum), (
            f"Cargo.lock registry package {name!r} must include a 64-hex checksum"
        )

    assert observed_local == sorted(local_packages), (
        f"Cargo.lock local package set mismatch: expected={sorted(local_packages)!r}, "
        f"observed={sorted(observed_local)!r}"
    )


def verify_lock(
    path: Path,
    policy: dict,
    trusted_manifest: dict | None = None,
) -> None:
    lock = tomllib.loads(path.read_text(encoding="utf-8"))
    expected_format_version = int(policy["lock_graph"]["lockfile_format_version"])
    assert lock.get("version") == expected_format_version, (
        f"Cargo.lock format version mismatch: "
        f"expected={expected_format_version}, observed={lock.get('version')!r}"
    )
    verify_lock_graph_integrity(lock, policy)
    if trusted_manifest is not None:
        verify_lock_graph_against_manifest(lock, trusted_manifest, policy)
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

    assert os.environ["GITHUB_WORKFLOW_REF"] == (
        f"{repo}/.github/workflows/d6u-trusted-evidence-attestation.yml@refs/heads/main"
    )
    verify_trusted_workflow_identity(
        policy,
        repo,
        os.environ["GITHUB_WORKFLOW_SHA"],
        token,
    )

    assert event["repository"]["full_name"] == repo
    verify_executor_run_event(executor_run, policy, repo)
    verify_executor_workflow_against_run_head(
        executor_run,
        policy,
        repo,
        token,
    )

    expected_files = {
        "d6u-runtime-evidence.txt",
        "d6u-runtime-test.log",
        "Cargo.lock",
    }
    verify_artifact_layout(
        artifact_dir,
        expected_files,
        int(policy["artifact_max_entries"]),
    )
    verify_artifact_size_limits(
        artifact_dir,
        policy["artifact_max_bytes"],
        policy["artifact_max_total_bytes"],
    )

    evidence = artifact_dir / "d6u-runtime-evidence.txt"
    test_log = artifact_dir / "d6u-runtime-test.log"
    lockfile = artifact_dir / "Cargo.lock"
    record = load_record(evidence)

    assert set(record) == set(policy["record_fields"])
    verify_executor_run_record(executor_run, record, policy, repo)
    assert record["status"] == "runtime-reference-evidence", "runtime evidence status mismatch"
    assert record["workflow_run_id"] == str(executor_run["id"])
    assert record["workflow_run_attempt"] == str(executor_run["run_attempt"])
    assert record["executor_run_id"] == str(executor_run["id"])
    assert record["executor_run_attempt"] == str(executor_run["run_attempt"])
    assert record["attestation_status"] == "deferred-to-trusted-builder", "trusted-builder handoff status mismatch"
    assert record["claim_ceiling"] == policy["claim_ceiling"]
    verify_record_metadata(record, policy)

    verify_executor_workflow_identity(record, policy, repo, token)
    trigger = verify_trigger_run(record, policy, repo, token)

    assert record["source_commit"] == trigger["head_sha"]
    assert record["source_repository"] == repo

    assert record["test_log_sha256"] == sha256(test_log)
    assert record["cargo_lock_sha256"] == sha256(lockfile)

    verify_cases(test_log.read_text(encoding="utf-8"), policy)
    trusted_manifest_bytes = contents_bytes_from_api(
        repo,
        policy["lock_graph"]["manifest_path"],
        record["source_commit"],
        token,
    )
    expected_manifest_blob = policy["required_source_blobs"][
        policy["lock_graph"]["manifest_path"]
    ]
    assert _git_blob_sha1(trusted_manifest_bytes) == expected_manifest_blob, (
        "trusted Cargo.toml bytes do not match the policy-pinned Git blob"
    )
    trusted_manifest = tomllib.loads(trusted_manifest_bytes.decode("utf-8"))
    verify_lock(lockfile, policy, trusted_manifest)

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
