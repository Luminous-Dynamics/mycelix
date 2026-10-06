#!/usr/bin/env python3
"""Independent raw-object causal-join verifier for Security Kernel qualification.

No network access and no repository-local imports. The verifier reconstructs the
candidate/run/job/artifact/receipt relationships from raw GitHub API snapshots
rather than consuming S2's derived identity variables.
"""
import base64
import hashlib
import json
import re
import sys
from pathlib import Path

SCHEMA = "security-kernel-causal-join-reference-v1"

BASE_REPOSITORY = "Luminous-Dynamics/mycelix"
BASE_REPOSITORY_ID = 1176351975
BASE_BRANCH = "main"
S0_PATH = ".github/workflows/security-kernel-trusted-dispatch.yml"
S0_BLOB = "ac91d40e653b1ed2ddeb4b7d7111954d0fa4f4bb"
S1_PATH = ".github/workflows/security-kernel-independent-qualification.yml"
S1_BLOB = "42b9bfe548a90475ce1a4dc531c76991facd2111"
S2_PATH = ".github/workflows/security-kernel-trusted-result-verifier.yml"
BINDING_PATH = ".github/security-kernel/reference_verify_execution_binding.py"
CAUSAL_PATH = ".github/security-kernel/reference_verify_causal_join.py"
POLICY_PATH = ".github/security-kernel/reference_verify_source_policy.py"
EXPECTED_JOB_NAME = "Independent Security Kernel"

REQUIRED_S1_STEPS = (
    "Checkout trusted qualification root",
    "Verify trusted pull-request-target invocation",
    "Resolve exact candidate source",
    "Static trust-surface audit",
    "Snapshot exact candidate source identity",
    "Snapshot locked dependency identity",
    "Pull and preflight pinned sandbox image",
    "Prepare locked dependency subject",
    "Vendor locked dependency closure in fetch sandbox",
    "Execute sandbox negative controls",
    "Execute candidate qualification in disposable networkless sandbox",
    "Verify candidate source immutability",
    "Verify dependency substrate immutability",
    "Emit qualification receipt",
    "Upload qualification receipt",
    "Verify retained qualification receipt",
)

RECEIPT_KEYS = frozenset(
    {
        "schema",
        "candidate_repository",
        "candidate_repository_id",
        "candidate_pr",
        "candidate_sha",
        "candidate_tree",
        "candidate_source_tree_entry_count",
        "candidate_source_file_count",
        "candidate_source_total_path_bytes",
        "candidate_source_total_bytes",
        "candidate_source_max_blob_bytes",
        "candidate_source_digest",
        "candidate_cargo_lock_format",
        "candidate_cargo_lock_sha256",
        "candidate_cargo_lock_package_count",
        "candidate_cargo_lock_dependency_edges",
        "vendor_digest",
        "sandbox_negative_controls",
        "sandbox_negative_controls_log_sha256",
        "dependency_substrate_postflight",
        "git_fetch_image",
        "sandbox_image",
        "sandbox_network",
        "sandbox_rootfs",
        "sandbox_user",
        "sandbox_capabilities",
        "sandbox_no_new_privileges",
        "source_event",
        "trusted_dispatcher_workflow",
        "trusted_workflow_ref",
        "trusted_workflow_sha",
        "trusted_workflow_name",
        "trusted_repository_id",
        "trusted_workflow_repository",
        "trusted_workflow_file_path",
        "trusted_ref_protected",
        "cache_mode",
        "registered_s1_workflow_blob_sha",
        "called_workflow_ref",
        "called_workflow_sha",
        "s1_run_id",
        "s1_run_attempt",
        "rust_version",
        "rust_commit",
        "qualification_pass",
    }
)

DIGEST_FIELDS = (
    "candidate_source_digest",
    "candidate_cargo_lock_sha256",
    "vendor_digest",
    "sandbox_negative_controls_log_sha256",
)

COUNT_FIELDS = (
    "candidate_source_tree_entry_count",
    "candidate_source_file_count",
    "candidate_source_total_path_bytes",
    "candidate_source_total_bytes",
    "candidate_source_max_blob_bytes",
    "candidate_cargo_lock_package_count",
    "candidate_cargo_lock_dependency_edges",
)


def fail(message: str) -> None:
    raise SystemExit(f"CAUSAL_REFERENCE_FAIL: {message}")


def need(value, description: str):
    if value is None:
        fail(f"missing {description}")
    return value


def parse_receipt(text: str) -> dict:
    if not isinstance(text, str) or not text:
        fail("receipt text is empty")
    receipt = {}
    for line in text.splitlines():
        if "=" not in line:
            fail(f"malformed receipt line: {line!r}")
        key, value = line.split("=", 1)
        if not re.fullmatch(r"[a-z0-9_]+", key):
            fail(f"invalid receipt key: {key!r}")
        if key in receipt:
            fail(f"duplicate receipt key: {key!r}")
        receipt[key] = value
    if set(receipt) != RECEIPT_KEYS:
        fail(
            "receipt closed-world schema mismatch: "
            f"missing={sorted(RECEIPT_KEYS - set(receipt))!r} "
            f"extra={sorted(set(receipt) - RECEIPT_KEYS)!r}"
        )
    return receipt


def verify(snapshot: object) -> dict:
    if not isinstance(snapshot, dict):
        fail("snapshot must be a JSON object")
    expected_top = {
        "schema",
        "event",
        "repository",
        "run",
        "pull_request",
        "jobs",
        "artifact_list_view",
        "artifact_list_total_count",
        "artifact_id_view",
        "receipt_text",
        "main_sha",
        "dispatcher_workflow_blob_sha",
        "s1_workflow_blob_sha",
        "workflow_file_snapshots",
    }
    if set(snapshot) != expected_top:
        fail(
            "snapshot closed-world schema mismatch: "
            f"missing={sorted(expected_top - set(snapshot))!r} "
            f"extra={sorted(set(snapshot) - expected_top)!r}"
        )
    if snapshot["schema"] != SCHEMA:
        fail(f"unexpected snapshot schema: {snapshot['schema']!r}")

    event = need(snapshot["event"], "workflow_run event identity")
    assert set(event) == {
        "run_id",
        "run_attempt",
        "verifier_workflow_sha",
        "verifier_workflow_blob_sha",
        "reference_verifier_blob_sha",
        "causal_join_verifier_blob_sha",
        "source_policy_verifier_blob_sha",
    }
    repository = need(snapshot["repository"], "repository object")
    run = need(snapshot["run"], "workflow run")
    pr = need(snapshot["pull_request"], "pull request")
    jobs = need(snapshot["jobs"], "workflow jobs")
    artifact = need(snapshot["artifact_list_view"], "artifact list view")
    artifact_total_count = snapshot["artifact_list_total_count"]
    assert type(artifact_total_count) is int and artifact_total_count == 1
    artifact_by_id = need(snapshot["artifact_id_view"], "artifact ID view")
    receipt = parse_receipt(snapshot["receipt_text"])

    assert repository["full_name"] == BASE_REPOSITORY
    assert repository["id"] == BASE_REPOSITORY_ID

    assert event["run_id"] == run["id"]
    assert event["run_attempt"] == run["run_attempt"]
    assert type(run["run_attempt"]) is int and run["run_attempt"] >= 1
    assert type(run.get("run_number")) is int and run["run_number"] >= 1
    assert run["status"] == "completed"
    assert run["conclusion"] == "success"
    assert run["event"] == "pull_request_target"
    assert run["ref"] == f"refs/heads/{BASE_BRANCH}"
    assert run["path"] == S0_PATH
    assert run["workflow_ref"] == f"{BASE_REPOSITORY}/{S0_PATH}@refs/heads/{BASE_BRANCH}"
    assert run["repository"]["full_name"] == BASE_REPOSITORY
    assert run["repository"]["id"] == BASE_REPOSITORY_ID
    assert snapshot["main_sha"] == run["workflow_sha"]
    assert re.fullmatch(r"[0-9a-f]{40}", run["workflow_sha"])

    head_repository = run.get("head_repository")
    assert isinstance(head_repository, dict)
    assert type(head_repository.get("id")) is int and head_repository["id"] > 0
    assert isinstance(run.get("head_branch"), str) and run["head_branch"]
    assert re.fullmatch(r"[0-9a-f]{40}", run.get("head_sha") or "")

    assert snapshot["dispatcher_workflow_blob_sha"] == S0_BLOB
    assert snapshot["s1_workflow_blob_sha"] == S1_BLOB

    workflow_snapshots = snapshot["workflow_file_snapshots"]
    expected_snapshot_keys = {"s0", "s1", "s2", "binding", "causal", "policy"}
    event_verifier_sha = snapshot["event"]["verifier_workflow_sha"]
    event_verifier_blob_sha = snapshot["event"]["verifier_workflow_blob_sha"]
    event_binding_blob_sha = snapshot["event"]["reference_verifier_blob_sha"]
    event_causal_blob_sha = snapshot["event"]["causal_join_verifier_blob_sha"]
    event_policy_blob_sha = snapshot["event"]["source_policy_verifier_blob_sha"]
    assert set(workflow_snapshots) == expected_snapshot_keys
    assert re.fullmatch(r"[0-9a-f]{40}", event_verifier_sha)
    assert re.fullmatch(r"[0-9a-f]{40}", event_verifier_blob_sha)
    assert re.fullmatch(r"[0-9a-f]{40}", event_binding_blob_sha)
    assert re.fullmatch(r"[0-9a-f]{40}", event_causal_blob_sha)
    assert re.fullmatch(r"[0-9a-f]{40}", event_policy_blob_sha)
    assert event_binding_blob_sha == snapshot["workflow_file_snapshots"]["binding"]["sha"]
    assert event_causal_blob_sha == snapshot["workflow_file_snapshots"]["causal"]["sha"]
    assert event_policy_blob_sha == snapshot["workflow_file_snapshots"]["policy"]["sha"]

    def verify_file_snapshot(name: str, expected_path: str, expected_ref: str, expected_sha: str | None = None):
        record = workflow_snapshots[name]
        assert set(record) == {"path", "ref", "sha", "encoding", "content"}
        assert record["path"] == expected_path
        assert record["ref"] == expected_ref
        assert record["encoding"] == "base64"
        assert re.fullmatch(r"[0-9a-f]{40}", record["sha"])
        if expected_sha is not None:
            assert record["sha"] == expected_sha
        encoded = record["content"].replace("\n", "")
        raw = base64.b64decode(encoded, validate=True)
        assert raw, f"{name} workflow snapshot is empty"
        git_header = f"blob {len(raw)}\0".encode("ascii")
        computed = hashlib.sha1(git_header + raw).hexdigest()
        assert computed == record["sha"], f"{name} content does not hash to its advertised Git blob SHA"
        return raw

    verify_file_snapshot("s0", S0_PATH, run["workflow_sha"], S0_BLOB)
    verify_file_snapshot("s1", S1_PATH, run["workflow_sha"], S1_BLOB)
    verify_file_snapshot("s2", S2_PATH, event_verifier_sha, event_verifier_blob_sha)
    verify_file_snapshot("binding", BINDING_PATH, event_verifier_sha, event_binding_blob_sha)
    verify_file_snapshot("causal", CAUSAL_PATH, event_verifier_sha, event_causal_blob_sha)
    verify_file_snapshot("policy", POLICY_PATH, event_verifier_sha, event_policy_blob_sha)

    title = run.get("display_title", "")
    match = re.fullmatch(
        r"SEC-KERNEL-DISPATCH repo_id=([0-9]+) pr=([0-9]+) candidate=([0-9a-f]{40})",
        title,
    )
    assert match, f"unexpected dispatcher run title: {title!r}"
    candidate_repository_id, candidate_pr, candidate_sha = match.groups()
    candidate_repository_id = int(candidate_repository_id)
    candidate_pr = int(candidate_pr)

    run_prs = run.get("pull_requests") or []
    assert len(run_prs) == 1
    assert run_prs[0]["number"] == candidate_pr

    assert pr["number"] == candidate_pr
    assert pr["state"] == "open"
    assert pr["base"]["repo"]["full_name"] == BASE_REPOSITORY
    assert pr["base"]["ref"] == BASE_BRANCH
    assert pr["head"]["sha"] == candidate_sha
    pr_head_repo = need(pr["head"].get("repo"), "PR head repository")
    assert pr_head_repo["full_name"] == head_repository.get("full_name")
    assert pr_head_repo["id"] == candidate_repository_id
    assert pr_head_repo["id"] == head_repository["id"]

    assert len(jobs) == 2
    assert all(job["run_id"] == run["id"] for job in jobs)
    assert all(job["status"] == "completed" for job in jobs)

    resolver = [j for j in jobs if j.get("name") == "Resolve exact pull request subject"]
    assert len(resolver) == 1
    assert resolver[0]["conclusion"] == "success"
    assert type(resolver[0]["id"]) is int and resolver[0]["id"] > 0

    s1 = [
        j for j in jobs
        if j.get("name") == EXPECTED_JOB_NAME
        or j.get("name", "").endswith(" / " + EXPECTED_JOB_NAME)
    ]
    assert len(s1) == 1
    s1_job = s1[0]
    assert s1_job["conclusion"] == "success"
    assert type(s1_job["id"]) is int and s1_job["id"] > 0

    step_names = [step.get("name") for step in (s1_job.get("steps") or [])]
    assert all(isinstance(name, str) and name for name in step_names)
    assert len(step_names) == len(set(step_names))
    assert set(REQUIRED_S1_STEPS).issubset(step_names)
    assert all(step_names.count(name) == 1 for name in REQUIRED_S1_STEPS)
    assert all(
        step["conclusion"] == "success"
        for step in (s1_job.get("steps") or [])
        if step.get("name") in REQUIRED_S1_STEPS
    )

    expected_artifact_name = (
        f"security-kernel-independent-qualification-{candidate_sha}-attempt-{run['run_attempt']}.txt"
    )
    assert artifact["name"] == expected_artifact_name
    assert artifact_by_id["name"] == expected_artifact_name
    assert artifact["id"] == artifact_by_id["id"]
    assert artifact["expired"] is False
    assert artifact_by_id["expired"] is False
    assert type(artifact["id"]) is int and artifact["id"] > 0
    assert artifact["size_in_bytes"] == artifact_by_id["size_in_bytes"]
    assert type(artifact["size_in_bytes"]) is int and artifact["size_in_bytes"] > 0
    assert artifact["digest"] == artifact_by_id["digest"]
    assert re.fullmatch(r"sha256:[0-9a-f]{64}", artifact["digest"])

    artifact_run = artifact.get("workflow_run")
    artifact_run_by_id = artifact_by_id.get("workflow_run")
    assert isinstance(artifact_run, dict)
    assert artifact_run_by_id == artifact_run
    assert artifact_run["id"] == run["id"]
    assert artifact_run["repository_id"] == run["repository"]["id"]
    assert artifact_run["head_repository_id"] == head_repository["id"]
    assert artifact_run["head_branch"] == run["head_branch"]
    assert artifact_run["head_sha"] == run["head_sha"]

    receipt_digest = hashlib.sha256(snapshot["receipt_text"].encode("utf-8")).hexdigest()
    assert re.fullmatch(r"[0-9a-f]{64}", receipt_digest)

    exact = {
        "schema": "security-kernel-independent-qualification-v1",
        "candidate_repository": pr_head_repo["full_name"],
        "candidate_repository_id": str(candidate_repository_id),
        "candidate_pr": str(candidate_pr),
        "candidate_sha": candidate_sha,
        "candidate_cargo_lock_format": "v4",
        "source_event": "pull_request_target",
        "trusted_dispatcher_workflow": run["workflow_ref"],
        "trusted_workflow_ref": run["workflow_ref"],
        "trusted_workflow_sha": run["workflow_sha"],
        "trusted_workflow_name": "Security Kernel Qualification — Trusted Dispatcher",
        "trusted_repository_id": str(BASE_REPOSITORY_ID),
        "trusted_workflow_repository": BASE_REPOSITORY,
        "trusted_workflow_file_path": S0_PATH,
        "trusted_ref_protected": "true",
        "cache_mode": "none",
        "registered_s1_workflow_blob_sha": S1_BLOB,
        "called_workflow_ref": f"{BASE_REPOSITORY}/{S1_PATH}@refs/heads/{BASE_BRANCH}",
        "called_workflow_sha": run["workflow_sha"],
        "s1_run_id": str(run["id"]),
        "s1_run_attempt": str(run["run_attempt"]),
        "sandbox_network": "none",
        "sandbox_rootfs": "read-only",
        "sandbox_user": "non-root",
        "sandbox_capabilities": "all-dropped",
        "sandbox_no_new_privileges": "true",
        "sandbox_negative_controls": "passed",
        "dependency_substrate_postflight": "passed",
        "qualification_pass": "true",
    }
    for key, value in exact.items():
        assert receipt[key] == value, f"receipt {key!r} mismatch"

    for key in DIGEST_FIELDS:
        assert re.fullmatch(r"[0-9a-f]{64}", receipt[key])

    for key in COUNT_FIELDS:
        assert re.fullmatch(r"[0-9]+", receipt[key])

    snapshot_bytes = json.dumps(
        snapshot, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    snapshot_sha256 = hashlib.sha256(snapshot_bytes).hexdigest()

    result = {
        "schema": SCHEMA,
        "snapshot_sha256": snapshot_sha256,
        "workflow_file_snapshot_sha256": hashlib.sha256(
            json.dumps(snapshot["workflow_file_snapshots"], sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
        ).hexdigest(),
        "candidate_repository": pr_head_repo["full_name"],
        "candidate_repository_id": candidate_repository_id,
        "candidate_pr": candidate_pr,
        "candidate_sha": candidate_sha,
        "run_id": run["id"],
        "run_attempt": run["run_attempt"],
        "run_number": run["run_number"],
        "resolver_job_id": resolver[0]["id"],
        "s1_job_id": s1_job["id"],
        "artifact_id": artifact["id"],
        "artifact_name": artifact["name"],
        "artifact_digest": artifact["digest"],
        "receipt_content_sha256": receipt_digest,
        "reference_result": "verified",
    }
    return result


def assert_mutation_rejected(snapshot: dict) -> int:
    mutations = []

    def add(label, path, replacement):
        mutations.append((label, path, replacement))

    add("event run id", ("event", "run_id"), snapshot["event"]["run_id"] + 1)
    add("event run attempt", ("event", "run_attempt"), snapshot["event"]["run_attempt"] + 1)
    add("event verifier SHA", ("event", "verifier_workflow_sha"), "0" * 40)
    add("event verifier blob", ("event", "verifier_workflow_blob_sha"), "1" * 40)
    add("run SHA", ("run", "workflow_sha"), "2" * 40)
    add("run title", ("run", "display_title"), snapshot["run"]["display_title"] + "-mutated")
    add("PR head SHA", ("pull_request", "head", "sha"), "3" * 40)
    resolver_index = next(
        i for i, job in enumerate(snapshot["jobs"])
        if job.get("name") == "Resolve exact pull request subject"
    )
    s1_index = next(
        i for i, job in enumerate(snapshot["jobs"])
        if job.get("name") == EXPECTED_JOB_NAME
        or job.get("name", "").endswith(" / " + EXPECTED_JOB_NAME)
    )
    add("resolver job run join", ("jobs", resolver_index, "run_id"), snapshot["jobs"][resolver_index]["run_id"] + 1)
    add("S1 job run join", ("jobs", s1_index, "run_id"), snapshot["jobs"][s1_index]["run_id"] + 1)
    add("artifact list total", ("artifact_list_total_count",), 2)
    add("artifact name", ("artifact_list_view", "name"), snapshot["artifact_list_view"]["name"] + "-mutated")
    add("artifact id", ("artifact_list_view", "id"), snapshot["artifact_list_view"]["id"] + 1)
    add("artifact digest", ("artifact_list_view", "digest"), "sha256:" + "4" * 64)
    add("artifact head SHA", ("artifact_list_view", "workflow_run", "head_sha"), "5" * 40)
    add("receipt text", ("receipt_text",), snapshot["receipt_text"] + "\nmutated=true")
    add(
        "S0 bytes",
        ("workflow_file_snapshots", "s0", "content"),
        snapshot["workflow_file_snapshots"]["s0"]["content"] + "AA==",
    )
    add(
        "S1 bytes",
        ("workflow_file_snapshots", "s1", "content"),
        snapshot["workflow_file_snapshots"]["s1"]["content"] + "AA==",
    )
    add(
        "S2 bytes",
        ("workflow_file_snapshots", "s2", "content"),
        snapshot["workflow_file_snapshots"]["s2"]["content"] + "AA==",
    )
    add(
        "binding verifier bytes",
        ("workflow_file_snapshots", "binding", "content"),
        snapshot["workflow_file_snapshots"]["binding"]["content"] + "AA==",
    )
    add(
        "causal verifier bytes",
        ("workflow_file_snapshots", "causal", "content"),
        snapshot["workflow_file_snapshots"]["causal"]["content"] + "AA==",
    )
    add(
        "policy verifier bytes",
        ("workflow_file_snapshots", "policy", "content"),
        snapshot["workflow_file_snapshots"]["policy"]["content"] + "AA==",
    )

    for label, path, replacement in mutations:
        mutated = json.loads(json.dumps(snapshot))
        cursor = mutated
        for key in path[:-1]:
            cursor = cursor[key]
        cursor[path[-1]] = replacement
        try:
            verify(mutated)
        except (AssertionError, SystemExit):
            continue
        fail(f"causal acceptance invariant under mutation of {label}")
    return len(mutations)


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_verify_causal_join.py <snapshot.json>")
    snapshot = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    mutation_count = assert_mutation_rejected(snapshot)
    result = verify(snapshot)
    result["metamorphic_mutations_verified"] = mutation_count
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
