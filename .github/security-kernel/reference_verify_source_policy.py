#!/usr/bin/env python3
"""Independent source-policy verifier for the Security Kernel workflow stack.

This is a narrow, fail-closed source policy checker. It does not parse arbitrary YAML;
it verifies security-critical workflow headers, permission blocks, pinned actions/images,
isolation markers, and forbidden privilege-escalation constructs from raw bytes.
"""
import base64
import hashlib
import json
import re
import sys
from pathlib import Path

SCHEMA = "security-kernel-source-policy-reference-v1"
BASE_REPOSITORY = "Luminous-Dynamics/mycelix"
S0_PATH = ".github/workflows/security-kernel-trusted-dispatch.yml"
S1_PATH = ".github/workflows/security-kernel-independent-qualification.yml"
S2_PATH = ".github/workflows/security-kernel-trusted-result-verifier.yml"
BINDING_PATH = ".github/security-kernel/reference_verify_execution_binding.py"
CAUSAL_PATH = ".github/security-kernel/reference_verify_causal_join.py"
POLICY_PATH = ".github/security-kernel/reference_verify_source_policy.py"

CHECKOUT_SHA = "3d3c42e5aac5ba805825da76410c181273ba90b1"
UPLOAD_SHA = "043fb46d1a93c77aae656e7c1c64a875d1fc6a0a"
GIT_FETCH_IMAGE = "docker.io/bitnami/git@sha256:3b25b57de5a24330fe87931ef20285b1cf965fabaf3156ce5c5f0eecf37d5329"
RUST_IMAGE = "docker.io/library/rust@sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d"
READ_PERMISSIONS = ("actions: read", "contents: read", "pull-requests: read")

def fail(message: str) -> None:
    raise SystemExit(f"POLICY_REFERENCE_FAIL: {message}")

def normalized_lines(raw: bytes) -> list[str]:
    text = raw.decode("utf-8")
    return text.replace("\r\n", "\n").replace("\r", "\n").split("\n")

def active_uses(lines: list[str]) -> list[str]:
    return [line.strip() for line in lines if re.match(r"\s+uses:\s+", line) and not line.lstrip().startswith("#")]

def exact_count(lines: list[str], pattern: str) -> int:
    return sum(1 for line in lines if re.fullmatch(pattern, line.strip()))

def require(lines: list[str], fragment: str, description: str) -> None:
    if not any(fragment in line for line in lines):
        fail(f"{description}: missing {fragment!r}")

def require_exact(lines: list[str], expected: str, description: str) -> None:
    if exact_count(lines, re.escape(expected)) != 1:
        fail(f"{description}: expected exactly one {expected!r}")

def require_permission_block(lines: list[str], expected_occurrences: int, description: str) -> None:
    blocks = []
    for i, line in enumerate(lines):
        if line.strip() != "permissions:":
            continue
        values = tuple(lines[i + offset].strip() for offset in range(1, 4) if i + offset < len(lines))
        blocks.append(values)
    if blocks.count(READ_PERMISSIONS) != expected_occurrences:
        fail(f"{description}: expected {expected_occurrences} exact read-only permission blocks, found {blocks!r}")

def forbid(lines: list[str], fragments: tuple[str, ...], description: str) -> None:
    joined = "\n".join(lines)
    for fragment in fragments:
        if fragment in joined:
            fail(f"{description}: forbidden construct {fragment!r}")

def verify_action_pins(lines: list[str], expected_action_shas: dict[str, str]) -> None:
    for use in active_uses(lines):
        match = re.search(r"uses:\s*([^@]+)@([0-9a-fA-F]{40})(?:\s+#.*)?$", use)
        if not match:
            fail(f"action reference is not pinned to a full commit SHA: {use!r}")
        action, sha = match.groups()
        expected = expected_action_shas.get(action)
        if expected is not None and sha.lower() != expected:
            fail(f"unexpected pin for {action}: {sha}")


def require_exact_action_set(lines: list[str], expected_actions: tuple[str, ...], description: str) -> None:
    actual = []
    for use in active_uses(lines):
        match = re.search(r"uses:\s*([^@]+)@([0-9a-fA-F]{40})(?:\s+#.*)?$", use)
        if not match:
            fail(f"{description}: action reference is not a recognized pinned action: {use!r}")
        actual.append(f"{match.group(1)}@{match.group(2).lower()}")
    if tuple(actual) != tuple(expected_actions):
        fail(f"{description}: expected exact action set {expected_actions!r}, found {tuple(actual)!r}")

def verify_s0(raw: bytes) -> None:
    lines = normalized_lines(raw)
    require_exact(lines, "name: Security Kernel Qualification — Trusted Dispatcher", "S0 name")
    require_exact(lines, "pull_request_target:", "S0 trigger")
    require_exact(lines, "types: [opened, synchronize, reopened, ready_for_review]", "S0 trigger types")
    require_permission_block(lines, 2, "S0 permissions")
    require_exact(lines, 'BASE_REPOSITORY: "Luminous-Dynamics/mycelix"', "S0 repository")
    require_exact(lines, 'BASE_REPOSITORY_ID: "1176351975"', "S0 repository id")
    require_exact(lines, 'BASE_BRANCH: "main"', "S0 base branch")
    require_exact(lines, 'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "42b9bfe548a90475ce1a4dc531c76991facd2111"', "S0 S1 pin")
    require_exact(lines, "cache-mode: none", "S0 cache mode")
    require_exact(lines, 'test "$GITHUB_REF_PROTECTED" = "true"', "S0 protected-ref check")
    require_exact(lines, 'test "$GITHUB_EVENT_NAME" = "pull_request_target"', "S0 event check")
    require(lines, "./.github/workflows/security-kernel-independent-qualification.yml", "S0 local S1 call")
    forbid(lines, ("actions/checkout@", "git checkout ", "git fetch ", "actions: write", "contents: write", "pull-requests: write", "id-token:", "secrets:"), "S0 trust boundary")
    verify_action_pins(lines, {"actions/checkout": CHECKOUT_SHA, "actions/upload-artifact": UPLOAD_SHA})
    require_exact_action_set(lines, (), "S0 actions")

def verify_s1(raw: bytes) -> None:
    lines = normalized_lines(raw)
    require_exact(lines, "name: Security Kernel Independent Qualification", "S1 name")
    require_exact(lines, "workflow_call:", "S1 trigger")
    if exact_count(lines, re.escape("workflow_dispatch:")):
        fail("S1 must not expose workflow_dispatch")
    require_permission_block(lines, 1, "S1 permissions")
    require_exact(lines, "cache-mode: none", "S1 cache mode")
    require(lines, "uses: actions/checkout@" + CHECKOUT_SHA, "S1 pinned checkout")
    require_exact(lines, "persist-credentials: false", "S1 checkout credentials")
    require_exact(lines, 'FETCH_IMAGE: "' + GIT_FETCH_IMAGE + '"', "S1 pinned git image")
    require(lines, '"' + RUST_IMAGE + '"', "S1 pinned Rust image")
    require(lines, "--network=bridge", "S1 fetch sandbox network")
    require(lines, "--network=none", "S1 execution sandbox network")
    require(lines, "--read-only", "S1 read-only sandbox")
    require(lines, "--cap-drop=ALL", "S1 dropped capabilities")
    require(lines, "--security-opt=no-new-privileges:true", "S1 no-new-privileges")
    require(lines, "--env GITHUB_TOKEN=", "S1 token clearing")
    require(lines, "--env GH_TOKEN=", "S1 token clearing")
    require(lines, "test ! -e /var/run/docker.sock", "S1 Docker socket negative control")
    require(lines, "uses: actions/upload-artifact@" + UPLOAD_SHA, "S1 pinned artifact upload")
    forbid(lines, ("pull_request_target:", "workflow_dispatch:", "actions: write", "contents: write", "pull-requests: write", "id-token:", "--network=host", "--privileged", "--cap-add", "docker.sock:/"), "S1 privilege boundary")
    verify_action_pins(lines, {"actions/checkout": CHECKOUT_SHA, "actions/upload-artifact": UPLOAD_SHA})
    require_exact_action_set(
        lines,
        (f"actions/checkout@{CHECKOUT_SHA}", f"actions/upload-artifact@{UPLOAD_SHA}"),
        "S1 actions",
    )

def verify_s2(raw: bytes, expected_policy_blob_sha: str) -> None:
    lines = normalized_lines(raw)
    require_exact(lines, "name: Security Kernel Qualification — Trusted Result Verifier", "S2 name")
    require_exact(lines, "workflow_run:", "S2 trigger")
    require_exact(lines, 'workflows: ["Security Kernel Qualification — Trusted Dispatcher"]', "S2 workflow trigger")
    require_exact(lines, "types: [completed]", "S2 trigger type")
    if exact_count(lines, re.escape("workflow_dispatch:")):
        fail("S2 must not expose workflow_dispatch")
    require_permission_block(lines, 1, "S2 permissions")
    require_exact(lines, "cache-mode: none", "S2 cache mode")
    require_exact(lines, "ref: " + "$" + "{{ github.workflow_sha }}", "S2 exact-workflow checkout")
    require_exact(lines, "fetch-depth: 0", "S2 full-history checkout")
    require_exact(lines, "persist-credentials: false", "S2 checkout credentials")
    require_exact(lines, 'test "$GITHUB_REF_PROTECTED" = "true"', "S2 protected-ref check")
    require_exact(lines, 'test "$GITHUB_REF" = "refs/heads/main"', "S2 main-ref check")
    require_exact(lines, 'TRUSTED_DISPATCHER_WORKFLOW_BLOB_SHA: "ac91d40e653b1ed2ddeb4b7d7111954d0fa4f4bb"', "S2 S0 pin")
    require_exact(lines, 'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "42b9bfe548a90475ce1a4dc531c76991facd2111"', "S2 S1 pin")
    require_exact(lines, 'REFERENCE_VERIFIER_PATH: ".github/security-kernel/reference_verify_execution_binding.py"', "S2 binding oracle path")
    require_exact(lines, 'CAUSAL_JOIN_VERIFIER_PATH: ".github/security-kernel/reference_verify_causal_join.py"', "S2 causal oracle path")
    require_exact(lines, 'SOURCE_POLICY_VERIFIER_PATH: ".github/security-kernel/reference_verify_source_policy.py"', "S2 policy oracle path")
    require_exact(lines, 'REFERENCE_VERIFIER_BLOB_SHA: "24b5ea20e3a5946b106c22ece9fedcdb85e9c7db"', "S2 binding oracle pin")
    require_exact(lines, 'CAUSAL_JOIN_VERIFIER_BLOB_SHA: "2c9d4d48dab41255944e48300893a62c668b5d4a"', "S2 causal oracle pin")
    require_exact(lines, f'SOURCE_POLICY_VERIFIER_BLOB_SHA: "{expected_policy_blob_sha}"', "S2 policy oracle self-pin")
    forbid(lines, ("actions: write", "contents: write", "pull-requests: write", "id-token:", "actions/upload-artifact@", "docker run ", "docker exec "), "S2 read-only verifier boundary")
    verify_action_pins(lines, {"actions/checkout": CHECKOUT_SHA})
    require_exact_action_set(lines, (f"actions/checkout@{CHECKOUT_SHA}",), "S2 actions")

def verify_file_identity(record: dict) -> bytes:
    if set(record) != {"path", "ref", "sha", "encoding", "content"}:
        fail("workflow file snapshot schema drift")
    assert record["encoding"] == "base64"
    assert re.fullmatch(r"[0-9a-f]{40}", record["sha"])
    raw = base64.b64decode(record["content"], validate=True)
    computed = hashlib.sha1(f"blob {len(raw)}\0".encode() + raw).hexdigest()
    assert computed == record["sha"]
    return raw

def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_verify_source_policy.py <snapshot.json>")
    snapshot = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    expected_files = {
        "s0": S0_PATH,
        "s1": S1_PATH,
        "s2": S2_PATH,
        "binding": BINDING_PATH,
        "causal": CAUSAL_PATH,
        "policy": POLICY_PATH,
    }
    if set(snapshot) != {"schema", "files"} or snapshot["schema"] != SCHEMA:
        fail("source-policy snapshot schema mismatch")
    files = snapshot["files"]
    if set(files) != set(expected_files):
        fail("source-policy file census mismatch")
    raw = {}
    for name, expected_path in expected_files.items():
        if files[name]["path"] != expected_path:
            fail(f"{name} path mismatch")
        raw[name] = verify_file_identity(files[name])
    verify_s0(raw["s0"])
    verify_s1(raw["s1"])
    verify_s2(raw["s2"], files["policy"]["sha"])
    print(json.dumps({"schema": SCHEMA, "policy_result": "verified", "workflow_file_count": 3, "reference_file_count": 3, "action_pins_verified": 3, "pinned_images_verified": 2, "forbidden_escalations_checked": 18}, sort_keys=True, separators=(",", ":")))

if __name__ == "__main__":
    main()
