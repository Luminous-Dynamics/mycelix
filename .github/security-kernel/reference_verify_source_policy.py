#!/usr/bin/env python3
"""Independent fail-closed source-policy verifier for the Security Kernel workflow stack.

The verifier intentionally does not parse arbitrary YAML. It validates a narrow,
closed-world source policy from raw Git blob snapshots: trigger topology, job/step
topology, permission blocks, pinned external actions, trusted pin relations, and
forbidden privilege-escalation constructs.
"""

import base64
import hashlib
import json
import re
import sys
from pathlib import Path

SCHEMA = "security-kernel-source-policy-reference-v1"

S0 = ".github/workflows/security-kernel-trusted-dispatch.yml"
S1 = ".github/workflows/security-kernel-independent-qualification.yml"
S2 = ".github/workflows/security-kernel-trusted-result-verifier.yml"
RETENTION = ".github/security-kernel/reference_verify_evidence_retention_binding.py"
EXECUTION = ".github/security-kernel/reference_verify_execution_binding.py"
POLICY = ".github/security-kernel/reference_verify_source_policy.py"

CHECKOUT = "actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1"
UPLOAD = "actions/upload-artifact@043fb46d1a93c77aae656e7c1c64a875d1fc6a0a"
DOWNLOAD = "actions/download-artifact@3e5f45b2cfb9172054b4087a40e8e0b5a5461e7c"
GIT_FETCH_IMAGE = "docker.io/bitnami/git@sha256:3b25b57de5a24330fe87931ef20285b1cf965fabaf3156ce5c5f0eecf37d5329"
RUST_IMAGE = "docker.io/library/rust@sha256:9af5f5f37d3035dd18d216e348e946ff1fc8c7fa7998c7443cedfe880231110d"

S1_STEPS = (
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
    "Upload sandbox negative-control transcript",
    "Execute candidate qualification in disposable networkless sandbox",
    "Verify candidate source immutability",
    "Verify dependency substrate immutability",
    "Emit qualification receipt",
    "Upload qualification receipt",
    "Verify retained qualification receipt",
)

S2_STEPS = (
    "Checkout exact verifier workflow commit",
    "Verify trusted dispatcher, reusable S1, and qualification gates",
    "Verify retained negative-control evidence binding",
    "Download retained qualification receipt through official artifact client",
    "Download retained sandbox negative-control transcript through official artifact client",
    "Verify official receipt transport and publish verified result",
)

READ_PERMISSIONS = ("actions: read", "contents: read", "pull-requests: read")


def fail(message: str) -> None:
    raise SystemExit(f"SOURCE_POLICY_REFERENCE_FAIL: {message}")


def lines(raw: bytes) -> list[str]:
    return raw.decode("utf-8").replace("\r\n", "\n").replace("\r", "\n").split("\n")


def exact_count(values: list[str], expected: str) -> int:
    return sum(1 for value in values if value.strip() == expected)


def top_level_keys_after(lines_: list[str], heading: str) -> tuple[str, ...]:
    try:
        start = next(i for i, line in enumerate(lines_) if line.strip() == heading)
    except StopIteration:
        fail(f"missing {heading!r}")
    found = []
    for line in lines_[start + 1 :]:
        if line and not line.startswith((" ", "\t")):
            break
        match = re.fullmatch(r"  ([A-Za-z0-9_-]+):\s*", line)
        if match:
            found.append(match.group(1))
    return tuple(found)


def job_keys(lines_: list[str]) -> tuple[str, ...]:
    try:
        start = next(i for i, line in enumerate(lines_) if line.strip() == "jobs:")
    except StopIteration:
        fail("missing jobs block")
    found = []
    for line in lines_[start + 1 :]:
        match = re.fullmatch(r"  ([A-Za-z0-9_-]+):\s*", line)
        if match:
            found.append(match.group(1))
    return tuple(found)


def step_names(lines_: list[str]) -> tuple[str, ...]:
    return tuple(
        match.group(1)
        for line in lines_
        if (match := re.fullmatch(r"\s{6}- name: (.+)", line))
    )


def external_uses(lines_: list[str]) -> tuple[str, ...]:
    return tuple(
        match.group(1).strip()
        for line in lines_
        if (match := re.fullmatch(r"\s*uses:\s+(.+)", line))
        and not match.group(1).strip().startswith("./")
    )


def local_uses(lines_: list[str]) -> tuple[str, ...]:
    return tuple(
        match.group(1).strip()
        for line in lines_
        if (match := re.fullmatch(r"\s*uses:\s+(.+)", line))
        and match.group(1).strip().startswith("./")
    )


def require_permissions(lines_: list[str], count: int) -> None:
    blocks = []
    for i, line in enumerate(lines_):
        if line.strip() != "permissions:":
            continue
        indent = len(line) - len(line.lstrip(" "))
        child_indent = indent + 2
        block = []
        for candidate in lines_[i + 1 :]:
            if not candidate.strip():
                continue
            if len(candidate) - len(candidate.lstrip(" ")) <= indent:
                break
            actual_indent = len(candidate) - len(candidate.lstrip(" "))
            if actual_indent != child_indent:
                fail(f"permission block contains unexpected indentation: {candidate!r}")
            if not re.fullmatch(r"[A-Za-z0-9_-]+:\s+[^#]+", candidate.strip()):
                fail(f"permission block contains malformed entry: {candidate!r}")
            block.append(candidate.strip())
        blocks.append(tuple(block))
    if len(blocks) != count or any(block != READ_PERMISSIONS for block in blocks):
        fail(f"permission census mismatch: expected {count} exact read-only blocks, found {blocks!r}")


def expect_rejection(action, description: str) -> None:
    try:
        action()
    except SystemExit:
        return
    fail(f"adversarial mutation unexpectedly accepted by source-policy oracle: {description!r}")


def inject_extra_permission(raw: bytes) -> bytes:
    marker = b"  pull-requests: read\n"
    replacement = marker + b"  security-events: write\n"
    if marker not in raw:
        fail("permission regression fixture marker missing")
    return raw.replace(marker, replacement, 1)

def inject_extra_permission_with_blank(raw: bytes) -> bytes:
    marker = b"  pull-requests: read\n"
    replacement = marker + b"\n  security-events: write\n"
    if marker not in raw:
        fail("blank-separated permission regression fixture marker missing")
    return raw.replace(marker, replacement, 1)


def require_no_escalation(lines_: list[str], description: str) -> None:
    joined = "\n".join(lines_)
    forbidden = (
        "actions: write",
        "contents: write",
        "pull-requests: write",
        "id-token:",
        "secrets:",
        "--privileged",
        "--network=host",
        "--cap-add",
        "docker.sock:/",
        "workflow_dispatch:",
    )
    for fragment in forbidden:
        if fragment in joined:
            fail(f"{description}: forbidden construct {fragment!r}")


def require_no_fail_open_controls(lines_: list[str], description: str) -> None:
    joined = "\n".join(lines_)
    for fragment in ("continue-on-error:", "if: always()", "if: failure()", "if: cancelled()"):
        if fragment in joined:
            fail(f"{description}: forbidden fail-open control {fragment!r}")


def require_explicit_bash_for_run_steps(lines_: list[str], description: str) -> None:
    run_indices = [
        i for i, line in enumerate(lines_)
        if re.fullmatch(r"\s{8}run:\s*\|?\s*", line)
    ]
    for index in run_indices:
        if index == 0 or not re.fullmatch(r"\s{8}shell:\s+bash\s*", lines_[index - 1]):
            fail(f"{description}: every trusted run step must explicitly declare shell: bash")


def require_no_yaml_reuse_syntax(lines_: list[str], description: str) -> None:
    for line in lines_:
        # Block-scalar command bodies are intentionally excluded; only structural YAML
        # lines (indentation <= 8) are examined here.
        if len(line) - len(line.lstrip(" ")) > 8:
            continue
        if re.search(r"(^|\s)[&*][A-Za-z0-9_-]+(?:\s|$)", line):
            fail(f"{description}: YAML anchors/aliases are forbidden: {line!r}")
        if re.match(r"^\s{0,8}<<:\s*", line):
            fail(f"{description}: YAML merge keys are forbidden: {line!r}")


def require_following(lines_: list[str], step_name: str, expected_line: str, description: str) -> None:
    matches = [i for i, line in enumerate(lines_) if line.strip() == f"- name: {step_name}"]
    if len(matches) != 1:
        fail(f"{description}: expected exactly one step named {step_name!r}")
    index = matches[0]
    following = lines_[index + 1].strip() if index + 1 < len(lines_) else ""
    if following != expected_line:
        fail(f"{description}: expected {expected_line!r} immediately after {step_name!r}, found {following!r}")


def require_exact_actions(actual: tuple[str, ...], expected: tuple[str, ...], description: str) -> None:
    normalized = tuple(
        value.split("#", 1)[0].rstrip()
        for value in actual
    )
    if normalized != expected:
        fail(f"{description}: expected {expected!r}, found {normalized!r}")
    for value in actual:
        if not re.search(r"@[0-9a-f]{40}(?:\s+#.*)?$", value):
            fail(f"{description}: external action is not pinned to a full SHA: {value!r}")


def verify_s0(raw: bytes, expected_s1_sha: str) -> None:
    l = lines(raw)
    if exact_count(l, "name: Security Kernel Qualification — Trusted Dispatcher") != 1:
        fail("S0 name mismatch")
    if top_level_keys_after(l, "on:") != ("pull_request_target",):
        fail("S0 trigger topology mismatch")
    if exact_count(l, "types: [opened, synchronize, reopened, ready_for_review]") != 1:
        fail("S0 trigger types mismatch")
    require_permissions(l, 2)
    if exact_count(l, "cache-mode: none") != 1:
        fail("S0 must declare exactly one cache-mode: none gate")
    if exact_count(l, 'test "$GITHUB_REF_PROTECTED" = "true"') != 1:
        fail("S0 must contain exactly one protected-ref runtime guard")
    if exact_count(l, 'test "$GITHUB_EVENT_NAME" = "pull_request_target"') != 1:
        fail("S0 must contain exactly one pull_request_target runtime guard")
    if job_keys(l) != ("resolve", "qualify"):
        fail("S0 job topology mismatch")
    if step_names(l) != ("Verify trusted dispatcher context and exact PR identity",):
        fail("S0 step topology mismatch")
    if external_uses(l) != ():
        fail(f"S0 external action census mismatch: {external_uses(l)!r}")
    if local_uses(l) != ("./.github/workflows/security-kernel-independent-qualification.yml",):
        fail("S0 local reusable workflow census mismatch")
    if exact_count(l, f'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "{expected_s1_sha}"') != 1:
        fail("S0 registered S1 blob pin mismatch")
    if exact_count(l, f'trusted_workflow_blob_sha: "{expected_s1_sha}"') != 1:
        fail("S0 delegated S1 workflow input pin mismatch")
    if exact_count(l, 'BASE_REPOSITORY: "Luminous-Dynamics/mycelix"') != 1:
        fail("S0 base repository mismatch")
    if exact_count(l, 'BASE_REPOSITORY_ID: "1176351975"') != 1:
        fail("S0 base repository ID mismatch")
    if exact_count(l, 'BASE_BRANCH: "main"') != 1:
        fail("S0 base branch mismatch")
    require_no_fail_open_controls(l, "S0")
    require_no_yaml_reuse_syntax(l, S0)
    require_explicit_bash_for_run_steps(l, S0)
    require_no_escalation(l, "S0")
    if any("git fetch " in x or "git checkout " in x or "actions/checkout@" in x for x in l):
        fail("S0 must remain metadata-only")


def verify_s1(raw: bytes, expected_s1_sha: str) -> None:
    l = lines(raw)
    if exact_count(l, "name: Security Kernel Independent Qualification") != 1:
        fail("S1 name mismatch")
    if top_level_keys_after(l, "on:") != ("workflow_call",):
        fail("S1 trigger topology mismatch")
    require_permissions(l, 1)
    if exact_count(l, "cache-mode: none") != 1:
        fail("S1 must declare exactly one cache-mode: none gate")
    if exact_count(l, 'test "$GITHUB_REF_PROTECTED" = "true"') != 1:
        fail("S1 must contain exactly one protected-ref runtime guard")
    if exact_count(l, 'test "$GITHUB_EVENT_NAME" = "pull_request_target"') != 1:
        fail("S1 must contain exactly one pull_request_target runtime guard")
    if job_keys(l) != ("qualify",):
        fail("S1 job topology mismatch")
    if tuple(step_names(l)) != S1_STEPS:
        fail("S1 step topology mismatch")
    require_exact_actions(
        external_uses(l),
        (CHECKOUT, UPLOAD, UPLOAD),
        "S1 external actions",
    )
    if local_uses(l):
        fail(f"S1 unexpectedly contains local reusable workflow calls: {local_uses(l)!r}")
    if exact_count(l, f'FETCH_IMAGE: "{GIT_FETCH_IMAGE}"') != 1:
        fail("S1 fetch sandbox image digest pin mismatch")
    if exact_count(l, f'SANDBOX_IMAGE: "{RUST_IMAGE}"') != 1:
        fail("S1 candidate sandbox image digest pin mismatch")
    if exact_count(l, f'TRUSTED_WORKFLOW_BLOB_SHA: "{expected_s1_sha}"') != 1:
        fail("S1 trusted workflow blob pin mismatch")
    require_no_fail_open_controls(l, "S1")
    require_no_yaml_reuse_syntax(l, S1)
    require_explicit_bash_for_run_steps(l, S1)
    for required in (
        "--network=bridge",
        "--network=none",
        "--read-only",
        "--cap-drop=ALL",
        "--security-opt=no-new-privileges:true",
        "--env GITHUB_TOKEN=",
        "--env GH_TOKEN=",
    ):
        if required not in "\n".join(l):
            fail(f"S1 isolation control missing: {required!r}")


def verify_s2(raw: bytes, expected_s0_sha: str, expected_s1_sha: str, expected_retention_sha: str, expected_execution_sha: str, expected_policy_sha: str) -> None:
    l = lines(raw)
    if exact_count(l, "name: Security Kernel Qualification — Trusted Result Verifier") != 1:
        fail("S2 name mismatch")
    if top_level_keys_after(l, "on:") != ("workflow_run",):
        fail("S2 trigger topology mismatch")
    if exact_count(l, 'workflows: ["Security Kernel Qualification — Trusted Dispatcher"]') != 1:
        fail("S2 workflow trigger mismatch")
    if exact_count(l, "types: [completed]") != 1:
        fail("S2 trigger type mismatch")
    require_permissions(l, 1)
    if exact_count(l, "cache-mode: none") != 1:
        fail("S2 must declare exactly one cache-mode: none gate")
    if exact_count(l, 'test "$GITHUB_REF_PROTECTED" = "true"') != 1:
        fail("S2 must contain exactly one protected-ref runtime guard")
    if exact_count(l, 'test "$GITHUB_REF" = "refs/heads/main"') != 1:
        fail("S2 must contain exactly one main-ref runtime guard")
    if job_keys(l) != ("verify",):
        fail("S2 job topology mismatch")
    if tuple(step_names(l)) != S2_STEPS:
        fail("S2 step topology mismatch")
    require_exact_actions(
        external_uses(l),
        (CHECKOUT, DOWNLOAD, DOWNLOAD),
        "S2 external actions",
    )
    if local_uses(l):
        fail("S2 unexpectedly contains local reusable workflow calls")
    for key, expected in (
        ("TRUSTED_DISPATCHER_WORKFLOW_BLOB_SHA", expected_s0_sha),
        ("TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA", expected_s1_sha),
        ("RETENTION_REFERENCE_VERIFIER_BLOB_SHA", expected_retention_sha),
    ):
        if exact_count(l, f'{key}: "{expected}"') != 1:
            fail(f"S2 {key} mismatch")
    if exact_count(l, 'RETENTION_REFERENCE_VERIFIER_PATH: ".github/security-kernel/reference_verify_evidence_retention_binding.py"') != 1:
        fail("S2 retention verifier path mismatch")
    if exact_count(l, 'EXECUTION_REFERENCE_VERIFIER_PATH: ".github/security-kernel/reference_verify_execution_binding.py"') != 1:
        fail("S2 execution verifier path mismatch")
    if exact_count(l, f'EXECUTION_REFERENCE_VERIFIER_BLOB_SHA: "{expected_execution_sha}"') != 1:
        fail("S2 execution verifier pin mismatch")
    if exact_count(l, 'SOURCE_POLICY_VERIFIER_PATH: ".github/security-kernel/reference_verify_source_policy.py"') != 1:
        fail("S2 source-policy verifier path mismatch")
    if exact_count(l, f'SOURCE_POLICY_VERIFIER_BLOB_SHA: "{expected_policy_sha}"') != 1:
        fail("S2 source-policy verifier self-pin mismatch")
    joined = "\n".join(l)
    for required in (
        'parsed_download_url = urllib.parse.urlparse(download_url)',
        'assert parsed_download_url.scheme == "https"',
        'assert not parsed_download_url.username and not parsed_download_url.password',
        'assert parsed_download_url.port in (None, 443)',
        'unexpected second HTTP redirect/status',
        'infos = archive.infolist()',
        'file_size <= 1024 * 1024',
        'compress_size <= 1024 * 1024',
    ):
        if required not in joined:
            fail(f"S2 artifact transport/decompression control missing: {required!r}")
    require_no_fail_open_controls(l, "S2")
    require_no_yaml_reuse_syntax(l, S2)
    require_explicit_bash_for_run_steps(l, S2)
    require_following(l, "Verify retained negative-control evidence binding", "if: success()", "S2 retention gate")
    require_following(l, "Download retained qualification receipt through official artifact client", "if: success()", "S2 receipt download gate")
    require_following(l, "Download retained sandbox negative-control transcript through official artifact client", "if: success()", "S2 transcript download gate")
    require_following(l, "Verify official receipt transport and publish verified result", "if: success()", "S2 final witness gate")
    require_no_escalation(l, "S2")


def verify_execution(raw: bytes) -> None:
    text = raw.decode("utf-8")
    if 'SCHEMA = "security-kernel-execution-binding-v2"' not in text:
        fail("execution oracle schema missing")
    if "urllib" in text or "requests" in text:
        fail("execution oracle unexpectedly contains network imports")
    if "repository-local" not in text:
        fail("execution oracle self-containment marker missing")
    if 'struct.pack(">Q"' not in text:
        fail("execution oracle must use the registered length-framed encoding")


def verify_retention(raw: bytes) -> None:
    l = lines(raw)
    text = "\n".join(l)
    if 'SCHEMA = "security-kernel-evidence-retention-binding-v1"' not in text:
        fail("retention oracle schema missing")
    if 'EXPECTED_REFERENCE_PATH = ".github/security-kernel/reference_verify_evidence_retention_binding.py"' not in text:
        fail("retention oracle registered path missing")
    if "urllib" in text or "requests" in text:
        fail("retention oracle unexpectedly contains network imports")
    if "subprocess" in text:
        fail("retention oracle must remain process-free")
    if "repository-local" not in text:
        fail("retention oracle self-containment marker missing")


def verify_file(record: dict, expected_path: str) -> bytes:
    if set(record) != {"path", "ref", "sha", "encoding", "content"}:
        fail("file snapshot schema drift")
    if record["path"] != expected_path or record["encoding"] != "base64":
        fail(f"file snapshot identity mismatch for {expected_path}")
    if not re.fullmatch(r"[0-9a-f]{40}", record["sha"]):
        fail(f"invalid Git blob SHA for {expected_path}")
    raw = base64.b64decode(record["content"], validate=True)
    computed = hashlib.sha1(f"blob {len(raw)}\0".encode("ascii") + raw).hexdigest()
    if computed != record["sha"]:
        fail(f"raw bytes do not hash to advertised Git blob SHA for {expected_path}")
    return raw


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_verify_source_policy.py <snapshot.json>")
    snapshot = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    if set(snapshot) != {"schema", "files"} or snapshot["schema"] != SCHEMA:
        fail("source-policy snapshot schema mismatch")
    files = snapshot["files"]
    expected = {"s0": S0, "s1": S1, "s2": S2, "retention": RETENTION, "execution": EXECUTION, "policy": POLICY}
    if set(files) != set(expected):
        fail("source-policy file census mismatch")
    raw = {
        name: verify_file(files[name], path)
        for name, path in expected.items()
    }
    s1_sha = files["s1"]["sha"]
    s0_sha = files["s0"]["sha"]
    retention_sha = files["retention"]["sha"]
    execution_sha = files["execution"]["sha"]
    verify_s0(raw["s0"], s1_sha)
    verify_s1(raw["s1"], s1_sha)
    verify_s2(raw["s2"], s0_sha, s1_sha, retention_sha, execution_sha, files["policy"]["sha"])
    verify_execution(raw["execution"])
    verify_retention(raw["retention"])

    # Regression: the oracle must reject a permission added after the approved
    # read-only entries rather than validating only a fixed three-line prefix.
    expect_rejection(
        lambda: verify_s0(inject_extra_permission(raw["s0"]), s1_sha),
        "S0 security-events: write",
    )
    expect_rejection(
        lambda: verify_s1(inject_extra_permission(raw["s1"]), s1_sha),
        "S1 security-events: write",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_extra_permission(raw["s2"]),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 security-events: write",
    )

    expect_rejection(
        lambda: verify_s0(inject_extra_permission_with_blank(raw["s0"]), s1_sha),
        "S0 blank-separated security-events: write",
    )
    expect_rejection(
        lambda: verify_s1(inject_extra_permission_with_blank(raw["s1"]), s1_sha),
        "S1 blank-separated security-events: write",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_extra_permission_with_blank(raw["s2"]),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 blank-separated security-events: write",
    )

    print(json.dumps({
        "schema": SCHEMA,
        "policy_result": "verified",
        "workflow_file_count": 3,
        "reference_file_count": 3,
        "action_pins_verified": 3,
        "exact_job_topologies_verified": 3,
        "exact_trigger_topologies_verified": 3,
        "exact_step_topologies_verified": 3,
    }, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
