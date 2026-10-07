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
VENDOR_VOLUME_CREATE = "docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=1024m,nr_inodes=150000"
VENDOR_VOLUME_RW = '--volume "$vendor_volume_name:/vendor:rw"'
VENDOR_VOLUME_RO = '--volume "$VENDOR_VOLUME_NAME:/vendor:ro"'
VENDOR_VOLUME_INSPECT = "vendor_volume_spec=\"$(docker volume inspect --format '{{.Driver}}|{{index .Options \"type\"}}|{{index .Options \"device\"}}|{{index .Options \"o\"}}' \"$vendor_volume_name\")\""
VENDOR_CONFIG_VOLUME_CREATE = "docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=16m,nr_inodes=64"
VENDOR_CONFIG_VOLUME_RW = '--volume "$vendor_config_volume_name:/vendor-config:rw"'
VENDOR_CONFIG_VOLUME_RO = '--volume "$VENDOR_CONFIG_VOLUME_NAME:/vendor-config:ro"'
VENDOR_CONFIG_VOLUME_INSPECT = "vendor_config_volume_spec=\"$(docker volume inspect --format '{{.Driver}}|{{index .Options \"type\"}}|{{index .Options \"device\"}}|{{index .Options \"o\"}}' \"$vendor_config_volume_name\")\""
SOURCE_RESOURCE_PROFILE = "v1"
SOURCE_MAX_BYTES = "1073741824"
SOURCE_MAX_INODES = "300000"
SOURCE_TMPFS_SIZE = "1024m"
SOURCE_TMPFS_NR_INODES = "300000"
SOURCE_VOLUME_CREATE = "docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=1024m,nr_inodes=300000"
SOURCE_VOLUME_RW = '--volume "$candidate_volume_name:/output:rw"'
SOURCE_VOLUME_RO = '--volume "$CANDIDATE_VOLUME_NAME:/source:ro"'
SOURCE_VOLUME_INSPECT = "candidate_volume_spec=\"$(docker volume inspect --format '{{.Driver}}|{{index .Options \"type\"}}|{{index .Options \"device\"}}|{{index .Options \"o\"}}' \"$candidate_volume_name\")\""
FETCH_TMPFS_PROFILE = "--tmpfs /tmp:rw,nosuid,nodev,noexec,size=512m,nr_inodes=600000"
SMALL_TMPFS_128_PROFILE = "--tmpfs /tmp:rw,nosuid,nodev,noexec,size=128m,nr_inodes=20000"
SMALL_TMPFS_64_PROFILE = "--tmpfs /tmp:rw,nosuid,nodev,noexec,size=64m,nr_inodes=8192"
CARGO_HOME_TMPFS_PROFILE = "--tmpfs /cargo-home:rw,nosuid,nodev,noexec,size=512m,nr_inodes=150000"
CANDIDATE_TMPFS_PROFILE = "--tmpfs /tmp:rw,nosuid,nodev,noexec,size=256m,nr_inodes=32768"
TARGET_TMPFS_PROFILE = "--tmpfs /target:rw,nosuid,nodev,size=8g,nr_inodes=600000"

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
    "Cleanup candidate source substrate",
    "Verify dependency substrate immutability",
    "Emit qualification receipt",
    "Upload qualification receipt",
    "Verify retained qualification receipt",
    "Final teardown barrier",
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
    if "continue-on-error:" in joined:
        fail(f"{description}: forbidden fail-open control continue-on-error")
    for line in lines_:
        if not re.match(r"^\s{6,8}if:\s*", line):
            continue
        if re.search(r"\balways\(\)|\bfailure\(\)", line):
            fail(f"{description}: forbidden fail-open status check: {line!r}")
        normalized = line.replace(" ", "")
        without_negated_cancelled = normalized.replace("!cancelled()", "")
        if "cancelled()" in without_negated_cancelled:
            fail(f"{description}: positive cancelled() status check is forbidden: {line!r}")


def require_no_fail_open_probe_conditions(lines_: list[str], description: str) -> None:
    for line in lines_:
        if re.match(
            r"\s*if\s+(?:docker|git|find|sort|df|du|sha256sum|stat|tar)\b.*\|\s*grep\b",
            line,
        ):
            fail(f"{description}: external probe failure is masked by an if-pipeline: {line!r}")


def require_explicit_bash_for_run_steps(lines_: list[str], description: str) -> None:
    current = None
    in_run_block = False

    def flush() -> None:
        nonlocal current, in_run_block
        if current is None:
            return
        if current["run"] and current["shell"] != ["bash"]:
            fail(f"{description}: run step {current['name']!r} must explicitly declare exactly one shell: bash")
        in_run_block = False

    for line in lines_:
        match = re.fullmatch(r"\s{6}- name: (.+)", line)
        if match:
            flush()
            current = {"name": match.group(1), "run": [], "shell": []}
            continue
        if current is None:
            continue
        if in_run_block:
            continue
        if re.fullmatch(r"\s{8}run:\s*\|?\s*", line):
            current["run"].append(line)
            in_run_block = True
        elif re.fullmatch(r"\s{8}shell:\s+bash\s*", line):
            current["shell"].append("bash")


def require_step_execution_modes(lines_: list[str], description: str) -> None:
    current = None
    in_run_block = False

    def flush() -> None:
        nonlocal current, in_run_block
        if current is None:
            return
        modes = current["run"] + current["uses"]
        if len(modes) != 1:
            fail(f"{description}: step {current['name']!r} must contain exactly one run/uses execution mode")
        if current["run"]:
            if current["shell"] != ["bash"]:
                fail(f"{description}: run step {current['name']!r} must explicitly declare exactly one shell: bash")
        elif current["shell"]:
            fail(f"{description}: uses step {current['name']!r} must not declare a shell")
        in_run_block = False

    for line in lines_:
        match = re.fullmatch(r"\s{6}- name: (.+)", line)
        if match:
            flush()
            current = {"name": match.group(1), "run": [], "uses": [], "shell": []}
            continue
        if current is None:
            continue
        if in_run_block:
            continue
        if re.fullmatch(r"\s{8}run:\s*\|?\s*", line):
            current["run"].append(line)
            in_run_block = True
        elif re.fullmatch(r"\s{8}uses:\s+.+", line):
            current["uses"].append(line)
        elif re.fullmatch(r"\s{8}shell:\s+(.+)\s*", line):
            current["shell"].append(re.fullmatch(r"\s{8}shell:\s+(.+)\s*", line).group(1))


def require_no_duplicate_step_keys(lines_: list[str], description: str) -> None:
    current_name = None
    counts = {}
    def flush() -> None:
        if counts:
            duplicates = sorted(key for key, count in counts.items() if count > 1)
            if duplicates:
                fail(f"{description}: duplicate step mapping keys under {current_name!r}: {duplicates!r}")
    for line in lines_:
        step_match = re.match(r"^\s{6}- name: (.+)$", line)
        if step_match:
            flush()
            current_name = step_match.group(1)
            counts = {}
            continue
        if re.match(r"^\s{6}- ", line):
            flush()
            current_name = None
            counts = {}
            continue
        if current_name is None:
            continue
        key_match = re.fullmatch(r"\s{8}([A-Za-z0-9_-]+):(?:\s+.*)?", line)
        if key_match:
            key = key_match.group(1)
            counts[key] = counts.get(key, 0) + 1
    flush()


def inject_run_step_conditional_layout(raw: bytes) -> bytes:
    marker = b"      - name: Execute candidate qualification in disposable networkless sandbox\n"
    insertion = marker + b"        if: success()\n"
    if marker not in raw:
        fail("conditional-step shell regression fixture marker missing")
    mutated = raw.replace(marker, insertion, 1)
    return mutated


def inject_run_step_without_shell(raw: bytes) -> bytes:
    marker = b"      - name: Execute candidate qualification in disposable networkless sandbox\n        shell: bash\n"
    replacement = b"      - name: Execute candidate qualification in disposable networkless sandbox\n"
    if marker not in raw:
        fail("missing-shell regression fixture marker missing")
    return raw.replace(marker, replacement, 1)


def inject_masked_docker_cleanup(raw: bytes) -> bytes:
    marker = b'          container_name="security-kernel-negative-controls-$RANDOM"\n'
    replacement = marker + b'          docker rm -f "$container_name" >/dev/null 2>&1 || true\n'
    if marker not in raw:
        fail("masked Docker cleanup regression fixture marker missing")
    return raw.replace(marker, replacement, 1)


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


def require_python_heredocs_compile(lines_: list[str], description: str) -> None:
    """Compile every Python heredoc after YAML block indentation is removed."""
    i = 0
    while i < len(lines_):
        line = lines_[i]
        match = re.fullmatch(r"(?P<indent> {8})run:\s*\|?\s*", line)
        if not match:
            i += 1
            continue
        content_indent = len(match.group("indent")) + 2
        i += 1
        while i < len(lines_):
            current = lines_[i]
            if current.strip() and len(current) - len(current.lstrip(" ")) < content_indent:
                break
            if "python3 -" in current and "<<" in current:
                j = i + 1
                body = []
                while j < len(lines_) and lines_[j].strip() != "PY":
                    heredoc_line = lines_[j]
                    if heredoc_line.strip():
                        indent = len(heredoc_line) - len(heredoc_line.lstrip(" "))
                        if indent < content_indent:
                            fail(f"{description}: Python heredoc escapes YAML block indentation at line {j + 1}")
                        body.append(heredoc_line[content_indent:])
                    else:
                        body.append("")
                    j += 1
                if j >= len(lines_):
                    fail(f"{description}: unterminated Python heredoc starting at line {i + 1}")
                delimiter_indent = len(lines_[j]) - len(lines_[j].lstrip(" "))
                if delimiter_indent != content_indent:
                    fail(f"{description}: Python heredoc delimiter indentation drift at line {j + 1}")
                try:
                    compile(
                        "\n".join(body),
                        f"<{description}-python-heredoc-{i + 1}>",
                        "exec",
                    )
                except SyntaxError as exc:
                    fail(f"{description}: Python heredoc syntax invalid: {exc}")
                i = j
            i += 1


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
    require_python_heredocs_compile(l, "VERIFY_S0")
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
    require_no_duplicate_step_keys(l, S0)
    require_step_execution_modes(l, S0)
    require_no_escalation(l, "S0")
    if any("git fetch " in x or "git checkout " in x or "actions/checkout@" in x for x in l):
        fail("S0 must remain metadata-only")


def verify_s1(raw: bytes, expected_s1_sha: str) -> None:
    l = lines(raw)
    require_python_heredocs_compile(l, "VERIFY_S1")
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
    if exact_count(l, 'TRUSTED_WORKFLOW_BLOB_SHA: ${{ inputs.trusted_workflow_blob_sha }}') != 1:
        fail("S1 trusted workflow blob input binding mismatch")
    if exact_count(l, 'CANDIDATE_REPOSITORY_ID: ${{ inputs.candidate_repository_id }}') != 1:
        fail("S1 candidate repository ID binding mismatch")
    for key, expected in (
        ("VENDOR_RESOURCE_PROFILE", "v2"), ("VENDOR_MAX_BYTES", "1073741824"),
        ("VENDOR_MAX_FILES", "100000"), ("VENDOR_MAX_INODES", "150000"),
        ("VENDOR_TMPFS_SIZE", "1024m"), ("VENDOR_TMPFS_NR_INODES", "150000"),
        ("VENDOR_CONFIG_RESOURCE_PROFILE", "v1"), ("VENDOR_CONFIG_MAX_BYTES", "16777216"),
        ("VENDOR_CONFIG_MAX_INODES", "64"), ("VENDOR_CONFIG_TMPFS_SIZE", "16m"),
        ("VENDOR_CONFIG_TMPFS_NR_INODES", "64"),
    ):
        if exact_count(l, f'  {key}: "{expected}"') != 1:
            fail(f"S1 vendor resource profile mismatch: {key}")
    for key, expected in (
        ("DEPENDENCY_MANIFEST_MAX_BYTES", "2097152"),
        ("DEPENDENCY_LOCK_MAX_BYTES", "33554432"),
    ):
        if exact_count(l, f'  {key}: "{expected}"') != 1:
            fail(f"S1 host-staging resource profile mismatch: {key}")
    require_no_fail_open_controls(l, "S1")
    require_no_fail_open_probe_conditions(l, "S1")
    joined = "\n".join(l)
    if "CANDIDATE_ROOT" in joined or "candidate_root" in joined or "security-kernel-candidate-root" in joined:
        fail("S1 must not contain a runner-backed candidate staging root")
    if 'test "$(stat -f -c \'%T\' "$candidate_volume_mountpoint")" = "tmpfs"' not in joined:
        fail("S1 candidate volume mountpoint must be independently confirmed as tmpfs")
    if 'python3 - "$CANDIDATE_REPOSITORY" "$CANDIDATE_REPOSITORY_ID" "$CANDIDATE_SHA" <<\'PY\' > "$RUNNER_TEMP/security-kernel-candidate-tree.env"' not in joined:
        fail("S1 candidate resolution must bind repository name, repository ID, and candidate SHA together")
    if 'assert repository_obj.get("full_name") == repository' not in joined:
        fail("S1 candidate resolution must revalidate repository full-name identity")
    if 'assert repository_obj.get("id") == repository_id' not in joined:
        fail("S1 candidate resolution must revalidate repository ID identity")
    if 'assert repository_obj.get("disabled") is not True' not in joined:
        fail("S1 candidate resolution must reject a disabled repository")
    if 'assert post_repository_obj.get("full_name") == repository' not in joined:
        fail("S1 candidate resolution must revalidate repository identity after commit resolution")
    if 'assert post_repository_obj.get("id") == repository_id' not in joined:
        fail("S1 candidate resolution must revalidate repository ID after commit resolution")
    if 'assert post_repository_obj.get("disabled") is not True' not in joined:
        fail("S1 candidate resolution must reject a repository disabled after commit resolution")
    if "Final teardown barrier" not in joined or "if: ${{ !cancelled() }}" not in joined:
        fail("S1 final teardown barrier missing or cancellation semantics weakened")
    if 'candidate_volume_name="security-kernel-source-$GITHUB_RUN_ID-$GITHUB_RUN_ATTEMPT"' not in joined:
        fail("S1 final teardown candidate volume identity missing")
    if 'vendor_volume_name="security-kernel-vendor-$GITHUB_RUN_ID-$GITHUB_RUN_ATTEMPT"' not in joined:
        fail("S1 final teardown vendor volume identity missing")
    if 'if ! existing_volumes="$(docker volume ls --format \'{{.Name}}\')"; then' not in joined:
        fail("S1 final teardown must inventory volumes fail-closed")
    if 'if ! docker volume rm "$volume_name" >/dev/null 2>&1; then' not in joined:
        fail("S1 final teardown volume removal must be fail-closed")
    if 'if ! remaining_volumes="$(docker volume ls --format \'{{.Name}}\')"; then' not in joined:
        fail("S1 final teardown must re-inventory after removal")
    if 'printf "%s\\n" "$existing_volumes" | grep -Fxq "$volume_name"' not in joined:
        fail("S1 final teardown ownership gate missing")
    if "final teardown volume remains" not in joined:
        fail("S1 final teardown volume disappearance check missing")
    if "Cleanup candidate source substrate" not in joined or "!cancelled()" not in joined:
        fail("S1 candidate source cleanup lifecycle missing")
    if "steps.resolve_source.outcome" not in joined:
        fail("S1 candidate source cleanup must be bound to successful acquisition")
    if 'if ! docker volume rm "$candidate_volume_name" >/dev/null 2>&1; then' not in joined:
        fail("S1 source volume failure-trap cleanup missing")
    if 'if ! docker volume rm "$CANDIDATE_VOLUME_NAME" >/dev/null 2>&1; then' not in joined:
        fail("S1 final candidate source volume cleanup missing")
    for key, expected in (("SOURCE_RESOURCE_PROFILE", SOURCE_RESOURCE_PROFILE), ("SOURCE_MAX_BYTES", SOURCE_MAX_BYTES), ("SOURCE_MAX_INODES", SOURCE_MAX_INODES), ("SOURCE_TMPFS_SIZE", SOURCE_TMPFS_SIZE), ("SOURCE_TMPFS_NR_INODES", SOURCE_TMPFS_NR_INODES)):
        if exact_count(l, f'  {key}: "{expected}"') != 1:
            fail(f"S1 source resource profile mismatch: {key}")
    tmpfs_lines = [line.strip() for line in l if "--tmpfs " in line]
    if len(tmpfs_lines) != 10:
        fail("S1 writable tmpfs mount census mismatch")
    if any("nr_inodes=" not in line for line in tmpfs_lines):
        fail("S1 every writable tmpfs mount must declare an inode ceiling")
    for profile, expected_count in (
        (FETCH_TMPFS_PROFILE, 1),
        (SMALL_TMPFS_128_PROFILE, 2),
        (SMALL_TMPFS_64_PROFILE, 3),
        (CARGO_HOME_TMPFS_PROFILE, 2),
        (CANDIDATE_TMPFS_PROFILE, 1),
        (TARGET_TMPFS_PROFILE, 1),
    ):
        observed = sum(1 for line in tmpfs_lines if profile in line)
        if observed != expected_count:
            fail(f"S1 tmpfs resource profile mismatch: {profile} (expected {expected_count}, got {observed})")
    if exact_count(l, SOURCE_VOLUME_CREATE) != 1:
        fail("S1 candidate source volume create profile mismatch")
    if exact_count(l, SOURCE_VOLUME_RW) != 1:
        fail("S1 candidate source volume write mount count mismatch")
    if exact_count(l, SOURCE_VOLUME_RO) != 3:
        fail("S1 candidate source volume read-only mount count mismatch")
    if exact_count(l, SOURCE_VOLUME_INSPECT) != 1:
        fail("S1 candidate source volume inspection count mismatch")
    if 'test "$candidate_volume_spec" = "local|tmpfs|tmpfs|rw,nosuid,nodev,noexec,size=1024m,nr_inodes=300000"' not in joined:
        fail("S1 candidate source volume instantiated profile mismatch")
    if 'test "$(stat -f -c \'%T\' "$candidate_volume_mountpoint")" = "tmpfs"' not in joined:
        fail("S1 candidate volume filesystem type check missing")
    for required in ("source_volume_name=","source_volume_spec=","source_copy_bytes=","source_copy_files=","source_copy_inodes=","source_volume_digest="):
        if required not in joined:
            fail(f"S1 candidate source volume receipt field missing: {required!r}")
    for required in ('test "$source_copy_bytes" -le "$SOURCE_MAX_BYTES"','test "$source_copy_files" -le 200000','test "$source_copy_inodes" -le "$SOURCE_MAX_INODES"'):
        if required not in joined:
            fail(f"S1 candidate source resource ceiling missing: {required!r}")
    if "source_copy_pipeline_status" in joined or "bounded_source_archive" in joined:
        fail("S1 obsolete host archive/staging pipeline residue detected")
    if exact_count(l, VENDOR_VOLUME_CREATE) != 1:
        fail("S1 vendor resource volume create profile mismatch")
    if exact_count(l, VENDOR_VOLUME_RW) != 1:
        fail("S1 vendor acquisition must use one bounded Docker volume for writes")
    if exact_count(l, VENDOR_VOLUME_RO) != 3:
        fail("S1 bounded vendor volume read-only mount count mismatch")
    if exact_count(l, VENDOR_VOLUME_INSPECT) != 1:
        fail("S1 vendor volume instantiation must be independently inspected")
    if exact_count(l, VENDOR_CONFIG_VOLUME_CREATE) != 1:
        fail("S1 vendor-config resource volume create profile mismatch")
    if exact_count(l, VENDOR_CONFIG_VOLUME_RW) != 1:
        fail("S1 vendor-config acquisition must use one bounded Docker volume for writes")
    if exact_count(l, VENDOR_CONFIG_VOLUME_RO) != 3:
        fail("S1 bounded vendor-config volume read-only mount count mismatch")
    if exact_count(l, VENDOR_CONFIG_VOLUME_INSPECT) != 1:
        fail("S1 vendor-config volume instantiation must be independently inspected")
    if 'if docker volume inspect "$vendor_volume_name"' in joined:
        fail("S1 vendor collision detection must not treat volume-inspect failure as absence")
    if 'if [ "$status" -ne 0 ] && docker volume inspect "$vendor_volume_name"' in joined:
        fail("S1 vendor cleanup must not use masked volume-inspect status")
    for required in (
        'if ! existing_vendor_volumes="$(docker volume ls --format \'{{.Name}}\')"; then',
        'if ! remaining_vendor_volumes="$(docker volume ls --format \'{{.Name}}\')"; then',
        'if ! cleanup_vendor_volumes="$(docker volume ls --format \'{{.Name}}\')"; then',
        'if printf "%s\\n" "$cleanup_vendor_volumes" | grep -Fxq "$vendor_config_volume_name"; then',
        'if ! docker volume rm "$vendor_config_volume_name" >/dev/null 2>&1; then status=1; fi',
        'if printf "%s\\n" "$remaining_vendor_volumes" | grep -Fxq "$vendor_config_volume_name"; then status=1; fi',
    ):
        if required not in joined:
            fail(f"S1 vendor fail-closed inventory control missing: {required!r}")
    if 'test "$vendor_volume_spec" = "local|tmpfs|tmpfs|rw,nosuid,nodev,noexec,size=1024m,nr_inodes=150000"' not in joined:
        fail("S1 vendor volume instantiated options mismatch")
    if 'test "$vendor_config_volume_spec" = "local|tmpfs|tmpfs|rw,nosuid,nodev,noexec,size=16m,nr_inodes=64"' not in joined:
        fail("S1 vendor-config volume instantiated options mismatch")
    if "cargo vendor --locked --manifest-path /source/crates/mycelix-bridge-common/Cargo.toml /vendor > /vendor-config/config.toml" not in joined:
        fail("S1 cargo vendor must read candidate dependencies directly from bounded source volume")
    if "dependency_source_mode=bounded-candidate-volume" not in joined:
        fail("S1 dependency subject must remain inside the bounded candidate volume")
    if "DEPENDENCY_ROOT:" in joined or "/subject/Cargo.toml" in joined:
        fail("S1 host-backed dependency subject residue detected")
    if 'vendor_config="$RUNNER_TEMP/' in joined:
        fail("S1 vendor-config must not use a host-backed runner-temp file")
    if 'if ! docker volume rm "$vendor_config_volume_name" >/dev/null 2>&1; then :; fi' in joined:
        fail("S1 vendor-config cleanup must not mask Docker volume-removal failures")
    if '--volume "$vendor_root:/vendor:rw"' in joined or '--volume "$VENDOR_ROOT:/vendor:rw"' in joined:
        fail("S1 vendor acquisition must not use a host-backed writable vendor directory")
    if "negative_controls_capture_limit=65536" not in joined:
        fail("S1 negative-control transcript capture must declare a 64 KiB host-storage ceiling")
    if "capture_negative_controls_output() {" not in joined:
        fail("S1 negative-control transcript bounded capture function missing")
    if "} 2>&1 | capture_negative_controls_output" not in joined:
        fail("S1 negative-control container output must pass through bounded capture")
    if 'pipeline_status=("${PIPESTATUS[@]}")' not in joined:
        fail("S1 negative-control Docker/capture pipeline status must be preserved independently")
    if 'test "${pipeline_status[0]}" -eq 0' not in joined or 'test "${pipeline_status[1]}" -eq 0' not in joined:
        fail("S1 negative-control Docker and capture failures must both fail closed")
    for required in (
        'vendor_volume_name="security-kernel-vendor-$GITHUB_RUN_ID-$GITHUB_RUN_ATTEMPT"',
        'vendor_config_volume_name="security-kernel-vendor-config-$GITHUB_RUN_ID-$GITHUB_RUN_ATTEMPT"',
        "vendor_observed_bytes=", "vendor_observed_files=", "vendor_observed_inodes=",
        "vendor_config_observed_bytes=", "vendor_config_observed_inodes=", "vendor_config_digest=",
        "vendor_cleanup_on_failure", "docker volume rm \"$vendor_volume_name\"",
        "docker volume rm \"$vendor_config_volume_name\"",
        "vendor volume remains after cleanup", "vendor-config volume remains after cleanup",
        "dependency_substrate=passed",
    ):
        if required not in joined:
            fail(f"S1 vendor resource-bound control missing: {required!r}")
    if any("docker rm -f " in line and "|| true" in line for line in l):
        fail("S1 sandbox setup/cleanup must not force-remove an existing container or mask Docker errors")
    for required in (
        'if docker ps -aq --filter "name=^/${container_name}$" | grep -q .; then',
        "security-kernel negative-control container name collision",
        'if docker ps -aq --filter "name=^/${name}$" | grep -q .; then',
        "security-kernel sandbox container name collision",
        'remaining="$(docker ps -aq --filter "name=^/${name}$")"',
        'if ! docker rm -f "$name" >/dev/null 2>&1; then',
        "security-kernel sandbox cleanup failed",
        "security-kernel sandbox container remains after cleanup",
    ):
        if required not in joined:
            fail(f"S1 fail-closed sandbox teardown control missing: {required!r}")
    require_no_yaml_reuse_syntax(l, S1)
    require_explicit_bash_for_run_steps(l, S1)
    require_no_duplicate_step_keys(l, S1)
    require_step_execution_modes(l, S1)
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
    require_python_heredocs_compile(l, "VERIFY_S2")
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
        'artifact_size <= 65536',
        'downloaded = response.read(65536 + 1)',
        'file_size <= 1024 * 1024',
        'compress_size <= 1024 * 1024',
    ):
        if required not in joined:
            fail(f"S2 artifact transport/decompression control missing: {required!r}")
    require_no_fail_open_controls(l, "S2")
    require_no_yaml_reuse_syntax(l, S2)
    require_explicit_bash_for_run_steps(l, S2)
    require_no_duplicate_step_keys(l, S2)
    require_step_execution_modes(l, S2)
    require_following(l, "Verify retained negative-control evidence binding", "if: success()", "S2 retention gate")
    require_following(l, "Download retained qualification receipt through official artifact client", "if: success()", "S2 receipt download gate")
    require_following(l, "Download retained sandbox negative-control transcript through official artifact client", "if: success()", "S2 transcript download gate")
    require_following(l, "Verify official receipt transport and publish verified result", "if: success()", "S2 final witness gate")
    require_no_escalation(l, "S2")


def verify_execution(raw: bytes) -> None:
    text = raw.decode("utf-8")
    if 'SCHEMA = "security-kernel-execution-binding-v3"' not in text:
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
    verify_s1(inject_run_step_conditional_layout(raw["s1"]), s1_sha)

    expect_rejection(
        lambda: verify_s1(
            inject_run_step_without_shell(raw["s1"]),
            s1_sha,
        ),
        "run step missing explicit shell",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b'              tree_entries="$(git -C /tmp/repository ls-tree -r "$resolved")"',
                b'              if git -C /tmp/repository ls-tree -r "$resolved" | grep -q "^160000 "; then',
                1,
            ),
            s1_sha,
        ),
        "unchecked git pipeline inside conditional",
    )


    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(
                b"          execution_reference_input = dict(evidence_binding)\\n",
                b"           execution_reference_input = dict(evidence_binding)\\n",
                1,
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 Python heredoc indentation drift",
    )

    expect_rejection(
        lambda: verify_s1(
            inject_masked_docker_cleanup(raw["s1"]),
            s1_sha,
        ),
        "masked pre-run Docker cleanup",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_VOLUME_RW.encode(), b'--volume "$vendor_root:/vendor:rw"', 1), s1_sha),
        "writable host-backed vendor root",
    )

    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b'assert repository_obj.get("id") == repository_id, "GitHub candidate repository ID changed during candidate resolution"\n',
                b'# repository ID temporal check removed\n',
                1,
            ),
            s1_sha,
        ),
        "candidate repository ID revalidation removed",
    )

    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b'assert post_repository_obj.get("id") == repository_id, "GitHub candidate repository ID changed after candidate commit resolution"\n',
                b'# post-resolution repository ID temporal check removed\n',
                1,
            ),
            s1_sha,
        ),
        "candidate repository ID post-resolution revalidation removed",
    )


    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(b"trap negative_control_cleanup_on_exit EXIT", b"# negative-control cleanup trap removed", 1),
            s1_sha,
        ),
        "negative-control container cleanup trap removed",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"        if: success()\n",
                b"        if: ${{ always() }}\n",
                1,
            ),
            s1_sha,
        ),
        "expression-wrapped always() status check",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"        if: success()\n",
                b"        if: ${{ failure() }}\n",
                1,
            ),
            s1_sha,
        ),
        "expression-wrapped failure() status check",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"        if: ${{ !cancelled() }}\n",
                b"        if: ${{ cancelled() }}\n",
                1,
            ),
            s1_sha,
        ),
        "positive cancelled() cleanup condition",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"        if: ${{ !cancelled() }}\n",
                b"        if: ${{ !cancelled() && cancelled() }}\n",
                1,
            ),
            s1_sha,
        ),
        "mixed negated and positive cancelled() status condition",
    )



    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"          set -euo pipefail\n",
                b"          set -euo pipefail\n          if sort /dev/null | grep -q .; then exit 1; fi\n",
                1,
            ),
            s1_sha,
        ),
        "unchecked sort pipeline inside conditional",
    )

    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b'test "$(stat -f -c \'%T\' "$candidate_volume_mountpoint")" = "tmpfs"\n',
                b"# candidate tmpfs identity removed\n",
                1,
            ),
            s1_sha,
        ),
        "candidate Docker tmpfs identity check removed",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b'--volume "$CANDIDATE_VOLUME_NAME:/source:ro"',
                b'--volume "$candidate_root:/source:ro"',
                1,
            ),
            s1_sha,
        ),
        "candidate source read-only mount replaced with host-backed root",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"dependency_source_mode=bounded-candidate-volume",
                b"dependency_source_mode=host-copy",
                1,
            ),
            s1_sha,
        ),
        "host-backed dependency subject reintroduced",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(b"negative_controls_capture_limit=65536", b"negative_controls_capture_limit=1", 1), s1_sha),
        "negative-control transcript capture ceiling weakened",
    )
    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(b"artifact_size <= 65536", b"artifact_size <= 1048576", 1),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 retained negative-control artifact ceiling weakened",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(b'pipeline_status=("${PIPESTATUS[@]}")\n', b"# negative-control pipeline status capture removed\n", 1), s1_sha),
        "negative-control pipeline status capture removed",
    )

    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_VOLUME_CREATE.encode(), b'docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=1024m', 1), s1_sha),
        "vendor tmpfs without inode ceiling",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_VOLUME_INSPECT.encode(), b'vendor_volume_spec="wrong|volume|driver|options"', 1), s1_sha),
        "vendor volume instantiated-option mismatch",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_CONFIG_VOLUME_CREATE.encode(), b'docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=16m', 1), s1_sha),
        "vendor-config tmpfs without inode ceiling",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_CONFIG_VOLUME_INSPECT.encode(), b'vendor_config_volume_spec="wrong|volume|driver|options"', 1), s1_sha),
        "vendor-config volume instantiated-option mismatch",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_CONFIG_VOLUME_RW.encode(), b'--volume "$vendor_root:/vendor-config:rw"', 1), s1_sha),
        "writable host-backed vendor-config root",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(b'cargo vendor --locked --manifest-path /subject/Cargo.toml /vendor > /vendor-config/config.toml', b'cargo vendor --locked --manifest-path /subject/Cargo.toml /vendor > /vendor-config', 1), s1_sha),
        "unbounded vendor-config output target",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(b'if ! docker volume rm "$VENDOR_CONFIG_VOLUME_NAME" >/dev/null; then\n', b'# vendor-config cleanup removed\n', 1), s1_sha),
        "vendor-config cleanup removed",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(b'if ! docker volume rm "$vendor_config_volume_name" >/dev/null 2>&1; then status=1; fi', b'if ! docker volume rm "$vendor_config_volume_name" >/dev/null 2>&1; then :; fi', 1), s1_sha),
        "masked vendor-config cleanup error",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(SOURCE_VOLUME_CREATE.encode(), b'docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=1024m', 1), s1_sha),
        "candidate source tmpfs without inode ceiling",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(SOURCE_VOLUME_RW.encode(), b'--volume "$candidate_root:/output:rw"', 1), s1_sha),
        "writable host-backed candidate source root",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(SOURCE_VOLUME_INSPECT.encode(), b'candidate_volume_spec="wrong|volume|driver|options"', 1), s1_sha),
        "candidate source instantiated-option mismatch",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(b"trap source_volume_cleanup_on_failure EXIT", b"# source cleanup trap removed", 1), s1_sha),
        "candidate source cleanup trap removed",
    )


    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(
                f'TRUSTED_DISPATCHER_WORKFLOW_BLOB_SHA: "{s0_sha}"'.encode(),
                b'TRUSTED_DISPATCHER_WORKFLOW_BLOB_SHA: "' + b"0" * 40 + b'"',
                1,
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 stale S0 dispatcher trust pin",
    )
    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(
                f'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "{s1_sha}"'.encode(),
                b'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "' + b"0" * 40 + b'"',
                1,
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 stale S1 qualification trust pin",
    )
    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(
                f'SOURCE_POLICY_VERIFIER_BLOB_SHA: "{files["policy"]["sha"]}"'.encode(),
                b'SOURCE_POLICY_VERIFIER_BLOB_SHA: "' + b"0" * 40 + b'"',
                1,
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 stale source-policy oracle trust pin",
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
