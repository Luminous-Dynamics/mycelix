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


def top_level_keys(lines_: list[str]) -> tuple[str, ...]:
    return tuple(
        match.group(1)
        for line in lines_
        if (match := re.fullmatch(r"([A-Za-z0-9_-]+):\s*", line))
    )


def require_exact_top_level_keys(
    lines_: list[str],
    expected: tuple[str, ...],
    description: str,
) -> None:
    actual = top_level_keys(lines_)
    if actual != expected:
        fail(f"{description}: top-level key census mismatch: expected {expected!r}, found {actual!r}")


def job_keys(lines_: list[str]) -> tuple[str, ...]:
    try:
        start = next(i for i, line in enumerate(lines_) if line.strip() == "jobs:")
    except StopIteration:
        fail("missing jobs block")
    found = []
    for line in lines_[start + 1 :]:
        if line and not line.startswith((" ", "\t")):
            break
        match = re.fullmatch(r"  ([A-Za-z0-9_-]+):\s*", line)
        if match:
            found.append(match.group(1))
    return tuple(found)


def step_names(lines_: list[str]) -> tuple[str, ...]:
    names = []
    for line in lines_:
        if not re.fullmatch(r"\s{6}-\s+(.+)", line):
            continue
        match = re.fullmatch(r"\s{6}- name: (.+)", line)
        if not match:
            fail(f"workflow step item must use the closed-world '- name:' form: {line!r}")
        names.append(match.group(1))
    return tuple(names)


def require_exact_job_keys(
    lines_: list[str],
    expected: tuple[tuple[str, tuple[str, ...]], ...],
    description: str,
) -> None:
    actual = []
    start = next((i for i, line in enumerate(lines_) if line.strip() == "jobs:"), None)
    if start is None:
        fail(f"{description}: missing jobs block")
    current_job = None
    current_keys = []

    def flush() -> None:
        if current_job is None:
            return
        actual.append((current_job, tuple(current_keys)))

    for line in lines_[start + 1 :]:
        if line and not line.startswith((" ", "\t")):
            break
        job_match = re.fullmatch(r"  ([A-Za-z0-9_-]+):\s*", line)
        if job_match:
            flush()
            current_job = job_match.group(1)
            current_keys = []
            continue
        if current_job is None:
            continue
        key_match = re.fullmatch(r"    ([A-Za-z0-9_-]+):(?:\s+.*)?", line)
        if key_match:
            current_keys.append(key_match.group(1))

    flush()
    if tuple(actual) != expected:
        fail(f"{description}: job key census mismatch: expected {expected!r}, found {actual!r}")


def require_exact_root_mapping(
    lines_: list[str],
    mapping_name: str,
    expected: tuple[str, ...],
    description: str,
) -> None:
    indexes = [
        i
        for i, line in enumerate(lines_)
        if line.strip() == f"{mapping_name}:" and not line.startswith((" ", "\t"))
    ]
    if len(indexes) != 1:
        fail(f"{description}: expected exactly one root {mapping_name!r} mapping")
    index = indexes[0]
    actual = []
    for line in lines_[index + 1:]:
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent <= 0:
            break
        if indent != 2:
            fail(f"{description}: unexpected {mapping_name} indentation: {line!r}")
        match = re.fullmatch(r"([A-Za-z0-9_-]+):\s+(.+)", line.strip())
        if not match:
            fail(f"{description}: malformed {mapping_name} entry: {line!r}")
        actual.append(f"{match.group(1)}: {match.group(2)}")
    if tuple(actual) != expected:
        fail(f"{description}: {mapping_name} mismatch: expected {expected!r}, found {tuple(actual)!r}")
def require_exact_job_mapping(
    lines_: list[str],
    job_name: str,
    mapping_name: str,
    expected: tuple[str, ...],
    description: str,
) -> None:
    matches = [i for i, line in enumerate(lines_) if line.strip() == f"{job_name}:"]
    if len(matches) != 1:
        fail(f"{description}: expected exactly one job named {job_name!r}")
    start = matches[0]
    heading = f"{mapping_name}:"
    indexes = []
    for i in range(start + 1, len(lines_)):
        if lines_[i].strip() == heading and len(lines_[i]) - len(lines_[i].lstrip(" ")) == 4:
            indexes.append(i)
    if len(indexes) != 1:
        fail(f"{description}: expected exactly one {mapping_name!r} mapping under {job_name!r}")
    index = indexes[0]
    actual = []
    for line in lines_[index + 1:]:
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent <= 4:
            break
        if indent != 6:
            fail(f"{description}: unexpected {mapping_name} indentation: {line!r}")
        match = re.fullmatch(r"([A-Za-z0-9_-]+):\s+(.+)", line.strip())
        if not match:
            fail(f"{description}: malformed {mapping_name} entry: {line!r}")
        actual.append(f"{match.group(1)}: {match.group(2)}")
    if tuple(actual) != expected:
        fail(f"{description}: {mapping_name} mismatch: expected {expected!r}, found {tuple(actual)!r}")


def require_exact_step_mapping(
    lines_: list[str],
    step_name: str,
    mapping_name: str,
    expected: tuple[str, ...],
    description: str,
) -> None:
    matches = [i for i, line in enumerate(lines_) if line.strip() == f"- name: {step_name}"]
    if len(matches) != 1:
        fail(f"{description}: expected exactly one step named {step_name!r}")
    start = matches[0]
    heading = f"{mapping_name}:"
    mapping_indexes = []
    for i in range(start + 1, len(lines_)):
        if lines_[i].strip() == heading and len(lines_[i]) - len(lines_[i].lstrip(" ")) == 8:
            if any(re.match(r"^\s{6}- name: ", line) for line in lines_[start + 1:i]):
                break
            mapping_indexes.append(i)
    if len(mapping_indexes) != 1:
        fail(f"{description}: expected exactly one {mapping_name!r} mapping under {step_name!r}")
    index = mapping_indexes[0]
    actual = []
    for line in lines_[index + 1:]:
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent <= 8:
            break
        if indent != 10:
            fail(f"{description}: unexpected {mapping_name} indentation: {line!r}")
        match = re.fullmatch(r"([A-Za-z0-9_-]+):\s+(.+)", line.strip())
        if not match:
            fail(f"{description}: malformed {mapping_name} entry: {line!r}")
        actual.append(f"{match.group(1)}: {match.group(2)}")
    if tuple(actual) != expected:
        fail(f"{description}: {mapping_name} mismatch: expected {expected!r}, found {tuple(actual)!r}")


def require_exact_step_keys(
    lines_: list[str],
    expected: tuple[tuple[str, tuple[str, ...]], ...],
    description: str,
) -> None:
    actual = []
    current_name = None
    current_keys = []

    def flush() -> None:
        if current_name is None:
            return
        actual.append((current_name, tuple(current_keys)))

    for line in lines_:
        match = re.fullmatch(r"\s{6}- name: (.+)", line)
        if match:
            flush()
            current_name = match.group(1)
            current_keys = ["name"]
            continue
        if current_name is None:
            continue
        match = re.fullmatch(r"\s{8}([A-Za-z0-9_-]+):(?:\s+.*)?", line)
        if match:
            current_keys.append(match.group(1))

    flush()
    if tuple(actual) != expected:
        fail(f"{description}: step key census mismatch: expected {expected!r}, found {actual!r}")


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


def require_no_fail_open_probe_conditions(lines_: list[str], description: str) -> None:
    for line in lines_:
        if re.match(r"\s*if\s+(?:docker|git|find)\b.*\|\s*grep\b", line):
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


def require_exact_step_conditionals(
    lines_: list[str],
    expected: tuple[tuple[str, str | None], ...],
    description: str,
) -> None:
    actual = []
    current_name = None
    current_conditionals = []

    def flush() -> None:
        if current_name is None:
            return
        if len(current_conditionals) > 1:
            fail(f"{description}: step {current_name!r} contains duplicate if mappings")
        actual.append((current_name, current_conditionals[0] if current_conditionals else None))

    for line in lines_:
        match = re.fullmatch(r"\s{6}- name: (.+)", line)
        if match:
            flush()
            current_name = match.group(1)
            current_conditionals = []
            continue
        if current_name is None:
            continue
        match = re.fullmatch(r"\s{8}if:\s*(.+)", line)
        if match:
            current_conditionals.append(match.group(1).strip())

    flush()
    if tuple(actual) != expected:
        fail(f"{description}: step conditional census mismatch: expected {expected!r}, found {actual!r}")


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


def require_no_quoted_structural_keys(lines_: list[str], description: str) -> None:
    for line in lines_:
        indentation = len(line) - len(line.lstrip(" "))
        if indentation > 8:
            continue
        if re.search(r'(?:^|[{,]\s*)(?:"[^"\n]*"|\'[^\'\n]*\')\s*:', line):
            fail(f"{description}: quoted YAML mapping keys are forbidden: {line!r}")


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
    require_exact_top_level_keys(l, ("name", "run-name", "on", "permissions", "concurrency", "env", "jobs"), S0)
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
    require_exact_job_keys(
        l,
        (
            ("resolve", ("name", "runs-on", "cache-mode", "timeout-minutes", "outputs", "steps")),
            ("qualify", ("name", "needs", "permissions", "cache-mode", "uses", "with")),
        ),
        S0,
    )
    for expected in (
        'runs-on: ubuntu-24.04',
        'cache-mode: none',
        'timeout-minutes: 10',
        'group: security-kernel-trusted-dispatch-pr-${{ github.event.pull_request.number }}',
        'cancel-in-progress: true',
    ):
        expected_count = 2 if expected == 'runs-on: ubuntu-24.04' or expected == 'cache-mode: none' else 1
        if exact_count(l, expected) != expected_count:
            fail(f"S0 scalar/concurrency value mismatch: {expected!r}")
    if step_names(l) != ("Verify trusted dispatcher context and exact PR identity",):
        fail("S0 step topology mismatch")
    if external_uses(l) != ():
        fail(f"S0 external action census mismatch: {external_uses(l)!r}")
    if local_uses(l) != ("./.github/workflows/security-kernel-independent-qualification.yml",):
        fail("S0 local reusable workflow census mismatch")
    require_exact_root_mapping(
        l,
        "env",
        (
            'BASE_REPOSITORY: "Luminous-Dynamics/mycelix"',
            'BASE_REPOSITORY_ID: "1176351975"',
            'BASE_BRANCH: "main"',
            'TRUSTED_INDEPENDENT_WORKFLOW_PATH: ".github/workflows/security-kernel-independent-qualification.yml"',
            'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "156f00beeb50f269847a1d3cd9bbb50aa8ec5514"',
            'TRUSTED_DISPATCHER_WORKFLOW_PATH: ".github/workflows/security-kernel-trusted-dispatch.yml"',
        ),
        "S0",
    )
    require_exact_job_mapping(
        l,
        "qualify",
        "with",
        (
            "candidate_pr: ${{ needs.resolve.outputs.candidate_pr }}",
            "candidate_sha: ${{ needs.resolve.outputs.candidate_sha }}",
            "candidate_repository: ${{ needs.resolve.outputs.candidate_repository }}",
            "candidate_repository_id: ${{ needs.resolve.outputs.candidate_repository_id }}",
            "trusted_workflow_blob_sha: ${{ inputs.trusted_workflow_blob_sha }}",
        ),
        S0,
    )
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
    require_no_quoted_structural_keys(l, S0)
    require_no_yaml_reuse_syntax(l, S0)
    require_explicit_bash_for_run_steps(l, S0)
    require_no_duplicate_step_keys(l, S0)
    require_step_execution_modes(l, S0)
    require_exact_step_keys(
        l,
        (("Verify trusted dispatcher context and exact PR identity", ("name", "id", "env", "shell", "run")),),
        S0,
    )
    require_exact_step_conditionals(l, (("Verify trusted dispatcher context and exact PR identity", None),), S0)
    require_no_escalation(l, "S0")
    if any("git fetch " in x or "git checkout " in x or "actions/checkout@" in x for x in l):
        fail("S0 must remain metadata-only")


def verify_s1(raw: bytes, expected_s1_sha: str) -> None:
    l = lines(raw)
    require_exact_top_level_keys(l, ("name", "on", "permissions", "cache-mode", "concurrency", "env", "jobs"), S1)
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
    require_exact_job_keys(
        l,
        (("qualify", ("name", "runs-on", "timeout-minutes", "steps")),),
        S1,
    )
    require_exact_root_mapping(
        l,
        "env",
        (
            'BASE_REPOSITORY: "Luminous-Dynamics/mycelix"',
            'BASE_REPOSITORY_ID: "1176351975"',
            'VENDOR_RESOURCE_PROFILE: "v2"',
            'VENDOR_MAX_BYTES: "1073741824"',
            'VENDOR_MAX_FILES: "100000"',
            'VENDOR_MAX_INODES: "150000"',
            'VENDOR_TMPFS_SIZE: "1024m"',
            'VENDOR_TMPFS_NR_INODES: "150000"',
        ),
        "S1",
    )
    if tuple(step_names(l)) != S1_STEPS:
        fail("S1 step topology mismatch")
    require_exact_actions(
        external_uses(l),
        (CHECKOUT, UPLOAD, UPLOAD),
        "S1 external actions",
    )
    for expected in (
        ("runs-on: ubuntu-24.04", 1),
        ("timeout-minutes: 45", 1),
        ("cache-mode: none", 1),
    ):
        if exact_count(l, expected[0]) != expected[1]:
            fail(f"S1 job value mismatch: {expected[0]!r}")
    require_exact_step_mapping(
        l,
        "Checkout trusted qualification root",
        "with",
        (
            "ref: ${{ github.workflow_sha }}",
            "fetch-depth: 0",
            "persist-credentials: false",
        ),
        S1,
    )
    require_exact_step_mapping(
        l,
        "Upload sandbox negative-control transcript",
        "with",
        (
            "name: security-kernel-negative-controls-${{ inputs.candidate_sha }}-attempt-${{ github.run_attempt }}.log",
            "path: ${{ runner.temp }}/security-kernel-negative-controls.log",
            "if-no-files-found: error",
            "archive: false",
            "overwrite: false",
            "retention-days: 90",
        ),
        S1,
    )
    require_exact_step_mapping(
        l,
        "Upload qualification receipt",
        "with",
        (
            "name: security-kernel-independent-qualification-${{ inputs.candidate_sha }}-attempt-${{ github.run_attempt }}.txt",
            "path: ${{ runner.temp }}/security-kernel-independent-qualification-${{ inputs.candidate_sha }}-attempt-${{ github.run_attempt }}.txt",
            "if-no-files-found: error",
            "archive: false",
            "overwrite: false",
            "retention-days: 90",
        ),
        S1,
    )
    if local_uses(l):
        fail(f"S1 unexpectedly contains local reusable workflow calls: {local_uses(l)!r}")
    if exact_count(l, f'FETCH_IMAGE: "{GIT_FETCH_IMAGE}"') != 1:
        fail("S1 fetch sandbox image digest pin mismatch")
    if exact_count(l, f'SANDBOX_IMAGE: "{RUST_IMAGE}"') != 1:
        fail("S1 candidate sandbox image digest pin mismatch")
    if exact_count(l, 'TRUSTED_WORKFLOW_BLOB_SHA: ${{ inputs.trusted_workflow_blob_sha }}') != 1:
        fail("S1 trusted workflow blob input binding mismatch")
    for key, expected in (("VENDOR_RESOURCE_PROFILE", "v2"), ("VENDOR_MAX_BYTES", "1073741824"), ("VENDOR_MAX_FILES", "100000"), ("VENDOR_MAX_INODES", "150000"), ("VENDOR_TMPFS_SIZE", "1024m"), ("VENDOR_TMPFS_NR_INODES", "150000")):
        if exact_count(l, f'  {key}: "{expected}"') != 1:
            fail(f"S1 vendor resource profile mismatch: {key}")
    require_no_fail_open_controls(l, "S1")
    require_no_fail_open_probe_conditions(l, "S1")
    joined = "\n".join(l)
    if exact_count(l, VENDOR_VOLUME_CREATE) != 1:
        fail("S1 vendor resource volume create profile mismatch")
    if exact_count(l, VENDOR_VOLUME_RW) != 1:
        fail("S1 vendor acquisition must use one bounded Docker volume for writes")
    if exact_count(l, VENDOR_VOLUME_RO) != 3:
        fail("S1 bounded vendor volume read-only mount count mismatch")
    if exact_count(l, VENDOR_VOLUME_INSPECT) != 1:
        fail("S1 vendor volume instantiation must be independently inspected")
    if 'test "$vendor_volume_spec" = "local|tmpfs|tmpfs|rw,nosuid,nodev,noexec,size=1024m,nr_inodes=150000"' not in joined:
        fail("S1 vendor volume instantiated options mismatch")
    if '--volume "$vendor_root:/vendor:rw"' in joined or '--volume "$VENDOR_ROOT:/vendor:rw"' in joined:
        fail("S1 vendor acquisition must not use a host-backed writable vendor directory")
    for required in ("vendor_volume_name=\"security-kernel-vendor-$GITHUB_RUN_ID-$GITHUB_RUN_ATTEMPT\"", "vendor_observed_bytes=", "vendor_observed_files=", "vendor_observed_inodes=", "vendor_cleanup_on_failure", "docker volume rm \"$vendor_volume_name\"", "vendor volume remains after cleanup", "dependency_substrate=passed"):
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
    require_no_quoted_structural_keys(l, S1)
    require_no_yaml_reuse_syntax(l, S1)
    require_explicit_bash_for_run_steps(l, S1)
    require_no_duplicate_step_keys(l, S1)
    require_step_execution_modes(l, S1)
    require_exact_step_keys(
        l,
        (
            ("Checkout trusted qualification root", ("name", "uses", "with")),
            ("Verify trusted pull-request-target invocation", ("name", "env", "shell", "run")),
            ("Resolve exact candidate source", ("name", "id", "env", "shell", "run")),
            ("Static trust-surface audit", ("name", "env", "shell", "run")),
            ("Snapshot exact candidate source identity", ("name", "id", "env", "shell", "run")),
            ("Snapshot locked dependency identity", ("name", "id", "env", "shell", "run")),
            ("Pull and preflight pinned sandbox image", ("name", "id", "env", "shell", "run")),
            ("Prepare locked dependency subject", ("name", "id", "env", "shell", "run")),
            ("Vendor locked dependency closure in fetch sandbox", ("name", "id", "env", "shell", "run")),
            ("Execute sandbox negative controls", ("name", "id", "env", "shell", "run")),
            ("Upload sandbox negative-control transcript", ("name", "if", "id", "uses", "with")),
            ("Execute candidate qualification in disposable networkless sandbox", ("name", "shell", "env", "run")),
            ("Verify candidate source immutability", ("name", "env", "shell", "run")),
            ("Verify dependency substrate immutability", ("name", "id", "env", "shell", "run")),
            ("Emit qualification receipt", ("name", "env", "shell", "run")),
            ("Upload qualification receipt", ("name", "if", "id", "uses", "with")),
            ("Verify retained qualification receipt", ("name", "if", "env", "shell", "run")),
        ),
        S1,
    )
    require_exact_step_conditionals(
        l,
        (
            ("Checkout trusted qualification root", None),
            ("Verify trusted pull-request-target invocation", None),
            ("Resolve exact candidate source", None),
            ("Static trust-surface audit", None),
            ("Snapshot exact candidate source identity", None),
            ("Snapshot locked dependency identity", None),
            ("Pull and preflight pinned sandbox image", None),
            ("Prepare locked dependency subject", None),
            ("Vendor locked dependency closure in fetch sandbox", None),
            ("Execute sandbox negative controls", None),
            ("Upload sandbox negative-control transcript", "success()"),
            ("Execute candidate qualification in disposable networkless sandbox", None),
            ("Verify candidate source immutability", None),
            ("Verify dependency substrate immutability", None),
            ("Emit qualification receipt", None),
            ("Upload qualification receipt", "success()"),
            ("Verify retained qualification receipt", "success()"),
        ),
        S1,
    )
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
    require_exact_top_level_keys(l, ("name", "on", "permissions", "concurrency", "env", "jobs"), S2)
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
    require_exact_job_keys(
        l,
        (("verify", ("name", "runs-on", "cache-mode", "timeout-minutes", "steps")),),
        S2,
    )
    if tuple(step_names(l)) != S2_STEPS:
        fail("S2 step topology mismatch")
    require_exact_actions(
        external_uses(l),
        (CHECKOUT, DOWNLOAD, DOWNLOAD),
        "S2 external actions",
    )
    for expected in (
        ("runs-on: ubuntu-24.04", 1),
        ("timeout-minutes: 15", 1),
        ("cache-mode: none", 1),
    ):
        if exact_count(l, expected[0]) != expected[1]:
            fail(f"S2 job value mismatch: {expected[0]!r}")
    require_exact_step_mapping(
        l,
        "Checkout exact verifier workflow commit",
        "with",
        (
            "ref: ${{ github.workflow_sha }}",
            "fetch-depth: 0",
            "persist-credentials: false",
        ),
        S2,
    )
    require_exact_step_mapping(
        l,
        "Download retained qualification receipt through official artifact client",
        "with",
        (
            "artifact-ids: ${{ steps.verify_result.outputs.artifact_id }}",
            "path: ${{ runner.temp }}/security-kernel-official-receipt",
            "github-token: ${{ github.token }}",
            "repository: ${{ github.repository }}",
            "run-id: ${{ steps.verify_result.outputs.trusted_dispatch_run_id }}",
            "digest-mismatch: error",
        ),
        S2,
    )
    require_exact_step_mapping(
        l,
        "Download retained sandbox negative-control transcript through official artifact client",
        "with",
        (
            "artifact-ids: ${{ steps.verify_negative_controls_log.outputs.negative_controls_log_artifact_id }}",
            "path: ${{ runner.temp }}/security-kernel-official-negative-controls",
            "github-token: ${{ github.token }}",
            "repository: ${{ github.repository }}",
            "run-id: ${{ steps.verify_result.outputs.trusted_dispatch_run_id }}",
            "digest-mismatch: error",
        ),
        S2,
    )
    require_exact_root_mapping(
        l,
        "env",
        (
            'BASE_REPOSITORY: "Luminous-Dynamics/mycelix"',
            'BASE_REPOSITORY_ID: "1176351975"',
            'BASE_BRANCH: "main"',
            'TRUSTED_DISPATCHER_WORKFLOW_PATH: ".github/workflows/security-kernel-trusted-dispatch.yml"',
            'TRUSTED_DISPATCHER_WORKFLOW_BLOB_SHA: "66e8adb99be391aad73e41d94c11c2434861f1c1"',
            'INDEPENDENT_WORKFLOW_PATH: ".github/workflows/security-kernel-independent-qualification.yml"',
            'TRUSTED_INDEPENDENT_WORKFLOW_BLOB_SHA: "156f00beeb50f269847a1d3cd9bbb50aa8ec5514"',
            'RETENTION_REFERENCE_VERIFIER_PATH: ".github/security-kernel/reference_verify_evidence_retention_binding.py"',
            'RETENTION_REFERENCE_VERIFIER_BLOB_SHA: "18dfd77cab186c0f467d81fcd6ce6d6d713795c2"',
            'EXECUTION_REFERENCE_VERIFIER_PATH: ".github/security-kernel/reference_verify_execution_binding.py"',
            'EXECUTION_REFERENCE_VERIFIER_BLOB_SHA: "18dca202667b1dc26f57c588efcab42087c8a49e"',
            'SOURCE_POLICY_VERIFIER_PATH: ".github/security-kernel/reference_verify_source_policy.py"',
            'SOURCE_POLICY_VERIFIER_BLOB_SHA: "9819e583b17fed8f35b425d3f0d1222ff8d86db6"',
            'EXPECTED_JOB_NAME: "Independent Security Kernel"',
        ),
        "S2",
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
    require_no_quoted_structural_keys(l, S2)
    require_no_yaml_reuse_syntax(l, S2)
    require_explicit_bash_for_run_steps(l, S2)
    require_no_duplicate_step_keys(l, S2)
    require_step_execution_modes(l, S2)
    require_exact_step_keys(
        l,
        (
            ("Checkout exact verifier workflow commit", ("name", "uses", "with")),
            ("Verify trusted dispatcher, reusable S1, and qualification gates", ("name", "id", "env", "shell", "run")),
            ("Verify retained negative-control evidence binding", ("name", "if", "id", "env", "shell", "run")),
            ("Download retained qualification receipt through official artifact client", ("name", "if", "uses", "with")),
            ("Download retained sandbox negative-control transcript through official artifact client", ("name", "if", "uses", "with")),
            ("Verify official receipt transport and publish verified result", ("name", "if", "env", "shell", "run")),
        ),
        S2,
    )
    require_exact_step_conditionals(
        l,
        (
            ("Checkout exact verifier workflow commit", None),
            ("Verify trusted dispatcher, reusable S1, and qualification gates", None),
            ("Verify retained negative-control evidence binding", "success()"),
            ("Download retained qualification receipt through official artifact client", "success()"),
            ("Download retained sandbox negative-control transcript through official artifact client", "success()"),
            ("Verify official receipt transport and publish verified result", "success()"),
        ),
        S2,
    )
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
        lambda: verify_s1(raw["s1"].replace(VENDOR_VOLUME_CREATE.encode(), b'docker volume create --driver local --opt type=tmpfs --opt device=tmpfs --opt o=rw,nosuid,nodev,noexec,size=1024m', 1), s1_sha),
        "vendor tmpfs without inode ceiling",
    )
    expect_rejection(
        lambda: verify_s1(raw["s1"].replace(VENDOR_VOLUME_INSPECT.encode(), b'vendor_volume_spec="wrong|volume|driver|options"', 1), s1_sha),
        "vendor volume instantiated-option mismatch",
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

    expect_rejection(
        lambda: verify_s0(
            raw["s0"].replace(
                b'  BASE_REPOSITORY_ID: "1176351975"\n',
                b'  BASE_REPOSITORY_ID: "1176351976"\n',
                1,
            ),
            s1_sha,
        ),
        "S0 root env repository ID drift",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b'  VENDOR_MAX_INODES: "150000"\n',
                b'  VENDOR_MAX_INODES: "149999"\n',
                1,
            ),
            s1_sha,
        ),
        "S1 root env vendor inode ceiling drift",
    )
    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(
                b'  EXPECTED_JOB_NAME: "Independent Security Kernel"\n',
                b'  EXPECTED_JOB_NAME: "Different Security Kernel"\n',
                1,
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 root env expected job-name drift",
    )

    def inject_unregistered_top_level_key(raw: bytes) -> bytes:
        marker = b"jobs:\n"
        if marker not in raw:
            fail("top-level-key regression fixture marker missing")
        return raw.replace(marker, b"defaults:\n  run:\n    shell: bash\n" + marker, 1)

    def inject_unapproved_conditional(raw: bytes, marker: bytes) -> bytes:
        if marker not in raw:
            fail(f"conditional regression fixture marker missing: {marker!r}")
        return raw.replace(marker, b"        if: false\n" + marker, 1)

    expect_rejection(
        lambda: verify_s0(inject_unregistered_top_level_key(raw["s0"]), s1_sha),
        "S0 unregistered top-level defaults",
    )
    expect_rejection(
        lambda: verify_s1(inject_unregistered_top_level_key(raw["s1"]), s1_sha),
        "S1 unregistered top-level defaults",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_unregistered_top_level_key(raw["s2"]),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 unregistered top-level defaults",
    )

    expect_rejection(
        lambda: verify_s0(
            inject_unapproved_conditional(
                raw["s0"],
                b"      - name: Verify trusted dispatcher context and exact PR identity\n",
            ),
            s1_sha,
        ),
        "S0 unapproved step conditional",
    )
    expect_rejection(
        lambda: verify_s1(
            inject_unapproved_conditional(
                raw["s1"],
                b"      - name: Execute candidate qualification in disposable networkless sandbox\n",
            ),
            s1_sha,
        ),
        "S1 unapproved step conditional",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_unapproved_conditional(
                raw["s2"],
                b"      - name: Checkout exact verifier workflow commit\n",
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 unapproved step conditional",
    )

    def inject_quoted_structural_key(raw: bytes) -> bytes:
        marker = b"jobs:\n"
        if marker not in raw:
            fail("quoted-key regression fixture marker missing")
        return raw.replace(marker, b'\"defaults\":\n  run:\n    shell: bash\n' + marker, 1)

    expect_rejection(
        lambda: verify_s0(inject_quoted_structural_key(raw["s0"]), s1_sha),
        "S0 quoted structural key",
    )
    expect_rejection(
        lambda: verify_s1(inject_quoted_structural_key(raw["s1"]), s1_sha),
        "S1 quoted structural key",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_quoted_structural_key(raw["s2"]),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 quoted structural key",
    )

    def inject_unapproved_step_property(raw: bytes, marker: bytes) -> bytes:
        if marker not in raw:
            fail(f"step-key regression fixture marker missing: {marker!r}")
        return raw.replace(marker, marker + b"        timeout-minutes: 1\n", 1)

    expect_rejection(
        lambda: verify_s0(
            inject_unapproved_step_property(
                raw["s0"],
                b"      - name: Verify trusted dispatcher context and exact PR identity\n",
            ),
            s1_sha,
        ),
        "S0 unapproved step property",
    )
    expect_rejection(
        lambda: verify_s1(
            inject_unapproved_step_property(
                raw["s1"],
                b"      - name: Execute candidate qualification in disposable networkless sandbox\n",
            ),
            s1_sha,
        ),
        "S1 unapproved step property",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_unapproved_step_property(
                raw["s2"],
                b"      - name: Checkout exact verifier workflow commit\n",
            ),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 unapproved step property",
    )

    def inject_unregistered_job_key(raw: bytes, marker: bytes) -> bytes:
        if marker not in raw:
            fail(f"job-key regression fixture marker missing: {marker!r}")
        return raw.replace(marker, marker + b"    if: false\n", 1)

    expect_rejection(
        lambda: verify_s0(
            inject_unregistered_job_key(raw["s0"], b"  resolve:\n"),
            s1_sha,
        ),
        "S0 unregistered job key",
    )
    expect_rejection(
        lambda: verify_s1(
            inject_unregistered_job_key(raw["s1"], b"  qualify:\n"),
            s1_sha,
        ),
        "S1 unregistered job key",
    )
    expect_rejection(
        lambda: verify_s2(
            inject_unregistered_job_key(raw["s2"], b"  verify:\n"),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 unregistered job key",
    )

    expect_rejection(
        lambda: verify_s0(
            raw["s0"].replace(
                b"candidate_sha: ${{ needs.resolve.outputs.candidate_sha }}",
                b"candidate_sha: ${{ needs.resolve.outputs.candidate_pr }}",
                1,
            ),
            s1_sha,
        ),
        "S0 mutated reusable-workflow candidate_sha input",
    )
    expect_rejection(
        lambda: verify_s1(
            raw["s1"].replace(
                b"ref: ${{ github.workflow_sha }}",
                b"ref: ${{ github.sha }}",
                1,
            ),
            s1_sha,
        ),
        "S1 mutated checkout ref",
    )
    expect_rejection(
        lambda: verify_s2(
            raw["s2"].replace(b"digest-mismatch: error", b"digest-mismatch: ignore", 1),
            s0_sha,
            s1_sha,
            retention_sha,
            execution_sha,
            files["policy"]["sha"],
        ),
        "S2 mutated artifact digest policy",
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
