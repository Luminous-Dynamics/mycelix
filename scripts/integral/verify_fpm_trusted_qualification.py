#!/usr/bin/env python3
"""Independent raw-object causal-join verifier for FPM trusted qualification."""

from __future__ import annotations

import base64
import datetime as dt
import hashlib
import json
import posixpath
import re
import shlex
import stat
import sys
import tomllib
import zipfile
from pathlib import Path
from typing import Any

BASE_REPOSITORY = "Luminous-Dynamics/mycelix"
BASE_REPOSITORY_ID = 1176351975
BASE_BRANCH = "main"

CANDIDATE_WORKFLOW_ID = 377461322
CANDIDATE_WORKFLOW_PATH = ".github/workflows/fpm-wasm-artifact-identity.yml"
TRUSTED_WORKFLOW_NAME = "FPM trusted qualification policy"
TRUSTED_WORKFLOW_PATH = ".github/workflows/fpm-trusted-qualification.yml"
INDEPENDENT_WORKFLOW_PATH = ".github/workflows/fpm-trusted-qualification-independent-verify.yml"

MANIFEST_PATH = "crates/fpm-wasm-artifact-identity/Cargo.toml"
MANIFEST_BLOB_SHA = "c94b53f61ed8a9bfb6249b1b339550dddd074d6c"

RUSTC_VERSION = "rustc 1.96.1"
RUSTC_COMMIT = "31fca3adb283cc9dfd56b49cdee9a96eb9c96ffd"

MAX_ARTIFACT_ARCHIVE_BYTES = 8 * 1024 * 1024
MAX_ARTIFACT_ARCHIVE_MEMBERS = 8
MAX_ARTIFACT_MEMBER_BYTES = 2 * 1024 * 1024
ALLOWED_ARTIFACT_COMPRESSION = frozenset({zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED})
CRATES_IO_REGISTRY_SOURCE = "registry+https://github.com/rust-lang/crates.io-index"
SANDBOX_SYSTEM_CLOSURE_PROFILE = "fpm-debian12-bookworm-gcc13.4-amd64-rust-1.96.1-v1"
SANDBOX_SYSTEM_CLOSURE_COMMANDS = (
    "bash", "env", "grep", "tr", "timeout", "cargo", "rustc", "rustfmt",
    "cc", "ld", "as", "ldd", "realpath", "sha256sum", "sed", "uname", "cat",
    "find", "sort", "mkdir", "chmod", "rm", "ln", "readelf",
)
LOCK_ROOT_PACKAGE = "fpm-wasm-artifact-identity"

RECEIPT_KEYS = frozenset(
    {
        "schema",
        "qualification",
        "repository",
        "repository_id",
        "pr_number",
        "subject_sha",
        "subject_tree_sha",
        "observed_postflight_head_sha",
        "observed_postflight_tree_sha",
        "base_sha",
        "trusted_policy_sha",
        "trusted_policy_blob_sha",
        "trusted_policy_ref",
        "trusted_workflow_run_id",
        "trusted_workflow_run_attempt",
        "upstream_workflow_run_id",
        "upstream_workflow_run_attempt",
        "upstream_workflow_id",
        "upstream_workflow_path",
        "upstream_workflow_conclusion",
        "manifest_blob_sha",
        "lock_mode",
        "lock_sha256",
        "rustc_version",
        "rustc_commit",
        "cargo_version",
        "candidate_uid",
        "candidate_gid",
        "candidate_execution_profile",
        "sandbox_image_digest",
        "sandbox_probe",
        "sandbox_system_closure",
        "sandbox_system_closure_sha256",
        "sandbox_target_closure",
        "sandbox_target_closure_sha256",
        "dependency_cache_sha256",
        "dependency_source_policy",
        "steps",
        "execution_pass",
        "procedure_trust",
        "promotion_authority",
    }
)

STEP_KEYS = frozenset(
    {
        "preflight",
        "checkout",
        "source",
        "toolchain",
        "lock",
        "dependencies",
        "sandbox_image",
        "fmt",
        "sandbox_probe",
        "compile",
        "tests",
        "postflight",
    }
)

INDEX_KEYS = frozenset(
    {
        "schema",
        "receipt_sha256",
        "artifact",
        "subject_sha",
        "subject_tree_sha",
        "trusted_policy_sha",
        "trusted_policy_blob_sha",
        "trusted_workflow_run_id",
    }
)

ENUMERATION_KEYS = frozenset(
    {
        "schema",
        "page_size",
        "max_pages",
        "max_artifacts",
        "total_count_reported",
        "enumerated_count",
        "page_counts",
        "terminal_page",
        "artifact_identity_sha256",
        "complete",
        "repeat_enumeration_verified",
        "repeat_total_count_reported",
        "repeat_page_counts",
        "repeat_artifact_identity_sha256",
    }
)

CONTROL_KEYS = frozenset(
    {
        "repository",
        "repository_id",
        "path",
        "ref",
        "workflow_ref",
        "workflow_sha",
        "workflow_blob_sha",
        "reference_verifier_path",
        "reference_verifier_blob_sha",
        "artifact_collector_path",
        "artifact_collector_blob_sha",
    }
)

INDEX_ARTIFACT_KEYS = frozenset(
    {
        "id",
        "sha256_hex",
        "url",
        "retention_days",
        "immutable_after_upload",
        "deletion_by_repository_writer_possible",
    }
)


def fail(message: str) -> None:
    raise SystemExit(f"FPM_REFERENCE_FAIL: {message}")


def need(value: Any, description: str) -> Any:
    if value is None:
        fail(f"missing {description}")
    return value


def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            fail(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def reject_nonstandard_constant(value: str) -> Any:
    fail(f"non-standard JSON constant is forbidden: {value}")


def load_canonical_json(path: Path) -> dict[str, Any]:
    raw = path.read_bytes()
    if not raw.endswith(b"\n"):
        fail(f"{path} must end in exactly one LF")
    canonical_bytes = raw[:-1]
    if canonical_bytes.endswith(b"\n"):
        fail(f"{path} has more than one trailing LF")
    try:
        text = canonical_bytes.decode("utf-8")
        value = json.loads(
            text,
            object_pairs_hook=reject_duplicate_keys,
            parse_constant=reject_nonstandard_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail(f"{path} is not valid UTF-8 JSON: {exc}")
    if not isinstance(value, dict):
        fail(f"{path} top level must be an object")
    reserialized = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    if reserialized != canonical_bytes:
        fail(f"{path} is not canonical JSON")
    return value


def load_strict_json(path: Path) -> dict[str, Any]:
    """Load snapshot JSON while rejecting duplicate keys and nonstandard constants."""
    try:
        value = json.loads(
            path.read_bytes().decode("utf-8"),
            object_pairs_hook=reject_duplicate_keys,
            parse_constant=reject_nonstandard_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail(f"{path} is not valid strict UTF-8 JSON: {exc}")
    if not isinstance(value, dict):
        fail(f"{path} top level must be an object")
    return value


def require_hex(value: Any, length: int, field: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(rf"[0-9a-f]{{{length}}}", value):
        fail(f"{field} is not canonical lowercase hex{length}")
    return value

def require_json_int(value: Any, field: str, minimum: int = 0) -> int:
    """Require a genuine JSON integer; reject bool and floating-point lookalikes."""
    if type(value) is not int:
        fail(f"{field} must be a JSON integer")
    if value < minimum:
        fail(f"{field} is below its minimum {minimum}")
    return value


def require_positive_decimal_string(value: Any, field: str, max_digits: int = 20) -> int:
    """Parse positive decimal IDs only when their wire representation is canonical."""
    pattern = rf"[1-9][0-9]{{0,{max_digits - 1}}}"
    if not isinstance(value, str) or not re.fullmatch(pattern, value):
        fail(f"{field} must be a canonical positive decimal string")
    return int(value)


def require_sha256(value: Any, field: str) -> str:
    return require_hex(value, 64, field)


def require_sha256_prefixed(value: Any, field: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", value):
        fail(f"{field} is not sha256:<64 lowercase hex>")
    return value


def verify_raw_artifact_archive(
    archive: Path,
    expected_member: str,
    expected_digest: str,
    expected_size_bytes: int,
    extracted: Path,
    label: str,
) -> dict[str, Any]:
    if not archive.is_file() or archive.is_symlink():
        fail(f"{label} artifact archive is not a regular file")
    archive_size = archive.stat().st_size
    if archive_size <= 0 or archive_size > MAX_ARTIFACT_ARCHIVE_BYTES:
        fail(f"{label} artifact archive size exceeds the closed-world bound")
    if archive_size != expected_size_bytes:
        fail(f"{label} artifact archive size does not match API metadata")
    if not re.fullmatch(r"sha256:[0-9a-f]{64}", expected_digest):
        fail(f"{label} artifact digest is malformed")
    archive_bytes = archive.read_bytes()
    archive_sha256 = hashlib.sha256(archive_bytes).hexdigest()
    if f"sha256:{archive_sha256}" != expected_digest:
        fail(f"{label} artifact archive digest mismatch")
    if extracted.exists() and (not extracted.is_file() or extracted.is_symlink()):
        fail(f"{label} extracted evidence is not a regular file")

    try:
        with zipfile.ZipFile(archive) as bundle:
            infos = bundle.infolist()
            if not infos:
                fail(f"{label} artifact archive is empty")
            if len(infos) > MAX_ARTIFACT_ARCHIVE_MEMBERS:
                fail(f"{label} artifact archive has too many members")

            names = [info.filename for info in infos]
            if len(names) != len(set(names)):
                fail(f"{label} artifact archive contains duplicate member names")
            if names != [expected_member]:
                fail(
                    f"{label} artifact archive member set mismatch: "
                    f"expected={[expected_member]!r} actual={names!r}"
                )

            info = infos[0]
            if (
                not expected_member
                or "\x00" in info.filename
                or "\\" in info.filename
                or info.filename.startswith("/")
            ):
                fail(f"{label} artifact archive member path is unsafe")
            parts = info.filename.split("/")
            if any(part in {"", ".", ".."} for part in parts):
                fail(f"{label} artifact archive member path is unsafe")
            if len(info.filename.encode("utf-8")) > 512:
                fail(f"{label} artifact archive member name is too long")
            if info.is_dir():
                fail(f"{label} artifact archive member is a directory")
            if info.flag_bits & 0x1:
                fail(f"{label} artifact archive member is encrypted")
            if info.compress_type not in ALLOWED_ARTIFACT_COMPRESSION:
                fail(f"{label} artifact archive uses an unsupported compression method")
            if info.file_size > MAX_ARTIFACT_MEMBER_BYTES:
                fail(f"{label} artifact archive member is too large")
            if info.compress_size > MAX_ARTIFACT_ARCHIVE_BYTES:
                fail(f"{label} artifact archive compressed member is too large")

            unix_mode = (info.external_attr >> 16) & 0xFFFF
            file_type = stat.S_IFMT(unix_mode)
            if file_type not in {0, stat.S_IFREG}:
                fail(f"{label} artifact archive member is not a regular file")
            if info.create_system == 0 and info.external_attr & 0x10:
                fail(f"{label} artifact archive member has directory attributes")

            try:
                with bundle.open(info, "r") as source:
                    member = source.read(MAX_ARTIFACT_MEMBER_BYTES + 1)
            except (OSError, RuntimeError, ValueError, zipfile.BadZipFile) as exc:
                fail(f"{label} artifact archive member could not be read: {exc}")
            if len(member) > MAX_ARTIFACT_MEMBER_BYTES:
                fail(f"{label} artifact archive member exceeds size bound")
            member_sha256 = hashlib.sha256(member).hexdigest()

            if extracted.exists():
                extracted_bytes = extracted.read_bytes()
                if extracted_bytes != member:
                    fail(f"{label} extracted evidence does not match raw archive member")
            else:
                extracted.parent.mkdir(parents=True, exist_ok=True)
                extracted.write_bytes(member)
                extracted.chmod(0o400)

            member_set_sha256 = hashlib.sha256(
                json.dumps([info.filename], separators=(",", ":"), ensure_ascii=True).encode("utf-8")
            ).hexdigest()
            return {
                "archive_sha256": archive_sha256,
                "archive_size_bytes": archive_size,
                "member_count": len(names),
                "member_names": names,
                "member_name": info.filename,
                "member_set_sha256": member_set_sha256,
                "member_size_bytes": len(member),
                "member_sha256": member_sha256,
                "compression_method": info.compress_type,
            }
    except zipfile.BadZipFile as exc:
        fail(f"{label} artifact archive is not a valid ZIP: {exc}")

def validate_artifact_lifetime(item: dict[str, Any], label: str) -> tuple[str, str]:
    created = item.get("created_at")
    expires = item.get("expires_at")
    if not isinstance(created, str) or not isinstance(expires, str):
        fail(f"{label} artifact timestamps are missing")
    try:
        created_dt = dt.datetime.fromisoformat(created.replace("Z", "+00:00"))
        expires_dt = dt.datetime.fromisoformat(expires.replace("Z", "+00:00"))
    except ValueError as exc:
        fail(f"{label} artifact timestamp is invalid: {exc}")
    if created_dt.tzinfo is None or expires_dt.tzinfo is None:
        fail(f"{label} artifact timestamps must be timezone-aware")
    if expires_dt <= created_dt:
        fail(f"{label} artifact expires_at is not after created_at")
    return created, expires


def verify_tracked_lock_source_policy(lock_bytes: bytes) -> None:
    try:
        lock = tomllib.loads(lock_bytes.decode("utf-8"))
    except (UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
        fail(f"tracked Cargo.lock is not valid TOML: {exc}")
    if not isinstance(lock, dict):
        fail("tracked Cargo.lock top level is not a table")

    packages = lock.get("package")
    if not isinstance(packages, list) or not packages:
        fail("tracked Cargo.lock does not contain package entries")

    if "patch" in lock or "replace" in lock:
        fail("tracked Cargo.lock contains unsupported patch/replace policy")

    source_free: list[str] = []
    for package in packages:
        if not isinstance(package, dict):
            fail("tracked Cargo.lock package entry is malformed")
        name = package.get("name")
        version = package.get("version")
        source = package.get("source")
        if not isinstance(name, str) or not name:
            fail("tracked Cargo.lock package name is malformed")
        if not isinstance(version, str) or not version:
            fail(f"tracked Cargo.lock package version is malformed: {name}")
        if source is None:
            source_free.append(name)
            continue
        if source != CRATES_IO_REGISTRY_SOURCE:
            fail(f"tracked Cargo.lock package uses unapproved source: {name}")
        checksum = package.get("checksum")
        if not isinstance(checksum, str) or not re.fullmatch(r"[0-9a-f]{64}", checksum):
            fail(f"tracked Cargo.lock registry package lacks canonical checksum: {name}")

    if source_free != [LOCK_ROOT_PACKAGE]:
        fail(
            "tracked Cargo.lock source-free package set is not exactly the workspace root: "
            f"{source_free!r}"
        )


def verify_sandbox_invocations(policy_text: str) -> None:
    lines = policy_text.splitlines()
    invocations: list[tuple[str, str]] = []
    for index, line in enumerate(lines):
        if "docker run --rm" not in line:
            continue
        step = "unknown"
        for prior in reversed(lines[: index + 1]):
            if prior.startswith("      - name: "):
                step = prior.removeprefix("      - name: ").strip()
                break

        command_parts = [line.rstrip().rstrip("\\").strip()]
        cursor = index + 1
        while cursor < len(lines):
            part = lines[cursor].rstrip()
            command_parts.append(part.rstrip("\\").strip())
            if '"${SANDBOX_IMAGE}"' in part:
                break
            cursor += 1
        else:
            fail(f"sandbox docker invocation in {step} has no pinned image terminator")
        invocations.append((step, " ".join(command_parts)))

    common = [
        "docker", "run", "--rm", "--pull=never", "--network", "none",
        "--read-only", "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
    ]
    candidate_mount = "type=bind,src=${GITHUB_WORKSPACE}/candidate,dst=/candidate,readonly"
    toolchain_mount = "type=bind,src=${FPM_TOOLCHAIN_ROOT},dst=/opt/fpm-rust,readonly"
    cargo_mount = "type=bind,src=${FPM_CARGO_HOME},dst=/cargo-ro,readonly"
    target_mount = "type=bind,src=${FPM_TARGET_DIR},dst=/target"
    target_readonly_mount = "type=bind,src=${FPM_TARGET_DIR},dst=/target,readonly"
    closure_mount = "type=bind,src=${TARGET_CLOSURE_FILE},dst=/tmp/fpm-target-closure.tsv,readonly"
    user_and_workdir = ["--user", "${CANDIDATE_UID}:${CANDIDATE_GID}", "--workdir", "/candidate"]

    expected_argv_by_step = {
        "Cargo fmt inside immutable sandbox": common + [
            "--pids-limit", "256", "--memory", "2g", "--memory-swap", "2g", "--cpus", "1",
            "--tmpfs", "/tmp:rw,nosuid,nodev,noexec,size=128m",
            "--mount", candidate_mount, "--mount", toolchain_mount,
        ] + user_and_workdir,
        "Probe hostile-code sandbox boundary": common + [
            "--pids-limit", "256", "--memory", "6g", "--memory-swap", "6g", "--cpus", "2",
            "--tmpfs", "/tmp:rw,nosuid,nodev,noexec,size=512m",
            "--mount", candidate_mount, "--mount", toolchain_mount, "--mount", cargo_mount, "--mount", target_mount,
        ] + user_and_workdir,
        "Compile test artifacts inside immutable offline sandbox": common + [
            "--pids-limit", "512", "--memory", "6g", "--memory-swap", "6g", "--cpus", "2",
            "--tmpfs", "/tmp:rw,nosuid,nodev,noexec,size=512m",
            "--mount", candidate_mount, "--mount", toolchain_mount, "--mount", cargo_mount, "--mount", target_mount,
        ] + user_and_workdir,
        "Execute precompiled test harness inside immutable sandbox": common + [
            "--pids-limit", "512", "--memory", "6g", "--memory-swap", "6g", "--cpus", "2",
            "--tmpfs", "/tmp:rw,nosuid,nodev,noexec,size=512m",
            "--mount", candidate_mount, "--mount", toolchain_mount, "--mount", target_readonly_mount, "--mount", closure_mount,
        ] + user_and_workdir,
    }
    actual_steps = {step for step, _ in invocations}
    if len(invocations) != len(expected_argv_by_step) or actual_steps != set(expected_argv_by_step):
        fail(f"unexpected sandbox invocation set: {sorted(actual_steps)!r}")

    for step, command in invocations:
        prefix, marker, _ = command.partition('"${SANDBOX_IMAGE}"')
        if not marker:
            fail(f"sandbox docker invocation in {step} does not terminate at the pinned image")
        try:
            actual_argv = shlex.split(prefix, posix=True)
        except ValueError as exc:
            fail(f"sandbox Docker argv in {step} is not shell-parseable: {exc}")
        if actual_argv != expected_argv_by_step[step]:
            fail(
                f"{step} sandbox Docker argv is not the exact allowlisted profile: "
                f"actual={actual_argv!r}"
            )


def verify_sandbox_programs(policy_text: str) -> None:
    lines = policy_text.splitlines()
    starts = [
        index for index, line in enumerate(lines)
        if "-ceu \"$(cat <<" in line and "FPM_SANDBOX_SCRIPT" in line
    ]
    if len(starts) != 4:
        fail(f"expected four embedded sandbox programs, found {len(starts)}")

    expected_steps = [
        "Cargo fmt inside immutable sandbox",
        "Probe hostile-code sandbox boundary",
        "Compile test artifacts inside immutable offline sandbox",
        "Execute precompiled test harness inside immutable sandbox",
    ]
    shared_required = (
        "umask 077",
        "export HOME=/tmp/home",
        "export PATH=/opt/fpm-rust/bin:/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin",
        "export TMPDIR=/tmp",
        "export CARGO_REGISTRIES_CRATES_IO_PROTOCOL=sparse",
        "export RUSTC_WRAPPER=",
        "test ! -e /var/run/docker.sock",
        "test ! -d /github",
        "test ! -d /home/runner",
        "test ! -L /.cargo",
        "test ! -e /.cargo/config",
        "test ! -e /.cargo/config.toml",
        "test -z \"$(env | grep '^GITHUB_' || true)\"",
    )

    for index, start in enumerate(starts):
        step = next(
            (
                line.removeprefix("      - name: ").strip()
                for line in reversed(lines[:start])
                if line.startswith("      - name: ")
            ),
            "unknown",
        )
        if step != expected_steps[index]:
            fail(f"embedded sandbox program order mismatch at index {index}: {step!r}")
        try:
            end = next(
                i for i in range(start + 1, len(lines))
                if lines[i] == "          FPM_SANDBOX_SCRIPT"
            )
        except StopIteration:
            fail(f"embedded sandbox program in {step} has no terminator")
        body = "\n".join(lines[start + 1:end])
        if not body.strip():
            fail(f"embedded sandbox program in {step} is empty")
        for token in shared_required:
            if token not in body:
                fail(f"embedded sandbox program in {step} is missing invariant: {token}")
        if "export CARGO_NET_OFFLINE=false" in body:
            fail(f"embedded sandbox program in {step} enables Cargo network access")
        if index < 3 and body.count("export CARGO_NET_OFFLINE=true") != 1:
            fail(f"embedded sandbox program in {step} must force Cargo offline mode exactly once")

        phase_required = {
            "Cargo fmt inside immutable sandbox": (
                "cargo fmt --check --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml",
            ),
            "Probe hostile-code sandbox boundary": (
                "rustc --crate-name fpm_toolchain_probe --edition 2024 -C linker=cc",
                "/target/.fpm-toolchain-probe",
            ),
            "Compile test artifacts inside immutable offline sandbox": (
                "cargo test --locked --offline --no-run --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml",
                "find /target/debug/deps",
            ),
            "Execute precompiled test harness inside immutable sandbox": (
                "/tmp/fpm-target-closure.tsv",
                "/target/debug/deps/",
            ),
        }[step]
        for token in phase_required:
            if token not in body:
                fail(f"embedded sandbox program in {step} is missing phase command: {token}")



def verify_sandbox_policy(policy_file: dict[str, Any], expected_image_digest: str) -> None:
    if policy_file.get("encoding") != "base64":
        fail("trusted policy file is not represented as base64 content")
    encoded = policy_file.get("content")
    if not isinstance(encoded, str) or not encoded:
        fail("trusted policy file content is missing")
    try:
        raw = base64.b64decode("".join(encoded.split()), validate=True)
    except (ValueError, base64.binascii.Error) as exc:
        fail(f"trusted policy file base64 is invalid: {exc}")
    actual_blob_sha = hashlib.sha1(
        f"blob {len(raw)}\0".encode("ascii") + raw
    ).hexdigest()
    if policy_file.get("sha") != actual_blob_sha:
        fail("trusted policy content does not match its Git blob SHA")
    policy_text = raw.decode("utf-8")
    required = (
        f"FPM_SANDBOX_IMAGE: gcc@{expected_image_digest}",
        "--network none",
        "--read-only",
        "--cap-drop ALL",
        "--security-opt no-new-privileges",
        "--pids-limit 512",
        "--memory 6g",
        "--memory-swap 6g",
        "--cpus 2",
        "--mount type=bind,src=\"${GITHUB_WORKSPACE}/candidate\",dst=/candidate,readonly",
        "--mount type=bind,src=\"${FPM_TOOLCHAIN_ROOT}\",dst=/opt/fpm-rust,readonly",
        "--mount type=bind,src=\"${FPM_CARGO_HOME}\",dst=/cargo-ro,readonly",
        "--mount type=bind,src=\"${FPM_TARGET_DIR}\",dst=/target",
        "--user \"${CANDIDATE_UID}:${CANDIDATE_GID}\"",
        "CARGO_NET_OFFLINE=true",
        "export PATH=/opt/fpm-rust/bin:/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin",
        "test ! -L /.cargo",
        "test ! -e /.cargo/config",
        "test ! -e /.cargo/config.toml",
        "cargo test --locked --offline --no-run --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml",
        "cargo fmt --check --manifest-path crates/fpm-wasm-artifact-identity/Cargo.toml",
        "rustc --crate-name fpm_toolchain_probe --edition 2024 -C linker=cc",
        "/target/.fpm-toolchain-probe",
    )
    for token in required:
        if token not in policy_text:
            fail(f"trusted sandbox policy is missing required invariant: {token}")
    forbidden = (
        "--privileged",
        "--pid=host",
        "--network host",
        "--cap-add",
        "docker.sock",
        'sudo -n -u fpm-untrusted env -i             HOME="/home/fpm-untrusted"',
    )
    verify_sandbox_invocations(policy_text)
    verify_sandbox_programs(policy_text)

    for token in forbidden:
        if token in policy_text:
            fail(f"trusted sandbox policy contains forbidden broadening: {token}")

    return policy_text

def require_canonical_absolute_path(value: Any, field: str) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith('/')
        or value.startswith('//')
        or not re.fullmatch(r"/[A-Za-z0-9._/+:-]+", value)
    ):
        fail(f'{field} is not a canonical absolute path')
    if posixpath.normpath(value) != value:
        fail(f'{field} contains a non-canonical path component')
    return value

def verify_sandbox_system_closure(closure: Any) -> None:
    if not isinstance(closure, dict):
        fail("sandbox_system_closure must be an object")
    expected_keys = {"profile", "image_digest", "architecture", "os_release_sha256", "libc_version", "commands", "libraries"}
    if set(closure) != expected_keys:
        fail("sandbox_system_closure schema mismatch")
    if closure["profile"] != SANDBOX_SYSTEM_CLOSURE_PROFILE:
        fail("unexpected sandbox system closure profile")
    if closure["image_digest"] != "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5":
        fail("sandbox system closure image digest mismatch")
    if closure["architecture"] != "linux/amd64":
        fail("sandbox system closure architecture mismatch")
    if not isinstance(closure["os_release_sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", closure["os_release_sha256"]):
        fail("sandbox system closure os-release digest is malformed")
    if not isinstance(closure["libc_version"], str) or not closure["libc_version"] or "\n" in closure["libc_version"] or "\t" in closure["libc_version"]:
        fail("sandbox system closure libc version is malformed")

    commands = closure["commands"]
    expected_command_names = sorted(SANDBOX_SYSTEM_CLOSURE_COMMANDS)
    if not isinstance(commands, list) or [item.get("name") for item in commands if isinstance(item, dict)] != expected_command_names:
        fail("sandbox system closure command set/order mismatch")
    seen_paths: set[str] = set()
    for item in commands:
        if not isinstance(item, dict) or set(item) != {"name", "path", "sha256"}:
            fail("sandbox system closure command entry schema mismatch")
        name, path, digest = item["name"], item["path"], item["sha256"]
        if not isinstance(name, str) or name not in SANDBOX_SYSTEM_CLOSURE_COMMANDS:
            fail("sandbox system closure command name is invalid")
        path = require_canonical_absolute_path(path, "sandbox system closure command path")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            fail(f"sandbox system closure command digest is malformed: {name}")
        if path in seen_paths:
            fail(f"sandbox system closure reuses executable path: {path}")
        seen_paths.add(path)
        if name in {"cargo", "rustc", "rustfmt"}:
            if not path.startswith("/opt/fpm-rust/"):
                fail(f"sandbox Rust tool path escaped immutable toolchain: {name}")
        elif not path.startswith(("/usr/", "/bin/", "/sbin/")):
            fail(f"sandbox system command escaped immutable image roots: {name}")

    libraries = closure["libraries"]
    if not isinstance(libraries, list) or not libraries or len(libraries) > 256:
        fail("sandbox system closure library set is invalid")
    library_paths: set[str] = set()
    library_path_order: list[str] = []
    for item in libraries:
        if not isinstance(item, dict) or set(item) != {"path", "sha256"}:
            fail("sandbox system closure library entry schema mismatch")
        path, digest = item["path"], item["sha256"]
        path = require_canonical_absolute_path(path, "sandbox system closure library path")
        if not path.startswith(("/lib/", "/lib64/", "/usr/lib/", "/usr/lib64/", "/usr/local/lib/", "/usr/local/lib64/", "/opt/fpm-rust/")):
            fail(f"sandbox system closure library escaped immutable roots: {path}")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            fail(f"sandbox system closure library digest is malformed: {path}")
        if path in library_paths:
            fail(f"sandbox system closure duplicates library path: {path}")
        library_paths.add(path)
        library_path_order.append(path)
    if library_path_order != sorted(library_path_order):
        fail("sandbox system closure libraries are not in canonical path order")
    if not any(path.endswith("/libc.so.6") for path in library_paths):
        fail("sandbox system closure does not record glibc libc.so.6")
    if not any("/ld-linux-" in path and path.endswith(".so.2") for path in library_paths):
        fail("sandbox system closure does not record the amd64 dynamic loader")

def verify_sandbox_target_closure(closure: Any) -> None:
    if not isinstance(closure, dict):
        fail("sandbox_target_closure must be an object")
    if set(closure) != {"executables", "libraries"}:
        fail("sandbox_target_closure schema mismatch")
    executables = closure["executables"]
    libraries = closure["libraries"]
    if not isinstance(executables, list) or not executables or len(executables) > 256:
        fail("sandbox_target_closure executable set is invalid")
    if not isinstance(libraries, list) or len(libraries) > 512:
        fail("sandbox_target_closure library set is invalid")
    seen_exec: set[str] = set()
    executable_path_order: list[str] = []
    for item in executables:
        if not isinstance(item, dict) or set(item) != {"path", "sha256"}:
            fail("sandbox_target_closure executable schema mismatch")
        path, digest = item["path"], item["sha256"]
        path = require_canonical_absolute_path(path, "sandbox_target_closure executable path")
        if not path.startswith("/target/debug/deps/"):
            fail("sandbox_target_closure executable path is outside target/debug/deps")
        if path in seen_exec:
            fail(f"sandbox_target_closure duplicate executable path: {path}")
        seen_exec.add(path)
        executable_path_order.append(path)
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            fail(f"sandbox_target_closure executable digest is malformed: {path}")
    if executable_path_order != sorted(executable_path_order):
        fail("sandbox_target_closure executables are not in canonical path order")
    seen_lib: set[str] = set()
    library_path_order: list[str] = []
    for item in libraries:
        if not isinstance(item, dict) or set(item) != {"path", "sha256"}:
            fail("sandbox_target_closure library schema mismatch")
        path, digest = item["path"], item["sha256"]
        path = require_canonical_absolute_path(path, "sandbox_target_closure library path")
        if not path.startswith(("/lib/", "/lib64/", "/usr/lib/", "/usr/lib64/", "/usr/local/lib/", "/usr/local/lib64/", "/opt/fpm-rust/")):
            fail(f"sandbox_target_closure library path escaped immutable roots: {path}")
        if path in seen_lib:
            fail(f"sandbox_target_closure duplicate library path: {path}")
        seen_lib.add(path)
        library_path_order.append(path)
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            fail(f"sandbox_target_closure library digest is malformed: {path}")
    if library_path_order != sorted(library_path_order):
        fail("sandbox_target_closure libraries are not in canonical path order")
    if not any(path.endswith("/libc.so.6") for path in seen_lib):
        fail("sandbox_target_closure does not record glibc libc.so.6")
    if not any("/ld-linux-" in path and path.endswith(".so.2") for path in seen_lib):
        fail("sandbox_target_closure does not record amd64 dynamic loader")

def verify_receipt(
    receipt: dict[str, Any],
    expected_trusted_run_id: int,
    expected_trusted_run_attempt: int,
    candidate_run: dict[str, Any],
    trusted_run: dict[str, Any],
    pr: dict[str, Any],
    commit: dict[str, Any],
    manifest: dict[str, Any],
    policy_file: dict[str, Any],
    verifier_control: dict[str, Any],
    candidate_lock: dict[str, Any],
    main_ref: dict[str, Any],
) -> tuple[str, str, str]:
    if set(receipt) != RECEIPT_KEYS:
        fail(
            "receipt closed-world mismatch: "
            f"missing={sorted(RECEIPT_KEYS - set(receipt))!r} "
            f"extra={sorted(set(receipt) - RECEIPT_KEYS)!r}"
        )

    if receipt["schema"] != "mycelix.fpm.trusted-qualification-receipt.v1":
        fail("unexpected receipt schema")
    if receipt["qualification"] != "FPM-WASM-ARTIFACT-IDENTITY-V1":
        fail("unexpected qualification name")
    if receipt["repository"] != BASE_REPOSITORY:
        fail("receipt repository mismatch")
    if require_json_int(receipt["repository_id"], "receipt.repository_id", minimum=1) != BASE_REPOSITORY_ID:
        fail("receipt repository_id mismatch")

    pr_number = require_positive_decimal_string(receipt["pr_number"], "receipt.pr_number", max_digits=10)
    if pr_number != require_json_int(pr.get("number"), "PR number", minimum=1):
        fail("receipt PR number mismatch")

    subject_sha = require_hex(receipt["subject_sha"], 40, "subject_sha")
    subject_tree = require_hex(receipt["subject_tree_sha"], 40, "subject_tree_sha")
    if receipt["observed_postflight_head_sha"] != subject_sha:
        fail("postflight head differs from subject")
    if receipt["observed_postflight_tree_sha"] != subject_tree:
        fail("postflight tree differs from subject")

    require_hex(receipt["base_sha"], 40, "base_sha")
    if receipt["base_sha"] != pr["base"]["sha"]:
        fail("receipt base SHA mismatch")
    if main_ref.get("ref") != "refs/heads/main":
        fail("live main ref does not name refs/heads/main")
    main_sha = require_hex(main_ref.get("object", {}).get("sha"), 40, "live main ref SHA")

    policy_sha = require_hex(receipt["trusted_policy_sha"], 40, "trusted_policy_sha")
    if policy_sha != trusted_run.get("head_sha"):
        fail("receipt trusted policy SHA does not equal trusted workflow run head SHA")
    if trusted_run.get("head_branch") != "main":
        fail("trusted workflow run is not on the default branch")
    if require_json_int(trusted_run.get("repository", {}).get("id"), "trusted workflow repository ID", minimum=1) != BASE_REPOSITORY_ID:
        fail("trusted workflow run repository ID mismatch")
    if receipt["trusted_policy_ref"] != "refs/heads/main":
        fail("receipt trusted policy ref mismatch")
    if receipt["trusted_policy_blob_sha"] != policy_file.get("sha"):
        fail("receipt trusted policy blob mismatch")
    if policy_file.get("path") != TRUSTED_WORKFLOW_PATH:
        fail("policy file path mismatch")
    policy_text = verify_sandbox_policy(
        policy_file,
        "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5",
    )

    trusted_run_id = require_json_int(receipt["trusted_workflow_run_id"], "receipt.trusted_workflow_run_id", minimum=1)
    trusted_run_attempt = require_json_int(receipt["trusted_workflow_run_attempt"], "receipt.trusted_workflow_run_attempt", minimum=1)
    if trusted_run_id != require_json_int(expected_trusted_run_id, "expected trusted workflow run ID", minimum=1):
        fail("trusted workflow run ID mismatch")
    if trusted_run_attempt != require_json_int(expected_trusted_run_attempt, "expected trusted workflow run attempt", minimum=1):
        fail("trusted workflow run attempt mismatch")

    upstream_run_id = require_positive_decimal_string(
        receipt["upstream_workflow_run_id"], "receipt.upstream_workflow_run_id"
    )
    upstream_run_attempt = require_positive_decimal_string(
        receipt["upstream_workflow_run_attempt"], "receipt.upstream_workflow_run_attempt", max_digits=6
    )
    upstream_workflow_id = require_json_int(
        receipt["upstream_workflow_id"], "receipt.upstream_workflow_id", minimum=1
    )
    if upstream_run_id != require_json_int(candidate_run.get("id"), "candidate workflow run ID", minimum=1):
        fail("candidate trigger run ID mismatch")
    if upstream_run_attempt != require_json_int(candidate_run.get("run_attempt"), "candidate workflow run attempt", minimum=1):
        fail("candidate trigger run attempt mismatch")
    if upstream_workflow_id != CANDIDATE_WORKFLOW_ID:
        fail("candidate workflow ID mismatch")
    if receipt["upstream_workflow_path"] != CANDIDATE_WORKFLOW_PATH:
        fail("candidate workflow path mismatch")
    if receipt["upstream_workflow_conclusion"] != candidate_run.get("conclusion", ""):
        fail("candidate workflow conclusion mismatch")

    if candidate_run["id"] != upstream_run_id:
        fail("candidate trigger run object mismatch")
    if require_json_int(candidate_run.get("workflow_id"), "candidate workflow ID", minimum=1) != CANDIDATE_WORKFLOW_ID:
        fail("candidate trigger workflow ID mismatch")
    if candidate_run["path"] != CANDIDATE_WORKFLOW_PATH:
        fail("candidate trigger path mismatch")
    if candidate_run["event"] != "pull_request":
        fail("candidate trigger event mismatch")
    if require_json_int(candidate_run["head_repository"].get("id"), "candidate head repository ID", minimum=1) != BASE_REPOSITORY_ID:
        fail("candidate trigger repository mismatch")
    if candidate_run["head_sha"] != subject_sha:
        fail("candidate trigger head differs from receipt subject")
    # run_attempt was already type-checked and bounded above as part of the
    # receipt-to-trigger join; do not accept bool/float aliases here.

    if require_json_int(trusted_run.get("id"), "trusted workflow run ID", minimum=1) != expected_trusted_run_id:
        fail("trusted workflow run object mismatch")
    if require_json_int(trusted_run.get("run_attempt"), "trusted workflow run attempt", minimum=1) != expected_trusted_run_attempt:
        fail("trusted workflow run attempt mismatch")
    if trusted_run["name"] != TRUSTED_WORKFLOW_NAME:
        fail("trusted workflow name mismatch")
    if trusted_run["path"] != TRUSTED_WORKFLOW_PATH:
        fail("trusted workflow path mismatch")
    if trusted_run["event"] != "workflow_run":
        fail("trusted workflow event mismatch")
    if trusted_run["status"] != "completed" or trusted_run["conclusion"] != "success":
        fail("trusted workflow did not complete successfully")
    if trusted_run["repository"]["full_name"] != BASE_REPOSITORY:
        fail("trusted workflow repository mismatch")

    if pr["state"] not in {"open", "closed"}:
        fail("PR state is outside GitHub historical PR states")
    if pr["base"]["ref"] != BASE_BRANCH:
        fail("PR base ref mismatch")
    if require_json_int(pr["base"]["repo"].get("id"), "PR base repository ID", minimum=1) != BASE_REPOSITORY_ID:
        fail("PR base repository mismatch")
    if require_json_int(pr["head"]["repo"].get("id"), "PR head repository ID", minimum=1) != BASE_REPOSITORY_ID:
        fail("PR head repository mismatch")
    if pr["head"]["repo"]["full_name"] != BASE_REPOSITORY:
        fail("PR head repository name mismatch")
    if pr["head"]["sha"] != subject_sha:
        fail("live PR head SHA mismatch")

    if commit["sha"] != subject_sha:
        fail("subject commit object mismatch")
    if commit["commit"]["tree"]["sha"] != subject_tree:
        fail("subject tree SHA mismatch")

    if manifest["path"] != MANIFEST_PATH:
        fail("manifest path mismatch")
    if manifest["sha"] != MANIFEST_BLOB_SHA:
        fail("manifest blob mismatch")


    if receipt["manifest_blob_sha"] != MANIFEST_BLOB_SHA:
        fail("receipt manifest blob mismatch")
    lock_sha = require_sha256(receipt["lock_sha256"], "lock_sha256")
    lock_mode = receipt["lock_mode"]
    if lock_mode not in {"tracked", "generated_for_run"}:
        fail("unknown lock mode")
    if lock_mode == "tracked":
        if candidate_lock.get("encoding") != "base64":
            fail("tracked candidate lockfile was not returned as base64")
        lock_content_b64 = candidate_lock.get("content")
        if not isinstance(lock_content_b64, str):
            fail("tracked candidate lockfile content missing")
        try:
            lock_bytes = base64.b64decode("".join(lock_content_b64.split()), validate=True)
        except (ValueError, base64.binascii.Error) as exc:
            fail(f"tracked candidate lockfile base64 invalid: {exc}")
        if hashlib.sha256(lock_bytes).hexdigest() != lock_sha:
            fail("tracked candidate lockfile digest mismatch")
        verify_tracked_lock_source_policy(lock_bytes)
    else:
        if candidate_lock.get("mode") != "generated_for_run":
            fail("unexpected generated lockfile marker")

    if receipt["rustc_version"] != RUSTC_VERSION:
        fail("rustc version mismatch")
    if receipt["rustc_commit"] != RUSTC_COMMIT:
        fail("rustc commit mismatch")
    if not isinstance(receipt["cargo_version"], str) or not receipt["cargo_version"].startswith("cargo 1.96.1"):
        fail("cargo version mismatch")
    if receipt["candidate_execution_profile"] != "fpm-docker-offline-v1":
        fail("unexpected candidate execution profile")
    require_json_int(receipt["candidate_uid"], "receipt.candidate_uid", minimum=1)
    require_json_int(receipt["candidate_gid"], "receipt.candidate_gid", minimum=1)
    require_sha256_prefixed(receipt["sandbox_image_digest"], "sandbox_image_digest")
    if receipt["sandbox_image_digest"] != "sha256:603634c53d477dd94dd224a3dd5c008e996dd3ec8ccc54ac6af9344d990339e5":
        fail("unexpected sandbox image digest")
    if receipt["sandbox_probe"] != "passed":
        fail("sandbox boundary probe did not pass")
    verify_sandbox_system_closure(receipt["sandbox_system_closure"])
    closure_canonical = json.dumps(receipt["sandbox_system_closure"], sort_keys=True, separators=(",", ":")).encode("utf-8")
    if receipt["sandbox_system_closure_sha256"] != hashlib.sha256(closure_canonical).hexdigest():
        fail("sandbox system closure commitment mismatch")
    verify_sandbox_target_closure(receipt["sandbox_target_closure"])
    target_closure_canonical = json.dumps(receipt["sandbox_target_closure"], sort_keys=True, separators=(",", ":")).encode("utf-8")
    if receipt["sandbox_target_closure_sha256"] != hashlib.sha256(target_closure_canonical).hexdigest():
        fail("sandbox target closure commitment mismatch")
    require_sha256(receipt["dependency_cache_sha256"], "dependency_cache_sha256")
    if receipt["dependency_source_policy"] != "crates-io-registry-only-v1":
        fail("unexpected dependency source policy")

    steps = receipt["steps"]
    if not isinstance(steps, dict) or set(steps) != STEP_KEYS:
        fail("receipt step outcome schema mismatch")
    if any(value != "success" for value in steps.values()):
        fail("receipt has non-success step outcome")
    if receipt["execution_pass"] is not True:
        fail("receipt execution_pass is not true")
    if receipt["procedure_trust"] != "trusted_default_branch_snapshot":
        fail("unexpected procedure trust value")
    if receipt["promotion_authority"] != "pending_repository_governance_evidence":
        fail("unexpected promotion authority")

    if verifier_control["repository"] != BASE_REPOSITORY:
        fail("independent verifier repository mismatch")
    if verifier_control["path"] != INDEPENDENT_WORKFLOW_PATH:
        fail("independent verifier path mismatch")
    if verifier_control["ref"] != "refs/heads/main":
        fail("independent verifier ref mismatch")
    verifier_sha = require_hex(verifier_control["workflow_sha"], 40, "independent verifier workflow_sha")
    verifier_blob_sha = require_hex(
        verifier_control["workflow_blob_sha"], 40, "independent verifier workflow_blob_sha"
    )
    if verifier_control["reference_verifier_path"] != "scripts/integral/verify_fpm_trusted_qualification.py":
        fail("reference verifier path mismatch")
    require_hex(verifier_control["reference_verifier_blob_sha"], 40, "reference verifier blob SHA")
    if verifier_control["artifact_collector_path"] != "scripts/integral/collect_fpm_trusted_artifacts.py":
        fail("artifact collector path mismatch")
    require_hex(verifier_control["artifact_collector_blob_sha"], 40, "artifact collector blob SHA")
    expected_workflow_ref = f"{BASE_REPOSITORY}/{INDEPENDENT_WORKFLOW_PATH}@refs/heads/main"
    if verifier_control["workflow_ref"] != expected_workflow_ref:
        fail("independent verifier workflow_ref mismatch")

    canonical = json.dumps(
        receipt, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest(), verifier_sha, verifier_blob_sha


def verify_artifact_enumeration(
    enumeration: dict[str, Any], items: list[dict[str, Any]]
) -> None:
    if set(enumeration) != ENUMERATION_KEYS:
        fail(
            "artifact enumeration closed-world mismatch: "
            f"missing={sorted(ENUMERATION_KEYS - set(enumeration))!r} "
            f"extra={sorted(set(enumeration) - ENUMERATION_KEYS)!r}"
        )
    if enumeration["schema"] != "mycelix.fpm.trusted-qualification-artifact-enumeration.v1":
        fail("unexpected artifact enumeration schema")
    # Python considers bool a subclass of int, and numeric equality also
    # accepts values such as 1.0 == 1. Treat JSON schema integers strictly:
    # otherwise malformed independent evidence can pass equality checks.
    integer_fields = (
        "page_size",
        "max_pages",
        "max_artifacts",
        "terminal_page",
        "enumerated_count",
        "total_count_reported",
        "repeat_total_count_reported",
    )
    for field in integer_fields:
        if type(enumeration[field]) is not int:
            fail(f"artifact enumeration {field} must be a JSON integer")
    if enumeration["page_size"] != 100:
        fail("unexpected artifact enumeration page size")
    if enumeration["max_pages"] != 4:
        fail("unexpected artifact enumeration page bound")
    if enumeration["max_artifacts"] != 256:
        fail("unexpected artifact enumeration global bound")
    if enumeration["complete"] is not True:
        fail("artifact enumeration is not marked complete")
    counts = enumeration["page_counts"]
    if not isinstance(counts, list) or not counts:
        fail("artifact enumeration page_counts is invalid")
    if any(type(x) is not int or x < 0 or x > 100 for x in counts):
        fail("artifact enumeration page count is invalid")
    if enumeration["terminal_page"] != len(counts):
        fail("terminal page does not match page_counts length")
    if enumeration["enumerated_count"] != len(items):
        fail("enumerated count does not match artifact list")
    if enumeration["total_count_reported"] != len(items):
        fail("reported total count does not match artifact list")
    if sum(counts) != len(items):
        fail("page counts do not sum to artifact count")
    if counts[-1] >= 100:
        fail("artifact enumeration did not observe a short/empty terminal page")
    if enumeration["repeat_enumeration_verified"] is not True:
        fail("artifact enumeration repeat-consistency check did not pass")
    repeat_counts = enumeration["repeat_page_counts"]
    if not isinstance(repeat_counts, list) or any(
        type(x) is not int or x < 0 or x > 100 for x in repeat_counts
    ):
        fail("repeat artifact page counts are invalid")
    if enumeration["repeat_total_count_reported"] != enumeration["total_count_reported"]:
        fail("repeat artifact total count differs")
    if repeat_counts != counts:
        fail("repeat artifact page counts differ")
    if enumeration["repeat_artifact_identity_sha256"] != enumeration["artifact_identity_sha256"]:
        fail("repeat artifact identity commitment differs")
    identities = []
    for position, item in enumerate(items):
        if type(item) is not dict:
            fail(f"artifact enumeration item {position} is not an object")
        artifact_id = require_json_int(
            item.get("id"), f"artifact enumeration item {position} ID", minimum=1
        )
        name = item.get("name")
        if not isinstance(name, str) or not name:
            fail(f"artifact enumeration item {position} has an invalid name")
        identities.append({"id": artifact_id, "name": name})
    commitment = hashlib.sha256(
        json.dumps(identities, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()
    if enumeration["artifact_identity_sha256"] != commitment:
        fail("artifact identity enumeration commitment mismatch")

def verify_index(
    index: dict[str, Any],
    receipt_digest: str,
    receipt: dict[str, Any],
    primary_artifact: dict[str, Any],
    trusted_run_id: int,
) -> None:
    if set(index) != INDEX_KEYS:
        fail(
            "index closed-world mismatch: "
            f"missing={sorted(INDEX_KEYS - set(index))!r} "
            f"extra={sorted(set(index) - INDEX_KEYS)!r}"
        )
    if index["schema"] != "mycelix.fpm.trusted-qualification-artifact-index.v1":
        fail("unexpected index schema")

    artifact = index["artifact"]
    if not isinstance(artifact, dict) or set(artifact) != INDEX_ARTIFACT_KEYS:
        fail("index artifact schema mismatch")

    artifact_id = require_json_int(artifact["id"], "index artifact ID", minimum=1)
    if artifact_id != primary_artifact["id"]:
        fail("index artifact ID mismatch")
    artifact_digest = require_sha256(artifact["sha256_hex"], "index artifact sha256_hex")
    if f"sha256:{artifact_digest}" != primary_artifact["digest"]:
        fail("index artifact digest mismatch")
    expected_artifact_url = (
        f"https://github.com/{BASE_REPOSITORY}/actions/runs/"
        f"{trusted_run_id}/artifacts/{artifact_id}"
    )
    if artifact["url"] != expected_artifact_url:
        fail("index artifact URL mismatch")
    if require_json_int(artifact["retention_days"], "index artifact retention_days", minimum=1) != 90:
        fail("unexpected retention policy")
    if artifact["immutable_after_upload"] is not True:
        fail("index does not record artifact immutability")
    if artifact["deletion_by_repository_writer_possible"] is not True:
        fail("index must preserve deletion caveat")

    if index["receipt_sha256"] != receipt_digest:
        fail("index receipt digest does not match canonical receipt")
    if index["subject_sha"] != receipt["subject_sha"]:
        fail("index subject SHA mismatch")
    if index["subject_tree_sha"] != receipt["subject_tree_sha"]:
        fail("index subject tree mismatch")
    if index["trusted_policy_sha"] != receipt["trusted_policy_sha"]:
        fail("index policy SHA mismatch")
    if index["trusted_policy_blob_sha"] != receipt["trusted_policy_blob_sha"]:
        fail("index policy blob mismatch")
    if require_json_int(index["trusted_workflow_run_id"], "index trusted_workflow_run_id", minimum=1) != trusted_run_id:
        fail("index trusted run ID mismatch")


def is_current_promotion_eligible(
    pr: dict[str, Any],
    receipt: dict[str, Any],
    trusted_run: dict[str, Any],
    main_sha: str,
) -> bool:
    return (
        receipt["lock_mode"] == "tracked"
        and pr["state"] == "open"
        and pr["draft"] is False
        and pr["head"]["sha"] == receipt["subject_sha"]
        and pr["base"]["sha"] == main_sha
        and trusted_run["head_sha"] == main_sha
    )


def verify(snapshot_dir: Path) -> dict[str, Any]:
    receipt = load_canonical_json(snapshot_dir / "qualification-receipt.json")
    index = load_canonical_json(snapshot_dir / "artifact-binding-index.json")
    trusted_run = load_strict_json(snapshot_dir / "trusted-run.json")
    candidate_run = load_strict_json(snapshot_dir / "candidate-run.json")
    pr = load_strict_json(snapshot_dir / "pull-request.json")
    commit = load_strict_json(snapshot_dir / "subject-commit.json")
    manifest = load_strict_json(snapshot_dir / "manifest.json")
    policy_file = load_strict_json(snapshot_dir / "policy-file.json")
    verifier_control = load_canonical_json(snapshot_dir / "verifier-control.json")
    if set(verifier_control) != CONTROL_KEYS:
        fail(
            "verifier-control closed-world mismatch: "
            f"missing={sorted(CONTROL_KEYS - set(verifier_control))!r} "
            f"extra={sorted(set(verifier_control) - CONTROL_KEYS)!r}"
        )
    candidate_lock = load_strict_json(snapshot_dir / "candidate-lock.json")
    main_ref = load_strict_json(snapshot_dir / "main-ref.json")
    main_sha = require_hex(main_ref.get("object", {}).get("sha"), 40, "live main ref SHA")
    artifacts = load_strict_json(snapshot_dir / "artifacts.json")
    enumeration = load_canonical_json(snapshot_dir / "artifact-enumeration.json")

    artifact_items = artifacts.get("artifacts")
    if not isinstance(artifact_items, list):
        fail("artifact list is malformed")

    verify_artifact_enumeration(enumeration, artifact_items)

    if len(artifact_items) != 2:
        fail(f"trusted qualification run must contain exactly 2 artifacts, found {len(artifact_items)}")

    subject_from_names = [
        item
        for item in artifact_items
        if re.fullmatch(
            r"fpm-trusted-qualification-[0-9a-f]{40}", item.get("name", "")
        )
    ]
    index_from_names = [
        item
        for item in artifact_items
        if re.fullmatch(
            r"fpm-trusted-qualification-index-[0-9a-f]{40}", item.get("name", "")
        )
    ]
    if len(subject_from_names) != 1:
        fail("expected exactly one receipt artifact")
    if len(index_from_names) != 1:
        fail("expected exactly one index artifact")
    expected_artifact_names = {
        subject_from_names[0]["name"],
        index_from_names[0]["name"],
    }
    if {item.get("name") for item in artifact_items} != expected_artifact_names:
        fail("trusted qualification artifact set contains an unexpected artifact")

    primary_artifact = subject_from_names[0]
    index_artifact = index_from_names[0]
    receipt_subject = require_hex(receipt["subject_sha"], 40, "receipt subject_sha")
    expected_receipt_name = f"fpm-trusted-qualification-{receipt_subject}"
    expected_index_name = f"fpm-trusted-qualification-index-{receipt_subject}"
    if primary_artifact["name"] != expected_receipt_name:
        fail("receipt artifact name does not bind to subject")
    if index_artifact["name"] != expected_index_name:
        fail("index artifact name does not bind to subject")

    for item, label in ((primary_artifact, "receipt"), (index_artifact, "index")):
        if item["expired"] is not False:
            fail(f"{label} artifact is expired")
        validate_artifact_lifetime(item, label)
        require_json_int(item.get("id"), f"{label} artifact ID", minimum=1)
        if require_json_int(item.get("size_in_bytes"), f"{label} artifact size", minimum=1) <= 0:
            fail(f"{label} artifact is empty")
        workflow_run = item.get("workflow_run")
        if type(workflow_run) is not dict:
            fail(f"{label} artifact workflow_run metadata is not an object")
        artifact_run_id = require_json_int(workflow_run.get("id"), f"{label} artifact workflow run ID", minimum=1)
        trusted_run_id_from_snapshot = require_json_int(trusted_run.get("id"), "trusted workflow run ID", minimum=1)
        if artifact_run_id != trusted_run_id_from_snapshot:
            fail(f"{label} artifact run mismatch")
        if require_json_int(workflow_run.get("repository_id"), f"{label} artifact repository ID", minimum=1) != BASE_REPOSITORY_ID:
            fail(f"{label} artifact repository mismatch")
        if require_json_int(workflow_run.get("head_repository_id"), f"{label} artifact head repository ID", minimum=1) != BASE_REPOSITORY_ID:
            fail(f"{label} artifact head repository mismatch")
        if item["workflow_run"]["head_sha"] != trusted_run["head_sha"]:
            fail(f"{label} artifact trusted-run head SHA mismatch")
        if item["workflow_run"]["head_branch"] != BASE_BRANCH:
            fail(f"{label} artifact trusted-run branch mismatch")
        require_sha256_prefixed(item["digest"], f"{label} artifact digest")

    receipt_archive = verify_raw_artifact_archive(
        archive=snapshot_dir / "raw/receipt.zip",
        expected_member="qualification-receipt.json",
        expected_digest=primary_artifact["digest"],
        expected_size_bytes=primary_artifact["size_in_bytes"],
        extracted=snapshot_dir / "qualification-receipt.json",
        label="receipt",
    )
    index_archive = verify_raw_artifact_archive(
        archive=snapshot_dir / "raw/index.zip",
        expected_member="artifact-binding-index.json",
        expected_digest=index_artifact["digest"],
        expected_size_bytes=index_artifact["size_in_bytes"],
        extracted=snapshot_dir / "artifact-binding-index.json",
        label="index",
    )

    receipt_digest, verifier_sha, verifier_blob_sha = verify_receipt(
        receipt=receipt,
        expected_trusted_run_id=trusted_run["id"],
        expected_trusted_run_attempt=trusted_run["run_attempt"],
        candidate_run=candidate_run,
        trusted_run=trusted_run,
        pr=pr,
        commit=commit,
        manifest=manifest,
        policy_file=policy_file,
        verifier_control=verifier_control,
        candidate_lock=candidate_lock,
        main_ref=main_ref,
    )

    verify_index(
        index=index,
        receipt_digest=receipt_digest,
        receipt=receipt,
        primary_artifact=primary_artifact,
        trusted_run_id=trusted_run["id"],
    )

    index_canonical = json.dumps(
        index, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    index_digest = hashlib.sha256(index_canonical).hexdigest()

    current_promotion_eligible = is_current_promotion_eligible(
        pr=pr,
        receipt=receipt,
        trusted_run=trusted_run,
        main_sha=main_sha,
    )

    return {
        "schema": "mycelix.fpm.trusted-qualification-reference-v1",
        "repository": BASE_REPOSITORY,
        "trusted_workflow_run_id": trusted_run["id"],
        "trusted_workflow_run_attempt": trusted_run["run_attempt"],
        "candidate_workflow_run_id": candidate_run["id"],
        "candidate_workflow_run_attempt": candidate_run["run_attempt"],
        "candidate_pr": pr["number"],
        "candidate_sha": receipt_subject,
        "candidate_tree": receipt["subject_tree_sha"],
        "receipt_content_sha256": receipt_digest,
        "receipt_artifact_id": primary_artifact["id"],
        "receipt_artifact_digest": primary_artifact["digest"],
        "receipt_artifact_created_at": primary_artifact["created_at"],
        "receipt_artifact_expires_at": primary_artifact["expires_at"],
        "receipt_artifact_archive_sha256": receipt_archive["archive_sha256"],
        "receipt_artifact_member_count": receipt_archive["member_count"],
        "receipt_artifact_member_names": receipt_archive["member_names"],
        "receipt_artifact_member_set_sha256": receipt_archive["member_set_sha256"],
        "receipt_artifact_member_sha256": receipt_archive["member_sha256"],
        "index_content_sha256": index_digest,
        "index_artifact_id": index_artifact["id"],
        "index_artifact_digest": index_artifact["digest"],
        "index_artifact_created_at": index_artifact["created_at"],
        "index_artifact_expires_at": index_artifact["expires_at"],
        "index_artifact_archive_sha256": index_archive["archive_sha256"],
        "index_artifact_member_count": index_archive["member_count"],
        "index_artifact_member_names": index_archive["member_names"],
        "index_artifact_member_set_sha256": index_archive["member_set_sha256"],
        "index_artifact_member_sha256": index_archive["member_sha256"],
        "trusted_policy_sha": receipt["trusted_policy_sha"],
        "trusted_policy_blob_sha": receipt["trusted_policy_blob_sha"],
        "independent_verifier_workflow_sha": verifier_sha,
        "independent_verifier_workflow_blob_sha": verifier_blob_sha,
        "reference_result": "verified",
        "historical_qualification_valid": True,
        "current_promotion_eligible": current_promotion_eligible,
    }


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: verify_fpm_trusted_qualification.py <snapshot-dir>")
    result = verify(Path(sys.argv[1]))
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
