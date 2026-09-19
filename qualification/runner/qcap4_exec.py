from __future__ import annotations

import hashlib
import os
import selectors
import subprocess
import sys
import time
from pathlib import Path

from qcap_canon import CapsuleError, canonical_json, capsule_commitment
from qcap_manifest import validate_manifest
from qcap3_limits import account_manifest_resources, limits_ref, validate_limits
from qcap4_containment import (
    ContainmentError,
    cleanup_empty,
    create_gate_cgroup,
    probe_containment_root,
    terminate_and_cleanup,
)
from qcap4_context import validate_execution_context_v4
from qcap4_receipt import compose_receipt_v4, gate_result, not_run_result
from qcap4_repo import (
    add_worktree,
    enable_linux_subreaper,
    preflight,
    reap_children_bounded,
    remove_worktree,
    state,
)

HERE = Path(__file__).resolve().parent
LAUNCH_WRAPPER = HERE / "qcap4_launch_wrapper.py"
LAUNCH_HANDSHAKE_MAX_BYTES = 4096
LAUNCH_HANDSHAKE_CEILING_SECONDS = 5.0
POST_EXIT_DRAIN_SECONDS = 0.25


def verified_gate_bytes(root, gate):
    root = Path(root).resolve()
    path = root
    for part in Path(gate["script"]).parts:
        path = path / part
        if path.is_symlink():
            raise CapsuleError("gate script symlink forbidden")
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise CapsuleError("gate script escapes capsule root") from error
    if not resolved.is_file():
        raise CapsuleError("gate script unavailable")
    content = resolved.read_bytes()
    if hashlib.sha256(content).hexdigest() != gate["sha256"]:
        raise CapsuleError("gate script digest changed before execution")
    return content


def snapshot_gate(parent, gate, content):
    directory = parent / "gate-snapshots"
    directory.mkdir(mode=0o700)
    token = hashlib.sha256(
        gate["id"].encode() + b"\0" + gate["sha256"].encode()
    ).hexdigest()
    path = directory / (token + ".gate")
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o700)
    except OSError as error:
        raise CapsuleError("gate snapshot creation failed") from error
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(path, 0o700)
    except OSError as error:
        try:
            path.unlink()
        except OSError:
            pass
        raise CapsuleError("gate snapshot write failed") from error
    if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != gate["sha256"]:
        raise CapsuleError("gate snapshot integrity failure")
    return path


def controlled_env(manifest, context, limits, attempt_id, worktree, scratch):
    return {
        "PATH": os.pathsep.join(
            [str(worktree / "qualification" / "runner" / "bin"), "/usr/bin", "/bin"]
        ),
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "TZ": "UTC",
        "PYTHONDONTWRITEBYTECODE": "1",
        "PYTHONHASHSEED": "0",
        "QCAP_CAPSULE_COMMITMENT": capsule_commitment(manifest),
        "QCAP_PRODUCT_SUBJECT_SHA": manifest["product_subject_sha"],
        "QCAP_PREDECESSOR_SHA": manifest["predecessor_sha"],
        "QCAP_EXPECTED_CHANGED_PATHS_JSON": canonical_json(
            manifest["expected_changed_paths"]
        ).decode(),
        "QCAP_EXPECTED_OBJECT_BLOBS_JSON": canonical_json(
            manifest["expected_object_blobs"]
        ).decode(),
        "QCAP_RESOLVED_RUNNER_COMMITMENT": context["resolved_runner_commitment"],
        "QCAP_RESOLVED_TOOLCHAIN_COMMITMENT": context[
            "resolved_toolchain_commitment"
        ],
        "QCAP_RESOLVED_ENVIRONMENT_COMMITMENT": context[
            "resolved_environment_commitment"
        ],
        "QCAP_RESOLVED_CONTAINMENT_COMMITMENT": context[
            "resolved_containment_commitment"
        ],
        "QCAP_CONTAINMENT_PROFILE_DIGEST": context["containment_profile_ref"]["digest"],
        "QCAP_EXECUTION_LIMITS_DIGEST": limits_ref(limits)["digest"],
        "QCAP_ATTEMPT_ID": attempt_id,
        "QCAP_SUBJECT_DIR": str(worktree),
        "QCAP_SCRATCH_DIR": str(scratch),
    }


def _read_launch_handshake(fd, process, deadline):
    os.set_blocking(fd, False)
    selector = selectors.DefaultSelector()
    selector.register(fd, selectors.EVENT_READ)
    data = bytearray()
    eof = False
    try:
        while not eof:
            now = time.monotonic()
            if now >= deadline:
                return "ContainmentFailure"
            events = selector.select(min(0.05, max(0.0, deadline - now)))
            for key, _ in events:
                try:
                    chunk = os.read(key.fd, 1024)
                except BlockingIOError:
                    continue
                if not chunk:
                    eof = True
                    break
                data.extend(chunk)
                if len(data) > LAUNCH_HANDSHAKE_MAX_BYTES:
                    return "ContainmentFailure"
            if process.poll() is not None and not events:
                continue
    finally:
        try:
            selector.close()
        except Exception:
            pass
        try:
            os.close(fd)
        except OSError:
            pass

    payload = bytes(data)
    if payload == b"MEMBER\n":
        return None
    if payload.startswith(b"MEMBER\nEXECERR:"):
        return "ProcessStartFailure"
    return "ContainmentFailure"


def _wait_direct(process, ceiling_seconds=0.1):
    if process.poll() is not None:
        return
    try:
        process.wait(timeout=max(0.01, ceiling_seconds))
    except subprocess.TimeoutExpired:
        pass


def _abort_containment(process, cgroup, cleanup_ceiling_ms):
    try:
        terminate_and_cleanup(cgroup, cleanup_ceiling_ms)
        _wait_direct(process, cleanup_ceiling_ms / 1000)
        reap_children_bounded(cleanup_ceiling_ms)
        return True
    except ContainmentError:
        _wait_direct(process, 0.05)
        return False


def _finish_containment(process, cgroup, cleanup_ceiling_ms):
    try:
        cleanup_empty(cgroup)
        _wait_direct(process, 0.05)
        reap_children_bounded(cleanup_ceiling_ms)
        return True
    except ContainmentError:
        try:
            terminate_and_cleanup(cgroup, cleanup_ceiling_ms)
            _wait_direct(process, cleanup_ceiling_ms / 1000)
            reap_children_bounded(cleanup_ceiling_ms)
        except ContainmentError:
            _wait_direct(process, 0.05)
        return False


def execute_bounded_cgroup(
    argv,
    cwd,
    env,
    timeout_seconds,
    max_output_bytes,
    cgroup,
    cleanup_ceiling_ms,
):
    if os.name != "posix":
        return b"", 20, "ContainmentFailure", False

    read_fd, write_fd = os.pipe()
    launch_env = dict(env)
    launch_env["QCAP4_LAUNCH_FD"] = str(write_fd)
    command = [
        sys.executable,
        "-I",
        "-B",
        str(LAUNCH_WRAPPER),
        str(Path(cgroup).resolve()),
        *argv,
    ]
    try:
        process = subprocess.Popen(
            command,
            cwd=cwd,
            env=launch_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
            pass_fds=(write_fd,),
        )
    except OSError:
        os.close(read_fd)
        os.close(write_fd)
        try:
            cleanup_empty(cgroup)
        except ContainmentError:
            return b"", 20, "ContainmentFailure", False
        return b"", 20, "ProcessStartFailure", False
    finally:
        try:
            os.close(write_fd)
        except OSError:
            pass

    started_at = time.monotonic()
    launch_reason = _read_launch_handshake(
        read_fd,
        process,
        started_at + LAUNCH_HANDSHAKE_CEILING_SECONDS,
    )
    if launch_reason is not None:
        cleaned = _abort_containment(process, cgroup, cleanup_ceiling_ms)
        try:
            if process.stdout is not None:
                process.stdout.close()
        except Exception:
            pass
        if not cleaned:
            return b"", 20, "ContainmentFailure", False
        return b"", 20, launch_reason, False

    deadline = time.monotonic() + timeout_seconds
    assert process.stdout is not None
    output_fd = process.stdout.fileno()
    os.set_blocking(output_fd, False)
    selector = selectors.DefaultSelector()
    selector.register(output_fd, selectors.EVENT_READ)
    captured = bytearray()
    eof = False
    reason = None
    exit_deadline = None
    truncated = False
    try:
        while True:
            now = time.monotonic()
            if process.poll() is None and now >= deadline:
                reason = "Timeout"
                break

            wait = 0.05 if process.poll() is None else 0.01
            events = selector.select(wait)
            for key, _ in events:
                remaining = max_output_bytes - len(captured)
                want = max(1, min(65536, remaining + 1))
                try:
                    chunk = os.read(key.fd, want)
                except BlockingIOError:
                    continue
                if not chunk:
                    eof = True
                    try:
                        selector.unregister(output_fd)
                    except Exception:
                        pass
                    break
                if len(chunk) > remaining:
                    if remaining:
                        captured.extend(chunk[:remaining])
                    reason = "OutputLimitExceeded"
                    truncated = True
                    break
                captured.extend(chunk)
            if reason is not None:
                break

            if process.poll() is not None:
                if eof:
                    break
                if exit_deadline is None:
                    exit_deadline = time.monotonic() + POST_EXIT_DRAIN_SECONDS
                if time.monotonic() >= exit_deadline:
                    reason = "OutputDrainTimeout"
                    break

        if reason is not None:
            if not _abort_containment(process, cgroup, cleanup_ceiling_ms):
                return b"", 20, "ContainmentFailure", False
            return bytes(captured), 20, reason, truncated

        return_code = process.returncode
        if return_code is None:
            try:
                return_code = process.wait(timeout=0.05)
            except subprocess.TimeoutExpired:
                if not _abort_containment(process, cgroup, cleanup_ceiling_ms):
                    return b"", 20, "ContainmentFailure", False
                return b"", 20, "RunnerInternalFailure", False

        if not _finish_containment(process, cgroup, cleanup_ceiling_ms):
            return b"", 20, "ContainmentFailure", False
        if return_code == 0:
            return bytes(captured), 0, None, False
        if return_code == 10:
            return bytes(captured), 10, None, False
        if return_code == 20:
            return bytes(captured), 20, "GateReportedRunnerFailure", False
        if return_code == 21:
            return bytes(captured), 20, "ProcessStartFailure", False
        return bytes(captured), 20, "UnexpectedExitCode", False
    finally:
        try:
            selector.close()
        except Exception:
            pass
        try:
            process.stdout.close()
        except Exception:
            pass


def _runner_failure_results(manifest, reason, captured=b""):
    results = []
    for index, gate in enumerate(manifest["gates"]):
        if index == 0:
            results.append(
                gate_result(
                    gate["id"],
                    "RunnerInfrastructureFailure",
                    20,
                    captured,
                    False,
                    reason,
                )
            )
        else:
            results.append(not_run_result(gate["id"]))
    return results


def execute_gate(
    manifest,
    root,
    repo,
    gate,
    attempt_id,
    context,
    limits,
    containment_profile,
    containment_root,
):
    parent = None
    worktree = None
    cgroup = None
    captured = b""
    code = 20
    reason = "RunnerInternalFailure"
    truncated = False
    try:
        gate_bytes = verified_gate_bytes(root, gate)
        parent, worktree = add_worktree(repo, manifest["product_subject_sha"])
        head, dirty = state(worktree)
        if head != manifest["product_subject_sha"] or dirty:
            raise CapsuleError("fresh worktree not exact")

        scratch = parent / "scratch"
        scratch.mkdir()
        script = snapshot_gate(parent, gate, gate_bytes)
        environment = controlled_env(
            manifest, context, limits, attempt_id, worktree, scratch
        )
        try:
            cgroup = create_gate_cgroup(containment_root, attempt_id, gate["id"])
        except ContainmentError:
            reason = "ContainmentFailure"
        else:
            captured, code, reason, truncated = execute_bounded_cgroup(
                [str(script), *gate["args"]],
                worktree,
                environment,
                gate["timeout_seconds"],
                limits["max_gate_output_bytes"],
                cgroup,
                containment_profile["cleanup_ceiling_ms"],
            )
            cgroup = None

        if code != 20:
            head, dirty = state(worktree)
            if head != manifest["product_subject_sha"] or dirty:
                code = 10
                reason = None
                truncated = False
    except CapsuleError as error:
        captured = (
            "qcap4_runner=RUNNER_FAILURE " + str(error) + "\n"
        ).encode()[: limits["max_gate_output_bytes"]]
        code = 20
        reason = "ArtifactIntegrityFailure"
        truncated = False
    finally:
        if cgroup is not None:
            try:
                terminate_and_cleanup(
                    cgroup, containment_profile["cleanup_ceiling_ms"]
                )
                reap_children_bounded(containment_profile["cleanup_ceiling_ms"])
            except ContainmentError:
                captured = b""
                code = 20
                reason = "ContainmentFailure"
                truncated = False
        if parent is not None:
            try:
                remove_worktree(repo, parent, worktree)
            except CapsuleError:
                if reason != "ContainmentFailure":
                    code = 20
                    reason = "WorktreeCleanupFailure"
                    truncated = False

    if reason == "ContainmentFailure":
        captured = b""
        truncated = False
    if captured:
        sys.stderr.buffer.write(captured + (b"" if captured.endswith(b"\n") else b"\n"))
    status = (
        "GatePass"
        if code == 0
        else ("GateFail" if code == 10 else "RunnerInfrastructureFailure")
    )
    return gate_result(gate["id"], status, code, captured, truncated, reason)


def run_capsule_v4(
    manifest,
    root,
    repo,
    repository_identity,
    attempt_id,
    context,
    limits,
    containment_profile,
    containment_root,
    expected_runner_commitment,
):
    enable_linux_subreaper()
    validate_limits(limits)
    validate_manifest(manifest, root)
    account_manifest_resources(manifest, root, limits)
    validate_execution_context_v4(
        context,
        manifest,
        limits,
        containment_profile,
        expected_runner_commitment=expected_runner_commitment,
    )
    preflight(manifest, repo, repository_identity)

    try:
        resolved = probe_containment_root(containment_root, containment_profile)
    except ContainmentError:
        return compose_receipt_v4(
            manifest,
            attempt_id,
            context,
            limits,
            containment_profile,
            _runner_failure_results(manifest, "ContainmentFailure"),
        )
    if resolved != context["resolved_containment_commitment"]:
        return compose_receipt_v4(
            manifest,
            attempt_id,
            context,
            limits,
            containment_profile,
            _runner_failure_results(manifest, "ContainmentFailure"),
        )

    results = []
    infrastructure_failure = False
    for gate in manifest["gates"]:
        if infrastructure_failure:
            results.append(not_run_result(gate["id"]))
            continue
        result = execute_gate(
            manifest,
            root,
            repo,
            gate,
            attempt_id,
            context,
            limits,
            containment_profile,
            containment_root,
        )
        results.append(result)
        infrastructure_failure = result["status"] == "RunnerInfrastructureFailure"
    return compose_receipt_v4(
        manifest, attempt_id, context, limits, containment_profile, results
    )
