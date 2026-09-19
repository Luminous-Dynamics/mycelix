import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from qcap_canon import CapsuleError
from qcap3_limits import limits_ref
from qcap4_containment import ContainmentError, containment_ref
from qcap4_context import validate_execution_context_v4
from qcap4_receipt import compose_receipt_v4, gate_result, not_run_result, verify_receipt_v4
import qcap4_exec

PROFILE = {
    "containment_profile_format_revision": 1,
    "profile_id": "containment:linux-cgroup-v2-kill-v1",
    "profile_revision": 1,
    "platform": "linux",
    "primitive": "cgroup-v2",
    "membership_start": "before-gate-exec",
    "termination_policy": "cgroup-kill",
    "empty_postcondition": "cgroup-events-populated-zero",
    "descendant_scope": "cgroup-membership-preserving",
    "session_process_group_escape_covered": True,
    "cleanup_ceiling_ms": 1000,
    "hostile_code_sandbox": False,
}
LIMITS = {
    "execution_limits_format_revision": 1,
    "max_args_per_gate": 8,
    "max_claim_nonclaim_utf8_bytes": 2048,
    "max_gate_count": 8,
    "max_gate_output_bytes": 16,
    "max_gate_script_bytes": 4096,
    "max_manifest_canonical_bytes": 16384,
    "max_total_arg_bytes": 1024,
    "max_total_gate_script_bytes": 8192,
    "profile_id": "qcap:vector-v4",
    "profile_revision": 1,
}
MANIFEST = {
    "capsule_format_revision": 1,
    "claim": "canonical café / 雪",
    "environment_profile_ref": {
        "digest": "5" * 64,
        "id": "environment:qcap-v4",
        "revision": 1,
    },
    "expected_changed_paths": ["a.txt", "é.txt"],
    "expected_object_blobs": {"a.txt": "2" * 40, "é.txt": "3" * 40},
    "gates": [
        {
            "args": [],
            "class": "lineage",
            "id": "lineage",
            "script": "lineage.sh",
            "sha256": "6" * 64,
            "timeout_seconds": 11,
        },
        {
            "args": ["雪"],
            "class": "oracle",
            "id": "oracle",
            "script": "oracle.py",
            "sha256": "7" * 64,
            "timeout_seconds": 12,
        },
    ],
    "nonclaims": ["network", "世界"],
    "predecessor_sha": "0" * 40,
    "product_subject_sha": "1" * 40,
    "repository_identity": "Luminous-Dynamics/mycelix",
    "theorem_id": "QCAP-V4-VECTOR-001",
    "theorem_revision": 1,
    "toolchain_profile_ref": {
        "digest": "4" * 64,
        "id": "toolchain:qcap-v4",
        "revision": 1,
    },
    "verdict": {"kind": "all", "required_gate_ids": ["lineage", "oracle"]},
}
CONTEXT = {
    "containment_profile_ref": containment_ref(PROFILE),
    "environment_profile_ref": MANIFEST["environment_profile_ref"],
    "execution_context_format_revision": 4,
    "execution_limits_profile_ref": limits_ref(LIMITS),
    "resolved_containment_commitment": "d" * 64,
    "resolved_environment_commitment": "b" * 64,
    "resolved_runner_commitment": "9" * 64,
    "resolved_toolchain_commitment": "a" * 64,
    "runner_profile_ref": {"digest": "8" * 64, "id": "runner:qcap-v4", "revision": 1},
    "toolchain_profile_ref": MANIFEST["toolchain_profile_ref"],
}


class Qcap4Runtime(unittest.TestCase):
    def test_context_binds_containment_profile(self):
        self.assertIs(
            validate_execution_context_v4(CONTEXT, MANIFEST, LIMITS, PROFILE), CONTEXT
        )
        bad = dict(CONTEXT)
        bad["containment_profile_ref"] = dict(CONTEXT["containment_profile_ref"])
        bad["containment_profile_ref"]["digest"] = "0" * 64
        with self.assertRaisesRegex(CapsuleError, "containment profile mismatch"):
            validate_execution_context_v4(bad, MANIFEST, LIMITS, PROFILE)

    def test_receipt_implementation_matches_frozen_v4_vectors(self):
        pass_results = [
            gate_result("lineage", "GatePass", 0, b"lineage\n"),
            gate_result("oracle", "GatePass", 0, "oracle\n".encode()),
        ]
        receipt = compose_receipt_v4(
            MANIFEST, "vector-pass", CONTEXT, LIMITS, PROFILE, pass_results
        )
        self.assertEqual(
            receipt["receipt_commitment"],
            "8266dcb59038db3856f9b40dc1adcad5c62ee37e2579d9e32a45117ad3fe0e31",
        )
        self.assertTrue(verify_receipt_v4(receipt, MANIFEST, LIMITS, PROFILE))

        containment_results = [
            gate_result(
                "lineage",
                "RunnerInfrastructureFailure",
                20,
                b"",
                False,
                "ContainmentFailure",
            ),
            not_run_result("oracle"),
        ]
        receipt = compose_receipt_v4(
            MANIFEST,
            "vector-containment-failure",
            CONTEXT,
            LIMITS,
            PROFILE,
            containment_results,
        )
        self.assertEqual(
            receipt["receipt_commitment"],
            "a74633d56427801e94a5860302529fb05c088965995a730b2fd6376bdbec4ed2",
        )

    def test_launch_wrapper_proves_membership_before_exec(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cgroup = root / "cg"
            cgroup.mkdir()
            procs = cgroup / "cgroup.procs"
            procs.write_text("")
            marker = root / "marker"
            gate = root / "gate.py"
            gate.write_text(
                "#!/usr/bin/env python3\n"
                "from pathlib import Path\n"
                "import os,sys\n"
                "members={int(x) for x in Path(sys.argv[2]).read_text().split() if x.isdigit()}\n"
                "Path(sys.argv[1]).write_text('member' if os.getpid() in members else 'missing')\n"
            )
            gate.chmod(0o755)
            read_fd, write_fd = os.pipe()
            env = dict(os.environ)
            env["QCAP4_LAUNCH_FD"] = str(write_fd)
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-I",
                    "-B",
                    str(HERE / "qcap4_launch_wrapper.py"),
                    str(cgroup),
                    str(gate),
                    str(marker),
                    str(procs),
                ],
                env=env,
                pass_fds=(write_fd,),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            os.close(write_fd)
            handshake = os.read(read_fd, 4096)
            os.close(read_fd)
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stdout + stderr)
            self.assertEqual(handshake, b"MEMBER\n")
            self.assertEqual(marker.read_text(), "member")

    def test_launch_wrapper_fails_before_gate_without_membership_control(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cgroup = root / "cg"
            cgroup.mkdir()
            marker = root / "marker"
            gate = root / "gate.sh"
            gate.write_text(f"#!/bin/sh\nprintf ran > '{marker}'\n")
            gate.chmod(0o755)
            read_fd, write_fd = os.pipe()
            env = dict(os.environ)
            env["QCAP4_LAUNCH_FD"] = str(write_fd)
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-I",
                    "-B",
                    str(HERE / "qcap4_launch_wrapper.py"),
                    str(cgroup),
                    str(gate),
                ],
                env=env,
                pass_fds=(write_fd,),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            os.close(write_fd)
            handshake = os.read(read_fd, 4096)
            os.close(read_fd)
            process.communicate(timeout=3)
            self.assertEqual(process.returncode, 20)
            self.assertTrue(handshake.startswith(b"ERR:"))
            self.assertFalse(marker.exists())

    def test_launch_wrapper_distinguishes_exec_failure_after_membership(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cgroup = root / "cg"
            cgroup.mkdir()
            (cgroup / "cgroup.procs").write_text("")
            read_fd, write_fd = os.pipe()
            env = dict(os.environ)
            env["QCAP4_LAUNCH_FD"] = str(write_fd)
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-I",
                    "-B",
                    str(HERE / "qcap4_launch_wrapper.py"),
                    str(cgroup),
                    str(root / "missing-gate"),
                ],
                env=env,
                pass_fds=(write_fd,),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            os.close(write_fd)
            chunks = []
            while True:
                chunk = os.read(read_fd, 4096)
                if not chunk:
                    break
                chunks.append(chunk)
            os.close(read_fd)
            process.communicate(timeout=3)
            self.assertEqual(process.returncode, 21)
            self.assertEqual(b"".join(chunks), b"MEMBER\n")

    def _fake_cgroup_and_gate(self, gate_body):
        temporary = tempfile.TemporaryDirectory()
        root = Path(temporary.name)
        cgroup = root / "cg"
        cgroup.mkdir()
        (cgroup / "cgroup.procs").write_text("")
        gate = root / "gate.sh"
        gate.write_text("#!/bin/sh\n" + gate_body)
        gate.chmod(0o755)
        return temporary, root, cgroup, gate

    def test_bounded_execution_accepts_clean_gate_only_after_cleanup(self):
        temporary, root, cgroup, gate = self._fake_cgroup_and_gate("printf ok\nexit 0\n")
        with temporary:
            with mock.patch.object(qcap4_exec, "cleanup_empty") as cleanup, mock.patch.object(
                qcap4_exec, "reap_children_bounded"
            ):
                captured, code, reason, truncated = qcap4_exec.execute_bounded_cgroup(
                    [str(gate)], root, {"PATH": "/usr/bin:/bin"}, 2, 64, cgroup, 1000
                )
            self.assertEqual((captured, code, reason, truncated), (b"ok", 0, None, False))
            cleanup.assert_called_once_with(cgroup)

    def test_output_limit_requires_containment_cleanup(self):
        temporary, root, cgroup, gate = self._fake_cgroup_and_gate(
            "printf 0123456789abcdef0123456789abcdef\nexit 0\n"
        )
        with temporary:
            with mock.patch.object(qcap4_exec, "terminate_and_cleanup") as terminate, mock.patch.object(
                qcap4_exec, "reap_children_bounded"
            ):
                captured, code, reason, truncated = qcap4_exec.execute_bounded_cgroup(
                    [str(gate)], root, {"PATH": "/usr/bin:/bin"}, 2, 8, cgroup, 1000
                )
            self.assertEqual(code, 20)
            self.assertEqual(reason, "OutputLimitExceeded")
            self.assertTrue(truncated)
            self.assertEqual(len(captured), 8)
            terminate.assert_called_once_with(cgroup, 1000)

    def test_residual_descendant_condition_becomes_containment_failure(self):
        temporary, root, cgroup, gate = self._fake_cgroup_and_gate("exit 0\n")
        with temporary:
            with mock.patch.object(
                qcap4_exec,
                "cleanup_empty",
                side_effect=ContainmentError("residual descendant process"),
            ), mock.patch.object(qcap4_exec, "terminate_and_cleanup"), mock.patch.object(
                qcap4_exec, "reap_children_bounded"
            ):
                captured, code, reason, truncated = qcap4_exec.execute_bounded_cgroup(
                    [str(gate)], root, {"PATH": "/usr/bin:/bin"}, 2, 64, cgroup, 1000
                )
            self.assertEqual(
                (captured, code, reason, truncated),
                (b"", 20, "ContainmentFailure", False),
            )

    def test_exec_failure_is_not_mislabeled_as_gate_reported_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cgroup = root / "cg"
            cgroup.mkdir()
            (cgroup / "cgroup.procs").write_text("")
            with mock.patch.object(qcap4_exec, "cleanup_empty"), mock.patch.object(
                qcap4_exec, "terminate_and_cleanup"
            ), mock.patch.object(qcap4_exec, "reap_children_bounded"):
                captured, code, reason, truncated = qcap4_exec.execute_bounded_cgroup(
                    [str(root / "missing")],
                    root,
                    {"PATH": "/usr/bin:/bin"},
                    2,
                    64,
                    cgroup,
                    1000,
                )
            self.assertEqual(
                (captured, code, reason, truncated),
                (b"qcap4_launch_wrapper=gate exec failed\n", 20, "ProcessStartFailure", False),
            )

    def test_containment_failure_overrides_output_limit_without_false_truncation_claim(self):
        temporary, root, cgroup, gate = self._fake_cgroup_and_gate(
            "printf 0123456789abcdef0123456789abcdef\nexit 0\n"
        )
        with temporary:
            with mock.patch.object(
                qcap4_exec,
                "terminate_and_cleanup",
                side_effect=ContainmentError("cleanup failed"),
            ):
                captured, code, reason, truncated = qcap4_exec.execute_bounded_cgroup(
                    [str(gate)], root, {"PATH": "/usr/bin:/bin"}, 2, 8, cgroup, 1000
                )
            self.assertEqual(
                (captured, code, reason, truncated),
                (b"", 20, "ContainmentFailure", False),
            )

    def test_missing_membership_control_is_typed_containment_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cgroup = root / "cg"
            cgroup.mkdir()
            gate = root / "gate.sh"
            gate.write_text("#!/bin/sh\nexit 0\n")
            gate.chmod(0o755)
            with mock.patch.object(qcap4_exec, "terminate_and_cleanup"), mock.patch.object(
                qcap4_exec, "reap_children_bounded"
            ):
                captured, code, reason, truncated = qcap4_exec.execute_bounded_cgroup(
                    [str(gate)], root, {"PATH": "/usr/bin:/bin"}, 2, 64, cgroup, 1000
                )
            self.assertEqual(
                (captured, code, reason, truncated),
                (b"", 20, "ContainmentFailure", False),
            )


if __name__ == "__main__":
    unittest.main()
