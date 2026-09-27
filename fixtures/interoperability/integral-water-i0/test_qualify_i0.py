#!/usr/bin/env python3
"""Mutation controls for MYC-INT-006H process-separated qualification."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import qualify_i0 as q


class FakeRunner:
    def __init__(self, *, fail_candidate: bool = False, evaluator_status: str = "PASS", mutate_results: bool = False):
        self.calls: list[list[str]] = []
        self.fail_candidate = fail_candidate
        self.evaluator_status = evaluator_status
        self.mutate_results = mutate_results

    def __call__(self, command, *, cwd, capture_output, text):
        command = list(command)
        self.calls.append(command)
        script = Path(command[1]).name
        if script == "build_candidate_input.py":
            out = Path(command[command.index("--output") + 1])
            out.write_text('{"sanitized":true}\n', encoding="utf-8")
        elif script == "conventional_i0_adapter.py":
            if self.fail_candidate:
                return SimpleNamespace(returncode=3, stdout="", stderr="candidate failed")
            out = Path(command[command.index("--output") + 1])
            out.write_text('{"result_profile":"oracle-blind-candidate-results-v1","results":[]}\n', encoding="utf-8")
        elif script == "evaluate_i0_results.py":
            results = Path(command[-3])
            if self.mutate_results:
                results.write_text('{"mutated":true}\n', encoding="utf-8")
            out = Path(command[command.index("--output") + 1])
            report = {
                "status": self.evaluator_status,
                "evaluator_profile": "test-evaluator",
                "candidate_adapter_profile": "test-adapter",
                "case_count": 19,
                "disposition_matches": 19,
                "assertion_checks": 28,
                "assertion_passes": 28,
            }
            out.write_text(json.dumps(report, sort_keys=True) + "\n", encoding="utf-8")
        return SimpleNamespace(returncode=0, stdout="", stderr="")


def make_base(root: Path) -> Path:
    for name in (q.SCHEMA, q.CORPUS, q.STIMULUS, *q.PROGRAMS):
        (root / name).write_text(f"{name}\n", encoding="utf-8")
    return root


class QualificationTests(unittest.TestCase):
    def test_candidate_command_is_oracle_blind_and_manifest_is_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            base = make_base(Path(name))
            first_runner = FakeRunner()
            first = q.run_qualification(base, runner=first_runner)
            second_runner = FakeRunner()
            second = q.run_qualification(base, runner=second_runner)
            self.assertEqual(first, second)
            candidate_call = next(call for call in first_runner.calls if Path(call[1]).name == "conventional_i0_adapter.py")
            joined = " ".join(candidate_call)
            self.assertNotIn(q.CORPUS, joined)
            self.assertNotIn(q.SCHEMA, joined)
            self.assertNotIn("evaluate_i0_results.py", joined)
            self.assertEqual(first["status"], "PASS")

    def test_candidate_failure_prevents_evaluator(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            base = make_base(Path(name))
            runner = FakeRunner(fail_candidate=True)
            with self.assertRaises(q.QualificationError):
                q.run_qualification(base, runner=runner)
            self.assertFalse(any(Path(call[1]).name == "evaluate_i0_results.py" for call in runner.calls))

    def test_evaluator_fail_prevents_qualification_pass(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            base = make_base(Path(name))
            with self.assertRaises(q.QualificationError):
                q.run_qualification(base, runner=FakeRunner(evaluator_status="FAIL"))

    def test_input_commitment_changes_when_input_changes(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            base = make_base(Path(name))
            before = q.run_qualification(base, runner=FakeRunner())
            (base / q.STIMULUS).write_text("changed\n", encoding="utf-8")
            after = q.run_qualification(base, runner=FakeRunner())
            self.assertNotEqual(before["input_commitments"][q.STIMULUS], after["input_commitments"][q.STIMULUS])

    def test_candidate_result_mutation_during_evaluation_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            base = make_base(Path(name))
            with self.assertRaises(q.QualificationError):
                q.run_qualification(base, runner=FakeRunner(mutate_results=True))


if __name__ == "__main__":
    unittest.main()
