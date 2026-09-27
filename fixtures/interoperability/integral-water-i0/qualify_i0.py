#!/usr/bin/env python3
"""Process-separated qualification orchestrator for MYC-INT-006H."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Callable, Sequence

QUALIFIER_PROFILE = "myc-int-006h-process-separated-qualifier-v1"

SCHEMA = "myc-int-006c-water-i0-corpus.schema.json"
CORPUS = "myc-int-006c-water-i0-corpus.v1.json"
STIMULUS = "myc-int-006e-water-i0-stimulus.v1.json"
PROGRAMS = (
    "validate_i0.py",
    "validate_stimulus.py",
    "build_candidate_input.py",
    "conventional_i0_adapter.py",
    "evaluate_i0_results.py",
)


class QualificationError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _run(runner: Callable[..., Any], command: list[str], *, cwd: Path, stage: str) -> Any:
    completed = runner(command, cwd=str(cwd), capture_output=True, text=True)
    if completed.returncode != 0:
        stdout = (completed.stdout or "").strip()
        stderr = (completed.stderr or "").strip()
        raise QualificationError(
            f"{stage} failed with exit {completed.returncode}; stdout={stdout!r}; stderr={stderr!r}"
        )
    return completed


def _assert_candidate_isolation(
    command: Sequence[str],
    *,
    corpus: Path,
    schema: Path,
    evaluator: Path,
    candidate_input: Path,
) -> None:
    normalized = [
        str(Path(arg).resolve())
        for arg in command
        if isinstance(arg, str) and ("/" in arg or "\\" in arg)
    ]
    forbidden = {str(corpus.resolve()), str(schema.resolve()), str(evaluator.resolve())}
    leaked = sorted(forbidden.intersection(normalized))
    if leaked:
        raise QualificationError(f"candidate command leaked oracle-bearing path(s): {leaked}")
    if str(candidate_input.resolve()) not in normalized:
        raise QualificationError("candidate command does not receive the sanitized candidate input")


def _canonical_stage_commands() -> dict[str, list[str]]:
    return {
        "corpus_validation": ["python3", "validate_i0.py", CORPUS, "--expected-sha256", "<corpus-sha256>"],
        "stimulus_validation": [
            "python3", "validate_stimulus.py", STIMULUS,
            "--corpus", CORPUS, "--expected-sha256", "<stimulus-sha256>",
        ],
        "candidate_input_projection": [
            "python3", "build_candidate_input.py",
            "--corpus", CORPUS, "--schema", SCHEMA, "--stimulus", STIMULUS,
            "--output", "<candidate-input>",
        ],
        "candidate_execution": [
            "python3", "conventional_i0_adapter.py", "<candidate-input>",
            "--output", "<candidate-results>",
        ],
        "semantic_evaluation": [
            "python3", "evaluate_i0_results.py",
            "--oracle", CORPUS, "--schema", SCHEMA,
            "<candidate-results>", "--output", "<evaluation-report>",
        ],
    }


def run_qualification(
    base_dir: Path,
    *,
    runner: Callable[..., Any] = subprocess.run,
) -> dict[str, Any]:
    base_dir = base_dir.resolve()
    paths = {name: base_dir / name for name in (SCHEMA, CORPUS, STIMULUS, *PROGRAMS)}
    missing = sorted(name for name, path in paths.items() if not path.is_file())
    if missing:
        raise QualificationError(f"missing qualification inputs: {missing}")

    input_commitments = {name: sha256_file(path) for name, path in sorted(paths.items())}
    corpus_sha = input_commitments[CORPUS]
    stimulus_sha = input_commitments[STIMULUS]

    with tempfile.TemporaryDirectory(prefix="myc-int-006h-") as temp_name:
        work = Path(temp_name)
        candidate_input = work / "candidate-input.json"
        candidate_results = work / "candidate-results.json"
        evaluation_report = work / "evaluation-report.json"

        _run(
            runner,
            [sys.executable, str(paths["validate_i0.py"]), str(paths[CORPUS]), "--expected-sha256", corpus_sha],
            cwd=base_dir,
            stage="corpus validation",
        )
        _run(
            runner,
            [
                sys.executable, str(paths["validate_stimulus.py"]), str(paths[STIMULUS]),
                "--corpus", str(paths[CORPUS]), "--expected-sha256", stimulus_sha,
            ],
            cwd=base_dir,
            stage="stimulus validation",
        )
        _run(
            runner,
            [
                sys.executable, str(paths["build_candidate_input.py"]),
                "--corpus", str(paths[CORPUS]), "--schema", str(paths[SCHEMA]),
                "--stimulus", str(paths[STIMULUS]), "--output", str(candidate_input),
            ],
            cwd=base_dir,
            stage="candidate input projection",
        )
        if not candidate_input.is_file():
            raise QualificationError("candidate input projection succeeded without output")
        candidate_input_sha = sha256_file(candidate_input)

        candidate_command = [
            sys.executable, str(paths["conventional_i0_adapter.py"]),
            str(candidate_input), "--output", str(candidate_results),
        ]
        _assert_candidate_isolation(
            candidate_command,
            corpus=paths[CORPUS],
            schema=paths[SCHEMA],
            evaluator=paths["evaluate_i0_results.py"],
            candidate_input=candidate_input,
        )
        _run(runner, candidate_command, cwd=base_dir, stage="candidate execution")
        if not candidate_results.is_file():
            raise QualificationError("candidate execution succeeded without output")
        candidate_results_sha_before = sha256_file(candidate_results)

        _run(
            runner,
            [
                sys.executable, str(paths["evaluate_i0_results.py"]),
                "--oracle", str(paths[CORPUS]), "--schema", str(paths[SCHEMA]),
                str(candidate_results), "--output", str(evaluation_report),
            ],
            cwd=base_dir,
            stage="semantic evaluation",
        )
        if not evaluation_report.is_file():
            raise QualificationError("semantic evaluator succeeded without report")

        candidate_results_sha_after = sha256_file(candidate_results)
        if candidate_results_sha_before != candidate_results_sha_after:
            raise QualificationError("candidate results mutated during semantic evaluation")

        try:
            report = json.loads(evaluation_report.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise QualificationError(f"invalid evaluator report: {exc}") from exc
        if report.get("status") != "PASS":
            raise QualificationError(f"evaluator report did not PASS: {report.get('status')!r}")

        output_commitments = {
            "candidate-input.json": candidate_input_sha,
            "candidate-results.json": candidate_results_sha_after,
            "evaluation-report.json": sha256_file(evaluation_report),
        }
        return {
            "qualifier_profile": QUALIFIER_PROFILE,
            "status": "PASS",
            "input_commitments": input_commitments,
            "output_commitments": output_commitments,
            "stage_commands": _canonical_stage_commands(),
            "evaluation_summary": {
                "evaluator_profile": report.get("evaluator_profile"),
                "candidate_adapter_profile": report.get("candidate_adapter_profile"),
                "case_count": report.get("case_count"),
                "disposition_matches": report.get("disposition_matches"),
                "assertion_checks": report.get("assertion_checks"),
                "assertion_passes": report.get("assertion_passes"),
            },
        }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Directory containing the frozen I0 corpus and qualification programs.",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        manifest = run_qualification(args.base_dir)
    except (OSError, QualificationError) as exc:
        print(f"QUALIFICATION FAILED: {exc}", file=sys.stderr)
        return 1

    encoded = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
