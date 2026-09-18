#!/usr/bin/env python3
"""Trusted post-run authority algebra for generic Mycelix CI.

This module performs no network calls and publishes no checks. It consumes:
- a trusted admission-v2 profile/evaluator,
- a complete PR-files observation,
- normalized metadata for the triggering Mycelix CI run,
- a complete Actions job observation.

It never trusts the triggering workflow's path-selection outputs or ci-pass job.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

AUTHORITY_ID = "generic-mycelix-ci-authority-v1"
AUTHORITY = "PostRunMergeAuthorityCandidate"
EXPECTED_WORKFLOW_NAME = "Mycelix CI"
EXPECTED_WORKFLOW_PATH = ".github/workflows/ci.yml"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")
KNOWN_JOB_CONCLUSIONS = {
    "success", "failure", "neutral", "cancelled", "skipped",
    "timed_out", "action_required", "stale", "startup_failure",
}

JOB_DISPLAY = {
    "format": "format",
    "test-commons": "test-commons",
    "test-civic": "test-civic",
    "test-hearth": "test-hearth",
    "test-finance": "test-finance",
    "test-finance-integration": "Finance Integration (SweetConductor)",
    "test-governance": "test-governance",
    "test-identity": "test-identity",
    "test-personal": "test-personal",
    "test-attribution": "test-attribution",
    "test-bridge": "test-bridge",
    "test-sdk": "test-sdk",
    "test-prism": "test-prism",
}


class AuthorityError(ValueError):
    pass


def require(ok: bool, message: str) -> None:
    if not ok:
        raise AuthorityError(message)


def sha40(value: Any) -> str:
    require(
        isinstance(value, str) and SHA_RE.fullmatch(value) is not None,
        f"expected lowercase 40-hex SHA, got {value!r}",
    )
    return value


def positive_int(value: Any, label: str) -> int:
    require(
        isinstance(value, int) and not isinstance(value, bool) and value > 0,
        f"{label} must be positive integer",
    )
    return value


def load_admission_module(path: Path):
    spec = importlib.util.spec_from_file_location("_trusted_admission_v2", path)
    require(spec is not None and spec.loader is not None, "cannot load admission evaluator")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class AuthorityResult:
    disposition: str
    reason: str
    subject_sha: str | None
    publish_authority: bool
    exit_code: int
    expected_required_jobs: tuple[str, ...]
    expected_informational_jobs: tuple[str, ...]
    observed_required: dict[str, str]
    observed_informational: dict[str, str]
    execution_drift: tuple[str, ...]

    def receipt(self, run: dict[str, Any], admission: Any | None) -> dict[str, Any]:
        return {
            "authority_id": AUTHORITY_ID,
            "authority": AUTHORITY,
            "disposition": self.disposition,
            "reason": self.reason,
            "subject_sha": self.subject_sha,
            "publish_authority": self.publish_authority,
            "run_id": run.get("run_id"),
            "run_attempt": run.get("run_attempt"),
            "workflow_name": run.get("workflow_name"),
            "workflow_path": run.get("workflow_path"),
            "workflow_conclusion": run.get("workflow_conclusion"),
            "admission_disposition": getattr(admission, "disposition", None),
            "expected_required_jobs": list(self.expected_required_jobs),
            "expected_informational_jobs": list(self.expected_informational_jobs),
            "observed_required": self.observed_required,
            "observed_informational": self.observed_informational,
            "execution_drift": list(self.execution_drift),
            "trusted_required_checks_passed": self.disposition
            in {"TrustedRequiredChecksPassed", "TrustedGenericCiNotRequired"},
            "grants_product_qualification": False,
        }


def outcome(
    disposition: str,
    reason: str,
    *,
    subject_sha: str | None,
    publish: bool,
    exit_code: int,
    expected_required: tuple[str, ...] = (),
    expected_info: tuple[str, ...] = (),
    observed_required: dict[str, str] | None = None,
    observed_info: dict[str, str] | None = None,
    drift: tuple[str, ...] = (),
) -> AuthorityResult:
    return AuthorityResult(
        disposition,
        reason,
        subject_sha,
        publish,
        exit_code,
        expected_required,
        expected_info,
        observed_required or {},
        observed_info or {},
        drift,
    )


def validate_run(run: Any) -> tuple[str, bool]:
    require(isinstance(run, dict), "run metadata must be object")
    require(run.get("workflow_name") == EXPECTED_WORKFLOW_NAME, "workflow name drift")
    require(run.get("workflow_path") == EXPECTED_WORKFLOW_PATH, "workflow path drift")
    require(run.get("event") == "pull_request", "authority applies only to pull_request runs")
    positive_int(run.get("run_id"), "run_id")
    positive_int(run.get("run_attempt"), "run_attempt")
    subject = sha40(run.get("pr_head_sha"))
    current = sha40(run.get("current_pr_head_sha"))
    conclusion = run.get("workflow_conclusion")
    require(isinstance(conclusion, str) and conclusion, "workflow_conclusion missing")
    return subject, subject != current


def normalize_jobs(raw: Any) -> dict[str, str]:
    require(isinstance(raw, list), "jobs observation must be list")
    by_name: dict[str, str] = {}
    for index, item in enumerate(raw):
        require(isinstance(item, dict), f"jobs[{index}] must be object")
        name = item.get("name")
        require(isinstance(name, str) and name, f"jobs[{index}].name invalid")
        require(name not in by_name, f"duplicate job name {name!r}")
        status = item.get("status")
        conclusion = item.get("conclusion")
        require(status == "completed", f"job {name!r} not completed: {status!r}")
        require(
            conclusion in KNOWN_JOB_CONCLUSIONS,
            f"job {name!r} unknown conclusion {conclusion!r}",
        )
        by_name[name] = conclusion
    return by_name


def evaluate(
    admission_module: Any,
    profile: dict[str, Any],
    run: dict[str, Any],
    declared_changed_files: Any,
    file_records: Any,
    jobs: Any,
) -> tuple[AuthorityResult, Any | None]:
    try:
        subject, stale = validate_run(run)
    except AuthorityError as exc:
        return (
            outcome(
                "TrustedAuthorityInputInvalid",
                str(exc),
                subject_sha=None,
                publish=False,
                exit_code=2,
            ),
            None,
        )

    if stale:
        return (
            outcome(
                "TrustedStaleSubject",
                "triggering run subject is not the current PR head; current-head authority suppressed",
                subject_sha=subject,
                publish=False,
                exit_code=0,
            ),
            None,
        )

    try:
        admission_module.validate_profile(profile)
        admission = admission_module.evaluate(
            profile, "pull_request", declared_changed_files, file_records
        )
    except Exception as exc:
        return (
            outcome(
                "TrustedAdmissionIndeterminate",
                f"trusted admission evaluation failed: {type(exc).__name__}",
                subject_sha=subject,
                publish=True,
                exit_code=2,
            ),
            None,
        )

    required_ids = tuple(profile.get("required_jobs") or ())
    info_ids = tuple(profile.get("informational_jobs") or ())
    try:
        require(required_ids, "required_jobs empty")
        require(set(required_ids).isdisjoint(info_ids), "required/informational overlap")
        require(
            set(required_ids) | set(info_ids) <= set(JOB_DISPLAY),
            "authority job display map incomplete",
        )
    except AuthorityError as exc:
        return (
            outcome(
                "TrustedAuthorityProfileInvalid",
                str(exc),
                subject_sha=subject,
                publish=True,
                exit_code=2,
            ),
            admission,
        )

    selected = set(admission.jobs)
    expected_required_ids = tuple(j for j in required_ids if j in selected)
    expected_info_ids = tuple(j for j in info_ids if j in selected)
    expected_required = tuple(JOB_DISPLAY[j] for j in expected_required_ids)
    expected_info = tuple(JOB_DISPLAY[j] for j in expected_info_ids)

    if admission.disposition == "AdmissionSkipped":
        require(not selected, "AdmissionSkipped unexpectedly selected jobs")
        return (
            outcome(
                "TrustedGenericCiNotRequired",
                "all current/previous PR paths are proven irrelevant to generic CI",
                subject_sha=subject,
                publish=True,
                exit_code=0,
            ),
            admission,
        )

    if not expected_required:
        return (
            outcome(
                "TrustedAuthorityProfileInvalid",
                f"{admission.disposition} selected no required jobs",
                subject_sha=subject,
                publish=True,
                exit_code=2,
                expected_required=expected_required,
                expected_info=expected_info,
            ),
            admission,
        )

    try:
        observed = normalize_jobs(jobs)
    except AuthorityError as exc:
        return (
            outcome(
                "TrustedJobsObservationIndeterminate",
                str(exc),
                subject_sha=subject,
                publish=True,
                exit_code=2,
                expected_required=expected_required,
                expected_info=expected_info,
            ),
            admission,
        )

    observed_required: dict[str, str] = {}
    observed_info: dict[str, str] = {}

    for name in expected_required:
        conclusion = observed.get(name)
        observed_required[name] = conclusion or "missing"
        if conclusion is None:
            return (
                outcome(
                    "TrustedRequiredJobMissing",
                    f"required job {name!r} missing",
                    subject_sha=subject,
                    publish=True,
                    exit_code=1,
                    expected_required=expected_required,
                    expected_info=expected_info,
                    observed_required=observed_required,
                    observed_info=observed_info,
                ),
                admission,
            )
        if conclusion != "success":
            disposition = (
                "TrustedRunCancelledOrSuperseded"
                if conclusion in {"cancelled", "stale"}
                else "TrustedRequiredJobNotSuccessful"
            )
            return (
                outcome(
                    disposition,
                    f"required job {name!r} concluded {conclusion!r}",
                    subject_sha=subject,
                    publish=True,
                    exit_code=1,
                    expected_required=expected_required,
                    expected_info=expected_info,
                    observed_required=observed_required,
                    observed_info=observed_info,
                ),
                admission,
            )

    for name in expected_info:
        observed_info[name] = observed.get(name) or "missing"

    selected_display = set(expected_required) | set(expected_info)
    control_names = {"changes", "ci-pass"}
    known_display = set(JOB_DISPLAY.values()) | control_names
    drift: list[str] = []
    for name, conclusion in observed.items():
        if name in known_display and name not in selected_display and name not in control_names:
            if conclusion != "skipped":
                drift.append(f"extra execution: {name}={conclusion}")
        elif name not in known_display:
            drift.append(f"unexpected job: {name}={conclusion}")

    for name, conclusion in observed_info.items():
        if conclusion != "success":
            drift.append(f"informational observation: {name}={conclusion}")

    return (
        outcome(
            "TrustedRequiredChecksPassed",
            "every independently expected merge-gating job completed successfully",
            subject_sha=subject,
            publish=True,
            exit_code=0,
            expected_required=expected_required,
            expected_info=expected_info,
            observed_required=observed_required,
            observed_info=observed_info,
            drift=tuple(sorted(set(drift))),
        ),
        admission,
    )


def self_test(admission_module: Any, profile: dict[str, Any]) -> None:
    subject_sha = "a" * 40
    run = {
        "workflow_name": EXPECTED_WORKFLOW_NAME,
        "workflow_path": EXPECTED_WORKFLOW_PATH,
        "event": "pull_request",
        "run_id": 123,
        "run_attempt": 1,
        "workflow_conclusion": "success",
        "pr_head_sha": subject_sha,
        "current_pr_head_sha": subject_sha,
    }

    def jobs_for(ids: list[str], *, extra: list[dict[str, str]] | None = None):
        observed = [
            {"name": JOB_DISPLAY[j], "status": "completed", "conclusion": "success"}
            for j in ids
        ]
        return observed + (extra or [])

    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "docs/lex-net/a.md"}],
        [],
    )
    assert result.disposition == "TrustedGenericCiNotRequired"
    assert result.publish_authority

    finance = ["format", "test-finance", "test-finance-integration"]
    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "mycelix-finance/src/lib.rs"}],
        jobs_for(finance),
    )
    assert result.disposition == "TrustedRequiredChecksPassed"

    required = list(profile["required_jobs"])
    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "future/runtime.rs"}],
        jobs_for(required),
    )
    assert result.disposition == "TrustedRequiredChecksPassed"
    assert JOB_DISPLAY["test-finance-integration"] not in result.expected_informational_jobs

    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "future/runtime.rs"}],
        jobs_for(required[:-1]),
    )
    assert result.disposition == "TrustedRequiredJobMissing"

    failed = jobs_for(required)
    failed[0]["conclusion"] = "failure"
    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "future/runtime.rs"}],
        failed,
    )
    assert result.disposition == "TrustedRequiredJobNotSuccessful"

    duplicate = jobs_for(required) + [jobs_for(required)[0]]
    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "future/runtime.rs"}],
        duplicate,
    )
    assert result.disposition == "TrustedJobsObservationIndeterminate"

    extra = [
        {"name": JOB_DISPLAY["test-sdk"], "status": "completed", "conclusion": "failure"}
    ]
    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "mycelix-finance/src/lib.rs"}],
        jobs_for(finance, extra=extra),
    )
    assert result.disposition == "TrustedRequiredChecksPassed"
    assert result.execution_drift

    informational_red = jobs_for(finance)
    for item in informational_red:
        if item["name"] == JOB_DISPLAY["test-finance-integration"]:
            item["conclusion"] = "failure"
    result, _ = evaluate(
        admission_module,
        profile,
        run,
        1,
        [{"filename": "mycelix-finance/src/lib.rs"}],
        informational_red,
    )
    assert result.disposition == "TrustedRequiredChecksPassed"
    assert result.execution_drift

    stale = dict(run)
    stale["current_pr_head_sha"] = "b" * 40
    result, _ = evaluate(
        admission_module,
        profile,
        stale,
        1,
        [{"filename": "future/runtime.rs"}],
        jobs_for(required),
    )
    assert result.disposition == "TrustedStaleSubject"
    assert not result.publish_authority

    wrong = dict(run)
    wrong["workflow_name"] = "Fake CI"
    result, _ = evaluate(
        admission_module,
        profile,
        wrong,
        1,
        [{"filename": "future/runtime.rs"}],
        jobs_for(required),
    )
    assert result.disposition == "TrustedAuthorityInputInvalid"
    assert not result.publish_authority


def read_json_arg(value: str) -> Any:
    if value.startswith("@"):
        return json.loads(Path(value[1:]).read_text(encoding="utf-8"))
    return json.loads(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="docs/ci/generic_ci_admission_v2.json")
    parser.add_argument(
        "--admission-evaluator", default="scripts/evaluate_generic_ci_admission_v2.py"
    )
    parser.add_argument("--run-json")
    parser.add_argument("--declared-changed-files", type=int)
    parser.add_argument("--files-json")
    parser.add_argument("--jobs-json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
        admission_module = load_admission_module(Path(args.admission_evaluator))
        if args.self_test:
            self_test(admission_module, profile)
            print(
                json.dumps(
                    {
                        "authority_id": AUTHORITY_ID,
                        "self_test": "PASS",
                        "authority": AUTHORITY,
                        "grants_product_qualification": False,
                    },
                    sort_keys=True,
                )
            )
            return 0

        require(args.run_json is not None, "--run-json required")
        require(args.files_json is not None, "--files-json required")
        require(args.jobs_json is not None, "--jobs-json required")
        run = read_json_arg(args.run_json)
        files = read_json_arg(args.files_json)
        jobs = read_json_arg(args.jobs_json)
        result, admission = evaluate(
            admission_module,
            profile,
            run,
            args.declared_changed_files,
            files,
            jobs,
        )
        print(json.dumps(result.receipt(run, admission), sort_keys=True))
        return result.exit_code
    except (AuthorityError, json.JSONDecodeError, OSError) as exc:
        print(
            json.dumps(
                {
                    "authority_id": AUTHORITY_ID,
                    "disposition": "TrustedAuthorityInputInvalid",
                    "reason": str(exc),
                    "publish_authority": False,
                    "trusted_required_checks_passed": False,
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
