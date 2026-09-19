#!/usr/bin/env python3
"""Fail-closed aggregate evaluator for the generic Mycelix CI workflow.

This script interprets the GitHub Actions ``needs`` object emitted by
``toJSON(needs)``. It distinguishes required-check success from product
failure, cancellation/supersession, missing prerequisites, and intentional
path-selection skips.

A zero exit status means only:

    all required checks passed for this frozen aggregate profile

It does not qualify product semantics or turn informational checks into gates.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from dataclasses import dataclass
from typing import Any

PROFILE_ID = "generic-mycelix-ci-required-checks-v1"

PRODUCTS: dict[str, str] = {
    "commons": "test-commons",
    "civic": "test-civic",
    "hearth": "test-hearth",
    "finance": "test-finance",
    "governance": "test-governance",
    "identity": "test-identity",
    "personal": "test-personal",
    "attribution": "test-attribution",
    "bridge": "test-bridge",
    "sdk": "test-sdk",
    "prism": "test-prism",
}
INFORMATIONAL: dict[str, str] = {
    "finance_integration": "test-finance-integration",
}
KNOWN_JOB_RESULTS = {"success", "failure", "cancelled", "skipped"}


@dataclass(frozen=True)
class Evaluation:
    disposition: str
    reason: str
    exit_code: int
    required_results: dict[str, str]
    informational_results: dict[str, str]

    def as_dict(self) -> dict[str, Any]:
        return {
            "profile_id": PROFILE_ID,
            "disposition": self.disposition,
            "reason": self.reason,
            "required_results": self.required_results,
            "informational_results": self.informational_results,
            "required_checks_passed": self.disposition == "RequiredChecksPassed",
        }


def _job(needs: dict[str, Any], name: str) -> dict[str, Any] | None:
    value = needs.get(name)
    return value if isinstance(value, dict) else None


def _result(needs: dict[str, Any], name: str) -> str | None:
    job = _job(needs, name)
    value = job.get("result") if job else None
    return value if isinstance(value, str) else None


def _fail(
    disposition: str,
    reason: str,
    required: dict[str, str],
    informational: dict[str, str],
    *,
    exit_code: int = 2,
) -> Evaluation:
    return Evaluation(disposition, reason, exit_code, required, informational)


def evaluate(event_name: str, needs: dict[str, Any]) -> Evaluation:
    required: dict[str, str] = {}
    informational: dict[str, str] = {}

    if event_name not in {"pull_request", "push"}:
        return _fail(
            "RequiredPrerequisiteIndeterminate",
            f"unsupported event_name={event_name!r}",
            required,
            informational,
        )

    # These roots must have executed successfully before any downstream skip
    # can be interpreted safely.
    for root in ("changes", "format"):
        result = _result(needs, root)
        required[root] = result or "missing"
        if result is None:
            return _fail(
                "RequiredPrerequisiteMissing",
                f"required root job {root!r} has no result",
                required,
                informational,
            )
        if result == "cancelled":
            return _fail(
                "RunCancelledOrSuperseded",
                f"required root job {root!r} was cancelled",
                required,
                informational,
            )
        if result == "failure":
            return _fail(
                "RequiredCheckFailed",
                f"required root job {root!r} failed",
                required,
                informational,
                exit_code=1,
            )
        if result != "success":
            return _fail(
                "RequiredPrerequisiteIndeterminate",
                f"required root job {root!r} has unexpected result {result!r}",
                required,
                informational,
            )

    changes = _job(needs, "changes") or {}
    outputs = changes.get("outputs")
    if not isinstance(outputs, dict):
        return _fail(
            "RequiredPrerequisiteMissing",
            "changes job has no outputs object",
            required,
            informational,
        )

    for product, job_name in PRODUCTS.items():
        result = _result(needs, job_name)
        required[job_name] = result or "missing"

        if result is None:
            return _fail(
                "RequiredPrerequisiteMissing",
                f"required job {job_name!r} has no result",
                required,
                informational,
            )
        if result not in KNOWN_JOB_RESULTS:
            return _fail(
                "RequiredPrerequisiteIndeterminate",
                f"required job {job_name!r} has unknown result {result!r}",
                required,
                informational,
            )

        if event_name == "push":
            selected = True
        else:
            raw_selected = outputs.get(product)
            if raw_selected not in {"true", "false"}:
                return _fail(
                    "RequiredPrerequisiteIndeterminate",
                    f"changes output {product!r} must be 'true' or 'false', got {raw_selected!r}",
                    required,
                    informational,
                )
            selected = raw_selected == "true"

        if selected:
            if result == "success":
                continue
            if result == "failure":
                return _fail(
                    "RequiredCheckFailed",
                    f"selected required job {job_name!r} failed",
                    required,
                    informational,
                    exit_code=1,
                )
            if result == "cancelled":
                return _fail(
                    "RunCancelledOrSuperseded",
                    f"selected required job {job_name!r} was cancelled",
                    required,
                    informational,
                )
            return _fail(
                "RequiredPrerequisiteIndeterminate",
                f"selected required job {job_name!r} was unexpectedly skipped",
                required,
                informational,
            )

        # On pull_request, an unselected product is allowed to be skipped only.
        if result != "skipped":
            return _fail(
                "RequiredPrerequisiteIndeterminate",
                f"unselected job {job_name!r} should be skipped, got {result!r}",
                required,
                informational,
            )

    for label, job_name in INFORMATIONAL.items():
        result = _result(needs, job_name)
        informational[label] = result or "missing"

    info_summary = ", ".join(
        f"{name}={state}" for name, state in sorted(informational.items())
    )
    return Evaluation(
        "RequiredChecksPassed",
        "all required checks passed for the frozen profile"
        + (f"; informational: {info_summary}" if info_summary else ""),
        0,
        required,
        informational,
    )


def _fixture(
    *,
    event_name: str = "pull_request",
    selected: set[str] | None = None,
) -> dict[str, Any]:
    selected = selected or set()
    needs: dict[str, Any] = {
        "changes": {
            "result": "success",
            "outputs": {
                product: ("true" if product in selected else "false")
                for product in PRODUCTS
            },
        },
        "format": {"result": "success", "outputs": {}},
    }
    for product, job_name in PRODUCTS.items():
        needs[job_name] = {
            "result": "success"
            if event_name == "push" or product in selected
            else "skipped",
            "outputs": {},
        }
    needs["test-finance-integration"] = {
        "result": "success"
        if event_name == "push" or "finance" in selected
        else "skipped",
        "outputs": {},
    }
    return needs


def self_test() -> None:
    # 1. Selected required jobs succeed; unselected jobs intentionally skip.
    needs = _fixture(selected={"commons", "finance"})
    assert evaluate("pull_request", needs).disposition == "RequiredChecksPassed"

    # 2. format failure.
    bad = copy.deepcopy(needs)
    bad["format"]["result"] = "failure"
    assert evaluate("pull_request", bad).disposition == "RequiredCheckFailed"

    # 3. format cancellation.
    bad = copy.deepcopy(needs)
    bad["format"]["result"] = "cancelled"
    assert evaluate("pull_request", bad).disposition == "RunCancelledOrSuperseded"

    # 4. changes cancellation with downstream skips.
    bad = copy.deepcopy(needs)
    bad["changes"]["result"] = "cancelled"
    assert evaluate("pull_request", bad).disposition == "RunCancelledOrSuperseded"

    # 5. No PR product paths selected.
    empty = _fixture()
    assert evaluate("pull_request", empty).disposition == "RequiredChecksPassed"

    # 6. Selected product failure.
    bad = copy.deepcopy(needs)
    bad["test-commons"]["result"] = "failure"
    assert evaluate("pull_request", bad).disposition == "RequiredCheckFailed"

    # 7. Selected product cancellation.
    bad = copy.deepcopy(needs)
    bad["test-commons"]["result"] = "cancelled"
    assert evaluate("pull_request", bad).disposition == "RunCancelledOrSuperseded"

    # 8. Selected product unexpectedly skipped.
    bad = copy.deepcopy(needs)
    bad["test-commons"]["result"] = "skipped"
    assert (
        evaluate("pull_request", bad).disposition
        == "RequiredPrerequisiteIndeterminate"
    )

    # 9. Unknown future result.
    bad = copy.deepcopy(needs)
    bad["test-commons"]["result"] = "neutral"
    assert (
        evaluate("pull_request", bad).disposition
        == "RequiredPrerequisiteIndeterminate"
    )

    # 10. Missing result.
    bad = copy.deepcopy(needs)
    del bad["test-commons"]["result"]
    assert evaluate("pull_request", bad).disposition == "RequiredPrerequisiteMissing"

    # 11. Informational failure does not fail required-check aggregate.
    bad = copy.deepcopy(needs)
    bad["test-finance-integration"]["result"] = "failure"
    outcome = evaluate("pull_request", bad)
    assert outcome.disposition == "RequiredChecksPassed"
    assert outcome.informational_results["finance_integration"] == "failure"

    # 12. Superseded/cancelled required root never becomes PASS.
    bad = _fixture(event_name="push")
    bad["changes"]["result"] = "cancelled"
    assert evaluate("push", bad).disposition == "RunCancelledOrSuperseded"

    # Extra: push requires every product job regardless of path outputs.
    push = _fixture(event_name="push")
    assert evaluate("push", push).disposition == "RequiredChecksPassed"
    push["test-sdk"]["result"] = "skipped"
    assert evaluate("push", push).disposition == "RequiredPrerequisiteIndeterminate"

    # Extra: an unselected PR job running unexpectedly is indeterminate, not PASS.
    bad = _fixture()
    bad["test-sdk"]["result"] = "success"
    assert (
        evaluate("pull_request", bad).disposition
        == "RequiredPrerequisiteIndeterminate"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--event-name",
        default=os.environ.get("GITHUB_EVENT_NAME", ""),
        help="GitHub event name; defaults to GITHUB_EVENT_NAME",
    )
    parser.add_argument(
        "--needs-json",
        default=os.environ.get("MYCELIX_CI_NEEDS_JSON", ""),
        help=(
            "serialized GitHub Actions needs object; defaults to "
            "MYCELIX_CI_NEEDS_JSON"
        ),
    )
    args = parser.parse_args()

    if args.self_test:
        self_test()
        print(json.dumps({"profile_id": PROFILE_ID, "self_test": "PASS"}, sort_keys=True))
        return 0

    if not args.needs_json:
        print(
            json.dumps(
                {
                    "profile_id": PROFILE_ID,
                    "disposition": "RequiredPrerequisiteMissing",
                    "reason": "MYCELIX_CI_NEEDS_JSON/--needs-json is empty",
                    "required_checks_passed": False,
                },
                sort_keys=True,
            )
        )
        return 2

    try:
        needs = json.loads(args.needs_json)
    except json.JSONDecodeError as exc:
        print(
            json.dumps(
                {
                    "profile_id": PROFILE_ID,
                    "disposition": "RequiredPrerequisiteIndeterminate",
                    "reason": f"invalid needs JSON: {exc}",
                    "required_checks_passed": False,
                },
                sort_keys=True,
            )
        )
        return 2

    if not isinstance(needs, dict):
        print(
            json.dumps(
                {
                    "profile_id": PROFILE_ID,
                    "disposition": "RequiredPrerequisiteIndeterminate",
                    "reason": "needs JSON root must be an object",
                    "required_checks_passed": False,
                },
                sort_keys=True,
            )
        )
        return 2

    outcome = evaluate(args.event_name, needs)
    print(json.dumps(outcome.as_dict(), sort_keys=True))
    return outcome.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
