#!/usr/bin/env python3
"""Ruleset generic-CI summary v2: v1 authority + typed informational observation."""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SUMMARY_ID = "ruleset-generic-ci-summary-v2"
AUTHORITY = "MergeAuthorityCandidate"
INFO_JOB = "test-finance-integration"
INFO_SELECTOR = "finance_integration"
ALLOWED_INFO_OBSERVATIONS = {"success", "failure"}


def load_v1(path: Path):
    spec = importlib.util.spec_from_file_location("_ruleset_summary_v1", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class Evaluation:
    disposition: str
    reason: str
    exit_code: int
    workflow_sha: str | None
    target_sha: str | None
    required_results: dict[str, str]
    informational_results: dict[str, str]
    informational_observations: dict[str, str]

    def receipt(self) -> dict[str, Any]:
        return {
            "summary_id": SUMMARY_ID,
            "authority": AUTHORITY,
            "disposition": self.disposition,
            "reason": self.reason,
            "workflow_sha": self.workflow_sha,
            "target_sha": self.target_sha,
            "required_results": self.required_results,
            "informational_results": self.informational_results,
            "informational_observations": self.informational_observations,
            "merge_authority_passed": self.disposition
            in {"TrustedRequiredChecksPassed", "TrustedGenericCiNotRequired"},
            "grants_product_qualification": False,
        }


def _from_v1(result: Any, observations: dict[str, str] | None = None) -> Evaluation:
    return Evaluation(
        result.disposition,
        result.reason,
        result.exit_code,
        result.workflow_sha,
        result.target_sha,
        dict(result.required_results),
        dict(result.informational_results),
        observations or {},
    )


def _fail_from_v1(result: Any, disposition: str, reason: str) -> Evaluation:
    return Evaluation(
        disposition,
        reason,
        2,
        result.workflow_sha,
        result.target_sha,
        dict(result.required_results),
        dict(result.informational_results),
        {},
    )


def evaluate(
    v1: Any,
    event_name: str,
    needs: dict[str, Any],
    *,
    workflow_sha: Any,
    target_sha: Any,
) -> Evaluation:
    base = v1.evaluate(
        event_name,
        needs,
        workflow_sha=workflow_sha,
        target_sha=target_sha,
    )
    if base.exit_code != 0:
        return _from_v1(base)

    changes = needs.get("changes")
    if not isinstance(changes, dict) or not isinstance(changes.get("outputs"), dict):
        return _fail_from_v1(
            base,
            "InformationalObservationIndeterminate",
            "trusted changes outputs unavailable after v1 PASS",
        )
    selected = changes["outputs"].get(INFO_SELECTOR) == "true"
    job = needs.get(INFO_JOB)
    if not isinstance(job, dict):
        return _fail_from_v1(
            base,
            "InformationalObservationIndeterminate",
            "Finance Integration job object missing after v1 PASS",
        )

    outputs = job.get("outputs")
    if selected:
        if not isinstance(outputs, dict):
            return _fail_from_v1(
                base,
                "InformationalObservationIndeterminate",
                "selected Finance Integration job has no outputs object",
            )
        observation = outputs.get("observation")
        if observation not in ALLOWED_INFO_OBSERVATIONS:
            return _fail_from_v1(
                base,
                "InformationalObservationIndeterminate",
                f"selected Finance Integration observation invalid {observation!r}",
            )
        return _from_v1(base, {INFO_JOB: observation})

    if isinstance(outputs, dict) and outputs.get("observation") not in {None, ""}:
        return _fail_from_v1(
            base,
            "InformationalObservationDrift",
            "unselected Finance Integration unexpectedly exported an observation",
        )
    return _from_v1(base)


def self_test(v1: Any) -> None:
    w = "a" * 40
    t = "b" * 40

    finance = v1.fixture(
        selected={"format", "finance", "finance_integration"},
        informational_result="success",
    )
    finance[INFO_JOB]["outputs"] = {"observation": "failure"}
    result = evaluate(v1, "pull_request", finance, workflow_sha=w, target_sha=t)
    assert result.disposition == "TrustedRequiredChecksPassed"
    assert result.informational_observations[INFO_JOB] == "failure"

    missing = v1.fixture(
        selected={"format", "finance", "finance_integration"},
        informational_result="success",
    )
    result = evaluate(v1, "pull_request", missing, workflow_sha=w, target_sha=t)
    assert result.disposition == "InformationalObservationIndeterminate"

    success = v1.fixture(
        selected={"format", "finance", "finance_integration"},
        informational_result="success",
    )
    success[INFO_JOB]["outputs"] = {"observation": "success"}
    result = evaluate(v1, "pull_request", success, workflow_sha=w, target_sha=t)
    assert result.disposition == "TrustedRequiredChecksPassed"
    assert result.informational_observations[INFO_JOB] == "success"

    docs = v1.fixture(
        disposition="GenericCiNotRequired",
        generic_required=False,
        selected=set(),
    )
    result = evaluate(v1, "pull_request", docs, workflow_sha=w, target_sha=t)
    assert result.disposition == "TrustedGenericCiNotRequired"
    assert not result.informational_observations

    drift = v1.fixture(selected={"finance"})
    drift[INFO_JOB]["outputs"] = {"observation": "failure"}
    result = evaluate(v1, "pull_request", drift, workflow_sha=w, target_sha=t)
    assert result.disposition == "InformationalObservationDrift"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--v1", default="scripts/evaluate_ruleset_generic_ci_summary_v1.py"
    )
    parser.add_argument("--event-name", default=os.getenv("GITHUB_EVENT_NAME", ""))
    parser.add_argument("--needs-json", default=os.getenv("MYCELIX_CI_NEEDS_JSON", ""))
    parser.add_argument("--workflow-sha", default=os.getenv("GITHUB_WORKFLOW_SHA", ""))
    parser.add_argument("--target-sha", default=os.getenv("GITHUB_SHA", ""))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        v1 = load_v1(Path(args.v1))
        if args.self_test:
            self_test(v1)
            print(
                json.dumps(
                    {
                        "summary_id": SUMMARY_ID,
                        "self_test": "PASS",
                        "authority": AUTHORITY,
                        "grants_product_qualification": False,
                    },
                    sort_keys=True,
                )
            )
            return 0

        if not args.needs_json:
            raise ValueError("needs JSON empty")
        needs = json.loads(args.needs_json)
        if not isinstance(needs, dict):
            raise ValueError("needs root must be object")
        result = evaluate(
            v1,
            args.event_name,
            needs,
            workflow_sha=args.workflow_sha,
            target_sha=args.target_sha,
        )
        print(json.dumps(result.receipt(), sort_keys=True))
        return result.exit_code
    except (OSError, json.JSONDecodeError, ValueError, RuntimeError) as exc:
        print(
            json.dumps(
                {
                    "summary_id": SUMMARY_ID,
                    "disposition": "TrustedSummaryInvalid",
                    "reason": str(exc),
                    "merge_authority_passed": False,
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
