#!/usr/bin/env python3
"""Trusted aggregate algebra for the ruleset-required generic CI workflow."""
from __future__ import annotations

import argparse
import copy
import json
import os
import re
from dataclasses import dataclass
from typing import Any

SUMMARY_ID = "ruleset-generic-ci-summary-v1"
AUTHORITY = "MergeAuthorityCandidate"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")

REQUIRED = {
    "format": "format",
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
INFORMATIONAL = {"finance_integration": "test-finance-integration"}
SELECTORS = tuple(REQUIRED) + tuple(INFORMATIONAL)
KNOWN_RESULTS = {"success", "failure", "cancelled", "skipped"}
PULL_ADMITTED = {"AdmitKnownRelevant", "AdmitUnknown"}


@dataclass(frozen=True)
class Evaluation:
    disposition: str
    reason: str
    exit_code: int
    workflow_sha: str | None
    target_sha: str | None
    required_results: dict[str, str]
    informational_results: dict[str, str]

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
            "merge_authority_passed": self.disposition
            in {"TrustedRequiredChecksPassed", "TrustedGenericCiNotRequired"},
            "grants_product_qualification": False,
        }


def _sha(value: Any) -> str | None:
    return value if isinstance(value, str) and SHA_RE.fullmatch(value) else None


def _job(needs: dict[str, Any], name: str) -> dict[str, Any] | None:
    value = needs.get(name)
    return value if isinstance(value, dict) else None


def _result(needs: dict[str, Any], name: str) -> str | None:
    job = _job(needs, name)
    value = job.get("result") if job else None
    return value if isinstance(value, str) else None


def _selector(outputs: dict[str, Any], key: str) -> bool | None:
    value = outputs.get(key)
    if value == "true":
        return True
    if value == "false":
        return False
    return None


def _fail(
    disposition: str,
    reason: str,
    workflow_sha: str | None,
    target_sha: str | None,
    required: dict[str, str],
    info: dict[str, str],
    exit_code: int = 2,
) -> Evaluation:
    return Evaluation(
        disposition,
        reason,
        exit_code,
        workflow_sha,
        target_sha,
        required,
        info,
    )


def evaluate(
    event_name: str,
    needs: dict[str, Any],
    *,
    workflow_sha: Any,
    target_sha: Any,
) -> Evaluation:
    required: dict[str, str] = {}
    info: dict[str, str] = {}
    trusted_workflow_sha = _sha(workflow_sha)
    tested_target_sha = _sha(target_sha)
    if trusted_workflow_sha is None or tested_target_sha is None:
        return _fail(
            "TrustedSubjectBindingInvalid",
            "workflow_sha/target_sha must both be lowercase 40-hex",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )
    if event_name not in {"pull_request", "merge_group"}:
        return _fail(
            "TrustedSubjectBindingInvalid",
            f"unsupported authority event {event_name!r}",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )

    changes_result = _result(needs, "changes")
    required["changes"] = changes_result or "missing"
    if changes_result is None:
        return _fail(
            "AuthorityRootMissing",
            "trusted changes/admission root has no result",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )
    if changes_result == "cancelled":
        return _fail(
            "TrustedRunCancelled",
            "trusted changes/admission root was cancelled",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )
    if changes_result != "success":
        return _fail(
            "AuthorityRootFailed",
            f"trusted changes/admission root concluded {changes_result!r}",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
            1 if changes_result == "failure" else 2,
        )

    changes = _job(needs, "changes") or {}
    outputs = changes.get("outputs")
    if not isinstance(outputs, dict):
        return _fail(
            "AuthorityRootMissing",
            "trusted changes/admission root has no outputs",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )

    disposition = outputs.get("disposition")
    authority_complete = outputs.get("authority_complete")
    generic_required = outputs.get("generic_ci_required")
    if authority_complete not in {"true", "false"}:
        return _fail(
            "AuthorityRootIndeterminate",
            f"authority_complete invalid {authority_complete!r}",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )
    if generic_required not in {"true", "false"}:
        return _fail(
            "AuthorityRootIndeterminate",
            f"generic_ci_required invalid {generic_required!r}",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )

    selected = {key: _selector(outputs, key) for key in SELECTORS}
    bad = [key for key, value in selected.items() if value is None]
    if bad:
        return _fail(
            "AuthorityRootIndeterminate",
            f"missing/invalid selectors: {bad}",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )

    if authority_complete != "true":
        return _fail(
            "AuthorityInputsIncomplete",
            f"router disposition {disposition!r} is not complete enough for merge authority",
            trusted_workflow_sha,
            tested_target_sha,
            required,
            info,
        )

    if event_name == "merge_group":
        if disposition != "AdmitMergeGroupRequired" or generic_required != "true":
            return _fail(
                "AuthorityGraphDrift",
                "merge_group requires AdmitMergeGroupRequired + generic_ci_required=true",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if not all(selected[key] is True for key in REQUIRED):
            return _fail(
                "AuthorityGraphDrift",
                "merge_group must select every required job",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if any(selected[key] is True for key in INFORMATIONAL):
            return _fail(
                "AuthorityGraphDrift",
                "merge_group informational jobs are not part of required fanout v1",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
    else:
        if disposition == "GenericCiNotRequired":
            if generic_required != "false" or any(selected.values()):
                return _fail(
                    "AuthorityGraphDrift",
                    "GenericCiNotRequired must select zero product jobs",
                    trusted_workflow_sha,
                    tested_target_sha,
                    required,
                    info,
                )
            for key, name in REQUIRED.items():
                value = _result(needs, name)
                required[name] = value or "missing"
                if value != "skipped":
                    return _fail(
                        "AuthorityGraphDrift",
                        f"non-required job {name!r} should be skipped, got {value!r}",
                        trusted_workflow_sha,
                        tested_target_sha,
                        required,
                        info,
                    )
            for key, name in INFORMATIONAL.items():
                value = _result(needs, name)
                info[name] = value or "missing"
                if value != "skipped":
                    return _fail(
                        "AuthorityGraphDrift",
                        f"non-required informational job {name!r} should be skipped, got {value!r}",
                        trusted_workflow_sha,
                        tested_target_sha,
                        required,
                        info,
                    )
            return Evaluation(
                "TrustedGenericCiNotRequired",
                "trusted complete admission proved generic product CI irrelevant",
                0,
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if disposition not in PULL_ADMITTED or generic_required != "true":
            return _fail(
                "AuthorityGraphDrift",
                f"unexpected complete pull_request disposition {disposition!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )

    for key, name in REQUIRED.items():
        value = _result(needs, name)
        required[name] = value or "missing"
        if value is None:
            return _fail(
                "RequiredJobMissing",
                f"required graph job {name!r} has no result",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if value not in KNOWN_RESULTS:
            return _fail(
                "RequiredJobIndeterminate",
                f"required graph job {name!r} has unknown result {value!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if selected[key]:
            if value == "success":
                continue
            if value == "cancelled":
                return _fail(
                    "TrustedRunCancelled",
                    f"selected required job {name!r} was cancelled",
                    trusted_workflow_sha,
                    tested_target_sha,
                    required,
                    info,
                )
            return _fail(
                "RequiredJobNotSuccessful",
                f"selected required job {name!r} concluded {value!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
                1 if value == "failure" else 2,
            )
        if value != "skipped":
            return _fail(
                "AuthorityGraphDrift",
                f"unselected required job {name!r} executed as {value!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )

    for key, name in INFORMATIONAL.items():
        value = _result(needs, name)
        info[name] = value or "missing"
        if value is None or value not in KNOWN_RESULTS:
            return _fail(
                "InformationalGraphIndeterminate",
                f"informational job {name!r} has invalid result {value!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if selected[key]:
            if value in {"success", "failure"}:
                continue
            return _fail(
                "InformationalSelectionDrift",
                f"selected informational job {name!r} did not execute to success/failure: {value!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )
        if value != "skipped":
            return _fail(
                "InformationalSelectionDrift",
                f"unselected informational job {name!r} executed as {value!r}",
                trusted_workflow_sha,
                tested_target_sha,
                required,
                info,
            )

    return Evaluation(
        "TrustedRequiredChecksPassed",
        "all selected merge-gating required jobs passed under complete trusted authority inputs",
        0,
        trusted_workflow_sha,
        tested_target_sha,
        required,
        info,
    )


def fixture(
    *,
    event_name: str = "pull_request",
    disposition: str = "AdmitKnownRelevant",
    authority_complete: bool = True,
    generic_required: bool = True,
    selected: set[str] | None = None,
    informational_result: str = "success",
) -> dict[str, Any]:
    selected = selected or set()
    outputs = {
        "disposition": disposition,
        "authority_complete": "true" if authority_complete else "false",
        "generic_ci_required": "true" if generic_required else "false",
    }
    for key in SELECTORS:
        outputs[key] = "true" if key in selected else "false"
    needs: dict[str, Any] = {
        "changes": {"result": "success", "outputs": outputs}
    }
    for key, name in REQUIRED.items():
        needs[name] = {
            "result": "success" if key in selected else "skipped",
            "outputs": {},
        }
    for key, name in INFORMATIONAL.items():
        needs[name] = {
            "result": informational_result if key in selected else "skipped",
            "outputs": {},
        }
    return needs


def self_test() -> None:
    w = "a" * 40
    t = "b" * 40

    docs = fixture(
        disposition="GenericCiNotRequired",
        generic_required=False,
        selected=set(),
    )
    assert evaluate("pull_request", docs, workflow_sha=w, target_sha=t).disposition == "TrustedGenericCiNotRequired"

    finance = fixture(selected={"format", "finance", "finance_integration"}, informational_result="failure")
    assert evaluate("pull_request", finance, workflow_sha=w, target_sha=t).disposition == "TrustedRequiredChecksPassed"

    all_required = set(REQUIRED)
    unknown = fixture(disposition="AdmitUnknown", selected=all_required)
    assert evaluate("pull_request", unknown, workflow_sha=w, target_sha=t).disposition == "TrustedRequiredChecksPassed"

    incomplete = fixture(
        disposition="AuthorityFilesIndeterminate",
        authority_complete=False,
        selected=all_required,
    )
    assert evaluate("pull_request", incomplete, workflow_sha=w, target_sha=t).disposition == "AuthorityInputsIncomplete"

    merge = fixture(
        event_name="merge_group",
        disposition="AdmitMergeGroupRequired",
        selected=all_required,
    )
    assert evaluate("merge_group", merge, workflow_sha=w, target_sha=t).disposition == "TrustedRequiredChecksPassed"

    failed = copy.deepcopy(finance)
    failed["test-finance"]["result"] = "failure"
    assert evaluate("pull_request", failed, workflow_sha=w, target_sha=t).disposition == "RequiredJobNotSuccessful"

    drift = fixture(selected={"finance"})
    drift["test-sdk"]["result"] = "success"
    assert evaluate("pull_request", drift, workflow_sha=w, target_sha=t).disposition == "AuthorityGraphDrift"

    info_skip = fixture(selected={"finance", "finance_integration"})
    info_skip["test-finance-integration"]["result"] = "skipped"
    assert evaluate("pull_request", info_skip, workflow_sha=w, target_sha=t).disposition == "InformationalSelectionDrift"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--event-name", default=os.getenv("GITHUB_EVENT_NAME", ""))
    parser.add_argument("--needs-json", default=os.getenv("MYCELIX_CI_NEEDS_JSON", ""))
    parser.add_argument("--workflow-sha", default=os.getenv("GITHUB_WORKFLOW_SHA", ""))
    parser.add_argument("--target-sha", default=os.getenv("GITHUB_SHA", ""))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
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
        print(
            json.dumps(
                {
                    "summary_id": SUMMARY_ID,
                    "disposition": "AuthorityRootMissing",
                    "merge_authority_passed": False,
                    "reason": "needs JSON empty",
                    "grants_product_qualification": False,
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
                    "summary_id": SUMMARY_ID,
                    "disposition": "AuthorityRootIndeterminate",
                    "merge_authority_passed": False,
                    "reason": str(exc),
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2
    if not isinstance(needs, dict):
        print(
            json.dumps(
                {
                    "summary_id": SUMMARY_ID,
                    "disposition": "AuthorityRootIndeterminate",
                    "merge_authority_passed": False,
                    "reason": "needs root must be object",
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2

    result = evaluate(
        args.event_name,
        needs,
        workflow_sha=args.workflow_sha,
        target_sha=args.target_sha,
    )
    print(json.dumps(result.receipt(), sort_keys=True))
    return result.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
