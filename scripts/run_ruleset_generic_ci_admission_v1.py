#!/usr/bin/env python3
"""Trusted admission router for the ruleset-required generic CI authority.

The router separates runner selection from merge-authority completeness.

- pull_request + complete bounded files observation: use admission-v2 closure.
- pull_request + incomplete/unobservable files: execute every required job, but
  mark authority_complete=false so a summary can never convert it into PASS.
- merge_group: execute every required job; no path-closure optimization.

Authority code is expected to execute from github.workflow_sha. Product jobs
execute github.sha separately.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROUTER_ID = "ruleset-generic-ci-admission-router-v1"
AUTHORITY = "RulesetSchedulingAuthority"
MANIFEST_ID = "ruleset-generic-ci-authority-v1"
PROFILE_ID = "generic-mycelix-ci-admission-v2"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")

OUTPUT_KEYS = {
    "format": "format",
    "test-commons": "commons",
    "test-civic": "civic",
    "test-hearth": "hearth",
    "test-finance": "finance",
    "test-finance-integration": "finance_integration",
    "test-governance": "governance",
    "test-identity": "identity",
    "test-personal": "personal",
    "test-attribution": "attribution",
    "test-bridge": "bridge",
    "test-sdk": "sdk",
    "test-prism": "prism",
}


class RouterError(ValueError):
    pass


def require(ok: bool, message: str) -> None:
    if not ok:
        raise RouterError(message)


def sha40(value: Any, label: str) -> str:
    require(
        isinstance(value, str) and SHA_RE.fullmatch(value) is not None,
        f"{label} must be lowercase 40-hex SHA",
    )
    return value


def positive_int(value: Any, label: str) -> int:
    require(
        isinstance(value, int) and not isinstance(value, bool) and value > 0,
        f"{label} must be positive integer",
    )
    return value


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    require(spec is not None and spec.loader is not None, f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def validate_manifest(manifest: Any, profile: dict[str, Any]) -> None:
    require(isinstance(manifest, dict), "manifest root must be object")
    require(manifest.get("authority_id") == MANIFEST_ID, "manifest identity drift")
    require(manifest.get("issue") == 1685, "manifest issue drift")
    require(manifest.get("authority") == "MergeAuthorityCandidate", "manifest authority drift")
    require(
        manifest.get("source_branch_candidate") == "ci-authority/v1-candidate",
        "candidate source branch drift",
    )
    require(manifest.get("promotion_target") == "ci-authority/v1", "promotion target drift")
    sha40(manifest.get("source_parent"), "source_parent")

    runner = manifest.get("runner")
    require(
        isinstance(runner, dict)
        and runner.get("scheduler") == "ubuntu-24.04"
        and runner.get("product") == "ubuntu-24.04",
        "runner family drift",
    )
    require(manifest.get("events") == ["pull_request", "merge_group"], "event set drift")

    actions = manifest.get("external_actions")
    require(isinstance(actions, dict) and actions, "external action lock missing")
    for action, commit in actions.items():
        require(isinstance(action, str) and action.count("/") == 1, "invalid action identity")
        sha40(commit, f"external action {action}")

    required_jobs = manifest.get("required_jobs")
    informational_jobs = manifest.get("informational_jobs")
    require(
        isinstance(required_jobs, list)
        and required_jobs
        and len(required_jobs) == len(set(required_jobs)),
        "required_jobs invalid",
    )
    require(
        isinstance(informational_jobs, list)
        and len(informational_jobs) == len(set(informational_jobs)),
        "informational_jobs invalid",
    )
    require(set(required_jobs).isdisjoint(informational_jobs), "required/informational overlap")
    require(
        set(required_jobs) == set(profile.get("required_jobs") or []),
        "manifest/profile required-job drift",
    )
    require(
        set(informational_jobs) == set(profile.get("informational_jobs") or []),
        "manifest/profile informational-job drift",
    )
    require(
        set(required_jobs) | set(informational_jobs) <= set(OUTPUT_KEYS),
        "router output map incomplete",
    )

    trust = manifest.get("trust_model")
    require(isinstance(trust, dict), "trust_model missing")
    require(trust.get("authority_checkout") == "github.workflow_sha", "authority checkout drift")
    require(trust.get("target_checkout") == "github.sha", "target checkout drift")
    require(trust.get("persist_credentials") is False, "checkout credentials must not persist")
    require(trust.get("candidate_policy_execution") is False, "candidate policy execution forbidden")
    require(trust.get("candidate_workflow_authority") is False, "candidate workflow authority forbidden")


def all_required(manifest: dict[str, Any]) -> tuple[str, ...]:
    return tuple(manifest["required_jobs"])


def all_jobs(manifest: dict[str, Any]) -> tuple[str, ...]:
    return tuple(manifest["required_jobs"] + manifest["informational_jobs"])


@dataclass(frozen=True)
class Decision:
    disposition: str
    authority_complete: bool
    generic_ci_required: bool
    selected_jobs: tuple[str, ...]
    declared_changed_files: int | None
    observed_file_records: int | None
    files_observable: bool | None
    reason: str

    def receipt(
        self,
        *,
        workflow_sha: str,
        target_sha: str,
        event_name: str,
        manifest: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "router_id": ROUTER_ID,
            "authority": AUTHORITY,
            "manifest_id": MANIFEST_ID,
            "profile_id": PROFILE_ID,
            "event_name": event_name,
            "workflow_sha": workflow_sha,
            "target_sha": target_sha,
            "disposition": self.disposition,
            "authority_complete": self.authority_complete,
            "generic_ci_required": self.generic_ci_required,
            "selected_jobs": list(self.selected_jobs),
            "declared_changed_files": self.declared_changed_files,
            "observed_file_records": self.observed_file_records,
            "files_observable": self.files_observable,
            "reason": self.reason,
            "required_jobs": list(manifest["required_jobs"]),
            "informational_jobs": list(manifest["informational_jobs"]),
            "grants_product_qualification": False,
        }


def decide(
    *,
    event_name: str,
    manifest: dict[str, Any],
    profile: dict[str, Any],
    admission_module: Any,
    declared_changed_files: int | None,
    file_records: Any,
    files_observable: bool | None,
    files_reason: str = "",
) -> Decision:
    validate_manifest(manifest, profile)
    admission_module.validate_profile(profile)

    if event_name == "merge_group":
        return Decision(
            "AdmitMergeGroupRequired",
            True,
            True,
            all_required(manifest),
            None,
            None,
            None,
            "merge_group executes every merge-gating required job; no PR-path optimization",
        )

    require(event_name == "pull_request", f"unsupported event {event_name!r}")
    require(
        isinstance(declared_changed_files, int)
        and not isinstance(declared_changed_files, bool)
        and declared_changed_files > 0,
        "pull_request declared_changed_files must be positive integer",
    )

    if files_observable is not True:
        return Decision(
            "AuthorityFilesIndeterminate",
            False,
            True,
            all_required(manifest),
            declared_changed_files,
            None if not isinstance(file_records, list) else len(file_records),
            False,
            files_reason or "PR-file observation incomplete; execute all required jobs but deny authority PASS",
        )

    if not isinstance(file_records, list):
        return Decision(
            "AuthorityFilesIndeterminate",
            False,
            True,
            all_required(manifest),
            declared_changed_files,
            None,
            False,
            "files_observable=true but file_records is not a list",
        )

    admission = admission_module.evaluate(
        profile, "pull_request", declared_changed_files, file_records
    )
    if admission.disposition in {"AdmissionErrorFailClosed", "AdmissionProfileInvalid"}:
        return Decision(
            "AuthorityAdmissionIndeterminate",
            False,
            True,
            all_required(manifest),
            declared_changed_files,
            len(file_records),
            True,
            f"trusted admission returned {admission.disposition}: {admission.reason}",
        )

    selected = tuple(admission.jobs)
    allowed = set(all_jobs(manifest))
    require(set(selected) <= allowed, "admission selected unknown job")

    if admission.disposition == "AdmissionSkipped":
        require(not selected, "AdmissionSkipped selected jobs")
        return Decision(
            "GenericCiNotRequired",
            True,
            False,
            (),
            declared_changed_files,
            len(file_records),
            True,
            admission.reason,
        )

    require(
        admission.disposition in {"AdmitKnownRelevant", "AdmitUnknown"},
        f"unexpected complete-observation admission {admission.disposition}",
    )
    required_selected = set(selected) & set(manifest["required_jobs"])
    require(required_selected, "admitted PR selected no required jobs")
    return Decision(
        admission.disposition,
        True,
        True,
        selected,
        declared_changed_files,
        len(file_records),
        True,
        admission.reason,
    )


def output_lines(decision: Decision, manifest: dict[str, Any]) -> list[str]:
    selected = set(decision.selected_jobs)
    lines = [
        f"disposition={decision.disposition}",
        f"authority_complete={'true' if decision.authority_complete else 'false'}",
        f"generic_ci_required={'true' if decision.generic_ci_required else 'false'}",
    ]
    for job_id, key in OUTPUT_KEYS.items():
        lines.append(f"{key}={'true' if job_id in selected else 'false'}")
    return lines


def _event_pull_request_metadata(event: Any) -> tuple[int, int]:
    require(isinstance(event, dict), "event root must be object")
    pull_request = event.get("pull_request")
    require(isinstance(pull_request, dict), "pull_request payload missing")
    number = positive_int(pull_request.get("number"), "pull_request.number")
    changed_files = positive_int(pull_request.get("changed_files"), "pull_request.changed_files")
    return number, changed_files


def observe_and_decide(
    *,
    event_name: str,
    event: Any,
    repository: str,
    token: str,
    manifest: dict[str, Any],
    profile: dict[str, Any],
    admission_module: Any,
    pr_files_module: Any,
) -> Decision:
    if event_name == "merge_group":
        return decide(
            event_name=event_name,
            manifest=manifest,
            profile=profile,
            admission_module=admission_module,
            declared_changed_files=None,
            file_records=None,
            files_observable=None,
        )

    number, changed_files = _event_pull_request_metadata(event)
    observation = pr_files_module.observe(
        repository=repository,
        pr_number=number,
        declared_changed_files=changed_files,
        token=token,
    )
    return decide(
        event_name=event_name,
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=changed_files,
        file_records=observation.records,
        files_observable=observation.observable,
        files_reason=observation.reason,
    )


def self_test(manifest: dict[str, Any], profile: dict[str, Any], admission_module: Any) -> None:
    merge = decide(
        event_name="merge_group",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=None,
        file_records=None,
        files_observable=None,
    )
    assert merge.authority_complete and set(merge.selected_jobs) == set(manifest["required_jobs"])
    assert "test-finance-integration" not in merge.selected_jobs

    docs = decide(
        event_name="pull_request",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=1,
        file_records=[{"filename": "docs/lex-net/a.md", "status": "modified"}],
        files_observable=True,
    )
    assert docs.disposition == "GenericCiNotRequired"
    assert docs.authority_complete and not docs.generic_ci_required and not docs.selected_jobs

    finance = decide(
        event_name="pull_request",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=1,
        file_records=[{"filename": "mycelix-finance/src/lib.rs", "status": "modified"}],
        files_observable=True,
    )
    assert finance.authority_complete
    assert set(finance.selected_jobs) == {"format", "test-finance", "test-finance-integration"}

    unknown = decide(
        event_name="pull_request",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=1,
        file_records=[{"filename": "future/runtime.rs", "status": "added"}],
        files_observable=True,
    )
    assert unknown.authority_complete
    assert set(unknown.selected_jobs) == set(manifest["required_jobs"])
    assert "test-finance-integration" not in unknown.selected_jobs

    unavailable = decide(
        event_name="pull_request",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=1,
        file_records=None,
        files_observable=False,
        files_reason="synthetic API outage",
    )
    assert unavailable.disposition == "AuthorityFilesIndeterminate"
    assert not unavailable.authority_complete
    assert set(unavailable.selected_jobs) == set(manifest["required_jobs"])

    renamed = decide(
        event_name="pull_request",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=1,
        file_records=[
            {
                "filename": "docs/lex-net/moved.md",
                "previous_filename": "mycelix-governance/src/lib.rs",
                "status": "renamed",
            }
        ],
        files_observable=True,
    )
    assert renamed.authority_complete
    assert set(renamed.selected_jobs) == {"format", "test-governance"}


def write_receipt(path: str | None, receipt: dict[str, Any]) -> None:
    if path:
        Path(path).write_text(json.dumps(receipt, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", default="docs/ci/ruleset_generic_ci_authority_v1.json")
    parser.add_argument("--profile", default="docs/ci/generic_ci_admission_v2.json")
    parser.add_argument("--admission-evaluator", default="scripts/evaluate_generic_ci_admission_v2.py")
    parser.add_argument("--pr-files-adapter", default="scripts/fetch_generic_ci_pr_files.py")
    parser.add_argument("--event-name", default=os.getenv("GITHUB_EVENT_NAME", ""))
    parser.add_argument("--event-json", default=os.getenv("GITHUB_EVENT_PATH", ""))
    parser.add_argument("--repository", default=os.getenv("GITHUB_REPOSITORY", ""))
    parser.add_argument("--token", default=os.getenv("GITHUB_TOKEN", ""))
    parser.add_argument("--workflow-sha", default=os.getenv("GITHUB_WORKFLOW_SHA", ""))
    parser.add_argument("--target-sha", default=os.getenv("GITHUB_SHA", ""))
    parser.add_argument("--receipt-output")
    parser.add_argument("--github-output", default=os.getenv("GITHUB_OUTPUT", ""))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
        profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
        admission_module = load_module(Path(args.admission_evaluator), "_ruleset_admission_v2")
        pr_files_module = load_module(Path(args.pr_files_adapter), "_ruleset_pr_files_v1")
        validate_manifest(manifest, profile)

        if args.self_test:
            self_test(manifest, profile, admission_module)
            print(
                json.dumps(
                    {
                        "router_id": ROUTER_ID,
                        "self_test": "PASS",
                        "authority": AUTHORITY,
                        "grants_product_qualification": False,
                    },
                    sort_keys=True,
                )
            )
            return 0

        workflow_sha = sha40(args.workflow_sha, "workflow_sha")
        target_sha = sha40(args.target_sha, "target_sha")
        require(args.event_name in manifest["events"], "event not authorized by manifest")
        require(args.event_json, "event JSON path missing")
        event = json.loads(Path(args.event_json).read_text(encoding="utf-8"))

        decision = observe_and_decide(
            event_name=args.event_name,
            event=event,
            repository=args.repository,
            token=args.token,
            manifest=manifest,
            profile=profile,
            admission_module=admission_module,
            pr_files_module=pr_files_module,
        )
        receipt = decision.receipt(
            workflow_sha=workflow_sha,
            target_sha=target_sha,
            event_name=args.event_name,
            manifest=manifest,
        )
        write_receipt(args.receipt_output, receipt)
        if args.github_output:
            with Path(args.github_output).open("a", encoding="utf-8") as handle:
                for line in output_lines(decision, manifest):
                    handle.write(line + "\n")
        print(json.dumps(receipt, sort_keys=True))
        return 0
    except (RouterError, OSError, json.JSONDecodeError) as exc:
        receipt = {
            "router_id": ROUTER_ID,
            "authority": AUTHORITY,
            "disposition": "AuthorityRouterInvalid",
            "authority_complete": False,
            "generic_ci_required": True,
            "selected_jobs": [],
            "reason": str(exc),
            "grants_product_qualification": False,
        }
        write_receipt(args.receipt_output, receipt)
        print(json.dumps(receipt, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
