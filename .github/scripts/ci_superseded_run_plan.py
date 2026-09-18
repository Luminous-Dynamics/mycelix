#!/usr/bin/env python3
"""Read-only superseded-run planner for CI-GOV-001D-A.

Uses only the qualified CI-GOV-001A GET client. Produces a deterministic plan
for queued pull-request runs from broad Mycelix CI that are provably
superseded across every associated open PR. It cannot mutate GitHub state.

V1 authority is intentionally fixed to `.github/workflows/ci.yml`; callers
cannot widen the workflow scope or override live observation time at runtime.
Only runs inside that fixed scope enter the committed plan envelope.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from ci_queue_census import GitHubReadOnlyClient

SCHEMA = "mycelix-ci-superseded-run-plan-v1"
ELIGIBLE_WORKFLOW_PATH = ".github/workflows/ci.yml"
MAX_PR_LOOKUPS = 500
PLAN_TTL_SECONDS = 900

ELIGIBLE = "SupersededHead"
CURRENT = "CurrentHead"
NO_PR = "NoPrAssociation"
NON_PR = "NonPullRequest"
OUT_OF_SCOPE = "WorkflowOutOfScope"
STATUS_OUT_OF_SCOPE = "StatusOutOfScope"
CLOSED = "ClosedPrAssociation"
UNKNOWN = "MetadataUnknown"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", default=os.environ.get("GITHUB_REPOSITORY", ""))
    parser.add_argument("--token-env", default="GITHUB_TOKEN")
    parser.add_argument("--json-output", type=Path)
    return parser.parse_args()


def association_numbers(run: dict[str, Any]) -> tuple[list[int], bool]:
    prs = run.get("pull_requests")
    if not isinstance(prs, list):
        return [], False
    numbers: set[int] = set()
    complete = True
    for item in prs:
        if not isinstance(item, dict) or not isinstance(item.get("number"), int):
            complete = False
            continue
        numbers.add(item["number"])
    return sorted(numbers), complete


def is_v1_scope(run: dict[str, Any]) -> bool:
    """True only for runs that are eligible to enter the v1 plan envelope."""
    return (
        run.get("path") == ELIGIBLE_WORKFLOW_PATH
        and run.get("event") == "pull_request"
        and run.get("status") == "queued"
    )


def runs_for_plan(queued_runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Return only runs inside the fixed v1 authority scope.

    Unrelated qualification/Forge/push/in-progress traffic is deliberately not
    committed into a broad-CI reclamation plan. This prevents unrelated queue
    activity from invalidating operator review of a plan that could never act
    on those runs. Malformed metadata on an otherwise in-scope run is retained
    and fails closed during classification.
    """
    return [run for run in queued_runs if is_v1_scope(run)]


def pr_numbers_for_lookup(queued_runs: list[dict[str, Any]]) -> list[int]:
    """Return PR numbers needed by v1 classification only.

    Out-of-scope workflows/events/statuses never consume the metadata budget.
    Malformed in-scope associations remain fail-closed during classification;
    any valid PR numbers beside malformed entries may still be looked up.
    """
    numbers: set[int] = set()
    for run in runs_for_plan(queued_runs):
        associated, _ = association_numbers(run)
        numbers.update(associated)
    return sorted(numbers)


def canonical_commitment(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def classify_run(
    run: dict[str, Any],
    pr_by_number: dict[int, dict[str, Any] | None],
    repository: str,
) -> dict[str, Any]:
    run_id = run.get("id")
    run_attempt = run.get("run_attempt")
    workflow_id = run.get("workflow_id")
    run_head = run.get("head_sha")
    created_at = run.get("created_at")
    event = run.get("event")
    status = run.get("status")
    path = run.get("path")
    numbers, associations_complete = association_numbers(run)
    associations: list[dict[str, Any]] = []

    classification = ELIGIBLE
    reason = "all associated open PR heads differ from queued run head"

    if path != ELIGIBLE_WORKFLOW_PATH:
        classification = OUT_OF_SCOPE
        reason = "workflow path is outside fixed v1 reclamation scope"
    elif event != "pull_request":
        classification = NON_PR
        reason = "event is not pull_request"
    elif status != "queued":
        classification = STATUS_OUT_OF_SCOPE
        reason = "v1 planner only considers queued runs"
    elif (
        not isinstance(run_id, int)
        or not isinstance(run_attempt, int)
        or run_attempt < 1
        or not isinstance(workflow_id, int)
        or not isinstance(run_head, str)
        or not run_head
        or not isinstance(created_at, str)
        or not created_at
        or not isinstance(repository, str)
        or "/" not in repository
    ):
        classification = UNKNOWN
        reason = "run/repository identity metadata is missing or malformed"
    elif not associations_complete:
        classification = UNKNOWN
        reason = "pull-request association metadata is malformed or incomplete"
    elif not numbers:
        classification = NO_PR
        reason = "run has no associated PR"
    else:
        has_unknown = False
        has_closed = False
        has_current = False

        for number in numbers:
            pr = pr_by_number.get(number)
            if not isinstance(pr, dict):
                associations.append(
                    {
                        "pr_number": number,
                        "state": "unknown",
                        "draft": None,
                        "current_head_sha": None,
                        "superseded": None,
                    }
                )
                has_unknown = True
                continue

            state = pr.get("state")
            draft = pr.get("draft")
            current_head = (pr.get("head") or {}).get("sha")
            superseded = (
                isinstance(current_head, str)
                and isinstance(run_head, str)
                and current_head != run_head
            )
            associations.append(
                {
                    "pr_number": number,
                    "state": state,
                    "draft": draft if isinstance(draft, bool) else None,
                    "current_head_sha": current_head if isinstance(current_head, str) else None,
                    "superseded": superseded if isinstance(current_head, str) else None,
                }
            )

            if not isinstance(current_head, str) or not current_head:
                has_unknown = True
            elif state != "open":
                has_closed = True
            elif current_head == run_head:
                has_current = True

        if has_unknown:
            classification = UNKNOWN
            reason = "one or more PR metadata lookups or current head SHAs are unresolved"
        elif has_closed:
            classification = CLOSED
            reason = "one or more associated PRs are not open"
        elif has_current:
            classification = CURRENT
            reason = "at least one associated open PR still points at queued run head"

    associations.sort(key=lambda item: item["pr_number"])

    claim = {
        "schema": SCHEMA,
        "repository": repository,
        "run_id": run_id,
        "run_attempt": run_attempt,
        "workflow_id": workflow_id,
        "workflow_name": run.get("name"),
        "workflow_path": path,
        "event": event,
        "status": status,
        "queued_head_sha": run_head,
        "created_at": created_at,
        "associations": associations,
        "classification": classification,
    }
    claim["plan_entry_commitment"] = canonical_commitment(claim)
    claim["reason"] = reason
    claim["eligible_for_cancellation_plan"] = classification == ELIGIBLE
    return claim


def build_plan(
    queued_runs: list[dict[str, Any]],
    pr_by_number: dict[int, dict[str, Any] | None],
    observed_at: datetime,
    repository: str,
) -> dict[str, Any]:
    observed_at = observed_at.astimezone(timezone.utc)
    expires_at = observed_at + timedelta(seconds=PLAN_TTL_SECONDS)

    scoped_runs = runs_for_plan(queued_runs)
    entries = [classify_run(run, pr_by_number, repository) for run in scoped_runs]
    entries.sort(
        key=lambda item: (str(item.get("created_at") or ""), int(item.get("run_id") or 0))
    )

    eligible = [entry for entry in entries if entry["eligible_for_cancellation_plan"]]
    body = {
        "schema": SCHEMA,
        "repository": repository,
        "observed_at": observed_at.isoformat().replace("+00:00", "Z"),
        "expires_at": expires_at.isoformat().replace("+00:00", "Z"),
        "scope": {
            "status": "queued",
            "event": "pull_request",
            "workflow_path": ELIGIBLE_WORKFLOW_PATH,
            "runtime_workflow_override_allowed": False,
            "runtime_time_override_allowed": False,
            "envelope_contains_only_in_scope_runs": True,
            "mutation_authority": False,
            "executor_requires_full_plan_envelope": True,
            "plan_ttl_seconds": PLAN_TTL_SECONDS,
        },
        "entry_count": len(entries),
        "eligible_count": len(eligible),
        "eligible_entries": eligible,
        "all_entries": entries,
    }
    commitment_payload = {
        "schema": body["schema"],
        "repository": body["repository"],
        "observed_at": body["observed_at"],
        "expires_at": body["expires_at"],
        "scope": body["scope"],
        "entry_count": body["entry_count"],
        "eligible_count": body["eligible_count"],
        "all_entry_commitments": [
            entry["plan_entry_commitment"] for entry in entries
        ],
        "eligible_entry_commitments": [
            entry["plan_entry_commitment"] for entry in eligible
        ],
    }
    body["plan_commitment"] = canonical_commitment(commitment_payload)
    return body


def render_markdown(plan: dict[str, Any]) -> str:
    lines = [
        "# Mycelix superseded-run reclamation plan",
        "",
        f"- Repository: `{plan['repository']}`",
        f"- Observed: `{plan['observed_at']}`",
        f"- Expires: `{plan['expires_at']}`",
        f"- Workflow scope: `{plan['scope']['workflow_path']}`",
        f"- In-scope queue entries inspected: **{plan['entry_count']}**",
        f"- Eligible superseded entries: **{plan['eligible_count']}**",
        f"- Plan commitment: `{plan['plan_commitment']}`",
        "",
        "## Eligible entries",
        "",
    ]
    if not plan["eligible_entries"]:
        lines.append("_None._")
    else:
        lines.extend(
            [
                "| Run | Attempt | Queued head | PR associations | Entry commitment |",
                "|---:|---:|---|---|---|",
            ]
        )
        for entry in plan["eligible_entries"]:
            prs = ", ".join(
                f"#{association['pr_number']}→{str(association['current_head_sha'])[:12]}"
                for association in entry["associations"]
            )
            lines.append(
                f"| {entry['run_id']} | {entry['run_attempt']} | "
                f"`{str(entry['queued_head_sha'])[:12]}` | {prs} | "
                f"`{entry['plan_entry_commitment'][:12]}` |"
            )
    lines.extend(
        [
            "",
            "> Read-only plan. Eligibility is not cancellation authority. "
            "A future executor must consume this full unexpired plan envelope and "
            "separately re-fetch/revalidate repository, run attempt, run status/head, "
            "and every PR association immediately before any mutation.",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    if not args.repository:
        raise SystemExit("--repository or GITHUB_REPOSITORY is required")
    token = os.environ.get(args.token_env, "")
    if not token:
        raise SystemExit(f"{args.token_env} is required")

    observed_at = datetime.now(timezone.utc)
    client = GitHubReadOnlyClient(args.repository, token)
    queued = client.list_runs("queued")

    pr_numbers = pr_numbers_for_lookup(queued)
    if len(pr_numbers) > MAX_PR_LOOKUPS:
        raise SystemExit(
            f"refusing {len(pr_numbers)} in-scope PR metadata lookups; "
            f"limit is {MAX_PR_LOOKUPS}"
        )

    pr_by_number: dict[int, dict[str, Any] | None] = {}
    for number in pr_numbers:
        pr_by_number[number] = client.get_pull_request(number)

    plan = build_plan(queued, pr_by_number, observed_at, args.repository)

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(plan, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    print(render_markdown(plan))
    return 0


if __name__ == "__main__":
    sys.exit(main())
