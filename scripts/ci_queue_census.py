#!/usr/bin/env python3
"""CI-GOV-001A: read-only GitHub Actions queue census."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

SCHEMA = "ci-gov-001a-queue-census-v1"
PER_PAGE = 100
DEFAULT_JOB_SAMPLE = 12


class CensusError(RuntimeError):
    pass


def parse_time(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise CensusError(f"invalid timestamp: {value!r}") from exc


def iso_z(value: datetime | None) -> str | None:
    if value is None:
        return None
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def get_json(url: str, token: str) -> dict[str, Any]:
    request = Request(
        url,
        method="GET",
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": "2022-11-28",
            "User-Agent": "mycelix-ci-gov-001a-read-only-census",
        },
    )
    try:
        with urlopen(request, timeout=30) as response:
            if getattr(response, "status", 200) != 200:
                raise CensusError(f"GET returned HTTP {response.status}: {url}")
            data = json.load(response)
    except HTTPError as exc:
        raise CensusError(f"GET returned HTTP {exc.code}: {url}") from exc
    except URLError as exc:
        raise CensusError(f"GET failed for {url}: {exc.reason}") from exc
    if not isinstance(data, dict):
        raise CensusError(f"GET returned non-object JSON: {url}")
    return data


def paged_runs(api_url: str, repo: str, token: str, status: str) -> list[dict[str, Any]]:
    runs: list[dict[str, Any]] = []
    for page in range(1, 101):
        query = urlencode({"status": status, "per_page": PER_PAGE, "page": page})
        data = get_json(f"{api_url}/repos/{repo}/actions/runs?{query}", token)
        batch = data.get("workflow_runs")
        if not isinstance(batch, list):
            raise CensusError(f"status={status!r} response omitted workflow_runs")
        runs.extend(item for item in batch if isinstance(item, dict))
        if len(batch) < PER_PAGE:
            return runs
    raise CensusError("refusing to page beyond 10,000 workflow runs")


def first_pr_number(run: dict[str, Any]) -> int | None:
    prs = run.get("pull_requests")
    if not isinstance(prs, list) or not prs or not isinstance(prs[0], dict):
        return None
    number = prs[0].get("number")
    return number if isinstance(number, int) else None


def summarize_queued(queued: list[dict[str, Any]], observed_at: datetime) -> dict[str, Any]:
    by_workflow: Counter[str] = Counter()
    by_event: Counter[str] = Counter()
    by_branch: Counter[str] = Counter()
    dated: list[tuple[datetime, dict[str, Any]]] = []
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)

    for run in queued:
        workflow_key = str(run.get("workflow_id") or run.get("name") or "<unknown>")
        by_workflow[str(run.get("name") or f"workflow:{workflow_key}")] += 1
        by_event[str(run.get("event") or "<unknown>")] += 1
        branch = str(run.get("head_branch") or "<none>")
        by_branch[branch] += 1

        created = parse_time(run.get("created_at"))
        if created is not None:
            dated.append((created, run))

        pr = first_pr_number(run)
        scope = f"pr:{pr}" if pr is not None else f"branch:{branch}"
        grouped[(workflow_key, scope)].append(run)

    distinct_old_ids: set[int] = set()
    duplicate_head_ids: set[int] = set()
    groups: list[dict[str, Any]] = []
    minimum = datetime.min.replace(tzinfo=timezone.utc)

    for (workflow_key, scope), members in grouped.items():
        if len(members) < 2:
            continue
        ordered = sorted(members, key=lambda r: parse_time(r.get("created_at")) or minimum)
        newest = ordered[-1]
        newest_sha = str(newest.get("head_sha") or "")
        older_distinct = [
            run for run in ordered[:-1]
            if str(run.get("head_sha") or "") and str(run.get("head_sha") or "") != newest_sha
        ]
        duplicates = [
            run for run in ordered[:-1]
            if newest_sha and str(run.get("head_sha") or "") == newest_sha
        ]
        if not older_distinct and not duplicates:
            continue
        for run in older_distinct:
            if isinstance(run.get("id"), int):
                distinct_old_ids.add(run["id"])
        for run in duplicates:
            if isinstance(run.get("id"), int):
                duplicate_head_ids.add(run["id"])
        groups.append(
            {
                "workflow_key": workflow_key,
                "scope": scope,
                "queued_run_count": len(ordered),
                "newest_run_id": newest.get("id"),
                "newest_head_sha": newest_sha or None,
                "older_distinct_head_count": len(older_distinct),
                "duplicate_exact_head_count": len(duplicates),
            }
        )

    dated.sort(key=lambda item: item[0])

    def describe(item: tuple[datetime, dict[str, Any]] | None) -> dict[str, Any] | None:
        if item is None:
            return None
        when, run = item
        return {
            "run_id": run.get("id"),
            "workflow": run.get("name"),
            "head_branch": run.get("head_branch"),
            "head_sha": run.get("head_sha"),
            "created_at": iso_z(when),
            "age_seconds": max(0, int((observed_at - when).total_seconds())),
        }

    return {
        "oldest_queued": describe(dated[0] if dated else None),
        "newest_queued": describe(dated[-1] if dated else None),
        "by_workflow": dict(sorted(by_workflow.items(), key=lambda kv: (-kv[1], kv[0]))),
        "by_event": dict(sorted(by_event.items(), key=lambda kv: (-kv[1], kv[0]))),
        "by_branch": dict(sorted(by_branch.items(), key=lambda kv: (-kv[1], kv[0]))),
        "supersession_telemetry": {
            "groups_with_multiple_queued_runs": len(groups),
            "older_distinct_head_candidate_count": len(distinct_old_ids),
            "duplicate_exact_head_candidate_count": len(duplicate_head_ids),
            "groups": sorted(groups, key=lambda g: (-g["queued_run_count"], g["workflow_key"], g["scope"])),
            "nonclaim": "candidate telemetry is not cancellation authority",
        },
    }


def bounded_job_sample(queued: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    if limit <= 0 or not queued:
        return []
    minimum = datetime.min.replace(tzinfo=timezone.utc)
    ordered = sorted(queued, key=lambda r: parse_time(r.get("created_at")) or minimum)
    if len(ordered) <= limit:
        return ordered
    left = limit // 2
    right = limit - left
    selected = ordered[:left] + ordered[-right:]
    seen: set[int] = set()
    result: list[dict[str, Any]] = []
    for run in selected:
        run_id = run.get("id")
        if isinstance(run_id, int) and run_id not in seen:
            seen.add(run_id)
            result.append(run)
    return result


def inspect_job_shape(api_url: str, repo: str, token: str, queued: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    observations: list[dict[str, Any]] = []
    for run in bounded_job_sample(queued, limit):
        run_id = run.get("id")
        if not isinstance(run_id, int):
            continue
        data = get_json(f"{api_url}/repos/{repo}/actions/runs/{run_id}/jobs?per_page=100", token)
        jobs = data.get("jobs")
        if not isinstance(jobs, list):
            raise CensusError(f"jobs response omitted jobs for run {run_id}")
        queued_without_steps = 0
        jobs_with_steps = 0
        for job in jobs:
            if not isinstance(job, dict):
                continue
            steps = job.get("steps")
            if isinstance(steps, list) and steps:
                jobs_with_steps += 1
            elif job.get("status") == "queued":
                queued_without_steps += 1
        observations.append(
            {
                "run_id": run_id,
                "workflow": run.get("name"),
                "created_at": run.get("created_at"),
                "job_count": len(jobs),
                "queued_jobs_without_steps": queued_without_steps,
                "jobs_with_steps": jobs_with_steps,
            }
        )
    return observations


def most_recent_success(api_url: str, repo: str, token: str) -> dict[str, Any] | None:
    query = urlencode({"status": "success", "per_page": 1, "page": 1})
    data = get_json(f"{api_url}/repos/{repo}/actions/runs?{query}", token)
    runs = data.get("workflow_runs")
    if not isinstance(runs, list) or not runs or not isinstance(runs[0], dict):
        return None
    run = runs[0]
    return {
        "run_id": run.get("id"),
        "workflow": run.get("name"),
        "head_sha": run.get("head_sha"),
        "created_at": run.get("created_at"),
        "updated_at": run.get("updated_at"),
    }


def build_report(repo: str, queued: list[dict[str, Any]], in_progress: list[dict[str, Any]], observed_at: datetime, job_sample: list[dict[str, Any]] | None = None, recent_success: dict[str, Any] | None = None) -> dict[str, Any]:
    queue_summary = summarize_queued(queued, observed_at)
    return {
        "schema": SCHEMA,
        "repository": repo,
        "observed_at_utc": iso_z(observed_at),
        "status_vocabulary": {
            "queued": "requested but not evidence of execution",
            "in_progress": "runner/job execution has begun",
            "success": "workflow conclusion only; domain claims still depend on the exact gate",
            "failure": "classify the exact failing boundary before interpreting the subject",
            "cancelled": "not a semantic PASS or FAIL unless an exact gate explicitly defines it",
        },
        "counts": {"queued": len(queued), "in_progress": len(in_progress)},
        **queue_summary,
        "job_shape_sample": job_sample or [],
        "most_recent_success": recent_success,
        "authority": {
            "read_only": True,
            "can_cancel_runs": False,
            "can_rerun_runs": False,
            "can_dispatch_workflows": False,
            "can_modify_repository": False,
        },
        "nonclaims": [
            "queue census does not identify the root cause of runner delay",
            "superseded-candidate telemetry does not authorize cancellation",
            "queued does not mean started, failed, or passed",
            "repository telemetry does not establish account billing or hosted-runner capacity",
        ],
    }


def self_test() -> None:
    now = datetime(2026, 9, 16, 18, 0, tzinfo=timezone.utc)
    queued = [
        {"id": 1, "workflow_id": 10, "name": "Gate", "event": "pull_request", "head_branch": "a", "head_sha": "aaa", "created_at": "2026-09-16T16:00:00Z", "pull_requests": [{"number": 7}]},
        {"id": 2, "workflow_id": 10, "name": "Gate", "event": "pull_request", "head_branch": "a", "head_sha": "bbb", "created_at": "2026-09-16T17:00:00Z", "pull_requests": [{"number": 7}]},
        {"id": 3, "workflow_id": 11, "name": "Other", "event": "pull_request", "head_branch": "b", "head_sha": "ccc", "created_at": "2026-09-16T17:30:00Z", "pull_requests": [{"number": 8}]},
        {"id": 4, "workflow_id": 11, "name": "Other", "event": "pull_request", "head_branch": "b", "head_sha": "ccc", "created_at": "2026-09-16T17:45:00Z", "pull_requests": [{"number": 8}]},
    ]
    report = build_report("owner/repo", queued, [], now)
    assert report["counts"] == {"queued": 4, "in_progress": 0}
    assert report["oldest_queued"]["run_id"] == 1
    assert report["newest_queued"]["run_id"] == 4
    supersession = report["supersession_telemetry"]
    assert supersession["groups_with_multiple_queued_runs"] == 2
    assert supersession["older_distinct_head_candidate_count"] == 1
    assert supersession["duplicate_exact_head_candidate_count"] == 1
    assert report["authority"]["read_only"] is True
    assert report["authority"]["can_cancel_runs"] is False
    print(json.dumps({"schema": SCHEMA, "self_test": "PASS"}, sort_keys=True))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY"))
    parser.add_argument("--api-url", default=os.environ.get("GITHUB_API_URL", "https://api.github.com"))
    parser.add_argument("--token-env", default="GITHUB_TOKEN")
    parser.add_argument("--job-sample", type=int, default=DEFAULT_JOB_SAMPLE)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if not args.repo or "/" not in args.repo:
        raise CensusError("--repo owner/name or GITHUB_REPOSITORY is required")
    token = os.environ.get(args.token_env)
    if not token and args.token_env != "GH_TOKEN":
        token = os.environ.get("GH_TOKEN")
    if not token:
        raise CensusError(f"read-only API token missing from {args.token_env} / GH_TOKEN")
    if args.job_sample < 0 or args.job_sample > 50:
        raise CensusError("--job-sample must be between 0 and 50")

    observed_at = datetime.now(timezone.utc)
    queued = paged_runs(args.api_url, args.repo, token, "queued")
    in_progress = paged_runs(args.api_url, args.repo, token, "in_progress")
    job_sample = inspect_job_shape(args.api_url, args.repo, token, queued, args.job_sample)
    report = build_report(
        args.repo,
        queued,
        in_progress,
        observed_at,
        job_sample=job_sample,
        recent_success=most_recent_success(args.api_url, args.repo, token),
    )
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    sys.stdout.write(encoded)


if __name__ == "__main__":
    try:
        main()
    except CensusError as exc:
        print(f"CI-GOV-001A FAIL: {exc}", file=sys.stderr)
        raise SystemExit(2)
