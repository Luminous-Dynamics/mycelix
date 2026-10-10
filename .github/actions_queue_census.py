#!/usr/bin/env python3
"""Read-only GitHub Actions queue census.

Requires Python 3.10+ and authenticated GitHub CLI access. Only GET requests are
made. This tool never cancels, reruns, dispatches, or changes workflow settings.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

SCHEMA = "mycelix.actions-queue-census.v1"


def parse_utc(value: str | None) -> datetime | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed.astimezone(timezone.utc) if parsed.tzinfo else None


def flatten_pages(payload: Any, list_key: str) -> tuple[list[dict[str, Any]], int | None]:
    """Normalize gh api --paginate --slurp responses and preserve count mismatches."""
    if isinstance(payload, dict):
        pages = [payload]
    elif isinstance(payload, list) and all(isinstance(page, dict) for page in payload):
        pages = payload
    else:
        raise ValueError("expected a JSON object or a slurped array of response objects")

    api_total: int | None = None
    by_id: dict[str, dict[str, Any]] = {}
    for page in pages:
        if "message" in page and list_key not in page:
            raise ValueError(f"GitHub API returned an error: {page.get('message')}")
        count = page.get("total_count")
        if isinstance(count, int):
            api_total = count if api_total is None else max(api_total, count)
        records = page.get(list_key)
        if not isinstance(records, list):
            raise ValueError(f"response page is missing a {list_key} array")
        for record in records:
            if not isinstance(record, dict) or record.get("id") is None:
                raise ValueError(f"{list_key} contains an item without an id")
            key = str(record["id"])
            previous = by_id.get(key)
            if previous is not None:
                for field in ("head_sha", "name"):
                    if field in record and previous.get(field) != record.get(field):
                        raise ValueError(f"conflicting duplicate id {key}")
            by_id[key] = record
    return list(by_id.values()), api_total


def run_identity(run: dict[str, Any]) -> dict[str, Any]:
    prs = run.get("pull_requests")
    numbers = sorted({
        p["number"] for p in prs
        if isinstance(p, dict) and isinstance(p.get("number"), int)
    }) if isinstance(prs, list) else []
    return {
        "run_id": run.get("id"),
        "workflow": run.get("name"),
        "run_number": run.get("run_number"),
        "event": run.get("event"),
        "status": run.get("status"),
        "conclusion": run.get("conclusion"),
        "branch": run.get("head_branch"),
        "head_sha": run.get("head_sha"),
        "pull_requests": numbers,
        "created_at": run.get("created_at"),
        "updated_at": run.get("updated_at"),
        "url": run.get("html_url"),
    }


def count_by(runs: Iterable[dict[str, Any]], key_fn) -> dict[str, int]:
    counts = Counter(str(key_fn(run)) for run in runs)
    return dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))


def summarize_status(payload: Any, now: datetime) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    runs, api_total = flatten_pages(payload, "workflow_runs")
    valid = [
        (parse_utc(run.get("created_at")), run)
        for run in runs
        if parse_utc(run.get("created_at")) is not None
    ]
    oldest = min(valid, key=lambda pair: pair[0])[1] if valid else None
    newest = max(valid, key=lambda pair: pair[0])[1] if valid else None
    age = max(0, int((now - min(dt for dt, _ in valid)).total_seconds())) if valid else None

    def pr_key(run: dict[str, Any]) -> str:
        numbers = run_identity(run)["pull_requests"]
        return ",".join(str(number) for number in numbers) if numbers else "no PR association"

    summary = {
        "api_total_count": api_total,
        "fetched_unique_count": len(runs),
        "pagination_complete_by_count": api_total is not None and api_total == len(runs),
        "by_workflow": count_by(runs, lambda run: run.get("name", "<missing workflow name>")),
        "by_event": count_by(runs, lambda run: run.get("event", "<missing event>")),
        "by_pull_request": count_by(runs, pr_key),
        "oldest_run": run_identity(oldest) if oldest else None,
        "newest_run": run_identity(newest) if newest else None,
        "oldest_age_seconds": age,
        "oldest_age_hours": round(age / 3600, 2) if age is not None else None,
    }
    return summary, [run_identity(run) for run in runs]


def gh_api(repo: str, endpoint: str) -> Any:
    command = ["gh", "api", "--paginate", "--slurp", f"repos/{repo}/{endpoint}"]
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"gh api failed ({completed.returncode}): {detail}")
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"gh api returned invalid JSON for {endpoint}: {exc}") from exc


def choose_samples(runs: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    """Select oldest/newest runs while preferring distinct workflow and PR combinations."""
    if limit <= 0:
        return []
    ordered = sorted(
        runs,
        key=lambda run: (
            parse_utc(run.get("created_at")) or datetime.max.replace(tzinfo=timezone.utc),
            int(run["run_id"]),
        ),
    )
    half = max(1, (limit + 1) // 2)
    candidates = ordered[:half] + list(reversed(ordered))[: max(1, limit // 2)]
    selected: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    seen_groups: set[tuple[str, tuple[int, ...]]] = set()
    for run in candidates + ordered:
        run_id = str(run["run_id"])
        group = (str(run.get("workflow")), tuple(run.get("pull_requests", [])))
        if run_id in seen_ids or (group in seen_groups and len(selected) < limit):
            continue
        selected.append(run)
        seen_ids.add(run_id)
        seen_groups.add(group)
        if len(selected) >= limit:
            break
    return selected


def collect_jobs(repo: str, run: dict[str, Any]) -> dict[str, Any]:
    payload = gh_api(repo, f"actions/runs/{run['run_id']}/jobs?per_page=100")
    jobs, api_total = flatten_pages(payload, "jobs")
    return {
        "run": run,
        "api_total_count": api_total,
        "fetched_unique_count": len(jobs),
        "pagination_complete_by_count": api_total is not None and api_total == len(jobs),
        "jobs": [{
            "job_id": job.get("id"),
            "name": job.get("name"),
            "status": job.get("status"),
            "conclusion": job.get("conclusion"),
            "runner_name": job.get("runner_name"),
            "runner_group_name": job.get("runner_group_name"),
            "labels": job.get("labels", []),
            "started_at": job.get("started_at"),
            "completed_at": job.get("completed_at"),
        } for job in jobs],
    }


def build_census(repo: str, queued_payload: Any, active_payload: Any,
                 now: datetime | None = None) -> dict[str, Any]:
    now = now or datetime.now(timezone.utc)
    if now.tzinfo is None:
        raise ValueError("capture timestamp must be timezone-aware")
    now = now.astimezone(timezone.utc)
    queued_summary, _ = summarize_status(queued_payload, now)
    active_summary, _ = summarize_status(active_payload, now)
    return {
        "schema": SCHEMA,
        "captured_at": now.isoformat().replace("+00:00", "Z"),
        "repository": repo,
        "read_only": True,
        "statuses": {"queued": queued_summary, "in_progress": active_summary},
        "queued_run_samples": [],
        "interpretation_boundary": (
            "This census records observed workflow-run/job metadata. It does not establish root cause, "
            "prove runner-assignment failure by itself, or qualify code. Compare sampled job metadata "
            "with organization/repository Actions runner usage and policy settings."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default="Luminous-Dynamics/mycelix",
                        help="owner/repository (default: Luminous-Dynamics/mycelix)")
    parser.add_argument("--output", type=Path, help="write JSON evidence to this path instead of stdout")
    parser.add_argument("--job-samples", type=int, default=6,
                        help="sample this many queued runs for runner metadata (default: 6; 0 disables)")
    args = parser.parse_args(argv)
    if "/" not in args.repo or any(not part for part in args.repo.split("/", 1)):
        parser.error("--repo must be in owner/repository form")
    if args.job_samples < 0:
        parser.error("--job-samples cannot be negative")
    if not shutil.which("gh"):
        print("error: GitHub CLI gh was not found on PATH", file=sys.stderr)
        return 2

    try:
        captured_at = datetime.now(timezone.utc)
        queued_payload = gh_api(args.repo, "actions/runs?status=queued&per_page=100")
        active_payload = gh_api(args.repo, "actions/runs?status=in_progress&per_page=100")
        result = build_census(args.repo, queued_payload, active_payload, captured_at)
        if args.job_samples:
            _, queued = flatten_pages(queued_payload, "workflow_runs")
            identities = [run_identity(run) for run in queued]
            result["queued_run_samples"] = [
                collect_jobs(args.repo, run)
                for run in choose_samples(identities, args.job_samples)
            ]
        encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded, encoding="utf-8")
        else:
            sys.stdout.write(encoded)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
