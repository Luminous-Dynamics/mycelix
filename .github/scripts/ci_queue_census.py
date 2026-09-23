#!/usr/bin/env python3
"""Read-only GitHub Actions queue census for CI-GOV-001A.

Uses only GET requests. It never cancels, reruns, edits, labels, or otherwise
mutates repository state.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

API_ROOT = "https://api.github.com"
API_VERSION = "2022-11-28"
PER_PAGE = 100
MAX_RUN_PAGES = 20
MAX_PR_LOOKUPS = 500


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", default=os.environ.get("GITHUB_REPOSITORY", ""))
    parser.add_argument("--token-env", default="GITHUB_TOKEN")
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--now", help="override UTC observation time for deterministic tests")
    return parser.parse_args()


def parse_time(value: str) -> datetime:
    value = value.replace("Z", "+00:00")
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


class GitHubApiError(RuntimeError):
    def __init__(self, path: str, status: int, body: str) -> None:
        super().__init__(f"GitHub GET {path} failed: HTTP {status}: {body}")
        self.path = path
        self.status = status
        self.body = body


class GitHubReadOnlyClient:
    def __init__(self, repository: str, token: str) -> None:
        parts = repository.split("/")
        if len(parts) != 2 or not all(parts):
            raise ValueError("repository must be owner/repo")
        self.repository = repository
        self.token = token

    def get_json(self, path: str, query: dict[str, str | int] | None = None) -> Any:
        if not path.startswith("/"):
            raise ValueError("API path must start with /")
        url = API_ROOT + path
        if query:
            url += "?" + urllib.parse.urlencode(query)
        request = urllib.request.Request(
            url,
            headers={
                "Accept": "application/vnd.github+json",
                "Authorization": f"Bearer {self.token}",
                "User-Agent": "mycelix-ci-gov-queue-census/1",
                "X-GitHub-Api-Version": API_VERSION,
            },
            method="GET",
        )
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise GitHubApiError(path, exc.code, body) from exc

    def list_runs(self, status: str) -> list[dict[str, Any]]:
        runs: list[dict[str, Any]] = []
        for page in range(1, MAX_RUN_PAGES + 1):
            payload = self.get_json(
                f"/repos/{self.repository}/actions/runs",
                {"status": status, "per_page": PER_PAGE, "page": page},
            )
            page_runs = payload.get("workflow_runs")
            if not isinstance(page_runs, list):
                raise RuntimeError("workflow_runs missing from GitHub response")
            runs.extend(page_runs)
            if len(page_runs) < PER_PAGE:
                return runs
        raise RuntimeError(
            f"{status} run census exceeded {MAX_RUN_PAGES * PER_PAGE} records; "
            "refusing a silently truncated report"
        )

    def get_pull_request(self, number: int) -> dict[str, Any] | None:
        try:
            payload = self.get_json(f"/repos/{self.repository}/pulls/{number}")
        except GitHubApiError as exc:
            if exc.status == 404:
                return None
            raise
        if not isinstance(payload, dict):
            raise RuntimeError(f"invalid PR response for #{number}")
        return payload


def run_pr_number(run: dict[str, Any]) -> int | None:
    prs = run.get("pull_requests")
    if not isinstance(prs, list) or not prs:
        return None
    number = prs[0].get("number")
    return number if isinstance(number, int) else None


def iso_age_seconds(now: datetime, created_at: str) -> int:
    age = int((now - parse_time(created_at)).total_seconds())
    return max(age, 0)


def human_duration(seconds: int | None) -> str:
    if seconds is None:
        return "n/a"
    days, rem = divmod(seconds, 86400)
    hours, rem = divmod(rem, 3600)
    minutes, _ = divmod(rem, 60)
    pieces = []
    if days:
        pieces.append(f"{days}d")
    if hours or days:
        pieces.append(f"{hours}h")
    pieces.append(f"{minutes}m")
    return " ".join(pieces)


def summarize(
    queued: list[dict[str, Any]],
    in_progress: list[dict[str, Any]],
    pr_by_number: dict[int, dict[str, Any]],
    now: datetime,
) -> dict[str, Any]:
    queued_workflows = Counter(str(run.get("name") or "<unnamed>") for run in queued)
    in_progress_workflows = Counter(
        str(run.get("name") or "<unnamed>") for run in in_progress
    )

    oldest_age = None
    oldest_run_id = None
    if queued:
        oldest = max(
            queued,
            key=lambda run: iso_age_seconds(now, str(run["created_at"])),
        )
        oldest_age = iso_age_seconds(now, str(oldest["created_at"]))
        oldest_run_id = oldest.get("id")

    pr_state_counts = Counter()
    unknown_pr_runs = 0
    superseded: list[dict[str, Any]] = []

    for run in queued:
        pr_number = run_pr_number(run)
        if pr_number is None:
            pr_state_counts["non_pr"] += 1
            continue
        pr = pr_by_number.get(pr_number)
        if pr is None:
            pr_state_counts["unknown"] += 1
            unknown_pr_runs += 1
            continue
        if pr.get("state") != "open":
            pr_state_counts["closed"] += 1
        elif pr.get("draft") is True:
            pr_state_counts["draft"] += 1
        else:
            pr_state_counts["ready"] += 1

        current_head = (pr.get("head") or {}).get("sha")
        run_head = run.get("head_sha")
        if current_head and run_head and current_head != run_head:
            superseded.append(
                {
                    "run_id": run.get("id"),
                    "workflow": run.get("name"),
                    "pr_number": pr_number,
                    "queued_head_sha": run_head,
                    "current_pr_head_sha": current_head,
                    "created_at": run.get("created_at"),
                }
            )

    superseded.sort(key=lambda item: str(item.get("created_at") or ""))

    return {
        "schema": "mycelix-ci-queue-census-v1",
        "observed_at": now.isoformat().replace("+00:00", "Z"),
        "queued_count": len(queued),
        "in_progress_count": len(in_progress),
        "oldest_queued_age_seconds": oldest_age,
        "oldest_queued_run_id": oldest_run_id,
        "queued_by_workflow": [
            {"workflow": name, "count": count}
            for name, count in queued_workflows.most_common()
        ],
        "in_progress_by_workflow": [
            {"workflow": name, "count": count}
            for name, count in in_progress_workflows.most_common()
        ],
        "queued_pr_state_counts": {
            key: pr_state_counts.get(key, 0)
            for key in ("draft", "ready", "closed", "unknown", "non_pr")
        },
        "unknown_pr_runs": unknown_pr_runs,
        "superseded_queued_count": len(superseded),
        "superseded_queued_runs": superseded,
    }


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# Mycelix CI queue census",
        "",
        f"- Observed: `{summary['observed_at']}`",
        f"- Queued: **{summary['queued_count']}**",
        f"- In progress: **{summary['in_progress_count']}**",
        (
            f"- Oldest queued: **{human_duration(summary['oldest_queued_age_seconds'])}**"
            + (
                f" (run `{summary['oldest_queued_run_id']}`)"
                if summary["oldest_queued_run_id"] is not None
                else ""
            )
        ),
        f"- Superseded queued PR heads: **{summary['superseded_queued_count']}**",
        "",
        "## Queued by workflow",
        "",
        "| Workflow | Queued |",
        "|---|---:|",
    ]
    for item in summary["queued_by_workflow"][:25]:
        lines.append(f"| {item['workflow']} | {item['count']} |")
    if not summary["queued_by_workflow"]:
        lines.append("| _none_ | 0 |")

    states = summary["queued_pr_state_counts"]
    lines.extend(
        [
            "",
            "## Queued run association",
            "",
            "| Association | Runs |",
            "|---|---:|",
            f"| Open draft PR | {states['draft']} |",
            f"| Open ready PR | {states['ready']} |",
            f"| Closed PR | {states['closed']} |",
            f"| PR metadata unresolved | {states['unknown']} |",
            f"| Non-PR event | {states['non_pr']} |",
            "",
            "## Superseded queued exact heads",
            "",
        ]
    )
    if summary["superseded_queued_runs"]:
        lines.extend(
            [
                "| Run | Workflow | PR | Queued head | Current head |",
                "|---:|---|---:|---|---|",
            ]
        )
        for item in summary["superseded_queued_runs"][:25]:
            lines.append(
                f"| {item['run_id']} | {item['workflow']} | #{item['pr_number']} | "
                f"`{str(item['queued_head_sha'])[:12]}` | "
                f"`{str(item['current_pr_head_sha'])[:12]}` |"
            )
        if len(summary["superseded_queued_runs"]) > 25:
            lines.append(
                f"\n_Only first 25 shown; total {len(summary['superseded_queued_runs'])}._"
            )
    else:
        lines.append("_None detected._")

    lines.extend(
        [
            "",
            "> Read-only telemetry. This report does not cancel, rerun, label, "
            "merge, or otherwise mutate workflow/PR state.",
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
    now = parse_time(args.now) if args.now else datetime.now(timezone.utc)

    client = GitHubReadOnlyClient(args.repository, token)
    queued = client.list_runs("queued")
    in_progress = client.list_runs("in_progress")

    pr_numbers = sorted(
        {
            number
            for run in queued + in_progress
            if (number := run_pr_number(run)) is not None
        }
    )
    if len(pr_numbers) > MAX_PR_LOOKUPS:
        raise SystemExit(
            f"refusing {len(pr_numbers)} PR metadata lookups; limit is {MAX_PR_LOOKUPS}"
        )

    pr_by_number = {}
    for number in pr_numbers:
        pr = client.get_pull_request(number)
        if pr is not None:
            pr_by_number[number] = pr
    summary = summarize(queued, in_progress, pr_by_number, now)

    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(render_markdown(summary))
    return 0


if __name__ == "__main__":
    sys.exit(main())
