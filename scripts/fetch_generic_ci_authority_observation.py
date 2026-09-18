#!/usr/bin/env python3
"""Bounded read-only observation adapter for trusted generic CI authority.

No check publication and no product execution. It reads:
- current PR metadata,
- bounded PR file records via the existing CI-GOV-001G adapter,
- latest-attempt jobs for one completed workflow run.

PR-files uncertainty is preserved for fail-closed admission. Jobs uncertainty is
preserved for trusted non-PASS. PR metadata uncertainty suppresses publication
because current-head binding cannot be established.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import socket
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import HTTPRedirectHandler, Request, build_opener

ADAPTER_ID = "generic-mycelix-ci-authority-observation-v1"
AUTHORITY = "ReadOnlyAuthorityObservation"
API_HOST = "api.github.com"
API_VERSION = "2022-11-28"
JOB_PAGE_SIZE = 100
MAX_JOBS = 1000
MAX_JOB_PAGES = 10
MAX_BODY_BYTES = 8 * 1024 * 1024
REPO_RE = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
SHA_RE = re.compile(r"^[0-9a-f]{40}$")


class ObservationError(RuntimeError):
    pass


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


FetchPage = Callable[[str, str], tuple[int, str | None, bytes]]


def _default_fetch(url: str, token: str) -> tuple[int, str | None, bytes]:
    request = Request(
        url,
        method="GET",
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": API_VERSION,
            "User-Agent": ADAPTER_ID,
        },
    )
    opener = build_opener(NoRedirect)
    try:
        with opener.open(request, timeout=15) as response:
            status = getattr(response, "status", response.getcode())
            if response.geturl() != url:
                raise ObservationError("GitHub API redirect/final URL drift refused")
            body = response.read(MAX_BODY_BYTES + 1)
            if len(body) > MAX_BODY_BYTES:
                raise ObservationError("GitHub API response exceeded byte bound")
            return status, response.headers.get("Link"), body
    except HTTPError as exc:
        raise ObservationError(f"GitHub API HTTP error {exc.code}") from exc
    except (URLError, TimeoutError, socket.timeout) as exc:
        raise ObservationError(
            f"GitHub API transport error: {type(exc).__name__}"
        ) from exc


def _validate_repo(value: Any) -> str:
    if not isinstance(value, str) or not REPO_RE.fullmatch(value):
        raise ObservationError("repository must be safe owner/name")
    return value


def _positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ObservationError(f"{label} must be positive integer")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ObservationError(f"{label} must be positive integer") from exc
    if number <= 0:
        raise ObservationError(f"{label} must be positive integer")
    return number


def _sha40(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise ObservationError(f"{label} must be lowercase 40-hex SHA")
    return value


def _json_body(status: int, body: bytes, label: str) -> Any:
    if status != 200:
        raise ObservationError(f"{label} returned HTTP {status}")
    try:
        return json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ObservationError(f"{label} returned invalid UTF-8 JSON") from exc


def _has_next(link_header: str | None) -> bool:
    return bool(link_header and any('rel="next"' in x for x in link_header.split(",")))


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ObservationError(f"cannot load module {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class AuthorityObservation:
    publish_subject_known: bool
    repository: str | None
    pr_number: int | None
    run_id: int | None
    run_attempt: int | None
    workflow_name: str | None
    workflow_path: str | None
    workflow_event: str | None
    workflow_conclusion: str | None
    triggering_run_head_sha: str | None
    event_pr_head_sha: str | None
    current_pr_head_sha: str | None
    current_pr_state: str | None
    declared_changed_files: int | None
    file_records: list[dict[str, str]] | None
    files_observable: bool
    files_reason: str
    jobs: list[dict[str, str]] | None
    jobs_observable: bool
    jobs_reason: str

    def run_metadata(self) -> dict[str, Any]:
        return {
            "workflow_name": self.workflow_name,
            "workflow_path": self.workflow_path,
            "event": self.workflow_event,
            "run_id": self.run_id,
            "run_attempt": self.run_attempt,
            "workflow_conclusion": self.workflow_conclusion,
            "triggering_run_head_sha": self.triggering_run_head_sha,
            "pr_number": self.pr_number,
            "pr_head_sha": self.event_pr_head_sha,
            "current_pr_head_sha": self.current_pr_head_sha,
            "current_pr_state": self.current_pr_state,
        }

    def receipt(self) -> dict[str, Any]:
        return {
            "adapter_id": ADAPTER_ID,
            "authority": AUTHORITY,
            "publish_subject_known": self.publish_subject_known,
            "repository": self.repository,
            "pr_number": self.pr_number,
            "run_id": self.run_id,
            "run_attempt": self.run_attempt,
            "workflow_name": self.workflow_name,
            "workflow_path": self.workflow_path,
            "workflow_event": self.workflow_event,
            "workflow_conclusion": self.workflow_conclusion,
            "triggering_run_head_sha": self.triggering_run_head_sha,
            "event_pr_head_sha": self.event_pr_head_sha,
            "current_pr_head_sha": self.current_pr_head_sha,
            "current_pr_state": self.current_pr_state,
            "declared_changed_files": self.declared_changed_files,
            "files_observable": self.files_observable,
            "files_reason": self.files_reason,
            "jobs_observable": self.jobs_observable,
            "jobs_reason": self.jobs_reason,
            "job_count": None if self.jobs is None else len(self.jobs),
            "grants_ci_pass": False,
            "grants_product_qualification": False,
        }


def _normalize_event(event: Any) -> dict[str, Any]:
    if not isinstance(event, dict):
        raise ObservationError("workflow_run event root must be object")
    workflow_run = event.get("workflow_run")
    if not isinstance(workflow_run, dict):
        raise ObservationError("workflow_run payload missing")
    if workflow_run.get("name") != "Mycelix CI":
        raise ObservationError("triggering workflow name drift")
    if workflow_run.get("path") != ".github/workflows/ci.yml":
        raise ObservationError("triggering workflow path drift")
    if workflow_run.get("event") != "pull_request":
        raise ObservationError("authority observation applies only to pull_request runs")
    run_id = _positive_int(workflow_run.get("id"), "workflow_run.id")
    run_attempt = _positive_int(
        workflow_run.get("run_attempt"), "workflow_run.run_attempt"
    )
    conclusion = workflow_run.get("conclusion")
    if not isinstance(conclusion, str) or not conclusion:
        raise ObservationError("workflow_run.conclusion missing")
    run_head = _sha40(workflow_run.get("head_sha"), "workflow_run.head_sha")
    pull_requests = workflow_run.get("pull_requests")
    if (
        not isinstance(pull_requests, list)
        or len(pull_requests) != 1
        or not isinstance(pull_requests[0], dict)
    ):
        raise ObservationError("workflow_run must bind exactly one pull request")
    pull_request = pull_requests[0]
    pr_number = _positive_int(
        pull_request.get("number"), "workflow_run.pull_requests[0].number"
    )
    head = pull_request.get("head")
    if not isinstance(head, dict):
        raise ObservationError("workflow_run pull request head missing")
    event_pr_head = _sha40(head.get("sha"), "workflow_run pull request head SHA")
    return {
        "run_id": run_id,
        "run_attempt": run_attempt,
        "workflow_name": workflow_run["name"],
        "workflow_path": workflow_run["path"],
        "workflow_event": workflow_run["event"],
        "workflow_conclusion": conclusion,
        "triggering_run_head_sha": run_head,
        "pr_number": pr_number,
        "event_pr_head_sha": event_pr_head,
    }


def _fetch_pr_metadata(
    repo: str, pr_number: int, token: str, fetch_page: FetchPage
) -> dict[str, Any]:
    url = f"https://{API_HOST}/repos/{quote(repo, safe='/')}/pulls/{pr_number}"
    status, _, body = fetch_page(url, token)
    payload = _json_body(status, body, "pull request metadata")
    if not isinstance(payload, dict):
        raise ObservationError("pull request metadata root must be object")
    if payload.get("number") != pr_number:
        raise ObservationError("pull request number drift")
    state = payload.get("state")
    if state not in {"open", "closed"}:
        raise ObservationError(f"unexpected pull request state {state!r}")
    head = payload.get("head")
    base = payload.get("base")
    if not isinstance(head, dict) or not isinstance(base, dict):
        raise ObservationError("pull request head/base metadata missing")
    current_head = _sha40(head.get("sha"), "current PR head SHA")
    _sha40(base.get("sha"), "current PR base SHA")
    changed_files = payload.get("changed_files")
    if (
        not isinstance(changed_files, int)
        or isinstance(changed_files, bool)
        or changed_files <= 0
    ):
        raise ObservationError("pull request changed_files must be positive integer")
    return {
        "state": state,
        "current_pr_head_sha": current_head,
        "changed_files": changed_files,
    }


def _fetch_jobs(
    repo: str,
    run_id: int,
    triggering_run_head_sha: str,
    token: str,
    fetch_page: FetchPage,
) -> tuple[list[dict[str, str]] | None, bool, str]:
    records: list[dict[str, str]] = []
    try:
        total: int | None = None
        expected_pages: int | None = None
        seen_ids: set[int] = set()
        for page in range(1, MAX_JOB_PAGES + 1):
            url = (
                f"https://{API_HOST}/repos/{quote(repo, safe='/')}/actions/runs/"
                f"{run_id}/jobs?filter=latest&per_page={JOB_PAGE_SIZE}&page={page}"
            )
            status, link, body = fetch_page(url, token)
            payload = _json_body(status, body, f"workflow jobs page {page}")
            if not isinstance(payload, dict):
                raise ObservationError("workflow jobs root must be object")
            page_total = payload.get("total_count")
            jobs = payload.get("jobs")
            if (
                not isinstance(page_total, int)
                or isinstance(page_total, bool)
                or page_total <= 0
            ):
                raise ObservationError("workflow jobs total_count invalid")
            if page_total > MAX_JOBS:
                raise ObservationError("workflow jobs exceed bounded maximum")
            if total is None:
                total = page_total
                expected_pages = (total + JOB_PAGE_SIZE - 1) // JOB_PAGE_SIZE
            elif page_total != total:
                raise ObservationError("workflow jobs total_count drift across pages")
            if not isinstance(jobs, list):
                raise ObservationError("workflow jobs page.jobs must be list")
            assert total is not None and expected_pages is not None
            expected_len = (
                JOB_PAGE_SIZE
                if page < expected_pages
                else total - JOB_PAGE_SIZE * (expected_pages - 1)
            )
            if page > expected_pages:
                raise ObservationError("workflow jobs returned unexpected extra page")
            if len(jobs) != expected_len:
                raise ObservationError(
                    f"workflow jobs page {page} length {len(jobs)} != expected {expected_len}"
                )
            if page < expected_pages and not _has_next(link):
                raise ObservationError("workflow jobs pagination missing rel=next")
            if page == expected_pages and _has_next(link):
                raise ObservationError("final workflow jobs page advertises rel=next")
            for index, item in enumerate(jobs):
                if not isinstance(item, dict):
                    raise ObservationError(f"workflow job {index} must be object")
                job_id = item.get("id")
                if (
                    not isinstance(job_id, int)
                    or isinstance(job_id, bool)
                    or job_id <= 0
                ):
                    raise ObservationError("workflow job id invalid")
                if job_id in seen_ids:
                    raise ObservationError("duplicate workflow job id")
                seen_ids.add(job_id)
                if item.get("run_id") != run_id:
                    raise ObservationError("workflow job run_id drift")
                if item.get("head_sha") != triggering_run_head_sha:
                    raise ObservationError("workflow job head_sha drift")
                name = item.get("name")
                status_value = item.get("status")
                conclusion = item.get("conclusion")
                if not isinstance(name, str) or not name:
                    raise ObservationError("workflow job name invalid")
                if not isinstance(status_value, str) or not status_value:
                    raise ObservationError("workflow job status invalid")
                if conclusion is not None and not isinstance(conclusion, str):
                    raise ObservationError("workflow job conclusion invalid")
                records.append(
                    {
                        "name": name,
                        "status": status_value,
                        "conclusion": conclusion,
                    }
                )
            if page == expected_pages:
                break
        if total is None or len(records) != total:
            raise ObservationError("workflow jobs observation incomplete")
        return records, True, "latest-attempt Actions jobs observation is complete"
    except ObservationError as exc:
        return None, False, str(exc)
    except Exception as exc:
        return None, False, f"unexpected jobs adapter error: {type(exc).__name__}"


def observe(
    *,
    event: Any,
    repository: Any,
    token: str,
    pr_files_module: Any,
    fetch_page: FetchPage = _default_fetch,
) -> AuthorityObservation:
    repo: str | None = None
    event_meta: dict[str, Any] | None = None
    try:
        repo = _validate_repo(repository)
        if not isinstance(token, str) or not token:
            raise ObservationError("GitHub token missing")
        event_meta = _normalize_event(event)
        pr_meta = _fetch_pr_metadata(
            repo, event_meta["pr_number"], token, fetch_page
        )
    except ObservationError as exc:
        return AuthorityObservation(
            False,
            repo,
            None if event_meta is None else event_meta.get("pr_number"),
            None if event_meta is None else event_meta.get("run_id"),
            None if event_meta is None else event_meta.get("run_attempt"),
            None if event_meta is None else event_meta.get("workflow_name"),
            None if event_meta is None else event_meta.get("workflow_path"),
            None if event_meta is None else event_meta.get("workflow_event"),
            None if event_meta is None else event_meta.get("workflow_conclusion"),
            None if event_meta is None else event_meta.get("triggering_run_head_sha"),
            None if event_meta is None else event_meta.get("event_pr_head_sha"),
            None,
            None,
            None,
            None,
            False,
            f"PR metadata unavailable: {exc}",
            None,
            False,
            "jobs observation not attempted without current-head binding",
        )

    files_observation = pr_files_module.observe(
        repository=repo,
        pr_number=event_meta["pr_number"],
        declared_changed_files=pr_meta["changed_files"],
        token=token,
        fetch_page=fetch_page,
    )
    jobs, jobs_ok, jobs_reason = _fetch_jobs(
        repo,
        event_meta["run_id"],
        event_meta["triggering_run_head_sha"],
        token,
        fetch_page,
    )
    return AuthorityObservation(
        True,
        repo,
        event_meta["pr_number"],
        event_meta["run_id"],
        event_meta["run_attempt"],
        event_meta["workflow_name"],
        event_meta["workflow_path"],
        event_meta["workflow_event"],
        event_meta["workflow_conclusion"],
        event_meta["triggering_run_head_sha"],
        event_meta["event_pr_head_sha"],
        pr_meta["current_pr_head_sha"],
        pr_meta["state"],
        pr_meta["changed_files"],
        files_observation.records,
        files_observation.observable,
        files_observation.reason,
        jobs,
        jobs_ok,
        jobs_reason,
    )


def _fake_fetch_factory(
    *,
    pr_payload: dict[str, Any],
    files_payload: list[dict[str, Any]],
    jobs_payload: dict[str, Any],
):
    def fetch(url: str, token: str):
        assert token == "token"
        if "/pulls/7/files?" in url:
            return 200, None, json.dumps(files_payload).encode()
        if url.endswith("/pulls/7"):
            return 200, None, json.dumps(pr_payload).encode()
        if "/actions/runs/99/jobs?" in url:
            return 200, None, json.dumps(jobs_payload).encode()
        raise AssertionError(url)

    return fetch


def self_test(pr_files_module: Any) -> None:
    event_head = "a" * 40
    event = {
        "workflow_run": {
            "id": 99,
            "run_attempt": 1,
            "name": "Mycelix CI",
            "path": ".github/workflows/ci.yml",
            "event": "pull_request",
            "conclusion": "success",
            "head_sha": "c" * 40,
            "pull_requests": [{"number": 7, "head": {"sha": event_head}}],
        }
    }
    pr_payload = {
        "number": 7,
        "state": "open",
        "head": {"sha": event_head},
        "base": {"sha": "b" * 40},
        "changed_files": 1,
    }
    files_payload = [{"filename": "docs/lex-net/a.md", "status": "modified"}]
    jobs_payload = {
        "total_count": 1,
        "jobs": [
            {
                "id": 1,
                "run_id": 99,
                "head_sha": "c" * 40,
                "name": "changes",
                "status": "completed",
                "conclusion": "success",
            }
        ],
    }
    observation = observe(
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        pr_files_module=pr_files_module,
        fetch_page=_fake_fetch_factory(
            pr_payload=pr_payload,
            files_payload=files_payload,
            jobs_payload=jobs_payload,
        ),
    )
    assert observation.publish_subject_known
    assert observation.files_observable and observation.jobs_observable
    assert observation.current_pr_head_sha == event_head
    assert len(observation.jobs or []) == 1

    stale_pr = dict(pr_payload)
    stale_pr["head"] = {"sha": "d" * 40}
    observation = observe(
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        pr_files_module=pr_files_module,
        fetch_page=_fake_fetch_factory(
            pr_payload=stale_pr,
            files_payload=files_payload,
            jobs_payload=jobs_payload,
        ),
    )
    assert observation.publish_subject_known
    assert observation.current_pr_head_sha != observation.event_pr_head_sha

    bad_jobs = dict(jobs_payload)
    bad_jobs["total_count"] = 2
    observation = observe(
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        pr_files_module=pr_files_module,
        fetch_page=_fake_fetch_factory(
            pr_payload=pr_payload,
            files_payload=files_payload,
            jobs_payload=bad_jobs,
        ),
    )
    assert observation.publish_subject_known
    assert not observation.jobs_observable and observation.jobs is None

    bad_pr = dict(pr_payload)
    bad_pr["number"] = 8
    observation = observe(
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        pr_files_module=pr_files_module,
        fetch_page=_fake_fetch_factory(
            pr_payload=bad_pr,
            files_payload=files_payload,
            jobs_payload=jobs_payload,
        ),
    )
    assert not observation.publish_subject_known


def _write(path: str | None, value: Any) -> None:
    if path:
        Path(path).write_text(
            json.dumps(value, sort_keys=True) + "\n", encoding="utf-8"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--event-json", default=os.getenv("GITHUB_EVENT_PATH"))
    parser.add_argument("--repository", default=os.getenv("GITHUB_REPOSITORY", ""))
    parser.add_argument("--token", default=os.getenv("GITHUB_TOKEN", ""))
    parser.add_argument(
        "--pr-files-adapter", default="scripts/fetch_generic_ci_pr_files.py"
    )
    parser.add_argument("--run-output")
    parser.add_argument("--files-output")
    parser.add_argument("--jobs-output")
    parser.add_argument("--receipt-output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        module = _load_module(Path(args.pr_files_adapter), "_trusted_pr_files_adapter")
        if args.self_test:
            self_test(module)
            print(
                json.dumps(
                    {
                        "adapter_id": ADAPTER_ID,
                        "self_test": "PASS",
                        "authority": AUTHORITY,
                        "grants_ci_pass": False,
                        "grants_product_qualification": False,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if not args.event_json:
            raise ObservationError("workflow_run event JSON path missing")
        event = json.loads(Path(args.event_json).read_text(encoding="utf-8"))
        observation = observe(
            event=event,
            repository=args.repository,
            token=args.token,
            pr_files_module=module,
        )
        _write(args.run_output, observation.run_metadata())
        _write(args.files_output, observation.file_records)
        _write(args.jobs_output, observation.jobs)
        _write(args.receipt_output, observation.receipt())
        print(json.dumps(observation.receipt(), sort_keys=True))
        return 0 if observation.publish_subject_known else 2
    except (ObservationError, OSError, json.JSONDecodeError) as exc:
        print(
            json.dumps(
                {
                    "adapter_id": ADAPTER_ID,
                    "authority": AUTHORITY,
                    "publish_subject_known": False,
                    "reason": str(exc),
                    "grants_ci_pass": False,
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
