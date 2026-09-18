#!/usr/bin/env python3
"""Exact-head commit-status publisher for trusted generic CI authority.

Consumes only trusted authority/observation receipts. It never publishes if the
subject is stale, closed, unknown, or internally inconsistent. The status
context is stable: "Mycelix CI Authority".
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import HTTPRedirectHandler, Request, build_opener

PUBLISHER_ID = "generic-mycelix-ci-authority-status-v1"
AUTHORITY = "ExactHeadCommitStatusPublisher"
CONTEXT = "Mycelix CI Authority"
API_HOST = "api.github.com"
API_VERSION = "2022-11-28"
MAX_BODY_BYTES = 4 * 1024 * 1024
REPO_RE = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
SHA_RE = re.compile(r"^[0-9a-f]{40}$")

PASS = {"TrustedRequiredChecksPassed", "TrustedGenericCiNotRequired"}
FAIL = {
    "TrustedRequiredJobMissing",
    "TrustedRequiredJobNotSuccessful",
    "TrustedRunCancelledOrSuperseded",
}
FIXED_DESCRIPTION = {
    "TrustedRequiredChecksPassed": "Trusted expected generic CI checks passed",
    "TrustedGenericCiNotRequired": "Trusted policy: generic CI not required",
    "TrustedRequiredJobMissing": "Trusted required generic CI job is missing",
    "TrustedRequiredJobNotSuccessful": "Trusted required generic CI job did not pass",
    "TrustedRunCancelledOrSuperseded": "Trusted required generic CI execution was cancelled",
}


class PublishError(RuntimeError):
    pass


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


Fetch = Callable[[str, str], tuple[int, bytes]]
Post = Callable[[str, str, bytes], tuple[int, bytes]]


def _validate_repo(value: Any) -> str:
    if not isinstance(value, str) or not REPO_RE.fullmatch(value):
        raise PublishError("repository must be safe owner/name")
    return value


def _sha40(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise PublishError(f"{label} must be lowercase 40-hex SHA")
    return value


def _positive_int(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise PublishError(f"{label} must be positive integer")
    return value


def _request(url: str, token: str, *, method: str, body: bytes | None = None):
    headers = {
        "Accept": "application/vnd.github+json",
        "Authorization": f"Bearer {token}",
        "X-GitHub-Api-Version": API_VERSION,
        "User-Agent": PUBLISHER_ID,
    }
    if body is not None:
        headers["Content-Type"] = "application/json"
    request = Request(url, method=method, headers=headers, data=body)
    opener = build_opener(NoRedirect)
    try:
        with opener.open(request, timeout=15) as response:
            if response.geturl() != url:
                raise PublishError("GitHub API redirect/final URL drift refused")
            data = response.read(MAX_BODY_BYTES + 1)
            if len(data) > MAX_BODY_BYTES:
                raise PublishError("GitHub API response exceeded byte bound")
            return getattr(response, "status", response.getcode()), data
    except HTTPError as exc:
        raise PublishError(f"GitHub API HTTP error {exc.code}") from exc
    except (URLError, TimeoutError, socket.timeout) as exc:
        raise PublishError(f"GitHub API transport error: {type(exc).__name__}") from exc


def _default_fetch(url: str, token: str):
    return _request(url, token, method="GET")


def _default_post(url: str, token: str, body: bytes):
    return _request(url, token, method="POST", body=body)


@dataclass(frozen=True)
class Publication:
    disposition: str
    subject_sha: str | None
    state: str | None
    posted: bool
    reason: str
    target_url: str | None

    def receipt(self) -> dict[str, Any]:
        return {
            "publisher_id": PUBLISHER_ID,
            "authority": AUTHORITY,
            "context": CONTEXT,
            "disposition": self.disposition,
            "subject_sha": self.subject_sha,
            "state": self.state,
            "posted": self.posted,
            "reason": self.reason,
            "target_url": self.target_url,
            "grants_product_qualification": False,
        }


def _desired(authority: dict[str, Any]) -> tuple[str, str]:
    disposition = authority.get("disposition")
    if disposition in PASS:
        state = "success"
    elif disposition in FAIL:
        state = "failure"
    else:
        state = "error"
    description = FIXED_DESCRIPTION.get(
        disposition, "Trusted generic CI authority is indeterminate"
    )
    return state, description


def _validate_binding(
    authority: Any, observation: Any, repository: str
) -> tuple[str, int, str, str]:
    if not isinstance(authority, dict) or not isinstance(observation, dict):
        raise PublishError("authority/observation receipts must be objects")
    if observation.get("publish_subject_known") is not True:
        raise PublishError("current subject is not known")
    if observation.get("current_pr_state") != "open":
        raise PublishError("current pull request is not open")
    if authority.get("publish_authority") is not True:
        raise PublishError("authority receipt forbids publication")

    subject = _sha40(authority.get("subject_sha"), "authority subject_sha")
    event_head = _sha40(observation.get("event_pr_head_sha"), "event PR head SHA")
    current_head = _sha40(observation.get("current_pr_head_sha"), "current PR head SHA")
    if not (subject == event_head == current_head):
        raise PublishError("exact-head binding mismatch")

    run_id = _positive_int(authority.get("run_id"), "authority run_id")
    if observation.get("run_id") != run_id:
        raise PublishError("run_id mismatch")
    if observation.get("run_attempt") != authority.get("run_attempt"):
        raise PublishError("run_attempt mismatch")
    if authority.get("workflow_name") != observation.get("workflow_name"):
        raise PublishError("workflow name mismatch")
    if authority.get("workflow_path") != observation.get("workflow_path"):
        raise PublishError("workflow path mismatch")

    if authority.get("trusted_required_checks_passed") not in {True, False}:
        raise PublishError("trusted_required_checks_passed missing")
    state, description = _desired(authority)
    if (state == "success") != authority["trusted_required_checks_passed"]:
        raise PublishError("authority PASS boolean/disposition mismatch")

    return subject, run_id, state, description


def _existing_same(
    repo: str,
    subject: str,
    state: str,
    description: str,
    target_url: str,
    token: str,
    fetch: Fetch,
) -> bool:
    url = (
        f"https://{API_HOST}/repos/{quote(repo, safe='/')}/commits/"
        f"{subject}/status?per_page=100&page=1"
    )
    status, body = fetch(url, token)
    if status != 200:
        raise PublishError(f"combined status returned HTTP {status}")
    try:
        payload = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PublishError("combined status returned invalid JSON") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("statuses"), list):
        raise PublishError("combined status schema invalid")
    for item in payload["statuses"]:
        if not isinstance(item, dict):
            continue
        if str(item.get("context", "")).casefold() != CONTEXT.casefold():
            continue
        return (
            item.get("state") == state
            and item.get("description") == description
            and item.get("target_url") == target_url
        )
    return False


def publish(
    *,
    repository: Any,
    token: str,
    authority: Any,
    observation: Any,
    fetch: Fetch = _default_fetch,
    post: Post = _default_post,
) -> Publication:
    repo = _validate_repo(repository)
    if not isinstance(token, str) or not token:
        return Publication(
            "PublicationSuppressed", None, None, False, "token missing", None
        )

    try:
        subject, _, state, description = _validate_binding(
            authority, observation, repo
        )
    except PublishError as exc:
        return Publication(
            "PublicationSuppressed", None, None, False, str(exc), None
        )

    target_url = f"https://github.com/{repo}/actions/runs/{authority['run_id']}"
    try:
        if _existing_same(
            repo, subject, state, description, target_url, token, fetch
        ):
            return Publication(
                "AuthorityStatusAlreadyCurrent",
                subject,
                state,
                False,
                "latest authority status already matches desired state",
                target_url,
            )

        url = f"https://{API_HOST}/repos/{quote(repo, safe='/')}/statuses/{subject}"
        payload = json.dumps(
            {
                "state": state,
                "target_url": target_url,
                "description": description,
                "context": CONTEXT,
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        status, body = post(url, token, payload)
        if status != 201:
            raise PublishError(f"create commit status returned HTTP {status}")
        try:
            created = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise PublishError("create commit status returned invalid JSON") from exc
        if not isinstance(created, dict):
            raise PublishError("create commit status schema invalid")
        if str(created.get("context", "")).casefold() != CONTEXT.casefold():
            raise PublishError("created status context drift")
        if created.get("state") != state:
            raise PublishError("created status state drift")
        return Publication(
            "AuthorityStatusPublished",
            subject,
            state,
            True,
            "exact-head authority status published",
            target_url,
        )
    except PublishError as exc:
        return Publication(
            "AuthorityStatusPublicationError",
            subject,
            "error",
            False,
            str(exc),
            target_url,
        )


def self_test() -> None:
    sha = "a" * 40
    authority = {
        "disposition": "TrustedRequiredChecksPassed",
        "subject_sha": sha,
        "publish_authority": True,
        "run_id": 99,
        "run_attempt": 1,
        "workflow_name": "Mycelix CI",
        "workflow_path": ".github/workflows/ci.yml",
        "trusted_required_checks_passed": True,
    }
    observation = {
        "publish_subject_known": True,
        "current_pr_state": "open",
        "event_pr_head_sha": sha,
        "current_pr_head_sha": sha,
        "run_id": 99,
        "run_attempt": 1,
        "workflow_name": "Mycelix CI",
        "workflow_path": ".github/workflows/ci.yml",
    }
    posted = []

    def fetch_empty(url, token):
        assert token == "token"
        return 200, json.dumps({"state": "pending", "statuses": []}).encode()

    def post_ok(url, token, body):
        posted.append(json.loads(body))
        return 201, json.dumps(dict(posted[-1])).encode()

    result = publish(
        repository="Luminous-Dynamics/mycelix",
        token="token",
        authority=authority,
        observation=observation,
        fetch=fetch_empty,
        post=post_ok,
    )
    assert result.posted and result.state == "success"
    assert posted[-1]["context"] == CONTEXT

    failed = dict(authority)
    failed["disposition"] = "TrustedRequiredJobMissing"
    failed["trusted_required_checks_passed"] = False
    result = publish(
        repository="Luminous-Dynamics/mycelix",
        token="token",
        authority=failed,
        observation=observation,
        fetch=fetch_empty,
        post=post_ok,
    )
    assert result.posted and result.state == "failure"

    indeterminate = dict(authority)
    indeterminate["disposition"] = "TrustedJobsObservationIndeterminate"
    indeterminate["trusted_required_checks_passed"] = False
    result = publish(
        repository="Luminous-Dynamics/mycelix",
        token="token",
        authority=indeterminate,
        observation=observation,
        fetch=fetch_empty,
        post=post_ok,
    )
    assert result.posted and result.state == "error"

    stale = dict(observation)
    stale["current_pr_head_sha"] = "b" * 40
    result = publish(
        repository="Luminous-Dynamics/mycelix",
        token="token",
        authority=authority,
        observation=stale,
        fetch=fetch_empty,
        post=post_ok,
    )
    assert result.disposition == "PublicationSuppressed" and not result.posted

    closed = dict(observation)
    closed["current_pr_state"] = "closed"
    result = publish(
        repository="Luminous-Dynamics/mycelix",
        token="token",
        authority=authority,
        observation=closed,
        fetch=fetch_empty,
        post=post_ok,
    )
    assert result.disposition == "PublicationSuppressed" and not result.posted

    description = FIXED_DESCRIPTION["TrustedRequiredChecksPassed"]
    target = "https://github.com/Luminous-Dynamics/mycelix/actions/runs/99"

    def fetch_same(url, token):
        return 200, json.dumps(
            {
                "state": "success",
                "statuses": [
                    {
                        "context": CONTEXT,
                        "state": "success",
                        "description": description,
                        "target_url": target,
                    }
                ],
            }
        ).encode()

    result = publish(
        repository="Luminous-Dynamics/mycelix",
        token="token",
        authority=authority,
        observation=observation,
        fetch=fetch_same,
        post=post_ok,
    )
    assert result.disposition == "AuthorityStatusAlreadyCurrent"
    assert not result.posted


def read_json(path: str) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", default=os.getenv("GITHUB_REPOSITORY", ""))
    parser.add_argument("--token", default=os.getenv("GITHUB_TOKEN", ""))
    parser.add_argument("--authority-receipt")
    parser.add_argument("--observation-receipt")
    parser.add_argument("--receipt-output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        print(
            json.dumps(
                {
                    "publisher_id": PUBLISHER_ID,
                    "self_test": "PASS",
                    "authority": AUTHORITY,
                    "context": CONTEXT,
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 0

    try:
        if not args.authority_receipt or not args.observation_receipt:
            raise PublishError("authority and observation receipts are required")
        result = publish(
            repository=args.repository,
            token=args.token,
            authority=read_json(args.authority_receipt),
            observation=read_json(args.observation_receipt),
        )
        receipt = result.receipt()
        if args.receipt_output:
            Path(args.receipt_output).write_text(
                json.dumps(receipt, sort_keys=True) + "\n", encoding="utf-8"
            )
        print(json.dumps(receipt, sort_keys=True))
        if result.disposition in {
            "AuthorityStatusPublished",
            "AuthorityStatusAlreadyCurrent",
            "PublicationSuppressed",
        }:
            return 0
        return 2
    except (PublishError, OSError, json.JSONDecodeError) as exc:
        print(
            json.dumps(
                {
                    "publisher_id": PUBLISHER_ID,
                    "disposition": "AuthorityStatusPublicationError",
                    "reason": str(exc),
                    "posted": False,
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
