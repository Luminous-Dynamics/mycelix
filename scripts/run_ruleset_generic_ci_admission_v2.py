#!/usr/bin/env python3
"""Ruleset generic-CI admission router v2 with fresh PR metadata binding.

v1's scheduling/authority-completeness algebra is preserved. v2 removes the
assumption that the Actions pull_request event payload contains a reliable
changed_files count: it GETs the exact PR metadata from api.github.com first,
then passes that declared count into the bounded PR-files adapter.

If current PR metadata cannot be observed, every merge-gating required job is
still selected but authority_complete=false.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import socket
import sys
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import HTTPRedirectHandler, Request, build_opener

ROUTER_ID = "ruleset-generic-ci-admission-router-v2"
AUTHORITY = "RulesetSchedulingAuthority"
API_HOST = "api.github.com"
API_VERSION = "2022-11-28"
MAX_METADATA_BYTES = 2 * 1024 * 1024
REPO_RE = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")


class MetadataError(RuntimeError):
    pass


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def require(ok: bool, message: str) -> None:
    if not ok:
        raise MetadataError(message)


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    require(spec is not None and spec.loader is not None, f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def positive_int(value: Any, label: str) -> int:
    require(isinstance(value, int) and not isinstance(value, bool) and value > 0,
            f"{label} must be positive integer")
    return value


def safe_repo(value: Any) -> str:
    require(isinstance(value, str) and REPO_RE.fullmatch(value) is not None,
            "repository must be owner/name")
    return value


def default_get(url: str, token: str) -> tuple[int, bytes]:
    request = Request(
        url,
        method="GET",
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": API_VERSION,
            "User-Agent": ROUTER_ID,
        },
    )
    opener = build_opener(NoRedirect)
    try:
        with opener.open(request, timeout=15) as response:
            if response.geturl() != url:
                raise MetadataError("GitHub PR metadata redirect/final URL drift refused")
            body = response.read(MAX_METADATA_BYTES + 1)
            if len(body) > MAX_METADATA_BYTES:
                raise MetadataError("GitHub PR metadata exceeded byte bound")
            return getattr(response, "status", response.getcode()), body
    except HTTPError as exc:
        raise MetadataError(f"GitHub PR metadata HTTP error {exc.code}") from exc
    except (URLError, TimeoutError, socket.timeout) as exc:
        raise MetadataError(f"GitHub PR metadata transport error: {type(exc).__name__}") from exc


Fetch = Callable[[str, str], tuple[int, bytes]]


def fetch_pr_metadata(*, repository: Any, pr_number: Any, token: str,
                      fetch: Fetch = default_get) -> dict[str, Any]:
    repo = safe_repo(repository)
    number = positive_int(pr_number, "pull request number")
    require(isinstance(token, str) and token, "GitHub token missing")
    url = f"https://{API_HOST}/repos/{quote(repo, safe='/')}/pulls/{number}"
    status, body = fetch(url, token)
    require(status == 200, f"GitHub PR metadata returned unexpected status {status}")
    try:
        payload = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MetadataError("GitHub PR metadata is not valid UTF-8 JSON") from exc
    require(isinstance(payload, dict), "GitHub PR metadata root must be object")
    require(payload.get("number") == number, "GitHub PR metadata number mismatch")
    changed_files = payload.get("changed_files")
    positive_int(changed_files, "pull request changed_files")
    state = payload.get("state")
    require(state in {"open", "closed"}, "GitHub PR metadata state invalid")
    base = payload.get("base")
    head = payload.get("head")
    require(isinstance(base, dict) and isinstance(head, dict), "GitHub PR metadata refs missing")
    head_sha = head.get("sha")
    require(isinstance(head_sha, str) and re.fullmatch(r"[0-9a-f]{40}", head_sha) is not None,
            "GitHub PR metadata head SHA invalid")
    return {
        "number": number,
        "changed_files": changed_files,
        "state": state,
        "head_sha": head_sha,
    }


def event_pr_number(event: Any) -> int:
    require(isinstance(event, dict), "event root must be object")
    number = event.get("number")
    if isinstance(number, int) and not isinstance(number, bool) and number > 0:
        return number
    pr = event.get("pull_request")
    require(isinstance(pr, dict), "pull_request payload missing")
    return positive_int(pr.get("number"), "pull_request.number")


def metadata_indeterminate(v1: Any, manifest: dict[str, Any], reason: str):
    return v1.Decision(
        "AuthorityPrMetadataIndeterminate",
        False,
        True,
        v1.all_required(manifest),
        None,
        None,
        False,
        reason,
    )


def observe_and_decide_v2(*, event_name: str, event: Any, repository: str,
                          token: str, manifest: dict[str, Any], profile: dict[str, Any],
                          admission_module: Any, pr_files_module: Any, v1: Any,
                          metadata_fetch: Fetch = default_get):
    if event_name == "merge_group":
        return v1.decide(
            event_name=event_name,
            manifest=manifest,
            profile=profile,
            admission_module=admission_module,
            declared_changed_files=None,
            file_records=None,
            files_observable=None,
        )
    require(event_name == "pull_request", f"unsupported event {event_name!r}")
    try:
        number = event_pr_number(event)
        metadata = fetch_pr_metadata(
            repository=repository,
            pr_number=number,
            token=token,
            fetch=metadata_fetch,
        )
    except MetadataError as exc:
        return metadata_indeterminate(v1, manifest, str(exc))

    if metadata["state"] != "open":
        return metadata_indeterminate(v1, manifest, f"pull request state is {metadata['state']!r}")

    observation = pr_files_module.observe(
        repository=repository,
        pr_number=number,
        declared_changed_files=metadata["changed_files"],
        token=token,
    )
    return v1.decide(
        event_name="pull_request",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        declared_changed_files=metadata["changed_files"],
        file_records=observation.records,
        files_observable=observation.observable,
        files_reason=observation.reason,
    )


def fake_metadata(payload: dict[str, Any], status: int = 200):
    def fetch(url: str, token: str):
        assert url.startswith("https://api.github.com/repos/")
        assert token == "token"
        return status, json.dumps(payload).encode("utf-8")
    return fetch


class FakeFiles:
    def __init__(self, records: list[dict[str, str]] | None, observable: bool = True):
        self._records = records
        self._observable = observable

    def observe(self, *, repository: Any, pr_number: Any,
                declared_changed_files: Any, token: str):
        class Observation:
            pass
        out = Observation()
        out.observable = self._observable
        out.records = self._records
        out.reason = "fake complete" if self._observable else "fake unavailable"
        if self._observable:
            assert declared_changed_files == len(self._records or [])
        return out


def self_test(*, manifest: dict[str, Any], profile: dict[str, Any],
              admission_module: Any, v1: Any) -> None:
    payload = {
        "number": 7,
        "changed_files": 1,
        "state": "open",
        "head": {"sha": "a" * 40},
        "base": {"sha": "b" * 40},
    }
    event = {"number": 7, "pull_request": {"number": 7}}
    docs = observe_and_decide_v2(
        event_name="pull_request",
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        pr_files_module=FakeFiles([{"filename": "docs/lex-net/a.md", "status": "modified"}]),
        v1=v1,
        metadata_fetch=fake_metadata(payload),
    )
    assert docs.disposition == "GenericCiNotRequired" and docs.authority_complete

    unknown = observe_and_decide_v2(
        event_name="pull_request",
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        pr_files_module=FakeFiles([{"filename": "future/x.rs", "status": "added"}]),
        v1=v1,
        metadata_fetch=fake_metadata(payload),
    )
    assert unknown.disposition == "AdmitUnknown"
    assert unknown.authority_complete
    assert set(unknown.selected_jobs) == set(manifest["required_jobs"])

    broken = observe_and_decide_v2(
        event_name="pull_request",
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        pr_files_module=FakeFiles(None, False),
        v1=v1,
        metadata_fetch=lambda url, token: (503, b"{}"),
    )
    assert broken.disposition == "AuthorityPrMetadataIndeterminate"
    assert not broken.authority_complete
    assert set(broken.selected_jobs) == set(manifest["required_jobs"])

    closed_payload = dict(payload)
    closed_payload["state"] = "closed"
    closed = observe_and_decide_v2(
        event_name="pull_request",
        event=event,
        repository="Luminous-Dynamics/mycelix",
        token="token",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        pr_files_module=FakeFiles([], True),
        v1=v1,
        metadata_fetch=fake_metadata(closed_payload),
    )
    assert closed.disposition == "AuthorityPrMetadataIndeterminate"
    assert not closed.authority_complete

    merge = observe_and_decide_v2(
        event_name="merge_group",
        event={},
        repository="Luminous-Dynamics/mycelix",
        token="token",
        manifest=manifest,
        profile=profile,
        admission_module=admission_module,
        pr_files_module=FakeFiles([], True),
        v1=v1,
    )
    assert merge.disposition == "AdmitMergeGroupRequired"
    assert merge.authority_complete


def write_receipt(path: str | None, receipt: dict[str, Any]) -> None:
    if path:
        Path(path).write_text(json.dumps(receipt, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--v1", default="scripts/run_ruleset_generic_ci_admission_v1.py")
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
        v1 = load_module(Path(args.v1), "_ruleset_router_v1")
        admission_module = load_module(Path(args.admission_evaluator), "_ruleset_admission_v2")
        pr_files_module = load_module(Path(args.pr_files_adapter), "_ruleset_pr_files_v1")
        manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
        profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
        v1.validate_manifest(manifest, profile)
        admission_module.validate_profile(profile)

        if args.self_test:
            self_test(
                manifest=manifest,
                profile=profile,
                admission_module=admission_module,
                v1=v1,
            )
            print(json.dumps({
                "router_id": ROUTER_ID,
                "self_test": "PASS",
                "authority": AUTHORITY,
                "grants_product_qualification": False,
            }, sort_keys=True))
            return 0

        workflow_sha = v1.sha40(args.workflow_sha, "workflow_sha")
        target_sha = v1.sha40(args.target_sha, "target_sha")
        require(args.event_name in manifest["events"], "event not authorized by manifest")
        require(args.event_json, "event JSON path missing")
        event = json.loads(Path(args.event_json).read_text(encoding="utf-8"))

        decision = observe_and_decide_v2(
            event_name=args.event_name,
            event=event,
            repository=args.repository,
            token=args.token,
            manifest=manifest,
            profile=profile,
            admission_module=admission_module,
            pr_files_module=pr_files_module,
            v1=v1,
        )
        receipt = decision.receipt(
            workflow_sha=workflow_sha,
            target_sha=target_sha,
            event_name=args.event_name,
            manifest=manifest,
        )
        receipt["router_id"] = ROUTER_ID
        write_receipt(args.receipt_output, receipt)
        if args.github_output:
            with Path(args.github_output).open("a", encoding="utf-8") as handle:
                for line in v1.output_lines(decision, manifest):
                    handle.write(line + "\n")
        print(json.dumps(receipt, sort_keys=True))
        return 0
    except (MetadataError, OSError, json.JSONDecodeError) as exc:
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
