#!/usr/bin/env python3
"""Trusted bounded GitHub adapter for generic Mycelix CI admission v2.

Security model:
- execute only from a trusted checkout (PR base SHA, or pushed main SHA);
- verify frozen profile/evaluator/workflow Git blobs before using GITHUB_TOKEN;
- use read-only GET requests to api.github.com only;
- bound pages, response bytes, and wall-clock time;
- every operational/schema/policy uncertainty selects ALL generic-CI jobs;
- an unhandled adapter crash is also safe once the workflow uses fail-open
  downstream conditions when the admission root itself is non-success.

This adapter grants scheduling authority only. It cannot grant CI PASS, product
qualification, review approval, merge authority, or deployment authority.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable

PROFILE_PATH = Path("docs/ci/generic_ci_admission_v2.json")
EVALUATOR_PATH = Path("scripts/evaluate_generic_ci_admission_v2.py")
WORKFLOW_PATH = ".github/workflows/ci.yml"
EXPECTED_PROFILE_BLOB = "b8b1a212cbd98ebff3b52afd0eece5627d78fd7e"
EXPECTED_EVALUATOR_BLOB = "f2165a851d44085b73f027f51ee9037e249fc4f4"
EXPECTED_BASELINE_WORKFLOW_BLOB = "516b24f407370f669b4f6d1e7cbc9fcd7154aa69"
API_ROOT = "https://api.github.com"
API_VERSION = "2026-03-10"
PAGE_SIZE = 100
MAX_FILES = 3000
MAX_PAGES = 30
MAX_PAGE_BYTES = 16 * 1024 * 1024
TOTAL_API_SECONDS = 60.0
REQUEST_TIMEOUT_SECONDS = 10.0
REPO_RE = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
ALLOWED_FILE_STATUS = {"added", "removed", "modified", "renamed", "copied", "changed", "unchanged"}

JOB_TO_OUTPUT = {
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
OUTPUT_KEYS = tuple(JOB_TO_OUTPUT.values())


class AdapterError(RuntimeError):
    pass


def _git(*args: str) -> str:
    cp = subprocess.run(
        ["git", *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return cp.stdout.strip()


def _git_blob(path: str) -> str:
    return _git("rev-parse", f"HEAD:{path}")


def _load_event(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AdapterError("event payload root must be an object")
    return value


def _load_policy(*, verify_git_blobs: bool = True) -> tuple[dict[str, Any], Any]:
    if verify_git_blobs:
        if _git_blob(str(PROFILE_PATH)) != EXPECTED_PROFILE_BLOB:
            raise AdapterError("admission profile Git blob drift")
        if _git_blob(str(EVALUATOR_PATH)) != EXPECTED_EVALUATOR_BLOB:
            raise AdapterError("admission evaluator Git blob drift")

    profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))

    spec = importlib.util.spec_from_file_location("generic_ci_admission_v2", EVALUATOR_PATH)
    if spec is None or spec.loader is None:
        raise AdapterError("unable to construct evaluator import spec")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.validate_profile(profile)

    workflow = profile.get("workflow")
    if not isinstance(workflow, dict):
        raise AdapterError("profile workflow binding missing")
    bound_blob = workflow.get("blob_sha")
    if bound_blob != EXPECTED_BASELINE_WORKFLOW_BLOB:
        raise AdapterError("profile baseline workflow binding drift")
    if verify_git_blobs and _git_blob(WORKFLOW_PATH) != bound_blob:
        raise AdapterError("trusted checkout workflow blob differs from profile baseline")

    expected_jobs = tuple(profile.get("all_jobs") or ())
    if set(expected_jobs) != set(JOB_TO_OUTPUT):
        raise AdapterError("profile all_jobs differs from adapter output vocabulary")
    return profile, module


def _expected_trusted_sha(event_name: str, event: dict[str, Any]) -> str:
    if event_name == "pull_request":
        pr = event.get("pull_request")
        if not isinstance(pr, dict):
            raise AdapterError("pull_request payload missing")
        base = pr.get("base")
        if not isinstance(base, dict):
            raise AdapterError("pull_request.base missing")
        sha = base.get("sha")
    elif event_name == "push":
        sha = event.get("after")
    else:
        raise AdapterError(f"unsupported event {event_name!r}")
    if not isinstance(sha, str) or not re.fullmatch(r"[0-9a-f]{40}", sha):
        raise AdapterError("trusted event SHA is not lowercase 40-hex")
    return sha


def _verify_trusted_checkout(event_name: str, event: dict[str, Any]) -> str:
    expected = _expected_trusted_sha(event_name, event)
    actual = _git("rev-parse", "HEAD")
    if actual != expected:
        raise AdapterError(f"checkout trust mismatch: HEAD={actual} expected={expected}")
    if _git("status", "--porcelain=v1"):
        raise AdapterError("trusted policy checkout is dirty before admission")
    return actual


def _repository(event: dict[str, Any]) -> str:
    env_repo = os.environ.get("GITHUB_REPOSITORY", "")
    repo = event.get("repository")
    payload_repo = repo.get("full_name") if isinstance(repo, dict) else None
    if not isinstance(payload_repo, str) or not REPO_RE.fullmatch(payload_repo):
        raise AdapterError("event repository.full_name invalid")
    if env_repo != payload_repo or not REPO_RE.fullmatch(env_repo):
        raise AdapterError("GITHUB_REPOSITORY does not match event repository")
    return env_repo


def _pull_metadata(event: dict[str, Any]) -> tuple[int, int]:
    number = event.get("number")
    pr = event.get("pull_request")
    changed = pr.get("changed_files") if isinstance(pr, dict) else None
    if not isinstance(number, int) or isinstance(number, bool) or number <= 0:
        raise AdapterError("pull request number invalid")
    if not isinstance(changed, int) or isinstance(changed, bool) or changed <= 0:
        raise AdapterError("pull_request.changed_files must be a positive integer")
    return number, changed


def _http_page(repo: str, number: int, page: int, token: str, timeout: float) -> list[Any]:
    if not (1 <= page <= MAX_PAGES):
        raise AdapterError("page outside bounded range")
    url = f"{API_ROOT}/repos/{repo}/pulls/{number}/files?per_page={PAGE_SIZE}&page={page}"
    req = urllib.request.Request(
        url,
        method="GET",
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "User-Agent": "mycelix-ci-governance-admission-v2",
            "X-GitHub-Api-Version": API_VERSION,
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            if response.status != 200:
                raise AdapterError(f"GitHub API returned HTTP {response.status}")
            body = response.read(MAX_PAGE_BYTES + 1)
    except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError, OSError) as exc:
        raise AdapterError(f"GitHub PR-files request failed: {type(exc).__name__}") from exc
    if len(body) > MAX_PAGE_BYTES:
        raise AdapterError("GitHub PR-files page exceeds byte bound")
    try:
        value = json.loads(body)
    except json.JSONDecodeError as exc:
        raise AdapterError("GitHub PR-files response is invalid JSON") from exc
    if not isinstance(value, list) or len(value) > PAGE_SIZE:
        raise AdapterError("GitHub PR-files page schema/count invalid")
    return value


def _reduce_file_record(value: Any) -> dict[str, str]:
    if not isinstance(value, dict):
        raise AdapterError("GitHub PR-files record must be an object")
    filename = value.get("filename")
    status = value.get("status")
    if not isinstance(filename, str) or not filename:
        raise AdapterError("GitHub PR-files record filename invalid")
    if status not in ALLOWED_FILE_STATUS:
        raise AdapterError(f"GitHub PR-files record status unknown: {status!r}")
    out = {"filename": filename}
    previous = value.get("previous_filename")
    if status == "renamed" and not isinstance(previous, str):
        raise AdapterError("renamed record missing previous_filename")
    if previous is not None:
        if not isinstance(previous, str) or not previous:
            raise AdapterError("previous_filename invalid")
        out["previous_filename"] = previous
    return out


def _fetch_records(
    repo: str,
    number: int,
    declared: int,
    token: str,
    *,
    fetch_page: Callable[[str, int, int, str, float], list[Any]] = _http_page,
) -> list[dict[str, str]]:
    if declared > MAX_FILES:
        raise AdapterError("declared file count exceeds adapter fetch bound")
    started = time.monotonic()
    records: list[dict[str, str]] = []
    page = 1
    while len(records) < declared:
        if page > MAX_PAGES:
            raise AdapterError("pagination exceeded maximum page count")
        remaining = TOTAL_API_SECONDS - (time.monotonic() - started)
        if remaining <= 0:
            raise AdapterError("PR-files API wall-clock budget exhausted")
        raw = fetch_page(repo, number, page, token, min(REQUEST_TIMEOUT_SECONDS, remaining))
        if not raw:
            raise AdapterError("PR-files API ended before declared file count")
        records.extend(_reduce_file_record(item) for item in raw)
        if len(records) > declared:
            raise AdapterError("PR-files API returned more records than declared")
        if len(raw) < PAGE_SIZE and len(records) != declared:
            raise AdapterError("short PR-files page before declared file count")
        page += 1
    if len(records) != declared:
        raise AdapterError("declared/observed file count mismatch after pagination")
    return records


def _hard_fallback(disposition: str, reason: str) -> dict[str, Any]:
    return {
        "profile_id": "generic-mycelix-ci-admission-v2",
        "authority": "SchedulingOnly",
        "source": "AdapterFailClosed",
        "disposition": disposition,
        "selected_jobs": list(JOB_TO_OUTPUT),
        "generic_ci_required": True,
        "grants_ci_pass": False,
        "grants_product_qualification": False,
        "reason": reason,
    }


def _receipt_outputs(receipt: dict[str, Any]) -> dict[str, str]:
    selected = receipt.get("selected_jobs")
    if not isinstance(selected, list) or any(not isinstance(x, str) for x in selected):
        return _fallback_outputs("AdmissionProfileInvalid", "receipt selected_jobs invalid")
    selected_set = set(selected)
    if not selected_set.issubset(JOB_TO_OUTPUT):
        return _fallback_outputs("AdmissionProfileInvalid", "receipt selected_jobs outside adapter vocabulary")
    disposition = receipt.get("disposition")
    generic = receipt.get("generic_ci_required")
    if not isinstance(disposition, str) or not isinstance(generic, bool):
        return _fallback_outputs("AdmissionProfileInvalid", "receipt disposition/generic_ci_required invalid")
    out = {
        "disposition": disposition,
        "generic_ci_required": "true" if generic else "false",
    }
    for job, key in JOB_TO_OUTPUT.items():
        out[key] = "true" if job in selected_set else "false"
    return out


def _fallback_outputs(disposition: str, reason: str) -> dict[str, str]:
    out = {"disposition": disposition, "generic_ci_required": "true"}
    out.update({key: "true" for key in OUTPUT_KEYS})
    out["fallback_reason_sha256"] = hashlib.sha256(reason.encode("utf-8")).hexdigest()
    return out


def _emit(receipt: dict[str, Any], outputs: dict[str, str], output_path: str | None) -> None:
    canonical = json.dumps(receipt, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    outputs = dict(outputs)
    outputs["receipt_sha256"] = digest
    print(canonical)
    if output_path:
        with open(output_path, "a", encoding="utf-8") as fh:
            for key, value in outputs.items():
                if "\n" in key or "\n" in value or "\r" in key or "\r" in value:
                    raise AdapterError("multiline GitHub output forbidden")
                fh.write(f"{key}={value}\n")


def _classify(
    profile: dict[str, Any],
    module: Any,
    event_name: str,
    event: dict[str, Any],
    token: str,
    *,
    fetch_page: Callable[[str, int, int, str, float], list[Any]] = _http_page,
) -> dict[str, Any]:
    if event_name == "push":
        return module.evaluate(profile, "push", None, None).receipt(profile)
    if event_name != "pull_request":
        return _hard_fallback("AdmissionErrorFailClosed", f"unsupported event {event_name!r}")
    repo = _repository(event)
    number, declared = _pull_metadata(event)
    if declared > MAX_FILES:
        return module.evaluate(profile, "pull_request", declared, None).receipt(profile)
    if not token:
        return _hard_fallback("AdmissionErrorFailClosed", "GITHUB_TOKEN is empty")
    try:
        records = _fetch_records(repo, number, declared, token, fetch_page=fetch_page)
    except AdapterError as exc:
        return _hard_fallback("AdmissionErrorFailClosed", str(exc))
    return module.evaluate(profile, "pull_request", declared, records).receipt(profile)


def self_test() -> None:
    profile, module = _load_policy(verify_git_blobs=False)

    base = {
        "number": 9,
        "repository": {"full_name": "Luminous-Dynamics/mycelix"},
        "pull_request": {
            "changed_files": 1,
            "base": {"sha": "0" * 40},
        },
    }
    old_repo = os.environ.get("GITHUB_REPOSITORY")
    os.environ["GITHUB_REPOSITORY"] = "Luminous-Dynamics/mycelix"
    try:
        def docs_page(repo: str, number: int, page: int, token: str, timeout: float) -> list[Any]:
            assert repo == "Luminous-Dynamics/mycelix" and number == 9 and page == 1
            return [{"filename": "docs/lex-net/a.md", "status": "modified"}]

        receipt = _classify(profile, module, "pull_request", base, "test-token", fetch_page=docs_page)
        outputs = _receipt_outputs(receipt)
        assert receipt["disposition"] == "AdmissionSkipped"
        assert outputs["generic_ci_required"] == "false"
        assert all(outputs[key] == "false" for key in OUTPUT_KEYS)

        unknown = json.loads(json.dumps(base))
        unknown["pull_request"]["changed_files"] = 2
        def unknown_page(repo: str, number: int, page: int, token: str, timeout: float) -> list[Any]:
            return [
                {"filename": "mycelix-finance/src/lib.rs", "status": "modified"},
                {"filename": "future-shared/runtime/new.rs", "status": "added"},
            ]
        receipt = _classify(profile, module, "pull_request", unknown, "test-token", fetch_page=unknown_page)
        outputs = _receipt_outputs(receipt)
        assert receipt["disposition"] == "AdmitUnknown"
        assert all(outputs[key] == "true" for key in OUTPUT_KEYS)

        renamed = json.loads(json.dumps(base))
        def rename_page(repo: str, number: int, page: int, token: str, timeout: float) -> list[Any]:
            return [{
                "filename": "docs/lex-net/moved.md",
                "previous_filename": "mycelix-governance/src/lib.rs",
                "status": "renamed",
            }]
        receipt = _classify(profile, module, "pull_request", renamed, "test-token", fetch_page=rename_page)
        outputs = _receipt_outputs(receipt)
        assert receipt["disposition"] == "AdmitKnownRelevant"
        assert outputs["governance"] == "true" and outputs["format"] == "true"

        large = json.loads(json.dumps(base))
        large["pull_request"]["changed_files"] = 3001
        receipt = _classify(profile, module, "pull_request", large, "test-token", fetch_page=lambda *a: (_ for _ in ()).throw(AssertionError("must not fetch")))
        assert receipt["disposition"] == "AdmitUnknownLarge"
        assert all(_receipt_outputs(receipt)[key] == "true" for key in OUTPUT_KEYS)

        def broken_page(*args: Any, **kwargs: Any) -> list[Any]:
            raise AdapterError("synthetic API failure")
        receipt = _classify(profile, module, "pull_request", base, "test-token", fetch_page=broken_page)
        assert receipt["disposition"] == "AdmissionErrorFailClosed"
        assert all(_receipt_outputs(receipt)[key] == "true" for key in OUTPUT_KEYS)

        assert _fallback_outputs("AdmissionProfileInvalid", "x")["sdk"] == "true"
    finally:
        if old_repo is None:
            os.environ.pop("GITHUB_REPOSITORY", None)
        else:
            os.environ["GITHUB_REPOSITORY"] = old_repo


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        print(json.dumps({"adapter_self_test": "PASS", "api_version": API_VERSION}, sort_keys=True))
        return 0

    output_path = os.environ.get("GITHUB_OUTPUT")
    event_name = os.environ.get("GITHUB_EVENT_NAME", "")
    event_path = os.environ.get("GITHUB_EVENT_PATH", "")
    token = os.environ.get("GITHUB_TOKEN", "")

    try:
        if not event_path:
            raise AdapterError("GITHUB_EVENT_PATH is empty")
        event = _load_event(event_path)
        _verify_trusted_checkout(event_name, event)
        profile, module = _load_policy(verify_git_blobs=True)
        receipt = _classify(profile, module, event_name, event, token)
        outputs = _receipt_outputs(receipt)
    except Exception as exc:  # operational uncertainty must select more CI, never less
        reason = f"{type(exc).__name__}: {exc}"
        receipt = _hard_fallback("AdmissionProfileInvalid", reason)
        outputs = _fallback_outputs("AdmissionProfileInvalid", reason)

    _emit(receipt, outputs, output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
