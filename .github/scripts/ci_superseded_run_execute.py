#!/usr/bin/env python3
"""CI-GOV-001D-B bounded executor. Dry-run by default; apply is explicit."""
from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from ci_queue_census import API_ROOT, API_VERSION, GitHubApiError, GitHubReadOnlyClient
from ci_superseded_run_plan import (
    ELIGIBLE,
    ELIGIBLE_WORKFLOW_PATH,
    PLAN_TTL_SECONDS,
    SCHEMA as PLAN_SCHEMA,
    association_numbers,
    canonical_commitment,
)

RECEIPT_SCHEMA = "mycelix-ci-superseded-run-cancellation-receipt-v1"
BATCH_SCHEMA = "mycelix-ci-superseded-run-cancellation-batch-v1"
MAX_SELECTED_RUNS = 5
LIVE_STATUSES = {"queued", "in_progress"}


class Refusal(RuntimeError):
    pass


def parse_time(value: str) -> datetime:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def entry_payload(entry: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "schema", "repository", "run_id", "run_attempt", "workflow_id",
        "workflow_name", "workflow_path", "event", "status", "queued_head_sha",
        "created_at", "associations", "classification",
    )
    return {key: entry.get(key) for key in keys}


def plan_payload(plan: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": plan["schema"],
        "repository": plan["repository"],
        "observed_at": plan["observed_at"],
        "expires_at": plan["expires_at"],
        "scope": plan["scope"],
        "entry_count": plan["entry_count"],
        "eligible_count": plan["eligible_count"],
        "all_entry_commitments": [e["plan_entry_commitment"] for e in plan["all_entries"]],
        "eligible_entry_commitments": [e["plan_entry_commitment"] for e in plan["eligible_entries"]],
    }


def validate_plan(plan: dict[str, Any], repository: str, now: datetime) -> dict[int, dict[str, Any]]:
    if plan.get("schema") != PLAN_SCHEMA or plan.get("repository") != repository:
        raise Refusal("planner schema/repository mismatch")
    exact_scope = {
        "status": "queued",
        "event": "pull_request",
        "workflow_path": ELIGIBLE_WORKFLOW_PATH,
        "runtime_workflow_override_allowed": False,
        "runtime_time_override_allowed": False,
        "envelope_contains_only_in_scope_runs": True,
        "mutation_authority": False,
        "executor_requires_full_plan_envelope": True,
        "plan_ttl_seconds": PLAN_TTL_SECONDS,
    }
    if plan.get("scope") != exact_scope:
        raise Refusal("plan scope differs from exact 001D-A v1 scope")
    observed = parse_time(str(plan.get("observed_at", "")))
    expires = parse_time(str(plan.get("expires_at", "")))
    now = now.astimezone(timezone.utc)
    if int((expires - observed).total_seconds()) != PLAN_TTL_SECONDS or now < observed or now > expires:
        raise Refusal("plan time envelope invalid or expired; replan required")

    all_entries = plan.get("all_entries")
    eligible_entries = plan.get("eligible_entries")
    if not isinstance(all_entries, list) or not isinstance(eligible_entries, list):
        raise Refusal("plan entries malformed")
    if plan.get("entry_count") != len(all_entries):
        raise Refusal("entry count mismatch")

    seen_ids: set[int] = set()
    seen_commitments: set[str] = set()
    eligible: dict[int, dict[str, Any]] = {}
    recomputed: list[dict[str, Any]] = []
    for entry in all_entries:
        if not isinstance(entry, dict):
            raise Refusal("non-object plan entry")
        run_id = entry.get("run_id")
        commitment = entry.get("plan_entry_commitment")
        if not isinstance(run_id, int) or run_id in seen_ids:
            raise Refusal("missing/duplicate run id")
        if not isinstance(commitment, str) or len(commitment) != 64 or commitment in seen_commitments:
            raise Refusal(f"run {run_id}: malformed/duplicate entry commitment")
        seen_ids.add(run_id)
        seen_commitments.add(commitment)
        if canonical_commitment(entry_payload(entry)) != commitment:
            raise Refusal(f"run {run_id}: entry commitment mismatch")
        flag = entry.get("eligible_for_cancellation_plan") is True
        classified = entry.get("classification") == ELIGIBLE
        if flag != classified:
            raise Refusal(f"run {run_id}: classification/eligibility conflict")
        if classified:
            recomputed.append(entry)
            eligible[run_id] = entry

    if plan.get("eligible_count") != len(recomputed):
        raise Refusal("eligible count mismatch")
    if [e.get("plan_entry_commitment") for e in eligible_entries] != [e["plan_entry_commitment"] for e in recomputed]:
        raise Refusal("eligible subset mismatch")
    if canonical_commitment(plan_payload(plan)) != plan.get("plan_commitment"):
        raise Refusal("plan commitment mismatch")
    return eligible


class GitHubCancellationClient:
    """Read access plus exactly one mutation endpoint: cancel one Actions run."""
    def __init__(self, repository: str, token: str) -> None:
        self.repository = repository
        self.token = token
        self.reader = GitHubReadOnlyClient(repository, token)

    def get_run(self, run_id: int) -> dict[str, Any]:
        payload = self.reader.get_json(f"/repos/{self.repository}/actions/runs/{run_id}")
        if not isinstance(payload, dict):
            raise RuntimeError(f"invalid run response for {run_id}")
        return payload

    def get_pull_request(self, number: int) -> dict[str, Any] | None:
        return self.reader.get_pull_request(number)

    def cancel_run(self, run_id: int) -> int:
        path = f"/repos/{self.repository}/actions/runs/{run_id}/cancel"
        request = urllib.request.Request(
            API_ROOT + path,
            headers={
                "Accept": "application/vnd.github+json",
                "Authorization": f"Bearer {self.token}",
                "User-Agent": "mycelix-ci-gov-superseded-run-executor/1",
                "X-GitHub-Api-Version": API_VERSION,
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                return int(response.status)
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise GitHubApiError(path, exc.code, body) from exc


def association_map(entry: dict[str, Any]) -> dict[int, dict[str, Any]]:
    items = entry.get("associations")
    if not isinstance(items, list) or not items:
        raise Refusal(f"run {entry.get('run_id')}: planned PR associations missing")
    out: dict[int, dict[str, Any]] = {}
    for item in items:
        if not isinstance(item, dict) or not isinstance(item.get("pr_number"), int):
            raise Refusal("malformed planned PR association")
        number = item["pr_number"]
        if number in out:
            raise Refusal("duplicate planned PR association")
        out[number] = item
    return out


def revalidate(entry: dict[str, Any], client: Any) -> dict[str, Any]:
    run_id = entry["run_id"]
    live = client.get_run(run_id)
    checks = (
        (live.get("id") == run_id, "live run id changed"),
        (live.get("run_attempt") == entry.get("run_attempt"), "run attempt changed"),
        (live.get("workflow_id") == entry.get("workflow_id"), "workflow id changed"),
        (live.get("path") == ELIGIBLE_WORKFLOW_PATH == entry.get("workflow_path"), "workflow path changed/out of scope"),
        (live.get("event") == "pull_request" == entry.get("event"), "event changed/out of scope"),
        (live.get("head_sha") == entry.get("queued_head_sha"), "queued head changed"),
        (live.get("status") in LIVE_STATUSES, "run status no longer cancellable"),
    )
    for ok, reason in checks:
        if not ok:
            raise Refusal(f"run {run_id}: {reason}")

    numbers, complete = association_numbers(live)
    planned = association_map(entry)
    if not complete or numbers != sorted(planned):
        raise Refusal(f"run {run_id}: PR association set changed/incomplete")

    live_prs = []
    for number in numbers:
        pr = client.get_pull_request(number)
        if not isinstance(pr, dict):
            raise Refusal(f"run {run_id}: PR #{number} metadata unresolved")
        current = (pr.get("head") or {}).get("sha")
        planned_item = planned[number]
        if pr.get("state") != "open" or not isinstance(current, str) or not current:
            raise Refusal(f"run {run_id}: PR #{number} no longer has complete open metadata")
        if current == live.get("head_sha"):
            raise Refusal(f"run {run_id}: PR #{number} points at queued head")
        if current != planned_item.get("current_head_sha"):
            raise Refusal(f"run {run_id}: PR #{number} advanced since plan; replan required")
        if planned_item.get("state") != "open" or planned_item.get("superseded") is not True:
            raise Refusal(f"run {run_id}: planned PR #{number} did not encode open supersession")
        live_prs.append({"pr_number": number, "current_head_sha": current})
    return {"live_status": live.get("status"), "live_prs": live_prs}


def finalize(receipt: dict[str, Any]) -> dict[str, Any]:
    out = dict(receipt)
    out.pop("receipt_commitment", None)
    out["receipt_commitment"] = canonical_commitment(out)
    return out


def execute(
    plan: dict[str, Any], selected: list[int], client: Any, repository: str,
    now: datetime, *, apply: bool, authority_ref: str, confirm_plan: str,
    receipt_sink: Callable[[dict[str, Any]], None] | None = None,
) -> dict[str, Any]:
    if not selected or len(selected) > MAX_SELECTED_RUNS or len(set(selected)) != len(selected):
        raise Refusal("select 1..5 unique run ids")
    if not authority_ref.strip():
        raise Refusal("non-empty authority reference required")
    eligible = validate_plan(plan, repository, now)
    if confirm_plan != plan.get("plan_commitment"):
        raise Refusal("exact plan commitment confirmation mismatch")
    for run_id in selected:
        if run_id not in eligible:
            raise Refusal(f"run {run_id} is not eligible in exact plan")

    batch = {
        "schema": BATCH_SCHEMA, "repository": repository,
        "plan_commitment": plan["plan_commitment"], "mode": "apply" if apply else "dry-run",
        "authority_ref": authority_ref, "selected_run_ids": selected,
        "stop_policy": "stop-on-first-refusal-or-mutation-error", "receipts": [],
        "outcome": "InProgress",
    }
    def emit() -> None:
        if receipt_sink:
            receipt_sink(batch)

    for run_id in selected:
        entry = eligible[run_id]
        base = {
            "schema": RECEIPT_SCHEMA, "repository": repository,
            "plan_commitment": plan["plan_commitment"],
            "plan_entry_commitment": entry["plan_entry_commitment"],
            "run_id": run_id, "run_attempt": entry["run_attempt"],
            "superseded_head_sha": entry["queued_head_sha"], "authority_ref": authority_ref,
            "mode": batch["mode"], "revalidated_at": now.astimezone(timezone.utc).isoformat().replace("+00:00", "Z"),
        }
        try:
            live = revalidate(entry, client)
        except Refusal as exc:
            base.update(outcome="RefusedAtRevalidation", reason=str(exc))
            batch["receipts"].append(finalize(base))
            batch["outcome"] = "StoppedOnRefusal"
            emit(); return batch

        base.update(pre_cancel_status=live["live_status"], successor_heads=live["live_prs"])
        if not apply:
            base["outcome"] = "DryRunEligibleNoMutation"
            batch["receipts"].append(finalize(base)); emit(); continue

        intent = dict(base); intent["outcome"] = "MutationIntentRecorded"
        batch["receipts"].append(finalize(intent)); emit()
        try:
            status = client.cancel_run(run_id)
        except GitHubApiError as exc:
            base.update(outcome="CancelRequestRejected", http_status=exc.status, reason=str(exc))
            batch["receipts"][-1] = finalize(base)
            batch["outcome"] = "StoppedOnMutationError"
            emit(); return batch
        except Exception as exc:
            base.update(outcome="CancelOutcomeUnknown", reason=f"{type(exc).__name__}: {exc}")
            batch["receipts"][-1] = finalize(base)
            batch["outcome"] = "StoppedOnMutationError"
            emit(); return batch

        base.update(outcome="CancelRequestAccepted", cancel_http_status=status)
        batch["receipts"][-1] = finalize(base); emit()

    batch["outcome"] = "ApplyRequestsAccepted" if apply else "DryRunComplete"
    emit(); return batch


def write_receipt(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    tmp.replace(path)
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    directory_fd = os.open(path.parent, flags)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("plan", type=Path)
    ap.add_argument("--repository", default=os.environ.get("GITHUB_REPOSITORY", ""))
    ap.add_argument("--token-env", default="GITHUB_TOKEN")
    ap.add_argument("--run-id", type=int, action="append", dest="run_ids", default=[])
    ap.add_argument("--confirm-plan", required=True)
    ap.add_argument("--authority-ref", required=True)
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--receipt-output", type=Path)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    if not args.repository:
        raise SystemExit("repository required")
    if args.apply and args.receipt_output is None:
        raise SystemExit("--apply requires --receipt-output")
    token = os.environ.get(args.token_env, "")
    if not token:
        raise SystemExit(f"{args.token_env} is required")
    plan = json.loads(args.plan.read_text())
    client = GitHubCancellationClient(args.repository, token)
    sink = (lambda payload: write_receipt(args.receipt_output, payload)) if args.receipt_output else None
    try:
        batch = execute(
            plan, args.run_ids, client, args.repository, datetime.now(timezone.utc),
            apply=args.apply, authority_ref=args.authority_ref,
            confirm_plan=args.confirm_plan, receipt_sink=sink,
        )
    except Refusal as exc:
        payload = {"schema": BATCH_SCHEMA, "outcome": "BatchRefusedBeforeExecution", "reason": str(exc)}
        if sink:
            sink(payload)
        print(json.dumps(payload, sort_keys=True))
        return 2
    print(json.dumps(batch, indent=2, sort_keys=True))
    return 3 if batch["outcome"] in {"StoppedOnRefusal", "StoppedOnMutationError"} else 0


if __name__ == "__main__":
    sys.exit(main())
