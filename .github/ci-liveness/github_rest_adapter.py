#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCHEMA = "mycelix-github-rest-evidence-ci-core-adapter-v1"
OUTPUT_SCHEMA = "mycelix-github-rest-evidence-ci-core-observation-v1"
PROFILE_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_PROFILE_V1\0"
OBSERVATION_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_OBSERVATION_V1\0"
IMPLEMENTATION_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_ADAPTER_V1\0"
MAX_INPUT_BYTES = 4 * 1024 * 1024
MAX_SAFE_INTEGER = (1 << 53) - 1
MAX_JOBS = 512
MAX_GATES_PER_JOB = 128
SUPPORTED_CORE_HEAD = "4190f855eb0f3c03a7a6b0decee84dd7edba07b4"

SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
REPOSITORY_RE = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
UTC_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z$")
RUN_STATUSES = {"queued", "in_progress", "completed", "waiting", "requested", "pending"}
JOB_STATUSES = {"queued", "in_progress", "completed", "waiting", "requested", "pending"}
STEP_STATUSES = {"queued", "in_progress", "completed", "waiting", "pending"}
CONCLUSIONS = {
    None,
    "success",
    "failure",
    "neutral",
    "cancelled",
    "skipped",
    "timed_out",
    "action_required",
    "stale",
    "startup_failure",
}
INFRA_TERMINALS = {"cancelled", "timed_out", "startup_failure", "stale"}


class AdapterError(ValueError):
    pass


_IMPLEMENTATION_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT_SHA256 = hashlib.sha256(
    IMPLEMENTATION_DOMAIN + _IMPLEMENTATION_BYTES
).hexdigest()
del _IMPLEMENTATION_BYTES


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in items:
        if key in out:
            raise AdapterError(f"duplicate JSON key: {key}")
        out[key] = value
    return out


def loads(text: str) -> Any:
    try:
        value = json.loads(text, object_pairs_hook=_pairs)
    except json.JSONDecodeError as exc:
        raise AdapterError(f"invalid JSON: {exc}") from exc
    _ijson(value, "json")
    return value


def _ijson(value: Any, context: str) -> None:
    if value is None or isinstance(value, (str, bool)):
        return
    if isinstance(value, int) and not isinstance(value, bool):
        if abs(value) > MAX_SAFE_INTEGER:
            raise AdapterError(f"{context} integer outside safe range")
        return
    if isinstance(value, float):
        raise AdapterError(f"{context} float not accepted")
    if isinstance(value, list):
        for i, item in enumerate(value):
            _ijson(item, f"{context}[{i}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise AdapterError(f"{context} key must be string")
            _ijson(item, f"{context}.{key}")
        return
    raise AdapterError(f"{context} unsupported value")


def canonical_json(value: Any) -> bytes:
    _ijson(value, "canonical")
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()


def _exact(obj: dict[str, Any], fields: set[str], context: str) -> None:
    if set(obj) != fields:
        raise AdapterError(f"{context} field mismatch")


def _text(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise AdapterError(f"{context} must be non-empty NUL-free string")
    return value


def _positive_int(value: Any, context: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not (0 < value <= MAX_SAFE_INTEGER):
        raise AdapterError(f"{context} must be positive safe integer")
    return value


def _path(value: Any, context: str) -> str:
    value = _text(value, context)
    if value.startswith("/") or value.endswith("/") or "\\" in value:
        raise AdapterError(f"{context} not canonical relative path")
    parts = value.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        raise AdapterError(f"{context} not canonical relative path")
    return value


def _timestamp(value: Any, context: str) -> datetime:
    value = _text(value, context)
    if not UTC_RE.fullmatch(value):
        raise AdapterError(f"{context} must be RFC3339 UTC Z")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise AdapterError(f"invalid {context}") from exc
    if parsed.tzinfo != timezone.utc:
        raise AdapterError(f"invalid {context}")
    return parsed


def _status(status: Any, conclusion: Any, allowed: set[str], context: str) -> tuple[str, str | None]:
    status = _text(status, f"{context}.status")
    if status not in allowed:
        raise AdapterError(f"unsupported {context} status: {status}")
    if conclusion not in CONCLUSIONS:
        raise AdapterError(f"unsupported {context} conclusion")
    if status != "completed" and conclusion is not None:
        raise AdapterError(f"{context} conclusion before completion")
    if status == "completed" and conclusion is None:
        raise AdapterError(f"{context} completed without conclusion")
    return status, conclusion


def normalize_profile(profile: Any) -> tuple[dict[str, Any], str]:
    if not isinstance(profile, dict):
        raise AdapterError("profile must be object")
    _exact(profile, {
        "schema", "profile_id", "repository", "repository_id", "theorem_id",
        "qualification_head", "workflow_path", "required_jobs",
    }, "profile")
    if profile["schema"] != SCHEMA:
        raise AdapterError("unsupported profile schema")
    profile_id = _text(profile["profile_id"], "profile_id")
    repository = _text(profile["repository"], "repository")
    if not REPOSITORY_RE.fullmatch(repository):
        raise AdapterError("repository must be owner/name")
    repository_id = _positive_int(profile["repository_id"], "repository_id")
    theorem_id = _text(profile["theorem_id"], "theorem_id")
    head = _text(profile["qualification_head"], "qualification_head")
    if not SHA1_RE.fullmatch(head):
        raise AdapterError("qualification_head must be lowercase SHA-1")
    workflow_path = _path(profile["workflow_path"], "workflow_path")
    raw_jobs = profile["required_jobs"]
    if not isinstance(raw_jobs, list) or not raw_jobs or len(raw_jobs) > MAX_JOBS:
        raise AdapterError("required_jobs out of bounds")
    jobs: list[dict[str, Any]] = []
    keys: set[str] = set()
    names: set[str] = set()
    for raw in raw_jobs:
        if not isinstance(raw, dict):
            raise AdapterError("job profile must be object")
        _exact(raw, {"job_key", "observed_name", "required_gate_names", "depends_on"}, "job profile")
        key = _text(raw["job_key"], "job_key")
        name = _text(raw["observed_name"], "observed_name")
        if key in keys or name in names:
            raise AdapterError("duplicate job key/name")
        keys.add(key); names.add(name)
        gates = raw["required_gate_names"]
        deps = raw["depends_on"]
        if not isinstance(gates, list) or not gates or len(gates) > MAX_GATES_PER_JOB:
            raise AdapterError("required_gate_names out of bounds")
        if not isinstance(deps, list):
            raise AdapterError("depends_on must be list")
        gates = [_text(g, "gate") for g in gates]
        deps = [_text(d, "dependency") for d in deps]
        if len(set(gates)) != len(gates) or len(set(deps)) != len(deps) or key in deps:
            raise AdapterError("duplicate/self gate/dependency")
        jobs.append({"job_key": key, "observed_name": name, "required_gate_names": sorted(gates), "depends_on": sorted(deps)})
    jobs.sort(key=lambda j: j["job_key"])
    known = {j["job_key"] for j in jobs}
    graph = {j["job_key"]: [d for d in j["depends_on"] if d in known] for j in jobs}
    visiting: set[str] = set(); done: set[str] = set()
    def visit(k: str) -> None:
        if k in done: return
        if k in visiting: raise AdapterError("cycle in required dependency graph")
        visiting.add(k)
        for d in graph[k]: visit(d)
        visiting.remove(k); done.add(k)
    for j in jobs:
        if any(dep not in known for dep in j["depends_on"]):
            raise AdapterError("dependency must name a required job key")
    for k in sorted(known): visit(k)
    normalized = {
        "schema": SCHEMA, "profile_id": profile_id, "repository": repository,
        "repository_id": repository_id, "theorem_id": theorem_id,
        "qualification_head": head, "workflow_path": workflow_path,
        "required_jobs": jobs,
    }
    commitment = hashlib.sha256(PROFILE_DOMAIN + canonical_json(normalized)).hexdigest()
    return normalized, commitment


def _project_run(raw: Any, profile: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(raw, dict): raise AdapterError("run must be object")
    for k in ("id", "head_sha", "path", "status", "conclusion", "repository"):
        if k not in raw: raise AdapterError(f"run missing {k}")
    rid = _positive_int(raw["id"], "run.id")
    head = _text(raw["head_sha"], "run.head_sha")
    path = _path(raw["path"], "run.path")
    if head != profile["qualification_head"] or path != profile["workflow_path"]:
        raise AdapterError("run identity mismatch")
    repo = raw["repository"]
    if not isinstance(repo, dict) or repo.get("id") != profile["repository_id"] or repo.get("full_name") != profile["repository"]:
        raise AdapterError("repository identity mismatch")
    status, conclusion = _status(raw["status"], raw["conclusion"], RUN_STATUSES, "run")
    return {"id": rid, "head_sha": head, "path": path, "status": status, "conclusion": conclusion}


def _runner_assignment(runner_id: Any, runner_name: Any) -> str:
    if runner_id in (0, None) and runner_name in ("", None):
        if (runner_id == 0 and runner_name == "") or (runner_id is None and runner_name is None):
            return "NotObserved"
    if isinstance(runner_id, int) and not isinstance(runner_id, bool) and 0 < runner_id <= MAX_SAFE_INTEGER and isinstance(runner_name, str) and runner_name:
        return "Observed"
    return "Unknown"


def _step(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict): raise AdapterError("step must be object")
    for k in ("name", "status", "conclusion", "started_at"):
        if k not in raw: raise AdapterError(f"step missing {k}")
    name = _text(raw["name"], "step.name")
    status, conclusion = _status(raw["status"], raw["conclusion"], STEP_STATUSES, "step")
    started = raw["started_at"]
    if started is not None: _timestamp(started, "step.started_at")
    return {"name": name, "status": status, "conclusion": conclusion, "started_at": started}


def _project_jobs(raw: Any, run: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows = raw.get("jobs") if isinstance(raw, dict) else raw
    if not isinstance(rows, list) or len(rows) > MAX_JOBS:
        raise AdapterError("jobs out of bounds")
    out: dict[str, dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, dict): raise AdapterError("job must be object")
        for k in ("id", "name", "status", "conclusion", "steps", "runner_id", "runner_name"):
            if k not in row: raise AdapterError(f"job missing {k}")
        jid = _positive_int(row["id"], "job.id")
        name = _text(row["name"], "job.name")
        if name in out: raise AdapterError("duplicate observed job name")
        if "run_id" in row and row["run_id"] != run["id"]: raise AdapterError("job run_id mismatch")
        if "head_sha" in row and row["head_sha"] != run["head_sha"]: raise AdapterError("job head_sha mismatch")
        status, conclusion = _status(row["status"], row["conclusion"], JOB_STATUSES, "job")
        raw_steps = row["steps"]
        if raw_steps is None:
            steps = None
        elif isinstance(raw_steps, list):
            steps = [_step(x) for x in raw_steps]
            if len({x["name"] for x in steps}) != len(steps): raise AdapterError("duplicate step name")
        else: raise AdapterError("steps must be list or null")
        created_at = row.get("created_at")
        if created_at is not None: _timestamp(created_at, "job.created_at")
        out[name] = {
            "id": jid, "name": name, "status": status, "conclusion": conclusion,
            "steps": steps, "runner_assignment": _runner_assignment(row["runner_id"], row["runner_name"]),
            "created_at": created_at,
        }
    return out


def _gate_state(job: dict[str, Any], gates: list[str]) -> tuple[str, bool | None]:
    steps = job["steps"]
    if steps is None: return "Unknown", None
    by_name = {s["name"]: s for s in steps}
    if any(g not in by_name for g in gates):
        return ("NoTheoremStepExecuted", None) if not steps else ("Unknown", None)
    selected = [by_name[g] for g in gates]
    started = [s for s in selected if s["started_at"] is not None or s["status"] in {"in_progress", "completed"}]
    if not started: return "NoTheoremStepExecuted", None
    if all(s["status"] == "completed" for s in selected):
        return "AllRegisteredTheoremStepsExecuted", all(s["conclusion"] == "success" for s in selected)
    return "SomeTheoremStepsExecuted", None


def _dependency_state(spec: dict[str, Any], by_key: dict[str, dict[str, Any]]) -> str:
    if not spec["depends_on"]: return "EligibleForRunner"
    states = []
    for dep in spec["depends_on"]:
        job = by_key.get(dep)
        if job is None: return "Unknown"
        if job["status"] != "completed": states.append("waiting")
        elif job["conclusion"] == "success": states.append("ok")
        elif job["conclusion"] == "skipped": states.append("skipped")
        else: states.append("failed")
    if "failed" in states: return "DependencyFailed"
    if "skipped" in states: return "DependencySkipped"
    if "waiting" in states: return "WaitingOnRequiredDependency"
    return "EligibleForRunner"


def _job_status(status: str) -> str:
    return {"queued":"Queued", "waiting":"Queued", "requested":"Queued", "pending":"Queued", "in_progress":"InProgress", "completed":"Completed"}[status]


def _job_conclusion(value: str | None) -> str | None:
    if value is None: return None
    return {
        "success":"Success", "failure":"Failure", "cancelled":"Cancelled", "skipped":"Skipped",
        "timed_out":"TimedOut", "action_required":"ActionRequired", "neutral":"Neutral",
        "startup_failure":"StartupFailure", "stale":"Unknown",
    }[value]


def _failure_class(job: dict[str, Any], gate_state: str, all_gates_success: bool | None) -> str:
    if job["status"] != "completed": return "NotApplicable"
    c = job["conclusion"]
    if c == "success" and gate_state == "AllRegisteredTheoremStepsExecuted" and all_gates_success is True:
        return "NotApplicable"
    if c == "failure" and gate_state in {"SomeTheoremStepsExecuted", "AllRegisteredTheoremStepsExecuted"} and all_gates_success is False:
        return "RegisteredTheoremGate"
    if c in INFRA_TERMINALS and gate_state == "NoTheoremStepExecuted":
        return "RunnerInfrastructureBeforeTheoremGate"
    return "Unknown"


def _provider_diagnostic(job: dict[str, Any], gate_state: str) -> str:
    if job["status"] in {"queued", "waiting", "requested", "pending"} and gate_state == "NoTheoremStepExecuted":
        return "QueuedNoTheoremStart"
    if job["status"] == "in_progress": return "Running"
    if job["status"] == "completed" and job["conclusion"] == "cancelled" and gate_state == "NoTheoremStepExecuted":
        if job["runner_assignment"] == "NotObserved": return "TerminalCancelledNoStart"
        if job["runner_assignment"] == "Observed": return "TerminalCancelledBeforeTheorem"
        return "TerminalCancelledStartUnknown"
    if job["status"] == "completed" and job["conclusion"] in {"startup_failure", "timed_out", "stale"} and gate_state == "NoTheoremStepExecuted":
        return "TerminalInfrastructureNoStart"
    if job["status"] == "completed" and gate_state in {"SomeTheoremStepsExecuted", "AllRegisteredTheoremStepsExecuted"}:
        return "TerminalAfterTheoremStart"
    return "Indeterminate"


def adapt(run_raw: Any, jobs_raw: Any, profile_raw: Any, observed_at_utc: str) -> dict[str, Any]:
    observed_at = _timestamp(observed_at_utc, "observed_at_utc")
    profile, profile_commitment = normalize_profile(profile_raw)
    run = _project_run(run_raw, profile)
    jobs = _project_jobs(jobs_raw, run)
    by_key: dict[str, dict[str, Any]] = {}
    for spec in profile["required_jobs"]:
        observed = jobs.get(spec["observed_name"])
        if observed is None: raise AdapterError(f"missing required job: {spec['observed_name']}")
        by_key[spec["job_key"]] = observed

    core_jobs = []
    diagnostics = []
    for spec in profile["required_jobs"]:
        job = by_key[spec["job_key"]]
        gate_state, all_gate_success = _gate_state(job, spec["required_gate_names"])
        dep_state = _dependency_state(spec, by_key)
        queue_age = None
        if job["created_at"] is not None and job["status"] in {"queued", "waiting", "requested", "pending"}:
            age = int((observed_at - _timestamp(job["created_at"], "job.created_at")).total_seconds())
            if age < 0: raise AdapterError("observation precedes job creation")
            queue_age = age
        failure = _failure_class(job, gate_state, all_gate_success)
        core_jobs.append({
            "job_id": job["id"], "job_key": spec["job_key"], "status": _job_status(job["status"]),
            "conclusion": _job_conclusion(job["conclusion"]), "gate_execution": gate_state,
            "dependency_state": dep_state, "queue_age_seconds": queue_age, "failure_class": failure,
        })
        diagnostics.append({
            "job_key": spec["job_key"], "observed_name": spec["observed_name"],
            "runner_assignment": job["runner_assignment"], "step_metadata": "Missing" if job["steps"] is None else "Present",
            "provider_state": _provider_diagnostic(job, gate_state),
        })

    core_manifest = {
        "theorem_id": profile["theorem_id"], "repository_id": profile["repository_id"],
        "workflow_path": profile["workflow_path"], "qualification_head": profile["qualification_head"],
        "required_jobs": [{"job_key": j["job_key"]} for j in profile["required_jobs"]],
    }
    core_observation = {
        "repository_id": profile["repository_id"], "workflow_run_id": run["id"],
        "qualification_head": run["head_sha"], "workflow_path": run["path"], "jobs": core_jobs,
    }
    receipt = {
        "schema": OUTPUT_SCHEMA, "adapter_profile_id": profile["profile_id"],
        "adapter_profile_commitment_sha256": profile_commitment,
        "adapter_implementation_commitment_sha256": IMPLEMENTATION_COMMITMENT_SHA256,
        "supported_core_head": SUPPORTED_CORE_HEAD,
        "observed_at_utc": observed_at_utc, "core_manifest_v1": core_manifest,
        "core_observation_v1": core_observation, "provider_diagnostics": diagnostics,
        "observation_source_authenticity_verified": False,
        "github_api_response_authenticity_verified": False,
        "runner_identity_attested": False,
        "semantic_classification_performed": False,
        "qualification_result": None, "theorem_result": None,
        "qualification_authority": False, "evidence_authority": False,
        "failover_authority": False, "rerun_authority": False, "dispatch_authority": False,
    }
    return {**receipt, "adapter_observation_commitment_sha256": hashlib.sha256(OBSERVATION_DOMAIN + canonical_json(receipt)).hexdigest()}


def read_json(path: str) -> Any:
    p = Path(path)
    if p.stat().st_size > MAX_INPUT_BYTES: raise AdapterError("input too large")
    return loads(p.read_text())
