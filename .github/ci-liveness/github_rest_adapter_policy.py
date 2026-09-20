#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import importlib.util
import tempfile
import unicodedata
from pathlib import Path
from typing import Any

POLICY_SCHEMA = "mycelix-github-rest-evidence-ci-core-adapter-policy-v1"
POLICY_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_ADAPTER_POLICY_V1\0"
IMPLEMENTATION_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_ADAPTER_POLICY_IMPL_V1\0"
CORE_ADAPTER_GIT_BLOB = "603fd2b77bc588701dbcdae376ae31f5b49ce13b"
CORE_ADAPTER_IMPLEMENTATION = "68fdfa639868b9300aad36cfa84dede8b2056cc280cd223d9e2d97a76cefa1bf"
SEMANTIC_CORE_HEAD = "4190f855eb0f3c03a7a6b0decee84dd7edba07b4"
MAX_REQUIRED_JOBS_V1 = 128
MAX_OBSERVED_JOBS_V1 = 512
CORE_JOB_STATUSES = {"Queued", "InProgress", "Completed", "Unknown"}
CORE_JOB_CONCLUSIONS = {None, "Success", "Failure", "Cancelled", "Skipped", "TimedOut", "ActionRequired", "Neutral", "StartupFailure", "Unknown"}
CORE_GATE_STATES = {"NoTheoremStepExecuted", "SomeTheoremStepsExecuted", "AllRegisteredTheoremStepsExecuted", "Unknown"}
CORE_DEPENDENCY_STATES = {"EligibleForRunner", "WaitingOnRequiredDependency", "DependencyFailed", "DependencySkipped", "Unknown"}
CORE_FAILURE_CLASSES = {"NotApplicable", "RegisteredTheoremGate", "RunnerInfrastructureBeforeTheoremGate", "Unknown"}

class PolicyError(ValueError):
    pass

_IMPL_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT_SHA256 = hashlib.sha256(IMPLEMENTATION_DOMAIN + _IMPL_BYTES).hexdigest()
del _IMPL_BYTES

def _git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

def _evidence_id(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value or len(value.encode()) > 256:
        raise PolicyError(f"{name} not EvidenceId-compatible")
    if any(unicodedata.category(ch) == "Cc" for ch in value):
        raise PolicyError(f"{name} contains control character")
    return value

def _load_core(path: Path):
    # Read the caller path exactly once, bind those bytes, and execute only a
    # private materialization of the verified object. This makes the executed
    # module independent of subsequent mutations to the caller-controlled path.
    data = path.read_bytes()
    if _git_blob(data) != CORE_ADAPTER_GIT_BLOB:
        raise PolicyError("adapter core blob mismatch")
    with tempfile.TemporaryDirectory(prefix="mycelix-ci-adapter-") as td:
        frozen = Path(td) / "github_rest_adapter.py"
        frozen.write_bytes(data)
        if frozen.read_bytes() != data or _git_blob(frozen.read_bytes()) != CORE_ADAPTER_GIT_BLOB:
            raise PolicyError("adapter core materialization mismatch")
        spec = importlib.util.spec_from_file_location("mycelix_github_rest_adapter_core", frozen)
        if spec is None or spec.loader is None:
            raise PolicyError("cannot load adapter core")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        post = frozen.read_bytes()
        if post != data or _git_blob(post) != CORE_ADAPTER_GIT_BLOB:
            raise PolicyError("adapter core changed during import")
    if getattr(mod, "IMPLEMENTATION_COMMITMENT_SHA256", None) != CORE_ADAPTER_IMPLEMENTATION:
        raise PolicyError("adapter core implementation mismatch")
    if getattr(mod, "SUPPORTED_CORE_HEAD", None) != SEMANTIC_CORE_HEAD:
        raise PolicyError("semantic core head mismatch")
    return mod

def _snapshot_consistent(run_raw: Any, jobs_raw: Any) -> None:
    if not isinstance(run_raw, dict):
        raise PolicyError("run must be object")
    rows = jobs_raw.get("jobs") if isinstance(jobs_raw, dict) else jobs_raw
    if not isinstance(rows, list):
        raise PolicyError("jobs must be list")
    rs = run_raw.get("status")
    states = [row.get("status") for row in rows if isinstance(row, dict)]
    if len(states) != len(rows):
        raise PolicyError("job must be object")
    if rs == "completed" and any(s != "completed" for s in states):
        raise PolicyError("terminal run has non-terminal job snapshot")
    if rs in {"queued", "waiting", "requested", "pending"} and any(s in {"in_progress", "completed"} for s in states):
        raise PolicyError("pre-start run has started/completed job snapshot")

def _u64(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not (0 <= value <= (1 << 64) - 1):
        raise PolicyError(f"{name} not u64-compatible")
    return value

def _validate_job(job: dict[str, Any]) -> None:
    required = {"job_id", "job_key", "status", "conclusion", "gate_execution", "dependency_state", "queue_age_seconds", "failure_class"}
    if set(job) != required:
        raise PolicyError("core job field mismatch")
    _u64(job.get("job_id"), "job_id")
    _evidence_id(job.get("job_key"), "job_key")
    status, conclusion = job.get("status"), job.get("conclusion")
    gate, dependency, failure = job.get("gate_execution"), job.get("dependency_state"), job.get("failure_class")
    queue_age = job.get("queue_age_seconds")
    if status not in CORE_JOB_STATUSES or conclusion not in CORE_JOB_CONCLUSIONS:
        raise PolicyError("unsupported core status/conclusion")
    if gate not in CORE_GATE_STATES or dependency not in CORE_DEPENDENCY_STATES or failure not in CORE_FAILURE_CLASSES:
        raise PolicyError("unsupported core enum value")
    if queue_age is not None:
        _u64(queue_age, "queue_age_seconds")
    if (status == "Completed") != (conclusion is not None):
        raise PolicyError("core job completion/conclusion mismatch")
    if status == "Queued" and gate in {"SomeTheoremStepsExecuted", "AllRegisteredTheoremStepsExecuted"}:
        raise PolicyError("queued core job claims theorem execution")
    if failure == "RegisteredTheoremGate":
        if status != "Completed" or conclusion != "Failure" or gate not in {"SomeTheoremStepsExecuted", "AllRegisteredTheoremStepsExecuted"}:
            raise PolicyError("invalid semantic-failure tuple")
    elif failure == "RunnerInfrastructureBeforeTheoremGate":
        if status != "Completed" or conclusion not in {"Failure", "Cancelled", "TimedOut", "StartupFailure"} or gate != "NoTheoremStepExecuted":
            raise PolicyError("invalid infrastructure-failure tuple")
    elif failure == "NotApplicable":
        if conclusion in {"Failure", "TimedOut", "StartupFailure"}:
            raise PolicyError("invalid NotApplicable tuple")
    elif failure == "Unknown":
        if status != "Completed":
            raise PolicyError("Unknown failure class requires completed job")
    else:
        raise PolicyError("unsupported failure class")

def _postprocess(result: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(result)
    if out.get("supported_core_head") != SEMANTIC_CORE_HEAD:
        raise PolicyError("result semantic core mismatch")
    manifest = out.get("core_manifest_v1")
    obs = out.get("core_observation_v1")
    if not isinstance(manifest, dict) or not isinstance(obs, dict):
        raise PolicyError("missing normalized core inputs")
    if set(manifest) != {"theorem_id", "repository_id", "workflow_path", "qualification_head", "required_jobs"}:
        raise PolicyError("core manifest field mismatch")
    if set(obs) != {"repository_id", "workflow_run_id", "qualification_head", "workflow_path", "jobs"}:
        raise PolicyError("core observation field mismatch")
    _evidence_id(manifest.get("theorem_id"), "theorem_id")
    _evidence_id(manifest.get("workflow_path"), "workflow_path")
    _evidence_id(manifest.get("qualification_head"), "qualification_head")
    _u64(manifest.get("repository_id"), "repository_id")
    _u64(obs.get("repository_id"), "repository_id")
    _u64(obs.get("workflow_run_id"), "workflow_run_id")
    _evidence_id(obs.get("workflow_path"), "workflow_path")
    _evidence_id(obs.get("qualification_head"), "qualification_head")
    if manifest["repository_id"] != obs["repository_id"] or manifest["workflow_path"] != obs["workflow_path"] or manifest["qualification_head"] != obs["qualification_head"]:
        raise PolicyError("manifest/observation identity mismatch")
    specs = manifest.get("required_jobs")
    if not isinstance(specs, list) or not (1 <= len(specs) <= MAX_REQUIRED_JOBS_V1):
        raise PolicyError("required theorem job count outside core bound")
    required_keys = []
    for spec in specs:
        if not isinstance(spec, dict) or set(spec) != {"job_key"}: raise PolicyError("invalid required job")
        required_keys.append(_evidence_id(spec.get("job_key"), "job_key"))
    if len(set(required_keys)) != len(required_keys):
        raise PolicyError("duplicate required job key")
    jobs = obs.get("jobs")
    if not isinstance(jobs, list) or len(jobs) > MAX_OBSERVED_JOBS_V1:
        raise PolicyError("observed jobs outside core bound")
    observed_keys = []
    for job in jobs:
        if not isinstance(job, dict): raise PolicyError("invalid core job")
        observed_keys.append(_evidence_id(job.get("job_key"), "job_key"))
        # The older exact adapter mapped GitHub `stale` to core Unknown while
        # still labeling it pre-theorem infrastructure. The pinned Rust core
        # rejects that tuple, so policy degrades it to Unknown/Unknown.
        if job.get("conclusion") == "Unknown" and job.get("failure_class") == "RunnerInfrastructureBeforeTheoremGate":
            job["failure_class"] = "Unknown"
        _validate_job(job)
    if len(set(observed_keys)) != len(observed_keys):
        raise PolicyError("duplicate observed job key")
    if any(key not in set(observed_keys) for key in required_keys):
        raise PolicyError("missing required observed job")
    for key in (
        "observation_source_authenticity_verified", "github_api_response_authenticity_verified",
        "runner_identity_attested", "semantic_classification_performed", "qualification_authority",
        "evidence_authority", "failover_authority", "rerun_authority", "dispatch_authority",
    ):
        if out.get(key) is not False:
            raise PolicyError(f"authority ceiling broadened: {key}")
    if out.get("qualification_result") is not None or out.get("theorem_result") is not None:
        raise PolicyError("semantic result surfaced by provider adapter")
    return out

def adapt(run_raw: Any, jobs_raw: Any, profile_raw: Any, observed_at_utc: str, core_path: Path | None = None) -> dict[str, Any]:
    _snapshot_consistent(run_raw, jobs_raw)
    path = core_path or Path(__file__).with_name("github_rest_adapter.py")
    core = _load_core(path)
    result = _postprocess(core.adapt(run_raw, jobs_raw, profile_raw, observed_at_utc))
    result["adapter_policy_schema"] = POLICY_SCHEMA
    result["adapter_policy_implementation_commitment_sha256"] = IMPLEMENTATION_COMMITMENT_SHA256
    result["adapter_core_git_blob_sha1"] = CORE_ADAPTER_GIT_BLOB
    result["adapter_core_contract_validated"] = True
    payload = {k: v for k, v in result.items() if k != "adapter_policy_commitment_sha256"}
    result["adapter_policy_commitment_sha256"] = hashlib.sha256(POLICY_DOMAIN + core.canonical_json(payload)).hexdigest()
    return result
