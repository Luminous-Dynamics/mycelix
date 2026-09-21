#!/usr/bin/env python3
"""Generate a deterministic Rust harness for EVIDENCE-CI V1 frozen vectors."""

from __future__ import annotations

import argparse
import json
import sys
import unicodedata
from pathlib import Path

SCHEMA = "mycelix-evidence-ci-core-adapter-vectors-v1"
CORE_HEAD = "4190f855eb0f3c03a7a6b0decee84dd7edba07b4"
U64_MAX = (1 << 64) - 1

TOP_KEYS = {"schema", "supported_core_head", "vectors"}
VECTOR_KEYS = {
    "id", "core_manifest_v1", "core_observation_v1",
    "expected_core_liveness", "expected_provider_state",
}
MANIFEST_KEYS = {
    "theorem_id", "repository_id", "workflow_path",
    "qualification_head", "required_jobs",
}
REQUIRED_JOB_KEYS = {"job_key"}
OBS_KEYS = {
    "repository_id", "workflow_run_id", "qualification_head",
    "workflow_path", "jobs",
}
JOB_KEYS = {
    "job_id", "job_key", "status", "conclusion", "gate_execution",
    "dependency_state", "queue_age_seconds", "failure_class",
}

STATUS = {"Queued", "InProgress", "Completed", "Unknown"}
CONCLUSION = {
    "Success", "Failure", "Cancelled", "Skipped", "TimedOut",
    "ActionRequired", "Neutral", "StartupFailure", "Unknown",
}
GATE = {
    "NoTheoremStepExecuted", "SomeTheoremStepsExecuted",
    "AllRegisteredTheoremStepsExecuted", "Unknown",
}
DEPENDENCY = {
    "EligibleForRunner", "WaitingOnRequiredDependency",
    "DependencyFailed", "DependencySkipped", "Unknown",
}
FAILURE = {
    "NotApplicable", "RegisteredTheoremGate",
    "RunnerInfrastructureBeforeTheoremGate", "Unknown",
}
LIVENESS = {
    "AllRequiredJobsNoStart", "RequiredDependencyBlocked",
    "PartiallyExecutedAwaitingRequiredJob", "RequiredJobExecuting",
    "CompletedConjunctivePass", "CompletedConjunctiveFail",
    "InfrastructureInterrupted", "Indeterminate",
}

def fail(message: str) -> "None":
    raise ValueError(message)

def exact_keys(value: dict, expected: set[str], where: str) -> None:
    if set(value) != expected:
        fail(f"{where}: fields differ: got={sorted(value)} expected={sorted(expected)}")

def u64(value: object, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not (0 <= value <= U64_MAX):
        fail(f"{where}: expected u64")
    return value

def evidence_id(value: object, where: str) -> str:
    if not isinstance(value, str):
        fail(f"{where}: expected string")
    if not value or len(value.encode("utf-8")) > 256:
        fail(f"{where}: invalid EvidenceId length")
    if any(unicodedata.category(ch) == "Cc" for ch in value):
        fail(f"{where}: control character")
    return value

def enum(value: object, allowed: set[str], where: str) -> str:
    if not isinstance(value, str) or value not in allowed:
        fail(f"{where}: unsupported enum {value!r}")
    return value

def rust_string(value: str) -> str:
    # JSON string escaping is compatible with these reviewed ASCII EvidenceIds.
    if any(ord(ch) > 0x7F for ch in value):
        fail("non-ASCII EvidenceId is outside generator V1")
    return json.dumps(value, ensure_ascii=True)

def validate(data: object) -> dict:
    if not isinstance(data, dict):
        fail("root: expected object")
    exact_keys(data, TOP_KEYS, "root")
    if data["schema"] != SCHEMA:
        fail("root: unsupported schema")
    if data["supported_core_head"] != CORE_HEAD:
        fail("root: unsupported core head")
    vectors = data["vectors"]
    if not isinstance(vectors, list) or not vectors:
        fail("root: vectors must be a non-empty array")

    seen_vector_ids: set[str] = set()
    for i, vector in enumerate(vectors):
        where = f"vectors[{i}]"
        if not isinstance(vector, dict):
            fail(f"{where}: expected object")
        exact_keys(vector, VECTOR_KEYS, where)
        vector_id = evidence_id(vector["id"], f"{where}.id")
        if vector_id in seen_vector_ids:
            fail(f"{where}: duplicate vector id")
        seen_vector_ids.add(vector_id)
        enum(vector["expected_core_liveness"], LIVENESS, f"{where}.expected_core_liveness")
        evidence_id(vector["expected_provider_state"], f"{where}.expected_provider_state")

        manifest = vector["core_manifest_v1"]
        if not isinstance(manifest, dict):
            fail(f"{where}.core_manifest_v1: expected object")
        exact_keys(manifest, MANIFEST_KEYS, f"{where}.core_manifest_v1")
        evidence_id(manifest["theorem_id"], f"{where}.manifest.theorem_id")
        repo_id = u64(manifest["repository_id"], f"{where}.manifest.repository_id")
        workflow_path = evidence_id(manifest["workflow_path"], f"{where}.manifest.workflow_path")
        head = evidence_id(manifest["qualification_head"], f"{where}.manifest.qualification_head")
        required = manifest["required_jobs"]
        if not isinstance(required, list) or not (1 <= len(required) <= 128):
            fail(f"{where}.manifest.required_jobs: invalid count")
        required_keys: list[str] = []
        for j, spec in enumerate(required):
            if not isinstance(spec, dict):
                fail(f"{where}.manifest.required_jobs[{j}]: expected object")
            exact_keys(spec, REQUIRED_JOB_KEYS, f"{where}.manifest.required_jobs[{j}]")
            required_keys.append(evidence_id(spec["job_key"], f"{where}.manifest.required_jobs[{j}].job_key"))
        if len(set(required_keys)) != len(required_keys):
            fail(f"{where}.manifest.required_jobs: duplicate job_key")

        obs = vector["core_observation_v1"]
        if not isinstance(obs, dict):
            fail(f"{where}.core_observation_v1: expected object")
        exact_keys(obs, OBS_KEYS, f"{where}.core_observation_v1")
        if u64(obs["repository_id"], f"{where}.obs.repository_id") != repo_id:
            fail(f"{where}: repository mismatch")
        if evidence_id(obs["workflow_path"], f"{where}.obs.workflow_path") != workflow_path:
            fail(f"{where}: workflow path mismatch")
        if evidence_id(obs["qualification_head"], f"{where}.obs.qualification_head") != head:
            fail(f"{where}: qualification head mismatch")
        u64(obs["workflow_run_id"], f"{where}.obs.workflow_run_id")
        jobs = obs["jobs"]
        if not isinstance(jobs, list) or len(jobs) > 512:
            fail(f"{where}.obs.jobs: invalid count")
        observed: dict[str, int] = {}
        for j, job in enumerate(jobs):
            jw = f"{where}.obs.jobs[{j}]"
            if not isinstance(job, dict):
                fail(f"{jw}: expected object")
            exact_keys(job, JOB_KEYS, jw)
            job_key = evidence_id(job["job_key"], f"{jw}.job_key")
            if job_key in observed:
                fail(f"{jw}: duplicate job_key")
            observed[job_key] = u64(job["job_id"], f"{jw}.job_id")
            enum(job["status"], STATUS, f"{jw}.status")
            if job["conclusion"] is not None:
                enum(job["conclusion"], CONCLUSION, f"{jw}.conclusion")
            enum(job["gate_execution"], GATE, f"{jw}.gate_execution")
            enum(job["dependency_state"], DEPENDENCY, f"{jw}.dependency_state")
            enum(job["failure_class"], FAILURE, f"{jw}.failure_class")
            if job["queue_age_seconds"] is not None:
                u64(job["queue_age_seconds"], f"{jw}.queue_age_seconds")
        missing = [key for key in required_keys if key not in observed]
        if missing:
            fail(f"{where}: missing required jobs {missing}")
    return data

def render(data: dict) -> str:
    validate(data)
    lines = [
        "use mycelix_evidence_ci_core::{",
        "    classify_liveness_v1, derive_conjunctive_receipt_v1,",
        "    ConjunctiveVerdictV1, DependencyStateV1, EvidenceId, FailureClassV1,",
        "    GateExecutionStateV1, JobConclusionV1, JobObservationV1, JobStatusV1,",
        "    RequiredJobManifestV1, RequiredJobSpecV1, RunLivenessV1, WorkflowRunObservationV1,",
        "};",
        "",
        "fn eid(value: &str) -> EvidenceId { EvidenceId::new(value).unwrap() }",
        "",
        "fn main() {",
    ]
    verdict = {
        "CompletedConjunctivePass": "Pass",
        "CompletedConjunctiveFail": "SemanticFail",
        "InfrastructureInterrupted": "InfrastructureInterrupted",
        "Indeterminate": "Indeterminate",
        "AllRequiredJobsNoStart": "Incomplete",
        "RequiredDependencyBlocked": "Incomplete",
        "PartiallyExecutedAwaitingRequiredJob": "Incomplete",
        "RequiredJobExecuting": "Incomplete",
    }
    for i, vector in enumerate(data["vectors"]):
        m = vector["core_manifest_v1"]
        o = vector["core_observation_v1"]
        lines.append(f"    // {vector['id']}")
        lines.append(f"    let manifest_{i} = RequiredJobManifestV1 {{")
        lines.append(f"        theorem_id: eid({rust_string(m['theorem_id'])}),")
        lines.append(f"        repository_id: {m['repository_id']},")
        lines.append(f"        workflow_path: eid({rust_string(m['workflow_path'])}),")
        lines.append(f"        qualification_head: eid({rust_string(m['qualification_head'])}),")
        lines.append("        required_jobs: vec![")
        for spec in m["required_jobs"]:
            lines.append(f"            RequiredJobSpecV1 {{ job_key: eid({rust_string(spec['job_key'])}) }},")
        lines += ["        ],", "    };"]
        lines.append(f"    let observation_{i} = WorkflowRunObservationV1 {{")
        lines.append(f"        repository_id: {o['repository_id']},")
        lines.append(f"        workflow_run_id: {o['workflow_run_id']},")
        lines.append(f"        qualification_head: eid({rust_string(o['qualification_head'])}),")
        lines.append(f"        workflow_path: eid({rust_string(o['workflow_path'])}),")
        lines.append("        jobs: vec![")
        for job in o["jobs"]:
            conc = "None" if job["conclusion"] is None else f"Some(JobConclusionV1::{job['conclusion']})"
            age = "None" if job["queue_age_seconds"] is None else f"Some({job['queue_age_seconds']})"
            lines += [
                "            JobObservationV1 {",
                f"                job_id: {job['job_id']},",
                f"                job_key: eid({rust_string(job['job_key'])}),",
                f"                status: JobStatusV1::{job['status']},",
                f"                conclusion: {conc},",
                f"                gate_execution: GateExecutionStateV1::{job['gate_execution']},",
                f"                dependency_state: DependencyStateV1::{job['dependency_state']},",
                f"                queue_age_seconds: {age},",
                f"                failure_class: FailureClassV1::{job['failure_class']},",
                "            },",
            ]
        lines += ["        ],", "    };"]
        expected = vector["expected_core_liveness"]
        lines.append(f"    let got_{i} = classify_liveness_v1(&manifest_{i}, &observation_{i}).unwrap();")
        lines.append(f"    assert_eq!(got_{i}, RunLivenessV1::{expected});")
        lines.append(f"    let receipt_{i} = derive_conjunctive_receipt_v1(&manifest_{i}, &observation_{i}).unwrap();")
        lines.append(f"    assert_eq!(receipt_{i}.liveness, RunLivenessV1::{expected});")
        lines.append(f"    assert_eq!(receipt_{i}.verdict, ConjunctiveVerdictV1::{verdict[expected]});")
        observed_by_key = {job["job_key"]: job["job_id"] for job in o["jobs"]}
        ids = ", ".join(str(observed_by_key[spec["job_key"]]) for spec in m["required_jobs"])
        lines.append(f"    assert_eq!(receipt_{i}.required_job_ids, vec![{ids}]);")
        lines.append(f"    println!({rust_string(vector['id'] + ': ' + expected)});")
        lines.append("")
    lines += ["}", ""]
    return "\n".join(lines)

def load(path: Path) -> dict:
    return validate(json.loads(path.read_text(encoding="utf-8")))

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("vectors", type=Path)
    args = parser.parse_args(argv)
    try:
        data = load(args.vectors)
        sys.stdout.write(render(data))
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"vector harness generation refused: {exc}", file=sys.stderr)
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
