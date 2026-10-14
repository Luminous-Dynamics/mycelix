#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

CANDIDATE_WORKFLOW_REL = ".github/workflows/sym-civic-018-cbor-encoding.yml"
CANDIDATE_MANIFEST_REL = "mycelix-workspace/docs/civic-resilience/sym_civic_018_cbor_encoding_v1.json"
CANDIDATE_DOC_REL = "mycelix-workspace/docs/civic-resilience/SYM_CIVIC_018_CBOR_ENCODING_V1.md"
CANDIDATE_QUALIFIER_REL = "scripts/qualification/sym_civic_018_cbor_encoding_v1.py"
TRUSTED_WORKFLOW_REL = ".github/workflows/sym-civic-018-independent-semantic.yml"
TRUSTED_MANIFEST_REL = "scripts/qualification/sym_civic_018_independent_semantic_v1.json"
TRUSTED_VERIFIER_REL = "scripts/qualification/sym_civic_018_independent_semantic_v1.py"
RECEIPT_CONTRACT_REL = "scripts/qualification/sym_civic_018_independent_semantic_receipt_contract_v1.json"
RECEIPT_BUILDER_REL = "scripts/qualification/sym_civic_018_independent_semantic_receipt_v1.py"
EXPECTED_REPO_ID = 1176351975
EXPECTED_REPO = "Luminous-Dynamics/mycelix"
EXPECTED_CANDIDATE_PR = 4322
EXPECTED_PARENT = "1196889eb2a849ddc359d8d05edf063aa9537699"
EXPECTED_CANDIDATE = "11d5586c555f47b9e682a24597e04340a5234373"
EXPECTED_TRIGGER_WORKFLOW_ID = 375144995
EXPECTED_TRIGGER_WORKFLOW = CANDIDATE_WORKFLOW_REL
EXPECTED_JOB = "CBOR encoding boundary"
EXPECTED_CANDIDATE_BLOBS = {
    CANDIDATE_WORKFLOW_REL: "27359c1bc7d130780ca82bababa927a1952f170b",
    CANDIDATE_DOC_REL: "1d92fc0304402ff61316313a6df5b5230b0803ef",
    CANDIDATE_MANIFEST_REL: "b9be91fb92b929809d580c6f4b86f5516d5144eb",
    CANDIDATE_QUALIFIER_REL: "af388f237c4b99b028464caf173d586d4ef1b352",
}
SHA40 = re.compile(r"^[0-9a-f]{40}$")


def reject_duplicate_json_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON object key")
        result[key] = value
    return result


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicate_json_keys)


def write_canonical(path: Path, value: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"
    )


def canonical_bytes(value: dict):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sha256_bytes(data: bytes):
    return hashlib.sha256(data).hexdigest()


def git(root: Path, *args: str):
    return subprocess.check_output(["git", *args], cwd=root, text=True).strip()


def require(condition: bool, message: str):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--candidate-root", required=True, type=Path)
    parser.add_argument("--trusted-root", required=True, type=Path)
    parser.add_argument("--run-json", required=True, type=Path)
    parser.add_argument("--jobs-json", required=True, type=Path)
    parser.add_argument("--live-pr-json", required=True, type=Path)
    parser.add_argument("--trigger-workflow-json", required=True, type=Path)
    parser.add_argument("--semantic-result", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--verification-output", required=True, type=Path)
    args = parser.parse_args()

    contract = load_json(args.contract)
    require(contract["schema"] == "MYCELIX-SYM-CIVIC-018-INDEPENDENT-SEMANTIC-RECEIPT-CONTRACT-V1", "receipt contract schema mismatch")
    require(contract["receipt_schema"] == "MYCELIX-SYM-CIVIC-018-INDEPENDENT-SEMANTIC-RECEIPT-V1", "receipt schema mismatch")
    require(contract["canonicalization"] == "JSON-SORT-KEYS-SEPARATORS-UTF8-V1", "canonicalization profile mismatch")
    require(set(contract["required_top_level"]) == {
        "receipt_id_sha256","schema","subject","trigger_execution","trusted_control_plane",
        "candidate_surface","semantic_result","reported_execution","disposition","claim_ceiling"
    }, "receipt contract key set mismatch")

    run = load_json(args.run_json)
    jobs = load_json(args.jobs_json)
    live_pr = load_json(args.live_pr_json)
    trigger_workflow = load_json(args.trigger_workflow_json)
    semantic = load_json(args.semantic_result)

    require(run["id"] > 0, "invalid trigger run ID")
    require(run["head_sha"] == EXPECTED_CANDIDATE, "trigger candidate mismatch")
    require(run["event"] == "pull_request", "trigger event mismatch")
    require(run["workflow_id"] == EXPECTED_TRIGGER_WORKFLOW_ID, "trigger workflow ID mismatch")
    require(run["path"] == EXPECTED_TRIGGER_WORKFLOW, "trigger workflow path mismatch")
    require(run["repository"]["id"] == EXPECTED_REPO_ID and run["repository"]["full_name"] == EXPECTED_REPO, "trigger repository mismatch")
    require(run["head_repository"]["id"] == EXPECTED_REPO_ID and run["head_repository"]["full_name"] == EXPECTED_REPO, "trigger head repository mismatch")
    require(run["conclusion"] == "success", "trigger conclusion mismatch")
    require(isinstance(run.get("run_attempt"), int) and run["run_attempt"] >= 1, "invalid trigger run attempt")
    require(SHA40.fullmatch(run.get("workflow_sha","")), "trigger workflow SHA missing/invalid")

    matching_jobs = [
        job for job in jobs.get("jobs", [])
        if job.get("run_id") == run["id"]
        and job.get("name") == EXPECTED_JOB
        and job.get("status") == "completed"
        and job.get("conclusion") == "success"
    ]
    require(len(matching_jobs) == 1, "expected exactly one successful qualification job")
    job = matching_jobs[0]
    require(job.get("run_attempt") in (None, run["run_attempt"]), "job/run attempt mismatch")

    require(live_pr["number"] == EXPECTED_CANDIDATE_PR, "live PR number mismatch")
    require(live_pr["state"] == "open", "live PR not open at receipt creation")
    require(live_pr["head"]["sha"] == EXPECTED_CANDIDATE, "live PR head mismatch")
    require(live_pr["base"]["sha"] == EXPECTED_PARENT, "live PR base mismatch")
    require(live_pr["head"]["repo"]["id"] == EXPECTED_REPO_ID and live_pr["head"]["repo"]["full_name"] == EXPECTED_REPO, "live PR head repository mismatch")
    require(live_pr["base"]["repo"]["id"] == EXPECTED_REPO_ID and live_pr["base"]["repo"]["full_name"] == EXPECTED_REPO, "live PR base repository mismatch")

    require(trigger_workflow["path"] == EXPECTED_TRIGGER_WORKFLOW, "trigger workflow-file path mismatch")
    require(trigger_workflow["sha"] == EXPECTED_CANDIDATE_BLOBS[CANDIDATE_WORKFLOW_REL], "trigger workflow-file blob mismatch")

    require(git(args.candidate_root, "rev-parse", "HEAD") == EXPECTED_CANDIDATE, "candidate checkout mismatch")
    require(git(args.candidate_root, "rev-parse", f"{EXPECTED_CANDIDATE}^") == EXPECTED_PARENT, "candidate parent mismatch")
    require(git(args.candidate_root, "rev-list", "--count", f"{EXPECTED_PARENT}..{EXPECTED_CANDIDATE}") == "1", "candidate topology mismatch")
    candidate_tree = git(args.candidate_root, "rev-parse", f"{EXPECTED_CANDIDATE}^{{tree}}")

    actual_files = git(args.candidate_root, "diff", "--name-only", EXPECTED_PARENT, EXPECTED_CANDIDATE).splitlines()
    require(actual_files == sorted(EXPECTED_CANDIDATE_BLOBS), "candidate changed-file surface mismatch")
    actual_blobs = {
        path: git(args.candidate_root, "rev-parse", f"{EXPECTED_CANDIDATE}:{path}")
        for path in EXPECTED_CANDIDATE_BLOBS
    }
    require(actual_blobs == EXPECTED_CANDIDATE_BLOBS, "candidate research surface blob mismatch")

    trusted_head = git(args.trusted_root, "rev-parse", "HEAD")
    require(SHA40.fullmatch(trusted_head), "trusted workflow revision missing/invalid")
    trusted_blobs = {
        TRUSTED_WORKFLOW_REL: git(args.trusted_root, "rev-parse", f"{trusted_head}:{TRUSTED_WORKFLOW_REL}"),
        TRUSTED_MANIFEST_REL: git(args.trusted_root, "rev-parse", f"{trusted_head}:{TRUSTED_MANIFEST_REL}"),
        TRUSTED_VERIFIER_REL: git(args.trusted_root, "rev-parse", f"{trusted_head}:{TRUSTED_VERIFIER_REL}"),
        RECEIPT_CONTRACT_REL: git(args.trusted_root, "rev-parse", f"{trusted_head}:{RECEIPT_CONTRACT_REL}"),
        RECEIPT_BUILDER_REL: git(args.trusted_root, "rev-parse", f"{trusted_head}:{RECEIPT_BUILDER_REL}"),
    }
    require(all(SHA40.fullmatch(value) for value in trusted_blobs.values()), "invalid trusted blob identity")

    require(semantic["schema"] == "MYCELIX-SYM-CIVIC-018-INDEPENDENT-SEMANTIC-RESULT-V1", "semantic result schema mismatch")
    require(semantic["candidate_pr"] == EXPECTED_CANDIDATE_PR, "semantic result PR mismatch")
    require(semantic["candidate_sha"] == EXPECTED_CANDIDATE, "semantic result candidate mismatch")
    require(semantic["parent_sha"] == EXPECTED_PARENT, "semantic result parent mismatch")
    require(semantic["run_id"] == run["id"], "semantic result run mismatch")
    require(semantic["job_id"] == job["id"], "semantic result job mismatch")
    require(semantic["job_name"] == EXPECTED_JOB, "semantic result job mismatch")
    require(semantic["candidate_schema"] is not None, "semantic result candidate schema missing")
    require(semantic["candidate_program"] is not None, "semantic result candidate program missing")
    require(semantic["disposition"] == "TRUSTED_INDEPENDENT_SEMANTIC_RECOMPUTATION_PASS", "semantic result is not an independent semantic pass")
    require(semantic["boundary"] == "trusted_candidate_independent_semantic_recomputation", "semantic boundary mismatch")
    require(semantic["claim_ceiling"] == "SYNTHETIC_RESEARCH_ONLY", "semantic claim ceiling mismatch")
    require(semantic["metamorphic"] == "NOT_RECOMPUTED", "unexpected metamorphic claim")

    semantic_file_sha = sha256_bytes(args.semantic_result.read_bytes())

    receipt_core = {
        "schema": "MYCELIX-SYM-CIVIC-018-INDEPENDENT-SEMANTIC-RECEIPT-V1",
        "subject": {
            "repository_id": EXPECTED_REPO_ID,
            "repository": EXPECTED_REPO,
            "pr_number": EXPECTED_CANDIDATE_PR,
            "parent_sha": EXPECTED_PARENT,
            "candidate_sha": EXPECTED_CANDIDATE,
            "candidate_tree_sha": candidate_tree,
        },
        "trigger_execution": {
            "event": run["event"],
            "run_id": run["id"],
            "run_attempt": run["run_attempt"],
            "workflow_id": run["workflow_id"],
            "workflow_path": run["path"],
            "workflow_sha": run["workflow_sha"],
            "workflow_file_blob_sha": trigger_workflow["sha"],
            "job_id": job["id"],
            "job_name": job["name"],
            "job_conclusion": job["conclusion"],
        },
        "trusted_control_plane": {
            "workflow_sha": trusted_head,
            "workflow_ref": TRUSTED_WORKFLOW_REL,
            "workflow_file_blob_sha": trusted_blobs[TRUSTED_WORKFLOW_REL],
            "manifest_blob_sha": trusted_blobs[TRUSTED_MANIFEST_REL],
            "verifier_blob_sha": trusted_blobs[TRUSTED_VERIFIER_REL],
            "receipt_contract_blob_sha": trusted_blobs[RECEIPT_CONTRACT_REL],
            "receipt_builder_blob_sha": trusted_blobs[RECEIPT_BUILDER_REL],
        },
        "candidate_surface": {
            "changed_files": sorted(EXPECTED_CANDIDATE_BLOBS),
            "blob_sha": actual_blobs,
        },
        "semantic_result": {
            "schema": semantic["schema"],
            "candidate_schema": semantic["candidate_schema"],
            "candidate_program": semantic["candidate_program"],
            "message_corpus_sha256": semantic["message_corpus_sha256"],
            "depth_corpus_sha256": semantic["depth_corpus_sha256"],
            "resource_corpus_sha256": semantic["resource_corpus_sha256"],
            "message_results_sha256": semantic["message_results_sha256"],
            "counts": semantic["counts"],
            "depth_results": semantic["depth_results"],
            "resource_results": semantic["resource_results"],
            "anchors": semantic["anchors"],
            "metamorphic": "NOT_RECOMPUTED",
            "semantic_result_sha256": semantic_file_sha,
        },
        "reported_execution": {
            "trigger_run_conclusion": run["conclusion"],
            "trigger_job_conclusion": job["conclusion"],
            "independent_verifier_recomputed": True,
        },
        "disposition": {
            "result": semantic["disposition"],
            "boundary": semantic["boundary"],
        },
        "claim_ceiling": "SYNTHETIC_RESEARCH_ONLY",
    }

    receipt = dict(receipt_core)
    receipt["receipt_id_sha256"] = sha256_bytes(canonical_bytes(receipt_core))
    write_canonical(args.output, receipt)

    loaded = load_json(args.output)
    require(loaded == receipt, "receipt round-trip mismatch")
    require(
        loaded["receipt_id_sha256"]
        == sha256_bytes(canonical_bytes({k: v for k, v in loaded.items() if k != "receipt_id_sha256"})),
        "receipt identity hash mismatch",
    )

    verification = {
        "schema":"MYCELIX-SYM-CIVIC-018-INDEPENDENT-SEMANTIC-RECEIPT-VERIFICATION-V1",
        "receipt_id_sha256":receipt["receipt_id_sha256"],
        "receipt_file_sha256":sha256_bytes(args.output.read_bytes()),
        "semantic_result_sha256":semantic_file_sha,
        "verified":True,
        "claim_ceiling":"SYNTHETIC_RESEARCH_ONLY",
    }
    write_canonical(args.verification_output, verification)

    print("SYM-CIVIC-018-INDEPENDENT-SEMANTIC-RECEIPT=PASS")
    print("receipt_id_sha256="+receipt["receipt_id_sha256"])
    print("receipt_file_sha256="+verification["receipt_file_sha256"])
    print("semantic_result_sha256="+semantic_file_sha)


if __name__ == "__main__":
    main()
