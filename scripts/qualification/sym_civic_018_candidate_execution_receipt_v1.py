#!/usr/bin/env python3
from __future__ import annotations
import argparse, hashlib, json, re, subprocess
from pathlib import Path

SCHEMA = "MYCELIX-SYM-CIVIC-018-QUALIFICATION-RECEIPT-V1"
REPO_ID = 1176351975
REPO = "Luminous-Dynamics/mycelix"
PR = 4322
HEAD = "11d5586c555f47b9e682a24597e04340a5234373"
PARENT = "1196889eb2a849ddc359d8d05edf063aa9537699"
WORKFLOW_ID = 375144995
WORKFLOW_PATH = ".github/workflows/sym-civic-018-cbor-encoding.yml"
RUN_ID = 37714865009
JOB_ID = 113108933070
JOB_NAME = "CBOR encoding boundary"
EXPECTED_COUNTS = {
    "CBOR_ENCODING_REJECT": 17,
    "CBOR_ENCODING_SUFFICIENT": 9,
    "CBOR_ENCODING_UNRESOLVED": 0,
    "CBOR_MESSAGE_REJECT": 13,
    "CBOR_PARSE_ERROR": 12,
}
PREFIXES = {
    "cases": "SYM-CIVIC-018-CBOR CASES=",
    "corpus": "SYM-CIVIC-018-CORPUS_SHA256=",
    "depth": "SYM-CIVIC-018-CBOR DEPTH_VECTORS=",
    "depth_corpus": "SYM-CIVIC-018-DEPTH_CORPUS_SHA256=",
    "resource": "SYM-CIVIC-018-CBOR RESOURCE_VECTORS=",
    "resource_corpus": "SYM-CIVIC-018-RESOURCE_CORPUS_SHA256=",
    "derived": "SYM-CIVIC-018-CBOR DERIVED=",
    "metamorphic": "SYM-CIVIC-018-CBOR METAMORPHIC=",
}
SHA40 = re.compile(r"^[0-9a-f]{40}$")

def reject_duplicates(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError("duplicate JSON object key")
        out[key] = value
    return out

def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=reject_duplicates)

def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")

def git(root, *args):
    return subprocess.check_output(["git", *args], cwd=root, text=True).strip()

def one(log_text, prefix):
    values = [line.split(prefix, 1)[1].strip() for line in log_text.splitlines() if prefix in line]
    if len(values) != 1:
        raise AssertionError({"prefix": prefix, "matches": values})
    return values[0]

def verify_common(args, receipt=None):
    manifest = load_json(args.manifest)
    run = load_json(args.run_json)
    jobs = load_json(args.jobs_json)
    pr = load_json(args.pr_json)
    trigger = load_json(args.trigger_workflow_json)

    assert manifest["schema"] == "MYCELIX-SYM-CIVIC-018-TRUSTED-ADMISSION-V1"
    capture = manifest["candidate_execution_receipt"]
    assert capture["schema"] == SCHEMA

    assert run["id"] == RUN_ID and run["head_sha"] == HEAD
    assert run["event"] == "pull_request"
    assert run["workflow_id"] == WORKFLOW_ID and run["path"] == WORKFLOW_PATH
    assert run["repository"]["id"] == REPO_ID and run["repository"]["full_name"] == REPO
    assert run["head_repository"]["id"] == REPO_ID and run["head_repository"]["full_name"] == REPO
    assert run["conclusion"] == "success"
    assert SHA40.fullmatch(run["workflow_sha"])

    matches = [
        job for job in jobs.get("jobs", [])
        if job.get("run_id") == RUN_ID
        and job.get("id") == JOB_ID
        and job.get("name") == JOB_NAME
        and job.get("status") == "completed"
        and job.get("conclusion") == "success"
    ]
    assert len(matches) == 1

    assert pr["number"] == PR and pr["state"] == "open"
    assert pr["head"]["sha"] == HEAD and pr["base"]["sha"] == PARENT
    assert pr["head"]["repo"]["id"] == REPO_ID and pr["base"]["repo"]["id"] == REPO_ID
    assert pr["head"]["repo"]["full_name"] == REPO and pr["base"]["repo"]["full_name"] == REPO

    assert trigger["path"] == WORKFLOW_PATH
    assert trigger["sha"] == manifest["expected_blob_sha"][WORKFLOW_PATH]

    assert git(args.candidate_root, "rev-parse", f"{HEAD}^") == PARENT
    assert git(args.candidate_root, "rev-list", "--count", f"{PARENT}..{HEAD}") == "1"
    assert git(args.candidate_root, "diff", "--name-only", PARENT, HEAD).splitlines() == sorted(manifest["expected_changed_files"])
    for path, expected in manifest["expected_blob_sha"].items():
        assert git(args.candidate_root, "rev-parse", f"{HEAD}:{path}") == expected

    return manifest, run, pr, trigger

def build(args):
    manifest, run, pr, trigger = verify_common(args)
    log_bytes = args.job_log.read_bytes()
    log_text = log_bytes.decode("utf-8-sig")
    capture = manifest["candidate_execution_receipt"]

    observed = {
        "cases": int(one(log_text, PREFIXES["cases"])),
        "corpus": one(log_text, PREFIXES["corpus"]),
        "depth_vectors": json.loads(one(log_text, PREFIXES["depth"]), object_pairs_hook=reject_duplicates),
        "depth_corpus": one(log_text, PREFIXES["depth_corpus"]),
        "resource_vectors": json.loads(one(log_text, PREFIXES["resource"]), object_pairs_hook=reject_duplicates),
        "resource_corpus": one(log_text, PREFIXES["resource_corpus"]),
        "counts": json.loads(one(log_text, PREFIXES["derived"]), object_pairs_hook=reject_duplicates),
        "metamorphic": one(log_text, PREFIXES["metamorphic"]),
    }

    assert observed["cases"] == capture["cases"] == 51
    assert observed["corpus"] == capture["corpus_sha256"]
    assert observed["depth_vectors"] == capture["depth_vectors"]
    assert observed["depth_corpus"] == capture["depth_corpus_sha256"]
    assert observed["resource_vectors"] == capture["resource_vectors"]
    assert observed["resource_corpus"] == capture["resource_corpus_sha256"]
    assert observed["counts"] == capture["counts"] == EXPECTED_COUNTS
    assert observed["metamorphic"] == capture["metamorphic"] == "PASS"

    core = {
        "schema": SCHEMA,
        "claim_ceiling": capture["claim_ceiling"],
        "subject": {
            "repository_id": REPO_ID,
            "repository": REPO,
            "pr_number": PR,
            "parent_sha": PARENT,
            "candidate_sha": HEAD,
            "candidate_tree_sha": git(args.candidate_root, "rev-parse", f"{HEAD}^{{tree}}"),
        },
        "execution": {
            "event": run["event"],
            "run_id": RUN_ID,
            "run_attempt": run["run_attempt"],
            "workflow_id": WORKFLOW_ID,
            "workflow_path": WORKFLOW_PATH,
            "workflow_sha": run["workflow_sha"],
            "workflow_file_blob_sha": trigger["sha"],
            "job_id": JOB_ID,
            "job_name": JOB_NAME,
            "conclusion": "success",
        },
        "live_pr": {
            "state": pr["state"],
            "head_sha": pr["head"]["sha"],
            "base_sha": pr["base"]["sha"],
            "head_repository": pr["head"]["repo"]["full_name"],
            "base_repository": pr["base"]["repo"]["full_name"],
        },
        "candidate_surface": {
            "changed_files": sorted(manifest["expected_changed_files"]),
            "blob_sha": manifest["expected_blob_sha"],
        },
        "semantic_capture": {
            **observed,
            "ordered_result_commitment": capture["ordered_result_commitment"],
            "disposition": "PASS",
        },
        "evidence_source": {
            "source": "github-actions-job-log",
            "run_id": RUN_ID,
            "job_id": JOB_ID,
            "job_name": JOB_NAME,
            "job_log_sha256": hashlib.sha256(log_bytes).hexdigest(),
        },
        "nonclaims": [
            "candidate verifier is independently trusted",
            "production conformance",
            "cryptographic validity",
            "SCITT trust",
            "transport trust",
            "operational authority",
            "legal authority",
        ],
    }
    receipt = dict(core)
    receipt["receipt_id_sha256"] = hashlib.sha256(canonical(core)).hexdigest()
    args.output.write_bytes(canonical(receipt) + b"\n")
    print("candidate qualification receipt: PASS")
    print("receipt_id_sha256=" + receipt["receipt_id_sha256"])

def verify(args):
    manifest, run, pr, _ = verify_common(args)
    receipt = load_json(args.receipt)
    receipt_id = receipt.pop("receipt_id_sha256")
    assert receipt["schema"] == SCHEMA
    assert hashlib.sha256(canonical(receipt)).hexdigest() == receipt_id
    assert receipt["subject"]["candidate_sha"] == HEAD
    assert receipt["subject"]["parent_sha"] == PARENT
    assert receipt["execution"]["run_id"] == RUN_ID
    assert receipt["execution"]["job_id"] == JOB_ID
    assert receipt["evidence_source"]["job_log_sha256"] == hashlib.sha256(args.job_log.read_bytes()).hexdigest()
    assert receipt["semantic_capture"]["cases"] == 51
    assert receipt["semantic_capture"]["corpus"] == manifest["candidate_execution_receipt"]["corpus_sha256"]
    assert receipt["semantic_capture"]["depth_vectors"] == manifest["candidate_execution_receipt"]["depth_vectors"]
    assert receipt["semantic_capture"]["depth_corpus"] == manifest["candidate_execution_receipt"]["depth_corpus_sha256"]
    assert receipt["semantic_capture"]["resource_vectors"] == manifest["candidate_execution_receipt"]["resource_vectors"]
    assert receipt["semantic_capture"]["resource_corpus"] == manifest["candidate_execution_receipt"]["resource_corpus_sha256"]
    assert receipt["semantic_capture"]["counts"] == EXPECTED_COUNTS
    assert receipt["semantic_capture"]["metamorphic"] == "PASS"
    assert receipt["semantic_capture"]["ordered_result_commitment"] == manifest["candidate_execution_receipt"]["ordered_result_commitment"]
    assert receipt["claim_ceiling"] == "SYNTHETIC_RESEARCH_ONLY"
    print("candidate qualification receipt integrity: PASS")
    print("receipt_id_sha256=" + receipt_id)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--candidate-root", required=True, type=Path)
    p.add_argument("--manifest", required=True, type=Path)
    p.add_argument("--run-json", required=True, type=Path)
    p.add_argument("--jobs-json", required=True, type=Path)
    p.add_argument("--pr-json", required=True, type=Path)
    p.add_argument("--trigger-workflow-json", required=True, type=Path)
    p.add_argument("--job-log", required=True, type=Path)
    p.add_argument("--output", type=Path)
    p.add_argument("--receipt", type=Path)
    p.add_argument("--verify-only", action="store_true")
    args = p.parse_args()
    if args.verify_only:
        if args.receipt is None:
            p.error("--receipt is required with --verify-only")
        verify(args)
    else:
        if args.output is None:
            p.error("--output is required unless --verify-only")
        build(args)

if __name__ == "__main__":
    main()
