#!/usr/bin/env python3
"""Independent raw-object causal-join verifier for FPM trusted qualification."""

from __future__ import annotations

import base64
import datetime as dt
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

BASE_REPOSITORY = "Luminous-Dynamics/mycelix"
BASE_REPOSITORY_ID = 1176351975
BASE_BRANCH = "main"

CANDIDATE_WORKFLOW_ID = 377461322
CANDIDATE_WORKFLOW_PATH = ".github/workflows/fpm-wasm-artifact-identity.yml"
TRUSTED_WORKFLOW_NAME = "FPM trusted qualification policy"
TRUSTED_WORKFLOW_PATH = ".github/workflows/fpm-trusted-qualification.yml"
INDEPENDENT_WORKFLOW_PATH = ".github/workflows/fpm-trusted-qualification-independent-verify.yml"

MANIFEST_PATH = "crates/fpm-wasm-artifact-identity/Cargo.toml"
MANIFEST_BLOB_SHA = "c94b53f61ed8a9bfb6249b1b339550dddd074d6c"

RUSTC_VERSION = "rustc 1.96.1"
RUSTC_COMMIT = "31fca3adb283cc9dfd56b49cdee9a96eb9c96ffd"

RECEIPT_KEYS = frozenset(
    {
        "schema",
        "qualification",
        "repository",
        "repository_id",
        "pr_number",
        "subject_sha",
        "subject_tree_sha",
        "observed_postflight_head_sha",
        "observed_postflight_tree_sha",
        "base_sha",
        "trusted_policy_sha",
        "trusted_policy_blob_sha",
        "trusted_policy_ref",
        "trusted_workflow_run_id",
        "trusted_workflow_run_attempt",
        "upstream_workflow_run_id",
        "upstream_workflow_run_attempt",
        "upstream_workflow_id",
        "upstream_workflow_path",
        "upstream_workflow_conclusion",
        "manifest_blob_sha",
        "lock_mode",
        "lock_sha256",
        "rustc_version",
        "rustc_commit",
        "cargo_version",
        "candidate_uid",
        "candidate_execution_profile",
        "steps",
        "execution_pass",
        "procedure_trust",
        "promotion_authority",
    }
)

STEP_KEYS = frozenset(
    {"preflight", "checkout", "source", "toolchain", "lock", "fmt", "tests", "postflight"}
)

INDEX_KEYS = frozenset(
    {
        "schema",
        "receipt_sha256",
        "artifact",
        "subject_sha",
        "subject_tree_sha",
        "trusted_policy_sha",
        "trusted_policy_blob_sha",
        "trusted_workflow_run_id",
    }
)

ENUMERATION_KEYS = frozenset(
    {
        "schema",
        "page_size",
        "max_pages",
        "max_artifacts",
        "total_count_reported",
        "enumerated_count",
        "page_counts",
        "terminal_page",
        "artifact_identity_sha256",
        "complete",
    }
)

INDEX_ARTIFACT_KEYS = frozenset(
    {
        "id",
        "sha256_hex",
        "url",
        "retention_days",
        "immutable_after_upload",
        "deletion_by_repository_writer_possible",
    }
)


def fail(message: str) -> None:
    raise SystemExit(f"FPM_REFERENCE_FAIL: {message}")


def need(value: Any, description: str) -> Any:
    if value is None:
        fail(f"missing {description}")
    return value


def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            fail(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def load_canonical_json(path: Path) -> dict[str, Any]:
    raw = path.read_bytes()
    if not raw.endswith(b"\n"):
        fail(f"{path} must end in exactly one LF")
    canonical_bytes = raw[:-1]
    if canonical_bytes.endswith(b"\n"):
        fail(f"{path} has more than one trailing LF")
    try:
        text = canonical_bytes.decode("utf-8")
        value = json.loads(text, object_pairs_hook=reject_duplicate_keys)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail(f"{path} is not valid UTF-8 JSON: {exc}")
    if not isinstance(value, dict):
        fail(f"{path} top level must be an object")
    reserialized = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    if reserialized != canonical_bytes:
        fail(f"{path} is not canonical JSON")
    return value


def require_hex(value: Any, length: int, field: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(rf"[0-9a-f]{{{length}}}", value):
        fail(f"{field} is not canonical lowercase hex{length}")
    return value


def require_sha256(value: Any, field: str) -> str:
    return require_hex(value, 64, field)


def require_sha256_prefixed(value: Any, field: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", value):
        fail(f"{field} is not sha256:<64 lowercase hex>")
    return value


def validate_artifact_lifetime(item: dict[str, Any], label: str) -> tuple[str, str]:
    created = item.get("created_at")
    expires = item.get("expires_at")
    if not isinstance(created, str) or not isinstance(expires, str):
        fail(f"{label} artifact timestamps are missing")
    try:
        created_dt = dt.datetime.fromisoformat(created.replace("Z", "+00:00"))
        expires_dt = dt.datetime.fromisoformat(expires.replace("Z", "+00:00"))
    except ValueError as exc:
        fail(f"{label} artifact timestamp is invalid: {exc}")
    if created_dt.tzinfo is None or expires_dt.tzinfo is None:
        fail(f"{label} artifact timestamps must be timezone-aware")
    if expires_dt <= created_dt:
        fail(f"{label} artifact expires_at is not after created_at")
    return created, expires


def verify_receipt(
    receipt: dict[str, Any],
    expected_trusted_run_id: int,
    expected_trusted_run_attempt: int,
    candidate_run: dict[str, Any],
    trusted_run: dict[str, Any],
    pr: dict[str, Any],
    commit: dict[str, Any],
    manifest: dict[str, Any],
    policy_file: dict[str, Any],
    verifier_control: dict[str, Any],
    candidate_lock: dict[str, Any],
    main_ref: dict[str, Any],
) -> tuple[str, str, str]:
    if set(receipt) != RECEIPT_KEYS:
        fail(
            "receipt closed-world mismatch: "
            f"missing={sorted(RECEIPT_KEYS - set(receipt))!r} "
            f"extra={sorted(set(receipt) - RECEIPT_KEYS)!r}"
        )

    if receipt["schema"] != "mycelix.fpm.trusted-qualification-receipt.v1":
        fail("unexpected receipt schema")
    if receipt["qualification"] != "FPM-WASM-ARTIFACT-IDENTITY-V1":
        fail("unexpected qualification name")
    if receipt["repository"] != BASE_REPOSITORY:
        fail("receipt repository mismatch")
    if receipt["repository_id"] != BASE_REPOSITORY_ID:
        fail("receipt repository_id mismatch")

    pr_number = receipt["pr_number"]
    if not isinstance(pr_number, str) or not re.fullmatch(r"[1-9][0-9]*", pr_number):
        fail("invalid receipt pr_number")
    if int(pr_number) != pr["number"]:
        fail("receipt PR number mismatch")

    subject_sha = require_hex(receipt["subject_sha"], 40, "subject_sha")
    subject_tree = require_hex(receipt["subject_tree_sha"], 40, "subject_tree_sha")
    if receipt["observed_postflight_head_sha"] != subject_sha:
        fail("postflight head differs from subject")
    if receipt["observed_postflight_tree_sha"] != subject_tree:
        fail("postflight tree differs from subject")

    require_hex(receipt["base_sha"], 40, "base_sha")
    if receipt["base_sha"] != pr["base"]["sha"]:
        fail("receipt base SHA mismatch")
    if main_ref.get("ref") != "refs/heads/main":
        fail("live main ref does not name refs/heads/main")
    main_sha = require_hex(main_ref.get("object", {}).get("sha"), 40, "live main ref SHA")

    policy_sha = require_hex(receipt["trusted_policy_sha"], 40, "trusted_policy_sha")
    if policy_sha != trusted_run.get("head_sha"):
        fail("receipt trusted policy SHA does not equal trusted workflow run head SHA")
    if trusted_run.get("head_branch") != "main":
        fail("trusted workflow run is not on the default branch")
    if trusted_run.get("repository", {}).get("id") != BASE_REPOSITORY_ID:
        fail("trusted workflow run repository ID mismatch")
    if receipt["trusted_policy_ref"] != "refs/heads/main":
        fail("receipt trusted policy ref mismatch")
    if receipt["trusted_policy_blob_sha"] != policy_file.get("sha"):
        fail("receipt trusted policy blob mismatch")
    if policy_file.get("path") != TRUSTED_WORKFLOW_PATH:
        fail("policy file path mismatch")

    trusted_run_id = int(receipt["trusted_workflow_run_id"])
    if trusted_run_id != expected_trusted_run_id:
        fail("trusted workflow run ID mismatch")
    if int(receipt["trusted_workflow_run_attempt"]) != expected_trusted_run_attempt:
        fail("trusted workflow run attempt mismatch")

    if int(receipt["upstream_workflow_run_id"]) != int(candidate_run["id"]):
        fail("candidate trigger run ID mismatch")
    if int(receipt["upstream_workflow_run_attempt"]) != int(candidate_run["run_attempt"]):
        fail("candidate trigger run attempt mismatch")
    if int(receipt["upstream_workflow_id"]) != CANDIDATE_WORKFLOW_ID:
        fail("candidate workflow ID mismatch")
    if receipt["upstream_workflow_path"] != CANDIDATE_WORKFLOW_PATH:
        fail("candidate workflow path mismatch")
    if receipt["upstream_workflow_conclusion"] != candidate_run.get("conclusion", ""):
        fail("candidate workflow conclusion mismatch")

    if candidate_run["id"] != int(receipt["upstream_workflow_run_id"]):
        fail("candidate trigger run object mismatch")
    if candidate_run["workflow_id"] != CANDIDATE_WORKFLOW_ID:
        fail("candidate trigger workflow ID mismatch")
    if candidate_run["path"] != CANDIDATE_WORKFLOW_PATH:
        fail("candidate trigger path mismatch")
    if candidate_run["event"] != "pull_request":
        fail("candidate trigger event mismatch")
    if candidate_run["head_repository"]["id"] != BASE_REPOSITORY_ID:
        fail("candidate trigger repository mismatch")
    if candidate_run["head_sha"] != subject_sha:
        fail("candidate trigger head differs from receipt subject")
    if candidate_run["run_attempt"] < 1:
        fail("candidate trigger attempt invalid")

    if trusted_run["id"] != expected_trusted_run_id:
        fail("trusted workflow run object mismatch")
    if trusted_run["name"] != TRUSTED_WORKFLOW_NAME:
        fail("trusted workflow name mismatch")
    if trusted_run["path"] != TRUSTED_WORKFLOW_PATH:
        fail("trusted workflow path mismatch")
    if trusted_run["event"] != "workflow_run":
        fail("trusted workflow event mismatch")
    if trusted_run["status"] != "completed" or trusted_run["conclusion"] != "success":
        fail("trusted workflow did not complete successfully")
    if trusted_run["repository"]["full_name"] != BASE_REPOSITORY:
        fail("trusted workflow repository mismatch")

    if pr["state"] not in {"open", "closed"}:
        fail("PR state is outside GitHub historical PR states")
    if pr["base"]["ref"] != BASE_BRANCH:
        fail("PR base ref mismatch")
    if pr["base"]["repo"]["id"] != BASE_REPOSITORY_ID:
        fail("PR base repository mismatch")
    if pr["head"]["repo"]["id"] != BASE_REPOSITORY_ID:
        fail("PR head repository mismatch")
    if pr["head"]["repo"]["full_name"] != BASE_REPOSITORY:
        fail("PR head repository name mismatch")
    if pr["head"]["sha"] != subject_sha:
        fail("live PR head SHA mismatch")

    if commit["sha"] != subject_sha:
        fail("subject commit object mismatch")
    if commit["commit"]["tree"]["sha"] != subject_tree:
        fail("subject tree SHA mismatch")

    if manifest["path"] != MANIFEST_PATH:
        fail("manifest path mismatch")
    if manifest["sha"] != MANIFEST_BLOB_SHA:
        fail("manifest blob mismatch")


    if receipt["manifest_blob_sha"] != MANIFEST_BLOB_SHA:
        fail("receipt manifest blob mismatch")
    lock_sha = require_sha256(receipt["lock_sha256"], "lock_sha256")
    lock_mode = receipt["lock_mode"]
    if lock_mode not in {"tracked", "generated_for_run"}:
        fail("unknown lock mode")
    if lock_mode == "tracked":
        if candidate_lock.get("encoding") != "base64":
            fail("tracked candidate lockfile was not returned as base64")
        lock_content_b64 = candidate_lock.get("content")
        if not isinstance(lock_content_b64, str):
            fail("tracked candidate lockfile content missing")
        try:
            lock_bytes = base64.b64decode("".join(lock_content_b64.split()), validate=True)
        except (ValueError, base64.binascii.Error) as exc:
            fail(f"tracked candidate lockfile base64 invalid: {exc}")
        if hashlib.sha256(lock_bytes).hexdigest() != lock_sha:
            fail("tracked candidate lockfile digest mismatch")
    else:
        if candidate_lock.get("mode") != "generated_for_run":
            fail("unexpected generated lockfile marker")

    if receipt["rustc_version"] != RUSTC_VERSION:
        fail("rustc version mismatch")
    if receipt["rustc_commit"] != RUSTC_COMMIT:
        fail("rustc commit mismatch")
    if not isinstance(receipt["cargo_version"], str) or not receipt["cargo_version"].startswith("cargo 1.96.1"):
        fail("cargo version mismatch")
    if receipt["candidate_execution_profile"] != "fpm-untrusted.env-i.v2":
        fail("unexpected candidate execution profile")
    if not isinstance(receipt["candidate_uid"], int) or receipt["candidate_uid"] <= 0:
        fail("invalid candidate UID")

    steps = receipt["steps"]
    if not isinstance(steps, dict) or set(steps) != STEP_KEYS:
        fail("receipt step outcome schema mismatch")
    if any(value != "success" for value in steps.values()):
        fail("receipt has non-success step outcome")
    if receipt["execution_pass"] is not True:
        fail("receipt execution_pass is not true")
    if receipt["procedure_trust"] != "trusted_default_branch_snapshot":
        fail("unexpected procedure trust value")
    if receipt["promotion_authority"] != "pending_repository_governance_evidence":
        fail("unexpected promotion authority")

    if verifier_control["repository"] != BASE_REPOSITORY:
        fail("independent verifier repository mismatch")
    if verifier_control["path"] != INDEPENDENT_WORKFLOW_PATH:
        fail("independent verifier path mismatch")
    if verifier_control["ref"] != "refs/heads/main":
        fail("independent verifier ref mismatch")
    verifier_sha = require_hex(verifier_control["workflow_sha"], 40, "independent verifier workflow_sha")
    verifier_blob_sha = require_hex(
        verifier_control["workflow_blob_sha"], 40, "independent verifier workflow_blob_sha"
    )
    if verifier_control["reference_verifier_path"] != "scripts/integral/verify_fpm_trusted_qualification.py":
        fail("reference verifier path mismatch")
    reference_verifier_blob_sha = require_hex(
        verifier_control["reference_verifier_blob_sha"], 40, "reference verifier blob SHA"
    )
    if verifier_control["artifact_collector_path"] != "scripts/integral/collect_fpm_trusted_artifacts.py":
        fail("artifact collector path mismatch")
    artifact_collector_blob_sha = require_hex(
        verifier_control["artifact_collector_blob_sha"], 40, "artifact collector blob SHA"
    )
    expected_workflow_ref = f"{BASE_REPOSITORY}/{INDEPENDENT_WORKFLOW_PATH}@refs/heads/main"
    if verifier_control["workflow_ref"] != expected_workflow_ref:
        fail("independent verifier workflow_ref mismatch")

    canonical = json.dumps(
        receipt, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest(), verifier_sha, verifier_blob_sha


def verify_artifact_enumeration(
    enumeration: dict[str, Any], items: list[dict[str, Any]]
) -> None:
    if set(enumeration) != ENUMERATION_KEYS:
        fail(
            "artifact enumeration closed-world mismatch: "
            f"missing={sorted(ENUMERATION_KEYS - set(enumeration))!r} "
            f"extra={sorted(set(enumeration) - ENUMERATION_KEYS)!r}"
        )
    if enumeration["schema"] != "mycelix.fpm.trusted-qualification-artifact-enumeration.v1":
        fail("unexpected artifact enumeration schema")
    if enumeration["page_size"] != 100:
        fail("unexpected artifact enumeration page size")
    if enumeration["max_pages"] != 4:
        fail("unexpected artifact enumeration page bound")
    if enumeration["max_artifacts"] != 256:
        fail("unexpected artifact enumeration global bound")
    if enumeration["complete"] is not True:
        fail("artifact enumeration is not marked complete")
    counts = enumeration["page_counts"]
    if not isinstance(counts, list) or not counts:
        fail("artifact enumeration page_counts is invalid")
    if any(type(x) is not int or x < 0 or x > 100 for x in counts):
        fail("artifact enumeration page count is invalid")
    if enumeration["terminal_page"] != len(counts):
        fail("terminal page does not match page_counts length")
    if enumeration["enumerated_count"] != len(items):
        fail("enumerated count does not match artifact list")
    if enumeration["total_count_reported"] != len(items):
        fail("reported total count does not match artifact list")
    if sum(counts) != len(items):
        fail("page counts do not sum to artifact count")
    if counts[-1] >= 100:
        fail("artifact enumeration did not observe a short/empty terminal page")
    identities = [
        {"id": item["id"], "name": item["name"]}
        for item in items
    ]
    commitment = hashlib.sha256(
        json.dumps(identities, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()
    if enumeration["artifact_identity_sha256"] != commitment:
        fail("artifact identity enumeration commitment mismatch")

def verify_index(
    index: dict[str, Any],
    receipt_digest: str,
    receipt: dict[str, Any],
    primary_artifact: dict[str, Any],
    trusted_run_id: int,
) -> None:
    if set(index) != INDEX_KEYS:
        fail(
            "index closed-world mismatch: "
            f"missing={sorted(INDEX_KEYS - set(index))!r} "
            f"extra={sorted(set(index) - INDEX_KEYS)!r}"
        )
    if index["schema"] != "mycelix.fpm.trusted-qualification-artifact-index.v1":
        fail("unexpected index schema")

    artifact = index["artifact"]
    if not isinstance(artifact, dict) or set(artifact) != INDEX_ARTIFACT_KEYS:
        fail("index artifact schema mismatch")

    artifact_id = artifact["id"]
    if not isinstance(artifact_id, int) or artifact_id <= 0:
        fail("invalid index artifact ID")
    if artifact_id != primary_artifact["id"]:
        fail("index artifact ID mismatch")
    artifact_digest = require_sha256(artifact["sha256_hex"], "index artifact sha256_hex")
    if f"sha256:{artifact_digest}" != primary_artifact["digest"]:
        fail("index artifact digest mismatch")
    expected_artifact_url = (
        f"https://github.com/{BASE_REPOSITORY}/actions/runs/"
        f"{trusted_run_id}/artifacts/{artifact_id}"
    )
    if artifact["url"] != expected_artifact_url:
        fail("index artifact URL mismatch")
    if artifact["retention_days"] != 90:
        fail("unexpected retention policy")
    if artifact["immutable_after_upload"] is not True:
        fail("index does not record artifact immutability")
    if artifact["deletion_by_repository_writer_possible"] is not True:
        fail("index must preserve deletion caveat")

    if index["receipt_sha256"] != receipt_digest:
        fail("index receipt digest does not match canonical receipt")
    if index["subject_sha"] != receipt["subject_sha"]:
        fail("index subject SHA mismatch")
    if index["subject_tree_sha"] != receipt["subject_tree_sha"]:
        fail("index subject tree mismatch")
    if index["trusted_policy_sha"] != receipt["trusted_policy_sha"]:
        fail("index policy SHA mismatch")
    if index["trusted_policy_blob_sha"] != receipt["trusted_policy_blob_sha"]:
        fail("index policy blob mismatch")
    if int(index["trusted_workflow_run_id"]) != trusted_run_id:
        fail("index trusted run ID mismatch")


def verify(snapshot_dir: Path) -> dict[str, Any]:
    receipt = load_canonical_json(snapshot_dir / "qualification-receipt.json")
    index = load_canonical_json(snapshot_dir / "artifact-binding-index.json")
    trusted_run = json.loads((snapshot_dir / "trusted-run.json").read_text(encoding="utf-8"))
    candidate_run = json.loads((snapshot_dir / "candidate-run.json").read_text(encoding="utf-8"))
    pr = json.loads((snapshot_dir / "pull-request.json").read_text(encoding="utf-8"))
    commit = json.loads((snapshot_dir / "subject-commit.json").read_text(encoding="utf-8"))
    manifest = json.loads((snapshot_dir / "manifest.json").read_text(encoding="utf-8"))
    policy_file = json.loads((snapshot_dir / "policy-file.json").read_text(encoding="utf-8"))
    verifier_control = json.loads((snapshot_dir / "verifier-control.json").read_text(encoding="utf-8"))
    if verifier_control.get("reference_verifier_blob_sha") is None:
        fail("reference verifier blob SHA is missing")
    if verifier_control.get("artifact_collector_blob_sha") is None:
        fail("artifact collector blob SHA is missing")
    candidate_lock = json.loads((snapshot_dir / "candidate-lock.json").read_text(encoding="utf-8"))
    main_ref = json.loads((snapshot_dir / "main-ref.json").read_text(encoding="utf-8"))
    artifacts = json.loads((snapshot_dir / "artifacts.json").read_text(encoding="utf-8"))
    enumeration = load_canonical_json(snapshot_dir / "artifact-enumeration.json")

    artifact_items = artifacts.get("artifacts")
    if not isinstance(artifact_items, list):
        fail("artifact list is malformed")

    verify_artifact_enumeration(enumeration, artifact_items)

    if len(artifact_items) != 2:
        fail(f"trusted qualification run must contain exactly 2 artifacts, found {len(artifact_items)}")

    subject_from_names = [
        item
        for item in artifact_items
        if re.fullmatch(
            r"fpm-trusted-qualification-[0-9a-f]{40}", item.get("name", "")
        )
    ]
    index_from_names = [
        item
        for item in artifact_items
        if re.fullmatch(
            r"fpm-trusted-qualification-index-[0-9a-f]{40}", item.get("name", "")
        )
    ]
    if len(subject_from_names) != 1:
        fail("expected exactly one receipt artifact")
    if len(index_from_names) != 1:
        fail("expected exactly one index artifact")
    expected_artifact_names = {
        subject_from_names[0]["name"],
        index_from_names[0]["name"],
    }
    if {item.get("name") for item in artifact_items} != expected_artifact_names:
        fail("trusted qualification artifact set contains an unexpected artifact")

    primary_artifact = subject_from_names[0]
    index_artifact = index_from_names[0]
    receipt_subject = require_hex(receipt["subject_sha"], 40, "receipt subject_sha")
    expected_receipt_name = f"fpm-trusted-qualification-{receipt_subject}"
    expected_index_name = f"fpm-trusted-qualification-index-{receipt_subject}"
    if primary_artifact["name"] != expected_receipt_name:
        fail("receipt artifact name does not bind to subject")
    if index_artifact["name"] != expected_index_name:
        fail("index artifact name does not bind to subject")

    for item, label in ((primary_artifact, "receipt"), (index_artifact, "index")):
        if item["expired"] is not False:
            fail(f"{label} artifact is expired")
        validate_artifact_lifetime(item, label)
        if item["size_in_bytes"] <= 0:
            fail(f"{label} artifact is empty")
        if item["workflow_run"]["id"] != trusted_run["id"]:
            fail(f"{label} artifact run mismatch")
        if item["workflow_run"]["repository_id"] != BASE_REPOSITORY_ID:
            fail(f"{label} artifact repository mismatch")
        if item["workflow_run"]["head_repository_id"] != BASE_REPOSITORY_ID:
            fail(f"{label} artifact head repository mismatch")
        require_sha256_prefixed(item["digest"], f"{label} artifact digest")

    receipt_digest, verifier_sha, verifier_blob_sha = verify_receipt(
        receipt=receipt,
        expected_trusted_run_id=trusted_run["id"],
        expected_trusted_run_attempt=trusted_run["run_attempt"],
        candidate_run=candidate_run,
        trusted_run=trusted_run,
        pr=pr,
        commit=commit,
        manifest=manifest,
        policy_file=policy_file,
        verifier_control=verifier_control,
        candidate_lock=candidate_lock,
        main_ref=main_ref,
    )

    verify_index(
        index=index,
        receipt_digest=receipt_digest,
        receipt=receipt,
        primary_artifact=primary_artifact,
        trusted_run_id=trusted_run["id"],
    )

    index_canonical = json.dumps(
        index, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    index_digest = hashlib.sha256(index_canonical).hexdigest()

    current_promotion_eligible = (
        pr["state"] == "open"
        and pr["draft"] is False
        and pr["head"]["sha"] == receipt_subject
        and pr["base"]["sha"] == main_sha
    )

    return {
        "schema": "mycelix.fpm.trusted-qualification-reference-v1",
        "repository": BASE_REPOSITORY,
        "trusted_workflow_run_id": trusted_run["id"],
        "trusted_workflow_run_attempt": trusted_run["run_attempt"],
        "candidate_workflow_run_id": candidate_run["id"],
        "candidate_workflow_run_attempt": candidate_run["run_attempt"],
        "candidate_pr": pr["number"],
        "candidate_sha": receipt_subject,
        "candidate_tree": receipt["subject_tree_sha"],
        "receipt_content_sha256": receipt_digest,
        "receipt_artifact_id": primary_artifact["id"],
        "receipt_artifact_digest": primary_artifact["digest"],
        "receipt_artifact_created_at": primary_artifact["created_at"],
        "receipt_artifact_expires_at": primary_artifact["expires_at"],
        "index_content_sha256": index_digest,
        "index_artifact_id": index_artifact["id"],
        "index_artifact_digest": index_artifact["digest"],
        "index_artifact_created_at": index_artifact["created_at"],
        "index_artifact_expires_at": index_artifact["expires_at"],
        "trusted_policy_sha": receipt["trusted_policy_sha"],
        "trusted_policy_blob_sha": receipt["trusted_policy_blob_sha"],
        "independent_verifier_workflow_sha": verifier_sha,
        "independent_verifier_workflow_blob_sha": verifier_blob_sha,
        "reference_result": "verified",
        "historical_qualification_valid": True,
        "current_promotion_eligible": current_promotion_eligible,
    }


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: verify_fpm_trusted_qualification.py <snapshot-dir>")
    result = verify(Path(sys.argv[1]))
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
