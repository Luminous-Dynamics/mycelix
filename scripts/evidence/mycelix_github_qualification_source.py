from __future__ import annotations

import hashlib
import json
import re
from typing import Any

PROFILE_VERSION = "mycelix-github-qualification-source-v1"
PROVIDER = "github"

PROFILE_KEYS = {
    "profile_version",
    "source_profile",
    "technical_profile",
    "repository_id",
    "workflow_path",
    "designated_subject_sha",
    "designated_subject_tree_sha",
    "allowed_workflow_events",
    "expected_receipt_profile",
}
SOURCE_KEYS = {
    "provider",
    "source_evidence_id",
    "repository_id",
    "workflow_path",
    "workflow_event",
    "run_id",
    "run_attempt",
    "run_head_sha",
    "workflow_status",
    "workflow_conclusion",
    "tested_subject_sha",
    "tested_tree_sha",
    "exact_subject_assertion",
    "receipt_sha256",
    "receipt_subject_sha",
    "receipt_profile",
    "source_references",
}

WORKFLOW_STATUSES = {"queued", "in_progress", "completed"}
WORKFLOW_CONCLUSIONS = {
    None,
    "success",
    "failure",
    "cancelled",
    "skipped",
    "timed_out",
    "action_required",
}
EXACT_SUBJECT_ASSERTIONS = {
    "runtime_exact_checkout",
    "detached_exact_replay",
    "tree_equivalence_only",
    "merge_ref_only",
    "unproven",
}
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/+@=-]{0,255}$")
HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")


class SourceAdapterError(ValueError):
    pass


def _exact_keys(obj: dict[str, Any], allowed: set[str], context: str) -> None:
    if not isinstance(obj, dict):
        raise SourceAdapterError(f"{context}: expected object")
    unknown = set(obj) - allowed
    missing = allowed - set(obj)
    if unknown or missing:
        raise SourceAdapterError(
            f"{context}: exact keys required; unknown={sorted(unknown)} missing={sorted(missing)}"
        )


def _text(value: Any, context: str, max_bytes: int = 512) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SourceAdapterError(f"{context}: expected non-empty trimmed string")
    if len(value.encode("utf-8")) > max_bytes or any(ord(ch) < 32 for ch in value):
        raise SourceAdapterError(f"{context}: invalid or overlong string")
    return value


def _id(value: Any, context: str) -> str:
    value = _text(value, context, 256)
    if not ID_RE.fullmatch(value):
        raise SourceAdapterError(f"{context}: invalid identifier")
    return value


def _sha40(value: Any, context: str, *, optional: bool = False) -> str | None:
    if value is None and optional:
        return None
    if not isinstance(value, str) or not HEX40.fullmatch(value):
        raise SourceAdapterError(f"{context}: expected lowercase 40-hex SHA")
    return value


def _sha64(value: Any, context: str, *, optional: bool = False) -> str | None:
    if value is None and optional:
        return None
    if not isinstance(value, str) or not HEX64.fullmatch(value):
        raise SourceAdapterError(f"{context}: expected lowercase 64-hex digest")
    return value


def _positive_int(value: Any, context: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise SourceAdapterError(f"{context}: expected positive integer")
    return value


def validate_profile(profile: dict[str, Any]) -> dict[str, Any]:
    _exact_keys(profile, PROFILE_KEYS, "profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise SourceAdapterError("profile.profile_version: unsupported profile")
    _id(profile["source_profile"], "profile.source_profile")
    _id(profile["technical_profile"], "profile.technical_profile")
    _positive_int(profile["repository_id"], "profile.repository_id")
    _text(profile["workflow_path"], "profile.workflow_path", 512)
    _sha40(profile["designated_subject_sha"], "profile.designated_subject_sha")
    _sha40(
        profile["designated_subject_tree_sha"],
        "profile.designated_subject_tree_sha",
        optional=True,
    )
    events = profile["allowed_workflow_events"]
    if not isinstance(events, list) or not events or len(events) > 32:
        raise SourceAdapterError("profile.allowed_workflow_events: expected bounded non-empty list")
    normalized_events = []
    for index, event in enumerate(events):
        normalized_events.append(_id(event, f"profile.allowed_workflow_events[{index}]"))
    if len(set(normalized_events)) != len(normalized_events):
        raise SourceAdapterError("profile.allowed_workflow_events: duplicate event")
    _id(profile["expected_receipt_profile"], "profile.expected_receipt_profile")
    return profile


def _validate_receipt(profile: dict[str, Any], source: dict[str, Any]) -> bool:
    fields = (
        source["receipt_sha256"],
        source["receipt_subject_sha"],
        source["receipt_profile"],
    )
    present = [value is not None for value in fields]
    if any(present) and not all(present):
        raise SourceAdapterError("source: receipt fields must be all present or all null")
    if not any(present):
        return False

    _sha64(source["receipt_sha256"], "source.receipt_sha256")
    receipt_subject = _sha40(source["receipt_subject_sha"], "source.receipt_subject_sha")
    receipt_profile = _id(source["receipt_profile"], "source.receipt_profile")
    tested_subject = source["tested_subject_sha"]
    if tested_subject is None:
        raise SourceAdapterError(
            "source.receipt_subject_sha: receipt cannot substitute for missing tested-subject evidence"
        )
    if receipt_subject != tested_subject:
        raise SourceAdapterError(
            "source.receipt_subject_sha: receipt/tested-subject mismatch"
        )
    if receipt_profile != profile["expected_receipt_profile"]:
        raise SourceAdapterError("source.receipt_profile: profile mismatch")
    return True


def _validate_source_references(value: Any) -> list[str]:
    if not isinstance(value, list) or not value or len(value) > 32:
        raise SourceAdapterError("source.source_references: expected bounded non-empty list")
    result: list[str] = []
    for index, reference in enumerate(value):
        result.append(_text(reference, f"source.source_references[{index}]", 2048))
    if len(set(result)) != len(result):
        raise SourceAdapterError("source.source_references: duplicate reference")
    return result


def _exact_subject_proven(profile: dict[str, Any], source: dict[str, Any]) -> bool:
    designated = profile["designated_subject_sha"]
    tested = source["tested_subject_sha"]
    assertion = source["exact_subject_assertion"]

    if assertion == "runtime_exact_checkout":
        return tested == designated and source["run_head_sha"] == designated
    if assertion == "detached_exact_replay":
        return tested == designated
    if assertion in {"tree_equivalence_only", "merge_ref_only", "unproven"}:
        return False
    raise SourceAdapterError("source.exact_subject_assertion: unsupported assertion")


def _validate_assertion_shape(profile: dict[str, Any], source: dict[str, Any]) -> None:
    assertion = source["exact_subject_assertion"]
    tested_subject = source["tested_subject_sha"]
    tested_tree = source["tested_tree_sha"]
    designated = profile["designated_subject_sha"]

    if assertion == "runtime_exact_checkout":
        if tested_subject != designated or source["run_head_sha"] != designated:
            raise SourceAdapterError(
                "source.exact_subject_assertion: runtime exact checkout bindings do not match designated subject"
            )
    elif assertion == "detached_exact_replay":
        if tested_subject != designated:
            raise SourceAdapterError(
                "source.exact_subject_assertion: detached replay did not test designated subject"
            )
    elif assertion == "tree_equivalence_only":
        expected_tree = profile["designated_subject_tree_sha"]
        if expected_tree is None or tested_tree != expected_tree:
            raise SourceAdapterError(
                "source.exact_subject_assertion: tree-equivalence proof does not match designated tree"
            )
        if tested_subject == designated:
            raise SourceAdapterError(
                "source.exact_subject_assertion: exact commit is already known; do not downgrade to tree-only proof"
            )
    elif assertion == "merge_ref_only":
        if tested_subject is None:
            raise SourceAdapterError(
                "source.exact_subject_assertion: merge-ref profile requires observed tested commit"
            )
    elif assertion == "unproven":
        pass
    else:
        raise SourceAdapterError("source.exact_subject_assertion: unsupported assertion")


def _derive_disposition(
    source: dict[str, Any],
    exact_subject_proven: bool,
    receipt_present: bool,
) -> str:
    status = source["workflow_status"]
    conclusion = source["workflow_conclusion"]

    if status in {"queued", "in_progress"}:
        return "PENDING"
    if not exact_subject_proven:
        return "UNSUPPORTED"
    if conclusion == "success" and receipt_present:
        return "PASS"

    # V1 does not derive semantic RED from a run-level failure. Checkout/setup/upload or
    # unrelated job failures are execution facts, not registered theorem-gate failures.
    # A future step-level/negative-receipt composition theorem may establish FAIL.
    return "NOT_ASSESSED"


def normalize_github_qualification_source_v1(
    profile: dict[str, Any],
    source: dict[str, Any],
) -> dict[str, Any]:
    validate_profile(profile)
    _exact_keys(source, SOURCE_KEYS, "source")

    if source["provider"] != PROVIDER:
        raise SourceAdapterError("source.provider: expected github")
    _id(source["source_evidence_id"], "source.source_evidence_id")
    if source["repository_id"] != profile["repository_id"]:
        raise SourceAdapterError("source.repository_id: repository mismatch")
    if source["workflow_path"] != profile["workflow_path"]:
        raise SourceAdapterError("source.workflow_path: workflow path mismatch")
    workflow_event = _id(source["workflow_event"], "source.workflow_event")
    if workflow_event not in profile["allowed_workflow_events"]:
        raise SourceAdapterError("source.workflow_event: event not admitted by profile")

    _positive_int(source["run_id"], "source.run_id")
    _positive_int(source["run_attempt"], "source.run_attempt")
    _sha40(source["run_head_sha"], "source.run_head_sha")
    tested_subject = _sha40(
        source["tested_subject_sha"], "source.tested_subject_sha", optional=True
    )
    tested_tree = _sha40(
        source["tested_tree_sha"], "source.tested_tree_sha", optional=True
    )

    status = source["workflow_status"]
    conclusion = source["workflow_conclusion"]
    if status not in WORKFLOW_STATUSES:
        raise SourceAdapterError("source.workflow_status: unsupported status")
    if conclusion not in WORKFLOW_CONCLUSIONS:
        raise SourceAdapterError("source.workflow_conclusion: unsupported conclusion")
    if status in {"queued", "in_progress"} and conclusion is not None:
        raise SourceAdapterError("source: nonterminal workflow cannot have conclusion")
    if status == "completed" and conclusion is None:
        raise SourceAdapterError("source: completed workflow requires conclusion")

    assertion = source["exact_subject_assertion"]
    if assertion not in EXACT_SUBJECT_ASSERTIONS:
        raise SourceAdapterError("source.exact_subject_assertion: unsupported assertion")
    _validate_assertion_shape(profile, source)
    references = _validate_source_references(source["source_references"])
    receipt_present = _validate_receipt(profile, source)
    exact_subject_proven = _exact_subject_proven(profile, source)
    disposition = _derive_disposition(source, exact_subject_proven, receipt_present)

    return {
        "normalization_profile": PROFILE_VERSION,
        "source_profile": profile["source_profile"],
        "technical_profile": profile["technical_profile"],
        "source_evidence_id": source["source_evidence_id"],
        "repository_id": source["repository_id"],
        "workflow_path": source["workflow_path"],
        "workflow_event": workflow_event,
        "run_id": source["run_id"],
        "run_attempt": source["run_attempt"],
        "run_head_sha": source["run_head_sha"],
        "tested_subject_sha": tested_subject,
        "tested_tree_sha": tested_tree,
        "designated_subject_sha": profile["designated_subject_sha"],
        "designated_subject_tree_sha": profile["designated_subject_tree_sha"],
        "exact_subject_assertion": assertion,
        "exact_subject_proven": exact_subject_proven,
        "workflow_status": status,
        "workflow_conclusion": conclusion,
        "disposition": disposition,
        "receipt_sha256": source["receipt_sha256"],
        "receipt_subject_sha": source["receipt_subject_sha"],
        "receipt_profile": source["receipt_profile"],
        "source_references": references,
    }


def _project_normalized_technical_evidence_v1(
    normalized: dict[str, Any],
    *,
    evidence_id: str,
    designated_current_subject_sha: str,
    dependencies: list[dict[str, Any]] | None = None,
    nonclaims: list[str] | None = None,
) -> dict[str, Any]:
    _id(evidence_id, "evidence_id")
    _sha40(designated_current_subject_sha, "designated_current_subject_sha")
    if normalized.get("normalization_profile") != PROFILE_VERSION:
        raise SourceAdapterError("normalized source: wrong normalization profile")

    return {
        "id": evidence_id,
        "profile": normalized["technical_profile"],
        "subject_sha": normalized["designated_subject_sha"],
        "designated_current_subject_sha": designated_current_subject_sha,
        "workflow_status": normalized["workflow_status"],
        "workflow_conclusion": normalized["workflow_conclusion"],
        "disposition": normalized["disposition"],
        "receipt_sha256": (
            normalized["receipt_sha256"]
            if normalized["disposition"] == "PASS"
            else None
        ),
        "dependencies": [] if dependencies is None else dependencies,
        "nonclaims": [] if nonclaims is None else nonclaims,
    }


def project_github_technical_evidence_v1(
    profile: dict[str, Any],
    source: dict[str, Any],
    *,
    evidence_id: str,
    designated_current_subject_sha: str,
    dependencies: list[dict[str, Any]] | None = None,
    nonclaims: list[str] | None = None,
) -> dict[str, Any]:
    normalized = normalize_github_qualification_source_v1(profile, source)
    return _project_normalized_technical_evidence_v1(
        normalized,
        evidence_id=evidence_id,
        designated_current_subject_sha=designated_current_subject_sha,
        dependencies=dependencies,
        nonclaims=nonclaims,
    )


def canonical_normalized_source_bytes_v1(normalized: dict[str, Any]) -> bytes:
    if normalized.get("normalization_profile") != PROFILE_VERSION:
        raise SourceAdapterError("normalized source: wrong normalization profile")
    return (
        json.dumps(
            normalized,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        )
        + "\n"
    ).encode("utf-8")


def normalized_source_sha256_v1(normalized: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_normalized_source_bytes_v1(normalized)).hexdigest()
