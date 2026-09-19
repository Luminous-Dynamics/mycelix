from __future__ import annotations

import json
import re
from typing import Any

from mycelix_github_qualification_source import normalize_github_qualification_source_v1

PROFILE_VERSION = "mycelix-provider-source-evidence-v1"
GITHUB_QUALIFICATION_SOURCE_KIND = "github-qualification-source-v1"

POLICY_KEYS = {
    "profile_version",
    "provider_profile",
    "acquisition_profile",
    "source_kind",
    "repository_id",
}
ENVELOPE_KEYS = {
    "profile_version",
    "provider_profile",
    "acquisition_profile",
    "source_kind",
    "repository_id",
    "provider_object_refs",
    "run_id",
    "run_attempt",
    "evidence_id",
}
OBJECT_REF_KEYS = {"kind", "reference"}

ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/+@=-]{0,255}$")


class SourceEvidenceError(ValueError):
    pass


def _exact_keys(obj: dict[str, Any], allowed: set[str], context: str) -> None:
    if not isinstance(obj, dict):
        raise SourceEvidenceError(f"{context}: expected object")
    unknown = set(obj) - allowed
    missing = allowed - set(obj)
    if unknown or missing:
        raise SourceEvidenceError(
            f"{context}: exact keys required; unknown={sorted(unknown)} missing={sorted(missing)}"
        )


def _text(value: Any, context: str, max_bytes: int = 2048) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SourceEvidenceError(f"{context}: expected non-empty trimmed string")
    if len(value.encode("utf-8")) > max_bytes or any(ord(ch) < 32 for ch in value):
        raise SourceEvidenceError(f"{context}: invalid or overlong string")
    return value


def _id(value: Any, context: str) -> str:
    value = _text(value, context, 256)
    if not ID_RE.fullmatch(value):
        raise SourceEvidenceError(f"{context}: invalid identifier")
    return value


def _positive_int(value: Any, context: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise SourceEvidenceError(f"{context}: expected positive integer")
    return value


def validate_source_evidence_policy_v1(policy: dict[str, Any]) -> dict[str, Any]:
    _exact_keys(policy, POLICY_KEYS, "policy")
    if policy["profile_version"] != PROFILE_VERSION:
        raise SourceEvidenceError("policy.profile_version: unsupported profile")
    _id(policy["provider_profile"], "policy.provider_profile")
    _id(policy["acquisition_profile"], "policy.acquisition_profile")
    _id(policy["source_kind"], "policy.source_kind")
    _positive_int(policy["repository_id"], "policy.repository_id")
    return policy


def _validate_provider_object_refs(value: Any) -> list[dict[str, str]]:
    if not isinstance(value, list) or not value or len(value) > 64:
        raise SourceEvidenceError(
            "envelope.provider_object_refs: expected bounded non-empty list"
        )

    normalized: list[dict[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for index, item in enumerate(value):
        _exact_keys(item, OBJECT_REF_KEYS, f"envelope.provider_object_refs[{index}]")
        kind = _id(item["kind"], f"envelope.provider_object_refs[{index}].kind")
        reference = _text(
            item["reference"], f"envelope.provider_object_refs[{index}].reference"
        )
        identity = (kind, reference)
        if identity in seen:
            raise SourceEvidenceError("envelope.provider_object_refs: duplicate object reference")
        seen.add(identity)
        normalized.append({"kind": kind, "reference": reference})

    normalized.sort(key=lambda item: (item["kind"], item["reference"]))
    return normalized


def validate_source_evidence_envelope_v1(
    policy: dict[str, Any], envelope: dict[str, Any]
) -> dict[str, Any]:
    validate_source_evidence_policy_v1(policy)
    _exact_keys(envelope, ENVELOPE_KEYS, "envelope")
    if envelope["profile_version"] != PROFILE_VERSION:
        raise SourceEvidenceError("envelope.profile_version: unsupported profile")

    provider_profile = _id(envelope["provider_profile"], "envelope.provider_profile")
    acquisition_profile = _id(
        envelope["acquisition_profile"], "envelope.acquisition_profile"
    )
    source_kind = _id(envelope["source_kind"], "envelope.source_kind")
    repository_id = _positive_int(envelope["repository_id"], "envelope.repository_id")
    run_id = _positive_int(envelope["run_id"], "envelope.run_id")
    run_attempt = _positive_int(envelope["run_attempt"], "envelope.run_attempt")
    evidence_id = _id(envelope["evidence_id"], "envelope.evidence_id")
    provider_object_refs = _validate_provider_object_refs(envelope["provider_object_refs"])

    if provider_profile != policy["provider_profile"]:
        raise SourceEvidenceError("envelope.provider_profile: policy mismatch")
    if acquisition_profile != policy["acquisition_profile"]:
        raise SourceEvidenceError("envelope.acquisition_profile: policy mismatch")
    if source_kind != policy["source_kind"]:
        raise SourceEvidenceError("envelope.source_kind: policy mismatch")
    if repository_id != policy["repository_id"]:
        raise SourceEvidenceError("envelope.repository_id: policy mismatch")

    return {
        "profile_version": PROFILE_VERSION,
        "provider_profile": provider_profile,
        "acquisition_profile": acquisition_profile,
        "source_kind": source_kind,
        "repository_id": repository_id,
        "provider_object_refs": provider_object_refs,
        "run_id": run_id,
        "run_attempt": run_attempt,
        "evidence_id": evidence_id,
    }


def validate_source_evidence_set_v1(
    policy: dict[str, Any], envelopes: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    if not isinstance(envelopes, list) or len(envelopes) > 4096:
        raise SourceEvidenceError("envelopes: expected bounded list")

    by_evidence_id: dict[str, dict[str, Any]] = {}
    for envelope in envelopes:
        validated = validate_source_evidence_envelope_v1(policy, envelope)
        evidence_id = validated["evidence_id"]
        previous = by_evidence_id.get(evidence_id)
        if previous is not None and previous != validated:
            raise SourceEvidenceError(
                "envelopes: one evidence identity cannot name multiple provider-source envelopes"
            )
        by_evidence_id[evidence_id] = validated

    return [by_evidence_id[key] for key in sorted(by_evidence_id)]


def _object_kinds(envelope: dict[str, Any]) -> set[str]:
    return {item["kind"] for item in envelope["provider_object_refs"]}


def _object_references(envelope: dict[str, Any]) -> set[str]:
    return {item["reference"] for item in envelope["provider_object_refs"]}


def bind_github_qualification_source_v1(
    policy: dict[str, Any],
    envelope: dict[str, Any],
    source: dict[str, Any],
) -> dict[str, Any]:
    bound_envelope = validate_source_evidence_envelope_v1(policy, envelope)
    if bound_envelope["source_kind"] != GITHUB_QUALIFICATION_SOURCE_KIND:
        raise SourceEvidenceError(
            "envelope.source_kind: expected github qualification source profile"
        )

    if source.get("provider") != "github":
        raise SourceEvidenceError("source.provider: expected github")
    if source.get("repository_id") != bound_envelope["repository_id"]:
        raise SourceEvidenceError("source.repository_id: envelope mismatch")
    if source.get("run_id") != bound_envelope["run_id"]:
        raise SourceEvidenceError("source.run_id: envelope mismatch")
    if source.get("run_attempt") != bound_envelope["run_attempt"]:
        raise SourceEvidenceError("source.run_attempt: envelope mismatch")
    if source.get("source_evidence_id") != bound_envelope["evidence_id"]:
        raise SourceEvidenceError("source.source_evidence_id: envelope mismatch")

    source_references = source.get("source_references")
    if not isinstance(source_references, list):
        raise SourceEvidenceError("source.source_references: expected list")
    missing_references = _object_references(bound_envelope) - set(source_references)
    if missing_references:
        raise SourceEvidenceError(
            f"source.source_references: missing envelope references {sorted(missing_references)}"
        )

    kinds = _object_kinds(bound_envelope)
    receipt_present = any(
        source.get(field) is not None
        for field in ("receipt_sha256", "receipt_subject_sha", "receipt_profile")
    )
    if receipt_present and not ({"qualification_receipt", "qualification_artifact"} & kinds):
        raise SourceEvidenceError(
            "envelope.provider_object_refs: receipt-bearing source requires receipt/artifact provenance"
        )

    assertion = source.get("exact_subject_assertion")
    if assertion != "unproven" and "tested_subject_proof" not in kinds:
        raise SourceEvidenceError(
            "envelope.provider_object_refs: exact-subject assertion requires tested-subject proof provenance"
        )

    return bound_envelope


def normalize_bound_github_qualification_source_v1(
    policy: dict[str, Any],
    envelope: dict[str, Any],
    source_profile: dict[str, Any],
    source: dict[str, Any],
) -> dict[str, Any]:
    bound_envelope = bind_github_qualification_source_v1(policy, envelope, source)
    normalized_source = normalize_github_qualification_source_v1(source_profile, source)
    return {
        "source_evidence": bound_envelope,
        "normalized_source": normalized_source,
    }


def canonical_bound_source_bytes_v1(
    policy: dict[str, Any],
    envelope: dict[str, Any],
    source_profile: dict[str, Any],
    source: dict[str, Any],
) -> bytes:
    bound = normalize_bound_github_qualification_source_v1(
        policy, envelope, source_profile, source
    )
    return (
        json.dumps(
            bound,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        )
        + "\n"
    ).encode("utf-8")
