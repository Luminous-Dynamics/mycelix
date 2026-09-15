from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

MANIFEST_VERSION = "mycelix-evidence-manifest-v1"

TOP_LEVEL_KEYS = {
    "manifest_version",
    "technical_evidence",
    "security_evidence",
    "commercial_evidence",
    "capital_evidence",
    "claims",
    "nonclaims",
}

TECH_KEYS = {
    "id",
    "profile",
    "subject_sha",
    "designated_current_subject_sha",
    "workflow_status",
    "workflow_conclusion",
    "disposition",
    "receipt_sha256",
    "dependencies",
    "nonclaims",
}
SECURITY_KEYS = {"id", "profile", "disposition", "subject_ref", "nonclaims"}
COMMERCIAL_KEYS = {"id", "profile", "disposition", "subject_ref", "paid", "nonclaims"}
CAPITAL_KEYS = {"id", "profile", "disposition", "subject_ref", "nonclaims"}
CLAIM_KEYS = {"id", "text", "authority", "evidence_refs", "model_ref", "nonclaims"}
DEPENDENCY_KEYS = {"id", "required_disposition"}

TECH_DISPOSITIONS = {
    "PASS",
    "FAIL",
    "PENDING",
    "INFRASTRUCTURE_NONEXECUTION",
    "SUPERSEDED",
    "NOT_ASSESSED",
    "UNSUPPORTED",
}
OTHER_DISPOSITIONS = {"PASS", "FAIL", "PENDING", "NOT_ASSESSED", "UNSUPPORTED"}
WORKFLOW_STATUSES = {"queued", "in_progress", "completed", "not_applicable"}
WORKFLOW_CONCLUSIONS = {
    None,
    "success",
    "failure",
    "cancelled",
    "skipped",
    "timed_out",
    "action_required",
}
CLAIM_AUTHORITIES = {
    "Qualified",
    "ExternallyAssured",
    "Observed",
    "CustomerAttested",
    "IndependentlyVerifiedOutcome",
    "Projected",
    "Aspirational",
    "Unsupported",
    "Stale",
}
HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/+-]{0,127}$")


class ManifestError(ValueError):
    pass


def _require_exact_keys(obj: dict[str, Any], allowed: set[str], context: str) -> None:
    unknown = set(obj) - allowed
    if unknown:
        raise ManifestError(f"{context}: unknown keys: {sorted(unknown)}")


def _require_text(value: Any, context: str, max_bytes: int = 512) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ManifestError(f"{context}: expected non-empty trimmed string")
    if len(value.encode("utf-8")) > max_bytes:
        raise ManifestError(f"{context}: string too long")
    return value


def _require_id(value: Any, context: str) -> str:
    text = _require_text(value, context, 128)
    if not ID_RE.fullmatch(text):
        raise ManifestError(f"{context}: invalid identifier")
    return text


def _require_list(value: Any, context: str, max_items: int = 4096) -> list[Any]:
    if not isinstance(value, list):
        raise ManifestError(f"{context}: expected list")
    if len(value) > max_items:
        raise ManifestError(f"{context}: too many items")
    return value


def _validate_dependencies(deps: Any, context: str) -> None:
    for idx, dep in enumerate(_require_list(deps, context, 256)):
        if not isinstance(dep, dict):
            raise ManifestError(f"{context}[{idx}]: expected object")
        _require_exact_keys(dep, DEPENDENCY_KEYS, f"{context}[{idx}]")
        _require_id(dep.get("id"), f"{context}[{idx}].id")
        if dep.get("required_disposition") != "PASS":
            raise ManifestError(
                f"{context}[{idx}].required_disposition: only PASS is supported in v1"
            )


def _validate_nonclaims(value: Any, context: str) -> None:
    for idx, item in enumerate(_require_list(value, context, 256)):
        _require_text(item, f"{context}[{idx}]", 2048)


def _validate_technical(item: dict[str, Any], index: int) -> None:
    context = f"technical_evidence[{index}]"
    _require_exact_keys(item, TECH_KEYS, context)
    _require_id(item.get("id"), f"{context}.id")
    _require_id(item.get("profile"), f"{context}.profile")

    subject = item.get("subject_sha")
    current_subject = item.get("designated_current_subject_sha")
    if not isinstance(subject, str) or not HEX40.fullmatch(subject):
        raise ManifestError(f"{context}.subject_sha: expected lowercase 40-hex SHA")
    if not isinstance(current_subject, str) or not HEX40.fullmatch(current_subject):
        raise ManifestError(
            f"{context}.designated_current_subject_sha: expected lowercase 40-hex SHA"
        )

    status = item.get("workflow_status")
    conclusion = item.get("workflow_conclusion")
    disposition = item.get("disposition")
    if status not in WORKFLOW_STATUSES:
        raise ManifestError(f"{context}.workflow_status: unsupported status")
    if conclusion not in WORKFLOW_CONCLUSIONS:
        raise ManifestError(f"{context}.workflow_conclusion: unsupported conclusion")
    if disposition not in TECH_DISPOSITIONS:
        raise ManifestError(f"{context}.disposition: unsupported disposition")

    receipt = item.get("receipt_sha256")
    if receipt is not None and (
        not isinstance(receipt, str) or not HEX64.fullmatch(receipt)
    ):
        raise ManifestError(
            f"{context}.receipt_sha256: expected null or lowercase 64-hex digest"
        )

    if status in {"queued", "in_progress"}:
        if conclusion is not None:
            raise ManifestError(f"{context}: pending workflow cannot have conclusion")
        if disposition == "PASS":
            raise ManifestError(
                f"{context}: queued/in-progress workflow cannot be PASS"
            )
    if status == "completed" and conclusion is None:
        raise ManifestError(f"{context}: completed workflow requires conclusion")
    if disposition == "PASS":
        if status != "completed" or conclusion != "success":
            raise ManifestError(f"{context}: PASS requires completed/success")
        if receipt is None:
            raise ManifestError(f"{context}: PASS requires receipt_sha256")
        if subject != current_subject:
            raise ManifestError(
                f"{context}: stale historical subject cannot be current PASS"
            )
    if disposition == "PENDING" and status not in {"queued", "in_progress"}:
        raise ManifestError(
            f"{context}: PENDING requires queued/in_progress workflow"
        )

    _validate_dependencies(item.get("dependencies", []), f"{context}.dependencies")
    _validate_nonclaims(item.get("nonclaims", []), f"{context}.nonclaims")


def _validate_simple_plane(
    item: dict[str, Any], index: int, plane: str, keys: set[str]
) -> None:
    context = f"{plane}[{index}]"
    _require_exact_keys(item, keys, context)
    _require_id(item.get("id"), f"{context}.id")
    _require_id(item.get("profile"), f"{context}.profile")
    if item.get("disposition") not in OTHER_DISPOSITIONS:
        raise ManifestError(f"{context}.disposition: unsupported disposition")
    _require_text(item.get("subject_ref"), f"{context}.subject_ref", 1024)
    if plane == "commercial_evidence" and not isinstance(item.get("paid"), bool):
        raise ManifestError(f"{context}.paid: expected boolean")
    _validate_nonclaims(item.get("nonclaims", []), f"{context}.nonclaims")


def _collect_refs(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    refs: dict[str, dict[str, Any]] = {}
    for plane in (
        "technical_evidence",
        "security_evidence",
        "commercial_evidence",
        "capital_evidence",
    ):
        prefix = plane.removesuffix("_evidence")
        for item in manifest[plane]:
            ref = f"{prefix}:{item['id']}"
            if ref in refs:
                raise ManifestError(f"duplicate evidence reference: {ref}")
            refs[ref] = item
    return refs


def _validate_claims(manifest: dict[str, Any]) -> None:
    refs = _collect_refs(manifest)
    for index, claim in enumerate(manifest["claims"]):
        context = f"claims[{index}]"
        if not isinstance(claim, dict):
            raise ManifestError(f"{context}: expected object")
        _require_exact_keys(claim, CLAIM_KEYS, context)
        _require_id(claim.get("id"), f"{context}.id")
        _require_text(claim.get("text"), f"{context}.text", 4096)
        authority = claim.get("authority")
        if authority not in CLAIM_AUTHORITIES:
            raise ManifestError(f"{context}.authority: unsupported authority")

        evidence_refs = _require_list(
            claim.get("evidence_refs"), f"{context}.evidence_refs", 128
        )
        if not evidence_refs and authority not in {"Aspirational", "Unsupported"}:
            raise ManifestError(
                f"{context}: authority {authority} requires evidence_refs"
            )
        for ridx, ref in enumerate(evidence_refs):
            ref = _require_text(ref, f"{context}.evidence_refs[{ridx}]", 256)
            if ref not in refs:
                raise ManifestError(f"{context}: unknown evidence ref {ref}")

        model_ref = claim.get("model_ref")
        if model_ref is not None:
            _require_text(model_ref, f"{context}.model_ref", 1024)
        if authority == "Projected" and model_ref is None:
            raise ManifestError(f"{context}: Projected claim requires model_ref")

        if authority == "Qualified":
            qualifying = []
            for ref in evidence_refs:
                if not ref.startswith("technical:"):
                    continue
                item = refs[ref]
                qualifying.append(
                    item["disposition"] == "PASS"
                    and item["subject_sha"]
                    == item["designated_current_subject_sha"]
                )
            if not any(qualifying):
                raise ManifestError(
                    f"{context}: Qualified claim requires current technical PASS evidence"
                )

        if authority == "ExternallyAssured":
            if not any(
                ref.startswith("security:")
                and refs[ref]["disposition"] == "PASS"
                for ref in evidence_refs
            ):
                raise ManifestError(
                    f"{context}: ExternallyAssured claim requires security PASS evidence"
                )

        _validate_nonclaims(claim.get("nonclaims", []), f"{context}.nonclaims")


def validate_manifest(manifest: Any) -> dict[str, Any]:
    if not isinstance(manifest, dict):
        raise ManifestError("manifest: expected object")
    _require_exact_keys(manifest, TOP_LEVEL_KEYS, "manifest")
    if manifest.get("manifest_version") != MANIFEST_VERSION:
        raise ManifestError("manifest_version: unsupported profile")

    for key in (
        "technical_evidence",
        "security_evidence",
        "commercial_evidence",
        "capital_evidence",
        "claims",
        "nonclaims",
    ):
        _require_list(manifest.get(key), key)

    seen_ids: set[tuple[str, str]] = set()
    for idx, item in enumerate(manifest["technical_evidence"]):
        if not isinstance(item, dict):
            raise ManifestError(f"technical_evidence[{idx}]: expected object")
        _validate_technical(item, idx)
        marker = ("technical", item["id"])
        if marker in seen_ids:
            raise ManifestError(f"duplicate technical evidence id: {item['id']}")
        seen_ids.add(marker)

    for plane, keys in (
        ("security_evidence", SECURITY_KEYS),
        ("commercial_evidence", COMMERCIAL_KEYS),
        ("capital_evidence", CAPITAL_KEYS),
    ):
        prefix = plane.removesuffix("_evidence")
        for idx, item in enumerate(manifest[plane]):
            if not isinstance(item, dict):
                raise ManifestError(f"{plane}[{idx}]: expected object")
            _validate_simple_plane(item, idx, plane, keys)
            marker = (prefix, item["id"])
            if marker in seen_ids:
                raise ManifestError(
                    f"duplicate {prefix} evidence id: {item['id']}"
                )
            seen_ids.add(marker)

    _validate_nonclaims(manifest["nonclaims"], "nonclaims")
    _validate_claims(manifest)
    return manifest


def _currentness(item: dict[str, Any]) -> str:
    if item["subject_sha"] == item["designated_current_subject_sha"]:
        return "Current"
    return "Historical/Stale"


def render_markdown(manifest: dict[str, Any]) -> str:
    validate_manifest(manifest)
    lines = [
        "# Mycelix Evidence Status",
        "",
        f"Profile: `{MANIFEST_VERSION}`",
        "",
        "## Technical evidence",
        "",
        "| ID | Profile | Disposition | Workflow | Currentness | Receipt |",
        "|---|---|---|---|---|---|",
    ]
    for item in manifest["technical_evidence"]:
        receipt = (
            item["receipt_sha256"][:12] + "…"
            if item["receipt_sha256"]
            else "—"
        )
        workflow = item["workflow_status"]
        if item["workflow_conclusion"] is not None:
            workflow += f"/{item['workflow_conclusion']}"
        lines.append(
            f"| `{item['id']}` | `{item['profile']}` | "
            f"**{item['disposition']}** | {workflow} | {_currentness(item)} | "
            f"`{receipt}` |"
        )

    lines += [
        "",
        "## Other evidence planes",
        "",
        f"- Security evidence records: {len(manifest['security_evidence'])}",
        f"- Commercial evidence records: {len(manifest['commercial_evidence'])}",
        f"- Capital evidence records: {len(manifest['capital_evidence'])}",
        "",
        "## Claims",
        "",
        "| Claim | Authority | Evidence |",
        "|---|---|---|",
    ]
    for claim in manifest["claims"]:
        refs = ", ".join(f"`{ref}`" for ref in claim["evidence_refs"]) or "—"
        lines.append(
            f"| {claim['text']} | **{claim['authority']}** | {refs} |"
        )

    if manifest["nonclaims"]:
        lines += ["", "## Manifest nonclaims", ""]
        lines.extend(f"- {item}" for item in manifest["nonclaims"])

    lines.append("")
    return "\n".join(lines)


def canonical_json_bytes(manifest: dict[str, Any]) -> bytes:
    validate_manifest(manifest)
    encoded = json.dumps(
        manifest,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return (encoded + "\n").encode("utf-8")


def manifest_sha256(manifest: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_json_bytes(manifest)).hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate and render a Mycelix evidence manifest v1."
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--markdown-out", type=Path)
    parser.add_argument("--print-sha256", action="store_true")
    args = parser.parse_args(argv)

    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        validate_manifest(manifest)
        rendered = render_markdown(manifest)
    except (OSError, json.JSONDecodeError, ManifestError) as exc:
        print(f"manifest validation failed: {exc}", file=sys.stderr)
        return 1

    if args.markdown_out:
        args.markdown_out.write_text(rendered, encoding="utf-8", newline="\n")
    else:
        sys.stdout.write(rendered)
    if args.print_sha256:
        print(manifest_sha256(manifest), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
