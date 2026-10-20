#!/usr/bin/env python3
"""Validate the v1 constitutional refinement crosswalk using only stdlib."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
DEFAULT_MANIFEST = REPO / "mycelix-governance/specs/constitutional-refinement-crosswalk.v1.json"
DEFAULT_SCHEMA = REPO / "mycelix-governance/specs/constitutional-refinement-crosswalk.v1.schema.json"
RUST_TYPES = REPO / "mycelix-governance/crates/constitutional-claim-lifecycle/src/types.rs"
TLA_MODEL = REPO / "mycelix-governance/specs/ConstitutionalClaimLifecycle.tla"

SCHEMA_ID = "mycelix.constitutional-refinement-crosswalk.v1"
RELATIONSHIP_KINDS = {
    "exact_enumeration",
    "projection",
    "action_refinement",
    "ghost_environment_state",
    "stuttering_refinement",
    "out_of_model",
    "future_unimplemented",
}
ACTION_CARDINALITIES = {
    "one_to_one",
    "concrete_atomic_to_formal_microsteps",
    "formal_atomic_to_concrete_microsteps",
    "environment_plus_concrete_action",
    "projection_only_no_action_correspondence",
}
QUALIFICATION_STATUSES = {"qualified", "pending", "out_of_model"}
FORMAL_ROLES = {"state", "action", "ghost", "derived_predicate", "environment_input", "atom"}
SYMBOL_TYPES = {
    "enum_variant",
    "derived_absence",
    "method",
    "field",
    "transaction_boundary",
    "runtime_entry",
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_blob_sha(path: Path) -> str:
    proc = subprocess.run(
        ["git", "hash-object", str(path)],
        cwd=REPO,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"git hash-object failed for {path}: {proc.stderr.strip()}")
    return proc.stdout.strip()


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def parse_rust_enum_variants(text: str, enum_name: str) -> list[str]:
    marker = f"pub enum {enum_name} {{"
    start = text.find(marker)
    if start < 0:
        raise RuntimeError(f"Rust enum not found: {enum_name}")
    body = text[start + len(marker):]
    depth = 1
    variants: list[str] = []
    for raw_line in body.splitlines():
        line = raw_line.strip()
        if depth == 1:
            match = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\s*(?:,|\{|\()", line)
            if match:
                variants.append(match.group(1))
        depth += raw_line.count("{") - raw_line.count("}")
        if depth == 0:
            break
    if not variants:
        raise RuntimeError(f"no variants parsed for {enum_name}")
    return variants


def parse_tla_string_set(text: str, name: str) -> list[str]:
    match = re.search(rf"{re.escape(name)}\s*==\s*\{{(.*?)\}}", text, re.DOTALL)
    if not match:
        raise RuntimeError(f"TLA+ set not found: {name}")
    values = re.findall(r'"([^"]+)"', match.group(1))
    if not values:
        raise RuntimeError(f"no atoms parsed for TLA+ set {name}")
    return values


def validate_shape(manifest: dict[str, Any], errors: list[str]) -> None:
    required = {
        "schema",
        "crosswalk_id",
        "version",
        "status",
        "qualified_prerequisites",
        "relationships",
        "pending_extensions",
    }
    require(set(manifest) == required, f"top-level keys must be exactly {sorted(required)}", errors)
    require(manifest.get("schema") == SCHEMA_ID, "wrong schema identity", errors)
    require(manifest.get("status") in {"draft", "qualified"}, "status must be draft or qualified", errors)
    require(bool(re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", str(manifest.get("version", "")))), "version must be semantic x.y.z", errors)
    require(isinstance(manifest.get("qualified_prerequisites"), list) and bool(manifest.get("qualified_prerequisites")), "qualified_prerequisites must be a non-empty list", errors)
    require(isinstance(manifest.get("relationships"), list) and bool(manifest.get("relationships")), "relationships must be a non-empty list", errors)
    require(isinstance(manifest.get("pending_extensions"), list), "pending_extensions must be a list", errors)


def validate_prerequisites(items: list[dict[str, Any]], errors: list[str]) -> None:
    ids: set[str] = set()
    for index, item in enumerate(items):
        prefix = f"qualified_prerequisites[{index}]"
        required = {"id", "semantic_head", "verifier_head", "qualification_run_id", "artifact_digest"}
        require(set(item) == required, f"{prefix}: wrong keys", errors)
        require(item.get("id") not in ids, f"{prefix}: duplicate id {item.get('id')}", errors)
        ids.add(str(item.get("id")))
        for key in ("semantic_head", "verifier_head"):
            require(bool(re.fullmatch(r"[0-9a-f]{40}", str(item.get(key, "")))), f"{prefix}: invalid {key}", errors)
        require(isinstance(item.get("qualification_run_id"), int) and item["qualification_run_id"] > 0, f"{prefix}: invalid qualification_run_id", errors)
        require(bool(re.fullmatch(r"sha256:[0-9a-f]{64}", str(item.get("artifact_digest", "")))), f"{prefix}: invalid artifact_digest", errors)


def validate_relationships(items: list[dict[str, Any]], errors: list[str]) -> None:
    ids: set[str] = set()
    for index, item in enumerate(items):
        prefix = f"relationships[{index}]"
        required = {"id", "qualification_status", "kind", "action_cardinality", "concrete", "formal", "rationale"}
        require(set(item) == required, f"{prefix}: wrong keys", errors)
        item_id = str(item.get("id", ""))
        require(bool(item_id) and item_id not in ids, f"{prefix}: duplicate/empty id {item_id}", errors)
        ids.add(item_id)
        q = item.get("qualification_status")
        kind = item.get("kind")
        cardinality = item.get("action_cardinality")
        require(q in QUALIFICATION_STATUSES, f"{prefix}: invalid qualification_status {q}", errors)
        require(kind in RELATIONSHIP_KINDS, f"{prefix}: invalid kind {kind}", errors)
        require(cardinality in ACTION_CARDINALITIES, f"{prefix}: invalid action_cardinality {cardinality}", errors)
        require(not (q == "qualified" and kind in {"future_unimplemented", "out_of_model"}), f"{prefix}: qualified row cannot be {kind}", errors)
        require(bool(str(item.get("rationale", "")).strip()), f"{prefix}: empty rationale", errors)

        concrete = item.get("concrete", {})
        formal = item.get("formal", {})
        concrete_required = {
            "language", "crate", "source_path", "source_git_blob_sha", "symbol_type", "symbol",
            "persisted", "crash_observable", "transaction_boundary",
        }
        formal_required = {
            "formalism", "model_path", "model_git_blob_sha", "symbol", "role", "concrete_commit_correspondence",
        }
        require(set(concrete) == concrete_required, f"{prefix}: wrong concrete keys", errors)
        require(set(formal) == formal_required, f"{prefix}: wrong formal keys", errors)
        require(concrete.get("language") in {"rust", "holochain_runtime"}, f"{prefix}: invalid concrete language", errors)
        require(concrete.get("symbol_type") in SYMBOL_TYPES, f"{prefix}: invalid concrete symbol_type", errors)
        require(isinstance(concrete.get("persisted"), bool), f"{prefix}: persisted must be boolean", errors)
        require(isinstance(concrete.get("crash_observable"), bool), f"{prefix}: crash_observable must be boolean", errors)
        require(formal.get("formalism") in {"tla+", "alloy"}, f"{prefix}: invalid formalism", errors)
        require(formal.get("role") in FORMAL_ROLES, f"{prefix}: invalid formal role", errors)
        require(isinstance(formal.get("concrete_commit_correspondence"), bool), f"{prefix}: concrete_commit_correspondence must be boolean", errors)

        for path_key, digest_key, side in (
            ("source_path", "source_git_blob_sha", concrete),
            ("model_path", "model_git_blob_sha", formal),
        ):
            raw_path = str(side.get(path_key, ""))
            path = REPO / raw_path
            require(path.is_file(), f"{prefix}: missing file {raw_path}", errors)
            if path.is_file():
                actual = git_blob_sha(path)
                require(actual == side.get(digest_key), f"{prefix}: {raw_path} blob drift expected={side.get(digest_key)} actual={actual}", errors)


def validate_pending(items: list[dict[str, Any]], manifest_status: str, errors: list[str]) -> None:
    ids: set[str] = set()
    for index, item in enumerate(items):
        prefix = f"pending_extensions[{index}]"
        required = {"id", "issue", "status", "candidate_semantic_head", "candidate_verifier_head", "reason"}
        require(set(item) == required, f"{prefix}: wrong keys", errors)
        item_id = str(item.get("id", ""))
        require(bool(item_id) and item_id not in ids, f"{prefix}: duplicate/empty id {item_id}", errors)
        ids.add(item_id)
        require(item.get("status") in {"pending_qualification", "pending_design"}, f"{prefix}: invalid status", errors)
        require(isinstance(item.get("issue"), int) and item["issue"] > 0, f"{prefix}: invalid issue", errors)
        for key in ("candidate_semantic_head", "candidate_verifier_head"):
            value = item.get(key)
            require(value is None or bool(re.fullmatch(r"[0-9a-f]{40}", str(value))), f"{prefix}: invalid {key}", errors)
        require(bool(str(item.get("reason", "")).strip()), f"{prefix}: empty reason", errors)
    if manifest_status == "qualified":
        require(not items, "qualified manifest cannot retain pending_extensions", errors)


def validate_census(manifest: dict[str, Any], errors: list[str]) -> dict[str, Any]:
    rust_variants = parse_rust_enum_variants(RUST_TYPES.read_text(encoding="utf-8"), "ClaimLifecycleStatus")
    tla_statuses = parse_tla_string_set(TLA_MODEL.read_text(encoding="utf-8"), "Statuses")

    mapped_rust = [
        row["concrete"]["symbol"].split("::")[-1]
        for row in manifest["relationships"]
        if row["concrete"]["symbol_type"] == "enum_variant"
        and row["concrete"]["source_path"] == "mycelix-governance/crates/constitutional-claim-lifecycle/src/types.rs"
    ]
    mapped_tla = [
        row["formal"]["symbol"]
        for row in manifest["relationships"]
        if row["formal"]["model_path"] == "mycelix-governance/specs/ConstitutionalClaimLifecycle.tla"
        and row["formal"]["role"] == "state"
    ]

    require(len(mapped_rust) == len(set(mapped_rust)), "Rust lifecycle variant mapped more than once", errors)
    require(set(mapped_rust) == set(rust_variants), f"Rust lifecycle census mismatch source={sorted(rust_variants)} mapped={sorted(mapped_rust)}", errors)
    require(len(mapped_tla) == len(set(mapped_tla)), "TLA+ lifecycle status mapped more than once", errors)
    require(set(mapped_tla) == set(tla_statuses), f"TLA+ status census mismatch source={sorted(tla_statuses)} mapped={sorted(mapped_tla)}", errors)

    absent = [row for row in manifest["relationships"] if row["formal"]["symbol"] == "Absent"]
    require(len(absent) == 1 and absent[0]["concrete"]["symbol_type"] == "derived_absence", "Absent must map exactly once to derived_absence", errors)

    return {
        "rust_claim_lifecycle_status_variants": rust_variants,
        "mapped_rust_variants": mapped_rust,
        "tla_status_atoms": tla_statuses,
        "mapped_tla_status_atoms": mapped_tla,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()

    manifest_path = args.manifest.resolve()
    schema_path = args.schema.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    errors: list[str] = []

    require(schema.get("$schema") == "https://json-schema.org/draft/2020-12/schema", "schema file must declare draft 2020-12", errors)
    require(schema.get("$id") == "https://mycelix.dev/schemas/constitutional-refinement-crosswalk.v1.schema.json", "unexpected schema $id", errors)

    validate_shape(manifest, errors)
    if isinstance(manifest.get("qualified_prerequisites"), list):
        validate_prerequisites(manifest["qualified_prerequisites"], errors)
    if isinstance(manifest.get("relationships"), list):
        validate_relationships(manifest["relationships"], errors)
    if isinstance(manifest.get("pending_extensions"), list):
        validate_pending(manifest["pending_extensions"], str(manifest.get("status", "")), errors)

    census: dict[str, Any] = {}
    if isinstance(manifest.get("relationships"), list):
        census = validate_census(manifest, errors)

    receipt = {
        "schema": "mycelix.constitutional-refinement-crosswalk-validation.v1",
        "passed": not errors,
        "manifest": str(manifest_path.relative_to(REPO)),
        "manifest_sha256": sha256_file(manifest_path),
        "schema_file": str(schema_path.relative_to(REPO)),
        "schema_sha256": sha256_file(schema_path),
        "rust_types_git_blob_sha": git_blob_sha(RUST_TYPES),
        "tla_model_git_blob_sha": git_blob_sha(TLA_MODEL),
        "census": census,
        "errors": errors,
    }

    rendered = json.dumps(receipt, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
