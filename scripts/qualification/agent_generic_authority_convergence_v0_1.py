#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/agents/agent-generic-authority-convergence-v0.1.json"
DOC = ROOT / "mycelix-workspace/docs/agents/AGENT_GENERIC_AUTHORITY_CONVERGENCE_V0.1.md"

EXPECTED_AGENT_PARENT = "7b729880b651ab1e476baad3903d4620041223ed"
EXPECTED_AUTHORITY_PARENT = "5e945700c4c0dba015f571acd2e754475a5143fc"
EXPECTED_HISTORICAL_SOURCE = "8e4a04f9e03caf36bc58cd30bc5ce92e40c8fe0f"
EXPECTED_RECEIPT_SHA256 = "9c444ebc06595fcd19b2fcc28fd93af859cac4376f1c0d615faf4c3afc39fa05"
EXPECTED_LOCK_SHA256 = "38889f5045c6e5f6ea3b9427e056429b316021b244cdec6fda4fbd1afb49888d"
EXPECTED_ROOTS = [
    "crates/mycelix-institutional-core",
    "crates/mycelix-authority-identity",
    "crates/mycelix-authority-freshness",
    "crates/mycelix-authority-delegation-policy",
    "crates/mycelix-authority-delegation",
]
EXPECTED_ROOT_TREES = {
    "crates/mycelix-institutional-core": "6602af340acaa660ffd7e4d46a2f84e67009f396",
    "crates/mycelix-authority-identity": "ab98e976ee7acc67e7b5d2af7fc0d16a476f2710",
    "crates/mycelix-authority-freshness": "76ec24222cfbd7a7d528996eb27df2f33dcc3c5a",
    "crates/mycelix-authority-delegation-policy": "9c41ee69990e0fbb78a58dd82ed693df0889c862",
    "crates/mycelix-authority-delegation": "665db95344b9787a377a4e06c74e434914535389",
}
EXPECTED_RECEIPT_KEYS = {
    "admitted_root_trees",
    "authority_source_full_qualification_inherited",
    "cargo_offline_for_semantic_commands",
    "cargo_version",
    "checkout_action",
    "github_repository",
    "github_run_attempt",
    "github_run_id",
    "github_workflow",
    "local_build_hooks_allowed",
    "local_git_dependencies_allowed",
    "os_network_sandbox_claimed",
    "qualification_lock_sha256",
    "qualification_pass",
    "runner_arch",
    "runner_os",
    "rust_toolchain",
    "rust_toolchain_action",
    "schema",
    "source_parent",
    "subject_head",
}
EXACT_QUALIFICATION_PATHS = {
    ".github/workflows/agent-generic-authority-convergence.yml",
    "scripts/qualification/agent_generic_authority_convergence_v0_1.py",
    "mycelix-workspace/docs/agents/AGENT_GENERIC_AUTHORITY_CONVERGENCE_V0.1.md",
    "mycelix-workspace/docs/agents/agent-generic-authority-convergence-v0.1.json",
    "mycelix-workspace/docs/agents/evidence/agent-generic-authority-convergence-v0.1/authority-source-receipt.json",
    "mycelix-workspace/docs/agents/evidence/agent-generic-authority-convergence-v0.1/authority-source-Cargo.lock",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", "-C", ROOT, *args], text=True).strip()


def reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_json_strict(path: Path) -> tuple[dict[str, object], bytes]:
    raw = path.read_bytes()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise SystemExit(f"non-UTF-8 JSON evidence: {path}: {exc}") from exc
    try:
        data = json.loads(text, object_pairs_hook=reject_duplicate_keys)
    except (json.JSONDecodeError, ValueError) as exc:
        raise SystemExit(f"invalid strict JSON: {path}: {exc}") from exc
    require(type(data) is dict, f"JSON root must be object: {path}")
    return data, raw


def exact_type(value: object, expected: type, label: str) -> None:
    require(type(value) is expected, f"{label} has wrong JSON type")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> None:
    manifest, _ = load_json_strict(MANIFEST)

    require(manifest.get("schema") == "mycelix.agent.generic-authority-convergence.v0.1", "wrong manifest schema")
    require(manifest.get("agent_parent") == EXPECTED_AGENT_PARENT, "wrong AGENT parent")
    require(manifest.get("agent_parent_qualification_run") == 34788543972, "wrong AGENT parent qualification run")
    require(manifest.get("authority_parent") == EXPECTED_AUTHORITY_PARENT, "wrong authority parent")
    require(manifest.get("authority_parent_pr") == 809, "wrong authority parent PR")
    require(manifest.get("authority_parent_semantic_qualification_run") == 34833153899, "wrong authority qualification run")
    require(manifest.get("authority_parent_semantic_qualification_attempt") == 1, "wrong authority qualification attempt")
    require(manifest.get("authority_parent_semantic_qualification_pass") is True, "authority parent PASS must be explicit")
    require(manifest.get("authority_parent_receipt_schema") == "mycelix.authority-delegation-semantic-repair.receipt.v0.2", "wrong receipt schema in manifest")
    require(manifest.get("authority_parent_receipt_sha256") == EXPECTED_RECEIPT_SHA256, "wrong receipt SHA in manifest")
    require(manifest.get("authority_parent_qualification_lock_sha256") == EXPECTED_LOCK_SHA256, "wrong lock SHA in manifest")
    require(manifest.get("historical_authority_source") == EXPECTED_HISTORICAL_SOURCE, "wrong historical authority source")
    require(manifest.get("historical_authority_source_pr") == 77, "wrong historical source PR")
    require(manifest.get("historical_authority_source_workflow_run") == 33865664837, "wrong historical source run")
    require(manifest.get("authority_source_full_qualification_inherited") is False, "must not inherit #77 full qualification")
    require(manifest.get("semantic_requalification_toolchain") == "1.98.1", "semantic Rust toolchain drift")
    require(manifest.get("semantic_requalification_cargo") == "cargo 1.98.1 (797e8a9bc 2026-08-05)", "semantic Cargo drift")
    require(manifest.get("checkout_action") == "actions/checkout@11bd71901bbe5b1630ceea73d27597364c9af683", "checkout action drift")
    require(manifest.get("rust_toolchain_action") == "dtolnay/rust-toolchain@6bed0761d98439e5a578e2877258200ad565ba87", "Rust action drift")
    require(manifest.get("admitted_roots") == EXPECTED_ROOTS, "admitted roots drift")
    require(manifest.get("admitted_root_trees") == EXPECTED_ROOT_TREES, "admitted root tree census drift")
    require(manifest.get("agent_002_identity_claim_blocked") is True, "AGENT-002 claim must remain blocked")
    require(manifest.get("full_agent_security_claim_blocked") is True, "full-security claim must remain blocked")

    receipt_path = ROOT / str(manifest["authority_parent_receipt_file"])
    lock_path = ROOT / str(manifest["authority_parent_lock_file"])
    receipt, receipt_raw = load_json_strict(receipt_path)

    require(set(receipt) == EXPECTED_RECEIPT_KEYS, f"receipt key-set drift: {sorted(set(receipt) ^ EXPECTED_RECEIPT_KEYS)}")
    for key in (
        "schema", "subject_head", "source_parent", "github_repository", "github_workflow",
        "runner_os", "runner_arch", "rust_toolchain", "cargo_version", "checkout_action",
        "rust_toolchain_action", "qualification_lock_sha256",
    ):
        exact_type(receipt[key], str, f"receipt.{key}")
    for key in ("github_run_id", "github_run_attempt"):
        exact_type(receipt[key], int, f"receipt.{key}")
    for key in (
        "qualification_pass", "local_build_hooks_allowed", "local_git_dependencies_allowed",
        "cargo_offline_for_semantic_commands", "os_network_sandbox_claimed",
        "authority_source_full_qualification_inherited",
    ):
        exact_type(receipt[key], bool, f"receipt.{key}")
    exact_type(receipt["admitted_root_trees"], dict, "receipt.admitted_root_trees")

    canonical = json.dumps(receipt, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    require(receipt_raw == canonical, "receipt file is not exact canonical compact JSON bytes")
    require(sha256(receipt_raw) == EXPECTED_RECEIPT_SHA256, "receipt byte SHA mismatch")
    require(receipt["schema"] == "mycelix.authority-delegation-semantic-repair.receipt.v0.2", "wrong source receipt schema")
    require(receipt["qualification_pass"] is True, "source receipt does not claim PASS")
    require(receipt["subject_head"] == EXPECTED_AUTHORITY_PARENT, "receipt subject does not equal authority parent")
    require(receipt["source_parent"] == EXPECTED_HISTORICAL_SOURCE, "receipt source parent drift")
    require(receipt["github_repository"] == "Luminous-Dynamics/mycelix", "receipt repository drift")
    require(receipt["github_run_id"] == 34833153899, "receipt run drift")
    require(receipt["github_run_attempt"] == 1, "receipt run attempt drift")
    require(receipt["github_workflow"] == "Authority Delegation Semantic Repair", "receipt workflow drift")
    require(receipt["rust_toolchain"] == "1.98.1", "receipt Rust drift")
    require(receipt["cargo_version"] == "cargo 1.98.1 (797e8a9bc 2026-08-05)", "receipt Cargo drift")
    require(receipt["checkout_action"] == "actions/checkout@11bd71901bbe5b1630ceea73d27597364c9af683", "receipt checkout action drift")
    require(receipt["rust_toolchain_action"] == "dtolnay/rust-toolchain@6bed0761d98439e5a578e2877258200ad565ba87", "receipt Rust action drift")
    require(receipt["local_build_hooks_allowed"] is False, "receipt unexpectedly permits local build hooks")
    require(receipt["local_git_dependencies_allowed"] is False, "receipt unexpectedly permits git dependencies")
    require(receipt["cargo_offline_for_semantic_commands"] is True, "receipt lacks Cargo-offline semantic claim")
    require(receipt["os_network_sandbox_claimed"] is False, "receipt must not claim OS network sandbox")
    require(receipt["authority_source_full_qualification_inherited"] is False, "receipt must not inherit #77 full qualification")
    require(receipt["admitted_root_trees"] == EXPECTED_ROOT_TREES, "receipt root census drift")

    lock_raw = lock_path.read_bytes()
    require(sha256(lock_raw) == EXPECTED_LOCK_SHA256, "carried Cargo.lock SHA mismatch")
    require(receipt["qualification_lock_sha256"] == EXPECTED_LOCK_SHA256, "receipt lock SHA mismatch")

    parents = git("rev-list", "--parents", "-n1", "HEAD").split()
    require(len(parents) == 3, "convergence head must have exactly two parents")
    require(parents[1] == EXPECTED_AGENT_PARENT, "wrong first parent")
    require(parents[2] == EXPECTED_AUTHORITY_PARENT, "wrong second parent")

    for rel in EXPECTED_ROOTS:
        require((ROOT / rel).is_dir(), f"missing admitted root: {rel}")
        manifest_tree = str(manifest["admitted_root_trees"][rel])
        receipt_tree = str(receipt["admitted_root_trees"][rel])
        head_tree = git("rev-parse", f"HEAD:{rel}")
        parent_tree = git("rev-parse", f"HEAD^2:{rel}")
        require(
            manifest_tree == receipt_tree == head_tree == parent_tree == EXPECTED_ROOT_TREES[rel],
            f"root tree continuity failure: {rel}",
        )

    changed = [line for line in git("diff", "--name-only", EXPECTED_AGENT_PARENT, "HEAD").splitlines() if line]
    unexpected = []
    root_prefixes = tuple(root + "/" for root in EXPECTED_ROOTS)
    for path in changed:
        if path.startswith(root_prefixes):
            continue
        if path in EXACT_QUALIFICATION_PATHS:
            continue
        unexpected.append(path)
    require(not unexpected, f"unexpected selective-materialization paths: {unexpected}")

    core = (ROOT / "crates/mycelix-institutional-core/src/lib.rs").read_text()
    require("pub struct $name(pub String);" in core, "id_type! no longer emits canonical public struct")
    require('id_type!(PrincipalId, "principal_id");' in core, "missing canonical institutional PrincipalId")
    for primitive in ("pub struct AuthorityGrant", "pub struct Intent", "pub struct ActionRequest"):
        require(primitive in core, f"missing institutional primitive: {primitive}")

    for rel in EXPECTED_ROOTS:
        for path in (ROOT / rel).rglob("*"):
            if path.is_file():
                require("AgentPrincipalId" not in path.read_text(errors="ignore"), f"parallel AgentPrincipalId universe in {path}")

    doc = DOC.read_text()
    require("parallel `AgentPrincipalId` is forbidden" in doc, "missing PrincipalId reuse rule")
    require("AGENT authority convergence PASS != upstream PR #77 full qualification" in doc, "missing #77 negative claim")
    require("AGENT authority convergence PASS != AGENT-002 identity PASS" in doc, "missing AGENT-002 negative claim")
    require("AGENT authority convergence PASS != full agent security" in doc, "missing full-security negative claim")

    print("AGENT generic authority convergence v0.1 structural/evidence contract: PASS")


if __name__ == "__main__":
    main()
