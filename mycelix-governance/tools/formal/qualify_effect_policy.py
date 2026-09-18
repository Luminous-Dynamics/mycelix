#!/usr/bin/env python3
"""Exact-head qualification for MYC-CONST-003D1B constitutional effect policy."""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
GOV = REPO / "mycelix-governance"
CRATE = GOV / "crates" / "constitutional-effect-policy"
DOC = GOV / "docs" / "CONSTITUTIONAL_EFFECT_POLICY_V0_1.md"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-effect-policy-qualification.yml"
INPUTS = [
    CRATE / "Cargo.toml",
    CRATE / "src" / "lib.rs",
    CRATE / "tests" / "policy.rs",
    CRATE / "tests" / "hardening.rs",
    DOC,
    SCRIPT,
    WORKFLOW,
]

MUTATIONS: dict[str, dict[str, str]] = {
    "bypass-capability-evidence": {
        "before": "    if has_stronger_than_unproven(profile)\n        && !profile",
        "after": "    if false && has_stronger_than_unproven(profile)\n        && !profile",
        "test": "capability_stronger_than_unproven_requires_evidence",
    },
    "allow-notification-as-durable": {
        "before": "    if is_notification && action.requires_durable_success {",
        "after": "    if false && is_notification && action.requires_durable_success {",
        "test": "best_effort_notification_cannot_masquerade_as_durable_success",
    },
    "allow-durable-effect-opt-out": {
        "before": "    if !is_notification && !action.requires_durable_success {",
        "after": "    if false && !is_notification && !action.requires_durable_success {",
        "test": "durable_effect_cannot_opt_out_of_durable_success_requirement",
    },
    "ignore-provider-effect-contract": {
        "before": "    if !provider_effect_matches(action) {",
        "after": "    if false && !provider_effect_matches(action) {",
        "test": "transfer_bridge_intent_record_cannot_satisfy_value_transfer",
    },
    "allow-retry-with-unstable-identity": {
        "before": "    if !stable_operation_identity {",
        "after": "    if false && !stable_operation_identity {",
        "test": "unstable_operation_identity_disables_automatic_retry",
    },
    "allow-unsafe-irreversible-batch": {
        "before": "            if durable.iter().any(unsafe_irreversible_or_unknown) {\n                return Err(AdmissibilityError::MultiActionRequiresSingleAction);\n            }\n            if durable\n                .iter()\n                .any(|a| matches!(a.retry_policy, RetryPolicy::NeverAutomatic))\n            {\n                return Err(AdmissibilityError::MultiActionRequiresSingleAction);\n            }",
        "after": "            // MUTANT: both independent fail-closed guards removed.",
        "test": "irreversible_nonqueryable_multi_action_batch_is_rejected",
    },
    "allow-saga-without-compensation-authority": {
        "before": "                if !options.compensation_authority_bound {",
        "after": "                if false && !options.compensation_authority_bound {",
        "test": "saga_requires_explicitly_bound_compensation_authority",
    },
    "weaken-provider-atomic-profile-equality": {
        "before": "        planned.action.provider == first.action.provider\n            && matches!(",
        "after": "        planned.action.provider.provider_id == first.action.provider.provider_id\n            && planned.action.provider.version == first.action.provider.version\n            && matches!(",
        "test": "same_provider_and_version_are_not_enough_to_claim_atomic_batch",
    },
    "accept-stale-plan": {
        "before": "    if &expected != plan {",
        "after": "    if false && &expected != plan {",
        "test": "provider_profile_drift_invalidates_committed_plan",
    },
}


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO.resolve()))
    except ValueError:
        return str(path.resolve())


def snap() -> dict[str, str]:
    out = {}
    for path in INPUTS:
        if not path.is_file():
            raise RuntimeError(f"required input missing: {path}")
        out[rel(path)] = sha(path)
    return out


def run(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> tuple[int, str]:
    merged = os.environ.copy()
    if env:
        merged.update(env)
    p = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        env=merged,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    return p.returncode, p.stdout


def log(outdir: Path, name: str, text: str) -> tuple[str, str]:
    path = outdir / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return rel(path), sha(path)


def with_worktree(head: str, label: str):
    td = tempfile.TemporaryDirectory(prefix=f"mycelix-effect-policy-{label}-")
    root = Path(td.name)
    wt = root / "repo"
    rc, output = run(["git", "worktree", "add", "--detach", str(wt), head], cwd=REPO)
    if rc != 0:
        td.cleanup()
        raise RuntimeError(f"worktree add failed for {label}: {output}")
    return td, root, wt


def test_case(head: str, label: str, outdir: Path, cargo: str, mutation: dict[str, str] | None) -> dict[str, Any]:
    td, root, wt = with_worktree(head, label)
    try:
        crate = wt / "mycelix-governance" / "crates" / "constitutional-effect-policy"
        expected = None
        if mutation:
            src = crate / "src" / "lib.rs"
            original = src.read_text(encoding="utf-8")
            changed = original.replace(mutation["before"], mutation["after"], 1)
            if changed == original:
                raise RuntimeError(f"mutation did not apply: {label}")
            src.write_text(changed, encoding="utf-8")
            expected = mutation["test"]
        cmd = [cargo, "test", "--manifest-path", str(crate / "Cargo.toml"), "--all-targets"]
        rc, output = run(cmd, cwd=wt, env={"CARGO_TARGET_DIR": str(root / "target")})
        failed = sorted(set(re.findall(r"test ([A-Za-z0-9_:]+) \.\.\. FAILED", output)))
        lp, digest = log(outdir, f"rust/{label}.log", output)
        if mutation:
            passed = rc != 0 and expected in failed
            expectation = f"MUTATION_CAUGHT:{expected}"
        else:
            passed = rc == 0 and "test result: ok" in output
            expectation = "ALL_TARGETS_PASS"
        return {
            "label": label,
            "command": cmd,
            "returncode": rc,
            "expected": expectation,
            "failed_tests": failed,
            "log": lp,
            "log_sha256": digest,
            "passed": passed,
        }
    finally:
        run(["git", "worktree", "remove", "--force", str(wt)], cwd=REPO)
        td.cleanup()


def tool_checks(head: str, outdir: Path, cargo: str) -> dict[str, Any]:
    td, root, wt = with_worktree(head, "tools")
    try:
        manifest = wt / "mycelix-governance" / "crates" / "constitutional-effect-policy" / "Cargo.toml"
        commands = [
            [cargo, "fmt", "--manifest-path", str(manifest), "--", "--check"],
            [cargo, "clippy", "--manifest-path", str(manifest), "--all-targets", "--", "-D", "warnings"],
        ]
        pieces = []
        ok = True
        for cmd in commands:
            rc, output = run(cmd, cwd=wt, env={"CARGO_TARGET_DIR": str(root / "target")})
            pieces.append(f"$ {' '.join(cmd)}\n{output}")
            ok = ok and rc == 0
        lp, digest = log(outdir, "rust/tool-checks.log", "\n".join(pieces))
        return {"commands": commands, "log": lp, "log_sha256": digest, "passed": ok}
    finally:
        run(["git", "worktree", "remove", "--force", str(wt)], cwd=REPO)
        td.cleanup()


def manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    return {
        str(path.relative_to(outdir)): {"sha256": sha(path), "size": path.stat().st_size}
        for path in sorted(outdir.rglob("*"))
        if path.is_file() and path.name != "artifact-manifest.json"
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--expected-head", required=True)
    ap.add_argument("--expected-subject-head", required=True)
    ap.add_argument("--expected-main-parent", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cargo", default="cargo")
    args = ap.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-effect-policy-qualification.v2",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "expected_subject_head": args.expected_subject_head,
        "expected_main_parent": args.expected_main_parent,
        "passed": False,
    }

    try:
        pre = snap()
        lineage = [("HEAD", args.expected_head), ("HEAD^", args.expected_subject_head), ("HEAD^^", args.expected_main_parent)]
        observed = {}
        for ref, expected in lineage:
            rc, output = run(["git", "rev-parse", ref], cwd=REPO)
            actual = output.strip()
            if rc != 0 or actual != expected:
                raise RuntimeError(f"lineage mismatch {ref}: expected={expected} actual={actual}")
            observed[ref] = actual

        crc, cargo_version = run([args.cargo, "--version"])
        rrc, rustc_version = run(["rustc", "--version"])
        if crc != 0 or rrc != 0:
            raise RuntimeError("Rust version probe failed")

        canonical = test_case(args.expected_subject_head, "canonical", outdir, args.cargo, None)
        tools = tool_checks(args.expected_subject_head, outdir, args.cargo)
        negatives = [
            test_case(args.expected_subject_head, label, outdir, args.cargo, mutation)
            for label, mutation in MUTATIONS.items()
        ]

        doc = DOC.read_text(encoding="utf-8")
        required = [
            "Provider capabilities are evidence-bearing inputs",
            "Every non-notification effect must require durable constitutional success",
            "Provider effect semantics must satisfy the action's required effect",
            "RecordsIntentOrAuditOnly",
            "PartialCompletionSemantics::Required",
            "SagaWithExplicitCompensation",
            "no physical exactly-once claim",
        ]
        contract = {"missing_terms": [term for term in required if term not in doc]}
        contract["passed"] = not contract["missing_terms"]

        post = snap()
        immutable = pre == post
        receipt.update({
            "git_lineage": observed,
            "environment": {
                "python": sys.version,
                "platform": platform.platform(),
                "cargo": cargo_version.strip(),
                "rustc": rustc_version.strip(),
                "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                "runner_os": os.environ.get("RUNNER_OS"),
            },
            "preflight_input_sha256": pre,
            "canonical_rust": canonical,
            "tool_checks": tools,
            "negative_controls": negatives,
            "semantic_contract": contract,
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            canonical["passed"]
            and tools["passed"]
            and all(case["passed"] for case in negatives)
            and contract["passed"]
            and immutable
        )
    except Exception as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        try:
            post = snap()
            receipt["postflight_input_sha256"] = post
            receipt["postflight_immutable"] = receipt.get("preflight_input_sha256") == post
        except Exception as post_exc:
            receipt["postflight_error"] = f"{type(post_exc).__name__}: {post_exc}"

    receipt["finished_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    receipt_path = outdir / "qualification-receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (outdir / "artifact-manifest.json").write_text(json.dumps(manifest(outdir), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": receipt["passed"], "receipt": str(receipt_path)}, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
