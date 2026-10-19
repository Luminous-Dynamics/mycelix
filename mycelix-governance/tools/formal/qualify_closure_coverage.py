#!/usr/bin/env python3
"""Exact-head qualification for MYC-CONST-003B3A closure coverage.

Evidence classes:
- exact Git lineage and input hashes;
- Rust all-target tests in a detached temporary worktree;
- mutation control: suppress temporal-fault dominance -> exact fault regression must fail;
- mutation control: change closure threshold >= R-1 to > R-1 -> exact boundary/truth-table regression must fail;
- postflight immutability of the checked-out subject.

The qualifier deliberately does not re-prove parent TLA+ semantics. It records the
parent semantic SHA and binds the Rust<->TLA+ crosswalk; execution against main
should occur only after the parent 003B3 exact-head qualifier has passed.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
GOV = REPO / "mycelix-governance"
CRATE = GOV / "crates" / "constitutional-closure-coverage"
WORKSPACE = GOV / "Cargo.toml"
SRC = CRATE / "src" / "lib.rs"
TESTS = CRATE / "tests" / "coverage.rs"
SEMANTIC_DOC = GOV / "docs" / "CONSTITUTIONAL_CLOSURE_COVERAGE_V0_1.md"
CROSSWALK = GOV / "docs" / "CONSTITUTIONAL_CLOSURE_COVERAGE_REFINEMENT_V0_1.md"
TEMPORAL_SRC = GOV / "crates" / "constitutional-temporal-provenance" / "src" / "lib.rs"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-closure-coverage-qualification.yml"

INPUTS = [
    WORKSPACE,
    CRATE / "Cargo.toml",
    SRC,
    TESTS,
    SEMANTIC_DOC,
    CROSSWALK,
    TEMPORAL_SRC,
    SCRIPT,
    WORKFLOW,
]

FAULT_TEST = "contradiction_fault_revokes_closure_authority_without_erasing_history"
BOUNDARY_TEST = "healthy_domain_truth_table_is_exactly_watermark_reaches_r_minus_one"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO.resolve()))
    except ValueError:
        return str(path.resolve())


def snapshot(paths: list[Path]) -> dict[str, str]:
    out: dict[str, str] = {}
    for path in paths:
        if not path.is_file():
            raise RuntimeError(f"required input missing: {path}")
        out[rel(path)] = sha256_file(path)
    return out


def run(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> tuple[int, str]:
    merged = os.environ.copy()
    if env:
        merged.update(env)
    proc = subprocess.run(
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
    return proc.returncode, proc.stdout


def write_log(path: Path, text: str) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return sha256_file(path)


def cargo_test_in_worktree(
    *,
    head: str,
    outdir: Path,
    label: str,
    mutate: str | None = None,
    cargo: str = "cargo",
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-closure-coverage-{label}-") as td:
        root = Path(td)
        worktree = root / "repo"
        rc_add, add_out = run(["git", "worktree", "add", "--detach", str(worktree), head], cwd=REPO)
        if rc_add != 0:
            log_path = outdir / "rust" / f"{label}.log"
            return {
                "label": label,
                "returncode": rc_add,
                "passed": False,
                "error": "git worktree add failed",
                "log": rel(log_path),
                "log_sha256": write_log(log_path, add_out),
            }

        try:
            if mutate:
                src = worktree / "mycelix-governance" / "crates" / "constitutional-closure-coverage" / "src" / "lib.rs"
                text = src.read_text(encoding="utf-8")
                if mutate == "disable_fault_dominance":
                    before = "if let Some(fault) = state.integrity_fault.clone() {"
                    after = "if let Some(fault) = state.integrity_fault.clone().filter(|_| false) {"
                    text2 = text.replace(before, after, 1)
                elif mutate == "off_by_one_threshold":
                    before = "closure.closed_through_effective_seq >= required_closed_through_effective_seq"
                    after = "closure.closed_through_effective_seq > required_closed_through_effective_seq"
                    text2 = text.replace(before, after, 1)
                else:
                    raise RuntimeError(f"unknown mutation: {mutate}")
                if text2 == text:
                    raise RuntimeError(f"mutation did not apply: {mutate}")
                src.write_text(text2, encoding="utf-8")

            target = root / "target"
            manifest = worktree / "mycelix-governance" / "crates" / "constitutional-closure-coverage" / "Cargo.toml"
            cmd = [cargo, "test", "--manifest-path", str(manifest), "--all-targets"]
            rc, output = run(cmd, cwd=worktree, env={"CARGO_TARGET_DIR": str(target)})

            log_path = outdir / "rust" / f"{label}.log"
            log_sha = write_log(log_path, output)
            lock_path = worktree / "mycelix-governance" / "Cargo.lock"
            lock_sha = sha256_file(lock_path) if lock_path.is_file() else None

            failed_tests = sorted(set(re.findall(r"test ([A-Za-z0-9_:]+) \.\.\. FAILED", output)))
            result: dict[str, Any] = {
                "label": label,
                "command": cmd,
                "returncode": rc,
                "failed_tests": failed_tests,
                "derived_lock_sha256": lock_sha,
                "log": rel(log_path),
                "log_sha256": log_sha,
            }

            if mutate is None:
                result["expected"] = "ALL_TARGETS_PASS"
                result["passed"] = rc == 0 and "test result: ok" in output
            elif mutate == "disable_fault_dominance":
                result["expected"] = f"MUTATION_CAUGHT:{FAULT_TEST}"
                result["passed"] = rc != 0 and FAULT_TEST in failed_tests
            elif mutate == "off_by_one_threshold":
                result["expected"] = f"MUTATION_CAUGHT:{BOUNDARY_TEST}"
                result["passed"] = rc != 0 and BOUNDARY_TEST in failed_tests
            return result
        finally:
            run(["git", "worktree", "remove", "--force", str(worktree)], cwd=REPO)


def artifact_manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for p in sorted(x for x in outdir.rglob("*") if x.is_file() and x.name != "artifact-manifest.json"):
        result[str(p.relative_to(outdir))] = {"sha256": sha256_file(p), "size": p.stat().st_size}
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--expected-head", required=True)
    ap.add_argument("--expected-semantic-head", required=True)
    ap.add_argument("--expected-parent-temporal-head", required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cargo", default="cargo")
    args = ap.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-closure-coverage-qualification.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "expected_semantic_head": args.expected_semantic_head,
        "expected_parent_temporal_head": args.expected_parent_temporal_head,
        "passed": False,
    }

    try:
        pre = snapshot(INPUTS)
        rc_head, actual_head = run(["git", "rev-parse", "HEAD"], cwd=REPO)
        rc_parent, semantic_head = run(["git", "rev-parse", "HEAD^"], cwd=REPO)
        rc_grand, temporal_head = run(["git", "rev-parse", "HEAD^^"], cwd=REPO)
        if rc_head or rc_parent or rc_grand:
            raise RuntimeError("git lineage probe failed")
        actual_head = actual_head.strip()
        semantic_head = semantic_head.strip()
        temporal_head = temporal_head.strip()
        if actual_head != args.expected_head:
            raise RuntimeError(f"raw-head mismatch: {actual_head}")
        if semantic_head != args.expected_semantic_head:
            raise RuntimeError(f"semantic-parent mismatch: {semantic_head}")
        if temporal_head != args.expected_parent_temporal_head:
            raise RuntimeError(f"temporal-parent mismatch: {temporal_head}")

        cargo_rc, cargo_version = run([args.cargo, "--version"])
        rustc_rc, rustc_version = run(["rustc", "--version"])
        if cargo_rc != 0 or rustc_rc != 0:
            raise RuntimeError("Rust toolchain version probe failed")

        receipt.update({
            "git_head": actual_head,
            "semantic_head": semantic_head,
            "parent_temporal_head": temporal_head,
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
        })

        canonical = cargo_test_in_worktree(
            head=args.expected_head,
            outdir=outdir,
            label="canonical",
            cargo=args.cargo,
        )
        negative_fault = cargo_test_in_worktree(
            head=args.expected_head,
            outdir=outdir,
            label="negative-disable-fault-dominance",
            mutate="disable_fault_dominance",
            cargo=args.cargo,
        )
        negative_boundary = cargo_test_in_worktree(
            head=args.expected_head,
            outdir=outdir,
            label="negative-off-by-one-threshold",
            mutate="off_by_one_threshold",
            cargo=args.cargo,
        )

        crosswalk = CROSSWALK.read_text(encoding="utf-8")
        required_crosswalk_terms = [
            "fault = TRUE",
            "IntegrityFault",
            "closureThrough >= R - 1",
            "SharedComparable",
            "Independent",
            "quotient-by-effective-sequence",
        ]
        crosswalk_contract = {
            "required_terms": required_crosswalk_terms,
            "missing_terms": [term for term in required_crosswalk_terms if term not in crosswalk],
        }
        crosswalk_contract["passed"] = not crosswalk_contract["missing_terms"]

        post = snapshot(INPUTS)
        immutable = pre == post
        receipt.update({
            "canonical_rust": canonical,
            "negative_controls": [negative_fault, negative_boundary],
            "crosswalk_contract": crosswalk_contract,
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            canonical["passed"]
            and negative_fault["passed"]
            and negative_boundary["passed"]
            and crosswalk_contract["passed"]
            and immutable
        )
    except Exception as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        try:
            post = snapshot(INPUTS)
            receipt["postflight_input_sha256"] = post
            receipt["postflight_immutable"] = receipt.get("preflight_input_sha256") == post
        except Exception as post_exc:
            receipt["postflight_error"] = f"{type(post_exc).__name__}: {post_exc}"

    receipt["finished_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    receipt_path = outdir / "qualification-receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = artifact_manifest(outdir)
    (outdir / "artifact-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"passed": receipt["passed"], "receipt": str(receipt_path)}, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
