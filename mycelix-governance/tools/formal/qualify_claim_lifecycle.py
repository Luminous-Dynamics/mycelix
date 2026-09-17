#!/usr/bin/env python3
"""Exact-head qualification for MYC-CONST-003B2 constitutional claim lifecycle.

Evidence classes:
- exact verifier -> lifecycle semantic -> qualified closure -> qualified temporal lineage;
- Rust all-target tests in a detached worktree;
- Rust mutation controls for temporal snapshot attachment, competitor resolution, and revocation blocking;
- TLC bounded safety for the lifecycle model;
- six named reachability/non-vacuity witnesses;
- TLC mutation controls for competitor resolution, revocation blocking, and fault provenance;
- exact tool hashes and postflight immutability.
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
CRATE = GOV / "crates" / "constitutional-claim-lifecycle"
SPEC_ROOT = GOV / "specs"
WORKSPACE = GOV / "Cargo.toml"
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-claim-lifecycle-qualification.yml"
DOC = GOV / "docs" / "CONSTITUTIONAL_CLAIM_LIFECYCLE_V0_1.md"

TLA_SPEC = SPEC_ROOT / "ConstitutionalClaimLifecycle.tla"
SAFETY_CONFIG = SPEC_ROOT / "ConstitutionalClaimLifecycle.cfg"
REACHABILITY_CONFIGS = {
    "conflict": (
        SPEC_ROOT / "ConstitutionalClaimLifecycle.reach-conflict.cfg",
        "NeverConflictRejectedReached",
    ),
    "blocked": (
        SPEC_ROOT / "ConstitutionalClaimLifecycle.reach-blocked.cfg",
        "NeverRevocationBlockedReached",
    ),
    "closed": (
        SPEC_ROOT / "ConstitutionalClaimLifecycle.reach-closed.cfg",
        "NeverRevokedClosedReached",
    ),
    "late": (
        SPEC_ROOT / "ConstitutionalClaimLifecycle.reach-late.cfg",
        "NeverLatePreRevocationFinalityReached",
    ),
    "fault": (
        SPEC_ROOT / "ConstitutionalClaimLifecycle.reach-fault.cfg",
        "NeverFaultHaltedReached",
    ),
    "history": (
        SPEC_ROOT / "ConstitutionalClaimLifecycle.reach-history.cfg",
        "NeverHistoricalTerminalPreservedAfterFaultReached",
    ),
}

LIFECYCLE_SOURCES = [
    CRATE / "Cargo.toml",
    CRATE / "src" / "lib.rs",
    CRATE / "src" / "types.rs",
    CRATE / "src" / "state.rs",
    CRATE / "src" / "transitions.rs",
    CRATE / "src" / "invariants.rs",
    CRATE / "tests" / "lifecycle.rs",
]

INPUTS = [
    WORKSPACE,
    *LIFECYCLE_SOURCES,
    GOV / "crates" / "constitutional-closure-coverage" / "src" / "lib.rs",
    GOV / "crates" / "constitutional-temporal-provenance" / "src" / "lib.rs",
    GOV / "crates" / "constitutional-consumption" / "src" / "lib.rs",
    DOC,
    TLA_SPEC,
    SAFETY_CONFIG,
    *(p for p, _ in REACHABILITY_CONFIGS.values()),
    PINS,
    SCRIPT,
    WORKFLOW,
]

RUST_NEGATIVE_CONTROLS = {
    "allow-prepopulated-temporal": {
        "file": "state.rs",
        "before": "if !Self::temporal_is_pristine(&temporal) {",
        "after": "if false && !Self::temporal_is_pristine(&temporal) {",
        "expected_test": "constructor_rejects_prepopulated_temporal_state",
    },
    "disable-competitor-resolution": {
        "file": "transitions.rs",
        "before": "&& other.status.is_live()",
        "after": "&& false",
        "expected_test": "finalizing_winner_terminalizes_competitor_without_deleting_evidence",
    },
    "disable-revocation-blocking": {
        "file": "transitions.rs",
        "before": "Some(ClaimLifecycleStatus::PendingExecutable) => true,",
        "after": "Some(ClaimLifecycleStatus::PendingExecutable) => false,",
        "expected_test": "blocked_claim_can_still_finalize_with_late_observed_pre_revocation_proof",
    },
}


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


def run(
    cmd: list[str],
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
) -> tuple[int, str]:
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


def parse_tlc_stats(output: str) -> dict[str, int | None]:
    def get(pattern: str) -> int | None:
        match = re.search(pattern, output)
        return int(match.group(1).replace(",", "")) if match else None

    return {
        "states_generated": get(r"([0-9,]+) states generated"),
        "distinct_states": get(r"([0-9,]+) distinct states found"),
        "depth": get(r"depth of the complete state graph search is ([0-9,]+)"),
    }


def tlc_violations(output: str) -> list[str]:
    return sorted(set(re.findall(r"Invariant ([A-Za-z_][A-Za-z0-9_]*) is violated", output)))


def run_tlc_case(
    *,
    label: str,
    spec_text: str,
    cfg_text: str,
    java: str,
    jar: Path,
    outdir: Path,
    expected_violation: str | None,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-lifecycle-tlc-{label}-") as td:
        work = Path(td)
        spec = work / TLA_SPEC.name
        cfg = work / "ConstitutionalClaimLifecycle.cfg"
        spec.write_text(spec_text, encoding="utf-8")
        cfg.write_text(cfg_text, encoding="utf-8")
        cmd = [
            java,
            "-cp",
            str(jar),
            "tlc2.TLC",
            "-workers",
            "1",
            "-deadlock",
            "-config",
            cfg.name,
            spec.name,
        ]
        rc, output = run(cmd, cwd=work)

    log = outdir / "tlc" / f"{label}.log"
    log_sha = write_log(log, output)
    stats = parse_tlc_stats(output)
    violations = tlc_violations(output)
    explored = (stats["states_generated"] or 0) > 0 and (stats["distinct_states"] or 0) > 0

    if expected_violation is None:
        passed = rc == 0 and explored and not violations and "No error has been found" in output
        expected = "SAFETY_PASS"
    else:
        passed = explored and violations == [expected_violation]
        expected = f"EXACT_VIOLATION:{expected_violation}"

    return {
        "label": label,
        "command": cmd,
        "returncode": rc,
        "expected": expected,
        "observed_invariant_violations": violations,
        "observed_no_error": "No error has been found" in output,
        "stats": stats,
        "log": rel(log),
        "log_sha256": log_sha,
        "passed": passed,
    }


def cargo_test_in_worktree(
    *,
    head: str,
    outdir: Path,
    label: str,
    mutation: dict[str, str] | None,
    cargo: str,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-lifecycle-rust-{label}-") as td:
        root = Path(td)
        worktree = root / "repo"
        rc_add, add_out = run(["git", "worktree", "add", "--detach", str(worktree), head], cwd=REPO)
        if rc_add != 0:
            log = outdir / "rust" / f"{label}.log"
            return {
                "label": label,
                "returncode": rc_add,
                "passed": False,
                "error": "git worktree add failed",
                "log": rel(log),
                "log_sha256": write_log(log, add_out),
            }

        try:
            expected_test = None
            if mutation is not None:
                src = (
                    worktree
                    / "mycelix-governance"
                    / "crates"
                    / "constitutional-claim-lifecycle"
                    / "src"
                    / mutation["file"]
                )
                text = src.read_text(encoding="utf-8")
                mutated = text.replace(mutation["before"], mutation["after"], 1)
                if mutated == text:
                    raise RuntimeError(f"Rust mutation did not apply: {label}")
                src.write_text(mutated, encoding="utf-8")
                expected_test = mutation["expected_test"]

            target = root / "target"
            manifest = (
                worktree
                / "mycelix-governance"
                / "crates"
                / "constitutional-claim-lifecycle"
                / "Cargo.toml"
            )
            cmd = [cargo, "test", "--manifest-path", str(manifest), "--all-targets"]
            rc, output = run(cmd, cwd=worktree, env={"CARGO_TARGET_DIR": str(target)})
            failed_tests = sorted(set(re.findall(r"test ([A-Za-z0-9_:]+) ... FAILED", output)))

            log = outdir / "rust" / f"{label}.log"
            log_sha = write_log(log, output)
            lock = worktree / "mycelix-governance" / "Cargo.lock"
            lock_sha = None
            if lock.is_file():
                lock_sha = sha256_file(lock)
                if mutation is None:
                    dest = outdir / "rust" / "derived-Cargo.lock"
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(lock, dest)

            result: dict[str, Any] = {
                "label": label,
                "command": cmd,
                "returncode": rc,
                "failed_tests": failed_tests,
                "derived_lock_sha256": lock_sha,
                "log": rel(log),
                "log_sha256": log_sha,
            }
            if mutation is None:
                result["expected"] = "ALL_TARGETS_PASS"
                result["passed"] = rc == 0 and "test result: ok" in output
            else:
                result["expected"] = f"MUTATION_CAUGHT:{expected_test}"
                result["passed"] = rc != 0 and expected_test in failed_tests
            return result
        finally:
            run(["git", "worktree", "remove", "--force", str(worktree)], cwd=REPO)


def artifact_manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for path in sorted(x for x in outdir.rglob("*") if x.is_file() and x.name != "artifact-manifest.json"):
        result[str(path.relative_to(outdir))] = {
            "sha256": sha256_file(path),
            "size": path.stat().st_size,
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--expected-head", required=True)
    parser.add_argument("--expected-semantic-head", required=True)
    parser.add_argument("--expected-parent-closure-head", required=True)
    parser.add_argument("--expected-parent-temporal-head", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--java", default="java")
    parser.add_argument("--cargo", default="cargo")
    args = parser.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-claim-lifecycle-qualification.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "expected_semantic_head": args.expected_semantic_head,
        "expected_parent_closure_head": args.expected_parent_closure_head,
        "expected_parent_temporal_head": args.expected_parent_temporal_head,
        "passed": False,
    }

    try:
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        pre = snapshot(INPUTS)

        actual_tla = sha256_file(args.tla_jar)
        if actual_tla != pins["tla2tools"]["sha256"]:
            raise RuntimeError("TLA+ JAR SHA-256 mismatch")

        probes = {}
        for suffix, expected in [
            ("HEAD", args.expected_head),
            ("HEAD^", args.expected_semantic_head),
            ("HEAD^^", args.expected_parent_closure_head),
            ("HEAD^^^", args.expected_parent_temporal_head),
        ]:
            rc, value = run(["git", "rev-parse", suffix], cwd=REPO)
            if rc != 0 or value.strip() != expected:
                raise RuntimeError(
                    f"lineage mismatch {suffix}: expected={expected} actual={value.strip()}"
                )
            probes[suffix] = value.strip()

        java_rc, java_version = run([args.java, "-version"])
        cargo_rc, cargo_version = run([args.cargo, "--version"])
        rustc_rc, rustc_version = run(["rustc", "--version"])
        if java_rc != 0 or cargo_rc != 0 or rustc_rc != 0:
            raise RuntimeError("tool version probe failed")

        receipt.update(
            {
                "git_lineage": probes,
                "environment": {
                    "python": sys.version,
                    "platform": platform.platform(),
                    "java": java_version.strip(),
                    "cargo": cargo_version.strip(),
                    "rustc": rustc_version.strip(),
                    "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                    "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                    "runner_os": os.environ.get("RUNNER_OS"),
                },
                "tla2tools": {
                    "version": pins["tla2tools"]["version"],
                    "sha256": actual_tla,
                },
                "preflight_input_sha256": pre,
            }
        )

        canonical_rust = cargo_test_in_worktree(
            head=args.expected_head,
            outdir=outdir,
            label="canonical",
            mutation=None,
            cargo=args.cargo,
        )
        rust_negative_controls = [
            cargo_test_in_worktree(
                head=args.expected_head,
                outdir=outdir,
                label=label,
                mutation=mutation,
                cargo=args.cargo,
            )
            for label, mutation in RUST_NEGATIVE_CONTROLS.items()
        ]

        spec_text = TLA_SPEC.read_text(encoding="utf-8")
        safety = run_tlc_case(
            label="safety",
            spec_text=spec_text,
            cfg_text=SAFETY_CONFIG.read_text(encoding="utf-8"),
            java=args.java,
            jar=args.tla_jar.resolve(),
            outdir=outdir,
            expected_violation=None,
        )
        reachability = [
            run_tlc_case(
                label=f"reach-{name}",
                spec_text=spec_text,
                cfg_text=path.read_text(encoding="utf-8"),
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation=expected,
            )
            for name, (path, expected) in REACHABILITY_CONFIGS.items()
        ]

        canonical_cfg = SAFETY_CONFIG.read_text(encoding="utf-8")

        no_competitor_resolution = spec_text.replace(
            'ELSE IF status[x] \\in LiveStatuses THEN "Rejected"',
            'ELSE IF status[x] \\in LiveStatuses THEN status[x]',
            1,
        )
        if no_competitor_resolution == spec_text:
            raise RuntimeError("TLA mutation did not apply: competitor resolution")

        no_revocation_block = spec_text.replace(
            'IF status[x] = "Pending" THEN "Blocked" ELSE status[x]',
            'IF status[x] = "Pending" THEN status[x] ELSE status[x]',
            1,
        )
        if no_revocation_block == spec_text:
            raise RuntimeError("TLA mutation did not apply: revocation blocking")

        no_fault_flag = spec_text.replace(
            '/\\ coverage\' = "Fault"\n    /\\ fault\' = TRUE',
            '/\\ coverage\' = "Fault"\n    /\\ fault\' = fault',
            1,
        )
        if no_fault_flag == spec_text:
            raise RuntimeError("TLA mutation did not apply: fault provenance")

        tlc_negative_controls = [
            run_tlc_case(
                label="negative-competitor-resolution",
                spec_text=no_competitor_resolution,
                cfg_text=canonical_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="CompetitorsResolved",
            ),
            run_tlc_case(
                label="negative-revocation-blocking",
                spec_text=no_revocation_block,
                cfg_text=canonical_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="RevocationBlocksPending",
            ),
            run_tlc_case(
                label="negative-halt-without-fault",
                spec_text=no_fault_flag,
                cfg_text=canonical_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="HaltedRequiresFault",
            ),
        ]

        doc_text = DOC.read_text(encoding="utf-8")
        required_doc_terms = [
            "owns",
            "TemporalEvidenceState",
            "transactional",
            "non-pristine",
            "accepted owned finality-evidence ID",
            "caller-supplied temporal snapshots",
        ]
        doc_contract = {
            "required_terms": required_doc_terms,
            "missing_terms": [term for term in required_doc_terms if term not in doc_text],
        }
        doc_contract["passed"] = not doc_contract["missing_terms"]

        post = snapshot(INPUTS)
        immutable = pre == post
        receipt.update(
            {
                "canonical_rust": canonical_rust,
                "rust_negative_controls": rust_negative_controls,
                "tlc_safety": safety,
                "tlc_reachability": reachability,
                "tlc_negative_controls": tlc_negative_controls,
                "semantic_doc_contract": doc_contract,
                "postflight_input_sha256": post,
                "postflight_immutable": immutable,
            }
        )
        receipt["passed"] = (
            canonical_rust["passed"]
            and all(item["passed"] for item in rust_negative_controls)
            and safety["passed"]
            and all(item["passed"] for item in reachability)
            and all(item["passed"] for item in tlc_negative_controls)
            and doc_contract["passed"]
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
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (outdir / "artifact-manifest.json").write_text(
        json.dumps(artifact_manifest(outdir), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"passed": receipt["passed"], "receipt": str(receipt_path)}, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
