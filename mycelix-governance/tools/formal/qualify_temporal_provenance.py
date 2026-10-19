#!/usr/bin/env python3
"""Exact-head qualification for constitutional temporal provenance / closure v0.1."""

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
SPEC_ROOT = REPO / "mycelix-governance" / "specs"
CRATE_ROOT = REPO / "mycelix-governance" / "crates" / "constitutional-temporal-provenance"
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-temporal-provenance-qualification.yml"
WORKSPACE = REPO / "mycelix-governance" / "Cargo.toml"

TLA_SPEC = SPEC_ROOT / "ConstitutionalEvidenceClosure.tla"
SAFETY_CONFIGS = {
    "witnessed": SPEC_ROOT / "ConstitutionalEvidenceClosure.witnessed.cfg",
    "detection": SPEC_ROOT / "ConstitutionalEvidenceClosure.detection.cfg",
    "strong": SPEC_ROOT / "ConstitutionalEvidenceClosure.strong.cfg",
}
REACHABILITY_CONFIGS = {
    "late_pre_revocation": (
        SPEC_ROOT / "ConstitutionalEvidenceClosure.reach-late-pre-revocation.cfg",
        "NeverLatePreRevocationFinalityAccepted",
    ),
    "post_closure_fault": (
        SPEC_ROOT / "ConstitutionalEvidenceClosure.reach-post-closure-fault.cfg",
        "NeverPostClosureContradictionFault",
    ),
    "closure": (
        SPEC_ROOT / "ConstitutionalEvidenceClosure.reach-closure.cfg",
        "NeverClosureReached",
    ),
}

INPUTS = [
    WORKSPACE,
    CRATE_ROOT / "Cargo.toml",
    CRATE_ROOT / "src" / "lib.rs",
    CRATE_ROOT / "tests" / "temporal.rs",
    TLA_SPEC,
    *SAFETY_CONFIGS.values(),
    *(p for p, _ in REACHABILITY_CONFIGS.values()),
    PINS,
    SCRIPT,
    WORKFLOW,
]


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


def write_log(path: Path, text: str) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return sha256_file(path)


def parse_tlc_stats(output: str) -> dict[str, int | None]:
    def get(pattern: str) -> int | None:
        m = re.search(pattern, output)
        return int(m.group(1).replace(",", "")) if m else None
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
    with tempfile.TemporaryDirectory(prefix=f"mycelix-closure-tlc-{label}-") as td:
        work = Path(td)
        spec = work / TLA_SPEC.name
        cfg = work / "ConstitutionalEvidenceClosure.cfg"
        spec.write_text(spec_text, encoding="utf-8")
        cfg.write_text(cfg_text, encoding="utf-8")
        cmd = [
            java, "-cp", str(jar), "tlc2.TLC",
            "-workers", "1", "-deadlock",
            "-config", cfg.name, spec.name,
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


def run_rust_tests(cargo: str, expected_head: str, outdir: Path) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="mycelix-closure-rust-") as td:
        worktree = Path(td) / "repo"
        rc_add, add_out = run(["git", "worktree", "add", "--detach", str(worktree), expected_head], cwd=REPO)
        if rc_add != 0:
            log = add_out
            return {"command": ["git", "worktree", "add"], "returncode": rc_add, "passed": False,
                    "log_sha256": write_log(outdir / "rust" / "cargo-test.log", log)}
        try:
            target = Path(td) / "target"
            manifest = worktree / "mycelix-governance" / "crates" / "constitutional-temporal-provenance" / "Cargo.toml"
            cmd = [cargo, "test", "--manifest-path", str(manifest), "--all-targets"]
            rc, output = run(cmd, cwd=worktree, env={"CARGO_TARGET_DIR": str(target)})
            lock = worktree / "mycelix-governance" / "Cargo.lock"
            lock_sha = None
            if lock.is_file():
                dest = outdir / "rust" / "derived-Cargo.lock"
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(lock, dest)
                lock_sha = sha256_file(dest)
            log_sha = write_log(outdir / "rust" / "cargo-test.log", output)
            return {
                "command": cmd,
                "returncode": rc,
                "derived_lock_sha256": lock_sha,
                "log": rel(outdir / "rust" / "cargo-test.log"),
                "log_sha256": log_sha,
                "passed": rc == 0 and "test result: ok" in output,
            }
        finally:
            run(["git", "worktree", "remove", "--force", str(worktree)], cwd=REPO)


def artifact_manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for p in sorted(x for x in outdir.rglob("*") if x.is_file() and x.name != "artifact-manifest.json"):
        result[str(p.relative_to(outdir))] = {"sha256": sha256_file(p), "size": p.stat().st_size}
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tla-jar", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--expected-head", required=True)
    ap.add_argument("--java", default="java")
    ap.add_argument("--cargo", default="cargo")
    args = ap.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-temporal-provenance-qualification.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "passed": False,
    }

    try:
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        pre = snapshot(INPUTS)
        actual_tla = sha256_file(args.tla_jar)
        if actual_tla != pins["tla2tools"]["sha256"]:
            raise RuntimeError("TLA+ JAR SHA-256 mismatch")

        git_rc, git_head = run(["git", "rev-parse", "HEAD"], cwd=REPO)
        if git_rc != 0 or git_head.strip() != args.expected_head:
            raise RuntimeError(f"raw-head mismatch: expected={args.expected_head} actual={git_head.strip()}")

        java_rc, java_version = run([args.java, "-version"])
        cargo_rc, cargo_version = run([args.cargo, "--version"])
        rustc_rc, rustc_version = run(["rustc", "--version"])
        if java_rc != 0 or cargo_rc != 0 or rustc_rc != 0:
            raise RuntimeError("tool version probe failed")

        receipt.update({
            "git_head": git_head.strip(),
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
            "tla2tools": {"version": pins["tla2tools"]["version"], "sha256": actual_tla},
            "preflight_input_sha256": pre,
        })

        rust = run_rust_tests(args.cargo, args.expected_head, outdir)

        spec_text = TLA_SPEC.read_text(encoding="utf-8")
        safety = [
            run_tlc_case(
                label=f"safety-{name}", spec_text=spec_text,
                cfg_text=path.read_text(encoding="utf-8"),
                java=args.java, jar=args.tla_jar.resolve(), outdir=outdir,
                expected_violation=None,
            )
            for name, path in SAFETY_CONFIGS.items()
        ]
        reachability = [
            run_tlc_case(
                label=f"reach-{name}", spec_text=spec_text,
                cfg_text=path.read_text(encoding="utf-8"),
                java=args.java, jar=args.tla_jar.resolve(), outdir=outdir,
                expected_violation=expected,
            )
            for name, (path, expected) in REACHABILITY_CONFIGS.items()
        ]

        detection_cfg = SAFETY_CONFIGS["detection"].read_text(encoding="utf-8")
        witnessed_cfg = SAFETY_CONFIGS["witnessed"].read_text(encoding="utf-8")

        fail_open_detection = spec_text.replace(
            'ClosureAllowed == Profile # "DetectionOnly"',
            'ClosureAllowed == TRUE',
            1,
        )
        if fail_open_detection == spec_text:
            raise RuntimeError("negative control mutation failed: ClosureAllowed")

        no_fault_on_contradiction = spec_text.replace(
            "/\\ fault' = TRUE",
            "/\\ fault' = fault",
            1,
        )
        if no_fault_on_contradiction == spec_text:
            raise RuntimeError("negative control mutation failed: contradiction fault")

        negative_controls = [
            run_tlc_case(
                label="negative-detection-closure",
                spec_text=fail_open_detection,
                cfg_text=detection_cfg,
                java=args.java, jar=args.tla_jar.resolve(), outdir=outdir,
                expected_violation="DetectionOnlyHasNoClosure",
            ),
            run_tlc_case(
                label="negative-closure-without-fault",
                spec_text=no_fault_on_contradiction,
                cfg_text=witnessed_cfg,
                java=args.java, jar=args.tla_jar.resolve(), outdir=outdir,
                expected_violation="FaultHasClosureContradiction",
            ),
        ]

        post = snapshot(INPUTS)
        immutable = pre == post
        receipt.update({
            "rust": rust,
            "tlc_safety": safety,
            "tlc_reachability": reachability,
            "tlc_negative_controls": negative_controls,
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            rust["passed"]
            and all(x["passed"] for x in safety)
            and all(x["passed"] for x in reachability)
            and all(x["passed"] for x in negative_controls)
            and immutable
        )
    except Exception as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        try:
            receipt["postflight_input_sha256"] = snapshot(INPUTS)
            receipt["postflight_immutable"] = receipt.get("preflight_input_sha256") == receipt["postflight_input_sha256"]
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
