#!/usr/bin/env python3
"""Exact-head bounded TLC reachability qualification for Mycelix constitutional specs.

This runner is intentionally separate from safety/negative-control qualification.
Each case is expected to violate one exact `Never...` invariant, thereby proving
that the named constitutional history is reachable within the recorded finite
bound. Any parser/semantic error, unrelated invariant failure, missing state
search, tool/hash mismatch, head mismatch, or input mutation fails closed.
"""

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
SPEC_ROOT = REPO / "mycelix-governance" / "specs"
TLA_SPEC = SPEC_ROOT / "ConstitutionalConsumptionV2.tla"
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-formal-reachability-qualification.yml"

CASES = {
    "effect-late-revocation-cancel": (
        SPEC_ROOT / "ConstitutionalConsumptionV2.reach-effect-cancel.cfg",
        "NeverEffectLateEarlierRevocationCancellation",
    ),
    "finality-late-revocation-fault": (
        SPEC_ROOT / "ConstitutionalConsumptionV2.reach-finality-fault.cfg",
        "NeverFinalityLateEarlierRevocationFault",
    ),
    "finality-postcommit-effect": (
        SPEC_ROOT / "ConstitutionalConsumptionV2.reach-finality-postcommit-effect.cfg",
        "NeverFinalityPostCommitRevocationEffect",
    ),
    "effect-posteffect-fault": (
        SPEC_ROOT / "ConstitutionalConsumptionV2.reach-effect-posteffect-fault.cfg",
        "NeverEffectPostEffectContradictionFault",
    ),
}

FORMAL_INPUTS = [TLA_SPEC, *(cfg for cfg, _ in CASES.values()), PINS, SCRIPT, WORKFLOW]
PARSER_FAILURE_MARKERS = (
    "Parsing or semantic analysis failed",
    "Semantic errors:",
    "Lexical error",
    "TLC threw an unexpected exception",
)


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


def run(cmd: list[str], cwd: Path | None = None) -> tuple[int, str]:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    return proc.returncode, proc.stdout


def parse_stats(output: str) -> dict[str, int | None]:
    def grab(pattern: str) -> int | None:
        m = re.search(pattern, output, re.IGNORECASE)
        return int(m.group(1).replace(",", "")) if m else None

    return {
        "states_generated": grab(r"([0-9,]+) states generated"),
        "distinct_states": grab(r"([0-9,]+) distinct states found"),
        "depth": grab(r"depth of the complete state graph search is ([0-9,]+)"),
    }


def invariant_violations(output: str) -> list[str]:
    return re.findall(r"Invariant\s+([A-Za-z_][A-Za-z0-9_]*)\s+is violated", output, re.IGNORECASE)


def qualify_case(label: str, cfg: Path, expected: str, java: str, jar: Path, outdir: Path) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-reach-{label}-") as td:
        work = Path(td)
        spec_copy = work / TLA_SPEC.name
        cfg_copy = work / "ConstitutionalConsumptionV2.cfg"
        spec_copy.write_bytes(TLA_SPEC.read_bytes())
        cfg_copy.write_bytes(cfg.read_bytes())
        cmd = [
            java,
            "-cp",
            str(jar),
            "tlc2.TLC",
            "-workers",
            "1",
            "-deadlock",
            "-config",
            cfg_copy.name,
            spec_copy.name,
        ]
        rc, output = run(cmd, cwd=work)

    log_path = outdir / "tlc-reachability" / f"{label}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(output, encoding="utf-8")
    violations = invariant_violations(output)
    stats = parse_stats(output)
    parser_failure = any(marker.lower() in output.lower() for marker in PARSER_FAILURE_MARKERS)
    explored = (stats["states_generated"] or 0) > 0 and (stats["distinct_states"] or 0) > 0
    exact_violation = [v.lower() for v in violations] == [expected.lower()]
    passed = (not parser_failure) and explored and exact_violation

    return {
        "label": label,
        "config": rel(cfg),
        "config_sha256": sha256_file(cfg),
        "expected_reachability_invariant_violation": expected,
        "command": cmd,
        "returncode": rc,
        "parser_or_semantic_failure": parser_failure,
        "observed_invariant_violations": violations,
        "stats": stats,
        "log": rel(log_path),
        "log_sha256": sha256_file(log_path),
        "passed": passed,
    }


def artifact_manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    manifest: dict[str, dict[str, Any]] = {}
    for path in sorted(p for p in outdir.rglob("*") if p.is_file() and p.name != "artifact-manifest.json"):
        manifest[str(path.relative_to(outdir))] = {
            "sha256": sha256_file(path),
            "size": path.stat().st_size,
        }
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--expected-head", required=True)
    parser.add_argument("--java", default="java")
    args = parser.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.formal-reachability-qualification-receipt.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "passed": False,
    }

    try:
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        expected_tool_sha = pins["tla2tools"]["sha256"]
        actual_tool_sha = sha256_file(args.tla_jar)
        if actual_tool_sha != expected_tool_sha:
            raise RuntimeError(f"TLA+ JAR SHA-256 mismatch: expected {expected_tool_sha}, got {actual_tool_sha}")

        git_rc, git_head_out = run(["git", "rev-parse", "HEAD"], cwd=REPO)
        if git_rc != 0:
            raise RuntimeError("could not resolve Git HEAD")
        actual_head = git_head_out.strip()
        expected_head = args.expected_head.strip()
        if actual_head != expected_head:
            raise RuntimeError(f"raw-head mismatch: expected {expected_head}, got {actual_head}")

        preflight = snapshot(FORMAL_INPUTS)
        java_rc, java_version = run([args.java, "-version"])
        if java_rc != 0:
            raise RuntimeError("java -version failed")

        cases = [
            qualify_case(label, cfg, invariant, args.java, args.tla_jar.resolve(), outdir)
            for label, (cfg, invariant) in CASES.items()
        ]

        postflight = snapshot(FORMAL_INPUTS)
        immutable = preflight == postflight
        receipt.update({
            "git_head": actual_head,
            "expected_git_head": expected_head,
            "environment": {
                "python": sys.version,
                "platform": platform.platform(),
                "java_version_output": java_version.strip(),
                "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                "runner_os": os.environ.get("RUNNER_OS"),
                "image_os": os.environ.get("ImageOS"),
                "image_version": os.environ.get("ImageVersion"),
            },
            "tool": {
                "tla2tools_version_pin": pins["tla2tools"]["version"],
                "tla2tools_sha256": actual_tool_sha,
                "workers": 1,
                "deadlock_checking": False,
            },
            "spec": rel(TLA_SPEC),
            "spec_sha256": sha256_file(TLA_SPEC),
            "cases": cases,
            "preflight_input_sha256": preflight,
            "postflight_input_sha256": postflight,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = immutable and all(case["passed"] for case in cases)
        receipt["completed_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    except Exception as exc:
        receipt["fatal_error"] = f"{type(exc).__name__}: {exc}"
        receipt["completed_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        receipt["passed"] = False

    receipt_path = outdir / "reachability-receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (outdir / "artifact-manifest.json").write_text(
        json.dumps(artifact_manifest(outdir), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"receipt": str(receipt_path), "passed": receipt.get("passed", False)}, sort_keys=True))
    return 0 if receipt.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
