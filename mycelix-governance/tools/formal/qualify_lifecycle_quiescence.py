#!/usr/bin/env python3
"""Exact-head bounded qualification for MYC-CONST-003C3 lifecycle quiescence."""

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
SPEC_ROOT = GOV / "specs"
SPEC = SPEC_ROOT / "ConstitutionalLifecycleQuiescence.tla"
PARENT_LIFECYCLE_SPEC = SPEC_ROOT / "ConstitutionalClaimLifecycle.tla"
DOC = GOV / "docs" / "CONSTITUTIONAL_LIFECYCLE_QUIESCENCE_V0_1.md"
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-lifecycle-quiescence-qualification.yml"

SAFETY_CONFIGS = {
    "max6": SPEC_ROOT / "ConstitutionalLifecycleQuiescence.max6.cfg",
    "max8": SPEC_ROOT / "ConstitutionalLifecycleQuiescence.max8.cfg",
    "max10": SPEC_ROOT / "ConstitutionalLifecycleQuiescence.max10.cfg",
}

REACHABILITY_CONFIGS = {
    "awaiting": (
        SPEC_ROOT / "ConstitutionalLifecycleQuiescence.reach-awaiting.cfg",
        "NeverAwaitingExternalEvidenceReached",
    ),
    "active": (
        SPEC_ROOT / "ConstitutionalLifecycleQuiescence.reach-active.cfg",
        "NeverActiveResolutionReached",
    ),
    "resolved": (
        SPEC_ROOT / "ConstitutionalLifecycleQuiescence.reach-resolved.cfg",
        "NeverResolvedAfterWorkReached",
    ),
    "halt": (
        SPEC_ROOT / "ConstitutionalLifecycleQuiescence.reach-halt.cfg",
        "NeverIntegrityHaltReached",
    ),
    "closure": (
        SPEC_ROOT / "ConstitutionalLifecycleQuiescence.reach-closure.cfg",
        "NeverClosureResolutionReached",
    ),
    "effect": (
        SPEC_ROOT / "ConstitutionalLifecycleQuiescence.reach-effect.cfg",
        "NeverEffectResolutionReached",
    ),
}

INPUTS = [
    SPEC,
    PARENT_LIFECYCLE_SPEC,
    *SAFETY_CONFIGS.values(),
    *(path for path, _ in REACHABILITY_CONFIGS.values()),
    DOC,
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
    result: dict[str, str] = {}
    for path in paths:
        if not path.is_file():
            raise RuntimeError(f"required input missing: {path}")
        result[rel(path)] = sha256_file(path)
    return result


def run(cmd: list[str], cwd: Path | None = None) -> tuple[int, str]:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        env=os.environ.copy(),
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
    with tempfile.TemporaryDirectory(prefix=f"mycelix-quiescence-tlc-{label}-") as td:
        work = Path(td)
        spec = work / SPEC.name
        cfg = work / "ConstitutionalLifecycleQuiescence.cfg"
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
    parser.add_argument("--expected-lifecycle-head", required=True)
    parser.add_argument("--expected-closure-head", required=True)
    parser.add_argument("--expected-temporal-head", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--java", default="java")
    args = parser.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-lifecycle-quiescence-qualification.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "expected_semantic_head": args.expected_semantic_head,
        "expected_lifecycle_head": args.expected_lifecycle_head,
        "expected_closure_head": args.expected_closure_head,
        "expected_temporal_head": args.expected_temporal_head,
        "passed": False,
    }

    try:
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        pre = snapshot(INPUTS)
        jar_sha = sha256_file(args.tla_jar)
        if jar_sha != pins["tla2tools"]["sha256"]:
            raise RuntimeError("TLA+ JAR SHA-256 mismatch")

        lineage = [
            ("HEAD", args.expected_head),
            ("HEAD^", args.expected_semantic_head),
            ("HEAD^^", args.expected_lifecycle_head),
            ("HEAD^^^", args.expected_closure_head),
            ("HEAD^^^^", args.expected_temporal_head),
        ]
        observed_lineage: dict[str, str] = {}
        for ref, expected in lineage:
            rc, value = run(["git", "rev-parse", ref], cwd=REPO)
            actual = value.strip()
            if rc != 0 or actual != expected:
                raise RuntimeError(f"lineage mismatch {ref}: expected={expected} actual={actual}")
            observed_lineage[ref] = actual

        java_rc, java_version = run([args.java, "-version"])
        if java_rc != 0:
            raise RuntimeError("java version probe failed")

        receipt.update({
            "git_lineage": observed_lineage,
            "environment": {
                "python": sys.version,
                "platform": platform.platform(),
                "java": java_version.strip(),
                "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                "runner_os": os.environ.get("RUNNER_OS"),
            },
            "tla2tools": {
                "version": pins["tla2tools"]["version"],
                "sha256": jar_sha,
                "workers": 1,
            },
            "preflight_input_sha256": pre,
        })

        spec_text = SPEC.read_text(encoding="utf-8")
        safety_matrix = [
            run_tlc_case(
                label=f"safety-{name}",
                spec_text=spec_text,
                cfg_text=path.read_text(encoding="utf-8"),
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation=None,
            )
            for name, path in SAFETY_CONFIGS.items()
        ]

        distinct_counts = [item["stats"]["distinct_states"] for item in safety_matrix]
        monotonic_counts = all(
            left is not None and right is not None and right >= left
            for left, right in zip(distinct_counts, distinct_counts[1:])
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

        mutant_finality = spec_text.replace(
            "       (\\E c \\in Claims : ResolveFinality(c))\n",
            "       FALSE\n",
            1,
        )
        mutant_closure = spec_text.replace("    \\/ ResolveClosure\n", "", 1)
        mutant_effect = spec_text.replace("    \\/ ApplyEffect\n", "", 1)
        if mutant_finality == spec_text:
            raise RuntimeError("finality resolver mutation did not apply")
        if mutant_closure == spec_text:
            raise RuntimeError("closure resolver mutation did not apply")
        if mutant_effect == spec_text:
            raise RuntimeError("effect resolver mutation did not apply")

        baseline_cfg = SAFETY_CONFIGS["max6"].read_text(encoding="utf-8")
        negative_controls = [
            run_tlc_case(
                label="negative-remove-finality-resolver",
                spec_text=mutant_finality,
                cfg_text=baseline_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="NoPrematureProtocolQuiescence",
            ),
            run_tlc_case(
                label="negative-remove-closure-resolver",
                spec_text=mutant_closure,
                cfg_text=baseline_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="NoPrematureProtocolQuiescence",
            ),
            run_tlc_case(
                label="negative-remove-effect-resolver",
                spec_text=mutant_effect,
                cfg_text=baseline_cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation="NoPrematureProtocolQuiescence",
            ),
        ]

        doc_text = DOC.read_text(encoding="utf-8")
        required_doc_terms = [
            "ExternalInputStep",
            "InternalResolutionStep",
            "ResolutionObligation",
            "ProtocolStall",
            "AwaitingExternalEvidence",
            "MaxSeq = 10",
            "No fairness assumption is hidden in `Spec`",
        ]
        required_spec_terms = [
            "NoPrematureProtocolQuiescence",
            "AwaitingExternalHasInputEnabled",
            "StateClass",
            "ResolvedAfterWorkReached",
        ]
        contract = {
            "missing_doc_terms": [term for term in required_doc_terms if term not in doc_text],
            "missing_spec_terms": [term for term in required_spec_terms if term not in spec_text],
        }
        contract["passed"] = not contract["missing_doc_terms"] and not contract["missing_spec_terms"]

        post = snapshot(INPUTS)
        immutable = pre == post
        receipt.update({
            "safety_matrix": safety_matrix,
            "bound_sensitivity": {
                "bounds": [6, 8, 10],
                "distinct_state_counts": distinct_counts,
                "nondecreasing_distinct_states": monotonic_counts,
            },
            "reachability": reachability,
            "negative_controls": negative_controls,
            "semantic_contract": contract,
            "parent_lifecycle_spec_sha256": sha256_file(PARENT_LIFECYCLE_SPEC),
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            all(item["passed"] for item in safety_matrix)
            and monotonic_counts
            and all(item["passed"] for item in reachability)
            and all(item["passed"] for item in negative_controls)
            and contract["passed"]
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
    (outdir / "artifact-manifest.json").write_text(
        json.dumps(artifact_manifest(outdir), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"passed": receipt["passed"], "receipt": str(receipt_path)}, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
