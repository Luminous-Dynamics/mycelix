#!/usr/bin/env python3
"""Evidence-bearing bounded safety qualification for Mycelix constitutional specs v2.

This runner owns evidence decisions. It reuses only deterministic negative-fixture
builders from qualify.py. Canonical model files are never edited in place.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import traceback
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
SPEC_ROOT = REPO / "mycelix-governance" / "specs"
TOOLS_ROOT = SCRIPT.parent
LEGACY = TOOLS_ROOT / "qualify.py"
PINS = TOOLS_ROOT / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-formal-qualification-v2.yml"

TLA_SPEC = SPEC_ROOT / "ConstitutionalConsumptionV2.tla"
TLA_CONFIGS = {
    "effect": SPEC_ROOT / "ConstitutionalConsumptionV2.effect.cfg",
    "finality": SPEC_ROOT / "ConstitutionalConsumptionV2.finality.cfg",
}
ALLOY_MODELS = {
    "authority": SPEC_ROOT / "alloy" / "ConstitutionalAuthority.als",
    "delegation_budget": SPEC_ROOT / "alloy" / "ConstitutionalDelegationBudget.als",
}

FORMAL_INPUTS = [
    TLA_SPEC,
    *TLA_CONFIGS.values(),
    *ALLOY_MODELS.values(),
    PINS,
    LEGACY,
    SCRIPT,
    WORKFLOW,
]


def load_legacy():
    spec = importlib.util.spec_from_file_location("mycelix_formal_legacy", LEGACY)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load legacy fixture helpers")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


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
    p = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
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


def tlc_cmd(java: str, jar: Path, cfg: Path, spec: Path) -> list[str]:
    return [
        java, "-cp", str(jar), "tlc2.TLC",
        "-workers", "1",
        "-deadlock",
        "-config", cfg.name,
        spec.name,
    ]


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
    with tempfile.TemporaryDirectory(prefix=f"mycelix-tlc-v2-{label}-") as td:
        work = Path(td)
        spec = work / TLA_SPEC.name
        cfg = work / "ConstitutionalConsumptionV2.cfg"
        spec.write_text(spec_text, encoding="utf-8")
        cfg.write_text(cfg_text, encoding="utf-8")
        cmd = tlc_cmd(java, jar, cfg, spec)
        rc, output = run(cmd, cwd=work)

    log = outdir / "tlc" / f"{label}.log"
    log_sha = write_log(log, output)
    stats = parse_tlc_stats(output)
    violations = tlc_violations(output)
    explored = (stats["states_generated"] or 0) > 0 and (stats["distinct_states"] or 0) > 0

    if expected_violation is None:
        passed = (
            rc == 0
            and explored
            and not violations
            and "No error has been found" in output
        )
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


def parse_alloy_commands(source: str) -> dict[str, dict[str, Any]]:
    pattern = re.compile(
        r"(?ms)^\s*(run|check)\s+([A-Za-z_][A-Za-z0-9_]*)\b(.*?)(?=^\s*(?:run|check)\s+|\Z)"
    )
    result: dict[str, dict[str, Any]] = {}
    for m in pattern.finditer(source):
        kind, name, tail = m.groups()
        e = re.search(r"\bexpect\s+([01])\b", tail)
        if e is None:
            raise RuntimeError(f"Alloy command {name} has no explicit expect 0/1")
        result[name] = {"kind": kind, "expects": int(e.group(1))}
    if not result:
        raise RuntimeError("no Alloy commands found")
    return result


def alloy_cmd(
    java: str,
    jar: Path,
    solver: str,
    model: Path,
    output: Path,
    command: str | None = None,
) -> list[str]:
    cmd = [
        java, "-jar", str(jar), "exec",
        "--solver", solver,
        "--type", "none",
        "--output", str(output),
        "--force",
        "--quiet",
    ]
    if command:
        cmd += ["--command", command]
    cmd.append(str(model))
    return cmd


def observed_alloy(receipt: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, info in receipt.get("commands", {}).items():
        sols = info.get("solution") or []
        result[name] = {
            "type": info.get("type"),
            "observed": "SAT" if sols else "UNSAT",
            "solution_count": len(sols),
            "overall": info.get("overall"),
            "scopes": info.get("scopes"),
            "bitwidth": info.get("bitwidth"),
            "receipt_expects": info.get("expects"),
        }
    return result


def run_alloy_case(
    *,
    label: str,
    source: str,
    source_name: str,
    java: str,
    jar: Path,
    solver: str,
    outdir: Path,
    canonical: bool,
    command_name: str | None = None,
) -> dict[str, Any]:
    declared = parse_alloy_commands(source)
    with tempfile.TemporaryDirectory(prefix=f"mycelix-alloy-v2-{label}-") as td:
        work = Path(td)
        model = work / source_name
        output_dir = work / "alloy-output"
        model.write_text(source, encoding="utf-8")
        cmd = alloy_cmd(java, jar, solver, model, output_dir, command_name)
        rc, console = run(cmd, cwd=work)
        raw = output_dir / "receipt.json"
        receipt = json.loads(raw.read_text(encoding="utf-8")) if raw.is_file() else None
        dest = outdir / "alloy" / f"{label}.receipt.json"
        raw_sha = None
        if raw.is_file():
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(raw, dest)
            raw_sha = sha256_file(dest)

    log = outdir / "alloy" / f"{label}.log"
    log_sha = write_log(log, console)

    if receipt is None:
        return {
            "label": label, "command": cmd, "returncode": rc,
            "error": "receipt.json missing",
            "log": rel(log), "log_sha256": log_sha, "passed": False,
        }

    observed = observed_alloy(receipt)
    receipt_solver = str(receipt.get("solver", ""))
    if canonical:
        same_set = set(declared) == set(observed)
        comparisons = {}
        for name, declaration in declared.items():
            obs = observed.get(name)
            actual_sat = bool(obs and obs["observed"] == "SAT")
            expected_sat = declaration["expects"] == 1
            comparisons[name] = {
                "declared": declaration,
                "observed": obs,
                "matches": obs is not None and actual_sat == expected_sat,
            }
        passed = (
            rc == 0
            and same_set
            and all(x["matches"] for x in comparisons.values())
            and receipt_solver.lower() == solver.lower()
        )
    else:
        comparisons = {}
        obs = observed.get(command_name or "")
        passed = (
            obs is not None
            and obs["observed"] == "SAT"
            and receipt_solver.lower() == solver.lower()
        )

    return {
        "label": label,
        "command": cmd,
        "returncode": rc,
        "solver": receipt_solver,
        "declared_commands": declared,
        "observed_commands": observed,
        "comparisons": comparisons,
        "raw_receipt": rel(dest) if raw_sha else None,
        "raw_receipt_sha256": raw_sha,
        "log": rel(log),
        "log_sha256": log_sha,
        "passed": passed,
    }


def artifact_manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    result = {}
    for p in sorted(x for x in outdir.rglob("*") if x.is_file() and x.name != "artifact-manifest-v2.json"):
        result[str(p.relative_to(outdir))] = {"sha256": sha256_file(p), "size": p.stat().st_size}
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tla-jar", type=Path, required=True)
    ap.add_argument("--alloy-jar", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--expected-head", required=True)
    ap.add_argument("--java", default="java")
    args = ap.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.formal-safety-qualification-receipt.v2",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "passed": False,
    }

    try:
        legacy = load_legacy()
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        pre = snapshot(FORMAL_INPUTS)

        actual_tla = sha256_file(args.tla_jar)
        actual_alloy = sha256_file(args.alloy_jar)
        if actual_tla != pins["tla2tools"]["sha256"]:
            raise RuntimeError("TLA+ JAR SHA-256 mismatch")
        if actual_alloy != pins["alloy"]["sha256"]:
            raise RuntimeError("Alloy JAR SHA-256 mismatch")

        git_rc, git_head = run(["git", "rev-parse", "HEAD"], cwd=REPO)
        if git_rc != 0 or git_head.strip() != args.expected_head:
            raise RuntimeError(
                f"raw-head mismatch: expected={args.expected_head} actual={git_head.strip()}"
            )
        java_rc, java_version = run([args.java, "-version"])
        if java_rc != 0:
            raise RuntimeError("java -version failed")

        receipt.update({
            "git_head": git_head.strip(),
            "environment": {
                "python": sys.version,
                "platform": platform.platform(),
                "java_version_output": java_version.strip(),
                "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                "runner_os": os.environ.get("RUNNER_OS"),
            },
            "pins_manifest_sha256": sha256_file(PINS),
            "tools": {
                "tla2tools": {"version_pin": pins["tla2tools"]["version"], "sha256": actual_tla},
                "alloy": {
                    "version_pin": pins["alloy"]["version"],
                    "sha256": actual_alloy,
                    "solver": pins["alloy"]["solver"],
                },
            },
            "preflight_input_sha256": pre,
        })

        spec_text = TLA_SPEC.read_text(encoding="utf-8")
        cfg_texts = {n: p.read_text(encoding="utf-8") for n, p in TLA_CONFIGS.items()}

        tlc_canonical = [
            run_tlc_case(
                label=f"canonical-{name}",
                spec_text=spec_text,
                cfg_text=cfg,
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation=None,
            )
            for name, cfg in cfg_texts.items()
        ]

        tlc_negative = []
        for c in legacy.build_tlc_negative_controls(
            spec_text, cfg_texts["effect"], cfg_texts["finality"]
        ):
            tlc_negative.append(run_tlc_case(
                label=c["label"],
                spec_text=c["spec"],
                cfg_text=c["cfg"],
                java=args.java,
                jar=args.tla_jar.resolve(),
                outdir=outdir,
                expected_violation=c["invariant"],
            ))

        models = {n: p.read_text(encoding="utf-8") for n, p in ALLOY_MODELS.items()}
        solver = pins["alloy"]["solver"]
        alloy_canonical = [
            run_alloy_case(
                label=f"canonical-{name}",
                source=models[name],
                source_name=path.name,
                java=args.java,
                jar=args.alloy_jar.resolve(),
                solver=solver,
                outdir=outdir,
                canonical=True,
            )
            for name, path in ALLOY_MODELS.items()
        ]

        alloy_negative = []
        for c in legacy.build_alloy_negative_controls(models):
            alloy_negative.append(run_alloy_case(
                label=c["label"],
                source=c["model"],
                source_name=c["source_name"],
                java=args.java,
                jar=args.alloy_jar.resolve(),
                solver=solver,
                outdir=outdir,
                canonical=False,
                command_name=c["command"],
            ))

        post = snapshot(FORMAL_INPUTS)
        immutable = pre == post
        receipt["tlc"] = {"canonical": tlc_canonical, "negative_controls": tlc_negative}
        receipt["alloy"] = {"canonical": alloy_canonical, "negative_controls": alloy_negative}
        receipt["postflight_input_sha256"] = post
        receipt["postflight_immutable"] = immutable

        results = tlc_canonical + tlc_negative + alloy_canonical + alloy_negative
        receipt["passed"] = immutable and all(bool(x.get("passed")) for x in results)
        receipt["completed_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    except Exception as exc:
        receipt["fatal_error"] = f"{type(exc).__name__}: {exc}"
        receipt["traceback"] = traceback.format_exc()
        receipt["completed_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        receipt["passed"] = False

    rpath = outdir / "qualification-receipt-v2.json"
    rpath.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (outdir / "artifact-manifest-v2.json").write_text(
        json.dumps(artifact_manifest(outdir), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "receipt": str(rpath),
        "receipt_sha256": sha256_file(rpath),
        "passed": receipt["passed"],
    }, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
