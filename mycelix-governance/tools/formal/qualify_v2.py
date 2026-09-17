#!/usr/bin/env python3
"""Evidence-bearing exact-head bounded formal qualification for Mycelix.

This v2 runner separates bounded safety from deadlock checking, treats source
`expect 0/1` declarations as the Alloy oracle, retains raw receipts/logs, and
fails closed on head/tool/input provenance drift. Canonical model files are
never mutated; negative controls operate only on temporary copies.
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
import traceback
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
SPEC_ROOT = REPO / "mycelix-governance" / "specs"
TLA_SPEC = SPEC_ROOT / "ConstitutionalConsumptionV2.tla"
TLA_CONFIGS = {
    "effect": SPEC_ROOT / "ConstitutionalConsumptionV2.effect.cfg",
    "finality": SPEC_ROOT / "ConstitutionalConsumptionV2.finality.cfg",
}
ALLOY_MODELS = {
    "authority": SPEC_ROOT / "alloy" / "ConstitutionalAuthority.als",
    "delegation_budget": SPEC_ROOT / "alloy" / "ConstitutionalDelegationBudget.als",
}
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-formal-qualification.yml"
FORMAL_INPUTS = [TLA_SPEC, *TLA_CONFIGS.values(), *ALLOY_MODELS.values(), PINS, SCRIPT, WORKFLOW]
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


def write_log(path: Path, text: str) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return sha256_file(path)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one mutation target, found {count}")
    return text.replace(old, new, 1)


def replace_in_section(text: str, start: str, end: str, old: str, new: str, label: str) -> str:
    start_i = text.find(start)
    if start_i < 0:
        raise RuntimeError(f"{label}: start marker not found: {start!r}")
    end_i = text.find(end, start_i + len(start))
    if end_i < 0:
        raise RuntimeError(f"{label}: end marker not found: {end!r}")
    section = text[start_i:end_i]
    count = section.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one target inside section, found {count}")
    section = section.replace(old, new, 1)
    return text[:start_i] + section + text[end_i:]


def remove_fact(text: str, fact_name: str, label: str) -> str:
    pattern = re.compile(rf"\nfact\s+{re.escape(fact_name)}\s*\{{.*?\n\}}\n", re.S)
    new, count = pattern.subn("\n", text, count=1)
    if count != 1:
        raise RuntimeError(f"{label}: fact {fact_name} was not uniquely removable")
    return new


def parse_tlc_stats(output: str) -> dict[str, int | None]:
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


def tlc_command(java: str, tla_jar: Path, cfg: Path, spec: Path) -> list[str]:
    # The model has an intentional finite event-clock bound. Deadlock at MaxSeq
    # is not part of the safety claim and is qualified separately if desired.
    return [
        java,
        "-cp",
        str(tla_jar),
        "tlc2.TLC",
        "-workers",
        "1",
        "-deadlock",
        "-config",
        cfg.name,
        spec.name,
    ]


def run_tlc_case(
    *,
    label: str,
    spec_text: str,
    cfg_text: str,
    java: str,
    tla_jar: Path,
    evidence_dir: Path,
    expect_success: bool,
    expected_invariant: str | None = None,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-tlc-{label}-") as td:
        work = Path(td)
        spec = work / TLA_SPEC.name
        cfg = work / "ConstitutionalConsumptionV2.cfg"
        spec.write_text(spec_text, encoding="utf-8")
        cfg.write_text(cfg_text, encoding="utf-8")
        cmd = tlc_command(java, tla_jar, cfg, spec)
        rc, output = run(cmd, cwd=work)

    log_path = evidence_dir / "tlc" / f"{label}.log"
    log_sha = write_log(log_path, output)
    stats = parse_tlc_stats(output)
    explored = (stats["states_generated"] or 0) > 0 and (stats["distinct_states"] or 0) > 0
    parser_failure = any(marker.lower() in output.lower() for marker in PARSER_FAILURE_MARKERS)
    violations = invariant_violations(output)
    no_error = "No error has been found" in output

    if expect_success:
        passed = rc == 0 and no_error and explored and not parser_failure and not violations
    else:
        exact = expected_invariant is not None and [v.lower() for v in violations] == [expected_invariant.lower()]
        passed = explored and not parser_failure and exact

    return {
        "label": label,
        "command": cmd,
        "returncode": rc,
        "expected": "PASS" if expect_success else f"VIOLATE:{expected_invariant}",
        "deadlock_checking": False,
        "parser_or_semantic_failure": parser_failure,
        "observed_no_error": no_error,
        "observed_invariant_violations": violations,
        "stats": stats,
        "log": rel(log_path),
        "log_sha256": log_sha,
        "passed": passed,
    }


def parse_alloy_commands(source: str) -> dict[str, dict[str, Any]]:
    pattern = re.compile(r"(?ms)^\s*(run|check)\s+([A-Za-z_][A-Za-z0-9_]*)\b(.*?)(?=^\s*(?:run|check)\s+|\Z)")
    commands: dict[str, dict[str, Any]] = {}
    for match in pattern.finditer(source):
        kind, name, tail = match.groups()
        exp = re.search(r"\bexpect\s+([01])\b", tail)
        if exp is None:
            raise RuntimeError(f"Alloy command {name} has no explicit expect 0/1")
        commands[name] = {"kind": kind, "expects": int(exp.group(1))}
    if not commands:
        raise RuntimeError("no Alloy run/check commands found")
    return commands


def alloy_command(java: str, alloy_jar: Path, solver: str, model: Path, outdir: Path, command_name: str | None) -> list[str]:
    cmd = [
        java,
        "-jar",
        str(alloy_jar),
        "exec",
        "--solver",
        solver,
        "--type",
        "none",
        "--output",
        str(outdir),
        "--force",
        "--quiet",
    ]
    if command_name:
        cmd += ["--command", command_name]
    cmd.append(str(model))
    return cmd


def classify_alloy_receipt(receipt: dict[str, Any], declared: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for name, info in receipt.get("commands", {}).items():
        solutions = info.get("solution") or []
        expected = declared.get(name, {}).get("expects")
        observed_sat = bool(solutions)
        out[name] = {
            "type": info.get("type"),
            "source_declared_expects": expected,
            "receipt_expects": info.get("expects"),
            "overall_scope": info.get("overall"),
            "scopes": info.get("scopes"),
            "bitwidth": info.get("bitwidth"),
            "observed": "SAT" if observed_sat else "UNSAT",
            "solution_count": len(solutions),
            "passed": expected in (0, 1) and observed_sat == (expected == 1),
        }
    return out


def run_alloy_case(
    *,
    label: str,
    source_text: str,
    source_name: str,
    java: str,
    alloy_jar: Path,
    solver: str,
    evidence_dir: Path,
    canonical: bool,
    command_name: str | None = None,
    negative_expect_sat: bool = False,
) -> dict[str, Any]:
    declared = parse_alloy_commands(source_text)
    with tempfile.TemporaryDirectory(prefix=f"mycelix-alloy-{label}-") as td:
        work = Path(td)
        model = work / source_name
        outdir = work / "alloy-output"
        model.write_text(source_text, encoding="utf-8")
        cmd = alloy_command(java, alloy_jar, solver, model, outdir, command_name)
        rc, output = run(cmd, cwd=work)
        receipt_path = outdir / "receipt.json"
        receipt = json.loads(receipt_path.read_text(encoding="utf-8")) if receipt_path.is_file() else None

        raw_dest = evidence_dir / "alloy" / f"{label}.receipt.json"
        raw_sha = None
        if receipt_path.is_file():
            raw_dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(receipt_path, raw_dest)
            raw_sha = sha256_file(raw_dest)

    log_path = evidence_dir / "alloy" / f"{label}.log"
    log_sha = write_log(log_path, output)

    if receipt is None:
        return {
            "label": label,
            "command": cmd,
            "returncode": rc,
            "error": "receipt.json missing",
            "log": rel(log_path),
            "log_sha256": log_sha,
            "passed": False,
        }

    observed = classify_alloy_receipt(receipt, declared)
    receipt_solver = str(receipt.get("solver", ""))
    solver_match = receipt_solver.lower() == solver.lower()

    if canonical:
        names_match = set(observed) == set(declared)
        outcome_match = names_match and all(observed[name]["passed"] for name in declared)
        passed = rc == 0 and solver_match and outcome_match
    else:
        if command_name is None or command_name not in observed:
            passed = False
        else:
            actual_sat = observed[command_name]["observed"] == "SAT"
            passed = solver_match and actual_sat == negative_expect_sat

    return {
        "label": label,
        "command": cmd,
        "returncode": rc,
        "solver": receipt_solver,
        "declared_commands": declared,
        "observed_commands": observed,
        "raw_receipt": rel(raw_dest) if raw_sha else None,
        "raw_receipt_sha256": raw_sha,
        "log": rel(log_path),
        "log_sha256": log_sha,
        "passed": passed,
    }


def build_tlc_negative_controls(spec: str, effect_cfg: str, finality_cfg: str) -> list[dict[str, str]]:
    controls: list[dict[str, str]] = []
    controls.append({
        "label": "negative-double-finalization",
        "spec": replace_in_section(spec, "Finalize(c) ==", "ObserveRevocation(r) ==", "    /\\ CompetingFinalizationAbsent(c)\n", "", "double-finalization"),
        "cfg": effect_cfg,
        "invariant": "AtMostOneFinalizedPerUse",
    })
    controls.append({
        "label": "negative-effect-without-finality",
        "spec": replace_in_section(spec, "Execute(c) ==", "Redeliver(c) ==", "    /\\ finalAt[c] # 0\n", "", "effect-without-finality"),
        "cfg": effect_cfg,
        "invariant": "NoEffectWithoutFinality",
    })
    one_use_cfg = replace_once(effect_cfg, "    MaxUses = 2\n", "    MaxUses = 1\n", "one-use-config")
    controls.append({
        "label": "negative-budget-overrun",
        "spec": replace_in_section(spec, "Finalize(c) ==", "ObserveRevocation(r) ==", "    /\\ Cardinality(FinalizedUses) < MaxUses\n", "", "budget-overrun"),
        "cfg": one_use_cfg,
        "invariant": "FinalizedWithinBudget",
    })
    controls.append({
        "label": "negative-post-fault-effect",
        "spec": replace_in_section(spec, "Execute(c) ==", "Redeliver(c) ==", "    /\\ ~fault\n", "", "post-fault-effect"),
        "cfg": finality_cfg,
        "invariant": "FaultFreezesFinalityAndEffects",
    })
    old_cutoff = (
        "ExecutionStillAuthorized(s) ==\n"
        "    IF Cutoff = \"Effect\"\n"
        "    THEN NoRevocationAtOrBefore(s)\n"
        "    ELSE TRUE\n"
    )
    controls.append({
        "label": "negative-effect-cutoff-ignore",
        "spec": replace_once(spec, old_cutoff, "ExecutionStillAuthorized(s) == TRUE\n", "effect-cutoff-ignore"),
        "cfg": effect_cfg,
        "invariant": "RevocationCutoffConsistentWhenFaultFree",
    })
    controls.append({
        "label": "negative-clock-overrun",
        "spec": replace_in_section(spec, "Finalize(c) ==", "ObserveRevocation(r) ==", "    /\\ clock' = NextSeq\n", "    /\\ clock' = clock\n", "clock-regression"),
        "cfg": effect_cfg,
        "invariant": "RecordedEventsDoNotExceedClock",
    })
    return controls


def build_alloy_negative_controls(models: dict[str, str]) -> list[dict[str, str]]:
    budget = models["delegation_budget"]
    authority = models["authority"]
    return [
        {
            "label": "negative-sibling-allocation-overlap",
            "model": remove_fact(budget, "SiblingAllocationsAreDisjoint", "sibling-overlap"),
            "source_name": ALLOY_MODELS["delegation_budget"].name,
            "command": "SiblingsCannotDuplicateAllowance",
        },
        {
            "label": "negative-delegation-scope-broadening",
            "model": replace_in_section(
                budget,
                "fact DelegationAttenuates {",
                "fact NondelegablePowersHaveNoDelegatedChildren {",
                "    NarrowerOrSame[c.scope, c.parent.scope]\n",
                "",
                "scope-broadening",
            ),
            "source_name": ALLOY_MODELS["delegation_budget"].name,
            "command": "DelegationNeverBroadensScope",
        },
        {
            "label": "negative-duplicate-holder-concurrence",
            "model": replace_in_section(
                authority,
                "fact ConcurrenceBindsOneEnvelopeAndUniqueHolders {",
                "assert EveryPowerHasExactlyOneOwner {",
                "    all disj a, b: c.approvals | a.holder != b.holder\n",
                "",
                "duplicate-holder-concurrence",
            ),
            "source_name": ALLOY_MODELS["authority"].name,
            "command": "ConcurrenceCannotCountOneHolderTwice",
        },
    ]


def artifact_manifest(outdir: Path) -> dict[str, dict[str, Any]]:
    manifest: dict[str, dict[str, Any]] = {}
    for path in sorted(p for p in outdir.rglob("*") if p.is_file() and p.name != "artifact-manifest.json"):
        manifest[str(path.relative_to(outdir))] = {"sha256": sha256_file(path), "size": path.stat().st_size}
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--alloy-jar", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--expected-head", required=True)
    parser.add_argument("--java", default="java")
    args = parser.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.formal-qualification-receipt.v2",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "passed": False,
    }

    try:
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        expected_tla = pins["tla2tools"]["sha256"]
        expected_alloy = pins["alloy"]["sha256"]
        actual_tla = sha256_file(args.tla_jar)
        actual_alloy = sha256_file(args.alloy_jar)
        if actual_tla != expected_tla:
            raise RuntimeError(f"TLA+ JAR SHA-256 mismatch: expected {expected_tla}, got {actual_tla}")
        if actual_alloy != expected_alloy:
            raise RuntimeError(f"Alloy JAR SHA-256 mismatch: expected {expected_alloy}, got {actual_alloy}")

        git_rc, git_out = run(["git", "rev-parse", "HEAD"], cwd=REPO)
        if git_rc != 0:
            raise RuntimeError("could not resolve Git HEAD")
        actual_head = git_out.strip()
        expected_head = args.expected_head.strip()
        if actual_head != expected_head:
            raise RuntimeError(f"raw-head mismatch: expected {expected_head}, got {actual_head}")

        preflight = snapshot(FORMAL_INPUTS)
        java_rc, java_version = run([args.java, "-version"])
        if java_rc != 0:
            raise RuntimeError("java -version failed")

        spec_text = TLA_SPEC.read_text(encoding="utf-8")
        effect_cfg = TLA_CONFIGS["effect"].read_text(encoding="utf-8")
        finality_cfg = TLA_CONFIGS["finality"].read_text(encoding="utf-8")

        tlc_canonical = [
            run_tlc_case(
                label=f"canonical-{name}",
                spec_text=spec_text,
                cfg_text=cfg.read_text(encoding="utf-8"),
                java=args.java,
                tla_jar=args.tla_jar.resolve(),
                evidence_dir=outdir,
                expect_success=True,
            )
            for name, cfg in TLA_CONFIGS.items()
        ]
        tlc_negative = [
            run_tlc_case(
                label=control["label"],
                spec_text=control["spec"],
                cfg_text=control["cfg"],
                java=args.java,
                tla_jar=args.tla_jar.resolve(),
                evidence_dir=outdir,
                expect_success=False,
                expected_invariant=control["invariant"],
            )
            for control in build_tlc_negative_controls(spec_text, effect_cfg, finality_cfg)
        ]

        model_texts = {name: path.read_text(encoding="utf-8") for name, path in ALLOY_MODELS.items()}
        solver = pins["alloy"]["solver"]
        alloy_canonical = [
            run_alloy_case(
                label=f"canonical-{name}",
                source_text=model_texts[name],
                source_name=path.name,
                java=args.java,
                alloy_jar=args.alloy_jar.resolve(),
                solver=solver,
                evidence_dir=outdir,
                canonical=True,
            )
            for name, path in ALLOY_MODELS.items()
        ]
        alloy_negative = [
            run_alloy_case(
                label=control["label"],
                source_text=control["model"],
                source_name=control["source_name"],
                java=args.java,
                alloy_jar=args.alloy_jar.resolve(),
                solver=solver,
                evidence_dir=outdir,
                canonical=False,
                command_name=control["command"],
                negative_expect_sat=True,
            )
            for control in build_alloy_negative_controls(model_texts)
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
            "tools": {
                "tla2tools": {"version_pin": pins["tla2tools"]["version"], "sha256": actual_tla, "workers": 1, "deadlock_checking": False},
                "alloy": {"version_pin": pins["alloy"]["version"], "sha256": actual_alloy, "solver": solver},
            },
            "preflight_input_sha256": preflight,
            "tlc": {"canonical": tlc_canonical, "negative_controls": tlc_negative},
            "alloy": {"canonical": alloy_canonical, "negative_controls": alloy_negative},
            "postflight_input_sha256": postflight,
            "postflight_immutable": immutable,
        })
        all_results = tlc_canonical + tlc_negative + alloy_canonical + alloy_negative
        receipt["passed"] = immutable and all(bool(item.get("passed")) for item in all_results)
        receipt["completed_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    except Exception as exc:
        receipt["fatal_error"] = f"{type(exc).__name__}: {exc}"
        receipt["traceback"] = traceback.format_exc()
        receipt["completed_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        receipt["passed"] = False

    receipt_path = outdir / "qualification-receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (outdir / "artifact-manifest.json").write_text(json.dumps(artifact_manifest(outdir), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"receipt": str(receipt_path), "passed": receipt.get("passed", False)}, sort_keys=True))
    return 0 if receipt.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
