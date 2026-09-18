#!/usr/bin/env python3
"""Exact-head qualification for the constitutional refinement crosswalk scaffold."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
GOV = REPO / "mycelix-governance"
VALIDATOR = GOV / "tools" / "formal" / "validate_refinement_crosswalk.py"
MANIFEST = GOV / "specs" / "constitutional-refinement-crosswalk.v1.json"
SCHEMA = GOV / "specs" / "constitutional-refinement-crosswalk.v1.schema.json"
DOC = GOV / "docs" / "CONSTITUTIONAL_REFINEMENT_CROSSWALK_V0_1.md"
RUST_TYPES = GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "types.rs"
RUST_STATE = GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "state.rs"
TLA_MODEL = GOV / "specs" / "ConstitutionalClaimLifecycle.tla"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-refinement-crosswalk-qualification.yml"

INPUTS = [
    VALIDATOR,
    MANIFEST,
    SCHEMA,
    DOC,
    RUST_TYPES,
    RUST_STATE,
    TLA_MODEL,
    SCRIPT,
    WORKFLOW,
]

MUTATIONS = {
    "unmapped-rust-variant": "Rust lifecycle census mismatch",
    "unmapped-tla-status": "TLA+ status census mismatch",
    "duplicate-relationship": "duplicate/empty id",
    "wrong-source-blob": "blob drift",
    "forge-absent-enum": "Absent must map exactly once to derived_absence",
    "premature-qualified-status": "qualified manifest cannot retain pending_extensions",
}


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
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


def apply_mutation(worktree: Path, label: str) -> None:
    gov = worktree / "mycelix-governance"
    if label == "unmapped-rust-variant":
        path = gov / "crates" / "constitutional-claim-lifecycle" / "src" / "types.rs"
        text = path.read_text(encoding="utf-8")
        anchor = "    PendingExecutable,\n"
        if anchor not in text:
            raise RuntimeError("Rust mutation anchor missing")
        path.write_text(text.replace(anchor, anchor + "    SyntheticUnmapped,\n", 1), encoding="utf-8")
        return

    if label == "unmapped-tla-status":
        path = gov / "specs" / "ConstitutionalClaimLifecycle.tla"
        text = path.read_text(encoding="utf-8")
        anchor = '    "Halted"\n}\nTerminalStatuses'
        replacement = '    "Halted",\n    "SyntheticStatus"\n}\nTerminalStatuses'
        if anchor not in text:
            raise RuntimeError("TLA mutation anchor missing")
        path.write_text(text.replace(anchor, replacement, 1), encoding="utf-8")
        return

    path = gov / "specs" / "constitutional-refinement-crosswalk.v1.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    if label == "duplicate-relationship":
        data["relationships"].append(dict(data["relationships"][1]))
    elif label == "wrong-source-blob":
        data["relationships"][1]["concrete"]["source_git_blob_sha"] = "0" * 40
    elif label == "forge-absent-enum":
        data["relationships"][0]["concrete"]["symbol_type"] = "enum_variant"
    elif label == "premature-qualified-status":
        data["status"] = "qualified"
    else:
        raise RuntimeError(f"unknown mutation: {label}")
    path.write_text(json.dumps(data, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def validator_case(
    *,
    head: str,
    label: str,
    python: str,
    outdir: Path,
    mutation: str | None,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-crosswalk-{label}-") as td:
        root = Path(td)
        worktree = root / "repo"
        rc_add, add_out = run(["git", "worktree", "add", "--detach", str(worktree), head], cwd=REPO)
        if rc_add != 0:
            log = outdir / "cases" / f"{label}.log"
            return {
                "label": label,
                "passed": False,
                "returncode": rc_add,
                "error": "git worktree add failed",
                "log": rel(log),
                "log_sha256": write_log(log, add_out),
            }
        try:
            if mutation:
                apply_mutation(worktree, mutation)
            validator = worktree / "mycelix-governance" / "tools" / "formal" / "validate_refinement_crosswalk.py"
            receipt_path = root / "validator-receipt.json"
            cmd = [python, str(validator), "--out", str(receipt_path)]
            rc, output = run(cmd, cwd=worktree)
            log = outdir / "cases" / f"{label}.log"
            log_sha = write_log(log, output)
            parsed = None
            if receipt_path.is_file():
                parsed = json.loads(receipt_path.read_text(encoding="utf-8"))
                dest = outdir / "cases" / f"{label}-receipt.json"
                dest.write_text(json.dumps(parsed, indent=2, sort_keys=True) + "\n", encoding="utf-8")

            result: dict[str, Any] = {
                "label": label,
                "command": cmd,
                "returncode": rc,
                "validator_receipt": parsed,
                "log": rel(log),
                "log_sha256": log_sha,
            }
            if mutation is None:
                result["expected"] = "VALIDATOR_PASS"
                result["passed"] = rc == 0 and isinstance(parsed, dict) and parsed.get("passed") is True
            else:
                expected = MUTATIONS[mutation]
                errors = parsed.get("errors", []) if isinstance(parsed, dict) else []
                rendered_errors = "\n".join(str(x) for x in errors)
                result["expected"] = f"MUTATION_CAUGHT:{expected}"
                result["matched_diagnostic"] = expected in rendered_errors
                result["passed"] = rc != 0 and parsed is not None and parsed.get("passed") is False and expected in rendered_errors
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
    parser.add_argument("--expected-head", required=True)
    parser.add_argument("--expected-semantic-head", required=True)
    parser.add_argument("--expected-lifecycle-head", required=True)
    parser.add_argument("--expected-closure-head", required=True)
    parser.add_argument("--expected-temporal-head", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--python", default="python3")
    args = parser.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-refinement-crosswalk-qualification.v1",
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "expected_git_head": args.expected_head,
        "expected_semantic_head": args.expected_semantic_head,
        "expected_lifecycle_head": args.expected_lifecycle_head,
        "expected_closure_head": args.expected_closure_head,
        "expected_temporal_head": args.expected_temporal_head,
        "passed": False,
    }

    try:
        pre = snapshot(INPUTS)
        lineage = [
            ("HEAD", args.expected_head),
            ("HEAD^", args.expected_semantic_head),
            ("HEAD^^", args.expected_lifecycle_head),
            ("HEAD^^^", args.expected_closure_head),
            ("HEAD^^^^", args.expected_temporal_head),
        ]
        observed: dict[str, str] = {}
        for ref_name, expected in lineage:
            rc, output = run(["git", "rev-parse", ref_name], cwd=REPO)
            actual = output.strip()
            if rc != 0 or actual != expected:
                raise RuntimeError(f"lineage mismatch {ref_name}: expected={expected} actual={actual}")
            observed[ref_name] = actual

        py_rc, py_version = run([args.python, "--version"])
        git_rc, git_version = run(["git", "--version"])
        if py_rc != 0 or git_rc != 0:
            raise RuntimeError("tool version probe failed")

        canonical = validator_case(
            head=args.expected_head,
            label="canonical",
            python=args.python,
            outdir=outdir,
            mutation=None,
        )
        negative_controls = [
            validator_case(
                head=args.expected_head,
                label=label,
                python=args.python,
                outdir=outdir,
                mutation=label,
            )
            for label in MUTATIONS
        ]

        doc = DOC.read_text(encoding="utf-8")
        required_terms = [
            "ghost_environment_state",
            "stuttering_refinement",
            "persisted = false",
            "pending_extensions",
            "Absent",
        ]
        semantic_contract = {
            "missing_terms": [term for term in required_terms if term not in doc],
        }
        semantic_contract["passed"] = not semantic_contract["missing_terms"]

        post = snapshot(INPUTS)
        immutable = pre == post
        receipt.update({
            "git_lineage": observed,
            "environment": {
                "python": py_version.strip(),
                "git": git_version.strip(),
                "platform": platform.platform(),
                "github_run_id": os.environ.get("GITHUB_RUN_ID"),
                "github_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
                "runner_os": os.environ.get("RUNNER_OS"),
            },
            "preflight_input_sha256": pre,
            "canonical": canonical,
            "negative_controls": negative_controls,
            "semantic_contract": semantic_contract,
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            canonical["passed"]
            and all(case["passed"] for case in negative_controls)
            and semantic_contract["passed"]
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
