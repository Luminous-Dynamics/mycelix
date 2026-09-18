#!/usr/bin/env python3
"""Exact-head qualification for MYC-CONST-003B4 claim-content binding."""

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
PINS = SCRIPT.parent / "pins.json"
WORKFLOW = REPO / ".github" / "workflows" / "constitutional-claim-binding-qualification.yml"
DOC = GOV / "docs" / "CONSTITUTIONAL_CLAIM_BINDING_V0_1.md"
ALLOY_SPEC = GOV / "specs" / "ConstitutionalClaimBinding.als"
WORKSPACE = GOV / "Cargo.toml"

CRATE_DIRS = {
    "consumption": "constitutional-consumption",
    "temporal": "constitutional-temporal-provenance",
    "lifecycle": "constitutional-claim-lifecycle",
}

INPUTS = [
    WORKSPACE,
    GOV / "crates" / "constitutional-consumption" / "Cargo.toml",
    GOV / "crates" / "constitutional-consumption" / "src" / "lib.rs",
    GOV / "crates" / "constitutional-consumption" / "src" / "binding.rs",
    GOV / "crates" / "constitutional-consumption" / "tests" / "model.rs",
    GOV / "crates" / "constitutional-consumption" / "tests" / "claim_binding.rs",
    GOV / "crates" / "constitutional-temporal-provenance" / "Cargo.toml",
    GOV / "crates" / "constitutional-temporal-provenance" / "src" / "lib.rs",
    GOV / "crates" / "constitutional-temporal-provenance" / "tests" / "temporal.rs",
    GOV / "crates" / "constitutional-temporal-provenance" / "tests" / "binding.rs",
    GOV / "crates" / "constitutional-claim-lifecycle" / "Cargo.toml",
    GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "lib.rs",
    GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "types.rs",
    GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "state.rs",
    GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "transitions.rs",
    GOV / "crates" / "constitutional-claim-lifecycle" / "src" / "invariants.rs",
    GOV / "crates" / "constitutional-claim-lifecycle" / "tests" / "lifecycle.rs",
    DOC,
    ALLOY_SPEC,
    PINS,
    SCRIPT,
    WORKFLOW,
]

RUST_MUTATIONS: dict[str, dict[str, str]] = {
    "disable-bound-proof-binding": {
        "crate": "consumption",
        "file": "src/lib.rs",
        "before": "if self.claim_binding != claim.binding() {",
        "after": "if false {",
        "expected_test": "same_claim_id_with_mutated_payload_is_rejected",
    },
    "omit-payload-from-canonical-bytes": {
        "crate": "consumption",
        "file": "src/binding.rs",
        "before": "        push_str(&mut out, &self.payload_digest)?;\n",
        "after": "",
        "expected_test": "canonical_bytes_change_for_each_security_relevant_field",
    },
    "disable-temporal-rebinding-guard": {
        "crate": "temporal",
        "file": "src/lib.rs",
        "before": "            Some(existing) if existing != &bound.claim_binding => {",
        "after": "            Some(existing) if false && existing != &bound.claim_binding => {",
        "expected_test": "duplicate_evidence_id_with_different_binding_is_rejected",
    },
    "allow-retroactive-unbound-upgrade": {
        "crate": "temporal",
        "file": "src/lib.rs",
        "before": "            None if self.existing_finality(&evidence_id).is_some() => {",
        "after": "            None if false && self.existing_finality(&evidence_id).is_some() => {",
        "expected_test": "legacy_unbound_evidence_cannot_be_retroactively_upgraded",
    },
    "disable-restored-binding-census": {
        "crate": "lifecycle",
        "file": "src/invariants.rs",
        "before": "                let Some(binding) = self.temporal.finality_binding(evidence_id) else {\n                    return Err(ClaimLifecycleError::InvariantViolation);\n                };",
        "after": "                let Some(binding) = self.temporal.finality_binding(evidence_id) else {\n                    continue;\n                };",
        "expected_test": "invariants::binding_recovery_tests::restored_lifecycle_rejects_unbound_finality_record",
    },
    "disable-lifecycle-evidence-binding-check": {
        "crate": "lifecycle",
        "file": "src/transitions.rs",
        "before": "            || evidence_binding != &expected_binding\n",
        "after": "",
        "expected_test": "temporal_evidence_with_correct_ids_but_wrong_binding_cannot_authorize",
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


def crate_source(worktree: Path, mutation: dict[str, str]) -> Path:
    return (
        worktree
        / "mycelix-governance"
        / "crates"
        / CRATE_DIRS[mutation["crate"]]
        / mutation["file"]
    )


def cargo_case(
    *,
    head: str,
    outdir: Path,
    label: str,
    crate: str,
    cargo: str,
    mutation: dict[str, str] | None = None,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-binding-rust-{label}-") as td:
        root = Path(td)
        worktree = root / "repo"
        rc_add, add_out = run(["git", "worktree", "add", "--detach", str(worktree), head], cwd=REPO)
        if rc_add != 0:
            log = outdir / "rust" / f"{label}.log"
            return {
                "label": label,
                "passed": False,
                "returncode": rc_add,
                "error": "git worktree add failed",
                "log": rel(log),
                "log_sha256": write_log(log, add_out),
            }

        try:
            expected_test = None
            if mutation is not None:
                source = crate_source(worktree, mutation)
                text = source.read_text(encoding="utf-8")
                mutated = text.replace(mutation["before"], mutation["after"], 1)
                if mutated == text:
                    raise RuntimeError(f"mutation did not apply: {label}")
                source.write_text(mutated, encoding="utf-8")
                expected_test = mutation["expected_test"]

            manifest = (
                worktree
                / "mycelix-governance"
                / "crates"
                / CRATE_DIRS[crate]
                / "Cargo.toml"
            )
            target = root / "target"
            cmd = [cargo, "test", "--manifest-path", str(manifest), "--all-targets"]
            rc, output = run(cmd, cwd=worktree, env={"CARGO_TARGET_DIR": str(target)})
            failed_tests = sorted(set(re.findall(r"test ([A-Za-z0-9_:]+) \.\.\. FAILED", output)))
            log = outdir / "rust" / f"{label}.log"
            log_sha = write_log(log, output)

            lock = worktree / "mycelix-governance" / "Cargo.lock"
            lock_sha = sha256_file(lock) if lock.is_file() else None
            if mutation is None and lock.is_file():
                dest = outdir / "rust" / f"derived-Cargo-{crate}.lock"
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(lock, dest)

            result: dict[str, Any] = {
                "label": label,
                "crate": crate,
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


def alloy_case(
    *,
    label: str,
    spec_text: str,
    alloy_jar: Path,
    java: str,
    outdir: Path,
    expect_solution: bool,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix=f"mycelix-binding-alloy-{label}-") as td:
        work = Path(td)
        model = work / "ConstitutionalClaimBinding.als"
        alloy_out = work / "alloy-output"
        model.write_text(spec_text, encoding="utf-8")
        cmd = [
            java,
            "-jar",
            str(alloy_jar),
            "exec",
            "-q",
            "-o",
            str(alloy_out),
            "-f",
            "-s",
            "sat4j",
            str(model),
        ]
        rc, output = run(
            cmd,
            cwd=work,
            env={"JAVA_TOOL_OPTIONS": "-Djava.awt.headless=true"},
        )

        receipt_path = alloy_out / "receipt.json"
        receipt_present = receipt_path.is_file()
        command_label = None
        solution_count = None
        receipt_sha = None
        parse_error = None
        if receipt_present:
            try:
                alloy_receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
                commands = alloy_receipt.get("commands", {})
                if len(commands) != 1:
                    raise RuntimeError(f"expected exactly one Alloy command receipt, got {len(commands)}")
                command_label, command_receipt = next(iter(commands.items()))
                solutions = command_receipt.get("solution", [])
                if not isinstance(solutions, list):
                    raise RuntimeError("Alloy command receipt solution field is not a list")
                solution_count = len(solutions)
                receipt_sha = sha256_file(receipt_path)
                dest = outdir / "alloy" / f"{label}-receipt.json"
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(receipt_path, dest)
            except Exception as exc:
                parse_error = f"{type(exc).__name__}: {exc}"

    log = outdir / "alloy" / f"{label}.log"
    log_sha = write_log(log, output)
    observed_solution = solution_count is not None and solution_count > 0
    return {
        "label": label,
        "command": cmd,
        "returncode": rc,
        "expected_solution": expect_solution,
        "receipt_present": receipt_present,
        "receipt_sha256": receipt_sha,
        "command_label": command_label,
        "solution_count": solution_count,
        "parse_error": parse_error,
        "log": rel(log),
        "log_sha256": log_sha,
        "passed": (
            rc == 0
            and receipt_present
            and parse_error is None
            and observed_solution == expect_solution
        ),
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
    parser.add_argument("--alloy-jar", type=Path, required=True)
    parser.add_argument("--expected-head", required=True)
    parser.add_argument("--expected-semantic-head", required=True)
    parser.add_argument("--expected-lifecycle-head", required=True)
    parser.add_argument("--expected-closure-head", required=True)
    parser.add_argument("--expected-temporal-head", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--java", default="java")
    parser.add_argument("--cargo", default="cargo")
    args = parser.parse_args()

    outdir = args.out.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    receipt: dict[str, Any] = {
        "schema": "mycelix.constitutional-claim-binding-qualification.v2",
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

        alloy_sha = sha256_file(args.alloy_jar)
        if alloy_sha != pins["alloy"]["sha256"]:
            raise RuntimeError("Alloy JAR SHA-256 mismatch")

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
        cargo_rc, cargo_version = run([args.cargo, "--version"])
        rustc_rc, rustc_version = run(["rustc", "--version"])
        if java_rc != 0 or cargo_rc != 0 or rustc_rc != 0:
            raise RuntimeError("tool version probe failed")

        receipt.update({
            "git_lineage": observed_lineage,
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
            "alloy": {
                "version": pins["alloy"]["version"],
                "solver": pins["alloy"]["solver"],
                "sha256": alloy_sha,
                "evidence_format": "Alloy CLI receipt.json command solution cardinality",
            },
            "preflight_input_sha256": pre,
        })

        canonical_rust = [
            cargo_case(
                head=args.expected_head,
                outdir=outdir,
                label=f"canonical-{crate}",
                crate=crate,
                cargo=args.cargo,
            )
            for crate in ("consumption", "temporal", "lifecycle")
        ]

        rust_negative_controls = [
            cargo_case(
                head=args.expected_head,
                outdir=outdir,
                label=label,
                crate=mutation["crate"],
                cargo=args.cargo,
                mutation=mutation,
            )
            for label, mutation in RUST_MUTATIONS.items()
        ]

        alloy_text = ALLOY_SPEC.read_text(encoding="utf-8")
        safety_text = re.sub(
            r"(?m)^run LegacyIdCollisionCandidate[^\n]*\n?",
            "",
            alloy_text,
        )
        witness_text = re.sub(
            r"(?m)^check BindingAuthenticatesAtMostOneClaim[^\n]*\n?",
            "",
            alloy_text,
        )
        if safety_text == alloy_text or witness_text == alloy_text:
            raise RuntimeError("failed to isolate Alloy safety/non-vacuity commands")

        mutant_text = safety_text.replace("  b.payload = c.payload\n", "", 1)
        if mutant_text == safety_text:
            raise RuntimeError("Alloy negative-control mutation did not apply")

        alloy_safety = alloy_case(
            label="canonical-injectivity",
            spec_text=safety_text,
            alloy_jar=args.alloy_jar.resolve(),
            java=args.java,
            outdir=outdir,
            expect_solution=False,
        )
        alloy_witness = alloy_case(
            label="nonvacuity-legacy-id-collision",
            spec_text=witness_text,
            alloy_jar=args.alloy_jar.resolve(),
            java=args.java,
            outdir=outdir,
            expect_solution=True,
        )
        alloy_negative = alloy_case(
            label="negative-omit-payload-binding",
            spec_text=mutant_text,
            alloy_jar=args.alloy_jar.resolve(),
            java=args.java,
            outdir=outdir,
            expect_solution=True,
        )

        doc_text = DOC.read_text(encoding="utf-8")
        required_doc_terms = [
            "BoundFinalityProof",
            "BoundFinalityEvidence",
            "Retained unbound evidence is not lifecycle authority",
            "non-retroactive",
            "one formal Claim atom",
            "ConstitutionalClaimBinding.als",
        ]
        model_terms = [
            "BindingAuthenticatesAtMostOneClaim",
            "LegacyIdCollisionCandidate",
            "b.payload = c.payload",
        ]
        contract = {
            "missing_doc_terms": [term for term in required_doc_terms if term not in doc_text],
            "missing_model_terms": [term for term in model_terms if term not in alloy_text],
        }
        contract["passed"] = not contract["missing_doc_terms"] and not contract["missing_model_terms"]

        post = snapshot(INPUTS)
        immutable = pre == post
        receipt.update({
            "canonical_rust": canonical_rust,
            "rust_negative_controls": rust_negative_controls,
            "alloy_safety": alloy_safety,
            "alloy_nonvacuity": alloy_witness,
            "alloy_negative_control": alloy_negative,
            "semantic_contract": contract,
            "postflight_input_sha256": post,
            "postflight_immutable": immutable,
        })
        receipt["passed"] = (
            all(item["passed"] for item in canonical_rust)
            and all(item["passed"] for item in rust_negative_controls)
            and alloy_safety["passed"]
            and alloy_witness["passed"]
            and alloy_negative["passed"]
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
