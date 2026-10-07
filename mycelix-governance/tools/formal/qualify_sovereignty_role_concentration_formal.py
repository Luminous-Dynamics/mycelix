#!/usr/bin/env python3
"""Candidate-only exact-head formal qualifier for role-concentration independence."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

FAIL = "ROLE_CONCENTRATION_FORMAL_QUALIFICATION_FAIL: "
CANONICAL_OK = "Model checking completed. No error has been found."

TLA_INVARIANTS = [
    "TypeOK",
    "RoleConcentrationRequiresFinding",
    "FullControlRequiresIndependentExternalReview",
]

TLA_NEGATIVE = {
    "role-conflict": "RoleConcentrationRequiresFinding",
    "full-control-review": "FullControlRequiresIndependentExternalReview",
    "self-review-full-control": "FullControlRequiresIndependentExternalReview",
    "same-role-review-full-control": "FullControlRequiresIndependentExternalReview",
}

ALLOY_SAT = {
    "ConcentratedRolesWithFinding",
    "FullControlWithIndependentReview",
    "FullControlWithRoleDisjointReview",
}

ALLOY_UNSAT_RUNS = {
    "SelfReviewOnlyFullControl",
    "SameRoleReviewerFullControl",
}

ALLOY_UNSAT_CHECKS = {
    "RoleConcentrationRequiresFindingInvariant",
    "FullControlRequiresIndependentReviewInvariant",
    "SelfReviewAloneDoesNotCountAsIndependentReview",
}


def fail(message: str) -> "NoReturn":
    raise RuntimeError(FAIL + message)


def git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_json(path: Path) -> tuple[dict, bytes]:
    raw = path.read_bytes()
    try:
        return json.loads(raw.decode("utf-8")), raw
    except Exception as exc:
        fail(f"invalid JSON {path}: {exc}")


def run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)


def record(evidence: Path, name: str, cmd: list[str], result: subprocess.CompletedProcess[str]) -> dict:
    log = evidence / f"{name}.log"
    log.write_text(result.stdout, encoding="utf-8")
    return {
        "command": cmd,
        "returncode": result.returncode,
        "log_sha256": sha256_file(log),
        "log_path": log.name,
    }


def show_blob(subject_sha: str, path: str, output: Path) -> str:
    data = subprocess.check_output(["git", "show", f"{subject_sha}:{path}"])
    output.write_bytes(data)
    return git_blob_sha1(data)


def outcomes(output: str) -> list[dict]:
    rows = []
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            rows.append(json.loads(line))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", type=Path, required=True)
    ap.add_argument("--pins", type=Path, required=True)
    ap.add_argument("--crosswalk", type=Path, required=True)
    ap.add_argument("--alloy-runner", type=Path, required=True)
    ap.add_argument("--alloy-class-dir", type=Path, required=True)
    ap.add_argument("--tla-jar", type=Path, required=True)
    ap.add_argument("--alloy-jar", type=Path, required=True)
    ap.add_argument("--evidence-dir", type=Path, required=True)
    args = ap.parse_args()

    evidence = args.evidence_dir.resolve()
    evidence.mkdir(parents=True, exist_ok=True)

    profile, profile_bytes = read_json(args.profile)
    pins, pins_bytes = read_json(args.pins)
    crosswalk, crosswalk_bytes = read_json(args.crosswalk)

    if profile.get("schema") != "mycelix.sovereignty-role-concentration-formal-qualification-profile.v2":
        fail("unexpected role concentration profile schema")
    if profile.get("status") != "candidate-verification-profile":
        fail("profile is not candidate-verification-profile")
    if profile.get("authority") != "non-authoritative":
        fail("profile is not explicitly non-authoritative")
    if profile.get("claim_contract", {}).get("candidate_only") is not True:
        fail("profile candidate_only contract missing")

    if crosswalk.get("schema") != "mycelix.sovereignty-role-concentration-formal-crosswalk.v2":
        fail("unexpected role concentration crosswalk schema")
    if crosswalk.get("status") != "candidate-verification" or crosswalk.get("authority") != "non-authoritative":
        fail("crosswalk is not explicitly non-authoritative")

    subject = profile["semantic_subject"]
    subject_sha = subject["head_sha"]
    subject_tree = subject["tree_sha"]
    if crosswalk.get("formal_subject_head") != subject_sha:
        fail("crosswalk subject head mismatch")
    if crosswalk.get("formal_subject_tree") != subject_tree:
        fail("crosswalk subject tree mismatch")

    subprocess.check_call(["git", "rev-parse", f"{subject_sha}^{{commit}}"],
                          stdout=subprocess.DEVNULL)
    actual_tree = subprocess.check_output(
        ["git", "rev-parse", f"{subject_sha}^{{tree}}"], text=True
    ).strip()
    if actual_tree != subject_tree:
        fail("frozen subject tree mismatch")

    frozen = evidence / "subject"
    frozen.mkdir(parents=True, exist_ok=True)
    tla_meta = profile["models"]["tla"]
    neg_meta = profile["models"]["negative_tla"]
    alloy_meta = profile["models"]["alloy"]
    ref_meta = profile["models"]["reference"]

    blobs = {
        "tla": show_blob(subject_sha, tla_meta["path"], frozen / "model.tla"),
        "cfg": show_blob(subject_sha, tla_meta["config_path"], frozen / "model.cfg"),
        "negative_tla": show_blob(subject_sha, neg_meta["path"], frozen / "negative.tla"),
        "alloy": show_blob(subject_sha, alloy_meta["path"], frozen / "model.als"),
        "reference": show_blob(subject_sha, ref_meta["path"], frozen / "reference.py"),
    }

    expected_blobs = {
        "tla": tla_meta["git_blob_sha"],
        "cfg": tla_meta["config_git_blob_sha"],
        "negative_tla": neg_meta["git_blob_sha"],
        "alloy": alloy_meta["git_blob_sha"],
        "reference": ref_meta["git_blob_sha"],
    }
    for name, actual in blobs.items():
        if actual != expected_blobs[name]:
            fail(f"{name} frozen blob mismatch: {actual} != {expected_blobs[name]}")

    verifier = profile.get("verifier", {})
    bindings = {
        "qualifier": (Path(__file__), verifier.get("qualifier_path"), verifier.get("qualifier_blob_sha")),
        "pins": (args.pins, verifier.get("pins_path"), verifier.get("pins_blob_sha")),
        "crosswalk": (args.crosswalk, verifier.get("crosswalk_path"), verifier.get("crosswalk_blob_sha")),
        "alloy_runner": (args.alloy_runner, verifier.get("alloy_runner_path"), verifier.get("alloy_runner_blob_sha")),
    }
    for name, (path, expected_path, expected_blob) in bindings.items():
        if expected_path and path.as_posix() != expected_path:
            fail(name + " path does not match profile")
        if expected_blob and git_blob_sha1(path.read_bytes()) != expected_blob:
            fail(name + " blob does not match profile")

    if verifier.get("workflow_path"):
        workflow_path = Path(verifier["workflow_path"])
        if git_blob_sha1(workflow_path.read_bytes()) != verifier.get("workflow_git_blob_sha"):
            fail("workflow blob does not match profile")

    if pins.get("schema") != "mycelix.sovereignty-role-concentration-formal-tool-pins.v1":
        fail("unexpected role concentration tool pin schema")
    if sha256_file(args.tla_jar) != pins["tools"]["tla2tools"]["sha256"]:
        fail("TLA tool digest mismatch")
    if sha256_file(args.alloy_jar) != pins["tools"]["alloy"]["sha256"]:
        fail("Alloy tool digest mismatch")

    workflow = Path(".github/workflows/sovereignty-role-concentration-formal-candidate.yml")
    if not workflow.exists():
        fail("candidate workflow file is missing")
    workflow_text = workflow.read_text(encoding="utf-8")
    for action, sha in pins["github_actions"].items():
        needle = {
            "checkout": f"actions/checkout@{sha}",
            "install_nix": f"cachix/install-nix-action@{sha}",
            "upload_artifact": f"actions/upload-artifact@{sha}",
        }[action]
        if needle not in workflow_text:
            fail("workflow action pin mismatch: " + action)

    baseline = {
        "profile": sha256_file(args.profile),
        "pins": sha256_file(args.pins),
        "crosswalk": sha256_file(args.crosswalk),
        "workflow": sha256_file(workflow),
        "qualifier": sha256_file(Path(__file__)),
        "alloy_runner": sha256_file(args.alloy_runner),
    }

    receipt = {
        "schema": "mycelix.sovereignty-role-concentration-formal-receipt.v2",
        "result": "ExecutedFail",
        "authority": "non-authoritative",
        "subject": {
            "repository": subject["repository"],
            "head_sha": subject_sha,
            "tree_sha": subject_tree,
            "model_blobs": blobs,
        },
        "verifier": {
            "profile_sha256": hashlib.sha256(profile_bytes).hexdigest(),
            "pins_sha256": hashlib.sha256(pins_bytes).hexdigest(),
            "crosswalk_sha256": hashlib.sha256(crosswalk_bytes).hexdigest(),
            "qualifier_git_blob_sha": git_blob_sha1(Path(__file__).read_bytes()),
            "alloy_runner_git_blob_sha": git_blob_sha1(args.alloy_runner.read_bytes()),
        },
        "tools": {
            "tla2tools": {"version": pins["tools"]["tla2tools"]["version"], "sha256": sha256_file(args.tla_jar)},
            "alloy": {"version": pins["tools"]["alloy"]["version"], "sha256": sha256_file(args.alloy_jar), "solver": "SAT4J"},
        },
        "canonical": {},
        "negative_controls": {"tla": {}},
        "nonclaims": profile["nonclaims"],
    }

    try:
        ref = run([sys.executable, str(frozen / "reference.py")])
        receipt["canonical"]["reference"] = record(evidence, "reference", [sys.executable, str(frozen / "reference.py")], ref)
        if ref.returncode != 0:
            fail("reference oracle failed")
        for marker in [
            "CANONICAL PASS: no bounded role-concentration invariant violation through depth 4",
            "NEGATIVE PASS: role-conflict -> RoleConcentrationRequiresFinding counterexample",
            "NEGATIVE PASS: full-control-review -> FullControlRequiresIndependentExternalReview counterexample",
            "NEGATIVE PASS: self-review-full-control -> FullControlRequiresIndependentExternalReview counterexample",
            "NEGATIVE PASS: same-role-review -> FullControlRequiresIndependentExternalReview counterexample",
            "BOUNDED ROLE-CONCENTRATION REFERENCE EXPLORATION PASS: smoke evidence only",
        ]:
            if marker not in ref.stdout:
                fail("reference oracle missing marker: " + marker)

        tla_cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(frozen / "model.cfg"), str(frozen / "model.tla")]
        tla = run(tla_cmd)
        receipt["canonical"]["tla"] = record(evidence, "tla-canonical", tla_cmd, tla)
        if tla.returncode != 0 or CANONICAL_OK not in tla.stdout:
            fail("canonical TLA+ did not complete cleanly")
        for invariant in TLA_INVARIANTS:
            if f"Error: Invariant {invariant} is violated." in tla.stdout:
                fail("canonical TLA+ invariant violated: " + invariant)

        neg_dir = evidence / "tla-negative"
        neg_dir.mkdir(exist_ok=True)
        neg_src = (frozen / "negative.tla").read_text(encoding="utf-8")
        for control, target in TLA_NEGATIVE.items():
            work = neg_dir / control
            work.mkdir()
            wrapper = work / "ArtificialSovereigntyRoleConcentrationV1NegativeControls.tla"
            canonical = work / "ArtificialSovereigntyRoleConcentrationV1.tla"
            config = work / "negative.cfg"
            wrapper.write_text(neg_src, encoding="utf-8")
            canonical.write_text((frozen / "model.tla").read_text(encoding="utf-8"), encoding="utf-8")
            cfg = (
                "CONSTANTS\nS1 = S1\nS2 = S2\n"
                "Operator = Operator\nVerifier = Verifier\nEvidenceArchive = EvidenceArchive\n"
                "Adjudicator = Adjudicator\nMaxTime = 8\n"
                f'Control = "{control}"\n\n'
                "INIT Init\nNEXT NegativeNext\nINVARIANTS\n" +
                "\n".join(TLA_INVARIANTS) + "\nCHECK_DEADLOCK FALSE\n"
            )
            config.write_text(cfg, encoding="utf-8")
            cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(config), str(wrapper)]
            result = run(cmd)
            receipt["negative_controls"]["tla"][control] = {
                **record(evidence, "tla-negative-" + control, cmd, result),
                "target_invariant": target,
                "wrapper_sha256": sha256_file(wrapper),
                "canonical_module_sha256": sha256_file(canonical),
                "config_sha256": sha256_file(config),
            }
            if result.returncode == 0 or f"Error: Invariant {target} is violated." not in result.stdout:
                fail("TLA negative control failed: " + control)

        alloy_cmd = ["java", "-cp", f"{args.alloy_class_dir}:{args.alloy_jar}",
                     "SovereigntyRoleConcentrationAlloyQualificationRunner", str(frozen / "model.als")]
        alloy = run(alloy_cmd)
        receipt["canonical"]["alloy"] = record(evidence, "alloy-canonical", alloy_cmd, alloy)
        if alloy.returncode != 0:
            fail("canonical Alloy runner failed")
        rows = outcomes(alloy.stdout)
        by_label = {row["label"]: row for row in rows}
        expected_labels = ALLOY_SAT | ALLOY_UNSAT_RUNS | ALLOY_UNSAT_CHECKS
        if set(by_label) != expected_labels:
            fail("Alloy command label set mismatch")
        for label in ALLOY_SAT:
            if by_label[label]["actual"] != "SAT":
                fail("expected SAT run failed: " + label)
        for label in ALLOY_UNSAT_RUNS:
            if by_label[label]["actual"] != "UNSAT":
                fail("expected UNSAT run failed: " + label)
        for label in ALLOY_UNSAT_CHECKS:
            if by_label[label]["actual"] != "UNSAT" or by_label[label]["expects"] != 0:
                fail("expected UNSAT check failed: " + label)
        scope = alloy_meta["scope"]
        for label, row in by_label.items():
            cmd_text = row["command"]
            if f"for {scope['overall']}" not in cmd_text or f"{scope['int']} int" not in cmd_text:
                fail("Alloy overall/int scope mismatch: " + label)
            for name in ("Subject", "Role"):
                token = f"{scope[name]} {name}"
                if token not in cmd_text:
                    fail("Alloy atom scope mismatch: " + token + " for " + label)

        receipt["canonical"]["alloy"]["commands"] = rows
        receipt["canonical"]["alloy"]["solver"] = "SAT4J"
        receipt["canonical"]["alloy"]["bitwidth"] = alloy_meta["bitwidth"]

        if {k: sha256_file(Path(k)) for k in baseline} != baseline:
            fail("verifier inputs changed during qualification")
        if subprocess.run(["git", "status", "--porcelain"], text=True, stdout=subprocess.PIPE).stdout.strip():
            fail("repository changed during qualification")

        receipt["result"] = "CandidateQualifiedExactHead"
        receipt["postflight"] = {"inputs_unchanged": True, "git_status_clean": True, "baseline": baseline}
    except Exception as exc:
        receipt["failure"] = str(exc)
        (evidence / "sovereignty-role-concentration-formal-receipt-v2.json").write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(receipt, sort_keys=True))
        print(str(exc), file=sys.stderr)
        return 1

    (evidence / "sovereignty-role-concentration-formal-receipt-v2.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
