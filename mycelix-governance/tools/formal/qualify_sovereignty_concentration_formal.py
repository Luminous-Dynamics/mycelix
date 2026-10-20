#!/usr/bin/env python3
"""Candidate-only bounded formal qualifier for the frozen concentration subject."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

FAIL = "CONCENTRATION_FORMAL_QUALIFICATION_FAIL: "
CANONICAL_OK = "Model checking completed. No error has been found."

TLA_INVARIANTS = [
    "TypeOK",
    "AuthorityHasExplicitSource",
    "JurisdictionHasExplicitSource",
    "ScaleDoesNotIncreasePoliticalWeight",
    "HighSwitchingCostTriggersReview",
]

TLA_NEGATIVE = {
    "scale-weight": "ScaleDoesNotIncreasePoliticalWeight",
    "gatekeep-authority": "AuthorityHasExplicitSource",
    "acquire-jurisdiction": "JurisdictionHasExplicitSource",
    "critical-jurisdiction": "JurisdictionHasExplicitSource",
    "switching-review": "HighSwitchingCostTriggersReview",
}

ALLOY_SAT = {
    "HighScaleWithoutWeightIncrease",
    "CriticalOperatorWithoutJurisdiction",
    "GatekeeperWithoutConstitutionalPower",
    "AcquisitionWithoutConstitutionalTransfer",
    "HighSwitchingCostRequiresReviewWitness",
}

ALLOY_UNSAT = {
    "ScaleDoesNotIncreasePoliticalWeight",
    "JurisdictionHasExplicitSourceInvariant",
    "AcquisitionDoesNotTransferAuthority",
    "GatekeepingDoesNotTransferAuthority",
    "HighSwitchingCostIsReviewable",
}


def fail(message: str) -> "NoReturn":
    raise RuntimeError(FAIL + message)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def read_json(path: Path) -> tuple[dict, bytes]:
    raw = path.read_bytes()
    try:
        return json.loads(raw.decode("utf-8")), raw
    except Exception as exc:
        fail(f"invalid JSON {path}: {exc}")


def run(cmd: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, stdout=subprocess.PIPE,
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


def git_show(commit: str, path: str, output: Path) -> str:
    data = subprocess.check_output(["git", "show", f"{commit}:{path}"])
    output.write_bytes(data)
    return git_blob_sha1(data)


def alloy_outcomes(output: str) -> list[dict]:
    rows = []
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            rows.append(json.loads(line))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", type=Path, required=True)
    ap.add_argument("--pins", type=Path, required=True)
    ap.add_argument("--alloy-runner", type=Path, required=True)
    ap.add_argument("--alloy-class-dir", type=Path, required=True)
    ap.add_argument("--tla-jar", type=Path, required=True)
    ap.add_argument("--alloy-jar", type=Path, required=True)
    ap.add_argument("--evidence-dir", type=Path, required=True)
    ap.add_argument("--crosswalk", type=Path, required=True)
    args = ap.parse_args()

    evidence = args.evidence_dir.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    profile, profile_bytes = read_json(args.profile)
    pins, pins_bytes = read_json(args.pins)
    crosswalk, crosswalk_bytes = read_json(args.crosswalk)

    if profile.get("schema") != "mycelix.sovereignty-concentration-formal-qualification-profile.v1":
        fail("unexpected profile schema")
    if profile.get("status") != "candidate-verification-profile":
        fail("profile is not candidate-verification-profile")
    if profile.get("authority") != "non-authoritative":
        fail("profile is not explicitly non-authoritative")
    if profile.get("claim_contract", {}).get("candidate_only") is not True:
        fail("profile candidate_only contract missing")
    if crosswalk.get("schema") != "mycelix.sovereignty-concentration-formal-crosswalk.v1":
        fail("unexpected concentration crosswalk schema")
    if crosswalk.get("status") != "candidate-verification" or crosswalk.get("authority") != "non-authoritative":
        fail("concentration crosswalk is not explicitly non-authoritative")

    subject = profile["semantic_subject"]
    subject_sha = subject["head_sha"]
    subject_tree = subject["tree_sha"]
    if crosswalk.get("formal_subject_head") != subject_sha or crosswalk.get("formal_subject_tree") != subject_tree:
        fail("crosswalk subject identity mismatch")
    if not re.fullmatch(r"[0-9a-f]{40}", subject_sha):
        fail("invalid frozen subject SHA")
    if not re.fullmatch(r"[0-9a-f]{40}", subject_tree):
        fail("invalid frozen subject tree SHA")
    if subprocess.check_output(["git", "rev-parse", f"{subject_sha}^{{commit}}"]], text=True).strip() != subject_sha:
        fail("frozen subject commit is not present locally")
    if subprocess.check_output(["git", "rev-parse", f"{subject_sha}^{{tree}}"]], text=True).strip() != subject_tree:
        fail("frozen subject tree mismatch")

    model_root = evidence / "subject"
    model_root.mkdir(parents=True, exist_ok=True)
    tla_meta = profile["models"]["tla"]
    neg_meta = profile["models"]["negative_tla"]
    alloy_meta = profile["models"]["alloy"]
    ref_meta = profile["models"]["reference"]

    tla_blob = git_show(subject_sha, tla_meta["path"], model_root / "model.tla")
    cfg_blob = git_show(subject_sha, tla_meta["config_path"], model_root / "model.cfg")
    neg_blob = git_show(subject_sha, neg_meta["path"], model_root / "negative.tla")
    alloy_blob = git_show(subject_sha, alloy_meta["path"], model_root / "model.als")
    ref_blob = git_show(subject_sha, ref_meta["path"], model_root / "reference.py")

    if crosswalk.get("reference_oracle_blob") != ref_blob:
        fail("crosswalk reference oracle commitment mismatch")
    verifier_meta = profile.get("verifier", {})
    expected_bindings = {
        "qualifier": (Path(__file__), verifier_meta.get("qualifier_path"), verifier_meta.get("qualifier_blob_sha")),
        "alloy_runner": (args.alloy_runner, verifier_meta.get("alloy_runner_path"), verifier_meta.get("alloy_runner_blob_sha")),
        "pins": (args.pins, verifier_meta.get("pins_path"), verifier_meta.get("pins_blob_sha")),
        "crosswalk": (args.crosswalk, verifier_meta.get("crosswalk_path"), verifier_meta.get("crosswalk_blob_sha")),
    }
    for label, (path, expected_path, expected_blob) in expected_bindings.items():
        if not expected_path or path.as_posix() != expected_path:
            fail(label + " path does not match profile binding")
        if not expected_blob:
            fail(label + " blob commitment is missing from profile")
        if git_blob_sha1(path.read_bytes()) != expected_blob:
            fail(label + " blob does not match profile binding")

    for actual, expected, label in [
        (tla_blob, tla_meta["git_blob_sha"], "TLA"),
        (cfg_blob, tla_meta["config_git_blob_sha"], "TLA config"),
        (neg_blob, neg_meta["git_blob_sha"], "TLA negative controls"),
        (alloy_blob, alloy_meta["git_blob_sha"], "Alloy"),
        (ref_blob, ref_meta["git_blob_sha"], "reference oracle"),
    ]:
        if actual != expected:
            fail(f"{label} frozen blob mismatch: {actual} != {expected}")

    if pins.get("schema") != "mycelix.sovereignty-concentration-formal-tool-pins.v1":
        fail("unexpected tool pin schema")
    if sha256_file(args.tla_jar) != pins["tools"]["tla2tools"]["sha256"]:
        fail("TLA tool digest mismatch")
    if sha256_file(args.alloy_jar) != pins["tools"]["alloy"]["sha256"]:
        fail("Alloy tool digest mismatch")

    workflow_path = Path(".github/workflows/sovereignty-concentration-formal-candidate.yml")
    workflow = workflow_path.read_text(encoding="utf-8")
    workflow_meta = profile.get("verifier", {})
    if workflow_meta.get("workflow_path") != workflow_path.as_posix():
        fail("workflow path does not match profile binding")
    if git_blob_sha1(workflow_path.read_bytes()) != workflow_meta.get("workflow_git_blob_sha"):
        fail("workflow bytes do not match profile binding")
    for action, sha in pins["github_actions"].items():
        needle = {
            "checkout": f"actions/checkout@{sha}",
            "install_nix": f"cachix/install-nix-action@{sha}",
            "upload_artifact": f"actions/upload-artifact@{sha}",
        }[action]
        if needle not in workflow:
            fail(f"workflow action pin mismatch: {action}")

    receipt = {
        "schema": "mycelix.sovereignty-concentration-formal-receipt.v1",
        "result": "ExecutedFail",
        "authority": "non-authoritative",
        "subject": {"repository": subject["repository"], "head_sha": subject_sha, "tree_sha": subject_tree,
                    "model_blobs": {"tla": tla_blob, "cfg": cfg_blob, "negative_tla": neg_blob,
                                    "alloy": alloy_blob, "reference": ref_blob}},
        "verifier": {"profile_sha256": hashlib.sha256(profile_bytes).hexdigest(),
                     "pins_sha256": hashlib.sha256(pins_bytes).hexdigest(),
                     "crosswalk_sha256": hashlib.sha256(crosswalk_bytes).hexdigest(),
                     "qualifier_git_blob_sha": git_blob_sha1(Path(__file__).read_bytes()),
                     "alloy_runner_git_blob_sha": git_blob_sha1(args.alloy_runner.read_bytes()),
                     "pins_git_blob_sha": git_blob_sha1(args.pins.read_bytes()),
                     "crosswalk_git_blob_sha": git_blob_sha1(args.crosswalk.read_bytes())},
        "tools": {"tla2tools": {"version": pins["tools"]["tla2tools"]["version"], "sha256": sha256_file(args.tla_jar)},
                  "alloy": {"version": pins["tools"]["alloy"]["version"], "sha256": sha256_file(args.alloy_jar), "solver": "SAT4J"}},
        "canonical": {}, "negative_controls": {"tla": {}, "alloy": {}},
        "nonclaims": profile["nonclaims"],
    }

    baseline = {
        "profile": sha256_file(args.profile),
        "pins": sha256_file(args.pins),
        "workflow": hashlib.sha256(workflow.encode("utf-8")).hexdigest(),
        "alloy_runner": sha256_file(args.alloy_runner),
        "crosswalk": sha256_file(args.crosswalk),
    }

    try:
        ref = run([sys.executable, str(model_root / "reference.py")])
        receipt["canonical"]["reference"] = record(evidence, "reference", [sys.executable, str(model_root / "reference.py")], ref)
        if ref.returncode != 0:
            fail("reference oracle failed")
        expected = [
            "CANONICAL PASS: no bounded reference invariant violation through depth 4",
            "NEGATIVE PASS: accumulate -> ScaleDoesNotIncreasePoliticalWeight counterexample",
            "NEGATIVE PASS: gatekeep -> AuthorityHasExplicitSource counterexample",
            "NEGATIVE PASS: acquire -> JurisdictionHasExplicitSource counterexample",
            "NEGATIVE PASS: critical -> JurisdictionHasExplicitSource counterexample",
            "BOUNDED CONCENTRATION REFERENCE EXPLORATION PASS: smoke evidence only",
        ]
        for marker in expected:
            if marker not in ref.stdout:
                fail("reference oracle missing expected output: " + marker)

        cfg = model_root / "model.cfg"
        tla_cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(cfg), str(model_root / "model.tla")]
        tla = run(tla_cmd)
        receipt["canonical"]["tla"] = record(evidence, "tla-canonical", tla_cmd, tla)
        if tla.returncode != 0 or CANONICAL_OK not in tla.stdout:
            fail("canonical TLA+ did not complete cleanly")
        for invariant in TLA_INVARIANTS:
            if f"Error: Invariant {invariant} is violated." in tla.stdout:
                fail("canonical TLA+ invariant violated: " + invariant)

        negative_dir = evidence / "tla-negative"
        negative_dir.mkdir()
        negative_source = (model_root / "negative.tla").read_text(encoding="utf-8")
        for control, target in TLA_NEGATIVE.items():
            work = negative_dir / control
            work.mkdir()
            wrapper = work / "ArtificialSovereigntyConcentrationV1NegativeControls.tla"
            canonical = work / "ArtificialSovereigntyConcentrationV1.tla"
            config = work / "negative.cfg"
            wrapper.write_text(negative_source, encoding="utf-8")
            canonical.write_text((model_root / "model.tla").read_text(encoding="utf-8"), encoding="utf-8")
            cfg_text = (
                "CONSTANTS\n"
                "S1 = S1\nS2 = S2\n"
                "Compute = Compute\nCloud = Cloud\nData = Data\nEnergy = Energy\n"
                "J1 = J1\nJ2 = J2\nPowerA = PowerA\nPowerB = PowerB\n"
                "MaxSwitchingCost = 4\nReviewThreshold = 2\nMaxTime = 6\n"
                f'Control = "{control}"\n\n'
                "INIT Init\nNEXT NegativeNext\nINVARIANTS\n" +
                "\n".join(TLA_INVARIANTS) + "\nCHECK_DEADLOCK FALSE\n"
            )
            config.write_text(cfg_text, encoding="utf-8")
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
                fail(f"TLA negative control failed: {control}")

        alloy_cmd = ["java", "-cp", f"{args.alloy_class_dir}:{args.alloy_jar}",
                     "SovereigntyConcentrationAlloyQualificationRunner", str(model_root / "model.als")]
        alloy = run(alloy_cmd)
        receipt["canonical"]["alloy"] = record(evidence, "alloy-canonical", alloy_cmd, alloy)
        if alloy.returncode != 0:
            fail("canonical Alloy runner failed")
        rows = alloy_outcomes(alloy.stdout)
        by_label = {row["label"]: row for row in rows}
        if set(by_label) != ALLOY_SAT | ALLOY_UNSAT:
            fail("Alloy command label set mismatch")
        for label in ALLOY_SAT:
            if by_label[label]["actual"] != "SAT":
                fail("Alloy expected SAT failed: " + label)
        for label in ALLOY_UNSAT:
            if by_label[label]["actual"] != "UNSAT" or by_label[label]["expects"] != 0:
                fail("Alloy expected UNSAT failed: " + label)
        scope = alloy_meta["scope"]
        for label, row in by_label.items():
            cmd_text = row["command"]
            if f"for {scope['overall']}" not in cmd_text or f"{scope['int']} int" not in cmd_text:
                fail("Alloy scope contract mismatch: " + label)
            for name in ("Subject","Resource","Power","Jurisdiction","Acquisition","Gatekeeping"):
                token = f"{scope[name]} {name}"
                if token not in cmd_text:
                    fail("Alloy atom scope contract mismatch: " + token)
        receipt["canonical"]["alloy"]["commands"] = rows
        receipt["canonical"]["alloy"]["solver"] = "SAT4J"
        receipt["canonical"]["alloy"]["bitwidth"] = profile["models"]["alloy"]["bitwidth"]

        after = {
            "profile": sha256_file(args.profile),
            "pins": sha256_file(args.pins),
            "workflow": hashlib.sha256(Path(".github/workflows/sovereignty-concentration-formal-candidate.yml").read_bytes()).hexdigest(),
            "alloy_runner": sha256_file(args.alloy_runner),
        }
        if after != baseline:
            fail("verifier inputs changed during qualification")
        if subprocess.run(["git", "status", "--porcelain"], text=True, stdout=subprocess.PIPE).stdout.strip():
            fail("repository changed during qualification")

        receipt["result"] = "CandidateQualifiedExactHead"
        receipt["postflight"] = {"inputs_unchanged": True, "git_status_clean": True, "baseline": baseline}
    except Exception as exc:
        receipt["failure"] = str(exc)
        (evidence / "sovereignty-concentration-formal-receipt-v1.json").write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(receipt, sort_keys=True))
        print(str(exc), file=sys.stderr)
        return 1

    (evidence / "sovereignty-concentration-formal-receipt-v1.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
