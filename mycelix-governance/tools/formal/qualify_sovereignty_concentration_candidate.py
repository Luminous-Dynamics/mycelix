#!/usr/bin/env python3
"""Candidate-only formal execution/classification for sovereignty concentration v1."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

TLA_OK = "Model checking completed. No error has been found."
TLA_CONTROLS = {
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

def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def git_blob(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()

def run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

def emit(path: Path, data: object) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")

def alloy_rows(output: str) -> list[dict]:
    rows = []
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            rows.append(json.loads(line))
    return rows

def fail(message: str) -> None:
    raise RuntimeError("CONCENTRATION_CANDIDATE_FAIL: " + message)

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--tla", type=Path, required=True)
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--negative-tla", type=Path, required=True)
    parser.add_argument("--alloy", type=Path, required=True)
    parser.add_argument("--alloy-runner-class-dir", type=Path, required=True)
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--alloy-jar", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    args = parser.parse_args()

    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    if profile["status"] != "candidate-verification-profile" or profile["authority"] != "non-authoritative":
        fail("profile is not an explicitly non-authoritative candidate profile")
    if profile["profile_id"] != "SOV-AI-CONCENTRATION-FORMAL-QUAL-001-V1":
        fail("unexpected concentration profile id")

    model = profile["models"]
    for path, expected in [
        (args.tla, model["tla"]["git_blob_sha"]),
        (args.cfg, model["tla"]["config_git_blob_sha"]),
        (args.negative_tla, model["negative_tla"]["git_blob_sha"]),
        (args.alloy, model["alloy"]["git_blob_sha"]),
        (Path("mycelix-governance/tools/formal/sovereignty_concentration_reference_explorer.py"), model["reference"]["git_blob_sha"]),
    ]:
        if git_blob(path.read_bytes()) != expected:
            fail(f"candidate formal byte mismatch: {path}")

    pins = {
        "tla2tools": {
            "version": "1.7.4",
            "sha256": "936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88",
        },
        "alloy": {
            "version": "6.2.0",
            "sha256": "6b8c1cb5bc93bedfc7c61435c4e1ab6e688a242dc702a394628d9a9801edb78d",
        },
    }
    if sha256(args.tla_jar) != pins["tla2tools"]["sha256"]:
        fail("TLA tool hash mismatch")
    if sha256(args.alloy_jar) != pins["alloy"]["sha256"]:
        fail("Alloy tool hash mismatch")

    baseline = {
        str(p): sha256(p)
        for p in [
            args.profile, args.tla, args.cfg, args.negative_tla, args.alloy
        ]
    }

    receipt = {
        "receipt_schema": "sovereignty-concentration-candidate-receipt-v1",
        "result": "CandidateExecuted",
        "claim_scope": "candidate bounded formal execution/classification only",
        "subject": profile["semantic_subject"],
        "model_blobs": {
            "tla": model["tla"]["git_blob_sha"],
            "cfg": model["tla"]["config_git_blob_sha"],
            "negative_tla": model["negative_tla"]["git_blob_sha"],
            "alloy": model["alloy"]["git_blob_sha"],
            "reference": model["reference"]["git_blob_sha"],
        },
        "tools": pins,
        "tla": {"canonical": {}, "negative_controls": {}},
        "alloy": {"canonical": {}, "scope": profile["models"]["alloy"]["scope"]},
        "reference": {},
        "nonclaims": profile["nonclaims"],
    }

    try:
        reference = run([sys.executable, str(Path("mycelix-governance/tools/formal/sovereignty_concentration_reference_explorer.py"))])
        (args.evidence_dir / "reference.log").write_text(reference.stdout, encoding="utf-8")
        receipt["reference"] = {"returncode": reference.returncode, "log_sha256": sha256(args.evidence_dir / "reference.log")}
        if reference.returncode != 0:
            fail("reference explorer failed")
        for control, target in [
            ("accumulate", "ScaleDoesNotIncreasePoliticalWeight"),
            ("gatekeep", "AuthorityHasExplicitSource"),
            ("acquire", "JurisdictionHasExplicitSource"),
            ("critical", "JurisdictionHasExplicitSource"),
        ]:
            marker = f"NEGATIVE PASS: {control} -> {target} counterexample"
            if marker not in reference.stdout:
                fail(f"reference control missing: {marker}")

        tla_cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(args.cfg), str(args.tla)]
        tla = run(tla_cmd)
        (args.evidence_dir / "tla-canonical.log").write_text(tla.stdout, encoding="utf-8")
        receipt["tla"]["canonical"] = {"command": tla_cmd, "returncode": tla.returncode,
                                       "log_sha256": sha256(args.evidence_dir / "tla-canonical.log")}
        if tla.returncode != 0 or TLA_OK not in tla.stdout:
            fail("canonical TLA run did not terminate cleanly")

        negative_source = args.negative_tla.read_text(encoding="utf-8")
        canonical_source = args.tla.read_text(encoding="utf-8")
        for control, target in TLA_CONTROLS.items():
            work = args.evidence_dir / "tla-negative" / control
            work.mkdir(parents=True)
            wrapper = work / args.negative_tla.name
            canonical = work / args.tla.name
            cfg = work / "negative.cfg"
            wrapper.write_text(negative_source, encoding="utf-8")
            canonical.write_text(canonical_source, encoding="utf-8")
            cfg.write_text(
                "CONSTANTS\n"
                "S1 = S1\nS2 = S2\nCompute = Compute\nCloud = Cloud\nData = Data\nEnergy = Energy\n"
                "J1 = J1\nJ2 = J2\nPowerA = PowerA\nPowerB = PowerB\n"
                "MaxSwitchingCost = 4\nReviewThreshold = 2\nMaxTime = 6\n"
                f'Control = "{control}"\n\nINIT Init\nNEXT NegativeNext\nINVARIANTS\n'
                + "\n".join(model["tla"]["invariants"]) + "\nCHECK_DEADLOCK FALSE\n",
                encoding="utf-8",
            )
            cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(cfg), str(wrapper)]
            result = run(cmd)
            logfile = args.evidence_dir / f"tla-negative-{control}.log"
            logfile.write_text(result.stdout, encoding="utf-8")
            receipt["tla"]["negative_controls"][control] = {
                "target_invariant": target,
                "command": cmd,
                "returncode": result.returncode,
                "log_sha256": sha256(logfile),
            }
            if result.returncode == 0 or f"Error: Invariant {target} is violated." not in result.stdout:
                fail(f"TLA negative control failed: {control} -> {target}")

        alloy_cp = f"{args.alloy_runner_class_dir}:{args.alloy_jar}"
        alloy_cmd = ["java", "-cp", alloy_cp, "SovereigntyConcentrationAlloyRunner", str(args.alloy)]
        alloy = run(alloy_cmd)
        (args.evidence_dir / "alloy.log").write_text(alloy.stdout, encoding="utf-8")
        receipt["alloy"]["execution"] = {"command": alloy_cmd, "returncode": alloy.returncode,
                                         "log_sha256": sha256(args.evidence_dir / "alloy.log")}
        if alloy.returncode != 0:
            fail("Alloy runner failed")
        rows = alloy_rows(alloy.stdout)
        labels = {r["label"] for r in rows}
        if labels != ALLOY_SAT | ALLOY_UNSAT:
            fail("Alloy command label set mismatch")
        for row in rows:
            if f"{profile['models']['alloy']['bitwidth']} int" not in row["command"]:
                fail("Alloy integer bitwidth missing from command: " + row["label"])
            if f"for {profile['models']['alloy']['scope']['overall']}" not in row["command"]:
                fail("Alloy overall scope missing from command: " + row["label"])
        by = {r["label"]: r for r in rows}
        for label in ALLOY_SAT:
            if by[label]["actual"] != "SAT" or by[label]["check"]:
                fail("Alloy expected-SAT mismatch: " + label)
        for label in ALLOY_UNSAT:
            if by[label]["actual"] != "UNSAT" or not by[label]["check"] or by[label]["expects"] != 0:
                fail("Alloy expected-UNSAT mismatch: " + label)
        receipt["alloy"]["commands"] = rows

        if any(sha256(Path(p)) != digest for p, digest in baseline.items()):
            fail("candidate inputs changed during execution")

        receipt["postflight"] = {"inputs_unchanged": True}
        emit(args.evidence_dir / "sovereignty-concentration-candidate-receipt-v1.json", receipt)
        print(json.dumps(receipt, indent=2, sort_keys=True))
        return 0

    except Exception as exc:
        receipt["failure"] = str(exc)
        emit(args.evidence_dir / "sovereignty-concentration-candidate-receipt-v1.json", receipt)
        print(json.dumps(receipt, indent=2, sort_keys=True))
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
