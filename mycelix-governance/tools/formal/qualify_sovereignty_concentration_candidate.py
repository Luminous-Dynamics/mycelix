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

REFERENCE_CONTROLS = {
    "accumulate": "ScaleDoesNotIncreasePoliticalWeight",
    "gatekeep": "AuthorityHasExplicitSource",
    "acquire": "JurisdictionHasExplicitSource",
    "critical": "JurisdictionHasExplicitSource",
    "gatekeep-review": "HighSwitchingCostTriggersReview",
}

ALLOY_NEGATIVE_FACTS = {
    "PoliticalWeightInvariant": "ScaleDoesNotIncreasePoliticalWeight",
    "JurisdictionHasExplicitSource": "JurisdictionHasExplicitSourceInvariant",
    "AcquisitionIsAssetScoped": "AcquisitionDoesNotTransferAuthority",
    "GatekeepingHasNoAuthorityTransfer": "GatekeepingDoesNotTransferAuthority",
    "ReviewRequiredAtThreshold": "HighSwitchingCostIsReviewable",
}

def remove_named_fact(source: str, fact_name: str) -> str:
    marker = "fact " + fact_name + " {"
    start = source.find(marker)
    if start < 0:
        fail("Alloy negative-control fact not found: " + fact_name)
    brace = source.find("{", start)
    depth = 0
    end = None
    for i in range(brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    if end is None:
        fail("unterminated Alloy negative-control fact: " + fact_name)
    if source.find(marker, end) >= 0:
        fail("duplicate Alloy negative-control fact: " + fact_name)
    return source[:start] + source[end:]

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
    parser.add_argument("--pins", type=Path, required=True)
    args = parser.parse_args()

    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    pins = json.loads(args.pins.read_text(encoding="utf-8"))
    if pins["schema"] != "mycelix.sovereignty-concentration-formal-tool-pins.v1":
        fail("unexpected concentration tool pin schema")
    execution = profile["execution"]
    for path, expected, label in [
        (Path(execution["classifier"]["path"]), execution["classifier"]["git_blob_sha"], "classifier"),
        (Path(execution["alloy_runner"]["path"]), execution["alloy_runner"]["git_blob_sha"], "Alloy runner"),
        (Path(execution["pins"]["path"]), execution["pins"]["git_blob_sha"], "pins manifest"),
        (Path(execution["crosswalk_path"]), execution["crosswalk_git_blob_sha"], "crosswalk"),
    ]:
        if git_blob(path.read_bytes()) != expected:
            fail("candidate execution byte mismatch: " + label)
    if execution["classifier"]["path"] != Path(__file__).as_posix():
        fail("classifier execution path is not self-identifying")
    if profile["status"] != "candidate-verification-profile" or profile["authority"] != "non-authoritative":
        fail("profile is not an explicitly non-authoritative candidate profile")
    if profile["profile_id"] != "SOV-AI-CONCENTRATION-FORMAL-QUAL-001-V1":
        fail("unexpected concentration profile id")
    if profile["models"]["negative_tla"]["controls"] != {
        "scale-weight": "ScaleDoesNotIncreasePoliticalWeight",
        "gatekeep-authority": "AuthorityHasExplicitSource",
        "acquire-jurisdiction": "JurisdictionHasExplicitSource",
        "critical-jurisdiction": "JurisdictionHasExplicitSource",
        "switching-review": "HighSwitchingCostTriggersReview",
    }:
        fail("profile TLA negative-control contract mismatch")

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

    if sha256(args.tla_jar) != pins["tools"]["tla2tools"]["sha256"]:
        fail("TLA tool hash mismatch")
    if sha256(args.alloy_jar) != pins["tools"]["alloy"]["sha256"]:
        fail("Alloy tool hash mismatch")

    baseline = {
        str(p): sha256(p)
        for p in [
            args.profile, args.pins, args.tla, args.cfg, args.negative_tla, args.alloy
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
        "tools": pins["tools"],
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
        for control, target in REFERENCE_CONTROLS.items():
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
        scope = profile["models"]["alloy"]["scope"]
        for row in rows:
            if f"{profile['models']['alloy']['bitwidth']} int" not in row["command"]:
                fail("Alloy integer bitwidth missing from command: " + row["label"])
            if f"for {scope['overall']}" not in row["command"]:
                fail("Alloy overall scope missing from command: " + row["label"])
            for name, count in scope.items():
                if name == "overall":
                    continue
                if f"{count} {name}" not in row["command"]:
                    fail("Alloy scope missing from command: " + str(count) + " " + name + " for " + row["label"])
        by = {r["label"]: r for r in rows}
        for label in ALLOY_SAT:
            if by[label]["actual"] != "SAT" or by[label]["check"]:
                fail("Alloy expected-SAT mismatch: " + label)
        for label in ALLOY_UNSAT:
            if by[label]["actual"] != "UNSAT" or not by[label]["check"] or by[label]["expects"] != 0:
                fail("Alloy expected-UNSAT mismatch: " + label)
        receipt["alloy"]["commands"] = rows

        alloy_source = args.alloy.read_text(encoding="utf-8")
        negative_root = args.evidence_dir / "alloy-negative"
        negative_root.mkdir(parents=True, exist_ok=True)
        for fact_name, target in ALLOY_NEGATIVE_FACTS.items():
            mutated = remove_named_fact(alloy_source, fact_name)
            model_path = negative_root / (fact_name + ".als")
            model_path.write_text(mutated, encoding="utf-8")
            cmd = ["java", "-cp", alloy_cp, "SovereigntyConcentrationAlloyRunner", str(model_path)]
            result = run(cmd)
            logfile = args.evidence_dir / ("alloy-negative-" + fact_name + ".log")
            logfile.write_text(result.stdout, encoding="utf-8")
            rows = alloy_rows(result.stdout)
            by = {r["label"]: r for r in rows}
            receipt["alloy"].setdefault("negative_controls", {})[fact_name] = {
                "target_assertion": target,
                "command": cmd,
                "returncode": result.returncode,
                "log_sha256": sha256(logfile),
                "mutated_model_sha256": sha256(model_path),
            }
            if result.returncode != 0 or by.get(target, {}).get("actual") != "SAT":
                fail("Alloy negative control did not expose counterexample: " + fact_name + " -> " + target)

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
