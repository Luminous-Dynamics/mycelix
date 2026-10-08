#!/usr/bin/env python3
"""Candidate-only regression for the exact subject-tree fail-closed gate."""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "mycelix-governance/tools/formal/qualify_effective_contestability.py"
PROFILE = ROOT / "docs/qualification/SOVEREIGNTY_CONTESTABILITY_FORMAL_QUALIFICATION_PROFILE_V1.json"
PINS = ROOT / "mycelix-governance/tools/formal/effective_contestability_pins.json"
CROSSWALK = ROOT / "docs/qualification/SOVEREIGNTY_CONTESTABILITY_FORMAL_CROSSWALK_V1.json"

def main() -> int:
    with tempfile.TemporaryDirectory(prefix="contestability-tree-gate-") as d:
        tmp = Path(d)
        work = tmp / "repo"
        work.mkdir()
        runner = work / "qualify_effective_contestability.py"
        shutil.copy2(RUNNER, runner)
        for src in (PROFILE, PINS, CROSSWALK):
            shutil.copy2(src, work / src.name)
        runtime = {
            "schema": "mycelix.effective-contestability-formal-runtime.v1",
            "nixpkgs_rev": "a50bf0c1b07873c1a53892292017041a8f0a1288",
            "jdk_package": "jdk17_headless",
            "java_major": 17,
            "java_version": 'openjdk version "17.0.99"',
            "javac_version": "javac 17.0.99",
        }
        (work / "runtime.json").write_text(json.dumps(runtime) + "\n", encoding="utf-8")
        args = [
            "python3", str(runner),
            "--profile", str(work / PROFILE.name),
            "--pins", str(work / PINS.name),
            "--tla", str(tmp / "tla"),
            "--cfg", str(tmp / "cfg"),
            "--negative-tla", str(tmp / "negative"),
            "--alloy", str(tmp / "alloy"),
            "--alloy-runner-java", str(tmp / "runner.java"),
            "--alloy-runner-class", str(tmp / "runner.class"),
            "--alloy-runner-class-dir", str(tmp),
            "--tla-jar", str(tmp / "tla.jar"),
            "--alloy-jar", str(tmp / "alloy.jar"),
            "--evidence-dir", str(tmp / "evidence"),
            "--workflow", str(tmp / ".github/workflows/sovereignty-contestability-formal-candidate.yml"),
            "--crosswalk", str(work / CROSSWALK.name),
            "--reference-explorer", str(tmp / "reference.py"),
            "--runtime-metadata", str(work / "runtime.json"),
        ]
        result = subprocess.run(args, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode != 2:
            raise AssertionError(f"expected exact-tree blocker exit 2, got {result.returncode}:\n{result.stdout}")
        if "BlockedMissingExactSubjectTree" not in result.stdout:
            raise AssertionError(f"missing fail-closed blocker marker:\n{result.stdout}")
    print("EFFECTIVE CONTESTABILITY TREE-GATE SELF-TEST PASS: missing exact tree blocks qualification")
    print("NON-AUTHORITATIVE: candidate verifier regression evidence only")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
