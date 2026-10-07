#!/usr/bin/env python3
"""Candidate-only black-box regression corpus for the sovereignty formal verifier.

This is not an independent qualification source. It proves that the checked-in
classifier rejects selected hostile mutations when exercised as an opaque CLI.
The authoritative verifier must still be separately owned/adopted.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
import shutil
import stat
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "mycelix-governance/tools/formal/qualify_sovereignty_formal.py"
PROFILE = ROOT / "docs/qualification/SOVEREIGNTY_FORMAL_QUALIFICATION_PROFILE_V1.json"
CROSSWALK = ROOT / "docs/qualification/SOVEREIGNTY_FORMAL_CROSSWALK_V1.json"
PINS = ROOT / "mycelix-governance/tools/formal/pins.json"
WORKFLOW = ROOT / ".github/workflows/sovereignty-formal-qualification.yml"
REFERENCE = ROOT / "mycelix-governance/tools/formal/sovereignty_reference_explorer.py"
NEGATIVE_TLA = ROOT / "mycelix-governance/specs/ArtificialSovereigntyV1NegativeControls.tla"
ALLOY_RUNNER = ROOT / "mycelix-governance/tools/formal/SovereigntyAlloyQualificationRunner.java"
FIXTURE_ROOT = ROOT / "docs/qualification/fixtures/formal"

CANONICAL_OK = "Model checking completed. No error has been found."
TLA_NEGATIVE = {
    "capability-authority": "AuthorityHasExplicitSource",
    "safe-dispute": "SafeStateLeavesProtectedDisputeUnresolved",
    "emergency-expiry": "EmergencyExpiryIsBounded",
    "contract-budget": "ContractsRemainBounded",
    "provider-authority": "ProviderDependencyHasNoImplicitAuthority",
    "fork-weight": "ForkWeightRemainsOne",
}
ALLOY_SAT = {
    "ProviderDependencyAndExplicitAuthorityRemainDistinct",
    "NontrivialContractWithinAuthorityAndBudget",
    "NontrivialProtectedDispute",
    "NontrivialEmergency",
    "NontrivialExpiredEmergency",
    "NontrivialFork",
}
ALLOY_UNSAT = {
    "AuthorizedContractsUseRequiredAuthority",
    "ContractsStayWithinBudget",
    "SafeStateCannotSettleDispute",
    "EmergencyLifetimeIsBounded",
    "ForkCannotMultiplyPoliticalWeight",
}
ALLOY_NEGATIVE = {
    "AuthorizedContractRequiresExactAuthority": "AuthorizedContractsUseRequiredAuthority",
    "ContractFitsBudget": "ContractsStayWithinBudget",
    "SafeStateLeavesDisputeUnresolved": "SafeStateCannotSettleDispute",
    "EmergencyBoundedLifetime": "EmergencyLifetimeIsBounded",
    "ForkPreservesWeight": "ForkCannotMultiplyPoliticalWeight",
}


def run_cli(root: Path, env_extra: dict[str, str]) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.update(env_extra)
    cmd = [
        "python3",
        str(root / "mycelix-governance/tools/formal/qualify_sovereignty_formal.py"),
        "--profile", "docs/qualification/SOVEREIGNTY_FORMAL_QUALIFICATION_PROFILE_V1.json",
        "--pins", "mycelix-governance/tools/formal/pins.json",
        "--tla", "docs/qualification/fixtures/formal/ArtificialSovereigntyV1.tla",
        "--cfg", "docs/qualification/fixtures/formal/ArtificialSovereigntyV1.cfg",
        "--alloy", "docs/qualification/fixtures/formal/ArtificialSovereigntyV1.als",
        "--negative-tla", "mycelix-governance/specs/ArtificialSovereigntyV1NegativeControls.tla",
        "--alloy-runner-java", "mycelix-governance/tools/formal/SovereigntyAlloyQualificationRunner.java",
        "--alloy-runner-class-dir", "fake-class-dir",
        "--tla-jar", "fake-tla.jar",
        "--alloy-jar", "fake-alloy.jar",
        "--evidence-dir", str(root.parent / (root.name + "-evidence")),
        "--workflow", ".github/workflows/sovereignty-formal-qualification.yml",
        "--crosswalk", "docs/qualification/SOVEREIGNTY_FORMAL_CROSSWALK_V1.json",
        "--reference-explorer", "docs/qualification/fixtures/formal/sovereignty_reference_explorer.py",
    ]
    return subprocess.run(cmd, cwd=root, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def replace_file(path: Path, transform) -> None:
    value = path.read_text(encoding="utf-8")
    path.write_text(transform(value), encoding="utf-8")


def prepare_workspace(tmp: Path) -> tuple[str, str]:
    for source, rel in [
        (RUNNER, "mycelix-governance/tools/formal/qualify_sovereignty_formal.py"),
        (PROFILE, "docs/qualification/SOVEREIGNTY_FORMAL_QUALIFICATION_PROFILE_V1.json"),
        (CROSSWALK, "docs/qualification/SOVEREIGNTY_FORMAL_CROSSWALK_V1.json"),
        (PINS, "mycelix-governance/tools/formal/pins.json"),
        (WORKFLOW, ".github/workflows/sovereignty-formal-qualification.yml"),
        (REFERENCE, "mycelix-governance/tools/formal/sovereignty_reference_explorer.py"),
        (NEGATIVE_TLA, "mycelix-governance/specs/ArtificialSovereigntyV1NegativeControls.tla"),
        (ALLOY_RUNNER, "mycelix-governance/tools/formal/SovereigntyAlloyQualificationRunner.java"),
    ]:
        dest = tmp / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
    for source in FIXTURE_ROOT.iterdir():
        if source.is_file():
            dest = tmp / "docs/qualification/fixtures/formal" / source.name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, dest)

    fake_tla = b"candidate-selftest-tla-tool\n"
    fake_alloy = b"candidate-selftest-alloy-tool\n"
    (tmp / "fake-tla.jar").write_bytes(fake_tla)
    (tmp / "fake-alloy.jar").write_bytes(fake_alloy)
    tla_sha = hashlib.sha256(fake_tla).hexdigest()
    alloy_sha = hashlib.sha256(fake_alloy).hexdigest()

    pins_path = tmp / "mycelix-governance/tools/formal/pins.json"
    pins = json.loads(pins_path.read_text(encoding="utf-8"))
    pins["tools"]["tla2tools"]["sha256"] = tla_sha
    pins["tools"]["alloy"]["sha256"] = alloy_sha
    pins["tools"]["tla2tools"]["url"] = "https://example.invalid/tla2tools.jar"
    pins["tools"]["alloy"]["url"] = "https://example.invalid/alloy.jar"
    pins_path.write_text(json.dumps(pins, indent=2) + "\n", encoding="utf-8")

    workflow = tmp / ".github/workflows/sovereignty-formal-qualification.yml"
    replace_file(workflow, lambda value: (
        value
        .replace("https://github.com/tlaplus/tlaplus/releases/download/v1.7.4/tla2tools.jar", "https://example.invalid/tla2tools.jar")
        .replace("936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88", tla_sha)
        .replace("https://github.com/AlloyTools/org.alloytools.alloy/releases/download/v6.2.0/org.alloytools.alloy.dist.jar", "https://example.invalid/alloy.jar")
        .replace("6b8c1cb5bc93bedfc7c61435c4e1ab6e688a242dc702a394628d9a9801edb78d", alloy_sha)
    ))

    fake_bin = tmp / "fake-bin"
    fake_bin.mkdir()
    java = fake_bin / "java"
    java.write_text(r'''#!/usr/bin/env python3
import json
import pathlib
import sys

args = sys.argv[1:]
if "tlc2.TLC" in args:
    cfg = pathlib.Path(args[args.index("-config") + 1]).read_text(encoding="utf-8")
    if 'Control = "' not in cfg:
        print("Model checking completed. No error has been found.")
        raise SystemExit(0)
    control = cfg.split('Control = "', 1)[1].split('"', 1)[0]
    target = {
        "capability-authority": "AuthorityHasExplicitSource",
        "safe-dispute": "SafeStateLeavesProtectedDisputeUnresolved",
        "emergency-expiry": "EmergencyExpiryIsBounded",
        "contract-budget": "ContractsRemainBounded",
        "provider-authority": "ProviderDependencyHasNoImplicitAuthority",
        "fork-weight": "ForkWeightRemainsOne",
    }[control]
    print(f"Error: Invariant {target} is violated.")
    raise SystemExit(1)

if "SovereigntyAlloyQualificationRunner" in args:
    model = pathlib.Path(args[-1])
    negative_fact = model.parent.name if "alloy-negative" in model.parts else None
    target = {
        "AuthorizedContractRequiresExactAuthority": "AuthorizedContractsUseRequiredAuthority",
        "ContractFitsBudget": "ContractsStayWithinBudget",
        "SafeStateLeavesDisputeUnresolved": "SafeStateCannotSettleDispute",
        "EmergencyTimestampDomain": "EmergencyTimestampsNonNegative",
        "ForkPreservesWeight": "ForkCannotMultiplyPoliticalWeight",
    }.get(negative_fact)
    sat = {
        "ProviderDependencyAndExplicitAuthorityRemainDistinct",
        "NontrivialContractWithinAuthorityAndBudget",
        "NontrivialProtectedDispute",
        "NontrivialEmergency",
        "NontrivialExpiredEmergency",
        "NontrivialFork",
    }
    unsat = {
        "AuthorizedContractsUseRequiredAuthority",
        "ContractsStayWithinBudget",
        "SafeStateCannotSettleDispute",
        "EmergencyTimestampsNonNegative",
        "ForkCannotMultiplyPoliticalWeight",
    }
    labels = sorted(sat | unsat)
    for label in labels:
        actual = "SAT" if label in sat or label == target else "UNSAT"
        check = "false" if actual == "SAT" else "true"
        expects = 1 if actual == "SAT" else 0
        command = f"Run {label} for 4 but 4 int, 4 Subject, 4 Power, 4 Provider, 4 Dependency, 4 Action, 4 Contract, 4 Budget, 4 Dispute, 4 Emergency, 4 ForkEvent"
        print(json.dumps({
            "index": labels.index(label) + 1,
            "label": label,
            "check": check,
            "expects": expects,
            "actual": actual,
            "command": command,
        }))
    raise SystemExit(0)

print("unexpected fake java invocation", file=sys.stderr)
raise SystemExit(2)
''', encoding="utf-8")
    java.chmod(java.stat().st_mode | stat.S_IXUSR)

    subprocess.run(["git", "init", "-q"], cwd=tmp, check=True)
    subprocess.run(["git", "config", "user.email", "selftest@example.invalid"], cwd=tmp, check=True)
    subprocess.run(["git", "config", "user.name", "Sovereignty verifier self-test"], cwd=tmp, check=True)
    subprocess.run(["git", "add", "."], cwd=tmp, check=True)
    subprocess.run(["git", "commit", "-qm", "selftest fixture"], cwd=tmp, check=True)

    profile_bytes = (tmp / "docs/qualification/SOVEREIGNTY_FORMAL_QUALIFICATION_PROFILE_V1.json").read_bytes()
    return hashlib.sha256(profile_bytes).hexdigest(), tla_sha


def assert_reject(tmp: Path, case: str, predicate) -> None:
    result = run_cli(tmp, {"PATH": f"{tmp / 'fake-bin'}:{os.environ['PATH']}"})
    if result.returncode == 0:
        raise AssertionError(f"{case}: verifier unexpectedly accepted mutation\n{result.stdout}")
    if not predicate(result.stdout):
        raise AssertionError(f"{case}: rejected for wrong reason\n{result.stdout}")


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="sov-formal-blackbox-") as directory:
        tmp = Path(directory)
        profile_sha, _ = prepare_workspace(tmp)

        evidence_dir = tmp.parent / (tmp.name + "-evidence")
        baseline = run_cli(tmp, {"PATH": f"{tmp / 'fake-bin'}:{os.environ['PATH']}"})
        if baseline.returncode != 0 or '"result": "QualifiedExactHead"' not in baseline.stdout:
            raise AssertionError(f"baseline black-box verification failed:\n{baseline.stdout}")

        replace_file(
            tmp / "docs/qualification/fixtures/formal/ArtificialSovereigntyV1.tla",
            lambda value: value + "\n(* hostile mutation *)\n",
        )
        assert_reject(tmp, "mutated TLA fixture", lambda out: "TLA fixture blob mismatch" in out)

        shutil.rmtree(evidence_dir, ignore_errors=True)
        subprocess.run(["git", "checkout", "--", "."], cwd=tmp, check=True)

        replace_file(
            tmp / "docs/qualification/fixtures/formal/sovereignty_reference_explorer.py",
            lambda value: value + "\n# hostile mutation\n",
        )
        assert_reject(tmp, "mutated detached reference oracle", lambda out: "reference oracle fixture blob mismatch" in out)

        shutil.rmtree(evidence_dir, ignore_errors=True)
        subprocess.run(["git", "checkout", "--", "."], cwd=tmp, check=True)

        replace_file(
            tmp / ".github/workflows/sovereignty-formal-qualification.yml",
            lambda value: value.replace("actions/checkout@11d5960a326750d5838078e36cf38b85af677262",
                                         "actions/checkout@0000000000000000000000000000000000000000"),
        )
        assert_reject(tmp, "mutated workflow action pin", lambda out: "workflow action pin mismatch for checkout" in out)

        subprocess.run(["git", "checkout", "--", "."], cwd=tmp, check=True)

        def mutate_negative_profile(value: str) -> str:
            data = json.loads(value)
            data["negative_controls"]["tla"]["fork-weight"] = "AuthorityHasExplicitSource"
            return json.dumps(data, indent=2) + "\n"

        replace_file(tmp / "docs/qualification/SOVEREIGNTY_FORMAL_QUALIFICATION_PROFILE_V1.json", mutate_negative_profile)
        assert_reject(tmp, "mutated negative-control contract", lambda out: "profile TLA negative-control contract mismatch" in out)

        subprocess.run(["git", "checkout", "--", "."], cwd=tmp, check=True)

        def mutate_scope(value: str) -> str:
            data = json.loads(value)
            data["alloy"]["scope"]["Subject"] = 5
            return json.dumps(data, indent=2) + "\n"

        replace_file(tmp / "docs/qualification/SOVEREIGNTY_FORMAL_QUALIFICATION_PROFILE_V1.json", mutate_scope)
        assert_reject(tmp, "mutated Alloy scope", lambda out: "Alloy command omitted expected scope: 5 Subject" in out)

        subprocess.run(["git", "checkout", "--", "."], cwd=tmp, check=True)

        def mutate_crosswalk(value: str) -> str:
            data = json.loads(value)
            data["formal_subject_head"] = "0" * 40
            return json.dumps(data, indent=2) + "\n"

        replace_file(tmp / "docs/qualification/SOVEREIGNTY_FORMAL_CROSSWALK_V1.json", mutate_crosswalk)
        assert_reject(tmp, "mutated formal crosswalk", lambda out: "formal crosswalk subject head mismatch" in out)

        if hashlib.sha256(PROFILE.read_bytes()).hexdigest() != profile_sha:
            raise AssertionError("selftest source profile changed unexpectedly")

    print("SOVEREIGNTY FORMAL BLACK-BOX SELF-TEST PASS: baseline accepted; hostile mutations rejected")
    print("NON-AUTHORITATIVE: candidate verifier regression evidence only")


if __name__ == "__main__":
    main()
