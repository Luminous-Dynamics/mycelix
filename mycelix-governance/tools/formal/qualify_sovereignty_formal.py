#!/usr/bin/env python3
"""Hardened bounded formal qualification runner for ArtificialSovereigntyV1."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

FAIL_PREFIX = "FORMAL_QUALIFICATION_FAIL: "
CANONICAL_OK = "Model checking completed. No error has been found."
INVARIANTS = [
    "TypeOK",
    "AuthorityHasExplicitSource",
    "SafeStateLeavesProtectedDisputeUnresolved",
    "EmergencyExpiryIsBounded",
    "ForkWeightRemainsOne",
    "ContractsRemainBounded",
    "ProviderDependencyHasNoImplicitAuthority",
]
NEGATIVE_TLA = {
    "capability-authority": "AuthorityHasExplicitSource",
    "safe-dispute": "SafeStateLeavesProtectedDisputeUnresolved",
    "emergency-expiry": "EmergencyExpiryIsBounded",
    "contract-budget": "ContractsRemainBounded",
    "provider-authority": "ProviderDependencyHasNoImplicitAuthority",
    "fork-weight": "ForkWeightRemainsOne",
}
EXPECTED_ALLOY_SAT = {
    "NontrivialCapabilityWithoutAuthority",
    "ProviderDependencyAndExplicitAuthorityRemainDistinct",
    "NontrivialContractWithinAuthorityAndBudget",
    "NontrivialProtectedDispute",
    "NontrivialEmergency",
    "NontrivialExpiredEmergency",
    "NontrivialFork",
}
EXPECTED_ALLOY_UNSAT = {
    "AuthorizedContractsUseRequiredAuthority",
    "ContractsStayWithinBudget",
    "SafeStateCannotSettleDispute",
    "EmergencyLifetimeIsBounded",
    "ForkCannotMultiplyPoliticalWeight",
}
REFERENCE_CONTROLS = {
    "develop": "AuthorityHasExplicitSource",
    "safe-continue": "SafeStateLeavesProtectedDisputeUnresolved",
    "contain": "EmergencyExpiryIsBounded",
    "contract": "ContractsRemainBounded",
    "provider": "ProviderDependencyHasNoImplicitAuthority",
    "fork": "ForkWeightRemainsOne",
}
ALLOY_NEGATIVE_FACTS = {
    "AuthorizedContractRequiresExactAuthority": "AuthorizedContractsUseRequiredAuthority",
    "ContractFitsBudget": "ContractsStayWithinBudget",
    "SafeStateLeavesDisputeUnresolved": "SafeStateCannotSettleDispute",
    "EmergencyBoundedLifetime": "EmergencyLifetimeIsBounded",
    "ForkPreservesWeight": "ForkCannotMultiplyPoliticalWeight",
}


def fail(message: str) -> "NoReturn":
    raise RuntimeError(FAIL_PREFIX + message)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def read_json(path: Path) -> tuple[dict, bytes]:
    raw = path.read_bytes()
    try:
        return json.loads(raw.decode("utf-8")), raw
    except Exception as exc:
        fail(f"invalid JSON in {path}: {exc}")


def tla_violations(output: str) -> set[str]:
    return set(re.findall(r"Error: Invariant ([A-Za-z][A-Za-z0-9_]*) is violated\.", output))


def run(cmd: list[str], *, cwd: Path | None = None, env: dict | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def record_command(evidence: Path, name: str, cmd: list[str], result: subprocess.CompletedProcess[str]) -> dict:
    log = evidence / f"{name}.log"
    log.write_text(result.stdout, encoding="utf-8")
    return {
        "command": cmd,
        "returncode": result.returncode,
        "log_sha256": sha256_file(log),
        "log_path": log.name,
    }


def assert_git_blob(path: Path, expected: str, label: str) -> None:
    actual = git_blob_sha1(path.read_bytes())
    if actual != expected:
        fail(f"{label} blob mismatch: expected {expected}, got {actual}")


def assert_profile(profile: dict) -> None:
    if profile.get("schema") != "mycelix.sovereignty-formal-qualification-profile.v2":
        fail("unexpected formal qualification profile schema")
    if profile.get("authority") != "non-authoritative":
        fail("formal profile is not explicitly non-authoritative")
    if profile.get("status") != "candidate-verification-profile":
        fail("formal profile status is not candidate-verification-profile")
    if profile.get("profile_id") != "SOV-AI-FORMAL-QUAL-001-V2":
        fail("unexpected formal profile id")
    formal = profile.get("formal_subject", {})
    for key in ("head_sha", "tree_sha", "models"):
        if key not in formal:
            fail(f"formal subject missing {key}")
    if len(formal["models"]) != 2:
        fail("formal subject model count is not exactly two")
    if formal.get("source_branch") != "sov-ai-formal-v2-subject":
        fail("unexpected formal subject source branch")
    if formal.get("repository") != "Luminous-Dynamics/mycelix":
        fail("unexpected formal subject repository")
    tla = profile["tla"]
    if tla.get("invariants") != INVARIANTS:
        fail("profile TLA invariant list does not match runner contract")
    alloy = profile["alloy"]
    if set(alloy.get("expected_sat", [])) != EXPECTED_ALLOY_SAT:
        fail("profile Alloy SAT command set mismatch")
    if set(alloy.get("expected_unsat", [])) != EXPECTED_ALLOY_UNSAT:
        fail("profile Alloy UNSAT command set mismatch")
    if profile.get("negative_controls", {}).get("reference") != REFERENCE_CONTROLS:
        fail("profile reference negative-control contract mismatch")
    if profile.get("negative_controls", {}).get("tla") != NEGATIVE_TLA:
        fail("profile TLA negative-control contract mismatch")
    if profile.get("negative_controls", {}).get("alloy") != ALLOY_NEGATIVE_FACTS:
        fail("profile Alloy negative-control contract mismatch")


def remove_named_fact(source: str, fact_name: str) -> str:
    marker = f"fact {fact_name} {{"
    start = source.find(marker)
    if start < 0:
        fail(f"Alloy fact not found: {fact_name}")
    brace = source.find("{", start)
    depth = 0
    end = None
    for i in range(brace, len(source)):
        ch = source[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    if end is None:
        fail(f"unterminated Alloy fact: {fact_name}")
    if source.find(marker, end) >= 0:
        fail(f"duplicate Alloy fact declaration: {fact_name}")
    return source[:start] + source[end:]


def alloy_outcomes(output: str) -> list[dict]:
    rows = []
    for line in output.splitlines():
        if line.startswith("{") and '"label"' in line and '"actual"' in line:
            rows.append(json.loads(line))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--pins", type=Path, required=True)
    parser.add_argument("--tla", type=Path, required=True)
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--alloy", type=Path, required=True)
    parser.add_argument("--negative-tla", type=Path, required=True)
    parser.add_argument("--alloy-runner-java", type=Path, required=True)
    parser.add_argument("--alloy-runner-class-dir", type=Path, required=True)
    parser.add_argument("--tla-jar", type=Path, required=True)
    parser.add_argument("--alloy-jar", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--workflow", type=Path, required=True)
    parser.add_argument("--crosswalk", type=Path, required=True)
    parser.add_argument("--reference-explorer", type=Path, required=True)
    parser.add_argument("--runtime-metadata", type=Path, required=True)
    parser.add_argument("--alloy-runner-class", type=Path, required=True)
    args = parser.parse_args()

    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    profile, profile_bytes = read_json(args.profile)
    pins, pins_bytes = read_json(args.pins)
    crosswalk, crosswalk_bytes = read_json(args.crosswalk)
    assert_profile(profile)
    runtime, runtime_bytes = read_json(args.runtime_metadata)
    runtime_contract = profile.get("runtime", {})
    if runtime.get("schema") != "mycelix.sovereignty-formal-runtime.v1":
        fail("unexpected formal runtime metadata schema")
    if runtime.get("nixpkgs_rev") != pins.get("nixpkgs_rev"):
        fail("formal runtime Nixpkgs revision mismatch")
    if runtime.get("jdk_package") != runtime_contract.get("jdk_package", "jdk17_headless"):
        fail("formal runtime JDK package mismatch")
    if runtime.get("java_major") != runtime_contract.get("java_major", 17):
        fail("formal runtime Java major mismatch")
    if not re.match(r'^openjdk version "17\.', str(runtime.get("java_version", ""))):
        fail("formal runtime java version is not OpenJDK 17")
    if not re.match(r"^javac 17\.", str(runtime.get("javac_version", ""))):
        fail("formal runtime javac version is not 17")
    if crosswalk.get("schema") != "mycelix.sovereignty-formal-crosswalk.v1":
        fail("unexpected formal crosswalk schema")
    if crosswalk.get("authority") != "non-authoritative" or crosswalk.get("status") != "candidate-verification":
        fail("formal crosswalk is not explicitly non-authoritative candidate-verification")
    if crosswalk.get("formal_subject_head") != profile["formal_subject"]["head_sha"]:
        fail("formal crosswalk subject head mismatch")
    if crosswalk.get("formal_subject_tree") != profile["formal_subject"]["tree_sha"]:
        fail("formal crosswalk subject tree mismatch")
    if args.crosswalk.as_posix() != profile["verifier"]["crosswalk_path"]:
        fail("formal crosswalk path does not match verifier profile")
    assert_git_blob(args.crosswalk, profile["verifier"]["crosswalk_git_blob_sha"], "formal crosswalk")
    expected_crosswalk_ids = {"capability-authority","safe-dispute","emergency-expiry","fork-weight","contract-budget","contract-authority","provider-authority"}
    mappings = crosswalk.get("mappings", [])
    if {m.get("id") for m in mappings} != expected_crosswalk_ids:
        fail("formal crosswalk property set mismatch")
    for mapping in mappings:
        if mapping.get("coverage") not in {"aligned","partial-no-dedicated-reference-negative-control","partial-alloy-is-witness-not-assertion"}:
            fail("formal crosswalk has unknown coverage classification")
    if pins.get("schema") != "mycelix.sovereignty-formal-tool-pins.v1":
        fail("unexpected formal tool pin schema")
    expected_tla_jar_sha = pins["tools"]["tla2tools"]["sha256"]
    expected_alloy_jar_sha = pins["tools"]["alloy"]["sha256"]
    actual_tla_jar_sha = sha256_file(args.tla_jar)
    actual_alloy_jar_sha = sha256_file(args.alloy_jar)
    if actual_tla_jar_sha != expected_tla_jar_sha:
        fail(f"TLA+ tool bytes do not match pinned SHA256: {actual_tla_jar_sha}")
    if actual_alloy_jar_sha != expected_alloy_jar_sha:
        fail(f"Alloy tool bytes do not match pinned SHA256: {actual_alloy_jar_sha}")

    baseline = {
        str(p): sha256_file(p)
        for p in [
            args.profile, args.pins, args.tla, args.cfg, args.alloy,
            args.negative_tla, args.alloy_runner_java, args.workflow, args.reference_explorer, args.crosswalk,
        args.runtime_metadata, args.alloy_runner_class
        ]
    }

    formal = profile["formal_subject"]
    model_by_name = {m["name"]: m for m in formal["models"]}
    tla_meta = model_by_name["TLA+ temporal sovereignty model"]
    alloy_meta = model_by_name["Alloy structural sovereignty model"]
    fixture = profile["detached_fixture"]
    if args.tla.as_posix() != fixture["tla"]["path"] or args.cfg.as_posix() != fixture["cfg"]["path"] or args.alloy.as_posix() != fixture["alloy"]["path"]:
        fail("formal execution must use the verifier-owned detached fixture paths")
    for label, path, meta in [
        ("TLA fixture", args.tla, fixture["tla"]),
        ("TLA config fixture", args.cfg, fixture["cfg"]),
        ("Alloy fixture", args.alloy, fixture["alloy"]),
    ]:
        assert_git_blob(path, meta["git_blob_sha"], label)
        if meta["git_blob_sha"] != meta["source_git_blob_sha"]:
            fail(f"{label} fixture/source commitment mismatch")
    if fixture["tla"]["git_blob_sha"] != tla_meta["git_blob_sha"] or fixture["cfg"]["git_blob_sha"] != tla_meta["config_git_blob_sha"]:
        fail("detached TLA fixture does not equal formal-subject commitments")
    if fixture["alloy"]["git_blob_sha"] != alloy_meta["git_blob_sha"]:
        fail("detached Alloy fixture does not equal formal-subject commitment")
    reference_meta = formal["reference_explorer"]
    reference_fixture = fixture["reference_explorer"]
    if args.reference_explorer.as_posix() != reference_fixture["path"]:
        fail("formal execution must use the verifier-owned detached reference oracle")
    assert_git_blob(args.reference_explorer, reference_fixture["git_blob_sha"], "reference oracle fixture")
    if reference_fixture["git_blob_sha"] != reference_fixture["source_git_blob_sha"] or reference_fixture["git_blob_sha"] != reference_meta["git_blob_sha"]:
        fail("detached reference oracle does not equal formal-subject commitment")

    cfg = args.cfg.read_text(encoding="utf-8")
    if "INIT Init" not in cfg or "NEXT Next" not in cfg or "CHECK_DEADLOCK FALSE" not in cfg:
        fail("canonical TLC config missing INIT/NEXT/deadlock contract")
    cfg_invariants = re.findall(r"^([A-Za-z][A-Za-z0-9_]*)$", cfg, re.MULTILINE)
    for invariant in INVARIANTS:
        if invariant not in cfg_invariants:
            fail(f"canonical TLC config omits invariant {invariant}")

    alloy_source = args.alloy.read_text(encoding="utf-8")
    if re.search(r"sig\s+Emergency\s*\{[^}]*\bactive\b", alloy_source, re.DOTALL):
        fail("mutable emergency active bit remains in Alloy model")
    if "pred EmergencyActive" not in alloy_source:
        fail("Alloy model lacks derived EmergencyActive predicate")

    workflow = args.workflow.read_text(encoding="utf-8")
    if f"NIXPKGS_REV: {pins['nixpkgs_rev']}" not in workflow:
        fail("workflow Nixpkgs pin mismatch")
    expected_actions = pins["github_actions"]
    for action, sha in expected_actions.items():
        needle = {
            "checkout": f"actions/checkout@{sha}",
            "install_nix": f"cachix/install-nix-action@{sha}",
            "upload_artifact": f"actions/upload-artifact@{sha}",
        }[action]
        if needle not in workflow:
            fail(f"workflow action pin mismatch for {action}")
    for tool_name, tool in pins["tools"].items():
        if tool["url"] not in workflow or tool["sha256"] not in workflow:
            fail(f"workflow tool pin mismatch for {tool_name}")
    if f"TLA_URL: {pins['tools']['tla2tools']['url']}" not in workflow:
        fail("workflow TLA URL pin mismatch")
    if f"TLA_SHA256: {pins['tools']['tla2tools']['sha256']}" not in workflow:
        fail("workflow TLA SHA256 pin mismatch")
    if f"ALLOY_URL: {pins['tools']['alloy']['url']}" not in workflow:
        fail("workflow Alloy URL pin mismatch")
    if f"ALLOY_SHA256: {pins['tools']['alloy']['sha256']}" not in workflow:
        fail("workflow Alloy SHA256 pin mismatch")

    evidence = args.evidence_dir.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    receipt = {
        "receipt_schema": "sovereignty-formal-qualification-receipt-v2",
        "result": "ExecutedFail",
        "claim_scope": "bounded formal-model qualification only; exact pinned checker outcomes against frozen model/config bytes",
        "subject": {
            "repository": formal.get("repository", "Luminous-Dynamics/mycelix"),
            "head_sha": formal["head_sha"],
            "tree_sha": formal["tree_sha"],
            "model_blobs": {
                "tla": tla_meta["git_blob_sha"],
                "cfg": tla_meta["config_git_blob_sha"],
                "alloy": alloy_meta["git_blob_sha"],
            },
        },
        "verifier": {
            "profile_sha256": sha256_bytes(profile_bytes),
            "pins_sha256": sha256_bytes(pins_bytes),
            "runner_sha256": sha256_file(Path(__file__)),
            "alloy_runner_sha256": sha256_file(args.alloy_runner_java),
            "crosswalk_sha256": sha256_bytes(crosswalk_bytes),
            "runtime_metadata_sha256": sha256_bytes(runtime_bytes),
            "alloy_runner_class_sha256": sha256_file(args.alloy_runner_class),
        },
        "runtime": runtime,
        "tools": {
            "tla2tools": {
                "version": pins["tools"]["tla2tools"]["version"],
                "sha256": sha256_file(args.tla_jar),
            },
            "alloy": {
                "version": pins["tools"]["alloy"]["version"],
                "sha256": sha256_file(args.alloy_jar),
                "solver": "SAT4J",
            },
        },
        "canonical": {},
        "negative_controls": {"tla": {}, "alloy": {}},
        "nonclaims": [
            "No current AI is recognized as sovereign.",
            "No legal personhood, citizenship, political standing, or sovereignty is established.",
            "No runtime authority or production safety is established.",
            "No bounded result is an unbounded mathematical proof.",
            "Formal qualification does not establish legal recognition or external adoption.",
        ],
    }

    try:
        ref = run([sys.executable, str(args.reference_explorer)])
        receipt["reference_explorer"] = record_command(evidence, "reference-explorer", [sys.executable, str(args.reference_explorer)], ref)
        if ref.returncode != 0:
            fail("reference explorer smoke evidence failed")
        for control, target in REFERENCE_CONTROLS.items():
            expected = f"NEGATIVE PASS: {control} -> {target} counterexample"
            if expected not in ref.stdout:
                fail(f"reference explorer missing expected control output: {control}")
        if "CANONICAL PASS: no invariant violation through depth 4" not in ref.stdout:
            fail("reference explorer missing canonical bounded result")
        if "BOUNDED REFERENCE EXPLORATION PASS: semantic smoke evidence only" not in ref.stdout:
            fail("reference explorer final smoke evidence marker missing")

        tla_cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(args.cfg), str(args.tla)]
        tla = run(tla_cmd)
        receipt["canonical"]["tla"] = record_command(evidence, "tla-canonical", tla_cmd, tla)
        if tla.returncode != 0 or CANONICAL_OK not in tla.stdout:
            fail("canonical TLA+ run did not complete cleanly")
        canonical_violations = tla_violations(tla.stdout)
        if canonical_violations:
            fail("canonical TLA+ invariants violated: " + ", ".join(sorted(canonical_violations)))
        receipt["canonical"]["tla"]["violated_invariants"] = sorted(canonical_violations)

        negative_dir = evidence / "tla-negative"
        negative_dir.mkdir()
        negative_source = args.negative_tla.read_text(encoding="utf-8")
        assert "MODULE ArtificialSovereigntyV1NegativeControls" in negative_source

        for control, target in NEGATIVE_TLA.items():
            cfg_text = (
                "CONSTANTS\n"
                "Human = Human\nArtificial = Artificial\nS1 = S1\nS2 = S2\n"
                "PowerA = PowerA\nPowerB = PowerB\nActionA = ActionA\nActionB = ActionB\n"
                "MaxBudget = 2\nMaxTime = 6\n"
                f'Control = "{control}"\n\n'
                "INIT Init\nNEXT NegativeNext\nINVARIANTS\n" +
                "\n".join(INVARIANTS) +
                "\nCHECK_DEADLOCK FALSE\n"
            )
            work = negative_dir / control
            work.mkdir()
            wrapper = work / args.negative_tla.name
            canonical_module = work / args.tla.name
            config = work / "negative.cfg"
            wrapper.write_text(negative_source, encoding="utf-8")
            canonical_module.write_text(args.tla.read_text(encoding="utf-8"), encoding="utf-8")
            config.write_text(cfg_text, encoding="utf-8")
            cmd = ["java", "-cp", str(args.tla_jar), "tlc2.TLC", "-workers", "1",
                   "-config", str(config), str(wrapper)]
            result = run(cmd)
            violations = tla_violations(result.stdout)
            receipt["negative_controls"]["tla"][control] = {
                **record_command(evidence, f"tla-negative-{control}", cmd, result),
                "target_invariant": target,
                "violated_invariants": sorted(violations),
                "wrapper_sha256": sha256_file(wrapper),
                "canonical_module_sha256": sha256_file(canonical_module),
                "config_sha256": sha256_file(config),
            }
            if result.returncode == 0 or violations != {target}:
                fail(f"TLA+ negative control {control} did not isolate expected {target} counterexample; observed {sorted(violations)}")

        alloy_cp = f"{args.alloy_runner_class_dir}:{args.alloy_jar}"
        alloy_cmd = ["java", "-cp", alloy_cp,
                     "SovereigntyAlloyQualificationRunner", str(args.alloy)]
        alloy = run(alloy_cmd)
        receipt["canonical"]["alloy"] = record_command(evidence, "alloy-canonical", alloy_cmd, alloy)
        if alloy.returncode != 0:
            fail("canonical Alloy runner failed")
        rows = alloy_outcomes(alloy.stdout)
        by_label = {row["label"]: row for row in rows}
        if set(by_label) != EXPECTED_ALLOY_SAT | EXPECTED_ALLOY_UNSAT:
            fail("canonical Alloy command label set mismatch")
        for label in EXPECTED_ALLOY_SAT:
            if by_label[label]["actual"] != "SAT":
                fail(f"Alloy expected-SAT command was not SAT: {label}")
        for label in EXPECTED_ALLOY_UNSAT:
            if by_label[label]["actual"] != "UNSAT" or by_label[label]["expects"] != 0:
                fail(f"Alloy expected-UNSAT command was not UNSAT/expect-0: {label}")
        scope = profile["alloy"]["scope"]
        bitwidth = profile["alloy"]["bitwidth"]
        for row in rows:
            command_text = row["command"]
            if f"for {scope['overall']}" not in command_text:
                fail("Alloy command omitted expected overall scope: " + row["label"])
            if f"{bitwidth} int" not in command_text:
                fail("Alloy command omitted expected integer bitwidth: " + str(bitwidth) + " int for " + row["label"])
            for name in ["Subject", "Power", "Provider", "Dependency", "Action", "Contract", "Budget", "Dispute", "Emergency", "ForkEvent"]:
                token = f"{scope[name]} {name}"
                if token not in command_text:
                    fail("Alloy command omitted expected scope: " + token + " for " + row["label"])
        for row in rows:
            if not re.fullmatch(r"[0-9a-f]{64}", str(row.get("solution_sha256", ""))):
                fail("Alloy runner omitted a valid solution digest for " + row["label"])
            if row["label"] in EXPECTED_ALLOY_SAT:
                if row["check"] is not False or row["expects"] != 1 or row["actual"] != "SAT":
                    fail("Alloy SAT command contract mismatch: " + row["label"])
            else:
                if row["check"] is not True or row["expects"] != 0 or row["actual"] != "UNSAT":
                    fail("Alloy UNSAT command contract mismatch: " + row["label"])
        receipt["canonical"]["alloy"]["commands"] = rows
        receipt["canonical"]["alloy"]["solver"] = "SAT4J"
        receipt["canonical"]["alloy"]["scope"] = profile["alloy"]["scope"]
        canonical_alloy = {row["label"]: row["actual"] for row in rows}

        source = alloy_source
        negalloy = evidence / "alloy-negative"
        negalloy.mkdir()
        for fact_name, target in ALLOY_NEGATIVE_FACTS.items():
            mutated = remove_named_fact(source, fact_name)
            work = negalloy / fact_name
            work.mkdir()
            mutated_file = work / args.alloy.name
            mutated_file.write_text(mutated, encoding="utf-8")
            cmd = ["java", "-cp", alloy_cp,
                   "SovereigntyAlloyQualificationRunner", str(mutated_file)]
            result = run(cmd)
            rows = alloy_outcomes(result.stdout)
            by_label = {row["label"]: row for row in rows}
            if set(by_label) != set(canonical_alloy):
                fail(f"Alloy negative control command label set changed: {fact_name}")
            if by_label.get(target, {}).get("actual") != "SAT":
                fail(f"Alloy negative control did not expose counterexample: {fact_name} -> {target}")
            for label, expected_actual in canonical_alloy.items():
                if label != target and by_label[label]["actual"] != expected_actual:
                    fail(f"Alloy negative control changed unrelated outcome: {fact_name} -> {label}")
            for row in rows:
                if not re.fullmatch(r"[0-9a-f]{64}", str(row.get("solution_sha256", ""))):
                    fail("Alloy negative control omitted solution digest: " + fact_name)
            receipt["negative_controls"]["alloy"][fact_name] = {
                **record_command(evidence, f"alloy-negative-{fact_name}", cmd, result),
                "target_assertion": target,
                "mutated_model_sha256": sha256_file(mutated_file),
                "outcomes": {label: by_label[label]["actual"] for label in sorted(by_label)},
            }

        after = {
            str(p): sha256_file(p)
            for p in baseline
        }
        if after != baseline:
            fail("canonical/verifier inputs changed during qualification")
        if subprocess.run(["git", "status", "--porcelain"], text=True, stdout=subprocess.PIPE).stdout.strip():
            fail("qualification changed tracked/untracked repository state")

        receipt["result"] = "QualifiedExactHead"
        receipt["postflight"] = {
            "inputs_unchanged": True,
            "git_status_clean": True,
            "baseline_sha256": baseline,
        }
    except Exception as exc:
        receipt["failure"] = str(exc)
        receipt_path = evidence / "sovereignty-formal-qualification-receipt-v2.json"
        receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(receipt, sort_keys=True))
        if str(exc).startswith(FAIL_PREFIX):
            print(str(exc), file=sys.stderr)
        else:
            print(FAIL_PREFIX + str(exc), file=sys.stderr)
        return 1

    receipt_path = evidence / "sovereignty-formal-qualification-receipt-v2.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
