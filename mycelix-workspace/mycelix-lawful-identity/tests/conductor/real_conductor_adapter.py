#!/usr/bin/env python3
"""Strict adapter from a Holochain-specific driver to evidence protocol v2."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from evidence_manifest import (
    EvidenceError,
    canonical_json_bytes,
    contract_hash,
    file_sha256_b64,
    load_and_validate_manifest,
    manifest_hash,
    require_hash32,
    require_text,
    sha256_b64,
)

ROOT = Path(__file__).resolve().parent
CONTRACT_PATH = ROOT / "settlement_scenarios_v2.json"
ADAPTER_PROTOCOL = "mycelix-conductor-scenario-adapter-v2"
DRIVER_PROTOCOL = "mycelix-real-conductor-driver-v1"
MAX_DRIVER_STDOUT_BYTES = 256 * 1024
DRIVER_TIMEOUT_SECONDS = int(os.environ.get("MYCELIX_CONDUCTOR_DRIVER_TIMEOUT_SECONDS", "900"))
REQUIRED_AGENTS = {"verifier", "prover_a", "prover_b"}


class AdapterError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AdapterError(message)


def invoke_driver(driver: Path, *args: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            [str(driver), *args],
            check=False,
            capture_output=True,
            timeout=DRIVER_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as error:
        raise AdapterError(f"driver timed out after {DRIVER_TIMEOUT_SECONDS}s") from error
    except OSError as error:
        raise AdapterError(f"failed to execute conductor driver: {error}") from error
    require(len(completed.stdout) <= MAX_DRIVER_STDOUT_BYTES, "driver stdout exceeded limit")
    stderr = completed.stderr.decode("utf-8", errors="replace").strip()
    require(
        completed.returncode == 0,
        f"driver exited {completed.returncode}" + (f": {stderr}" if stderr else ""),
    )
    try:
        value = json.loads(completed.stdout.decode("utf-8", errors="strict"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AdapterError(f"driver output is not one UTF-8 JSON value: {error}") from error
    require(isinstance(value, dict), "driver output must be a JSON object")
    require(value.get("driver_protocol") == DRIVER_PROTOCOL, "unsupported driver protocol")
    return value


def load_contract() -> dict[str, Any]:
    value = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    require(isinstance(value, dict), "scenario contract must be an object")
    scenarios = value.get("scenarios")
    require(isinstance(scenarios, list), "scenario contract must contain scenarios")
    return value


def scenario_by_id(contract: dict[str, Any], scenario_id: str) -> dict[str, Any]:
    matches = [scenario for scenario in contract["scenarios"] if scenario.get("id") == scenario_id]
    require(len(matches) == 1, f"unknown or duplicate scenario id: {scenario_id}")
    return matches[0]


def configured_paths() -> tuple[Path, Path]:
    driver_value = os.environ.get("MYCELIX_CONDUCTOR_DRIVER")
    evidence_value = os.environ.get("MYCELIX_CONDUCTOR_EVIDENCE_ROOT")
    require(bool(driver_value), "MYCELIX_CONDUCTOR_DRIVER is required")
    require(bool(evidence_value), "MYCELIX_CONDUCTOR_EVIDENCE_ROOT is required")
    driver = Path(driver_value).resolve(strict=True)
    evidence_root = Path(evidence_value).resolve(strict=True)
    require(driver.is_file(), "conductor driver is not a file")
    require(driver.stat().st_mode & 0o111 != 0, "conductor driver is not executable")
    require(not driver.is_symlink(), "conductor driver may not be a symlink")
    require(evidence_root.is_dir(), "evidence root is not a directory")
    require(not evidence_root.is_symlink(), "evidence root may not be a symlink")
    return driver, evidence_root


def validate_driver_capabilities(value: dict[str, Any]) -> dict[str, Any]:
    require_hash32(value.get("driver_release_hash"), "driver_release_hash")
    count = value.get("conductor_count")
    require(isinstance(count, int) and 3 <= count <= 16, "driver requires 3-16 conductors")
    require_text(value.get("dna_hash"), "dna_hash", 4096)
    require_text(value.get("happ_hash"), "happ_hash", 4096)
    require_text(value.get("holochain_version"), "holochain_version", 256)
    agents = value.get("agents")
    require(isinstance(agents, list), "agents must be a list")
    require(all(isinstance(agent, str) and agent for agent in agents), "agents must be non-empty strings")
    require(set(agents) >= REQUIRED_AGENTS, "driver is missing required agents")
    conductors = value.get("conductors")
    require(isinstance(conductors, list) and len(conductors) == count, "conductor metadata count mismatch")
    ids: list[str] = []
    databases: list[str] = []
    networks: list[str] = []
    for index, conductor in enumerate(conductors):
        require(isinstance(conductor, dict), f"conductors[{index}] must be an object")
        ids.append(require_text(conductor.get("id"), f"conductors[{index}].id"))
        require_text(conductor.get("agent_pub_key"), f"conductors[{index}].agent_pub_key", 4096)
        databases.append(require_hash32(conductor.get("database_identity_hash"), f"conductors[{index}].database_identity_hash"))
        networks.append(require_hash32(conductor.get("network_identity_hash"), f"conductors[{index}].network_identity_hash"))
    require(len(ids) == len(set(ids)), "conductor ids must be unique")
    require(len(databases) == len(set(databases)), "database identities must be unique")
    require(len(networks) == len(set(networks)), "network identities must be unique")
    require(set(ids) >= REQUIRED_AGENTS, "required conductor roles are missing")
    return value


def adapter_release_hash(driver_capabilities: dict[str, Any]) -> str:
    value = {
        "domain": "MYCELIX:RealConductorAdapterRelease:v1",
        "adapter_file_sha256": file_sha256_b64(Path(__file__).resolve()),
        "driver_release_hash": driver_capabilities["driver_release_hash"],
    }
    return sha256_b64(canonical_json_bytes(value))


def deployment_manifest_hash(driver_capabilities: dict[str, Any]) -> str:
    value = {
        "domain": "MYCELIX:RealConductorDeployment:v1",
        "scenario_contract_hash": contract_hash(CONTRACT_PATH),
        "driver_capabilities": driver_capabilities,
        "adapter_release_hash": adapter_release_hash(driver_capabilities),
    }
    return sha256_b64(canonical_json_bytes(value))


def capabilities(driver: Path) -> dict[str, Any]:
    driver_capabilities = validate_driver_capabilities(invoke_driver(driver, "--capabilities"))
    return {
        "adapter_protocol": ADAPTER_PROTOCOL,
        "adapter_kind": "real-conductor",
        "scenario_contract_hash": contract_hash(CONTRACT_PATH),
        "adapter_release_hash": adapter_release_hash(driver_capabilities),
        "deployment_manifest_hash": deployment_manifest_hash(driver_capabilities),
        "conductor_count": driver_capabilities["conductor_count"],
        "dna_hash": driver_capabilities["dna_hash"],
        "happ_hash": driver_capabilities["happ_hash"],
        "holochain_version": driver_capabilities["holochain_version"],
        "agents": driver_capabilities["agents"],
        "conductors": driver_capabilities["conductors"],
    }


def enumerate_files(scenario_dir: Path) -> list[dict[str, Any]]:
    files: list[dict[str, Any]] = []
    for path in sorted(scenario_dir.rglob("*")):
        require(not path.is_symlink(), f"driver evidence may not contain symlinks: {path}")
        if path.is_dir():
            continue
        require(path.is_file(), f"driver evidence contains a non-regular file: {path}")
        if path.name == "manifest.json" and path.parent == scenario_dir:
            continue
        relative = path.relative_to(scenario_dir).as_posix()
        files.append(
            {
                "path": relative,
                "sha256": file_sha256_b64(path),
                "size": path.stat().st_size,
            }
        )
    require(bool(files), "driver retained no evidence files")
    return files


def run_scenario(driver: Path, evidence_root: Path, scenario_id: str) -> dict[str, Any]:
    contract = load_contract()
    scenario = scenario_by_id(contract, scenario_id)
    scenario_dir = evidence_root / scenario_id
    require(scenario_dir.parent == evidence_root, "scenario path escaped evidence root")
    if scenario_dir.exists():
        require(not scenario_dir.is_symlink(), "scenario evidence directory may not be a symlink")
        require(scenario_dir.is_dir(), "scenario evidence path is not a directory")
        require(not any(scenario_dir.iterdir()), "scenario evidence directory must start empty")
    else:
        scenario_dir.mkdir(mode=0o700)

    driver_capabilities = validate_driver_capabilities(invoke_driver(driver, "--capabilities"))
    result = invoke_driver(
        driver,
        "--scenario",
        scenario_id,
        "--evidence-dir",
        str(scenario_dir),
    )
    require(result.get("scenario_id") == scenario_id, "driver changed scenario_id")
    expected_state = require_text(scenario.get("expected_state"), "expected_state")
    observed_state = require_text(result.get("observed_state"), "observed_state")
    require(observed_state == expected_state, f"expected {expected_state}, observed {observed_state}")
    started = result.get("started_at_unix_micros")
    completed = result.get("completed_at_unix_micros")
    require(isinstance(started, int) and started > 0, "driver start timestamp must be positive")
    require(isinstance(completed, int) and completed >= started, "driver completion precedes start")
    events = result.get("events")
    observations = result.get("observations")
    require(isinstance(events, list) and bool(events), "driver must return events")
    require(isinstance(observations, list), "driver must return observations")

    files = enumerate_files(scenario_dir)
    files_by_path = {item["path"]: item for item in files}
    normalized_observations = []
    for index, observation in enumerate(observations):
        require(isinstance(observation, dict), f"observations[{index}] must be an object")
        response_path = require_text(
            observation.get("response_path"),
            f"observations[{index}].response_path",
            4096,
        )
        require(response_path in files_by_path, f"observations[{index}] response file was not retained")
        normalized = dict(observation)
        normalized["response_sha256"] = files_by_path[response_path]["sha256"]
        normalized_observations.append(normalized)

    manifest = {
        "schema": "mycelix-conductor-evidence-manifest-v1",
        "scenario_contract_hash": contract_hash(CONTRACT_PATH),
        "scenario_id": scenario_id,
        "expected_state": expected_state,
        "observed_state": observed_state,
        "dna_hash": driver_capabilities["dna_hash"],
        "happ_hash": driver_capabilities["happ_hash"],
        "adapter_release_hash": adapter_release_hash(driver_capabilities),
        "started_at_unix_micros": started,
        "completed_at_unix_micros": completed,
        "conductors": driver_capabilities["conductors"],
        "events": events,
        "observations": normalized_observations,
        "files": files,
    }
    manifest_path = scenario_dir / "manifest.json"
    manifest_path.write_bytes(canonical_json_bytes(manifest) + b"\n")
    _, summary = load_and_validate_manifest(
        manifest_path,
        evidence_root,
        expected_contract_hash=contract_hash(CONTRACT_PATH),
        expected_scenario_id=scenario_id,
        expected_state=expected_state,
        expected_dna_hash=driver_capabilities["dna_hash"],
    )
    relative_manifest = manifest_path.relative_to(evidence_root).as_posix()
    return {
        "adapter_protocol": ADAPTER_PROTOCOL,
        "adapter_kind": "real-conductor",
        "scenario_id": scenario_id,
        "observed_state": observed_state,
        "evidence": {
            "level": "multi-conductor",
            "bundle_hash": summary["manifest_hash"],
            "manifest_hash": summary["manifest_hash"],
            "manifest_path": relative_manifest,
            "observed_by": summary["conductor_ids"],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capabilities", action="store_true")
    parser.add_argument("--scenario")
    args = parser.parse_args()
    try:
        driver, evidence_root = configured_paths()
        if args.capabilities:
            require(args.scenario is None, "--capabilities and --scenario are mutually exclusive")
            print(json.dumps(capabilities(driver), sort_keys=True, separators=(",", ":")))
            return 0
        require(bool(args.scenario), "--scenario is required")
        print(
            json.dumps(
                run_scenario(driver, evidence_root, args.scenario),
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        return 0
    except (AdapterError, EvidenceError, OSError, ValueError) as error:
        print(f"real conductor adapter: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
