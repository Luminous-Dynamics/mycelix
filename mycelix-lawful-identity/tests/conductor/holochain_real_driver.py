#!/usr/bin/env python3
"""Reviewed scenario orchestrator for the real-conductor evidence lane.

This executable owns scenario ordering, subprocess bounds, runtime dependency
pinning, evidence retention, and final-state agreement. A separately reviewed
Holochain control executable owns websocket calls, conductor lifecycle, and
network partition mechanics through the narrow control protocol documented in
HOLOCHAIN_CONTROL_PROTOCOL.md.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

DRIVER_PROTOCOL = "mycelix-real-conductor-driver-v1"
CONTROL_PROTOCOL = "mycelix-holochain-control-v1"
CONFIG_SCHEMA = "mycelix-holochain-real-driver-config-v1"
CONTRACT_SCHEMA = "mycelix-lawful-identity-conductor-scenarios-v2"
FINAL_OBSERVATION_SCHEMA = "mycelix-conductor-final-observation-v1"
MAX_CONFIG_BYTES = 256 * 1024
MAX_CONTROL_STDOUT_BYTES = 2 * 1024 * 1024
MAX_ARTIFACT_BYTES = 2 * 1024 * 1024
MAX_ARTIFACT_COUNT = 128
MAX_ACTION_HASHES = 512
MAX_SCENARIO_EVIDENCE_BYTES = 64 * 1024 * 1024
MAX_SCENARIO_STEPS = 128
REQUIRED_AGENTS = {"verifier", "prover_a", "prover_b"}


class DriverError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise DriverError(message)


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sha256_b64(data: bytes) -> str:
    return base64.b64encode(hashlib.sha256(data).digest()).decode("ascii")


def require_hash32(value: Any, name: str) -> str:
    require(isinstance(value, str) and len(value) == 44, f"{name} must be canonical standard-base64 SHA-256")
    try:
        decoded = base64.b64decode(value, validate=True)
    except Exception as error:
        raise DriverError(f"{name} is not canonical base64: {error}") from error
    require(len(decoded) == 32 and base64.b64encode(decoded).decode("ascii") == value, f"{name} must decode to exactly 32 bytes")
    return value


def require_text(value: Any, name: str, maximum: int = 4096) -> str:
    require(isinstance(value, str) and value != "", f"{name} must be non-empty text")
    require(len(value.encode("utf-8")) <= maximum, f"{name} exceeds {maximum} bytes")
    require(not any(ord(ch) < 0x20 for ch in value), f"{name} contains control characters")
    return value


def checked_path(value: str, name: str, *, executable: bool = False) -> Path:
    raw = Path(value).expanduser()
    absolute = raw if raw.is_absolute() else Path.cwd() / raw
    absolute = Path(os.path.abspath(absolute))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except OSError as error:
            raise DriverError(f"{name} cannot be inspected: {error}") from error
        require(not stat.S_ISLNK(mode), f"{name} may not traverse symlinks")
    resolved = absolute.resolve(strict=True)
    require(resolved.is_file(), f"{name} must be a regular file")
    if executable:
        require(resolved.stat().st_mode & 0o111 != 0, f"{name} must be executable")
    return resolved


def file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return base64.b64encode(digest.digest()).decode("ascii")


def confined_relative_path(value: Any, name: str) -> Path:
    text = require_text(value, name, 512)
    path = Path(text)
    require(not path.is_absolute(), f"{name} must be relative")
    require(path.parts and all(part not in {"", ".", ".."} for part in path.parts), f"{name} is not confined")
    return path


def load_json_file(path: Path, name: str, maximum: int) -> dict[str, Any]:
    require(path.stat().st_size <= maximum, f"{name} exceeds {maximum} bytes")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DriverError(f"invalid {name}: {error}") from error
    require(isinstance(value, dict), f"{name} must be a JSON object")
    return value


def validate_conductors(value: Any) -> list[dict[str, str]]:
    require(isinstance(value, list) and 3 <= len(value) <= 16, "conductors must contain 3-16 entries")
    output: list[dict[str, str]] = []
    ids: set[str] = set()
    databases: set[str] = set()
    networks: set[str] = set()
    for index, conductor in enumerate(value):
        require(isinstance(conductor, dict), f"conductors[{index}] must be an object")
        require(set(conductor) == {"id", "agent_pub_key", "database_identity_hash", "network_identity_hash"}, f"conductors[{index}] has missing or unknown fields")
        item = {
            "id": require_text(conductor["id"], f"conductors[{index}].id", 128),
            "agent_pub_key": require_text(conductor["agent_pub_key"], f"conductors[{index}].agent_pub_key"),
            "database_identity_hash": require_hash32(conductor["database_identity_hash"], f"conductors[{index}].database_identity_hash"),
            "network_identity_hash": require_hash32(conductor["network_identity_hash"], f"conductors[{index}].network_identity_hash"),
        }
        require(item["id"] not in ids, "conductor ids must be unique")
        require(item["database_identity_hash"] not in databases, "database identities must be unique")
        require(item["network_identity_hash"] not in networks, "network identities must be unique")
        ids.add(item["id"]); databases.add(item["database_identity_hash"]); networks.add(item["network_identity_hash"])
        output.append(item)
    require(ids >= REQUIRED_AGENTS, "required conductor roles are missing")
    return output


def load_config() -> tuple[dict[str, Any], Path, Path, Path, dict[str, Any]]:
    config_value = os.environ.get("MYCELIX_HOLOCHAIN_DRIVER_CONFIG")
    require(bool(config_value), "MYCELIX_HOLOCHAIN_DRIVER_CONFIG is required")
    config_path = checked_path(config_value, "driver config")
    config = load_json_file(config_path, "driver config", MAX_CONFIG_BYTES)
    expected = {
        "schema", "control_protocol", "control_executable", "control_executable_sha256",
        "scenario_contract", "scenario_contract_sha256", "holochain_version", "dna_hash",
        "happ_hash", "agents", "conductors", "step_timeout_seconds",
    }
    require(set(config) == expected, "driver config has missing or unknown fields")
    require(config["schema"] == CONFIG_SCHEMA, "unsupported driver config schema")
    require(config["control_protocol"] == CONTROL_PROTOCOL, "unsupported control protocol")
    control = checked_path(config["control_executable"], "control executable", executable=True)
    require(file_hash(control) == require_hash32(config["control_executable_sha256"], "control_executable_sha256"), "control executable hash mismatch")
    contract_path = checked_path(config["scenario_contract"], "scenario contract")
    require(file_hash(contract_path) == require_hash32(config["scenario_contract_sha256"], "scenario_contract_sha256"), "scenario contract hash mismatch")
    contract = load_json_file(contract_path, "scenario contract", MAX_CONFIG_BYTES)
    require(contract.get("schema") == CONTRACT_SCHEMA, "unexpected scenario contract schema")
    scenarios = contract.get("scenarios")
    require(isinstance(scenarios, list) and bool(scenarios), "scenario contract has no scenarios")
    require(len(scenarios) <= 128, "scenario contract contains too many scenarios")
    require_text(config["holochain_version"], "holochain_version", 256)
    require_text(config["dna_hash"], "dna_hash")
    require_text(config["happ_hash"], "happ_hash")
    agents = config["agents"]
    require(isinstance(agents, list) and all(isinstance(agent, str) and agent for agent in agents), "agents must be non-empty strings")
    require(set(agents) >= REQUIRED_AGENTS and len(agents) == len(set(agents)), "agents must be unique and include verifier/prover_a/prover_b")
    config["conductors"] = validate_conductors(config["conductors"])
    timeout = config["step_timeout_seconds"]
    require(isinstance(timeout, int) and 1 <= timeout <= 1800, "step_timeout_seconds must be 1-1800")
    return config, config_path, control, contract_path, contract


def runtime_lock_hash(config: dict[str, Any], config_path: Path, contract_path: Path) -> str:
    value = {
        "domain": "MYCELIX:HolochainRealDriverRuntimeLock:v1",
        "config_file_sha256": file_hash(config_path),
        "control_executable_sha256": config["control_executable_sha256"],
        "scenario_contract_sha256": file_hash(contract_path),
        "holochain_version": config["holochain_version"],
        "dna_hash": config["dna_hash"],
        "happ_hash": config["happ_hash"],
        "agents": config["agents"],
        "conductors": config["conductors"],
        "step_timeout_seconds": config["step_timeout_seconds"],
    }
    return sha256_b64(canonical_json_bytes(value))


def invoke_control(control: Path, expected_hash: str, timeout: int, *args: str) -> dict[str, Any]:
    require(file_hash(control) == expected_hash, "control executable changed before invocation")
    try:
        completed = subprocess.run([str(control), *args], check=False, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        raise DriverError(f"control executable timed out after {timeout}s") from error
    except OSError as error:
        raise DriverError(f"control executable failed to start: {error}") from error
    require(file_hash(control) == expected_hash, "control executable changed during invocation")
    require(len(completed.stdout) <= MAX_CONTROL_STDOUT_BYTES, "control stdout exceeds limit")
    stderr = completed.stderr.decode("utf-8", errors="replace").strip()
    require(completed.returncode == 0, f"control exited {completed.returncode}" + (f": {stderr}" if stderr else ""))
    try:
        value = json.loads(completed.stdout.decode("utf-8", errors="strict"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DriverError(f"control output is not one UTF-8 JSON value: {error}") from error
    require(isinstance(value, dict), "control output must be a JSON object")
    require(value.get("control_protocol") == CONTROL_PROTOCOL, "unsupported control response protocol")
    return value


def control_capabilities(config: dict[str, Any], control: Path) -> dict[str, Any]:
    value = invoke_control(control, config["control_executable_sha256"], config["step_timeout_seconds"], "--capabilities")
    expected = {"control_protocol", "holochain_version", "dna_hash", "happ_hash", "agents", "conductors"}
    require(set(value) == expected, "control capabilities have missing or unknown fields")
    require(value["holochain_version"] == config["holochain_version"], "control Holochain version differs from config")
    require(value["dna_hash"] == config["dna_hash"], "control DNA hash differs from config")
    require(value["happ_hash"] == config["happ_hash"], "control hApp hash differs from config")
    require(value["agents"] == config["agents"], "control agents differ from config")
    require(validate_conductors(value["conductors"]) == config["conductors"], "control conductor identities differ from config")
    return value


def capabilities(config: dict[str, Any], config_path: Path, control: Path, contract_path: Path) -> dict[str, Any]:
    control_capabilities(config, control)
    release_hash = require_hash32(os.environ.get("MYCELIX_DRIVER_RELEASE_HASH"), "MYCELIX_DRIVER_RELEASE_HASH")
    return {
        "driver_protocol": DRIVER_PROTOCOL,
        "driver_release_hash": release_hash,
        "runtime_lock_hash": runtime_lock_hash(config, config_path, contract_path),
        "conductor_count": len(config["conductors"]),
        "dna_hash": config["dna_hash"],
        "happ_hash": config["happ_hash"],
        "holochain_version": config["holochain_version"],
        "agents": config["agents"],
        "conductors": config["conductors"],
    }


def scenario_by_id(contract: dict[str, Any], scenario_id: str) -> dict[str, Any]:
    matches = [value for value in contract["scenarios"] if isinstance(value, dict) and value.get("id") == scenario_id]
    require(len(matches) == 1, f"unknown or duplicate scenario id: {scenario_id}")
    scenario = matches[0]
    require(set(scenario) == {"id", "partition", "steps", "expected_state", "assertions"}, "scenario has missing or unknown fields")
    require(isinstance(scenario["steps"], list) and all(isinstance(step, str) and step for step in scenario["steps"]), "scenario steps are malformed")
    require(len(scenario["steps"]) <= MAX_SCENARIO_STEPS, "scenario contains too many steps")
    return scenario


def prepare_evidence_dir(value: str) -> Path:
    raw = Path(value)
    require(raw.is_absolute(), "evidence directory must be absolute")
    require(not raw.is_symlink(), "evidence directory may not be a symlink")
    resolved = raw.resolve(strict=True)
    require(resolved.is_dir(), "evidence directory must exist")
    require(resolved.stat().st_mode & 0o022 == 0, "evidence directory may not be group- or world-writable")
    require(not any(resolved.iterdir()), "evidence directory must start empty")
    return resolved


def write_json(path: Path, value: Any) -> None:
    require(not path.exists() and not path.is_symlink(), f"evidence path already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json_bytes(value) + b"\n")
    os.chmod(path, 0o444)


def write_artifacts(step_dir: Path, artifacts: Any) -> None:
    require(isinstance(artifacts, list), "step artifacts must be a list")
    require(len(artifacts) <= MAX_ARTIFACT_COUNT, "step returned too many artifacts")
    total = 0
    for index, artifact in enumerate(artifacts):
        require(isinstance(artifact, dict) and set(artifact) == {"path", "content_b64"}, f"artifacts[{index}] is malformed")
        relative = confined_relative_path(artifact["path"], f"artifacts[{index}].path")
        encoded = require_text(artifact["content_b64"], f"artifacts[{index}].content_b64", MAX_ARTIFACT_BYTES * 2)
        try:
            content = base64.b64decode(encoded, validate=True)
        except Exception as error:
            raise DriverError(f"artifacts[{index}] is not canonical base64: {error}") from error
        require(base64.b64encode(content).decode("ascii") == encoded, f"artifacts[{index}] is not canonical base64")
        total += len(content)
        require(total <= MAX_ARTIFACT_BYTES, "step artifacts exceed total byte limit")
        destination = step_dir.joinpath(*relative.parts)
        require(destination.resolve(strict=False).is_relative_to(step_dir.resolve()), "artifact path escaped step directory")
        require(not destination.exists() and not destination.is_symlink(), "artifact path already exists")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        os.chmod(destination, 0o444)


def validate_action_hashes(value: Any, name: str) -> list[str]:
    require(isinstance(value, list) and len(value) <= MAX_ACTION_HASHES, f"{name} must be a bounded list")
    output: list[str] = []
    for index, action_hash in enumerate(value):
        output.append(require_text(action_hash, f"{name}[{index}]", 4096))
    require(len(output) == len(set(output)), f"{name} contains duplicates")
    return output


def run_step(config: dict[str, Any], control: Path, scenario_id: str, sequence: int, step: str, evidence_dir: Path) -> dict[str, Any]:
    request = {"control_protocol": CONTROL_PROTOCOL, "scenario_id": scenario_id, "step": step, "sequence": sequence}
    step_dir = evidence_dir / "steps" / f"{sequence:03d}-{step}"
    write_json(step_dir / "request.json", request)
    value = invoke_control(
        control, config["control_executable_sha256"], config["step_timeout_seconds"],
        "--step", "--scenario", scenario_id, "--step-name", step, "--sequence", str(sequence),
    )
    expected = {"control_protocol", "scenario_id", "step", "sequence", "at_unix_micros", "kind", "actor", "action_hashes", "artifacts"}
    require(set(value) == expected, "control step response has missing or unknown fields")
    require(value["scenario_id"] == scenario_id and value["step"] == step and value["sequence"] == sequence, "control step response changed request identity")
    require(isinstance(value["at_unix_micros"], int) and value["at_unix_micros"] > 0, "control step timestamp must be positive")
    require_text(value["kind"], "step.kind", 256)
    require_text(value["actor"], "step.actor", 256)
    validate_action_hashes(value["action_hashes"], "step.action_hashes")
    write_artifacts(step_dir / "artifacts", value["artifacts"])
    write_json(step_dir / "response.json", value)
    return {"sequence": sequence, "kind": value["kind"], "actor": value["actor"], "at_unix_micros": value["at_unix_micros"]}


def observe(config: dict[str, Any], control: Path, scenario_id: str, conductor_id: str, expected_state: str, evidence_dir: Path) -> dict[str, Any]:
    value = invoke_control(
        control, config["control_executable_sha256"], config["step_timeout_seconds"],
        "--observe", "--scenario", scenario_id, "--conductor", conductor_id,
    )
    expected = {"control_protocol", "scenario_id", "conductor_id", "state", "observed_at_unix_micros", "action_hashes", "chain_head_action_hash", "source_chain_sequence", "integrated_op_count"}
    require(set(value) == expected, "control observation has missing or unknown fields")
    require(value["scenario_id"] == scenario_id and value["conductor_id"] == conductor_id, "control observation changed request identity")
    require(value["state"] == expected_state, f"{conductor_id} observed {value['state']}, expected {expected_state}")
    require(isinstance(value["observed_at_unix_micros"], int) and value["observed_at_unix_micros"] > 0, "observation timestamp must be positive")
    actions = validate_action_hashes(value["action_hashes"], "observation.action_hashes")
    head = require_text(value["chain_head_action_hash"], "chain_head_action_hash")
    require(head in actions, "chain head must appear in action_hashes")
    require(isinstance(value["source_chain_sequence"], int) and value["source_chain_sequence"] >= 0, "source_chain_sequence must be non-negative")
    require(isinstance(value["integrated_op_count"], int) and value["integrated_op_count"] >= 0, "integrated_op_count must be non-negative")
    observation = {
        "schema": FINAL_OBSERVATION_SCHEMA,
        "scenario_id": scenario_id,
        "conductor_id": conductor_id,
        "state": expected_state,
        "observed_at_unix_micros": value["observed_at_unix_micros"],
        "action_hashes": actions,
        "chain_head_action_hash": head,
        "source_chain_sequence": value["source_chain_sequence"],
        "integrated_op_count": value["integrated_op_count"],
    }
    relative = Path("observations") / f"{conductor_id}.json"
    write_json(evidence_dir / relative, observation)
    return {
        "conductor_id": conductor_id,
        "state": expected_state,
        "observed_at_unix_micros": value["observed_at_unix_micros"],
        "response_path": relative.as_posix(),
    }


def evidence_directory_size(root: Path) -> int:
    total = 0
    for path in root.rglob("*"):
        require(not path.is_symlink(), f"evidence may not contain symlinks: {path}")
        if path.is_file():
            total += path.stat().st_size
            require(total <= MAX_SCENARIO_EVIDENCE_BYTES, "scenario evidence exceeds total byte limit")
    return total


def run_scenario(config: dict[str, Any], config_path: Path, control: Path, contract_path: Path, contract: dict[str, Any], scenario_id: str, evidence_dir: Path) -> dict[str, Any]:
    # Recheck deployment identity before and after the complete scenario.
    control_capabilities(config, control)
    initial_runtime_hash = runtime_lock_hash(config, config_path, contract_path)
    scenario = scenario_by_id(contract, scenario_id)
    started = time.time_ns() // 1000
    events = []
    for index, step in enumerate(scenario["steps"]):
        events.append(run_step(config, control, scenario_id, index, step, evidence_dir))
        evidence_directory_size(evidence_dir)
    require(all(events[index]["at_unix_micros"] <= events[index + 1]["at_unix_micros"] for index in range(len(events) - 1)), "control events are not monotonic")
    observations = [observe(config, control, scenario_id, conductor["id"], scenario["expected_state"], evidence_dir) for conductor in config["conductors"]]
    completed = max([time.time_ns() // 1000, *[item["observed_at_unix_micros"] for item in observations]])
    evidence_directory_size(evidence_dir)
    require(runtime_lock_hash(config, config_path, contract_path) == initial_runtime_hash, "driver runtime lock changed during scenario")
    control_capabilities(config, control)
    write_json(evidence_dir / "driver-runtime-lock.json", {
        "schema": "mycelix-holochain-real-driver-runtime-lock-v1",
        "runtime_lock_hash": initial_runtime_hash,
        "config_file_sha256": file_hash(config_path),
        "control_executable_sha256": config["control_executable_sha256"],
        "scenario_contract_sha256": file_hash(contract_path),
    })
    return {
        "driver_protocol": DRIVER_PROTOCOL,
        "scenario_id": scenario_id,
        "observed_state": scenario["expected_state"],
        "started_at_unix_micros": started,
        "completed_at_unix_micros": completed,
        "events": events,
        "observations": observations,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--capabilities", action="store_true")
    modes.add_argument("--scenario")
    parser.add_argument("--evidence-dir")
    args = parser.parse_args()
    try:
        config, config_path, control, contract_path, contract = load_config()
        if args.capabilities:
            require(args.evidence_dir is None, "--capabilities does not accept --evidence-dir")
            print(json.dumps(capabilities(config, config_path, control, contract_path), sort_keys=True, separators=(",", ":")))
            return 0
        require(bool(args.evidence_dir), "--scenario requires --evidence-dir")
        evidence_dir = prepare_evidence_dir(args.evidence_dir)
        result = run_scenario(config, config_path, control, contract_path, contract, args.scenario, evidence_dir)
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
        return 0
    except (DriverError, OSError, ValueError) as error:
        print(f"holochain real driver: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
