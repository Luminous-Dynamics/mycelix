#!/usr/bin/env python3
"""Create or verify a canonical runtime configuration for the real driver."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from holochain_real_driver import (
    CONFIG_SCHEMA,
    CONTROL_PROTOCOL,
    DriverError,
    canonical_json_bytes,
    checked_path,
    control_capabilities,
    file_hash,
    invoke_control,
    load_config,
    require,
    runtime_lock_hash,
    validate_conductors,
)


def create(control_value: Path, contract_value: Path, output: Path, timeout: int) -> dict:
    require(1 <= timeout <= 1800, "step timeout must be 1-1800 seconds")
    control = checked_path(str(control_value), "control executable", executable=True)
    contract = checked_path(str(contract_value), "scenario contract")
    require(not output.exists() and not output.is_symlink(), "driver configuration output already exists")
    capabilities = invoke_control(control, file_hash(control), timeout, "--capabilities")
    expected = {"control_protocol", "holochain_version", "dna_hash", "happ_hash", "agents", "conductors"}
    require(set(capabilities) == expected, "control capabilities have missing or unknown fields")
    require(capabilities["control_protocol"] == CONTROL_PROTOCOL, "unsupported control protocol")
    conductors = validate_conductors(capabilities["conductors"])
    agents = capabilities["agents"]
    require(isinstance(agents, list) and all(isinstance(agent, str) and agent for agent in agents), "agents must be non-empty strings")
    config = {
        "schema": CONFIG_SCHEMA,
        "control_protocol": CONTROL_PROTOCOL,
        "control_executable": str(control),
        "control_executable_sha256": file_hash(control),
        "scenario_contract": str(contract),
        "scenario_contract_sha256": file_hash(contract),
        "holochain_version": capabilities["holochain_version"],
        "dna_hash": capabilities["dna_hash"],
        "happ_hash": capabilities["happ_hash"],
        "agents": agents,
        "conductors": conductors,
        "step_timeout_seconds": timeout,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    require(not temporary.exists() and not temporary.is_symlink(), "temporary configuration path already exists")
    temporary.write_bytes(canonical_json_bytes(config) + b"\n")
    os.chmod(temporary, 0o444)
    os.replace(temporary, output)
    return config


def verify(config_path: Path) -> tuple[dict, str]:
    previous = os.environ.get("MYCELIX_HOLOCHAIN_DRIVER_CONFIG")
    os.environ["MYCELIX_HOLOCHAIN_DRIVER_CONFIG"] = str(config_path)
    try:
        config, resolved_config, control, contract_path, _ = load_config()
        control_capabilities(config, control)
        return config, runtime_lock_hash(config, resolved_config, contract_path)
    finally:
        if previous is None:
            os.environ.pop("MYCELIX_HOLOCHAIN_DRIVER_CONFIG", None)
        else:
            os.environ["MYCELIX_HOLOCHAIN_DRIVER_CONFIG"] = previous


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--create", action="store_true")
    mode.add_argument("--verify", action="store_true")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--control", type=Path)
    parser.add_argument("--scenario-contract", type=Path)
    parser.add_argument("--step-timeout-seconds", type=int, default=900)
    args = parser.parse_args()
    try:
        if args.create:
            require(args.control is not None and args.scenario_contract is not None, "--create requires --control and --scenario-contract")
            create(args.control, args.scenario_contract, args.config, args.step_timeout_seconds)
        else:
            require(args.control is None and args.scenario_contract is None, "--verify does not accept --control or --scenario-contract")
        config, lock_hash = verify(args.config)
        print("holochain driver config: PASS")
        print(f"runtime_lock_hash={lock_hash}")
        print(f"holochain_version={config['holochain_version']}")
        print(f"conductor_count={len(config['conductors'])}")
        return 0
    except (DriverError, OSError, ValueError) as error:
        print(f"holochain driver config: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
