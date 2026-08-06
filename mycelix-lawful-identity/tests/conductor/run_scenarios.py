#!/usr/bin/env python3
"""Run the scenario contract through a model or real-conductor adapter.

The release path deliberately avoids Python ``assert`` so ``python -O`` cannot
silently disable a security gate. Adapter processes are time- and output-bounded,
and malformed or ambiguous JSON fails closed.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from evidence_manifest import (
    EvidenceError,
    confined_relative_path,
    contract_hash,
    load_and_validate_manifest,
    require_hash32,
)

ROOT = Path(__file__).resolve().parent
SCENARIOS = ROOT / "settlement_scenarios_v2.json"
PROTOCOL = "mycelix-conductor-scenario-adapter-v2"
MAX_ADAPTER_STDOUT_BYTES = 64 * 1024
DEFAULT_TIMEOUT_SECONDS = 120


class GateError(RuntimeError):
    """A fail-closed scenario or adapter validation error."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise GateError(message)


def require_object(value: Any, description: str) -> dict[str, Any]:
    require(isinstance(value, dict), f"{description} must be a JSON object")
    return value


def invoke(adapter: Path, timeout_seconds: int, *args: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            [str(adapter), *args],
            check=False,
            capture_output=True,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired as error:
        raise GateError(
            f"adapter timed out after {timeout_seconds}s: {adapter} {' '.join(args)}"
        ) from error
    except OSError as error:
        raise GateError(f"failed to execute adapter {adapter}: {error}") from error

    if len(completed.stdout) > MAX_ADAPTER_STDOUT_BYTES:
        raise GateError(
            f"adapter stdout exceeds {MAX_ADAPTER_STDOUT_BYTES} bytes"
        )
    stderr = completed.stderr.decode("utf-8", errors="replace").strip()
    if completed.returncode != 0:
        detail = f": {stderr}" if stderr else ""
        raise GateError(
            f"adapter exited {completed.returncode} for {' '.join(args)}{detail}"
        )
    try:
        stdout = completed.stdout.decode("utf-8", errors="strict")
    except UnicodeDecodeError as error:
        raise GateError("adapter stdout must be valid UTF-8") from error
    try:
        payload = json.loads(stdout)
    except json.JSONDecodeError as error:
        raise GateError(f"adapter stdout is not one valid JSON value: {error}") from error
    return require_object(payload, "adapter response")


def require_text(payload: dict[str, Any], key: str) -> str:
    value = payload.get(key)
    require(isinstance(value, str) and bool(value), f"{key} must be non-empty text")
    return value


def validate_capabilities(
    capabilities: dict[str, Any],
    require_real: bool,
    expected_contract_hash: str,
) -> tuple[str, str | None]:
    require_text(capabilities, "adapter_protocol")
    require(
        capabilities["adapter_protocol"] == PROTOCOL,
        f"unsupported adapter protocol: {capabilities['adapter_protocol']!r}",
    )
    require(
        require_hash32(capabilities.get("scenario_contract_hash"), "scenario_contract_hash")
        == expected_contract_hash,
        "adapter is bound to a different scenario contract",
    )
    kind = require_text(capabilities, "adapter_kind")
    require(kind in {"model", "real-conductor"}, f"unsupported adapter kind: {kind}")
    count = capabilities.get("conductor_count")
    require(isinstance(count, int) and count >= 0, "conductor_count must be a non-negative integer")
    dna_hash: str | None = None
    if require_real:
        require(kind == "real-conductor", "release evidence requires adapter_kind=real-conductor")
        require(count >= 3, "real-conductor evidence requires at least three conductors")
        dna_hash = require_text(capabilities, "dna_hash")
        require_text(capabilities, "happ_hash")
        require_hash32(capabilities.get("adapter_release_hash"), "adapter_release_hash")
        agents = capabilities.get("agents")
        require(isinstance(agents, list), "agents must be a list")
        require(
            all(isinstance(agent, str) and agent for agent in agents),
            "agents must contain only non-empty strings",
        )
        require(
            set(agents) >= {"verifier", "prover_a", "prover_b"},
            "real-conductor adapter is missing a required agent",
        )
        require_hash32(capabilities.get("deployment_manifest_hash"), "deployment_manifest_hash")
    return kind, dna_hash


def validate_scenario_result(
    result: dict[str, Any],
    adapter_kind: str,
    scenario: dict[str, Any],
    require_real: bool,
    *,
    expected_contract_hash: str,
    expected_dna_hash: str | None,
    evidence_root: Path | None,
) -> None:
    require(
        require_text(result, "adapter_protocol") == PROTOCOL,
        "scenario response uses the wrong adapter protocol",
    )
    require(
        require_text(result, "adapter_kind") == adapter_kind,
        "scenario response changed adapter_kind",
    )
    require(
        require_text(result, "scenario_id") == scenario["id"],
        "scenario response changed scenario_id",
    )
    observed = require_text(result, "observed_state")
    require(
        observed == scenario["expected_state"],
        f"{scenario['id']}: expected {scenario['expected_state']}, observed {observed}",
    )
    if require_real:
        require(evidence_root is not None, "real evidence requires --evidence-root")
        evidence = require_object(result.get("evidence"), "scenario evidence")
        require(evidence.get("level") == "multi-conductor", "real evidence level must be multi-conductor")
        returned_bundle_hash = require_hash32(evidence.get("bundle_hash"), "bundle_hash")
        returned_manifest_hash = require_hash32(evidence.get("manifest_hash"), "manifest_hash")
        require(returned_bundle_hash == returned_manifest_hash, "bundle_hash must equal the canonical manifest hash")
        relative_manifest = confined_relative_path(evidence.get("manifest_path"), "manifest_path")
        manifest_path = evidence_root.joinpath(*relative_manifest.parts)
        _, summary = load_and_validate_manifest(
            manifest_path,
            evidence_root,
            expected_contract_hash=expected_contract_hash,
            expected_scenario_id=scenario["id"],
            expected_state=scenario["expected_state"],
            expected_dna_hash=expected_dna_hash,
        )
        require(summary["manifest_hash"] == returned_manifest_hash, "manifest hash does not match retained evidence")
        observed_by = evidence.get("observed_by")
        require(isinstance(observed_by, list) and bool(observed_by), "observed_by must be a non-empty list")
        require(
            all(isinstance(observer, str) and observer for observer in observed_by),
            "observed_by must contain only non-empty strings",
        )
        require(set(observed_by) == set(summary["conductor_ids"]), "observed_by does not cover the retained conductor set")


def load_contract() -> dict[str, Any]:
    try:
        contract = json.loads(SCENARIOS.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise GateError(f"failed to load scenario contract: {error}") from error
    contract = require_object(contract, "scenario contract")
    scenarios = contract.get("scenarios")
    require(isinstance(scenarios, list) and bool(scenarios), "scenario contract must contain scenarios")
    return contract


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--adapter", type=Path, required=True)
    parser.add_argument("--require-real", action="store_true")
    parser.add_argument("--timeout-seconds", type=int, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--evidence-root", type=Path)
    args = parser.parse_args()
    try:
        require(args.timeout_seconds > 0, "timeout must be positive")
        adapter = args.adapter.resolve(strict=True)
        require(adapter.is_file(), f"adapter is not a file: {adapter}")
        require(adapter.stat().st_mode & 0o111 != 0, f"adapter is not executable: {adapter}")
        evidence_root = None
        if args.evidence_root is not None:
            evidence_root = args.evidence_root.resolve(strict=True)
            require(evidence_root.is_dir(), "evidence root must be a directory")
        require(not args.require_real or evidence_root is not None, "--require-real also requires --evidence-root")

        contract = load_contract()
        expected_contract_hash = contract_hash(SCENARIOS)
        capabilities = invoke(adapter, args.timeout_seconds, "--capabilities")
        adapter_kind, expected_dna_hash = validate_capabilities(
            capabilities, args.require_real, expected_contract_hash
        )
        scenarios = contract["scenarios"]
        for scenario_value in scenarios:
            scenario = require_object(scenario_value, "scenario")
            scenario_id = require_text(scenario, "id")
            require_text(scenario, "expected_state")
            result = invoke(
                adapter,
                args.timeout_seconds,
                "--scenario",
                scenario_id,
            )
            validate_scenario_result(
                result,
                adapter_kind,
                scenario,
                args.require_real,
                expected_contract_hash=expected_contract_hash,
                expected_dna_hash=expected_dna_hash,
                evidence_root=evidence_root,
            )
        print(
            f"scenario adapter: PASS ({len(scenarios)} scenarios; kind={adapter_kind})"
        )
        return 0
    except (GateError, EvidenceError) as error:
        print(f"scenario adapter: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
