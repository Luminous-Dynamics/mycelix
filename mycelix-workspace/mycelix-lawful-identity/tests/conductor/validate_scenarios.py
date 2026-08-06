#!/usr/bin/env python3
"""Validate the machine-readable contract for the conductor evidence lane."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

PATH = Path(__file__).with_name("settlement_scenarios_v2.json")
ALLOWED_POSITIVE = {"PolicyAccepted"}
FAIL_CLOSED_STATES = {
    "AwaitingSettlement",
    "Submitted",
    "CredentialAnchorsPending",
    "ChallengeConsumptionConflict",
    "CredentialAnchorsExpired",
    "CredentialAnchorRefreshFork",
    "PolicyRenewalRequired",
    "PolicyRenewalFork",
    "PolicyExpired",
    "PolicyLineageRevoked",
}


class ContractError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def require_text(value: Any, name: str) -> str:
    require(isinstance(value, str) and bool(value), f"{name} must be non-empty text")
    return value


def main() -> int:
    try:
        data = json.loads(PATH.read_text(encoding="utf-8"))
        require(isinstance(data, dict), "scenario contract must be a JSON object")
        require(
            data.get("schema") == "mycelix-lawful-identity-conductor-scenarios-v2",
            "unexpected scenario contract schema",
        )
        agents = data.get("required_agents")
        require(isinstance(agents, list), "required_agents must be a list")
        require(set(agents) == {"verifier", "prover_a", "prover_b"}, "required agent set changed")

        invariant_values = data.get("required_invariants")
        require(isinstance(invariant_values, list), "required_invariants must be a list")
        required = set(invariant_values)
        require(bool(required), "required invariant set is empty")
        require(len(required) == len(invariant_values), "required invariants contain duplicates")

        scenario_values = data.get("scenarios")
        require(isinstance(scenario_values, list) and bool(scenario_values), "scenarios must be a non-empty list")
        seen_ids: set[str] = set()
        covered: set[str] = set()
        for index, scenario_value in enumerate(scenario_values):
            require(isinstance(scenario_value, dict), f"scenarios[{index}] must be an object")
            scenario_id = require_text(scenario_value.get("id"), f"scenarios[{index}].id")
            require(scenario_id not in seen_ids, f"duplicate scenario id: {scenario_id}")
            seen_ids.add(scenario_id)
            steps = scenario_value.get("steps")
            require(isinstance(steps, list) and bool(steps), f"scenario has no steps: {scenario_id}")
            require(all(isinstance(step, str) and step for step in steps), f"scenario has invalid steps: {scenario_id}")
            assertion_values = scenario_value.get("assertions")
            require(isinstance(assertion_values, list) and bool(assertion_values), f"scenario has no assertions: {scenario_id}")
            assertions = set(assertion_values)
            require(len(assertions) == len(assertion_values), f"scenario repeats assertions: {scenario_id}")
            require(assertions <= required, f"unknown invariant in {scenario_id}")
            covered |= assertions
            expected = require_text(scenario_value.get("expected_state"), f"{scenario_id}.expected_state")
            require(
                expected in ALLOWED_POSITIVE | FAIL_CLOSED_STATES,
                f"unclassified expected state in {scenario_id}: {expected}",
            )
            if "late-gossip-overrides-positive-state" in assertions:
                require(expected == "ChallengeConsumptionConflict", f"late gossip scenario is not fail-closed: {scenario_id}")
            if "policy-revocation-terminates-lineage" in assertions:
                require(expected == "PolicyLineageRevoked", f"revocation scenario has wrong state: {scenario_id}")
            if "ancestor-revocation-terminates-descendants" in assertions:
                require(expected == "PolicyLineageRevoked", f"ancestor revocation scenario has wrong state: {scenario_id}")

        missing = required - covered
        require(not missing, f"required invariants lack a scenario: {sorted(missing)}")
        print(f"conductor scenario contract v2: PASS ({len(seen_ids)} scenarios)")
        return 0
    except (ContractError, OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        print(f"conductor scenario contract v2: FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
