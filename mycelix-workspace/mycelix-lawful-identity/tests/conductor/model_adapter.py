#!/usr/bin/env python3
"""Executable model adapter for the conductor scenario-runner protocol.

This adapter is deliberately labeled `model`; it can validate orchestration and
expected state names, but it never satisfies `--require-real`.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MODEL_DIR = ROOT.parent / "protocol_model"
sys.path.insert(0, str(MODEL_DIR))
from evidence_manifest import contract_hash, file_sha256_b64  # noqa: E402
from settlement_conflict_model import (  # noqa: E402
    EXPECTED_REQUEST,
    EXPECTED_RESULT,
    SETTLEMENT,
    Model,
    accepted_model,
)


def require_model(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def verified_model() -> Model:
    model = Model(now=99)
    model.submit("A")
    model.now = SETTLEMENT
    require_model(model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT), "verification receipt setup failed")
    return model


def run_scenario(scenario_id: str) -> str:
    if scenario_id == "pre-settlement-refusal":
        model = Model(now=99)
        model.submit("A")
        model.now = SETTLEMENT - 1
    elif scenario_id == "duplicate-link-idempotence":
        model = Model(now=99)
        model.submit("A")
        model.duplicate_link("A")
        model.now = SETTLEMENT
        require_model(model.record_receipt("A", EXPECTED_REQUEST, EXPECTED_RESULT), "verification receipt setup failed")
    elif scenario_id == "partitioned-competing-proofs":
        model = Model(now=99)
        model.submit("A")
        model.submit("B")
        model.now = SETTLEMENT
    elif scenario_id == "late-gossip-after-acceptance":
        model = accepted_model()
        model.visible_proof_links.append("B")
    elif scenario_id == "credential-anchor-expiry":
        model = verified_model()
        require_model(model.record_anchor("A", "anchor-1", None, 1), "credential-anchor setup failed")
        model.now += 1
    elif scenario_id == "policy-lease-expiry":
        model = accepted_model(anchor_valid_for=300, lease=1)
        model.now += 1
    elif scenario_id == "emergency-policy-revocation":
        model = accepted_model()
        require_model(model.revoke_policy("decision-1"), "policy revocation setup failed")
    elif scenario_id == "anchor-refresh-renewal":
        model = accepted_model(anchor_valid_for=60, lease=30)
        model.now += 31
        require_model(model.record_anchor("A", "anchor-2", "anchor-1", 300), "anchor refresh setup failed")
        require_model(model.record_policy("A", "decision-2", "decision-1", "anchor-2", 120), "policy renewal setup failed")
    elif scenario_id == "anchor-refresh-suspends-old-policy":
        model = accepted_model(anchor_valid_for=600)
        require_model(model.record_anchor("A", "anchor-2", "anchor-1", 300), "anchor refresh setup failed")
    elif scenario_id == "partitioned-anchor-refresh-fork":
        model = accepted_model(anchor_valid_for=600)
        require_model(model.record_anchor("A", "anchor-2a", "anchor-1", 300), "first partitioned anchor refresh failed")
        require_model(
            model.record_anchor(
                "A", "anchor-2b", "anchor-1", 300, force_partition_write=True
            ),
            "second partitioned anchor refresh failed",
        )
    elif scenario_id == "partitioned-policy-renewal-fork":
        model = accepted_model(anchor_valid_for=600)
        require_model(model.record_policy("A", "decision-2a", "decision-1", "anchor-1", 120), "first partitioned policy renewal failed")
        require_model(
            model.record_policy(
                "A",
                "decision-2b",
                "decision-1",
                "anchor-1",
                120,
                force_partition_write=True,
            ),
            "second partitioned policy renewal failed",
        )
    elif scenario_id == "ancestor-revocation-after-renewal":
        model = accepted_model(anchor_valid_for=600)
        require_model(model.record_policy("A", "decision-2", "decision-1", "anchor-1", 120), "policy renewal setup failed")
        require_model(model.revoke_policy("decision-1"), "policy revocation setup failed")
    else:
        raise ValueError(f"unsupported scenario: {scenario_id}")
    return model.effective_state("A")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capabilities", action="store_true")
    parser.add_argument("--scenario")
    args = parser.parse_args()
    if args.capabilities:
        print(json.dumps({
            "adapter_protocol": "mycelix-conductor-scenario-adapter-v2",
            "adapter_kind": "model",
            "conductor_count": 0,
            "scenario_contract_hash": contract_hash(ROOT / "settlement_scenarios_v2.json"),
            "adapter_release_hash": file_sha256_b64(Path(__file__).resolve()),
        }, sort_keys=True))
        return 0
    if not args.scenario:
        parser.error("--scenario is required unless --capabilities is used")
    observed = run_scenario(args.scenario)
    print(json.dumps({
        "adapter_protocol": "mycelix-conductor-scenario-adapter-v2",
        "adapter_kind": "model",
        "scenario_id": args.scenario,
        "observed_state": observed,
        "evidence": {"level": "protocol-model"},
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
