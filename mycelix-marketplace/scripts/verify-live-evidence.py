#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = json.loads((ROOT / "frontend-leptos/bridge/package.json").read_text())
EXPECTED_CLIENT = PACKAGE["dependencies"]["@holochain/client"]
HEX64 = set("0123456789abcdef")


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise SystemExit(f"evidence receipt is not an object: {path}")
    return value


def digest_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(ch not in HEX64 for ch in value.lower()):
        raise SystemExit(f"{label} must be a lowercase SHA-256 digest")
    return value.lower()


def require_roles(receipt: dict[str, Any], roles: set[str]) -> None:
    inventories = receipt.get("active_roles")
    if not isinstance(inventories, dict) or not inventories:
        raise SystemExit(f"{receipt.get('scenario')}: active role inventory is missing")
    for actor, values in inventories.items():
        if not isinstance(values, list) or not roles.issubset(set(values)):
            raise SystemExit(
                f"{receipt.get('scenario')}: {actor} lacks required active roles {sorted(roles)}"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence_dir", type=Path)
    parser.add_argument("--profile", choices=["base", "settlement", "arbitration", "network"], required=True)
    parser.add_argument("--source-revision", required=True)
    parser.add_argument("--happ", type=Path, required=True)
    parser.add_argument("--marketplace-dna", type=Path, required=True)
    parser.add_argument("--finance-dna", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    required = ["lifecycle"]
    if args.profile == "settlement":
        required.append("settlement")
        if args.finance_dna is None:
            raise SystemExit("settlement promotion requires --finance-dna")
    if args.profile == "arbitration":
        required.append("arbitration")
    if args.profile == "network":
        required = ["network"]

    expected = {
        "source_revision": args.source_revision,
        "happ_sha256": digest_file(args.happ),
        "marketplace_dna_sha256": digest_file(args.marketplace_dna),
        "holochain_client_version": EXPECTED_CLIENT,
    }
    if args.finance_dna:
        expected["finance_dna_sha256"] = digest_file(args.finance_dna)

    receipts: dict[str, dict[str, Any]] = {}
    for scenario in required:
        path = args.evidence_dir / f"{scenario}.json"
        if not path.is_file():
            raise SystemExit(f"missing required live evidence: {path}")
        receipt = load(path)
        receipts[scenario] = receipt
        if receipt.get("schema_version") != 1:
            raise SystemExit(f"{scenario}: unsupported schema version")
        if receipt.get("scenario") != scenario or receipt.get("result") != "pass":
            raise SystemExit(f"{scenario}: receipt is not a passing {scenario} run")
        if receipt.get("fixture") is not False:
            raise SystemExit(f"{scenario}: fixture evidence cannot promote a release")
        for field, value in expected.items():
            if field == "finance_dna_sha256" and scenario != "settlement":
                continue
            if receipt.get(field) != value:
                raise SystemExit(
                    f"{scenario}: {field} does not match the promoted artifact"
                )
        require_sha(receipt["happ_sha256"], f"{scenario}.happ_sha256")
        require_sha(receipt["marketplace_dna_sha256"], f"{scenario}.marketplace_dna_sha256")
        require_roles(receipt, {"marketplace"})

    if args.profile != "network":
        lifecycle = receipts["lifecycle"].get("details", {})
        if lifecycle.get("delivered_revisions", 0) < 4 or lifecycle.get("cancellation_revisions", 0) < 2:
            raise SystemExit("lifecycle evidence does not prove delivery and cancellation revision chains")

    limitations: list[str] = []
    if args.profile == "settlement":
        settlement = receipts["settlement"]
        require_roles(settlement, {"marketplace", "finance"})
        details = settlement.get("details", {})
        if details.get("marketplace_status_after_settlement") != "delivered":
            raise SystemExit("settlement evidence rewrote Marketplace lifecycle finality")
        if details.get("fulfillment_events_for_transaction") != 1:
            raise SystemExit("settlement evidence did not deduplicate fulfillment reputation")
    if args.profile == "arbitration":
        arbitration = receipts["arbitration"].get("details", {})
        if arbitration.get("duplicate_vote_rejected") is not True:
            raise SystemExit("arbitration evidence did not reject a duplicate vote")
        if arbitration.get("idempotent_finalization") is not True:
            raise SystemExit("arbitration evidence did not prove idempotent finalization")
        if arbitration.get("conflict_injection_tested") is not True:
            limitations.append(
                "concurrent dispute-head injection is not proven; keep dispute UI gated"
            )
    if args.profile == "network":
        network = receipts["network"].get("details", {})
        if network.get("topology") != "two_conductor_isolated_network":
            raise SystemExit("network evidence used an unsupported topology")
        if network.get("conductor_count") != 2 or network.get("distinct_conductor_processes") is not True:
            raise SystemExit("network evidence does not prove two distinct conductor processes")
        if network.get("distinct_admin_endpoints") is not True:
            raise SystemExit("network evidence does not prove distinct admin endpoints")
        if network.get("service_isolation") != "local_controlled":
            raise SystemExit("network services were not locally controlled")
        for field in ["service_implementation_sha256", "control_hook_sha256", "topology_sha256"]:
            require_sha(network.get(field), f"network.details.{field}")
        for field in [
            "listing_propagation_ms",
            "transaction_propagation_ms",
            "safe_projection_visibility_ms",
            "unsafe_conflict_visibility_ms",
            "bilateral_approval_visibility_ms",
            "bilateral_resolution_visibility_ms",
        ]:
            value = network.get(field)
            if not isinstance(value, (int, float)) or value < 0 or value > 120000:
                raise SystemExit(f"network evidence has invalid {field}")
        if network.get("conflict_policy_version") != 2:
            raise SystemExit("network evidence used an unsupported conflict policy version")
        if network.get("partition_divergence_observed") is not True:
            raise SystemExit("network evidence did not observe divergent valid partition writes")

        if network.get("safe_terminal_dominance_observed") is not True:
            raise SystemExit("network evidence did not observe safe terminal dominance")
        if network.get("safe_projection_seen_by_all") is not True:
            raise SystemExit("network evidence did not expose the same safety projection on both peers")
        if network.get("safe_projection_state") != "auto_resolved":
            raise SystemExit("network safe case was not projected as auto_resolved")
        if network.get("safe_projection_reason") != "cancellation_dominates_pre_shipment":
            raise SystemExit("network safe case used an unexpected projection reason")
        if network.get("safe_canonical_status") != "cancelled":
            raise SystemExit("network safe case did not preserve the authored cancellation head")
        if sorted(network.get("safe_head_statuses", [])) != ["cancelled", "confirmed"]:
            raise SystemExit("network safe case does not contain confirmed/cancelled evidence")
        if network.get("safe_superseded_statuses") != ["confirmed"]:
            raise SystemExit("network safe case did not preserve confirmed as superseded evidence")
        safe_hashes = network.get("safe_head_hashes")
        if not isinstance(safe_hashes, list) or len(safe_hashes) != 2 or len(set(safe_hashes)) != 2:
            raise SystemExit("network safe-case head evidence is incomplete")

        if network.get("unsafe_conflict_observed") is not True or network.get("unsafe_conflict_seen_by_all") is not True:
            raise SystemExit("network evidence did not expose the same unsafe conflict after healing")
        if network.get("unsafe_arbitrary_winner_selected") is not False:
            raise SystemExit("network reducer selected an arbitrary winner for an unsafe conflict")
        if sorted(network.get("unsafe_head_statuses", [])) != ["cancelled", "shipped"]:
            raise SystemExit("network unsafe conflict does not contain shipped/cancelled heads")
        unsafe_hashes = network.get("unsafe_conflict_head_hashes")
        if not isinstance(unsafe_hashes, list) or len(unsafe_hashes) != 2 or len(set(unsafe_hashes)) != 2:
            raise SystemExit("network unsafe-conflict head evidence is incomplete")

        if network.get("bilateral_resolution_observed") is not True:
            raise SystemExit("network evidence did not exercise bilateral conflict authority")
        if network.get("bilateral_resolution_seen_by_all") is not True:
            raise SystemExit("network bilateral projection was not visible on both peers")
        if network.get("bilateral_resolution_state") != "authorized_resolved":
            raise SystemExit("network bilateral decision did not produce authorized_resolved")
        if network.get("bilateral_resolution_reason") != "bilateral_agreement":
            raise SystemExit("network bilateral decision used an unexpected authority reason")
        if network.get("bilateral_canonical_status") != "shipped":
            raise SystemExit("network bilateral decision did not select the approved shipped branch")
        bilateral_hashes = network.get("bilateral_head_hashes")
        if sorted(bilateral_hashes or []) != sorted(unsafe_hashes):
            raise SystemExit("network bilateral decision did not preserve the exact original head set")
        if network.get("bilateral_superseded_statuses") != ["cancelled"]:
            raise SystemExit("network bilateral decision did not preserve cancelled as superseded evidence")
        if not isinstance(network.get("bilateral_authority_count"), int) or network.get("bilateral_authority_count") < 1:
            raise SystemExit("network bilateral decision omitted its authority record")

    output = {
        "schema_version": 1,
        "profile": args.profile,
        "result": "promoted_with_limitations" if limitations else "promoted",
        "source_revision": args.source_revision,
        "artifacts": {
            "happ": {"path": str(args.happ), "sha256": expected["happ_sha256"]},
            "marketplace_dna": {
                "path": str(args.marketplace_dna),
                "sha256": expected["marketplace_dna_sha256"],
            },
            "finance_dna": None
            if not args.finance_dna
            else {
                "path": str(args.finance_dna),
                "sha256": expected["finance_dna_sha256"],
            },
        },
        "evidence": {name: digest_file(args.evidence_dir / f"{name}.json") for name in required},
        "limitations": limitations,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
