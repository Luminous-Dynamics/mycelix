#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CLIENT = json.loads((ROOT / "frontend-leptos/bridge/package.json").read_text())["dependencies"]["@holochain/client"]
SOURCE = "network-evidence-test-revision"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(temp: Path, happ: Path, dna: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/verify-live-evidence.py"),
            str(temp / "evidence"),
            "--profile",
            "network",
            "--source-revision",
            SOURCE,
            "--happ",
            str(happ),
            "--marketplace-dna",
            str(dna),
            "--output",
            str(temp / "promotion-network.json"),
        ],
        text=True,
        capture_output=True,
    )


def valid_receipt(happ: Path, dna: Path) -> dict:
    return {
        "schema_version": 1,
        "scenario": "network",
        "result": "pass",
        "fixture": False,
        "generated_at": "2026-07-15T00:00:00Z",
        "source_revision": SOURCE,
        "happ_sha256": sha(happ),
        "marketplace_dna_sha256": sha(dna),
        "finance_dna_sha256": None,
        "holochain_client_version": CLIENT,
        "active_roles": {"seller": ["marketplace"], "buyer": ["marketplace"]},
        "details": {
            "topology": "two_conductor_isolated_network",
            "conductor_count": 2,
            "distinct_conductor_processes": True,
            "distinct_admin_endpoints": True,
            "service_isolation": "local_controlled",
            "service_implementation_sha256": "a" * 64,
            "control_hook_sha256": "b" * 64,
            "topology_sha256": "c" * 64,
            "conflict_policy_version": 2,
            "listing_propagation_ms": 100,
            "transaction_propagation_ms": 200,
            "safe_projection_visibility_ms": 300,
            "unsafe_conflict_visibility_ms": 400,
            "partition_divergence_observed": True,
            "safe_terminal_dominance_observed": True,
            "safe_projection_seen_by_all": True,
            "safe_projection_state": "auto_resolved",
            "safe_projection_reason": "cancellation_dominates_pre_shipment",
            "safe_canonical_status": "cancelled",
            "safe_head_statuses": ["cancelled", "confirmed"],
            "safe_superseded_statuses": ["confirmed"],
            "safe_head_hashes": ["safe-a", "safe-b"],
            "unsafe_conflict_observed": True,
            "unsafe_conflict_seen_by_all": True,
            "unsafe_arbitrary_winner_selected": False,
            "unsafe_head_statuses": ["cancelled", "shipped"],
            "unsafe_conflict_head_hashes": ["unsafe-a", "unsafe-b"],
            "bilateral_resolution_observed": True,
            "bilateral_resolution_seen_by_all": True,
            "bilateral_resolution_state": "authorized_resolved",
            "bilateral_resolution_reason": "bilateral_agreement",
            "bilateral_canonical_status": "shipped",
            "bilateral_head_hashes": ["unsafe-a", "unsafe-b"],
            "bilateral_superseded_statuses": ["cancelled"],
            "bilateral_authority_count": 1,
            "bilateral_approval_visibility_ms": 450,
            "bilateral_resolution_visibility_ms": 500,
        },
    }


def main() -> None:
    with tempfile.TemporaryDirectory() as raw:
        temp = Path(raw)
        evidence = temp / "evidence"
        evidence.mkdir()
        happ = temp / "marketplace.happ"
        dna = temp / "marketplace.dna"
        happ.write_bytes(b"happ")
        dna.write_bytes(b"dna")
        receipt = valid_receipt(happ, dna)
        receipt_path = evidence / "network.json"
        receipt_path.write_text(json.dumps(receipt))
        passed = verify(temp, happ, dna)
        if passed.returncode != 0:
            raise SystemExit(passed.stderr or passed.stdout)
        promoted = json.loads((temp / "promotion-network.json").read_text())
        assert promoted["result"] == "promoted"

        receipt["details"]["unsafe_arbitrary_winner_selected"] = True
        receipt_path.write_text(json.dumps(receipt))
        rejected = verify(temp, happ, dna)
        assert rejected.returncode != 0
        assert "arbitrary winner" in (rejected.stderr + rejected.stdout)

        receipt = valid_receipt(happ, dna)
        receipt["details"]["safe_canonical_status"] = "confirmed"
        receipt_path.write_text(json.dumps(receipt))
        rejected = verify(temp, happ, dna)
        assert rejected.returncode != 0
        assert "authored cancellation" in (rejected.stderr + rejected.stdout)

        receipt = valid_receipt(happ, dna)
        receipt["details"]["unsafe_conflict_observed"] = False
        receipt_path.write_text(json.dumps(receipt))
        rejected = verify(temp, happ, dna)
        assert rejected.returncode != 0
        assert "same unsafe conflict" in (rejected.stderr + rejected.stdout)

        receipt = valid_receipt(happ, dna)
        receipt["details"]["conflict_policy_version"] = 3
        receipt_path.write_text(json.dumps(receipt))
        rejected = verify(temp, happ, dna)
        assert rejected.returncode != 0
        assert "unsupported conflict policy" in (rejected.stderr + rejected.stdout)

        receipt = valid_receipt(happ, dna)
        receipt["details"]["bilateral_resolution_state"] = "conflicted"
        receipt_path.write_text(json.dumps(receipt))
        rejected = verify(temp, happ, dna)
        assert rejected.returncode != 0
        assert "authorized_resolved" in (rejected.stderr + rejected.stdout)

        receipt = valid_receipt(happ, dna)
        receipt["details"]["bilateral_head_hashes"] = ["unsafe-a", "replacement"]
        receipt_path.write_text(json.dumps(receipt))
        rejected = verify(temp, happ, dna)
        assert rejected.returncode != 0
        assert "exact original head set" in (rejected.stderr + rejected.stdout)

    print("network evidence verifier tests passed")


if __name__ == "__main__":
    main()
