#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CLIENT = json.loads((ROOT / "frontend-leptos/bridge/package.json").read_text())["dependencies"]["@holochain/client"]
SOURCE = "evidence-test-revision"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def receipt(scenario: str, happ: Path, marketplace: Path, finance: Path | None, details: dict) -> dict:
    roles = ["marketplace"] + (["finance"] if scenario == "settlement" else [])
    return {
        "schema_version": 1,
        "scenario": scenario,
        "result": "pass",
        "fixture": False,
        "generated_at": "2026-07-14T00:00:00.000Z",
        "source_revision": SOURCE,
        "happ_sha256": sha(happ),
        "marketplace_dna_sha256": sha(marketplace),
        "finance_dna_sha256": sha(finance) if finance and scenario == "settlement" else None,
        "holochain_client_version": CLIENT,
        "active_roles": {"agent": roles},
        "details": details,
    }


def verify(temp: Path, profile: str, happ: Path, marketplace: Path, finance: Path | None = None) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(ROOT / "scripts/verify-live-evidence.py"),
        str(temp / "evidence"),
        "--profile",
        profile,
        "--source-revision",
        SOURCE,
        "--happ",
        str(happ),
        "--marketplace-dna",
        str(marketplace),
        "--output",
        str(temp / f"promotion-{profile}.json"),
    ]
    if finance:
        command += ["--finance-dna", str(finance)]
    return subprocess.run(command, text=True, capture_output=True)


def main() -> None:
    with tempfile.TemporaryDirectory() as raw:
        temp = Path(raw)
        evidence = temp / "evidence"
        evidence.mkdir()
        happ = temp / "marketplace.happ"
        marketplace = temp / "marketplace.dna"
        finance = temp / "finance.dna"
        happ.write_bytes(b"happ")
        marketplace.write_bytes(b"marketplace")
        finance.write_bytes(b"finance")

        lifecycle = receipt(
            "lifecycle",
            happ,
            marketplace,
            None,
            {"delivered_revisions": 4, "cancellation_revisions": 2},
        )
        settlement = receipt(
            "settlement",
            happ,
            marketplace,
            finance,
            {
                "marketplace_status_after_settlement": "delivered",
                "fulfillment_events_for_transaction": 1,
            },
        )
        arbitration = receipt(
            "arbitration",
            happ,
            marketplace,
            None,
            {
                "duplicate_vote_rejected": True,
                "idempotent_finalization": True,
                "conflict_injection_tested": False,
            },
        )
        for name, value in [
            ("lifecycle", lifecycle),
            ("settlement", settlement),
            ("arbitration", arbitration),
        ]:
            (evidence / f"{name}.json").write_text(json.dumps(value))

        result = verify(temp, "settlement", happ, marketplace, finance)
        if result.returncode != 0:
            raise SystemExit(result.stderr or result.stdout)
        promoted = json.loads((temp / "promotion-settlement.json").read_text())
        assert promoted["result"] == "promoted"

        result = verify(temp, "arbitration", happ, marketplace)
        if result.returncode != 0:
            raise SystemExit(result.stderr or result.stdout)
        promoted = json.loads((temp / "promotion-arbitration.json").read_text())
        assert promoted["result"] == "promoted_with_limitations"
        assert promoted["limitations"]

        settlement["fixture"] = True
        (evidence / "settlement.json").write_text(json.dumps(settlement))
        rejected = verify(temp, "settlement", happ, marketplace, finance)
        assert rejected.returncode != 0
        assert "fixture evidence cannot promote" in (rejected.stderr + rejected.stdout)

    print("live evidence verifier tests passed")


if __name__ == "__main__":
    main()
