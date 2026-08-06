#!/usr/bin/env python3
"""Tests for the deterministic aggregate evidence index."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from evidence_manifest import canonical_json_bytes, contract_hash, file_sha256_b64, sha256_b64  # noqa: E402

CONTRACT_PATH = ROOT / "settlement_scenarios_v2.json"
INDEXER = ROOT / "release_evidence_index.py"


def digest(value: str) -> str:
    return sha256_b64(value.encode("utf-8"))


def create_evidence(root: Path) -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    started = 1_700_000_000_000_000
    conductors = [
        {
            "id": conductor,
            "agent_pub_key": f"uhCAk-{conductor}",
            "database_identity_hash": digest(f"db:{conductor}"),
            "network_identity_hash": digest(f"network:{conductor}"),
        }
        for conductor in ("verifier", "prover_a", "prover_b")
    ]
    for scenario in contract["scenarios"]:
        scenario_dir = root / scenario["id"]
        observation_dir = scenario_dir / "observations"
        observation_dir.mkdir(parents=True)
        files = []
        observations = []
        for offset, conductor in enumerate(conductors, 1):
            relative = f"observations/{conductor['id']}.json"
            path = scenario_dir / relative
            path.write_bytes(canonical_json_bytes({"state": scenario["expected_state"]}) + b"\n")
            file_hash = file_sha256_b64(path)
            files.append({"path": relative, "sha256": file_hash, "size": path.stat().st_size})
            observations.append(
                {
                    "conductor_id": conductor["id"],
                    "state": scenario["expected_state"],
                    "observed_at_unix_micros": started + offset,
                    "response_path": relative,
                    "response_sha256": file_hash,
                }
            )
        manifest = {
            "schema": "mycelix-conductor-evidence-manifest-v1",
            "scenario_contract_hash": contract_hash(CONTRACT_PATH),
            "scenario_id": scenario["id"],
            "expected_state": scenario["expected_state"],
            "observed_state": scenario["expected_state"],
            "dna_hash": "uhC0k-test-dna",
            "happ_hash": "uhC0m-test-happ",
            "adapter_release_hash": digest("adapter-release"),
            "started_at_unix_micros": started,
            "completed_at_unix_micros": started + 10,
            "conductors": conductors,
            "events": [
                {"sequence": 0, "kind": "start", "actor": "harness", "at_unix_micros": started},
                {"sequence": 1, "kind": "end", "actor": "harness", "at_unix_micros": started + 10},
            ],
            "observations": observations,
            "files": files,
        }
        (scenario_dir / "manifest.json").write_bytes(canonical_json_bytes(manifest) + b"\n")


class ReleaseEvidenceIndexTests(unittest.TestCase):
    def invoke(self, root: Path, *extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(INDEXER), "--evidence-root", str(root), *extra],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )

    def test_build_and_verify(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            create_evidence(root)
            built = self.invoke(root)
            self.assertEqual(built.returncode, 0, built.stderr)
            verified = self.invoke(root, "--verify")
            self.assertEqual(verified.returncode, 0, verified.stderr)
            self.assertEqual(
                [line for line in built.stdout.splitlines() if "hash_b64=" in line],
                [line for line in verified.stdout.splitlines() if "hash_b64=" in line],
            )

    def test_cross_scenario_release_substitution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            create_evidence(root)
            manifest_path = root / "policy-lease-expiry/manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["adapter_release_hash"] = digest("other-adapter")
            manifest_path.write_bytes(canonical_json_bytes(manifest) + b"\n")
            completed = self.invoke(root)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("different adapter releases", completed.stderr)


if __name__ == "__main__":
    unittest.main()
