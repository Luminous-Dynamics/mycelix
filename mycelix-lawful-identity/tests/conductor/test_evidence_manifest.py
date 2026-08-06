#!/usr/bin/env python3
"""Tamper and confinement tests for retained real-conductor evidence."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from evidence_manifest import (  # noqa: E402
    EvidenceError,
    contract_hash,
    file_sha256_b64,
    load_and_validate_manifest,
    manifest_hash,
    sha256_b64,
)

CONTRACT = ROOT / "settlement_scenarios_v2.json"
SCENARIO_ID = "pre-settlement-refusal"
EXPECTED_STATE = "AwaitingSettlement"
DNA_HASH = "uhC0k-test-dna-hash"


def digest(label: str) -> str:
    return sha256_b64(label.encode("utf-8"))


def write_fixture(root: Path) -> tuple[Path, dict]:
    scenario_dir = root / SCENARIO_ID
    observations_dir = scenario_dir / "observations"
    observations_dir.mkdir(parents=True)
    conductors = []
    observations = []
    files = []
    started = 1_700_000_000_000_000
    completed = started + 10_000
    for offset, conductor_id in enumerate(("verifier", "prover_a", "prover_b"), 1):
        response = {
            "conductor_id": conductor_id,
            "observed_state": EXPECTED_STATE,
            "observed_at_unix_micros": started + offset * 1_000,
        }
        relative = Path("observations") / f"{conductor_id}.json"
        response_path = scenario_dir / relative
        response_path.write_text(
            json.dumps(response, sort_keys=True, separators=(",", ":")),
            encoding="utf-8",
        )
        response_hash = file_sha256_b64(response_path)
        conductors.append(
            {
                "id": conductor_id,
                "agent_pub_key": f"uhCAk-{conductor_id}",
                "database_identity_hash": digest(f"database:{conductor_id}"),
                "network_identity_hash": digest(f"network:{conductor_id}"),
            }
        )
        observations.append(
            {
                "conductor_id": conductor_id,
                "state": EXPECTED_STATE,
                "observed_at_unix_micros": response["observed_at_unix_micros"],
                "response_path": relative.as_posix(),
                "response_sha256": response_hash,
            }
        )
        files.append(
            {
                "path": relative.as_posix(),
                "sha256": response_hash,
                "size": response_path.stat().st_size,
            }
        )
    manifest = {
        "schema": "mycelix-conductor-evidence-manifest-v1",
        "scenario_contract_hash": contract_hash(CONTRACT),
        "scenario_id": SCENARIO_ID,
        "expected_state": EXPECTED_STATE,
        "observed_state": EXPECTED_STATE,
        "dna_hash": DNA_HASH,
        "happ_hash": "uhC0m-test-happ-hash",
        "adapter_release_hash": digest("adapter-release"),
        "started_at_unix_micros": started,
        "completed_at_unix_micros": completed,
        "conductors": conductors,
        "events": [
            {
                "sequence": 0,
                "kind": "scenario-start",
                "actor": "harness",
                "at_unix_micros": started,
            },
            {
                "sequence": 1,
                "kind": "final-observation",
                "actor": "harness",
                "at_unix_micros": completed,
            },
        ],
        "observations": observations,
        "files": files,
    }
    manifest_path = scenario_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return manifest_path, manifest


class EvidenceManifestTests(unittest.TestCase):
    def validate(self, manifest_path: Path, root: Path) -> dict:
        _, summary = load_and_validate_manifest(
            manifest_path,
            root,
            expected_contract_hash=contract_hash(CONTRACT),
            expected_scenario_id=SCENARIO_ID,
            expected_state=EXPECTED_STATE,
            expected_dna_hash=DNA_HASH,
        )
        return summary

    def test_valid_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, manifest = write_fixture(root)
            summary = self.validate(manifest_path, root)
            self.assertEqual(summary["manifest_hash"], manifest_hash(manifest))
            self.assertEqual(set(summary["conductor_ids"]), {"verifier", "prover_a", "prover_b"})

    def test_tampered_evidence_file_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, _ = write_fixture(root)
            (manifest_path.parent / "observations/verifier.json").write_text("tampered", encoding="utf-8")
            with self.assertRaisesRegex(EvidenceError, "size mismatch|hash mismatch"):
                self.validate(manifest_path, root)

    def test_unlisted_file_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, _ = write_fixture(root)
            (manifest_path.parent / "unlisted.log").write_text("hidden evidence", encoding="utf-8")
            with self.assertRaisesRegex(EvidenceError, "missing or unlisted files"):
                self.validate(manifest_path, root)

    def test_path_traversal_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, manifest = write_fixture(root)
            manifest["files"][0]["path"] = "../outside.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(EvidenceError, "may not contain"):
                self.validate(manifest_path, root)

    def test_duplicate_conductor_identity_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, manifest = write_fixture(root)
            manifest["conductors"][1]["database_identity_hash"] = manifest["conductors"][0]["database_identity_hash"]
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(EvidenceError, "distinct database identities"):
                self.validate(manifest_path, root)

    def test_missing_final_observation_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, manifest = write_fixture(root)
            manifest["observations"].pop()
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(EvidenceError, "every conductor"):
                self.validate(manifest_path, root)

    def test_contract_substitution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, manifest = write_fixture(root)
            manifest["scenario_contract_hash"] = digest("other-contract")
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(EvidenceError, "different scenario contract"):
                self.validate(manifest_path, root)

    def test_manifest_outside_evidence_root_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, tempfile.TemporaryDirectory() as outside:
            root = Path(temporary)
            manifest_path, _ = write_fixture(Path(outside))
            with self.assertRaisesRegex(EvidenceError, "escapes evidence root"):
                self.validate(manifest_path, root)


if __name__ == "__main__":
    unittest.main()
