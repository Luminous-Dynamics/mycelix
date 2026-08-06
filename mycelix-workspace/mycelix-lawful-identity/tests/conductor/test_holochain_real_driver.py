#!/usr/bin/env python3
from __future__ import annotations

import base64
import hashlib
import json
import os
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DRIVER = ROOT / "holochain_real_driver.py"
CONTRACT = ROOT / "settlement_scenarios_v2.json"
RELEASE_HASH = base64.b64encode(hashlib.sha256(b"reviewed-release").digest()).decode("ascii")


def hash_file(path: Path) -> str:
    return base64.b64encode(hashlib.sha256(path.read_bytes()).digest()).decode("ascii")


def digest(label: str) -> str:
    return base64.b64encode(hashlib.sha256(label.encode()).digest()).decode("ascii")


CONTROL_SOURCE = r'''#!/usr/bin/env python3
import argparse, base64, json, os, sys, time
P = "mycelix-holochain-control-v1"
conductors = [
  {"id":"verifier","agent_pub_key":"agent-verifier","database_identity_hash":"%s","network_identity_hash":"%s"},
  {"id":"prover_a","agent_pub_key":"agent-a","database_identity_hash":"%s","network_identity_hash":"%s"},
  {"id":"prover_b","agent_pub_key":"agent-b","database_identity_hash":"%s","network_identity_hash":"%s"},
]
parser=argparse.ArgumentParser(); mode=parser.add_mutually_exclusive_group(required=True)
mode.add_argument("--capabilities", action="store_true"); mode.add_argument("--step", action="store_true"); mode.add_argument("--observe", action="store_true")
parser.add_argument("--scenario"); parser.add_argument("--step-name"); parser.add_argument("--sequence", type=int); parser.add_argument("--conductor")
a=parser.parse_args(); now=1700000000000000 + (a.sequence or 0)
if a.capabilities:
 print(json.dumps({"control_protocol":P,"holochain_version":"0.6.1-test","dna_hash":"dna-test","happ_hash":"happ-test","agents":["verifier","prover_a","prover_b"],"conductors":conductors},sort_keys=True,separators=(",",":"))); sys.exit(0)
mode=os.environ.get("FAKE_CONTROL_MODE", "ok")
if a.step:
 mutate=os.environ.get("FAKE_MUTATE_CONTRACT")
 if mutate: open(mutate, "a", encoding="utf-8").write("\n")
 artifacts=[]
 if mode == "path-traversal": artifacts=[{"path":"../escape","content_b64":base64.b64encode(b"x").decode()}]
 value={"control_protocol":P,"scenario_id":a.scenario,"step":a.step_name,"sequence":a.sequence,"at_unix_micros":now,"kind":"scenario-step","actor":"harness","action_hashes":[f"action-{a.sequence}"],"artifacts":artifacts}
 if mode == "unknown-step-field": value["unexpected"]=True
 print(json.dumps(value,sort_keys=True,separators=(",",":"))); sys.exit(0)
state=os.environ.get("FAKE_EXPECTED_STATE", "AwaitingSettlement")
if mode == "divergent" and a.conductor == "prover_b": state="Submitted"
head=f"head-{a.conductor}"
print(json.dumps({"control_protocol":P,"scenario_id":a.scenario,"conductor_id":a.conductor,"state":state,"observed_at_unix_micros":now+100,"action_hashes":[head],"chain_head_action_hash":head,"source_chain_sequence":1,"integrated_op_count":3},sort_keys=True,separators=(",",":")))
''' % tuple(digest(value) for value in ["db-v","net-v","db-a","net-a","db-b","net-b"])


class HolochainRealDriverTests(unittest.TestCase):
    def fixture(self, root: Path) -> tuple[Path, dict[str, str]]:
        control = root / "control.py"
        control.write_text(CONTROL_SOURCE, encoding="utf-8")
        control.chmod(0o755)
        config = {
            "schema": "mycelix-holochain-real-driver-config-v1",
            "control_protocol": "mycelix-holochain-control-v1",
            "control_executable": str(control),
            "control_executable_sha256": hash_file(control),
            "scenario_contract": str(CONTRACT),
            "scenario_contract_sha256": hash_file(CONTRACT),
            "holochain_version": "0.6.1-test",
            "dna_hash": "dna-test",
            "happ_hash": "happ-test",
            "agents": ["verifier", "prover_a", "prover_b"],
            "conductors": [
                {"id":"verifier","agent_pub_key":"agent-verifier","database_identity_hash":digest("db-v"),"network_identity_hash":digest("net-v")},
                {"id":"prover_a","agent_pub_key":"agent-a","database_identity_hash":digest("db-a"),"network_identity_hash":digest("net-a")},
                {"id":"prover_b","agent_pub_key":"agent-b","database_identity_hash":digest("db-b"),"network_identity_hash":digest("net-b")},
            ],
            "step_timeout_seconds": 10,
        }
        config_path = root / "driver-config.json"
        config_path.write_text(json.dumps(config, sort_keys=True, separators=(",", ":")), encoding="utf-8")
        env = os.environ.copy()
        env["MYCELIX_HOLOCHAIN_DRIVER_CONFIG"] = str(config_path)
        env["MYCELIX_DRIVER_RELEASE_HASH"] = RELEASE_HASH
        return control, env

    def invoke(self, env: dict[str, str], *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([str(DRIVER), *args], env=env, text=True, capture_output=True, timeout=30, check=False)

    def test_capabilities_bind_runtime_lock(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            _, env = self.fixture(Path(temporary))
            result = self.invoke(env, "--capabilities")
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(result.stdout)
            self.assertEqual(value["driver_release_hash"], RELEASE_HASH)
            self.assertEqual(len(base64.b64decode(value["runtime_lock_hash"], validate=True)), 32)

    def test_scenario_is_orchestrated_and_evidence_is_retained(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); _, env = self.fixture(root)
            evidence = root / "evidence"; evidence.mkdir(mode=0o700)
            env["FAKE_EXPECTED_STATE"] = "AwaitingSettlement"
            result = self.invoke(env, "--scenario", "pre-settlement-refusal", "--evidence-dir", str(evidence))
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(result.stdout)
            self.assertEqual(value["observed_state"], "AwaitingSettlement")
            self.assertEqual({item["conductor_id"] for item in value["observations"]}, {"verifier", "prover_a", "prover_b"})
            self.assertTrue((evidence / "steps/000-issue/request.json").is_file())
            self.assertTrue((evidence / "observations/verifier.json").is_file())
            self.assertTrue((evidence / "driver-runtime-lock.json").is_file())

    def test_control_executable_substitution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); control, env = self.fixture(root)
            control.write_text(control.read_text() + "\n# modified\n")
            result = self.invoke(env, "--capabilities")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("hash mismatch", result.stderr)

    def test_divergent_conductor_state_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); _, env = self.fixture(root)
            evidence = root / "evidence"; evidence.mkdir(mode=0o700)
            env["FAKE_EXPECTED_STATE"] = "AwaitingSettlement"; env["FAKE_CONTROL_MODE"] = "divergent"
            result = self.invoke(env, "--scenario", "pre-settlement-refusal", "--evidence-dir", str(evidence))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("prover_b observed Submitted", result.stderr)

    def test_control_artifact_path_traversal_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); _, env = self.fixture(root)
            evidence = root / "evidence"; evidence.mkdir(mode=0o700)
            env["FAKE_EXPECTED_STATE"] = "AwaitingSettlement"; env["FAKE_CONTROL_MODE"] = "path-traversal"
            result = self.invoke(env, "--scenario", "pre-settlement-refusal", "--evidence-dir", str(evidence))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("not confined", result.stderr)

    def test_unknown_control_fields_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); _, env = self.fixture(root)
            evidence = root / "evidence"; evidence.mkdir(mode=0o700)
            env["FAKE_EXPECTED_STATE"] = "AwaitingSettlement"; env["FAKE_CONTROL_MODE"] = "unknown-step-field"
            result = self.invoke(env, "--scenario", "pre-settlement-refusal", "--evidence-dir", str(evidence))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("missing or unknown fields", result.stderr)

    def test_scenario_contract_mutation_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); _, env = self.fixture(root)
            contract_copy = root / "contract.json"; contract_copy.write_bytes(CONTRACT.read_bytes())
            config_path = Path(env["MYCELIX_HOLOCHAIN_DRIVER_CONFIG"]); config = json.loads(config_path.read_text())
            config["scenario_contract"] = str(contract_copy); config["scenario_contract_sha256"] = hash_file(contract_copy)
            config_path.write_text(json.dumps(config, sort_keys=True, separators=(",", ":")))
            evidence = root / "evidence"; evidence.mkdir(mode=0o700)
            env["FAKE_EXPECTED_STATE"] = "AwaitingSettlement"; env["FAKE_MUTATE_CONTRACT"] = str(contract_copy)
            result = self.invoke(env, "--scenario", "pre-settlement-refusal", "--evidence-dir", str(evidence))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("runtime lock changed", result.stderr)

    def test_world_writable_evidence_directory_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); _, env = self.fixture(root)
            evidence = root / "evidence"; evidence.mkdir(mode=0o777); evidence.chmod(0o777)
            env["FAKE_EXPECTED_STATE"] = "AwaitingSettlement"
            result = self.invoke(env, "--scenario", "pre-settlement-refusal", "--evidence-dir", str(evidence))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("group- or world-writable", result.stderr)

    def test_release_identity_is_mandatory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            _, env = self.fixture(Path(temporary)); env.pop("MYCELIX_DRIVER_RELEASE_HASH")
            result = self.invoke(env, "--capabilities")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("MYCELIX_DRIVER_RELEASE_HASH", result.stderr)


if __name__ == "__main__":
    unittest.main()
