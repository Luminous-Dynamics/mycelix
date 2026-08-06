#!/usr/bin/env python3
"""End-to-end test of the strict adapter with a temporary synthetic driver.

The driver exists only inside the temporary test directory. It is never shipped
as release evidence and is labeled synthetic in its source.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import textwrap
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from evidence_manifest import contract_hash, load_and_validate_manifest  # noqa: E402
ADAPTER = ROOT / "real_conductor_adapter.py"
RUNNER = ROOT / "run_scenarios.py"
CONTRACT = ROOT / "settlement_scenarios_v2.json"


def make_driver(path: Path) -> None:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    expected = {item["id"]: item["expected_state"] for item in contract["scenarios"]}
    script = f'''#!/usr/bin/env python3
# Synthetic test driver. Never valid as project release evidence.
import argparse, base64, hashlib, json
from pathlib import Path
EXPECTED = {expected!r}
def h(label):
    return base64.b64encode(hashlib.sha256(label.encode()).digest()).decode()
def caps():
    conductors=[]
    for cid in ("verifier","prover_a","prover_b"):
        conductors.append({{
            "id":cid,
            "agent_pub_key":"uhCAk-"+cid,
            "database_identity_hash":h("db:"+cid),
            "network_identity_hash":h("net:"+cid),
        }})
    return {{
        "driver_protocol":"mycelix-real-conductor-driver-v1",
        "driver_release_hash":h("synthetic-test-driver"),
        "conductor_count":3,
        "dna_hash":"uhC0k-test-dna",
        "happ_hash":"uhC0m-test-happ",
        "holochain_version":"0.6.1-test",
        "agents":["verifier","prover_a","prover_b"],
        "conductors":conductors,
    }}
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--capabilities",action="store_true")
    p.add_argument("--scenario")
    p.add_argument("--evidence-dir")
    a=p.parse_args()
    if a.capabilities:
        print(json.dumps(caps(),sort_keys=True)); return 0
    state=EXPECTED[a.scenario]
    root=Path(a.evidence_dir); (root/"observations").mkdir(parents=True,exist_ok=True)
    start=1700000000000000
    observations=[]
    for offset,cid in enumerate(("verifier","prover_a","prover_b"),1):
        relative=f"observations/{{cid}}.json"
        payload={{"conductor_id":cid,"state":state,"action_hashes":[f"uhCkk-{{a.scenario}}-{{cid}}"]}}
        (root/relative).write_text(json.dumps(payload,sort_keys=True),encoding="utf-8")
        observations.append({{
            "conductor_id":cid,
            "state":state,
            "observed_at_unix_micros":start+offset*1000,
            "response_path":relative,
        }})
    result={{
        "driver_protocol":"mycelix-real-conductor-driver-v1",
        "scenario_id":a.scenario,
        "observed_state":state,
        "started_at_unix_micros":start,
        "completed_at_unix_micros":start+10000,
        "events":[
            {{"sequence":0,"kind":"scenario-start","actor":"harness","at_unix_micros":start}},
            {{"sequence":1,"kind":"converged","actor":"harness","at_unix_micros":start+10000}},
        ],
        "observations":observations,
    }}
    print(json.dumps(result,sort_keys=True)); return 0
if __name__ == "__main__": raise SystemExit(main())
'''
    path.write_text(textwrap.dedent(script), encoding="utf-8")
    path.chmod(0o700)


class RealAdapterTests(unittest.TestCase):
    def test_adapter_builds_and_verifies_retained_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            driver = temp / "synthetic_driver.py"
            evidence = temp / "evidence"
            evidence.mkdir()
            make_driver(driver)
            env = os.environ.copy()
            env["MYCELIX_CONDUCTOR_DRIVER"] = str(driver)
            env["MYCELIX_CONDUCTOR_EVIDENCE_ROOT"] = str(evidence)
            env["MYCELIX_CONDUCTOR_DRIVER_TIMEOUT_SECONDS"] = "30"

            capabilities = subprocess.run(
                [str(ADAPTER), "--capabilities"],
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(capabilities.returncode, 0, capabilities.stderr)
            capability_value = json.loads(capabilities.stdout)
            self.assertEqual(capability_value["adapter_kind"], "real-conductor")

            scenario_id = "partitioned-competing-proofs"
            expected_state = "ChallengeConsumptionConflict"
            completed = subprocess.run(
                [str(ADAPTER), "--scenario", scenario_id],
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(completed.stdout)
            self.assertEqual(result["observed_state"], expected_state)
            manifest_path = evidence / result["evidence"]["manifest_path"]
            _, summary = load_and_validate_manifest(
                manifest_path,
                evidence,
                expected_contract_hash=contract_hash(CONTRACT),
                expected_scenario_id=scenario_id,
                expected_state=expected_state,
                expected_dna_hash=capability_value["dna_hash"],
            )
            self.assertEqual(summary["manifest_hash"], result["evidence"]["manifest_hash"])
            self.assertEqual(
                set(summary["conductor_ids"]),
                {"verifier", "prover_a", "prover_b"},
            )


if __name__ == "__main__":
    unittest.main()
