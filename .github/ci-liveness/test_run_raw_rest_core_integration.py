import copy
import json
import pathlib
import unittest

import run_raw_rest_core_integration as r

ROOT = pathlib.Path(__file__).parent
FIX = json.loads((ROOT / "amsap_raw_rest_replay.json").read_text())
VEC = json.loads(pathlib.Path("/tmp/vectorprep/frozen_vectors.json").read_text())
IDX = {v["id"]: v for v in VEC["vectors"]}


def stub_adapt(run, jobs, profile, observed):
    vid = "amsap-r1-terminal-cancelled-no-start" if run["id"] == 35443692878 else "amsap-r2-queued-no-start"
    v = IDX[vid]
    return {
        "supported_core_head": r.SEMANTIC_CORE_HEAD,
        "core_manifest_v1": copy.deepcopy(v["core_manifest_v1"]),
        "core_observation_v1": copy.deepcopy(v["core_observation_v1"]),
        "provider_diagnostics": [{"provider_state": v["expected_provider_state"]}],
        "observation_source_authenticity_verified": False,
        "github_api_response_authenticity_verified": False,
        "runner_identity_attested": False,
        "semantic_classification_performed": False,
        "qualification_authority": False,
        "evidence_authority": False,
        "failover_authority": False,
        "rerun_authority": False,
        "dispatch_authority": False,
        "qualification_result": None,
        "theorem_result": None,
    }


class T(unittest.TestCase):
    def test_exact_fixture_shape(self):
        self.assertEqual(len(r.validate_fixture_document(FIX)), 2)

    def test_replay_matches_vectors(self):
        self.assertEqual(len(r.evaluate(FIX, VEC, stub_adapt)), 2)

    def test_source_authenticity_cannot_be_claimed(self):
        x = copy.deepcopy(FIX); x["source_authenticity_claimed"] = True
        with self.assertRaises(r.ReplayError): r.validate_fixture_document(x)

    def test_case_vector_identity_is_closed(self):
        x = copy.deepcopy(FIX); x["cases"][0]["expected_vector_id"] = "other"
        with self.assertRaises(r.ReplayError): r.validate_fixture_document(x)

    def test_gate_profile_is_exactly_five_unique_names(self):
        x = copy.deepcopy(FIX); x["cases"][0]["profile"]["required_jobs"][0]["required_gate_names"].append("x")
        with self.assertRaises(r.ReplayError): r.validate_fixture_document(x)

    def test_manifest_mismatch_rejected(self):
        def bad(*args):
            out = stub_adapt(*args); out["core_manifest_v1"]["theorem_id"] = "BAD"; return out
        with self.assertRaises(r.ReplayError): r.evaluate(FIX, VEC, bad)

    def test_observation_mismatch_rejected(self):
        def bad(*args):
            out = stub_adapt(*args); out["core_observation_v1"]["jobs"][0]["job_id"] += 1; return out
        with self.assertRaises(r.ReplayError): r.evaluate(FIX, VEC, bad)

    def test_provider_state_mismatch_rejected(self):
        def bad(*args):
            out = stub_adapt(*args); out["provider_diagnostics"][0]["provider_state"] = "Indeterminate"; return out
        with self.assertRaises(r.ReplayError): r.evaluate(FIX, VEC, bad)

    def test_authority_broadening_rejected(self):
        def bad(*args):
            out = stub_adapt(*args); out["dispatch_authority"] = True; return out
        with self.assertRaises(r.ReplayError): r.evaluate(FIX, VEC, bad)

    def test_semantic_result_rejected(self):
        def bad(*args):
            out = stub_adapt(*args); out["theorem_result"] = "PASS"; return out
        with self.assertRaises(r.ReplayError): r.evaluate(FIX, VEC, bad)


if __name__ == "__main__":
    unittest.main()