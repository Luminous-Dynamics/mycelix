#!/usr/bin/env python3
import copy
import importlib.util
import json
import unittest
from pathlib import Path

HERE = Path(__file__).parent
spec = importlib.util.spec_from_file_location("gen", HERE / "generate_core_vector_harness.py")
gen = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(gen)

BASE = json.loads((HERE / "frozen_vectors.json").read_text())

class Tests(unittest.TestCase):
    def render(self, data=None):
        return gen.render(copy.deepcopy(BASE if data is None else data))

    def test_exact_vectors_render_both_expected_states(self):
        out = self.render()
        self.assertIn("RunLivenessV1::InfrastructureInterrupted", out)
        self.assertIn("RunLivenessV1::AllRequiredJobsNoStart", out)
        self.assertIn("ConjunctiveVerdictV1::InfrastructureInterrupted", out)
        self.assertIn("ConjunctiveVerdictV1::Incomplete", out)

    def test_deterministic(self):
        self.assertEqual(self.render(), self.render())

    def test_schema_and_core_are_pinned(self):
        bad = copy.deepcopy(BASE); bad["schema"] = "v2"
        with self.assertRaises(ValueError): self.render(bad)
        bad = copy.deepcopy(BASE); bad["supported_core_head"] = "0" * 40
        with self.assertRaises(ValueError): self.render(bad)

    def test_closed_fields(self):
        bad = copy.deepcopy(BASE); bad["extra"] = True
        with self.assertRaises(ValueError): self.render(bad)
        bad = copy.deepcopy(BASE); bad["vectors"][0]["core_observation_v1"]["jobs"][0]["extra"] = True
        with self.assertRaises(ValueError): self.render(bad)

    def test_duplicate_vector_and_job_keys_rejected(self):
        bad = copy.deepcopy(BASE); bad["vectors"].append(copy.deepcopy(bad["vectors"][0]))
        with self.assertRaises(ValueError): self.render(bad)
        bad = copy.deepcopy(BASE)
        bad["vectors"][0]["core_observation_v1"]["jobs"].append(copy.deepcopy(bad["vectors"][0]["core_observation_v1"]["jobs"][0]))
        with self.assertRaises(ValueError): self.render(bad)

    def test_manifest_observation_binding(self):
        bad = copy.deepcopy(BASE); bad["vectors"][0]["core_observation_v1"]["repository_id"] += 1
        with self.assertRaises(ValueError): self.render(bad)
        bad = copy.deepcopy(BASE); bad["vectors"][0]["core_observation_v1"]["qualification_head"] = "f" * 40
        with self.assertRaises(ValueError): self.render(bad)

    def test_required_job_must_exist(self):
        bad = copy.deepcopy(BASE); bad["vectors"][0]["core_manifest_v1"]["required_jobs"][0]["job_key"] = "other"
        with self.assertRaises(ValueError): self.render(bad)

    def test_enum_and_u64_bounds(self):
        bad = copy.deepcopy(BASE); bad["vectors"][0]["core_observation_v1"]["jobs"][0]["status"] = "Waiting"
        with self.assertRaises(ValueError): self.render(bad)
        bad = copy.deepcopy(BASE); bad["vectors"][0]["core_observation_v1"]["workflow_run_id"] = -1
        with self.assertRaises(ValueError): self.render(bad)

    def test_evidence_id_bounds(self):
        bad = copy.deepcopy(BASE); bad["vectors"][0]["id"] = "x" * 257
        with self.assertRaises(ValueError): self.render(bad)
        bad = copy.deepcopy(BASE); bad["vectors"][0]["id"] = "bad\nid"
        with self.assertRaises(ValueError): self.render(bad)

    def test_no_network_or_process_capability(self):
        source = (HERE / "generate_core_vector_harness.py").read_text()
        for token in ("subprocess", "socket", "requests", "urllib", "http.client", "os.system", "Popen"):
            self.assertNotIn(token, source)

if __name__ == "__main__":
    unittest.main()
