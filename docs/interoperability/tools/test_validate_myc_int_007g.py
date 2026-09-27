import copy
import unittest

from validate_myc_int_007g import EXPECTED_PARENT, validate


def federation_fixture():
    return {
        "nodes": [
            {"id": "A", "provenance": "Physical"},
            {"id": "B", "provenance": "ReplayBacked"},
            {"id": "C", "provenance": "Synthetic"},
            {"id": "D", "provenance": "Synthetic"},
            {"id": "E", "provenance": "Synthetic"},
            {"id": "F", "provenance": "Synthetic"},
            {"id": "G", "provenance": "Synthetic"},
            {"id": "H", "provenance": "ConventionalConformer"},
        ],
        "demonstration_phases": [{"id": f"D{i}"} for i in range(1, 12)],
        "fault_campaigns": [{"id": f"F{i:02d}"} for i in range(1, 19)],
    }


def run_fixture():
    provenance = {
        "A": "Physical", "B": "ReplayBacked", "C": "Synthetic", "D": "Synthetic",
        "E": "Synthetic", "F": "Synthetic", "G": "Synthetic", "H": "ConventionalConformer",
    }
    bindings = []
    for node in "ABCDEFGH":
        bindings.append({
            "fixture_node": node,
            "runtime_mode": "external-physical" if node == "A" else "process-or-container",
            "implementation_subject": None,
            "schema_generation": "from-007f-old-generation" if node == "G" else "from-007f",
            "provenance": provenance[node],
        })
    return {
        "parent_007f_subject": EXPECTED_PARENT,
        "execution_mode": "UNBOUND",
        "claim_ceiling": {
            "executed": False, "qualified": False,
            "scalability_established": False, "physical_effect_authorized": False,
        },
        "run_identity": {
            "orchestrator_tool": None, "orchestrator_version": None,
            "environment_profile": None, "environment_commitment": None,
        },
        "node_bindings": bindings,
        "workload": {"phase_ids": [f"D{i}" for i in range(1, 12)]},
        "fault_campaigns": [f"F{i:02d}" for i in range(1, 19)],
        "evidence_capture": [
            "run-manifest-exact-bytes", "implementation-build-identities",
            "node-logs-with-run-identity", "semantic-event-exports",
            "delivery-attempt-and-ack-traces", "authority-decision-evidence",
            "schema-and-translation-receipts", "conflict-and-reconciliation-records",
            "fault-injector-event-log", "final-state-export-commitments",
            "environment-capsule",
        ],
        "safety_invariants": [
            "federation-harness-does-not-gain-h1-actuator-authority",
            "edge-local-acquisition-does-not-depend-on-federation",
            "physical-node-stop-does-not-depend-on-network",
            "synthetic-replay-evidence-remains-labeled",
        ],
    }


class ValidatorTests(unittest.TestCase):
    def assert_rejected(self, mutate):
        federation = federation_fixture()
        run = run_fixture()
        mutate(federation, run)
        self.assertTrue(validate(federation, run))

    def test_pristine(self):
        self.assertEqual(validate(federation_fixture(), run_fixture()), [])

    def test_missing_h(self):
        self.assert_rejected(lambda f, r: r["node_bindings"].pop())

    def test_duplicate_a(self):
        self.assert_rejected(lambda f, r: r["node_bindings"].append(copy.deepcopy(r["node_bindings"][0])))

    def test_provenance_drift(self):
        self.assert_rejected(lambda f, r: r["node_bindings"][1].__setitem__("provenance", "Physical"))

    def test_old_generation_erased(self):
        self.assert_rejected(lambda f, r: r["node_bindings"][6].__setitem__("schema_generation", "from-007f"))

    def test_foreign_authority_phase_missing(self):
        self.assert_rejected(lambda f, r: r["workload"]["phase_ids"].remove("D5"))

    def test_unknown_delivery_fault_missing(self):
        self.assert_rejected(lambda f, r: r["fault_campaigns"].remove("F02"))

    def test_parent_drift(self):
        self.assert_rejected(lambda f, r: r.__setitem__("parent_007f_subject", "other"))

    def test_missing_evidence(self):
        self.assert_rejected(lambda f, r: r["evidence_capture"].remove("authority-decision-evidence"))

    def test_unknown_mode(self):
        self.assert_rejected(lambda f, r: r.__setitem__("execution_mode", "Magic"))

    def test_unbound_fake_build(self):
        self.assert_rejected(lambda f, r: r["node_bindings"][0].__setitem__("implementation_subject", "fake"))

    def test_actuator_authority(self):
        self.assert_rejected(lambda f, r: r["node_bindings"][0].__setitem__("actuator_authority", True))


if __name__ == "__main__":
    unittest.main()
