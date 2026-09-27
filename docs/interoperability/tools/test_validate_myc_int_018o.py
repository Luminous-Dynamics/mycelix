import unittest

from validate_myc_int_018o import EXPECTED_PARENT, validate


def bench_fixture():
    return {
        "stages": [
            {"id": "H1a", "name": "dry-instrumentation-power"},
            {"id": "H1b", "name": "clean-water-closed-loop"},
            {"id": "H1c", "name": "nutrient-solution-sensing"},
        ],
        "channels": [
            {"id": "flow-rate", "actuation_authority": False},
            {"id": "ph", "actuation_authority": False},
        ],
    }


def matrix_fixture():
    return {
        "claim_ceiling": {
            "wiring_approved": False,
            "electrical_safety_established": False,
            "hardware_compatible": False,
            "sensor_calibrated": False,
            "actuator_authorized": False,
        },
        "candidates": [
            {
                "candidate_id": "pump-dfrobot-fit0200",
                "acquisition_owner": "ActuatorProgramDeferred",
                "provider_profile": "NotSensorProviderV1",
            },
            {
                "candidate_id": "ph-atlas-kit-101p",
                "acquisition_owner": "LuminousEdgeSensorProvider",
                "provider_profile": "read-only local digital sensor",
            },
        ],
    }


def package_fixture():
    return {
        "parent_018n_subject": EXPECTED_PARENT,
        "package_status": "Draft",
        "claim_ceiling": {
            "wiring_approved": False,
            "energization_approved": False,
            "electrical_safety_certified": False,
            "sensor_calibration_established": False,
            "agronomic_qualified": False,
            "autonomous_control_authorized": False,
        },
        "delivered_hardware_inventory": [],
        "electrical_design": {
            "power_rails": [],
            "protection_devices": [],
            "switching_elements": [],
            "ground_domains": [],
            "isolation_barriers": [],
            "level_shifters_or_buffers": [],
            "connector_pin_map": [],
            "manual_stop_override": None,
            "wet_dry_boundary": None,
            "power_budget_reviewed": False,
        },
        "hydraulic_process": {"first_wet_test_medium": "clean-water-only"},
        "stage_gates": {
            "H1a": {"name": "dry-instrumentation-power", "required_checks": [], "approved": False},
            "H1b": {"name": "clean-water-closed-loop", "requires": ["H1a-approved"], "required_checks": [], "approved": False},
            "H1c": {"name": "nutrient-solution-sensing", "requires": ["H1b-evidence-reviewed"], "required_checks": ["automatic-dosing-disabled"], "approved": False},
        },
        "stop_abort_conditions": [
            "leak-detected",
            "unexpected-current-or-overtemperature",
            "manual-stop-unavailable",
            "unintended-actuator-state",
            "loss-of-containment",
            "critical-sensor-or-interface-failure",
            "unknown-wiring-or-device-identity",
            "operator-concern",
        ],
        "run_evidence_requirements": [
            "exact-installation-package-generation",
            "exact-run-profile",
            "exact-hardware-install-identities",
            "pre-run-checklist-result",
            "calibration-check-evidence-refs",
            "observation-export",
            "fault-action-log",
            "manual-interventions",
            "stop-abort-events",
            "post-run-inspection-result",
            "deviations-and-unknowns",
            "evidence-commitments",
        ],
    }


class ValidatorTests(unittest.TestCase):
    def assert_rejected(self, mutate):
        bench = bench_fixture()
        matrix = matrix_fixture()
        package = package_fixture()
        mutate(bench, matrix, package)
        self.assertTrue(validate(bench, matrix, package))

    def test_pristine(self):
        self.assertEqual(validate(bench_fixture(), matrix_fixture(), package_fixture()), [])

    def test_parent_drift(self):
        self.assert_rejected(lambda b, m, p: p.__setitem__("parent_018n_subject", "other"))

    def test_h1b_not_clean_water(self):
        self.assert_rejected(lambda b, m, p: p["hydraulic_process"].__setitem__("first_wet_test_medium", "nutrient-solution"))

    def test_auto_dosing_enabled_by_removal(self):
        self.assert_rejected(lambda b, m, p: p["stage_gates"]["H1c"]["required_checks"].remove("automatic-dosing-disabled"))

    def test_claim_ceiling_upgrade(self):
        self.assert_rejected(lambda b, m, p: p["claim_ceiling"].__setitem__("electrical_safety_certified", True))

    def test_manual_stop_removed(self):
        self.assert_rejected(lambda b, m, p: p["stop_abort_conditions"].remove("manual-stop-unavailable"))

    def test_manual_intervention_evidence_removed(self):
        self.assert_rejected(lambda b, m, p: p["run_evidence_requirements"].remove("manual-interventions"))

    def test_channel_authority_upgrade(self):
        self.assert_rejected(lambda b, m, p: b["channels"][0].__setitem__("actuation_authority", True))

    def test_pump_masquerades_as_sensor(self):
        self.assert_rejected(lambda b, m, p: m["candidates"][0].__setitem__("provider_profile", "read-only local digital sensor"))

    def test_revision_mismatch_hidden(self):
        def mutate(b, m, p):
            p["delivered_hardware_inventory"] = [{
                "candidate_id": "ph-atlas-kit-101p",
                "revision_mismatch": True,
                "deviation_from_candidate": "",
            }]
        self.assert_rejected(mutate)

    def test_unknown_candidate(self):
        self.assert_rejected(lambda b, m, p: p["delivered_hardware_inventory"].append({"candidate_id": "mystery"}))

    def test_network_required_for_stop(self):
        self.assert_rejected(lambda b, m, p: p.__setitem__("local_stop_dependencies", ["network"]))


if __name__ == "__main__":
    unittest.main()
