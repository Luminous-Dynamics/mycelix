#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve()
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
import stewardship_governance_composition_v1 as h1

POSITIVE = REPO / "docs/capital/evidence/myc-cap-002h1/example_case.json"
OLD_G2A = REPO / "docs/capital/evidence/myc-cap-002g2a/example_receipt.json"
G2C = REPO / "docs/capital/evidence/myc-cap-002g2c/example_receipt.json"

def load(path):
    return json.loads(path.read_text(encoding="utf-8"))

def positive():
    return load(POSITIVE)

class H1Tests(unittest.TestCase):
    def q(self, case):
        return h1.qualify(case)

    def test_positive_current(self):
        r = self.q(positive())
        self.assertEqual(r["governance_state"], "CURRENT")
        self.assertEqual(r["active_epoch"], 8)
        self.assertEqual(r["blockers"], [])

    def test_existing_epoch7_plus_epoch8_is_stale(self):
        c = positive()
        c["g2a_receipt"] = load(OLD_G2A)
        c["g2c_receipt"] = load(G2C)
        r = self.q(c)
        self.assertEqual(r["governance_state"], "STALE")
        self.assertIn("GOVERNANCE_EPOCH_MISMATCH", r["blockers"])
        self.assertIn("GOVERNANCE_PROFILE_MISMATCH", r["blockers"])

    def test_same_epoch_wrong_profile_stale(self):
        c = positive()
        c["g2a_receipt"]["active_g1_profile_sha256"] = "0" * 64
        r = self.q(c)
        self.assertEqual(r["governance_state"], "STALE")
        self.assertEqual(r["blockers"], ["GOVERNANCE_PROFILE_MISMATCH"])

    def test_wrong_registry_rejected(self):
        c = positive()
        c["g2a_receipt"]["registry_id"] = "registry:other"
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_wrong_project_rejected(self):
        c = positive()
        c["g2c_receipt"]["project_id"] = "other"
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_g2c_stale(self):
        c = positive()
        c["g2c_receipt"]["designation_state"] = "STALE"
        c["g2c_receipt"]["new_checkpoint"] = None
        r = self.q(c)
        self.assertEqual(r["governance_state"], "STALE")
        self.assertIn("CONSTITUTION_CHECKPOINT_STALE", r["blockers"])

    def test_g2c_active_requires_checkpoint(self):
        c = positive()
        c["g2c_receipt"]["new_checkpoint"] = None
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_g2a_historical(self):
        c = positive()
        c["g2a_receipt"]["currentness_state"] = "HISTORICAL"
        r = self.q(c)
        self.assertEqual(r["governance_state"], "STALE")
        self.assertIn("GOVERNANCE_DECISION_HISTORICAL", r["blockers"])

    def test_g2a_revoked(self):
        c = positive()
        c["g2a_receipt"]["currentness_state"] = "REVOKED"
        r = self.q(c)
        self.assertEqual(r["governance_state"], "BLOCKED")
        self.assertIn("GOVERNANCE_DECISION_REVOKED", r["blockers"])

    def test_g2a_pending(self):
        c = positive()
        c["g2a_receipt"]["currentness_state"] = "PENDING"
        r = self.q(c)
        self.assertEqual(r["governance_state"], "PENDING")

    def test_g2a_not_authorized(self):
        c = positive()
        c["g2a_receipt"]["decision_authorization_state"] = "BLOCKED"
        r = self.q(c)
        self.assertEqual(r["governance_state"], "BLOCKED")
        self.assertIn("GOVERNANCE_DECISION_NOT_AUTHORIZED", r["blockers"])

    def test_unsupported_action(self):
        c = positive()
        c["g2a_receipt"]["action_code"] = "ASSET_LOCK_REMOVAL"
        r = self.q(c)
        self.assertEqual(r["governance_state"], "UNSUPPORTED")

    def test_raw_g1_shape_rejected(self):
        c = positive()
        c["g2a_receipt"] = {"receipt_version": "mycelix-stewardship-decision-receipt-v1"}
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_g2b_shape_rejected_as_g2c(self):
        c = positive()
        c["g2c_receipt"] = {"receipt_version": "mycelix-constitution-amendment-receipt-v1"}
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_authority_contamination_g2a(self):
        c = positive()
        c["g2a_receipt"]["execution_authority_established"] = True
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_authority_contamination_g2c(self):
        c = positive()
        c["g2c_receipt"]["legal_validity_established"] = True
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_unknown_input_key_rejected(self):
        c = positive()
        c["g2a_receipt"]["raw_ballots"] = []
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_profile_actions_are_sorted_unique(self):
        c = positive()
        c["composition_profile"]["allowed_action_codes"] = ["OPERATOR_RENEWAL", "OPERATOR_RENEWAL"]
        with self.assertRaises(h1.CompositionError):
            self.q(c)

    def test_output_nonclaims_are_strong(self):
        r = self.q(positive())
        self.assertFalse(r["execution_authority_established"])
        self.assertFalse(r["legal_validity_established"])
        self.assertFalse(r["democratic_legitimacy_established"])

    def test_key_order_does_not_change_output(self):
        c = positive()
        reordered = json.loads(json.dumps(c, sort_keys=False))
        self.assertEqual(self.q(c), self.q(reordered))

    def test_blocker_order_deterministic(self):
        c = positive()
        c["g2a_receipt"] = load(OLD_G2A)
        r = self.q(c)
        self.assertEqual(r["blockers"], sorted(r["blockers"]))

    def test_replay_deterministic(self):
        c = positive()
        self.assertEqual(self.q(c), self.q(copy.deepcopy(c)))

if __name__ == "__main__":
    unittest.main()
