#!/usr/bin/env python3
from __future__ import annotations
import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve()
ROOT = HERE.parents[2]
for path in [ROOT / "scripts/capital"]:
    sys.path.insert(0, str(path))
spec = importlib.util.spec_from_file_location("g2b", ROOT / "scripts/capital/constitution_amendment_v1.py")
g2b = importlib.util.module_from_spec(spec)
sys.modules["g2b"] = g2b
assert spec.loader
spec.loader.exec_module(g2b)

FIXTURE = ROOT / "docs/capital/evidence/myc-cap-002g2b/example_case.json"

def load_case():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))

def sync_successor(c):
    sha = g2b.sha256_hex(c["successor_g1_profile"])
    c["authorization"]["successor_profile_sha256"] = sha
    c["transition"]["successor_profile_sha256"] = sha
    return sha

class AmendmentTests(unittest.TestCase):
    def q(self, case):
        return g2b.qualify(case).receipt()

    def test_positive_accepted(self):
        r=self.q(load_case())
        self.assertEqual(r["transition_state"], "ACCEPTED")
        self.assertEqual(r["computed_changed_paths"], ["/actions/OPERATOR_RENEWAL/required_chambers/PUBLIC/approval_ppm"])
        self.assertFalse(r["successor_designation_established"])

    def test_successor_epoch_skip_rejected(self):
        c=load_case(); c["transition"]["successor_epoch"]=9
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_wrong_prior_profile_rejected(self):
        c=load_case(); c["transition"]["prior_profile_sha256"]="0"*64
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_successor_must_differ(self):
        c=load_case(); c["successor_g1_profile"]=copy.deepcopy(c["prior_g1_profile"])
        sha=g2b.sha256_hex(c["successor_g1_profile"])
        c["authorization"]["successor_profile_sha256"]=sha
        c["transition"]["successor_profile_sha256"]=sha
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_hidden_unauthorized_change_blocks(self):
        c=load_case()
        c["successor_g1_profile"]["max_vote_count"] += 1
        sync_successor(c)
        r=self.q(c)
        self.assertEqual(r["transition_state"], "BLOCKED")
        self.assertIn("UNAUTHORIZED_CHANGE:/max_vote_count", r["blockers"])

    def test_asset_lock_unprotection_rejected(self):
        c=load_case()
        c["successor_g1_profile"]["actions"]["ASSET_LOCK_REMOVAL"]["prohibited"]=False
        sync_successor(c)
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_steward_seat_sale_unprotection_rejected(self):
        c=load_case()
        c["successor_g1_profile"]["actions"]["STEWARD_SEAT_SALE"]["prohibited"]=False
        sync_successor(c)
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_capital_chamber_rejected(self):
        c=load_case()
        c["successor_g1_profile"]["chambers"].append("CAPITAL")
        sync_successor(c)
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_protected_enforcer_weakening_rejected(self):
        c=load_case()
        c["successor_g1_profile"]["actions"]["ASSET_LOCK_REMOVAL"]["enforcer_required"]=False
        sync_successor(c)
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_historical_currentness_blocks(self):
        c=load_case(); c["prior_currentness_receipt"]["currentness_state"]="HISTORICAL"
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        r=self.q(c)
        self.assertEqual(r["transition_state"], "BLOCKED")
        self.assertIn("PRIOR_CURRENTNESS_NOT_CURRENT:HISTORICAL", r["blockers"])

    def test_revoked_currentness_blocks(self):
        c=load_case(); c["prior_currentness_receipt"]["currentness_state"]="REVOKED"
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        r=self.q(c)
        self.assertIn("PRIOR_CURRENTNESS_NOT_CURRENT:REVOKED", r["blockers"])

    def test_pending_currentness_blocks(self):
        c=load_case(); c["prior_currentness_receipt"]["currentness_state"]="PENDING"
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        r=self.q(c)
        self.assertIn("PRIOR_CURRENTNESS_NOT_CURRENT:PENDING", r["blockers"])

    def test_currentness_wrong_profile_rejected(self):
        c=load_case(); c["prior_currentness_receipt"]["active_g1_profile_sha256"]="0"*64
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_currentness_wrong_project_rejected(self):
        c=load_case(); c["prior_currentness_receipt"]["project_id"]="other"
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_currentness_wrong_registry_rejected(self):
        c=load_case(); c["prior_currentness_receipt"]["registry_id"]="other"
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_prohibited_class_blocks(self):
        c=load_case()
        c["authorization"]["amendment_class"]="REPRESENTATION"
        c["transition"]["amendment_class"]="REPRESENTATION"
        r=self.q(c)
        self.assertEqual(r["transition_state"], "BLOCKED")
        self.assertIn("PROHIBITED_AMENDMENT_CLASS:REPRESENTATION", r["blockers"])

    def test_emergency_injection_rejected(self):
        c=load_case(); c["authorization"]["emergency"]={"state":"ACTIVE"}
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_enforcer_substitution_rejected(self):
        c=load_case(); c["authorization"]["enforcer"]["authority_ref"]="enforcer:attacker"
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_enforcer_no_concurrence_blocks(self):
        c=load_case(); c["authorization"]["enforcer"]["concurred"]=False
        r=self.q(c)
        self.assertIn("ENFORCER_DID_NOT_CONCUR", r["blockers"])

    def test_quorum_failure_blocks(self):
        c=load_case(); c["authorization"]["chamber_votes"]["USERS"]["participating_count"]=70
        c["authorization"]["chamber_votes"]["USERS"]["approval_count"]=70
        r=self.q(c)
        self.assertIn("QUORUM_FAIL:USERS", r["blockers"])

    def test_approval_failure_blocks(self):
        c=load_case(); c["authorization"]["chamber_votes"]["PUBLIC"]["approval_count"]=10
        r=self.q(c)
        self.assertIn("APPROVAL_FAIL:PUBLIC", r["blockers"])

    def test_authorization_project_substitution_rejected(self):
        c=load_case(); c["authorization"]["project_id"]="other"
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_authorization_registry_substitution_rejected(self):
        c=load_case(); c["authorization"]["registry_id"]="other"
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_authorization_epoch_substitution_rejected(self):
        c=load_case(); c["authorization"]["prior_epoch"]=6
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_authorization_successor_digest_substitution_rejected(self):
        c=load_case(); c["authorization"]["successor_profile_sha256"]="0"*64
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_currentness_digest_substitution_rejected(self):
        c=load_case(); c["authorization"]["currentness_receipt_sha256"]="0"*64
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_previous_transition_digest_malformed_rejected(self):
        c=load_case(); c["transition"]["previous_transition_sha256"]="bad"
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_authority_contamination_rejected(self):
        c=load_case(); c["prior_currentness_receipt"]["execution_authority_established"]=True
        c["authorization"]["currentness_receipt_sha256"]=g2b.sha256_hex(c["prior_currentness_receipt"])
        with self.assertRaises(g2b.AmendmentError): self.q(c)

    def test_reordering_setlike_profile_fields_is_not_semantic_change(self):
        c=load_case()
        c["successor_g1_profile"]["chambers"].reverse()
        c["successor_g1_profile"]["conflict_codes"].reverse()
        c["successor_g1_profile"]["actions"]["OPERATOR_RENEWAL"]["required_recusal_conflicts"].reverse()
        sync_successor(c)
        r=self.q(c)
        self.assertEqual(r["computed_changed_paths"], ["/actions/OPERATOR_RENEWAL/required_chambers/PUBLIC/approval_ppm"])
        self.assertEqual(r["transition_state"], "ACCEPTED")

    def test_deterministic_nonclaims(self):
        c=load_case()
        a=self.q(c)
        b=self.q(copy.deepcopy(c))
        self.assertEqual(a,b)
        self.assertFalse(a["successor_designation_established"])
        self.assertFalse(a["legal_validity_established"])
        self.assertFalse(a["democratic_legitimacy_established"])

if __name__ == "__main__":
    unittest.main()
