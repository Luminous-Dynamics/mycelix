#!/usr/bin/env python3
from __future__ import annotations
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import public_service_composition_v1 as h

CASE = json.loads((HERE / "example_case.json").read_text(encoding="utf-8"))

class H2Tests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self, c): return h.qualify(c["profile"], c["service_gate_receipt"], c["service_currentness_receipt"]).receipt()
    def test_positive(self):
        r=self.q(self.case()); self.assertEqual(r["public_service_distribution_state"], "ELIGIBLE_CURRENT"); self.assertFalse(r["payment_authority_established"])
    def test_stale(self):
        c=self.case(); c["service_currentness_receipt"]["currentness_state"]="STALE"; self.assertEqual(self.q(c)["public_service_distribution_state"],"STALE")
    def test_pending(self):
        c=self.case(); c["service_currentness_receipt"]["currentness_state"]="PENDING"; self.assertEqual(self.q(c)["public_service_distribution_state"],"PENDING")
    def test_revoked(self):
        c=self.case(); c["service_currentness_receipt"]["currentness_state"]="REVOKED"; self.assertEqual(self.q(c)["public_service_distribution_state"],"REVOKED")
    def test_blocked_current(self):
        c=self.case(); c["service_gate_receipt"]["distribution_eligibility"]="BLOCKED"; c["service_gate_receipt"]["blockers"]=["BLOCKED_RESERVE","BLOCKED_AFFORDABILITY"]; c["service_currentness_receipt"]["service_distribution_eligibility"]="BLOCKED"; c["service_currentness_receipt"]["service_receipt_sha256"]=h.sha256_hex(c["service_gate_receipt"]); r=self.q(c); self.assertEqual(r["public_service_distribution_state"],"BLOCKED_SERVICE"); self.assertEqual(r["blockers"],sorted(r["blockers"]))
    def test_no_active_claim(self):
        c=self.case(); c["service_gate_receipt"]["distribution_eligibility"]="NO_ACTIVE_CLAIM"; c["service_gate_receipt"]["parent_remaining_claim_units"]=0; c["service_currentness_receipt"]["service_distribution_eligibility"]="NO_ACTIVE_CLAIM"; c["service_currentness_receipt"]["service_receipt_sha256"]=h.sha256_hex(c["service_gate_receipt"]); self.assertEqual(self.q(c)["public_service_distribution_state"],"NO_ACTIVE_CLAIM")
    def test_superseded_f_subject_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["service_gate_subject_sha"]="45361d938d9a56f66e6eab360b04d773f54b1c8c"; self.assertRaises(h.CompositionError,self.q,c)
    def test_wrong_project_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["project_id"]="fiber:other"; self.assertRaises(h.CompositionError,self.q,c)
    def test_measurement_mismatch_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["measurement_id"]="measure-other"; self.assertRaises(h.CompositionError,self.q,c)
    def test_service_profile_mismatch_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["service_gate_profile_sha256"]="0"*64; self.assertRaises(h.CompositionError,self.q,c)
    def test_service_receipt_digest_mismatch_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["service_receipt_sha256"]="0"*64; self.assertRaises(h.CompositionError,self.q,c)
    def test_eligibility_echo_mismatch_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["service_distribution_eligibility"]="BLOCKED"; self.assertRaises(h.CompositionError,self.q,c)
    def test_f_claim_mutation_rejected(self):
        c=self.case(); c["service_gate_receipt"]["claim_modified"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_f1_claim_mutation_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["claim_modified"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_local_wall_clock_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["uses_local_wall_clock"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_raw_measurement_snapshot_rejected(self):
        c=self.case(); c["service_gate_receipt"]={"measurement_id":"measure-001"}; self.assertRaises(h.CompositionError,self.q,c)
    def test_raw_designation_rejected(self):
        c=self.case(); c["service_currentness_receipt"]={"designation_id":"designation-001"}; self.assertRaises(h.CompositionError,self.q,c)
    def test_unknown_authority_field_rejected(self):
        c=self.case(); c["service_currentness_receipt"]["payment_authority_established"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_strong_nonclaims(self):
        r=self.q(self.case()); self.assertFalse(r["claim_modified"]); self.assertFalse(r["execution_authority_established"]); self.assertFalse(r["payment_authority_established"]); self.assertFalse(r["legal_distribution_authority_established"])
    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case()); self.assertEqual(h.canonical_bytes(a), h.canonical_bytes(b))

if __name__ == "__main__": unittest.main()
