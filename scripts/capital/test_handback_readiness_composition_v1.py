#!/usr/bin/env python3
from __future__ import annotations
import copy, json, sys, unittest
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import handback_readiness_composition_v1 as h
REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002h3/example_case.json").read_text(encoding="utf-8"))

class H3Tests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,c): return h.qualify(c["profile"],c["digital_receipt"],c["physical_receipt"])
    def test_positive(self):
        r=self.q(self.case()); self.assertEqual(r["handback_readiness_state"],"HANDOVER_READY"); self.assertEqual(r["blockers"],[])
    def test_digital_remediation(self):
        c=self.case(); c["digital_receipt"]["readiness_state"]="REMEDIATION_REQUIRED"; c["digital_receipt"]["blockers"]=["TRUST_ROOT_ROTATION_FAILED"]
        r=self.q(c); self.assertEqual(r["handback_readiness_state"],"REMEDIATION_REQUIRED"); self.assertIn("DIGITAL:TRUST_ROOT_ROTATION_FAILED",r["blockers"])
    def test_physical_remediation(self):
        c=self.case(); c["physical_receipt"]["physical_state"]="REMEDIATION_REQUIRED"; c["physical_receipt"]["blockers"]=["POWER_SYSTEM_FAILED"]
        r=self.q(c); self.assertEqual(r["handback_readiness_state"],"REMEDIATION_REQUIRED"); self.assertIn("PHYSICAL:POWER_SYSTEM_FAILED",r["blockers"])
    def test_simultaneous_remediation_preserved(self):
        c=self.case(); c["digital_receipt"]["readiness_state"]="REMEDIATION_REQUIRED"; c["digital_receipt"]["blockers"]=["DIGITAL_X"]; c["physical_receipt"]["physical_state"]="REMEDIATION_REQUIRED"; c["physical_receipt"]["blockers"]=["PHYSICAL_X"]
        r=self.q(c); self.assertEqual(r["handback_readiness_state"],"REMEDIATION_REQUIRED"); self.assertEqual(r["blockers"],sorted(r["blockers"])); self.assertEqual(len(r["blockers"]),2)
    def test_digital_incomplete(self):
        c=self.case(); c["digital_receipt"]["readiness_state"]="ASSESSMENT_INCOMPLETE"; c["digital_receipt"]["blockers"]=["NOT_ASSESSED:backup_restore"]
        self.assertEqual(self.q(c)["handback_readiness_state"],"ASSESSMENT_INCOMPLETE")
    def test_physical_incomplete(self):
        c=self.case(); c["physical_receipt"]["physical_state"]="ASSESSMENT_INCOMPLETE"; c["physical_receipt"]["blockers"]=["NOT_ASSESSED:power_system"]
        self.assertEqual(self.q(c)["handback_readiness_state"],"ASSESSMENT_INCOMPLETE")
    def test_reserve_deficient(self):
        c=self.case(); c["physical_receipt"]["physical_state"]="RESERVE_DEFICIENT"; c["physical_receipt"]["reserve_balance_units"]=100; c["physical_receipt"]["blockers"]=["HANDOVER_RESERVE_DEFICIENT"]
        self.assertEqual(self.q(c)["handback_readiness_state"],"RESERVE_DEFICIENT")
    def test_remediation_outranks_incomplete(self):
        c=self.case(); c["digital_receipt"]["readiness_state"]="ASSESSMENT_INCOMPLETE"; c["physical_receipt"]["physical_state"]="REMEDIATION_REQUIRED"; c["physical_receipt"]["blockers"]=["PHYSICAL_X"]
        self.assertEqual(self.q(c)["handback_readiness_state"],"REMEDIATION_REQUIRED")
    def test_incomplete_outranks_reserve(self):
        c=self.case(); c["digital_receipt"]["readiness_state"]="ASSESSMENT_INCOMPLETE"; c["physical_receipt"]["physical_state"]="RESERVE_DEFICIENT"; c["physical_receipt"]["reserve_balance_units"]=100
        self.assertEqual(self.q(c)["handback_readiness_state"],"ASSESSMENT_INCOMPLETE")
    def test_project_mismatch_rejected(self):
        c=self.case(); c["physical_receipt"]["project_id"]="fiber:other"; self.assertRaises(h.CompositionError,self.q,c)
    def test_parent_subject_mismatch_rejected(self):
        c=self.case(); c["physical_receipt"]["parent_subject_sha"]="0"*40; self.assertRaises(h.CompositionError,self.q,c)
    def test_parent_receipt_mismatch_rejected(self):
        c=self.case(); c["physical_receipt"]["parent_transition_receipt_sha256"]="0"*64; self.assertRaises(h.CompositionError,self.q,c)
    def test_financial_state_mismatch_rejected(self):
        c=self.case(); c["physical_receipt"]["parent_financial_state"]="RETURN_ENVELOPE_SATISFIED"; self.assertRaises(h.CompositionError,self.q,c)
    def test_remaining_claim_mismatch_rejected(self):
        c=self.case(); c["physical_receipt"]["parent_remaining_claim_units"]+=1; self.assertRaises(h.CompositionError,self.q,c)
    def test_digital_handover_contamination_rejected(self):
        c=self.case(); c["digital_receipt"]["handover_accepted"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_physical_legal_contamination_rejected(self):
        c=self.case(); c["physical_receipt"]["legal_transition_complete"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_raw_digital_assessment_rejected(self):
        c=self.case(); c["digital_receipt"]={"assessment_id":"assessment-001"}; self.assertRaises(h.CompositionError,self.q,c)
    def test_raw_physical_assessment_rejected(self):
        c=self.case(); c["physical_receipt"]={"assessment_id":"physical-assessment-001"}; self.assertRaises(h.CompositionError,self.q,c)
    def test_unknown_authority_field_rejected(self):
        c=self.case(); c["digital_receipt"]["operational_custody_accepted"]=True; self.assertRaises(h.CompositionError,self.q,c)
    def test_strong_nonclaims(self):
        r=self.q(self.case()); self.assertFalse(r["currentness_established"]); self.assertFalse(r["handover_accepted"]); self.assertFalse(r["operational_custody_accepted"]); self.assertFalse(r["legal_transition_complete"]); self.assertFalse(r["execution_authority_established"])
    def test_deterministic(self):
        self.assertEqual(h.canonical_bytes(self.q(self.case())),h.canonical_bytes(self.q(self.case())))

if __name__=="__main__": unittest.main()
