#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import physical_handback_v1 as p
REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002d4/example_case.json").read_text(encoding="utf-8"))

class PhysicalTests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,c): return p.qualify(c["profile"],c["parent_receipt"],c["assessment"]).receipt()
    def rebind(self,c):
        c["profile"]["parent_transition_receipt_sha256"]=p.sha256_hex(c["parent_receipt"])
        c["assessment"]["parent_transition_receipt_sha256"]=c["profile"]["parent_transition_receipt_sha256"]
        c["assessment"]["profile_sha256"]=p.sha256_hex(c["profile"])

    def test_positive(self):
        r=self.q(self.case())
        self.assertEqual(r["physical_state"],"PHYSICAL_CONDITION_ACCEPTABLE")
        self.assertFalse(r["handover_accepted"]); self.assertFalse(r["legal_transition_complete"])

    def test_condition_fail(self):
        c=self.case(); c["assessment"]["components"]["power_system"]["condition_state"]="FAIL"
        r=self.q(c); self.assertEqual(r["physical_state"],"REMEDIATION_REQUIRED")
        self.assertIn("CONDITION_FAIL:power_system",r["blockers"])

    def test_residual_life_deficit(self):
        c=self.case(); c["assessment"]["components"]["fiber_backbone"]["residual_life_units"]=119
        r=self.q(c); self.assertEqual(r["physical_state"],"REMEDIATION_REQUIRED")
        self.assertIn("RESIDUAL_LIFE_DEFICIT:fiber_backbone",r["blockers"])

    def test_not_assessed(self):
        c=self.case(); c["assessment"]["components"]["site_facility"]["condition_state"]="NOT_ASSESSED"; c["assessment"]["components"]["site_facility"]["residual_life_units"]=0
        r=self.q(c); self.assertEqual(r["physical_state"],"ASSESSMENT_INCOMPLETE")

    def test_reserve_deficient(self):
        c=self.case(); c["assessment"]["reserve_balance_units"]=3999999
        r=self.q(c); self.assertEqual(r["physical_state"],"RESERVE_DEFICIENT")
        self.assertIn("HANDBACK_RESERVE_DEFICIENT",r["blockers"])

    def test_deferred_maintenance_excess(self):
        c=self.case(); c["assessment"]["deferred_maintenance_units"]=2000001
        r=self.q(c); self.assertEqual(r["physical_state"],"REMEDIATION_REQUIRED")
        self.assertIn("DEFERRED_MAINTENANCE_EXCESS",r["blockers"])

    def test_self_assessment_rejected(self):
        c=self.case(); c["assessment"]["assessor_ref"]=c["assessment"]["outgoing_operator_ref"]
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_required_na_rejected(self):
        c=self.case(); c["assessment"]["components"]["power_system"]["condition_state"]="NOT_APPLICABLE"; c["assessment"]["components"]["power_system"]["residual_life_units"]=0
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_unknown_component_rejected(self):
        c=self.case(); c["assessment"]["components"]["attacker_component"]={"condition_state":"PASS","residual_life_units":999,"evidence_ref":"evidence:x"}
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_negative_residual_rejected(self):
        c=self.case(); c["assessment"]["components"]["power_system"]["residual_life_units"]=-1
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_float_residual_rejected(self):
        c=self.case(); c["assessment"]["components"]["power_system"]["residual_life_units"]=84.5
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_negative_reserve_rejected(self):
        c=self.case(); c["assessment"]["reserve_balance_units"]=-1
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_project_substitution_rejected(self):
        c=self.case(); c["assessment"]["project_id"]="fiber:other"
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_profile_substitution_rejected(self):
        c=self.case(); c["assessment"]["profile_sha256"]="f"*64
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_parent_substitution_rejected(self):
        c=self.case(); c["parent_receipt"]["remaining_claim_units"]+=1
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_financial_state_orthogonal_to_physical_fail(self):
        c=self.case()
        c["parent_receipt"]["financial_state"]="RETURN_ENVELOPE_SATISFIED"
        c["parent_receipt"]["remaining_claim_units"]=0
        self.rebind(c)
        c["assessment"]["components"]["power_system"]["condition_state"]="FAIL"
        r=self.q(c)
        self.assertEqual(r["parent_financial_state"],"RETURN_ENVELOPE_SATISFIED")
        self.assertEqual(r["physical_state"],"REMEDIATION_REQUIRED")

    def test_injected_title_field_rejected(self):
        c=self.case(); c["assessment"]["legal_transition_complete"]=True
        with self.assertRaises(p.PhysicalError): self.q(c)

    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case())
        self.assertEqual(p.canonical_bytes(a),p.canonical_bytes(b))

if __name__=="__main__": unittest.main()
