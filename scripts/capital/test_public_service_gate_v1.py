from __future__ import annotations
import copy, json, sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from public_service_gate_v1 import PROFILE_VERSION, GateError, qualify, sha256_hex

PARENT_SUBJECT="270b852e0ac744dfca3a2cb966bf53fce78f2ab9"
PARENT_RECEIPT=json.loads('{"counted_investor_distributions_units": 60000000, "created_entitlement_units": 133000000, "event_chain_tip_sha256": "f766650cbc8233f016ae3482f46708f395264cbc0baeb869ad7c09dd662d240d", "event_count": 8, "event_history_sha256": "b1e75101137bcecc60daba64aa0d232398b78d65e4e946723bd25e5ab23184b7", "financial_state": "CLAIM_ACTIVE", "grant_or_subsidy_units": 7000000, "handback_accepted": false, "impairments_units": 0, "initial_principal_units": 100000000, "legal_transition_complete": false, "nonclaims": ["financial satisfaction is not legal title transfer", "financial satisfaction is not democratic legitimacy", "financial satisfaction is not handback acceptance", "receipt validity is not accounting-standard compliance", "receipt validity is not tax or securities-law compliance", "receipt validity is not infrastructure safety or performance"], "preferred_return_cap_units": 20000000, "profile_sha256": "6929b9e089e3f7b272713f6ff8587bf429594a51a9d67d67d0d4468ba72b52e7", "profile_version": "mycelix-capital-to-commons-fixed-preferred-v1", "project_id": "fiber:jhb:test-001", "qualified_new_capital_units": 10000000, "receipt_version": "mycelix-commons-transition-receipt-v1", "recoverable_lifecycle_units": 3000000, "remaining_claim_units": 73000000, "required_reserve_units": 5000000, "reserve_balance_units": 5000000, "reserve_compliant": true, "retired_claim_ppm": 451127, "retired_claim_units": 60000000, "unit": "ZAR-cent"}')

def profile(parent=None):
    parent=parent or PARENT_RECEIPT
    return {
      "profile_version":PROFILE_VERSION,
      "project_id":"fiber:jhb:test-001",
      "unit":"ZAR-cent",
      "parent_subject_sha":PARENT_SUBJECT,
      "parent_transition_receipt_sha256":sha256_hex(parent),
      "max_amount_units":1_000_000_000,
      "min_coverage_ppm":950_000,
      "min_uptime_ppm":995_000,
      "max_tariff_units":15_000,
      "required_reserve_units":5_000_000,
      "max_deferred_maintenance_units":2_000_000,
      "required_continuity_state":"PASS",
    }

def snapshot(p=None, **overrides):
    p=p or profile()
    s={
      "measurement_id":"measure-001",
      "project_id":p["project_id"],
      "profile_sha256":sha256_hex(p),
      "parent_transition_receipt_sha256":p["parent_transition_receipt_sha256"],
      "measurement_epoch":1,
      "coverage_ppm":980_000,
      "uptime_ppm":999_000,
      "tariff_units":12_000,
      "reserve_balance_units":5_000_000,
      "deferred_maintenance_units":1_000_000,
      "continuity_state":"PASS",
      "authority_ref":"authority:service-steward",
      "evidence_ref":"evidence:measure-001",
    }
    s.update(overrides)
    return s

class Tests(unittest.TestCase):
  def test_eligible(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p)).receipt()
    self.assertEqual(r["distribution_eligibility"],"ELIGIBLE")
    self.assertEqual(r["blockers"],[])
    self.assertFalse(r["claim_modified"])
    self.assertEqual(r["parent_remaining_claim_units"],73_000_000)
  def test_affordability_blocks(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p,tariff_units=15_001)).receipt()
    self.assertIn("BLOCKED_AFFORDABILITY",r["blockers"])
  def test_coverage_blocks(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p,coverage_ppm=949_999)).receipt()
    self.assertIn("BLOCKED_SERVICE_COVERAGE",r["blockers"])
  def test_reliability_blocks(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p,uptime_ppm=994_999)).receipt()
    self.assertIn("BLOCKED_RELIABILITY",r["blockers"])
  def test_reserve_blocks(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p,reserve_balance_units=4_999_999)).receipt()
    self.assertIn("BLOCKED_RESERVE",r["blockers"])
  def test_maintenance_blocks(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p,deferred_maintenance_units=2_000_001)).receipt()
    self.assertIn("BLOCKED_MAINTENANCE",r["blockers"])
  def test_continuity_fail_and_not_assessed_block(self):
    p=profile()
    for state in ("FAIL","NOT_ASSESSED"):
      with self.subTest(state=state):
        r=qualify(p,PARENT_RECEIPT,snapshot(p,continuity_state=state)).receipt()
        self.assertIn("BLOCKED_RESILIENCE",r["blockers"])
  def test_multiple_blockers_are_preserved_in_order(self):
    p=profile(); r=qualify(p,PARENT_RECEIPT,snapshot(p,coverage_ppm=1,tariff_units=99_999,reserve_balance_units=0,continuity_state="FAIL")).receipt()
    self.assertEqual(r["blockers"],["BLOCKED_SERVICE_COVERAGE","BLOCKED_AFFORDABILITY","BLOCKED_RESERVE","BLOCKED_RESILIENCE"])
  def test_parent_substitution_rejected(self):
    p=profile(); parent=copy.deepcopy(PARENT_RECEIPT); parent["remaining_claim_units"]+=1
    with self.assertRaisesRegex(GateError,"semantic digest mismatch"): qualify(p,parent,snapshot(p))
  def test_profile_substitution_rejected(self):
    p=profile(); s=snapshot(p); s["profile_sha256"]="0"*64
    with self.assertRaisesRegex(GateError,"profile substitution"): qualify(p,PARENT_RECEIPT,s)
  def test_project_substitution_rejected(self):
    p=profile(); s=snapshot(p); s["project_id"]="fiber:other"
    with self.assertRaisesRegex(GateError,"project substitution"): qualify(p,PARENT_RECEIPT,s)
  def test_parent_receipt_ref_substitution_rejected(self):
    p=profile(); s=snapshot(p); s["parent_transition_receipt_sha256"]="0"*64
    with self.assertRaisesRegex(GateError,"parent substitution"): qualify(p,PARENT_RECEIPT,s)
  def test_unknown_penalty_return_field_fails_closed(self):
    p=profile(); s=snapshot(p); s["penalty_return_units"]=100
    with self.assertRaisesRegex(GateError,"unknown=.*penalty_return_units"): qualify(p,PARENT_RECEIPT,s)
  def test_no_active_claim_is_not_distribution_eligible(self):
    parent=copy.deepcopy(PARENT_RECEIPT); parent["remaining_claim_units"]=0; parent["financial_state"]="RETURN_ENVELOPE_SATISFIED"
    p=profile(parent); s=snapshot(p)
    r=qualify(p,parent,s).receipt()
    self.assertEqual(r["distribution_eligibility"],"NO_ACTIVE_CLAIM")
    self.assertFalse(r["claim_modified"])

if __name__=="__main__": unittest.main()
