from __future__ import annotations
import copy, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from digital_operational_sovereignty_v1 import DIMENSIONS, PROFILE_VERSION, SovereigntyError, qualify, sha256_hex

def parent_receipt():
    return {
      "counted_investor_distributions_units":60000000,"created_entitlement_units":133000000,
      "event_chain_tip_sha256":"f766650cbc8233f016ae3482f46708f395264cbc0baeb869ad7c09dd662d240d",
      "event_count":8,"event_history_sha256":"b1e75101137bcecc60daba64aa0d232398b78d65e4e946723bd25e5ab23184b7",
      "financial_state":"CLAIM_ACTIVE","grant_or_subsidy_units":7000000,"handback_accepted":False,
      "impairments_units":0,"initial_principal_units":100000000,"legal_transition_complete":False,
      "nonclaims":["x"],"preferred_return_cap_units":20000000,
      "profile_sha256":"6929b9e089e3f7b272713f6ff8587bf429594a51a9d67d67d0d4468ba72b52e7",
      "profile_version":"mycelix-capital-to-commons-fixed-preferred-v1","project_id":"fiber:jhb:test-001",
      "qualified_new_capital_units":10000000,"receipt_version":"mycelix-commons-transition-receipt-v1",
      "recoverable_lifecycle_units":3000000,"remaining_claim_units":73000000,"required_reserve_units":5000000,
      "reserve_balance_units":5000000,"reserve_compliant":True,"retired_claim_ppm":451127,
      "retired_claim_units":60000000,"unit":"ZAR-cent"}

def profile(parent=None, required=None):
    parent=copy.deepcopy(parent or parent_receipt())
    return {"profile_version":PROFILE_VERSION,"project_id":"fiber:jhb:test-001",
      "parent_subject_sha":"270b852e0ac744dfca3a2cb966bf53fce78f2ab9",
      "parent_transition_receipt_sha256":sha256_hex(parent),
      "required_dimensions":list(required or DIMENSIONS)}

def assessment(p=None, states=None):
    p=copy.deepcopy(p or profile())
    controls={d:"PASS" for d in DIMENSIONS}
    required=set(p["required_dimensions"])
    for d in DIMENSIONS:
        if d not in required: controls[d]="NOT_APPLICABLE"
    if states: controls.update(states)
    return {"assessment_id":"assessment-001","project_id":p["project_id"],"profile_sha256":sha256_hex(p),
      "parent_transition_receipt_sha256":p["parent_transition_receipt_sha256"],
      "outgoing_operator_ref":"operator:incumbent","assessor_ref":"assessor:independent",
      "scope_ref":"scope:digital-handback-v1","controls":controls,"evidence_ref":"evidence:assessment-001"}

class Tests(unittest.TestCase):
    def test_ready(self):
        p=profile(); out=qualify(p,parent_receipt(),assessment(p)).receipt()
        self.assertEqual(out["readiness_state"],"OPERATIONAL_TRANSFER_READY")
        self.assertFalse(out["handover_accepted"]); self.assertFalse(out["legal_transition_complete"])

    def test_root_rotation_failure_requires_remediation(self):
        p=profile(); a=assessment(p,{"trust_root_rotation":"FAIL"})
        out=qualify(p,parent_receipt(),a).receipt()
        self.assertEqual(out["readiness_state"],"REMEDIATION_REQUIRED")
        self.assertIn("FAIL:trust_root_rotation",out["blockers"])

    def test_not_assessed_is_incomplete(self):
        p=profile(); a=assessment(p,{"backup_restore":"NOT_ASSESSED"})
        self.assertEqual(qualify(p,parent_receipt(),a).receipt()["readiness_state"],"ASSESSMENT_INCOMPLETE")

    def test_fail_precedes_not_assessed(self):
        p=profile(); a=assessment(p,{"backup_restore":"NOT_ASSESSED","administrator_recovery":"FAIL"})
        self.assertEqual(qualify(p,parent_receipt(),a).receipt()["readiness_state"],"REMEDIATION_REQUIRED")

    def test_required_not_applicable_rejected(self):
        p=profile(); a=assessment(p); a["controls"]["backup_restore"]="NOT_APPLICABLE"
        with self.assertRaisesRegex(SovereigntyError,"required dimension cannot be NOT_APPLICABLE"): qualify(p,parent_receipt(),a)

    def test_nonrequired_dimension_must_be_not_applicable(self):
        p=profile(required=[d for d in DIMENSIONS if d!="sbom_inventory"])
        a=assessment(p); a["controls"]["sbom_inventory"]="PASS"
        with self.assertRaisesRegex(SovereigntyError,"non-required dimension must be NOT_APPLICABLE"): qualify(p,parent_receipt(),a)

    def test_self_assessment_rejected(self):
        p=profile(); a=assessment(p); a["assessor_ref"]=a["outgoing_operator_ref"]
        with self.assertRaisesRegex(SovereigntyError,"cannot be sole assessor"): qualify(p,parent_receipt(),a)

    def test_unknown_dimension_fails_closed(self):
        p=profile(); a=assessment(p); a["controls"]["root_backdoor"]="PASS"
        with self.assertRaisesRegex(SovereigntyError,"unknown=.*root_backdoor"): qualify(p,parent_receipt(),a)

    def test_legal_acceptance_injection_fails_closed(self):
        p=profile(); a=assessment(p); a["handover_accepted"]=True
        with self.assertRaisesRegex(SovereigntyError,"unknown=.*handover_accepted"): qualify(p,parent_receipt(),a)

    def test_project_substitution_fails(self):
        p=profile(); a=assessment(p); a["project_id"]="other"
        with self.assertRaisesRegex(SovereigntyError,"project substitution"): qualify(p,parent_receipt(),a)

    def test_profile_substitution_fails(self):
        p=profile(); a=assessment(p); a["profile_sha256"]="0"*64
        with self.assertRaisesRegex(SovereigntyError,"profile substitution"): qualify(p,parent_receipt(),a)

    def test_parent_substitution_fails(self):
        p=profile(); parent=parent_receipt(); parent["remaining_claim_units"]-=1
        with self.assertRaisesRegex(SovereigntyError,"semantic digest mismatch"): qualify(p,parent,assessment(p))

    def test_financial_state_is_orthogonal(self):
        active=parent_receipt(); p1=profile(active); a1=assessment(p1)
        out1=qualify(p1,active,a1).receipt()
        satisfied=parent_receipt(); satisfied["remaining_claim_units"]=0; satisfied["financial_state"]="RETURN_ENVELOPE_SATISFIED"
        p2=profile(satisfied); a2=assessment(p2)
        out2=qualify(p2,satisfied,a2).receipt()
        self.assertEqual(out1["readiness_state"],"OPERATIONAL_TRANSFER_READY")
        self.assertEqual(out2["readiness_state"],"OPERATIONAL_TRANSFER_READY")
        self.assertNotEqual(out1["parent_financial_state"],out2["parent_financial_state"])

    def test_zero_claim_does_not_override_failed_control(self):
        parent=parent_receipt(); parent["remaining_claim_units"]=0; parent["financial_state"]="RETURN_ENVELOPE_SATISFIED"
        p=profile(parent); a=assessment(p,{"trust_root_rotation":"FAIL"})
        self.assertEqual(qualify(p,parent,a).receipt()["readiness_state"],"REMEDIATION_REQUIRED")

    def test_deterministic(self):
        p=profile(); a=assessment(p)
        x=qualify(p,parent_receipt(),a).receipt(); y=qualify(copy.deepcopy(p),parent_receipt(),copy.deepcopy(a)).receipt()
        self.assertEqual(sha256_hex(x),sha256_hex(y))

if __name__=="__main__": unittest.main()
