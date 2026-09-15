#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import handover_acceptance_v1 as h

REPO_ROOT = HERE.parents[1]
CASE = json.loads((REPO_ROOT / "docs/capital/evidence/myc-cap-002d2/example_case.json").read_text(encoding="utf-8"))

class AcceptanceTests(unittest.TestCase):
    def case(self):
        return copy.deepcopy(CASE)

    def qualify(self, c):
        return h.qualify(c["profile"], c["readiness_receipt"], c["readiness_designation"], c["acceptance_event"]).receipt()

    def test_positive_acceptance(self):
        r = self.qualify(self.case())
        self.assertEqual(r["handover_acceptance_state"], "CUSTODY_ACCEPTED")
        self.assertTrue(r["operational_custody_accepted"])
        self.assertFalse(r["legal_title_transition_established"])
        self.assertFalse(r["constitutional_stewardship_transition_established"])

    def test_allowed_waiver(self):
        c=self.case()
        c["acceptance_event"]["inventory"]["operational_documentation"]="MISSING"
        c["acceptance_event"]["waivers"]=[{
            "waiver_id":"waiver-001",
            "requirement":"operational_documentation",
            "authority_ref":"authority:commons-waiver-board",
            "reason_ref":"reason:docs-remediation",
            "remediation_owner_ref":"owner:incoming-operator",
            "remediation_epoch_ref":"epoch:handover-plus-30d",
            "retention_ref":"retention:performance-security-001",
        }]
        r=self.qualify(c)
        self.assertEqual(r["handover_acceptance_state"], "CUSTODY_ACCEPTED_WITH_WAIVERS")
        self.assertEqual(r["missing_inventory"], ["operational_documentation"])

    def test_not_ready_rejected(self):
        c=self.case()
        c["readiness_receipt"]["readiness_state"]="REMEDIATION_REQUIRED"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_stale_designation_rejected(self):
        c=self.case()
        c["readiness_designation"]["designated_readiness_receipt_sha256"]="0"*64
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_pending_designation_rejected(self):
        c=self.case()
        c["readiness_designation"]["state"]="PENDING"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_revoked_designation_rejected(self):
        c=self.case()
        c["readiness_designation"]["state"]="REVOKED"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_readiness_registry_substitution_rejected(self):
        c=self.case(); c["readiness_designation"]["registry_id"]="registry:attacker"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_readiness_designation_authority_substitution_rejected(self):
        c=self.case(); c["readiness_designation"]["authority_ref"]="authority:attacker"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_wrong_acceptance_authority_rejected(self):
        c=self.case()
        c["acceptance_event"]["acceptance_authority_ref"]="authority:operator"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_wrong_custodian_rejected(self):
        c=self.case()
        c["acceptance_event"]["incoming_custodian_ref"]="operator:incumbent"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_nonwaivable_missing_rejected(self):
        c=self.case()
        c["acceptance_event"]["inventory"]["trust_root_transition_material"]="MISSING"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_waivable_missing_without_waiver_rejected(self):
        c=self.case()
        c["acceptance_event"]["inventory"]["operational_documentation"]="MISSING"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_waiver_for_present_item_rejected(self):
        c=self.case()
        c["acceptance_event"]["waivers"]=[{
            "waiver_id":"waiver-001","requirement":"operational_documentation",
            "authority_ref":"authority:commons-waiver-board","reason_ref":"reason:x",
            "remediation_owner_ref":"owner:y","remediation_epoch_ref":"epoch:z","retention_ref":"retention:r"}]
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_unauthorized_waiver_rejected(self):
        c=self.case()
        c["acceptance_event"]["inventory"]["operational_documentation"]="MISSING"
        c["acceptance_event"]["waivers"]=[{
            "waiver_id":"waiver-001","requirement":"operational_documentation",
            "authority_ref":"authority:operator","reason_ref":"reason:x",
            "remediation_owner_ref":"owner:y","remediation_epoch_ref":"epoch:z","retention_ref":"retention:r"}]
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_duplicate_waiver_requirement_rejected(self):
        c=self.case()
        c["acceptance_event"]["inventory"]["operational_documentation"]="MISSING"
        w={"waiver_id":"waiver-001","requirement":"operational_documentation",
           "authority_ref":"authority:commons-waiver-board","reason_ref":"reason:x",
           "remediation_owner_ref":"owner:y","remediation_epoch_ref":"epoch:z","retention_ref":"retention:r"}
        w2=copy.deepcopy(w); w2["waiver_id"]="waiver-002"
        c["acceptance_event"]["waivers"]=[w,w2]
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_continuity_failure_rejected(self):
        c=self.case(); c["acceptance_event"]["continuity_state"]="FAIL"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_project_substitution_rejected(self):
        c=self.case(); c["acceptance_event"]["project_id"]="fiber:other"
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_profile_substitution_rejected(self):
        c=self.case(); c["acceptance_event"]["profile_sha256"]="f"*64
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_readiness_authority_contamination_rejected(self):
        c=self.case(); c["readiness_receipt"]["handover_accepted"]=True
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_injected_legal_title_field_fails_closed(self):
        c=self.case(); c["acceptance_event"]["legal_title_transition_established"]=True
        with self.assertRaises(h.AcceptanceError): self.qualify(c)

    def test_receipt_is_deterministic(self):
        a=self.qualify(self.case()); b=self.qualify(self.case())
        self.assertEqual(h.canonical_bytes(a), h.canonical_bytes(b))

if __name__ == "__main__":
    unittest.main()
