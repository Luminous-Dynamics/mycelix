#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import return_envelope_currentness_v1 as a
REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002a2/example_case.json").read_text())

class Tests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,x): return a.qualify(x["profile"],x["financial_receipt"],x["designation"],x["events"]).receipt()
    def append(self,x,kind):
        i=len(x["events"])
        e={"seq":i,"event_id":f"financial-currentness-event-{i}","project_id":x["profile"]["project_id"],"profile_sha256":a.sha256_hex(x["profile"]),"designation_id":x["designation"]["designation_id"],"prev_event_sha256":None if i==0 else a.sha256_hex(x["events"][-1]),"kind":kind,"authority_ref":"authority:return-envelope-currentness-board","evidence_ref":f"evidence:return-envelope-currentness-{i}"}
        x["events"].append(e); return e

    def test_positive_current(self):
        r=self.q(self.case()); self.assertEqual(r["financial_checkpoint_currentness_state"],"CURRENT"); self.assertEqual(r["financial_state"],"CLAIM_ACTIVE"); self.assertEqual(r["remaining_claim_units"],73000000); self.assertTrue(r["reserve_compliant"]); self.assertFalse(r["payment_authority_established"]); self.assertFalse(r["uses_local_wall_clock"])
    def test_wrong_subject_rejected(self):
        x=self.case(); x["profile"]["financial_subject_sha"]="0"*40
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_wrong_receipt_commitment_rejected(self):
        x=self.case(); x["profile"]["financial_receipt_sha256"]="0"*64
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_wrong_financial_profile_rejected(self):
        x=self.case(); x["profile"]["financial_profile_sha256"]="0"*64
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_wrong_history_rejected(self):
        x=self.case(); x["profile"]["event_history_sha256"]="0"*64
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_wrong_tip_rejected(self):
        x=self.case(); x["profile"]["event_chain_tip_sha256"]="0"*64
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_unit_substitution_rejected(self):
        x=self.case(); x["profile"]["unit"]="USD-cent"
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_receipt_project_substitution_rejected(self):
        x=self.case(); x["financial_receipt"]["project_id"]="fiber:other"
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_receipt_unit_substitution_rejected(self):
        x=self.case(); x["financial_receipt"]["unit"]="USD-cent"
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_receipt_handback_contamination_rejected(self):
        x=self.case(); x["financial_receipt"]["handback_accepted"]=True
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_receipt_legal_contamination_rejected(self):
        x=self.case(); x["financial_receipt"]["legal_transition_complete"]=True
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_unknown_source_field_rejected(self):
        x=self.case(); x["financial_receipt"]["payment_authority_established"]=False
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_designated_receipt_mismatch_stale(self):
        x=self.case(); x["designation"]["designated_financial_receipt_sha256"]="a"*64; self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"STALE")
    def test_designated_history_mismatch_stale(self):
        x=self.case(); x["designation"]["designated_event_history_sha256"]="b"*64; self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"STALE")
    def test_designated_tip_mismatch_stale(self):
        x=self.case(); x["designation"]["designated_event_chain_tip_sha256"]="c"*64; self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"STALE")
    def test_pending_designation(self):
        x=self.case(); x["designation"]["designation_state"]="PENDING"; self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"PENDING")
    def test_revoked_designation(self):
        x=self.case(); x["designation"]["designation_state"]="REVOKED"; self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"REVOKED")
    def test_supersede_checkpoint_stale(self):
        x=self.case(); self.append(x,"SupersedeCheckpoint"); self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"STALE")
    def test_material_invalidation_stale(self):
        x=self.case(); self.append(x,"MaterialInvalidation"); self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"STALE")
    def test_pending_reconciliation(self):
        x=self.case(); self.append(x,"PendingReconciliation"); self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"PENDING")
    def test_revoke_event(self):
        x=self.case(); self.append(x,"RevokeEvidence"); self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"REVOKED")
    def test_pending_beats_stale(self):
        x=self.case(); self.append(x,"SupersedeCheckpoint"); self.append(x,"PendingReconciliation"); self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"PENDING")
    def test_revoked_beats_pending(self):
        x=self.case(); self.append(x,"PendingReconciliation"); self.append(x,"RevokeEvidence"); self.assertEqual(self.q(x)["financial_checkpoint_currentness_state"],"REVOKED")
    def test_broken_chain_rejected(self):
        x=self.case(); self.append(x,"SupersedeCheckpoint"); self.append(x,"PendingReconciliation"); x["events"][1]["prev_event_sha256"]="f"*64
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_duplicate_event_rejected(self):
        x=self.case(); self.append(x,"SupersedeCheckpoint"); self.append(x,"PendingReconciliation"); x["events"][1]["event_id"]=x["events"][0]["event_id"]
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_sequence_gap_rejected(self):
        x=self.case(); e=self.append(x,"SupersedeCheckpoint"); e["seq"]=8
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_unknown_event_rejected(self):
        x=self.case(); e=self.append(x,"SupersedeCheckpoint"); e["kind"]="TimeTravel"
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_designation_profile_substitution_rejected(self):
        x=self.case(); x["designation"]["profile_sha256"]="f"*64
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_designation_project_substitution_rejected(self):
        x=self.case(); x["designation"]["project_id"]="fiber:other"
        with self.assertRaises(a.FinancialCurrentnessError): self.q(x)
    def test_authority_ceiling(self):
        r=self.q(self.case())
        for k in ("payment_authority_established","accounting_compliance_established","tax_compliance_established","securities_law_compliance_established","legal_title_transition_established","handover_accepted","uses_local_wall_clock"): self.assertFalse(r[k])
    def test_deterministic(self):
        a1=self.q(self.case()); a2=self.q(self.case()); self.assertEqual(a.canonical_bytes(a1),a.canonical_bytes(a2))

if __name__=="__main__": unittest.main()
