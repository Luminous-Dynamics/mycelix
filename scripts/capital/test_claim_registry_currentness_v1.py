#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import claim_registry_currentness_v1 as c
REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002c1/example_case.json").read_text())

class Tests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,x): return c.qualify(x["profile"],x["claim_registry_receipt"],x["designation"],x["events"]).receipt()
    def append(self,x,kind):
        i=len(x["events"])
        e={
          "seq":i,"event_id":f"claim-currentness-event-{i}",
          "project_id":x["profile"]["project_id"],
          "profile_sha256":c.sha256_hex(x["profile"]),
          "designation_id":x["designation"]["designation_id"],
          "prev_event_sha256":None if i==0 else c.sha256_hex(x["events"][-1]),
          "kind":kind,"authority_ref":"authority:claim-registry-currentness-board",
          "evidence_ref":f"evidence:claim-currentness-{i}",
        }
        x["events"].append(e); return e

    def test_positive_current(self):
        r=self.q(self.case())
        self.assertEqual(r["claim_registry_currentness_state"],"CURRENT")
        self.assertEqual(r["active_claim_total_units"],73000000)
        self.assertFalse(r["payment_authority_established"])
        self.assertFalse(r["uses_local_wall_clock"])

    def test_holder_snapshot_commitment(self):
        r=self.q(self.case())
        self.assertEqual(r["holder_snapshot_sha256"],"48de5ec9d27ac4b74b2ef37a6e2584e52bb6a254f46f7f8638adf8f28a05a316")

    def test_wrong_subject_rejected(self):
        x=self.case(); x["profile"]["claim_registry_subject_sha"]="0"*40
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_wrong_receipt_commitment_rejected(self):
        x=self.case(); x["profile"]["claim_registry_receipt_sha256"]="0"*64
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_wrong_registry_profile_rejected(self):
        x=self.case(); x["profile"]["claim_registry_profile_sha256"]="0"*64
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_wrong_history_profile_rejected(self):
        x=self.case(); x["profile"]["operation_history_sha256"]="0"*64
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_wrong_tip_profile_rejected(self):
        x=self.case(); x["profile"]["operation_chain_tip_sha256"]="0"*64
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_receipt_project_substitution_rejected(self):
        x=self.case(); x["claim_registry_receipt"]["project_id"]="fiber:other"
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_receipt_conservation_false_rejected(self):
        x=self.case(); x["claim_registry_receipt"]["conservation_passed"]=False
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_asset_transferability_contamination_rejected(self):
        x=self.case(); x["claim_registry_receipt"]["commons_asset_transferable"]=True
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_active_total_mismatch_rejected(self):
        x=self.case(); x["claim_registry_receipt"]["active_claim_total_units"]-=1
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_designated_receipt_mismatch_stale(self):
        x=self.case(); x["designation"]["designated_claim_registry_receipt_sha256"]="a"*64
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"STALE")

    def test_designated_history_mismatch_stale(self):
        x=self.case(); x["designation"]["designated_operation_history_sha256"]="b"*64
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"STALE")

    def test_designated_tip_mismatch_stale(self):
        x=self.case(); x["designation"]["designated_operation_chain_tip_sha256"]="c"*64
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"STALE")

    def test_pending_designation(self):
        x=self.case(); x["designation"]["designation_state"]="PENDING"
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"PENDING")

    def test_revoked_designation(self):
        x=self.case(); x["designation"]["designation_state"]="REVOKED"
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"REVOKED")

    def test_supersede_tip_stale(self):
        x=self.case(); self.append(x,"SupersedeTip")
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"STALE")

    def test_material_invalidation_stale(self):
        x=self.case(); self.append(x,"MaterialInvalidation")
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"STALE")

    def test_pending_reconciliation(self):
        x=self.case(); self.append(x,"PendingReconciliation")
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"PENDING")

    def test_revoke_event(self):
        x=self.case(); self.append(x,"RevokeEvidence")
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"REVOKED")

    def test_precedence_pending_over_stale(self):
        x=self.case(); self.append(x,"SupersedeTip"); self.append(x,"PendingReconciliation")
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"PENDING")

    def test_precedence_revoked_over_pending(self):
        x=self.case(); self.append(x,"PendingReconciliation"); self.append(x,"RevokeEvidence")
        self.assertEqual(self.q(x)["claim_registry_currentness_state"],"REVOKED")

    def test_broken_chain_rejected(self):
        x=self.case(); self.append(x,"SupersedeTip"); self.append(x,"PendingReconciliation")
        x["events"][1]["prev_event_sha256"]="f"*64
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_duplicate_event_rejected(self):
        x=self.case(); self.append(x,"SupersedeTip"); self.append(x,"PendingReconciliation")
        x["events"][1]["event_id"]=x["events"][0]["event_id"]
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_sequence_gap_rejected(self):
        x=self.case(); e=self.append(x,"SupersedeTip"); e["seq"]=9
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_unknown_event_rejected(self):
        x=self.case(); e=self.append(x,"SupersedeTip"); e["kind"]="Unknown"
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_designation_profile_substitution_rejected(self):
        x=self.case(); x["designation"]["profile_sha256"]="f"*64
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_designation_project_substitution_rejected(self):
        x=self.case(); x["designation"]["project_id"]="fiber:other"
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_unknown_receipt_field_rejected(self):
        x=self.case(); x["claim_registry_receipt"]["payment_authority_established"]=False
        with self.assertRaises(c.ClaimCurrentnessError): self.q(x)

    def test_nonclaims_and_authority_ceiling(self):
        r=self.q(self.case())
        self.assertFalse(r["payment_authority_established"])
        self.assertFalse(r["asset_title_authority_established"])
        self.assertFalse(r["constitutional_authority_established"])
        self.assertFalse(r["operator_authority_established"])
        self.assertIn("claim-registry currentness is not project payment authority",r["nonclaims"])

    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case())
        self.assertEqual(c.canonical_bytes(a),c.canonical_bytes(b))

if __name__=="__main__": unittest.main()
