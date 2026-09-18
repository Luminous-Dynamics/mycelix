#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import custody_tip_currentness_v1 as c

REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002d3a/example_case.json").read_text(encoding="utf-8"))

class TipCurrentnessTests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,x): return c.qualify(x["profile"],x["d3_receipt"],x["designation"],x["events"]).receipt()
    def append(self,x,kind):
        i=len(x["events"])
        e={
            "seq":i,
            "event_id":f"custody-tip-currentness-evt-{i}",
            "project_id":x["profile"]["project_id"],
            "profile_sha256":c.sha256_hex(x["profile"]),
            "designation_id":x["designation"]["designation_id"],
            "prev_event_sha256":None if i==0 else c.sha256_hex(x["events"][-1]),
            "kind":kind,
            "authority_ref":"authority:custody-currentness-board",
            "evidence_ref":f"evidence:custody-tip-currentness-{i}",
        }
        x["events"].append(e); return e

    def test_positive_current(self):
        r=self.q(self.case())
        self.assertEqual(r["custody_tip_currentness_state"],"CURRENT")
        self.assertEqual(r["d3_custody_state"],"CURRENT")
        self.assertFalse(r["custody_health_established"])
        self.assertFalse(r["uses_local_wall_clock"])

    def test_currentness_orthogonal_to_d3_health(self):
        x=self.case()
        x["d3_receipt"]["custody_state"]="DEGRADED"
        x["profile"]["d3_receipt_sha256"]=c.sha256_hex(x["d3_receipt"])
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_profile_wrong_subject_rejected(self):
        x=self.case(); x["profile"]["d3_subject_sha"]="0"*40
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_profile_wrong_receipt_rejected(self):
        x=self.case(); x["profile"]["d3_receipt_sha256"]="0"*64
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_profile_wrong_d3_profile_rejected(self):
        x=self.case(); x["profile"]["d3_profile_sha256"]="0"*64
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_profile_wrong_history_rejected(self):
        x=self.case(); x["profile"]["d3_event_history_sha256"]="0"*64
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_profile_wrong_tip_rejected(self):
        x=self.case(); x["profile"]["d3_event_tip_sha256"]="0"*64
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_designation_old_subject_is_stale(self):
        x=self.case(); x["designation"]["designated_d3_subject_sha"]="0"*40
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"STALE")

    def test_designation_old_receipt_is_stale(self):
        x=self.case(); x["designation"]["designated_d3_receipt_sha256"]="0"*64
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"STALE")

    def test_designation_old_profile_is_stale(self):
        x=self.case(); x["designation"]["designated_d3_profile_sha256"]="0"*64
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"STALE")

    def test_designation_old_history_is_stale(self):
        x=self.case(); x["designation"]["designated_event_history_sha256"]="0"*64
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"STALE")

    def test_designation_old_tip_is_stale(self):
        x=self.case(); x["designation"]["designated_event_tip_sha256"]="0"*64
        r=self.q(x); self.assertEqual(r["custody_tip_currentness_state"],"STALE")
        self.assertIn("EVENT_TIP_NOT_DESIGNATED_CURRENT",r["blockers"])

    def test_pending_designation(self):
        x=self.case(); x["designation"]["designation_state"]="PENDING"
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"PENDING")

    def test_revoked_designation(self):
        x=self.case(); x["designation"]["designation_state"]="REVOKED"
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"REVOKED")

    def test_supersede_tip_event(self):
        x=self.case(); self.append(x,"SupersedeTip")
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"STALE")

    def test_pending_reconciliation(self):
        x=self.case(); self.append(x,"PendingReconciliation")
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"PENDING")

    def test_material_invalidation(self):
        x=self.case(); self.append(x,"MaterialInvalidation")
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"STALE")

    def test_revoke_evidence(self):
        x=self.case(); self.append(x,"RevokeEvidence")
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"REVOKED")

    def test_precedence_pending_over_stale(self):
        x=self.case(); self.append(x,"SupersedeTip"); self.append(x,"PendingReconciliation")
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"PENDING")

    def test_precedence_revoked_over_pending(self):
        x=self.case(); self.append(x,"PendingReconciliation"); self.append(x,"RevokeEvidence")
        self.assertEqual(self.q(x)["custody_tip_currentness_state"],"REVOKED")

    def test_broken_event_chain_rejected(self):
        x=self.case(); self.append(x,"SupersedeTip"); self.append(x,"PendingReconciliation")
        x["events"][1]["prev_event_sha256"]="f"*64
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_duplicate_event_id_rejected(self):
        x=self.case(); self.append(x,"SupersedeTip"); self.append(x,"PendingReconciliation")
        x["events"][1]["event_id"]=x["events"][0]["event_id"]
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_sequence_gap_rejected(self):
        x=self.case(); e=self.append(x,"SupersedeTip"); e["seq"]=2
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_unknown_event_kind_rejected(self):
        x=self.case(); e=self.append(x,"SupersedeTip"); e["kind"]="ClockTick"
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_project_substitution_rejected(self):
        x=self.case(); x["designation"]["project_id"]="fiber:other"
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_designation_profile_substitution_rejected(self):
        x=self.case(); x["designation"]["profile_sha256"]="f"*64
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_d3_currentness_contamination_rejected(self):
        x=self.case(); x["d3_receipt"]["event_tip_currentness_established"]=True
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_d3_wall_clock_contamination_rejected(self):
        x=self.case(); x["d3_receipt"]["uses_local_wall_clock"]=True
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_d3_legal_authority_contamination_rejected(self):
        x=self.case(); x["d3_receipt"]["legal_title_transition_established"]=True
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_unknown_d3_field_rejected(self):
        x=self.case(); x["d3_receipt"]["execution_authority_established"]=False
        with self.assertRaises(c.TipCurrentnessError): self.q(x)

    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case())
        self.assertEqual(c.canonical_bytes(a),c.canonical_bytes(b))

if __name__=="__main__":
    unittest.main()
