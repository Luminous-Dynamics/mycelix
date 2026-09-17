#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import handback_readiness_currentness_v1 as h

REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002h3a/example_case.json").read_text(encoding="utf-8"))

class CurrentnessTests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,x): return h.qualify(x["profile"],x["handback_receipt"],x["designation"],x["events"]).receipt()

    def rebind_h3(self,x):
        x["profile"]["handback_receipt_sha256"]=h.sha256_hex(x["handback_receipt"])
        x["designation"]["profile_sha256"]=h.sha256_hex(x["profile"])
        x["designation"]["designated_handback_receipt_sha256"]=h.sha256_hex(x["handback_receipt"])
        for e in x["events"]:
            e["profile_sha256"]=h.sha256_hex(x["profile"])

    def append(self,x,kind):
        i=len(x["events"])
        e={
            "seq":i,
            "event_id":f"handback-currentness-evt-{i}",
            "project_id":x["profile"]["project_id"],
            "profile_sha256":h.sha256_hex(x["profile"]),
            "designation_id":x["designation"]["designation_id"],
            "prev_event_sha256":None if i==0 else h.sha256_hex(x["events"][-1]),
            "kind":kind,
            "authority_ref":"authority:handback-currentness-board",
            "evidence_ref":f"evidence:handback-currentness-{i}",
        }
        x["events"].append(e)
        return e

    def test_positive_current(self):
        r=self.q(self.case())
        self.assertEqual(r["handback_readiness_currentness_state"],"CURRENT")
        self.assertEqual(r["handback_readiness_state"],"HANDOVER_READY")
        self.assertFalse(r["uses_local_wall_clock"])

    def test_currentness_orthogonal_to_readiness_outcome(self):
        x=self.case()
        x["handback_receipt"]["handback_readiness_state"]="REMEDIATION_REQUIRED"
        self.rebind_h3(x)
        r=self.q(x)
        self.assertEqual(r["handback_readiness_currentness_state"],"CURRENT")
        self.assertEqual(r["handback_readiness_state"],"REMEDIATION_REQUIRED")

    def test_wrong_designated_receipt_is_stale(self):
        x=self.case(); x["designation"]["designated_handback_receipt_sha256"]="a"*64
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"STALE")

    def test_pending_designation(self):
        x=self.case(); x["designation"]["designation_state"]="PENDING"
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"PENDING")

    def test_revoked_designation(self):
        x=self.case(); x["designation"]["designation_state"]="REVOKED"
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"REVOKED")

    def test_material_invalidation(self):
        x=self.case(); self.append(x,"MaterialInvalidation")
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"STALE")

    def test_pending_reassessment(self):
        x=self.case(); self.append(x,"PendingReassessment")
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"PENDING")

    def test_revoke_evidence(self):
        x=self.case(); self.append(x,"RevokeEvidence")
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"REVOKED")

    def test_precedence_pending_over_stale(self):
        x=self.case(); self.append(x,"MaterialInvalidation"); self.append(x,"PendingReassessment")
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"PENDING")

    def test_precedence_revoked_over_pending(self):
        x=self.case(); self.append(x,"PendingReassessment"); self.append(x,"RevokeEvidence")
        self.assertEqual(self.q(x)["handback_readiness_currentness_state"],"REVOKED")

    def test_project_substitution_in_h3_rejected(self):
        x=self.case(); x["handback_receipt"]["project_id"]="fiber:other"
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_designation_project_substitution_rejected(self):
        x=self.case(); x["designation"]["project_id"]="fiber:other"
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_profile_substitution_rejected(self):
        x=self.case(); x["designation"]["profile_sha256"]="f"*64
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_invalid_handback_subject_rejected(self):
        x=self.case(); x["profile"]["handback_subject_sha"]="not-a-sha"
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_broken_event_chain_rejected(self):
        x=self.case(); self.append(x,"MaterialInvalidation"); self.append(x,"PendingReassessment")
        x["events"][1]["prev_event_sha256"]="f"*64
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_duplicate_event_id_rejected(self):
        x=self.case(); self.append(x,"MaterialInvalidation"); self.append(x,"PendingReassessment")
        x["events"][1]["event_id"]=x["events"][0]["event_id"]
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_sequence_gap_rejected(self):
        x=self.case(); e=self.append(x,"MaterialInvalidation"); e["seq"]=7
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_unknown_event_kind_rejected(self):
        x=self.case(); e=self.append(x,"MaterialInvalidation"); e["kind"]="TimeTravel"
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_currentness_contaminated_h3_rejected(self):
        x=self.case(); x["handback_receipt"]["currentness_established"]=True; self.rebind_h3(x)
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_handover_authority_contamination_rejected(self):
        x=self.case(); x["handback_receipt"]["handover_accepted"]=True; self.rebind_h3(x)
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_execution_authority_contamination_rejected(self):
        x=self.case(); x["handback_receipt"]["execution_authority_established"]=True; self.rebind_h3(x)
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_unknown_h3_field_rejected(self):
        x=self.case(); x["handback_receipt"]["payment_authority_established"]=False; self.rebind_h3(x)
        with self.assertRaises(h.CurrentnessError): self.q(x)

    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case())
        self.assertEqual(h.canonical_bytes(a),h.canonical_bytes(b))

if __name__=="__main__":
    unittest.main()
