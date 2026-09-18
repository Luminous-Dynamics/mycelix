#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import composition_dimension_projection_v1 as h
REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002h0a/example_case.json").read_text(encoding="utf-8"))

class ProjectionTests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,x): return h.qualify(x).receipt()
    def dim(self,r,name): return next(d for d in r["dimensions"] if d["dimension_id"]==name)

    def test_positive_bundle(self):
        r=self.q(self.case()); self.assertEqual(len(r["dimensions"]),3); self.assertFalse(r["execution_authority_established"])
    def test_h1_mapping(self):
        d=self.dim(self.q(self.case()),"STEWARDSHIP_GOVERNANCE"); self.assertEqual((d["semantic_state"],d["currentness_state"]),("CURRENT","CURRENT"))
    def test_h2_eligible_current(self):
        d=self.dim(self.q(self.case()),"PUBLIC_SERVICE_DISTRIBUTION"); self.assertEqual((d["semantic_state"],d["currentness_state"]),("ELIGIBLE","CURRENT"))
    def test_h2_blocked_current(self):
        x=self.case(); x["h2_receipt"]["public_service_distribution_state"]="BLOCKED_SERVICE"
        d=h.project_h2(x["h2_receipt"],x["profile"]); self.assertEqual((d["semantic_state"],d["currentness_state"]),("BLOCKED_SERVICE","CURRENT"))
    def test_h2_stale_conservative(self):
        x=self.case(); x["h2_receipt"]["public_service_distribution_state"]="STALE"
        d=h.project_h2(x["h2_receipt"],x["profile"]); self.assertEqual((d["semantic_state"],d["currentness_state"]),("NOT_SEPARATELY_EXPOSED","STALE"))
    def test_h2_pending_conservative(self):
        x=self.case(); x["h2_receipt"]["public_service_distribution_state"]="PENDING"
        d=h.project_h2(x["h2_receipt"],x["profile"]); self.assertEqual(d["currentness_state"],"PENDING")
    def test_h2_revoked_conservative(self):
        x=self.case(); x["h2_receipt"]["public_service_distribution_state"]="REVOKED"
        d=h.project_h2(x["h2_receipt"],x["profile"]); self.assertEqual(d["currentness_state"],"REVOKED")
    def test_h2_no_active_claim(self):
        x=self.case(); x["h2_receipt"]["public_service_distribution_state"]="NO_ACTIVE_CLAIM"
        d=h.project_h2(x["h2_receipt"],x["profile"]); self.assertEqual((d["semantic_state"],d["currentness_state"]),("NO_ACTIVE_CLAIM","CURRENT"))
    def test_h3_mapping(self):
        d=self.dim(self.q(self.case()),"HANDBACK_READINESS"); self.assertEqual((d["semantic_state"],d["currentness_state"]),("HANDOVER_READY","NOT_ESTABLISHED"))
    def test_h3_remediation_still_not_current(self):
        x=self.case(); x["h3_receipt"]["handback_readiness_state"]="REMEDIATION_REQUIRED"
        d=h.project_h3(x["h3_receipt"],x["profile"]); self.assertEqual((d["semantic_state"],d["currentness_state"]),("REMEDIATION_REQUIRED","NOT_ESTABLISHED"))
    def test_h3_currentness_contamination(self):
        x=self.case(); x["h3_receipt"]["currentness_established"]=True
        with self.assertRaises(h.ProjectionError): h.project_h3(x["h3_receipt"],x["profile"])
    def test_h1_execution_contamination(self):
        x=self.case(); x["h1_receipt"]["execution_authority_established"]=True
        with self.assertRaises(h.ProjectionError): h.project_h1(x["h1_receipt"],x["profile"])
    def test_h2_payment_contamination(self):
        x=self.case(); x["h2_receipt"]["payment_authority_established"]=True
        with self.assertRaises(h.ProjectionError): h.project_h2(x["h2_receipt"],x["profile"])
    def test_h3_handover_contamination(self):
        x=self.case(); x["h3_receipt"]["handover_accepted"]=True
        with self.assertRaises(h.ProjectionError): h.project_h3(x["h3_receipt"],x["profile"])
    def test_unknown_h1_field(self):
        x=self.case(); x["h1_receipt"]["new"]=1
        with self.assertRaises(h.ProjectionError): h.project_h1(x["h1_receipt"],x["profile"])
    def test_wrong_h1_version(self):
        x=self.case(); x["h1_receipt"]["receipt_version"]="wrong"
        with self.assertRaises(h.ProjectionError): h.project_h1(x["h1_receipt"],x["profile"])
    def test_wrong_frozen_subject_binding(self):
        x=self.case(); x["profile"]["sources"]["STEWARDSHIP_GOVERNANCE"]["subject_sha"]="0"*40
        with self.assertRaises(h.ProjectionError): self.q(x)
    def test_cross_project_bundle(self):
        x=self.case(); x["h2_receipt"]["project_id"]="fiber:other"
        with self.assertRaises(h.ProjectionError): self.q(x)
    def test_source_blockers_preserved(self):
        x=self.case(); x["h2_receipt"]["blockers"]=["Z","A","Z"]
        d=h.project_h2(x["h2_receipt"],x["profile"]); self.assertEqual(d["blockers"],["A","Z"])
    def test_authority_ceiling_h1(self):
        d=self.dim(self.q(self.case()),"STEWARDSHIP_GOVERNANCE"); self.assertEqual(d["authority_ceiling"],"GOVERNANCE_STATE_ONLY")
    def test_authority_ceiling_h2(self):
        d=self.dim(self.q(self.case()),"PUBLIC_SERVICE_DISTRIBUTION"); self.assertEqual(d["authority_ceiling"],"SERVICE_DISTRIBUTION_STATE_ONLY")
    def test_authority_ceiling_h3(self):
        d=self.dim(self.q(self.case()),"HANDBACK_READINESS"); self.assertEqual(d["authority_ceiling"],"READINESS_ONLY")
    def test_no_projection_adds_authority(self):
        for d in self.q(self.case())["dimensions"]:
            self.assertFalse(d["execution_authority_established"]); self.assertFalse(d["payment_authority_established"])
            self.assertFalse(d["legal_title_authority_established"]); self.assertFalse(d["constitutional_authority_established"])
    def test_source_identity_digest_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case()); self.assertEqual(a["source_identity_sha256"],b["source_identity_sha256"])
    def test_dimension_order_deterministic(self):
        ids=[d["dimension_id"] for d in self.q(self.case())["dimensions"]]; self.assertEqual(ids,sorted(ids))
    def test_deterministic_bundle(self):
        a=self.q(self.case()); b=self.q(self.case()); self.assertEqual(h.canonical_bytes(a),h.canonical_bytes(b))

if __name__=="__main__": unittest.main()
