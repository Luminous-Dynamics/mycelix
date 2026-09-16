from __future__ import annotations
import copy, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from service_currentness_v1 import CurrentnessError, PROFILE_VERSION, qualify, sha256_hex

def service_receipt():
    return {
      "blockers": [], "claim_modified": False, "distribution_eligibility": "ELIGIBLE",
      "measurement_epoch": 1, "measurement_id": "measure-001",
      "nonclaims": ["x"],
      "observed": {"continuity_state":"PASS","coverage_ppm":980000,"deferred_maintenance_units":1000000,"reserve_balance_units":5000000,"tariff_units":12000,"uptime_ppm":999000},
      "parent_remaining_claim_units":73000000,
      "parent_subject_sha":"270b852e0ac744dfca3a2cb966bf53fce78f2ab9",
      "parent_transition_receipt_sha256":"60b7217980c615eab453eae25b74b27e75fc67cb336ea60d140f089341d50565",
      "profile_sha256":"18dc049978d41bfda3e2d5c48280ccaa37f7e03d5a04cbedc50af80db49e0937",
      "profile_version":"mycelix-public-service-distribution-gate-v1",
      "project_id":"fiber:jhb:test-001","receipt_version":"mycelix-public-service-gate-receipt-v1",
      "snapshot_sha256":"06276fa5f8c6622355bdcb874fe43f924c2b942bdf5991a11598e4a5de8269bf","unit":"ZAR-cent"}

def profile():
    return {"profile_version":PROFILE_VERSION,"project_id":"fiber:jhb:test-001",
      "service_gate_subject_sha":"92fe8408b52ce1b63808201e9f70ab5280a6f489",
      "service_gate_profile_sha256":"18dc049978d41bfda3e2d5c48280ccaa37f7e03d5a04cbedc50af80db49e0937",
      "designation_registry_id":"service-currentness-registry-v1","max_events":100}

def designation(p=None,r=None):
    p=copy.deepcopy(p or profile()); r=copy.deepcopy(r or service_receipt())
    return {"designation_id":"designation-001","registry_id":"service-currentness-registry-v1","registry_epoch":7,
      "project_id":p["project_id"],"profile_sha256":sha256_hex(p),"designated_measurement_id":r["measurement_id"],
      "designated_service_receipt_sha256":sha256_hex(r),"designation_state":"ACTIVE",
      "authority_ref":"authority:service-steward","evidence_ref":"evidence:designation-001"}

def event(kind,target="measure-001",seq=0,prev=None,p=None,d=None):
    p=p or profile(); d=d or designation(p)
    return {"seq":seq,"event_id":f"event-{seq}","project_id":p["project_id"],"profile_sha256":sha256_hex(p),
      "designation_id":d["designation_id"],"prev_event_sha256":prev,"kind":kind,
      "target_measurement_id":target,"authority_ref":f"authority:{seq}","evidence_ref":f"evidence:{seq}"}

class Tests(unittest.TestCase):
    def test_current(self):
        p=profile(); r=service_receipt(); d=designation(p,r)
        out=qualify(p,r,d,[]).receipt()
        self.assertEqual(out["currentness_state"],"CURRENT")
        self.assertFalse(out["claim_modified"]); self.assertFalse(out["uses_local_wall_clock"])

    def test_wrong_designated_measurement_is_stale(self):
        p=profile(); r=service_receipt(); d=designation(p,r); d["designated_measurement_id"]="measure-002"
        self.assertEqual(qualify(p,r,d,[]).receipt()["currentness_state"],"STALE")

    def test_wrong_designated_receipt_is_stale(self):
        p=profile(); r=service_receipt(); d=designation(p,r); d["designated_service_receipt_sha256"]="0"*64
        self.assertEqual(qualify(p,r,d,[]).receipt()["currentness_state"],"STALE")

    def test_pending_designation(self):
        p=profile(); r=service_receipt(); d=designation(p,r); d["designation_state"]="PENDING"
        self.assertEqual(qualify(p,r,d,[]).receipt()["currentness_state"],"PENDING")

    def test_revoked_designation(self):
        p=profile(); r=service_receipt(); d=designation(p,r); d["designation_state"]="REVOKED"
        self.assertEqual(qualify(p,r,d,[]).receipt()["currentness_state"],"REVOKED")

    def test_material_invalidation(self):
        p=profile(); r=service_receipt(); d=designation(p,r); ev=[event("MaterialInvalidation",p=p,d=d)]
        self.assertEqual(qualify(p,r,d,ev).receipt()["currentness_state"],"STALE")

    def test_pending_remeasurement(self):
        p=profile(); r=service_receipt(); d=designation(p,r); ev=[event("PendingRemeasurement",p=p,d=d)]
        self.assertEqual(qualify(p,r,d,ev).receipt()["currentness_state"],"PENDING")

    def test_revoke_event(self):
        p=profile(); r=service_receipt(); d=designation(p,r); ev=[event("RevokeEvidence",p=p,d=d)]
        self.assertEqual(qualify(p,r,d,ev).receipt()["currentness_state"],"REVOKED")

    def test_other_measurement_event_does_not_invalidate(self):
        p=profile(); r=service_receipt(); d=designation(p,r); ev=[event("MaterialInvalidation","measure-other",p=p,d=d)]
        self.assertEqual(qualify(p,r,d,ev).receipt()["currentness_state"],"CURRENT")

    def test_precedence_preserves_all_blockers(self):
        p=profile(); r=service_receipt(); d=designation(p,r)
        a=event("MaterialInvalidation",p=p,d=d)
        b=event("PendingRemeasurement",seq=1,prev=sha256_hex(a),p=p,d=d)
        c=event("RevokeEvidence",seq=2,prev=sha256_hex(b),p=p,d=d)
        out=qualify(p,r,d,[a,b,c]).receipt()
        self.assertEqual(out["currentness_state"],"REVOKED")
        self.assertEqual(set(out["blockers"]),{"MATERIAL_INVALIDATION","PENDING_REMEASUREMENT","EVIDENCE_REVOKED"})

    def test_project_substitution_fails(self):
        p=profile(); r=service_receipt(); r["project_id"]="other"
        with self.assertRaisesRegex(CurrentnessError,"project mismatch"): qualify(p,r,designation(p,service_receipt()),[])

    def test_service_profile_substitution_fails(self):
        p=profile(); r=service_receipt(); r["profile_sha256"]="0"*64
        with self.assertRaisesRegex(CurrentnessError,"service profile mismatch"): qualify(p,r,designation(p,service_receipt()),[])

    def test_designation_registry_substitution_fails(self):
        p=profile(); r=service_receipt(); d=designation(p,r); d["registry_id"]="evil"
        with self.assertRaisesRegex(CurrentnessError,"registry substitution"): qualify(p,r,d,[])

    def test_broken_event_chain_fails(self):
        p=profile(); r=service_receipt(); d=designation(p,r)
        a=event("MaterialInvalidation",p=p,d=d); b=event("RevokeEvidence",seq=1,prev="0"*64,p=p,d=d)
        with self.assertRaisesRegex(CurrentnessError,"broken event chain"): qualify(p,r,d,[a,b])

    def test_unknown_profile_field_fails_closed(self):
        p=profile(); p["max_age_seconds"]=60
        with self.assertRaisesRegex(CurrentnessError,"unknown=.*max_age_seconds"): qualify(p,service_receipt(),designation(profile(),service_receipt()),[])

    def test_wall_clock_field_fails_closed(self):
        p=profile(); r=service_receipt(); d=designation(p,r); d["expires_at"]="2099-01-01T00:00:00Z"
        with self.assertRaisesRegex(CurrentnessError,"unknown=.*expires_at"): qualify(p,r,d,[])

if __name__=="__main__": unittest.main()
