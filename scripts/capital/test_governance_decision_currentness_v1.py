#!/usr/bin/env python3
from __future__ import annotations
import copy
import json
import unittest
from pathlib import Path
import importlib.util
import sys

HERE = Path(__file__).resolve()
ROOT = HERE.parents[2]
MOD_PATH = ROOT / "scripts/capital/governance_decision_currentness_v1.py"
FIXTURE = ROOT / "docs/capital/evidence/myc-cap-002g2a/example_case.json"

spec = importlib.util.spec_from_file_location("g2a", MOD_PATH)
g2a = importlib.util.module_from_spec(spec)
sys.modules["g2a"] = g2a
assert spec.loader
spec.loader.exec_module(g2a)

def load_case():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))

class CurrentnessTests(unittest.TestCase):
    def derive(self, case):
        return g2a.derive(case).receipt()

    def test_current(self):
        self.assertEqual(self.derive(load_case())["currentness_state"], "CURRENT")

    def test_pending_designation(self):
        c=load_case(); c["designation"]["state"]="PENDING"
        self.assertEqual(self.derive(c)["currentness_state"], "PENDING")

    def test_revoked_designation_not_assessed(self):
        c=load_case(); c["designation"]["state"]="REVOKED"
        self.assertEqual(self.derive(c)["currentness_state"], "NOT_ASSESSED")

    def test_blocked_g1_not_assessed(self):
        c=load_case(); c["g1_receipt"]["authorization_state"]="BLOCKED"; c["g1_receipt"]["blockers"]=["X"]
        self.assertEqual(self.derive(c)["currentness_state"], "NOT_ASSESSED")

    def test_old_profile_historical(self):
        c=load_case()
        old = copy.deepcopy(c["g1_receipt"])
        successor=copy.deepcopy(c["designated_g1_profile"])
        successor["actions"]["OPERATOR_RENEWAL"]["required_chambers"]["PUBLIC"]["approval_ppm"]=650000
        successor_sha=g2a.sha256_hex(successor)
        c["designated_g1_profile"]=successor
        c["designation"]["active_profile_sha256"]=successor_sha
        c["g1_receipt"]=old
        self.assertEqual(self.derive(c)["currentness_state"], "HISTORICAL")

    def test_explicit_revocation(self):
        c=load_case()
        target=g2a.sha256_hex(c["g1_receipt"])
        c["revocations"]=[{
            "project_id":c["currentness_profile"]["project_id"],
            "registry_id":c["currentness_profile"]["registry_id"],
            "epoch":c["designation"]["epoch"],
            "decision_receipt_sha256":target,
            "authority_ref":c["currentness_profile"]["revocation_authority_ref"],
            "reason_code":"SUPERSEDED_DECISION",
            "evidence_ref":"evidence:revoke-1",
        }]
        self.assertEqual(self.derive(c)["currentness_state"], "REVOKED")

    def test_wrong_designation_authority_rejected(self):
        c=load_case(); c["designation"]["authority_ref"]="authority:attacker"
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_wrong_project_rejected(self):
        c=load_case(); c["designation"]["project_id"]="other"
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_wrong_registry_rejected(self):
        c=load_case(); c["designation"]["registry_id"]="registry:other"
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_epoch_bounds_rejected(self):
        c=load_case(); c["designation"]["epoch"]=c["currentness_profile"]["max_epoch"]+1
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_designation_digest_mismatch_rejected(self):
        c=load_case(); c["designation"]["active_profile_sha256"]="0"*64
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_asset_lock_unprotected_rejected(self):
        c=load_case(); c["designated_g1_profile"]["actions"]["ASSET_LOCK_REMOVAL"]["prohibited"]=False
        c["designation"]["active_profile_sha256"]=g2a.sha256_hex(c["designated_g1_profile"])
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_steward_seat_sale_unprotected_rejected(self):
        c=load_case(); c["designated_g1_profile"]["actions"]["STEWARD_SEAT_SALE"]["prohibited"]=False
        c["designation"]["active_profile_sha256"]=g2a.sha256_hex(c["designated_g1_profile"])
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_capital_chamber_rejected(self):
        c=load_case()
        c["designated_g1_profile"]["chambers"].append("CAPITAL")
        c["currentness_profile"]["allowed_chambers"].append("CAPITAL")
        c["designation"]["active_profile_sha256"]=g2a.sha256_hex(c["designated_g1_profile"])
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_chamber_topology_change_rejected(self):
        c=load_case()
        c["designated_g1_profile"]["chambers"].remove("WORKERS")
        c["designation"]["active_profile_sha256"]=g2a.sha256_hex(c["designated_g1_profile"])
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_revocation_wrong_authority_rejected(self):
        c=load_case()
        c["revocations"]=[{
            "project_id":c["currentness_profile"]["project_id"],
            "registry_id":c["currentness_profile"]["registry_id"],
            "epoch":c["designation"]["epoch"],
            "decision_receipt_sha256":g2a.sha256_hex(c["g1_receipt"]),
            "authority_ref":"authority:attacker",
            "reason_code":"X",
            "evidence_ref":"evidence:x",
        }]
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_revocation_wrong_epoch_rejected(self):
        c=load_case()
        c["revocations"]=[{
            "project_id":c["currentness_profile"]["project_id"],
            "registry_id":c["currentness_profile"]["registry_id"],
            "epoch":c["designation"]["epoch"]-1,
            "decision_receipt_sha256":g2a.sha256_hex(c["g1_receipt"]),
            "authority_ref":c["currentness_profile"]["revocation_authority_ref"],
            "reason_code":"X",
            "evidence_ref":"evidence:x",
        }]
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_duplicate_revocation_rejected(self):
        c=load_case()
        r={
            "project_id":c["currentness_profile"]["project_id"],
            "registry_id":c["currentness_profile"]["registry_id"],
            "epoch":c["designation"]["epoch"],
            "decision_receipt_sha256":g2a.sha256_hex(c["g1_receipt"]),
            "authority_ref":c["currentness_profile"]["revocation_authority_ref"],
            "reason_code":"X",
            "evidence_ref":"evidence:x",
        }
        c["revocations"]=[r,copy.deepcopy(r)]
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_authority_contamination_rejected(self):
        c=load_case(); c["g1_receipt"]["legal_validity_established"]=True
        with self.assertRaises(g2a.CurrentnessError): self.derive(c)

    def test_deterministic_and_revocation_order_independent(self):
        c=load_case()
        r1={
            "project_id":c["currentness_profile"]["project_id"],
            "registry_id":c["currentness_profile"]["registry_id"],
            "epoch":c["designation"]["epoch"],
            "decision_receipt_sha256":"1"*64,
            "authority_ref":c["currentness_profile"]["revocation_authority_ref"],
            "reason_code":"OTHER",
            "evidence_ref":"evidence:other",
        }
        r2=copy.deepcopy(r1); r2["decision_receipt_sha256"]="2"*64; r2["evidence_ref"]="evidence:other-2"
        c["revocations"]=[r1,r2]
        a=self.derive(c)
        c["revocations"].reverse()
        b=self.derive(c)
        self.assertEqual(a,b)
        self.assertFalse(a["execution_authority_established"])
        self.assertFalse(a["legal_validity_established"])
        self.assertFalse(a["democratic_legitimacy_established"])

if __name__ == "__main__":
    unittest.main()
