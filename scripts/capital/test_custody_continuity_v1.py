#!/usr/bin/env python3
from __future__ import annotations
import copy, json, unittest
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import custody_continuity_v1 as c

REPO_ROOT=HERE.parents[1]
CASE=json.loads((REPO_ROOT/"docs/capital/evidence/myc-cap-002d3/example_case.json").read_text(encoding="utf-8"))

class CustodyTests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,x): return c.qualify(x["profile"],x["acceptance_receipt"],x["events"]).receipt()

    def append(self,x,kind,*,blocker=None,authority=None,successor=None,successor_receipt=None,evidence=None):
        prior=x["events"][-1]
        mapping={
            "MaterialRegression":"regression_authority_ref",
            "VerifiedCure":"cure_authority_ref",
            "SuspendCustody":"custody_authority_ref",
            "ResumeCustody":"custody_authority_ref",
            "SuccessorAcceptance":"successor_authority_ref",
            "TerminateCustody":"custody_authority_ref",
        }
        i=len(x["events"])
        e={
            "seq":i,
            "event_id":f"custody-evt-{i}",
            "project_id":x["profile"]["project_id"],
            "profile_sha256":c.sha256_hex(x["profile"]),
            "prev_event_sha256":c.sha256_hex(prior),
            "kind":kind,
            "blocker_code":blocker,
            "successor_custodian_ref":successor,
            "successor_acceptance_receipt_sha256":successor_receipt,
            "authority_ref":authority or x["profile"][mapping[kind]],
            "evidence_ref":evidence or f"evidence:custody-{i}",
        }
        x["events"].append(e)
        return e

    def test_genesis_current(self):
        r=self.q(self.case())
        self.assertEqual(r["custody_state"],"CURRENT")
        self.assertEqual(r["current_custodian_ref"],"steward:community-trust")
        self.assertTrue(r["historical_handover_accepted"])

    def test_regression_degrades(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="TRUST_ROOT_ROTATION_LOST")
        r=self.q(x)
        self.assertEqual(r["custody_state"],"DEGRADED")
        self.assertEqual(r["active_blockers"],["TRUST_ROOT_ROTATION_LOST"])

    def test_verified_cure_restores_current(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="BACKUP_RESTORE_FAILED")
        self.append(x,"VerifiedCure",blocker="BACKUP_RESTORE_FAILED")
        self.assertEqual(self.q(x)["custody_state"],"CURRENT")

    def test_cure_nonactive_rejected(self):
        x=self.case()
        self.append(x,"VerifiedCure",blocker="BACKUP_RESTORE_FAILED")
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_duplicate_regression_rejected(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="MATERIAL_CYBER_INCIDENT")
        self.append(x,"MaterialRegression",blocker="MATERIAL_CYBER_INCIDENT")
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_unknown_blocker_rejected(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="ATTACKER_CODE")
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_suspend_preserves_blockers(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="ADMIN_RECOVERY_LOST")
        self.append(x,"SuspendCustody")
        r=self.q(x)
        self.assertEqual(r["custody_state"],"SUSPENDED")
        self.assertEqual(r["active_blockers"],["ADMIN_RECOVERY_LOST"])

    def test_resume_with_blocker_rejected(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="ADMIN_RECOVERY_LOST")
        self.append(x,"SuspendCustody")
        self.append(x,"ResumeCustody")
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_cure_then_resume(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="ADMIN_RECOVERY_LOST")
        self.append(x,"SuspendCustody")
        self.append(x,"VerifiedCure",blocker="ADMIN_RECOVERY_LOST")
        self.append(x,"ResumeCustody")
        self.assertEqual(self.q(x)["custody_state"],"CURRENT")

    def test_successor_supersedes(self):
        x=self.case()
        self.append(x,"SuccessorAcceptance",successor="steward:successor-trust",successor_receipt="a"*64)
        r=self.q(x)
        self.assertEqual(r["custody_state"],"SUPERSEDED")
        self.assertEqual(r["successor_custodian_ref"],"steward:successor-trust")
        self.assertEqual(r["current_custodian_ref"],"steward:successor-trust")

    def test_successor_with_blocker_rejected(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="CONTINUITY_CAPABILITY_LOST")
        self.append(x,"SuccessorAcceptance",successor="steward:successor-trust",successor_receipt="a"*64)
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_successor_same_custodian_rejected(self):
        x=self.case()
        self.append(x,"SuccessorAcceptance",successor=x["profile"]["initial_custodian_ref"],successor_receipt="a"*64)
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_terminal_lineage_cannot_continue(self):
        x=self.case()
        self.append(x,"SuccessorAcceptance",successor="steward:successor-trust",successor_receipt="a"*64)
        self.append(x,"MaterialRegression",blocker="MATERIAL_CYBER_INCIDENT")
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_terminate_terminal(self):
        x=self.case()
        self.append(x,"TerminateCustody")
        r=self.q(x)
        self.assertEqual(r["custody_state"],"TERMINATED")
        self.assertIsNone(r["current_custodian_ref"])

    def test_wrong_authority_rejected(self):
        x=self.case()
        self.append(x,"MaterialRegression",blocker="BACKUP_RESTORE_FAILED",authority="authority:operator")
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_project_substitution_rejected(self):
        x=self.case()
        self.append(x,"SuspendCustody")
        x["events"][-1]["project_id"]="fiber:other"
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_profile_substitution_rejected(self):
        x=self.case()
        self.append(x,"SuspendCustody")
        x["events"][-1]["profile_sha256"]="f"*64
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_chain_mutation_rejected(self):
        x=self.case()
        self.append(x,"SuspendCustody")
        x["events"][0]["evidence_ref"]="evidence:mutated"
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_duplicate_event_id_rejected(self):
        x=self.case()
        e=self.append(x,"SuspendCustody")
        e["event_id"]=x["events"][0]["event_id"]
        with self.assertRaises(c.CustodyError): self.q(x)


    def test_acceptance_subject_invalid_rejected(self):
        x=self.case(); x["profile"]["acceptance_subject_sha"]="not-a-sha"
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_acceptance_receipt_mutation_rejected(self):
        x=self.case()
        x["acceptance_receipt"]["incoming_custodian_ref"]="steward:attacker"
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_authority_contaminated_acceptance_rejected(self):
        x=self.case()
        x["acceptance_receipt"]["legal_title_transition_established"]=True
        x["profile"]["acceptance_receipt_sha256"]=c.sha256_hex(x["acceptance_receipt"])
        for e in x["events"]:
            e["profile_sha256"]=c.sha256_hex(x["profile"])
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_injected_legal_field_in_event_rejected(self):
        x=self.case()
        e=self.append(x,"SuspendCustody")
        e["legal_title_transition_established"]=True
        with self.assertRaises(c.CustodyError): self.q(x)

    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case())
        self.assertEqual(c.canonical_bytes(a),c.canonical_bytes(b))

if __name__=="__main__":
    unittest.main()
