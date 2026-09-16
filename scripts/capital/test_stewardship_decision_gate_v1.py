#!/usr/bin/env python3
from __future__ import annotations
import copy, json, sys, unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stewardship_decision_gate_v1 as g

CASE = json.loads((HERE / "example_case.json").read_text(encoding="utf-8"))

class Tests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self, c): return g.qualify(c["profile"], c["decision"]).receipt()
    def rebind(self, c): c["decision"]["profile_sha256"] = g.sha256_hex(c["profile"])

    def test_ordinary_authorized(self):
        r=self.q(self.case()); self.assertEqual(r["authorization_state"],"AUTHORIZED"); self.assertFalse(r["asset_lock_removed"])
    def test_constitutional_relabel_rejected(self):
        c=self.case(); c["decision"]["action_code"]="CONSTITUENCY_AMENDMENT"; c["decision"]["declared_class"]="ORDINARY"
        with self.assertRaisesRegex(g.GovernanceError,"class substitution"): self.q(c)
    def test_asset_lock_removal_always_blocked(self):
        c=self.case(); c["decision"]=constitutional_decision(c,"ASSET_LOCK_REMOVAL")
        r=self.q(c); self.assertEqual(r["authorization_state"],"BLOCKED"); self.assertIn("PROHIBITED_ACTION:ASSET_LOCK_REMOVAL",r["blockers"])
    def test_steward_seat_sale_always_blocked(self):
        c=self.case(); c["decision"]=constitutional_decision(c,"STEWARD_SEAT_SALE")
        r=self.q(c); self.assertIn("PROHIBITED_ACTION:STEWARD_SEAT_SALE",r["blockers"])
    def test_required_recusal_failure_blocks(self):
        c=self.case(); c["decision"]["conflicts"][0]["recused"]=False
        r=self.q(c); self.assertTrue(any(x.startswith("RECUSAL_FAIL:") for x in r["blockers"]))
    def test_recused_counted_vote_rejected(self):
        c=self.case(); c["decision"]["conflicts"][0]["counted_vote"]=True
        with self.assertRaisesRegex(g.GovernanceError,"recused vote counted"): self.q(c)
    def test_quorum_fail_blocks(self):
        c=self.case(); c["decision"]["chamber_votes"]["USERS"]["participating_count"]=49; c["decision"]["chamber_votes"]["USERS"]["approval_count"]=49
        r=self.q(c); self.assertIn("QUORUM_FAIL:USERS",r["blockers"])
    def test_approval_fail_blocks(self):
        c=self.case(); c["decision"]["chamber_votes"]["USERS"]["approval_count"]=30
        r=self.q(c); self.assertIn("APPROVAL_FAIL:USERS",r["blockers"])
    def test_missing_required_chamber_blocks(self):
        c=self.case(); del c["decision"]["chamber_votes"]["PUBLIC"]
        r=self.q(c); self.assertIn("MISSING_CHAMBER:PUBLIC",r["blockers"])
    def test_extra_nonrequired_chamber_rejected(self):
        c=self.case(); c["decision"]["chamber_votes"]["WORKERS"]={"eligible_count":10,"participating_count":10,"approval_count":10}
        with self.assertRaisesRegex(g.GovernanceError,"non-required chamber injected"): self.q(c)
    def test_constitutional_requires_enforcer(self):
        c=self.case(); c["decision"]=constitutional_decision(c,"CONSTITUENCY_AMENDMENT"); c["decision"]["enforcer"]=None
        r=self.q(c); self.assertIn("ENFORCER_MISSING",r["blockers"])
    def test_constitutional_enforcer_no_concurrence(self):
        c=self.case(); c["decision"]=constitutional_decision(c,"CONSTITUENCY_AMENDMENT"); c["decision"]["enforcer"]["concurred"]=False
        r=self.q(c); self.assertIn("ENFORCER_DID_NOT_CONCUR",r["blockers"])
    def test_emergency_authorized(self):
        c=self.case(); c["decision"]=emergency_decision(c)
        self.assertEqual(self.q(c)["authorization_state"],"AUTHORIZED")
    def test_emergency_pending_blocks(self):
        c=self.case(); c["decision"]=emergency_decision(c); c["decision"]["emergency"]["state"]="PENDING"
        self.assertIn("EMERGENCY_NOT_ACTIVE:PENDING",self.q(c)["blockers"])
    def test_emergency_scope_miss_blocks(self):
        c=self.case(); c["decision"]=emergency_decision(c); c["decision"]["emergency"]["scope_actions"]=["OTHER_ACTION"]
        self.assertIn("EMERGENCY_SCOPE_MISS",self.q(c)["blockers"])
    def test_unknown_action_rejected(self):
        c=self.case(); c["decision"]["action_code"]="SELL_THE_COMMONS"
        with self.assertRaisesRegex(g.GovernanceError,"unknown action"): self.q(c)
    def test_project_substitution_rejected(self):
        c=self.case(); c["decision"]["project_id"]="fiber:other"
        with self.assertRaisesRegex(g.GovernanceError,"project substitution"): self.q(c)
    def test_profile_substitution_rejected(self):
        c=self.case(); c["decision"]["profile_sha256"]="0"*64
        with self.assertRaisesRegex(g.GovernanceError,"profile substitution"): self.q(c)
    def test_float_vote_count_rejected(self):
        c=self.case(); c["decision"]["chamber_votes"]["USERS"]["approval_count"]=50.0
        with self.assertRaisesRegex(g.GovernanceError,"invalid USERS.approval_count"): self.q(c)
    def test_negative_vote_count_rejected(self):
        c=self.case(); c["decision"]["chamber_votes"]["USERS"]["approval_count"]=-1
        with self.assertRaisesRegex(g.GovernanceError,"invalid USERS.approval_count"): self.q(c)
    def test_duplicate_conflict_identity_rejected(self):
        c=self.case(); c["decision"]["conflicts"].append(copy.deepcopy(c["decision"]["conflicts"][0]))
        with self.assertRaisesRegex(g.GovernanceError,"duplicate conflict identity"): self.q(c)
    def test_legal_authority_injection_rejected(self):
        c=self.case(); c["decision"]["legal_validity_established"]=True
        with self.assertRaisesRegex(g.GovernanceError,"keys mismatch"): self.q(c)
    def test_deterministic(self):
        a=self.q(self.case()); b=self.q(self.case()); self.assertEqual(g.canonical_bytes(a),g.canonical_bytes(b))

def constitutional_decision(c, action):
    p=c["profile"]; rule=p["actions"][action]
    votes={k:{"eligible_count":100,"participating_count":90,"approval_count":85} for k in rule["required_chambers"]}
    return {
        "decision_id":"decision-constitutional-001","project_id":p["project_id"],"profile_sha256":g.sha256_hex(p),
        "action_code":action,"declared_class":"CONSTITUTIONAL","chamber_votes":votes,"conflicts":[],
        "enforcer":{"authority_ref":p["enforcer_ref"],"concurred":True,"evidence_ref":"evidence:enforcer-001"},
        "emergency":None,"evidence_ref":"evidence:decision-constitutional-001",
    }

def emergency_decision(c):
    p=c["profile"]; action="EMERGENCY_SERVICE_REROUTE"; rule=p["actions"][action]
    votes={k:{"eligible_count":100,"participating_count":80,"approval_count":75} for k in rule["required_chambers"]}
    return {
        "decision_id":"decision-emergency-001","project_id":p["project_id"],"profile_sha256":g.sha256_hex(p),
        "action_code":action,"declared_class":"EMERGENCY","chamber_votes":votes,"conflicts":[],"enforcer":None,
        "emergency":{"authority_ref":p["emergency_authority_ref"],"state":"ACTIVE","scope_actions":[action],"evidence_ref":"evidence:emergency-001"},
        "evidence_ref":"evidence:decision-emergency-001",
    }

if __name__=="__main__": unittest.main()
