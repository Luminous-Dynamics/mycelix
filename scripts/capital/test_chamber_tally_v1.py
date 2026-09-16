#!/usr/bin/env python3
from __future__ import annotations
import copy,json,sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE))
import chamber_tally_v1 as t
CASE=json.loads((ROOT/'docs/capital/evidence/myc-cap-002g1a/example_case.json').read_text())

class Tests(unittest.TestCase):
    def case(self): return copy.deepcopy(CASE)
    def q(self,c): return t.qualify(c).receipt()
    def test_positive_reconstruction(self):
        r=self.q(self.case()); self.assertEqual(r['chamber_threshold_state'],'PASS'); self.assertEqual(r['chamber_results']['USERS']['eligible_count'],4); self.assertEqual(r['chamber_results']['USERS']['approval_count'],2)
    def test_required_recusal_removed_from_denominator(self):
        r=self.q(self.case()); self.assertEqual(r['chamber_results']['USERS']['recused_count'],1); self.assertEqual(r['chamber_results']['USERS']['eligible_count'],4)
    def test_recused_ballot_rejected(self):
        c=self.case(); b=copy.deepcopy(c['ballots'][0]); b['ballot_id']='b-u5'; b['participant_ref']='member:u5'; c['ballots'].append(b)
        with self.assertRaisesRegex(t.TallyError,'recused participant'): self.q(c)
    def test_duplicate_participant_vote_rejected(self):
        c=self.case(); b=copy.deepcopy(c['ballots'][0]); b['ballot_id']='b-u1-second'; c['ballots'].append(b)
        with self.assertRaisesRegex(t.TallyError,'more than once'): self.q(c)
    def test_duplicate_ballot_id_rejected(self):
        c=self.case(); b=copy.deepcopy(c['ballots'][1]); b['ballot_id']=c['ballots'][0]['ballot_id']; c['ballots'].append(b)
        with self.assertRaisesRegex(t.TallyError,'duplicate ballot id'): self.q(c)
    def test_ineligible_ballot_rejected(self):
        c=self.case(); c['ballots'][0]['participant_ref']='member:outsider'
        with self.assertRaisesRegex(t.TallyError,'ineligible ballot'): self.q(c)
    def test_cross_chamber_ballot_rejected(self):
        c=self.case(); c['ballots'][0]['chamber']='PUBLIC'
        with self.assertRaisesRegex(t.TallyError,'cross-chamber ballot'): self.q(c)
    def test_stale_snapshot_rejected(self):
        c=self.case(); c['eligibility_snapshot']['registry_epoch']=6
        with self.assertRaisesRegex(t.TallyError,'stale eligibility epoch'): self.q(c)
    def test_stale_ballot_registry_rejected(self):
        c=self.case(); c['ballots'][0]['registry_epoch']=6
        with self.assertRaisesRegex(t.TallyError,'registry currentness'): self.q(c)
    def test_same_participant_multiple_chambers_rejected(self):
        c=self.case(); c['eligibility_snapshot']['chambers']['PUBLIC'].append('member:u1')
        with self.assertRaisesRegex(t.TallyError,'multiple chambers'): self.q(c)
    def test_duplicate_member_in_chamber_rejected(self):
        c=self.case(); c['eligibility_snapshot']['chambers']['USERS'].append('member:u1')
        with self.assertRaisesRegex(t.TallyError,'duplicate participant'): self.q(c)
    def test_conflict_wrong_chamber_rejected(self):
        c=self.case(); c['conflicts'][0]['chamber']='PUBLIC'
        with self.assertRaisesRegex(t.TallyError,'conflict chamber substitution'): self.q(c)
    def test_required_conflict_undisclosed_rejected(self):
        c=self.case(); c['conflicts'][0]['disclosed']=False
        with self.assertRaisesRegex(t.TallyError,'required conflict not disclosed'): self.q(c)
    def test_duplicate_conflict_rejected(self):
        c=self.case(); c['conflicts'].append(copy.deepcopy(c['conflicts'][0]))
        with self.assertRaisesRegex(t.TallyError,'duplicate conflict identity'): self.q(c)
    def test_quorum_failure_from_reconstructed_ballots(self):
        c=self.case(); c['ballots']=[b for b in c['ballots'] if not (b['chamber']=='USERS' and b['participant_ref'] in {'member:u2','member:u3'})]
        r=self.q(c); self.assertIn('QUORUM_FAIL:USERS',r['blockers'])
    def test_approval_failure_from_reconstructed_ballots(self):
        c=self.case();
        for b in c['ballots']:
            if b['chamber']=='USERS' and b['participant_ref']=='member:u2': b['vote']='REJECT'
        r=self.q(c); self.assertIn('APPROVAL_FAIL:USERS',r['blockers'])
    def test_unknown_vote_rejected(self):
        c=self.case(); c['ballots'][0]['vote']='WEIGHTED_APPROVE'
        with self.assertRaisesRegex(t.TallyError,'invalid vote value'): self.q(c)
    def test_profile_substitution_rejected(self):
        c=self.case(); c['tally_profile']['governance_profile_sha256']='0'*64
        with self.assertRaisesRegex(t.TallyError,'governance profile substitution'): self.q(c)
    def test_ballot_wrong_decision_rejected(self):
        c=self.case(); c['ballots'][0]['decision_id']='decision:other'
        with self.assertRaisesRegex(t.TallyError,'ballot decision substitution'): self.q(c)
    def test_snapshot_wrong_project_rejected(self):
        c=self.case(); c['eligibility_snapshot']['project_id']='fiber:other'
        with self.assertRaisesRegex(t.TallyError,'snapshot project substitution'): self.q(c)
    def test_authority_injection_rejected(self):
        c=self.case(); c['ballots'][0]['weight']=2
        with self.assertRaisesRegex(t.TallyError,'keys mismatch'): self.q(c)
    def test_deterministic_and_order_independent(self):
        c1=self.case(); c2=self.case(); c2['ballots']=list(reversed(c2['ballots'])); c2['conflicts']=list(reversed(c2['conflicts'])); c2['eligibility_snapshot']['chambers']['USERS']=list(reversed(c2['eligibility_snapshot']['chambers']['USERS']))
        self.assertEqual(t.canonical_bytes(self.q(c1)),t.canonical_bytes(self.q(c2)))

if __name__=='__main__': unittest.main()
