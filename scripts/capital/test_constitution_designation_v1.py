#!/usr/bin/env python3
from __future__ import annotations
import copy
import json
import sys
import unittest
from pathlib import Path

HERE=Path(__file__).resolve()
sys.path.insert(0,str(HERE.parent))
import constitution_designation_v1 as g2c
FIXTURE=HERE.parent/'example_case.json'

def load_case():
    return json.loads(FIXTURE.read_text())

def sync_event(c):
    amendment_sha=g2c.sha256_hex(c['amendment_receipt'])
    successor_sha=g2c.sha256_hex(c['successor_g1_profile'])
    c['designation_event']['amendment_receipt_sha256']=amendment_sha
    c['designation_event']['successor_profile_sha256']=successor_sha
    c['designation_event']['event_id']=f"designation:{c['amendment_receipt']['successor_epoch']}:{amendment_sha[:16]}"

class DesignationTests(unittest.TestCase):
    def q(self,c): return g2c.qualify(c).receipt()
    def test_positive_active(self):
        r=self.q(load_case()); self.assertEqual(r['designation_state'],'ACTIVE'); self.assertTrue(r['prior_epoch_decisions_become_historical']); self.assertIsNotNone(r['new_checkpoint'])
    def test_blocked_amendment_blocks(self):
        c=load_case(); c['amendment_receipt']['transition_state']='BLOCKED'; c['amendment_receipt']['blockers']=['X']; sync_event(c)
        r=self.q(c); self.assertEqual(r['designation_state'],'BLOCKED'); self.assertIn('AMENDMENT_NOT_ACCEPTED',r['blockers'])
    def test_amendment_designation_claim_blocks(self):
        c=load_case(); c['amendment_receipt']['successor_designation_established']=True; sync_event(c)
        r=self.q(c); self.assertEqual(r['designation_state'],'BLOCKED'); self.assertIn('AMENDMENT_ALREADY_CLAIMS_DESIGNATION',r['blockers'])
    def test_stale_checkpoint_digest(self):
        c=load_case(); c['designation_event']['expected_current_checkpoint_sha256']='0'*64
        r=self.q(c); self.assertEqual(r['designation_state'],'STALE'); self.assertEqual(r['blockers'],[]); self.assertIsNone(r['new_checkpoint'])
    def test_stale_chain_tip(self):
        c=load_case(); c['designation_event']['expected_current_chain_tip_sha256']='2'*64
        self.assertEqual(self.q(c)['designation_state'],'STALE')
    def test_stale_live_epoch_after_competing_cutover(self):
        c=load_case(); c['live_checkpoint']['active_epoch']=8; c['live_checkpoint']['active_profile_sha256']=c['amendment_receipt']['successor_profile_sha256']; c['live_checkpoint']['chain_tip_sha256']='3'*64
        self.assertEqual(self.q(c)['designation_state'],'STALE')
    def test_replay_winning_event_after_cutover_is_stale(self):
        c=load_case(); first=self.q(c); new=first['new_checkpoint']; c['live_checkpoint']=new
        self.assertEqual(self.q(c)['designation_state'],'STALE')
    def test_wrong_authority_rejected(self):
        c=load_case(); c['designation_event']['authority_ref']='authority:operator'
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_wrong_project_rejected(self):
        c=load_case(); c['designation_event']['project_id']='other'
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_wrong_registry_rejected(self):
        c=load_case(); c['designation_event']['registry_id']='other'
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_amendment_receipt_substitution_rejected(self):
        c=load_case(); c['designation_event']['amendment_receipt_sha256']='0'*64
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_successor_profile_substitution_rejected(self):
        c=load_case(); c['designation_event']['successor_profile_sha256']='0'*64
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_successor_bytes_must_match_amendment(self):
        c=load_case(); c['successor_g1_profile']['actions']['OPERATOR_RENEWAL']['required_chambers']['PUBLIC']['approval_ppm']=700000; sync_event(c)
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_asset_lock_weakening_rejected(self):
        c=load_case(); c['successor_g1_profile']['actions']['ASSET_LOCK_REMOVAL']['prohibited']=False; sync_event(c)
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_steward_sale_weakening_rejected(self):
        c=load_case(); c['successor_g1_profile']['actions']['STEWARD_SEAT_SALE']['prohibited']=False; sync_event(c)
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_capital_chamber_rejected(self):
        c=load_case(); c['successor_g1_profile']['chambers'].append('CAPITAL'); sync_event(c)
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_protected_enforcer_weakening_rejected(self):
        c=load_case(); c['successor_g1_profile']['actions']['ASSET_LOCK_REMOVAL']['enforcer_required']=False; sync_event(c)
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_epoch_skip_blocks(self):
        c=load_case(); c['amendment_receipt']['successor_epoch']=9; sync_event(c)
        r=self.q(c); self.assertEqual(r['designation_state'],'BLOCKED'); self.assertIn('AMENDMENT_EPOCH_NOT_CONTIGUOUS',r['blockers'])
    def test_current_checkpoint_must_be_active(self):
        c=load_case(); c['live_checkpoint']['state']='PENDING'
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_deterministic_event_id_rejected(self):
        c=load_case(); c['designation_event']['event_id']='designation:fake'
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_authority_contamination_rejected(self):
        c=load_case(); c['amendment_receipt']['legal_validity_established']=True; sync_event(c)
        with self.assertRaises(g2c.DesignationError): self.q(c)
    def test_active_has_strong_nonclaims(self):
        r=self.q(load_case()); self.assertFalse(r['execution_authority_established']); self.assertFalse(r['legal_validity_established']); self.assertFalse(r['democratic_legitimacy_established'])
    def test_stale_is_not_semantic_invalidity(self):
        c=load_case(); c['designation_event']['expected_current_checkpoint_sha256']='f'*64; r=self.q(c)
        self.assertEqual(r['designation_state'],'STALE'); self.assertIn('STALE means compare-and-swap currentness conflict, not semantic invalidity of the amendment',r['nonclaims'])
    def test_deterministic(self):
        c=load_case(); self.assertEqual(self.q(c),self.q(copy.deepcopy(c)))

if __name__=='__main__': unittest.main()
