from __future__ import annotations
import copy, json, sys, unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import public_service_distribution_composition_v1 as h2

def case():
    root=HERE.parents[1]
    return json.loads((root/'docs/capital/evidence/myc-cap-002h2/example_case.json').read_text())

class Tests(unittest.TestCase):
    def q(self,c): return h2.compose(c['service_gate_receipt'],c['service_currentness_receipt'])
    def sync(self,c):
        f=c['service_gate_receipt']; f1=c['service_currentness_receipt']
        f1['service_receipt_sha256']=h2.sha256_hex(f)
        f1['service_gate_profile_sha256']=f['profile_sha256']
        f1['measurement_id']=f['measurement_id']
        f1['service_distribution_eligibility']=f['distribution_eligibility']
    def test_positive(self): self.assertEqual(self.q(case())['public_service_distribution_state'],'ELIGIBLE_CURRENT')
    def test_stale(self):
        c=case(); c['service_currentness_receipt']['currentness_state']='STALE'; self.assertEqual(self.q(c)['public_service_distribution_state'],'STALE')
    def test_pending(self):
        c=case(); c['service_currentness_receipt']['currentness_state']='PENDING'; self.assertEqual(self.q(c)['public_service_distribution_state'],'PENDING')
    def test_revoked(self):
        c=case(); c['service_currentness_receipt']['currentness_state']='REVOKED'; self.assertEqual(self.q(c)['public_service_distribution_state'],'REVOKED')
    def test_blocked_service(self):
        c=case(); c['service_gate_receipt']['distribution_eligibility']='BLOCKED'; c['service_gate_receipt']['blockers']=['UPTIME_BELOW_MINIMUM']; self.sync(c)
        r=self.q(c); self.assertEqual(r['public_service_distribution_state'],'BLOCKED_SERVICE'); self.assertEqual(r['blockers'],['UPTIME_BELOW_MINIMUM'])
    def test_no_active_claim(self):
        c=case(); c['service_gate_receipt']['distribution_eligibility']='NO_ACTIVE_CLAIM'; self.sync(c); self.assertEqual(self.q(c)['public_service_distribution_state'],'NO_ACTIVE_CLAIM')
    def test_superseded_f_subject_rejected(self):
        c=case(); c['service_currentness_receipt']['service_gate_subject_sha']='45361d938d9a56f66e6eab360b04d773f54b1c8c'
        with self.assertRaisesRegex(h2.CompositionError,'wrong qualified F subject'): self.q(c)
    def test_project_mismatch(self):
        c=case(); c['service_currentness_receipt']['project_id']='other'
        with self.assertRaisesRegex(h2.CompositionError,'project mismatch'): self.q(c)
    def test_measurement_mismatch(self):
        c=case(); c['service_currentness_receipt']['measurement_id']='other'
        with self.assertRaisesRegex(h2.CompositionError,'measurement mismatch'): self.q(c)
    def test_profile_mismatch(self):
        c=case(); c['service_currentness_receipt']['service_gate_profile_sha256']='0'*64
        with self.assertRaisesRegex(h2.CompositionError,'service profile mismatch'): self.q(c)
    def test_receipt_digest_mismatch(self):
        c=case(); c['service_currentness_receipt']['service_receipt_sha256']='0'*64
        with self.assertRaisesRegex(h2.CompositionError,'service receipt digest mismatch'): self.q(c)
    def test_eligibility_echo_mismatch(self):
        c=case(); c['service_currentness_receipt']['service_distribution_eligibility']='BLOCKED'
        with self.assertRaisesRegex(h2.CompositionError,'eligibility echo mismatch'): self.q(c)
    def test_f_claim_mutation_rejected(self):
        c=case(); c['service_gate_receipt']['claim_modified']=True; self.sync(c)
        with self.assertRaisesRegex(h2.CompositionError,'claim_modified'): self.q(c)
    def test_f1_claim_mutation_rejected(self):
        c=case(); c['service_currentness_receipt']['claim_modified']=True
        with self.assertRaisesRegex(h2.CompositionError,'claim_modified'): self.q(c)
    def test_wall_clock_rejected(self):
        c=case(); c['service_currentness_receipt']['uses_local_wall_clock']=True
        with self.assertRaisesRegex(h2.CompositionError,'wall clock'): self.q(c)
    def test_raw_snapshot_rejected(self):
        c=case(); c['service_gate_receipt']=c['service_gate_receipt']['observed']
        with self.assertRaises(h2.CompositionError): self.q(c)
    def test_raw_designation_rejected(self):
        c=case(); c['service_currentness_receipt']={'currentness_state':'CURRENT'}
        with self.assertRaises(h2.CompositionError): self.q(c)
    def test_unknown_f_field_rejected(self):
        c=case(); c['service_gate_receipt']['payment_authority']=True
        with self.assertRaisesRegex(h2.CompositionError,'unknown=.*payment_authority'): self.q(c)
    def test_unknown_f1_field_rejected(self):
        c=case(); c['service_currentness_receipt']['execution_authority']=True
        with self.assertRaisesRegex(h2.CompositionError,'unknown=.*execution_authority'): self.q(c)
    def test_output_nonclaims(self):
        r=self.q(case()); self.assertFalse(r['claim_modified']); self.assertFalse(r['execution_authority_established']); self.assertFalse(r['payment_authority_established']); self.assertFalse(r['legal_distribution_authority_established'])
    def test_blocker_order_deterministic(self):
        c=case(); c['service_gate_receipt']['distribution_eligibility']='BLOCKED'; c['service_gate_receipt']['blockers']=['Z','A','Z']; self.sync(c); self.assertEqual(self.q(c)['blockers'],['A','Z'])
    def test_deterministic(self):
        c=case(); self.assertEqual(self.q(c),self.q(copy.deepcopy(c)))

if __name__=='__main__': unittest.main()
