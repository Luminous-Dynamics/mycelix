#!/usr/bin/env python3
"""Tests for NCF-ALLOC-4 identity splitting research harness."""
import unittest
import ncf_alloc4_identity_splitting as ncf

class IdentitySplittingTests(unittest.TestCase):
    def test_attacker_true_backlog_is_constant_when_ids_multiply(self):
        for count in ncf.ATTACKER_IDENTITY_COUNTS:
            case=ncf.make_case(count)
            self.assertEqual(sum(case["actual_by_id"][sid] for sid in case["attacker_ids"]), ncf.ATTACKER_TOTAL_DEMAND)
    def test_adding_attacker_ids_increases_equal_floor_capture(self):
        one=ncf.one_case(1)["policies"]["equal_floor50_raw_claims"]["attacker_allocation_share_ppm"]
        many=ncf.one_case(15)["policies"]["equal_floor50_raw_claims"]["attacker_allocation_share_ppm"]
        self.assertGreater(many,one)
    def test_median_cap_ceases_to_clip_after_identity_splitting(self):
        case=ncf.one_case(16)["policies"]
        raw=case["equal_floor50_raw_claims"]["attacker_allocation_share_ppm"]
        cap2=case["equal_floor50_cap2x_median"]["attacker_allocation_share_ppm"]
        self.assertEqual(raw,cap2)
    def test_cap_does_not_claim_to_prove_backlog(self):
        result=ncf.run_sweep()
        self.assertIn("does not model cryptographic identities",result["claim_ceiling"])
    def test_capacity_is_conserved_for_all_policies(self):
        for count in ncf.ATTACKER_IDENTITY_COUNTS:
            case=ncf.one_case(count)
            self.assertEqual(case["total_shard_ids"], count+ncf.HONEST_SHARDS)
            for policy in case["policies"].values():
                self.assertGreaterEqual(policy["attacker_allocated_units"], policy["attacker_served_units"])
                self.assertLessEqual(policy["attacker_allocation_share_ppm"], 1_000_000)

if __name__ == "__main__": unittest.main(verbosity=2)
