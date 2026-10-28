#!/usr/bin/env python3
"""Tests for NCF-ALLOC-3 bounded-influence allocation research harness."""
import random, unittest
import ncf_alloc3 as ncf

class RobustAllocationTests(unittest.TestCase):
    def test_capacity_conserved_for_policies_and_floors(self):
        claims = [((i*1237)%18000)+1 for i in range(ncf.SHARDS)]
        for floor in (0, 250_000, 500_000, 750_000, 1_000_000):
            for cap in (None, 1, 2, 4):
                plan=ncf.allocate(claims, floor_ppm=floor, cap_multiple=cap)
                self.assertEqual(sum(plan.values()), ncf.SPENDABLE); self.assertTrue(all(v>=0 for v in plan.values()))
    def test_stable_id_order_independence(self):
        ids=[f"shard-{i:02d}" for i in range(ncf.SHARDS)]; claims=[i*1009%7001 for i in range(ncf.SHARDS)]
        expected=ncf.allocate(claims, ids, cap_multiple=2); paired=list(zip(ids,claims)); random.Random(22).shuffle(paired)
        self.assertEqual(expected,ncf.allocate([v for _,v in paired],[k for k,_ in paired],cap_multiple=2))
    def test_duplicate_ids_reject(self):
        with self.assertRaises(ValueError): ncf.allocate([1]*ncf.SHARDS,["x"]*ncf.SHARDS)
    def test_negative_values_and_bad_caps_reject(self):
        with self.assertRaises(ValueError): ncf.allocate([-1]+[1]*15)
        with self.assertRaises(ValueError): ncf.capped_claims([1,2],0)
    def test_capping_reduces_two_shard_influence(self):
        actual=[10_000]*ncf.SHARDS; claims=actual[:]; claims[0]=claims[1]=100_000
        ids=[f"shard-{i:02d}" for i in range(ncf.SHARDS)]
        raw=ncf.allocate(claims,ids,floor_ppm=500_000); capped=ncf.allocate(claims,ids,floor_ppm=500_000,cap_multiple=2)
        self.assertLess(capped[ids[0]]+capped[ids[1]],raw[ids[0]]+raw[ids[1]])
        self.assertLess(sum(min(raw[ids[i]],actual[i]) for i in range(ncf.SHARDS)),
                        sum(min(capped[ids[i]],actual[i]) for i in range(ncf.SHARDS)))
    def test_cap_penalty_on_legitimate_4x_hotspot_is_visible(self):
        actual=ncf.hot_demand(0); ids=[f"shard-{i:02d}" for i in range(ncf.SHARDS)]
        raw=ncf.metrics(ncf.allocate(actual,ids,floor_ppm=500_000),actual,ids)
        cap2=ncf.metrics(ncf.allocate(actual,ids,floor_ppm=500_000,cap_multiple=2),actual,ids)
        cap4=ncf.metrics(ncf.allocate(actual,ids,floor_ppm=500_000,cap_multiple=4),actual,ids)
        self.assertLess(cap2["served_units"],raw["served_units"]); self.assertEqual(cap4["served_units"],raw["served_units"])
    def test_all_zero_claims_fall_back_to_equal(self):
        a=ncf.allocate([0]*ncf.SHARDS,floor_ppm=500_000,cap_multiple=2)
        self.assertLessEqual(max(a.values())-min(a.values()),1)
    def test_randomized_10000_cases_conserve_and_reorder(self):
        rng=random.Random(0xA110C3); ids=[f"shard-{i:02d}" for i in range(ncf.SHARDS)]
        for _ in range(10_000):
            claims=[rng.randrange(0,100_000) for _ in ids]; cap=rng.choice([None,1,2,4]); floor=rng.choice([0,250_000,500_000,750_000,1_000_000])
            expected=ncf.allocate(claims,ids,floor_ppm=floor,cap_multiple=cap); pairs=list(zip(ids,claims)); rng.shuffle(pairs)
            got=ncf.allocate([v for _,v in pairs],[k for k,_ in pairs],floor_ppm=floor,cap_multiple=cap)
            self.assertEqual(expected,got); self.assertEqual(sum(got.values()),ncf.SPENDABLE)
    def test_sweep_reports_model_claim_ceiling(self):
        r=ncf.attack_sweep(100,4); self.assertEqual(r["cases"],100); self.assertIn("Sybil",r["median_cap_assumption"])
        self.assertEqual(len(r["policies"]),5)

if __name__=="__main__": unittest.main(verbosity=2)
