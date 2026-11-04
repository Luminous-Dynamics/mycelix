#!/usr/bin/env python3
"""Tests for NCF-ALLOC-6 interleaving model."""
import unittest, hashlib
from ncf_alloc6_admission_races import Request, interleavings, run_naive, run_serializable, scenario

class AdmissionRaceTests(unittest.TestCase):
    def test_enumerates_all_70_order_preserving_interleavings(self):
        schedules=interleavings()
        self.assertEqual(len(schedules),70)
        self.assertTrue(all(s.count("A")==4 and s.count("B")==4 for s in schedules))
    def test_naive_same_work_check_then_commit_can_accept_duplicate_work(self):
        digest=hashlib.sha256(b"same-work").hexdigest()
        reqs={"A":Request("req-A",digest,amount=60),"B":Request("req-B",digest,amount=60)}
        rows=[run_naive(reqs,s) for s in interleavings()]
        duplicate=[r for r in rows if r["duplicate_work_accepted"]]
        self.assertGreater(len(duplicate),0)
        self.assertTrue(any(r["accepted_event_count"]==2 for r in duplicate))
    def test_naive_distinct_work_can_overshoot_cap(self):
        a=hashlib.sha256(b"a").hexdigest(); b=hashlib.sha256(b"b").hexdigest()
        reqs={"A":Request("A",a,amount=60),"B":Request("B",b,amount=60)}
        rows=[run_naive(reqs,s,"atomic_increment") for s in interleavings()]
        bad=[r for r in rows if r["cap_overshoot_units"]>0]
        self.assertGreater(len(bad),0)
        self.assertEqual(max(r["cap_overshoot_units"] for r in bad),20)
    def test_naive_same_request_id_can_accept_conflicting_payloads(self):
        a=hashlib.sha256(b"work-a").hexdigest(); b=hashlib.sha256(b"work-b").hexdigest()
        reqs={"A":Request("shared-id",a,amount=40),"B":Request("shared-id",b,amount=40)}
        rows=[run_naive(reqs,s,"atomic_increment") for s in interleavings()]
        conflicts=[r for r in rows if r["conflicting_request_payloads_accepted"]]
        self.assertGreater(len(conflicts),0)
        self.assertTrue(all(r["accepted_event_count"]==2 for r in conflicts))
    def test_snapshot_overwrite_can_understate_accepted_liability(self):
        a=hashlib.sha256(b"a").hexdigest(); b=hashlib.sha256(b"b").hexdigest()
        reqs={"A":Request("A",a,amount=40),"B":Request("B",b,amount=40)}
        rows=[run_naive(reqs,s,"snapshot_overwrite") for s in interleavings()]
        self.assertTrue(any(r["counter_understates_accepted_units"] for r in rows))
    def test_serializable_oracle_is_safe_for_both_request_orders(self):
        a=hashlib.sha256(b"a").hexdigest(); b=hashlib.sha256(b"b").hexdigest()
        reqs={"A":Request("A",a,amount=60),"B":Request("B",b,amount=60)}
        for order in (("A","B"),("B","A")):
            r=run_serializable(reqs,order)
            self.assertEqual(r["cap_overshoot_units"],0)
            self.assertEqual(r["accepted_event_count"],1)
    def test_atomic_idempotency_accepts_one_canonical_work_once(self):
        digest=hashlib.sha256(b"same").hexdigest()
        reqs={"A":Request("req-A",digest,amount=60),"B":Request("req-B",digest,amount=60)}
        for order in (("A","B"),("B","A")):
            r=run_serializable(reqs,order)
            self.assertEqual(r["accepted_event_count"],1)
            self.assertEqual(r["cap_overshoot_units"],0)
            self.assertFalse(r["duplicate_work_accepted"])
    def test_research_scenario_marks_claim_ceiling(self):
        digest=hashlib.sha256(b"a").hexdigest()
        x=scenario({"A":Request("A",digest),"B":Request("B",digest)},True)
        self.assertIn("finite abstract two-client model",x["claim_ceiling"])

if __name__=="__main__": unittest.main(verbosity=2)
