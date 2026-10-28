#!/usr/bin/env python3
"""NCF-ALLOC-5 sequential reference-model tests."""
import unittest
from ncf_alloc5_obligation_admission import Decision, ObligationBook, evidence, run_alias_scenario

class ObligationAdmissionTests(unittest.TestCase):
    def test_same_obligation_replayed_through_many_shards_counts_once(self):
        for aliases in (1,4,8,16,32):
            row=run_alias_scenario(aliases)
            self.assertEqual(row["actual_unique_obligation_units"],row["book_outstanding_units"])
            self.assertEqual(row["accepted_unique_obligations"],1)
            self.assertEqual(row["outcome_counts"].get("accepted",0),1)
            self.assertEqual(row["outcome_counts"].get("idempotent_replay",0),aliases-1)
    def test_exact_replay_across_epoch_returns_original_receipt(self):
        book=ObligationBook({("principal-A","instrument-1"):1000})
        first=book.apply(evidence("ob-1",epoch=1,amount_units=100))
        replay=book.apply(evidence("ob-1",shard_id="other-shard",epoch=99,sequence=500,amount_units=100))
        self.assertEqual(first.decision,Decision.ACCEPTED); self.assertEqual(replay.decision,Decision.IDEMPOTENT_REPLAY)
        self.assertEqual(first.receipt,replay.receipt); self.assertEqual(book.outstanding_for("principal-A","instrument-1"),100)
    def test_same_id_different_amount_rejects_without_mutation(self):
        book=ObligationBook({("principal-A","instrument-1"):1000})
        first=book.apply(evidence("ob-1",amount_units=100)); second=book.apply(evidence("ob-1",amount_units=101))
        self.assertEqual(first.decision,Decision.ACCEPTED); self.assertEqual(second.decision,Decision.REJECTED)
        self.assertEqual(second.reason,"obligation_id_conflicts_with_accepted_payload")
        self.assertEqual(book.outstanding_for("principal-A","instrument-1"),100)
    def test_same_id_different_principal_or_instrument_rejects(self):
        book=ObligationBook({("principal-A","instrument-1"):1000,("principal-B","instrument-1"):1000,("principal-A","instrument-2"):1000})
        self.assertEqual(book.apply(evidence("ob-1")).decision,Decision.ACCEPTED)
        self.assertEqual(book.apply(evidence("ob-1",principal_id="principal-B")).decision,Decision.REJECTED)
        self.assertEqual(book.apply(evidence("ob-1",instrument_id="instrument-2")).decision,Decision.REJECTED)
        self.assertEqual(book.accepted_count(),1)
    def test_principal_cap_aggregates_distinct_obligations_across_shards(self):
        book=ObligationBook({("principal-A","instrument-1"):150})
        self.assertEqual(book.apply(evidence("ob-1",shard_id="shard-a",amount_units=100)).decision,Decision.ACCEPTED)
        rejected=book.apply(evidence("ob-2",shard_id="shard-b",amount_units=51))
        self.assertEqual(rejected.decision,Decision.REJECTED); self.assertEqual(book.accepted_count(),1)
        self.assertEqual(book.outstanding_for("principal-A","instrument-1"),100)
        # A rejected ID is not consumed; a corrected eligible amount may be retried.
        self.assertEqual(book.apply(evidence("ob-2",shard_id="shard-b",amount_units=50)).decision,Decision.ACCEPTED)
    def test_unverified_or_malformed_evidence_fails_closed_without_id_consumption(self):
        book=ObligationBook({("principal-A","instrument-1"):1000})
        bad=book.apply(evidence("ob-1",amount_units=100,verifier_receipt_ok=False))
        self.assertEqual(bad.decision,Decision.REJECTED); self.assertEqual(book.accepted_count(),0)
        good=book.apply(evidence("ob-1",amount_units=100)); self.assertEqual(good.decision,Decision.ACCEPTED)
    def test_missing_cap_fails_closed(self):
        book=ObligationBook({})
        r=book.apply(evidence("ob-1",amount_units=100))
        self.assertEqual(r.decision,Decision.REJECTED); self.assertEqual(r.reason,"missing_principal_instrument_cap")
    def test_invalid_cap_rejects_on_book_creation(self):
        with self.assertRaises(ValueError): ObligationBook({(" ","instrument-1"):100})

if __name__ == "__main__": unittest.main(verbosity=2)
