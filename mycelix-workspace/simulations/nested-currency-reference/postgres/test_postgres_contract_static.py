#!/usr/bin/env python3
"""Static guardrails for PostgreSQL SQL contract; not a substitute for live SQL tests."""
from pathlib import Path
import unittest

ROOT = Path(__file__).parent
FUNCTIONS = ROOT / "002_functions.sql"
SMOKE = ROOT / "900_smoke_test.sql"


class CanonicalAliasHomeShardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.functions = FUNCTIONS.read_text(encoding="utf-8")
        cls.smoke = SMOKE.read_text(encoding="utf-8")
        start = cls.functions.index("-- Canonical digest aliases")
        end = cls.functions.index("-- Global lock order", start)
        cls.alias_block = cls.functions[start:end]

    def test_canonical_alias_requires_same_reservation_shard_and_epoch(self):
        for guard in (
            "v_existing.reservation_id <> p_reservation_id",
            "v_existing.shard_id <> p_shard_id",
            "v_existing.epoch <> p_epoch",
        ):
            with self.subTest(guard=guard):
                self.assertIn(guard, self.alias_block)

    def test_scope_conflict_rejects_before_request_binding_insert(self):
        guard_at = self.alias_block.index("v_existing.shard_id <> p_shard_id")
        conflict_at = self.alias_block.index("canonical_work_identity_or_home_shard_conflict")
        insert_at = self.alias_block.index("INSERT INTO ncf.request_binding")
        self.assertLess(guard_at, conflict_at)
        self.assertLess(conflict_at, insert_at)

    def test_original_context_alias_still_returns_replay_without_second_debit(self):
        self.assertIn("'decision', 'replay'", self.alias_block)
        self.assertIn("v_existing.obligation_id, 'replay', v_existing.receipt_json", self.alias_block)
        self.assertNotIn("UPDATE ncf.community_instrument_cap", self.alias_block)
        self.assertNotIn("UPDATE ncf.agreement_direction_cap", self.alias_block)
        self.assertNotIn("UPDATE ncf.reservation_slice", self.alias_block)

    def test_smoke_has_non_home_shard_negative_control(self):
        self.assertIn("'non-home-shard'", self.smoke)
        self.assertIn("canonical_work_identity_or_home_shard_conflict", self.smoke)
        self.assertIn("expected cross-shard canonical-work alias rejection", self.smoke)


if __name__ == "__main__":
    unittest.main()
