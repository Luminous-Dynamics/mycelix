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



class SqlFunctionInstallabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.functions = FUNCTIONS.read_text(encoding="utf-8")

    def test_plpgsql_body_delimiters_are_balanced(self):
        lines = self.functions.splitlines()
        open_delimiter = "AS " + chr(36) * 2
        close_delimiter = chr(36) * 2 + ";"
        openers = sum(line.strip() == open_delimiter for line in lines)
        closers = sum(line.strip() == close_delimiter for line in lines)
        self.assertGreaterEqual(openers, 1)
        self.assertEqual(openers, closers, "every dollar-quoted function body must have a matching close")
        self.assertNotRegex(self.functions, r"(?m)^\s*AS \$\s*$", "single-dollar AS opener is malformed")
        self.assertNotRegex(self.functions, r"(?m)^\s*\$;\s*$", "single-dollar function terminator is malformed")

    def test_all_security_definer_functions_put_pg_temp_last(self):
        import re
        definitions = re.split(r"(?m)(?=^CREATE OR REPLACE FUNCTION )", self.functions)
        definers = [definition for definition in definitions if "SECURITY DEFINER" in definition]
        self.assertEqual(len(definers), 7, "keep this assertion synchronized with the seven privileged NCF functions")
        for definition in definers:
            with self.subTest(function=definition.splitlines()[0]):
                self.assertRegex(
                    definition,
                    r"(?m)^SET search_path = pg_catalog, ncf, pg_temp$",
                    "SECURITY DEFINER functions must force the temp schema last",
                )


class PostgreSqlRoleSeparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.schema = (ROOT / "001_schema.sql").read_text(encoding="utf-8")
        cls.functions = FUNCTIONS.read_text(encoding="utf-8")
        cls.smoke = SMOKE.read_text(encoding="utf-8")

    def test_all_postgres_sql_files_have_balanced_dollar_delimiters(self):
        import re
        for path in sorted(ROOT.glob("*.sql")):
            sql = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                openers = re.findall(r"(?m)^\s*(?:AS|DO) \$\$", sql)
                closers = re.findall(r"(?m)^\s*\$\$;\s*$", sql)
                self.assertEqual(len(openers), len(closers), path.name)
                self.assertNotRegex(sql, r"(?m)^\s*(?:AS|DO) \$\s*$", path.name)
                self.assertNotRegex(sql, r"(?m)^\s*\$;\s*$", path.name)

    def test_schema_declares_distinct_nologin_capability_roles(self):
        for role in ("ncf_app", "ncf_dispatcher", "ncf_observer"):
            with self.subTest(role=role):
                self.assertIn("CREATE ROLE " + role + " NOLOGIN", self.schema)
                self.assertIn("GRANT USAGE ON SCHEMA ncf TO ncf_app, ncf_dispatcher, ncf_observer", self.schema)

    def test_economic_functions_only_granted_to_app_role(self):
        for grant in (
            "ncf.allocate_reservation_slice",
            "ncf.admit_obligation",
            "ncf.settle_obligation",
        ):
            self.assertRegex(self.functions, r"(?m)^GRANT EXECUTE ON FUNCTION " + re.escape(grant) + r"\([^;]+ TO ncf_app;$")
        self.assertNotIn("TO ncf_app;\nGRANT EXECUTE ON FUNCTION ncf.record_validation_receipts", self.functions)

    def test_dispatch_and_receipt_functions_are_separated_from_app(self):
        self.assertIn("GRANT EXECUTE ON FUNCTION ncf.lease_outbox_batch(text, integer, integer) TO ncf_dispatcher;", self.functions)
        self.assertIn("GRANT EXECUTE ON FUNCTION ncf.mark_outbox_retry(bigint, text, text, integer) TO ncf_dispatcher;", self.functions)
        self.assertIn("GRANT EXECUTE ON FUNCTION ncf.record_holochain_dispatch(bigint, text, text, text, text, text) TO ncf_dispatcher;", self.functions)
        self.assertIn("GRANT EXECUTE ON FUNCTION ncf.record_validation_receipts(text, text, integer, integer) TO ncf_observer;", self.functions)
        self.assertIn("REVOKE EXECUTE ON ALL FUNCTIONS IN SCHEMA ncf FROM ncf_app, ncf_dispatcher, ncf_observer;", self.functions)
        self.assertIn("ncf_app must not control dispatch or DHT validation observations", self.smoke)
        self.assertIn("ncf_dispatcher capability boundary is incorrect", self.smoke)
        self.assertIn("ncf_observer capability boundary is incorrect", self.smoke)


if __name__ == "__main__":
    unittest.main()
