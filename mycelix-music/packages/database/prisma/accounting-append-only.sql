-- Creator accounting authority records are append-only.
-- Run after Prisma schema deployment:
--   npm run db:accounting-guards --workspace=@mycelix/database
--
-- This intentionally does not guard the legacy "RoyaltyPayment" table: that
-- table remains historical rail-receipt projection data and is not authority.

CREATE OR REPLACE FUNCTION reject_music_accounting_authority_mutation()
RETURNS trigger
LANGUAGE plpgsql
AS $$
BEGIN
  RAISE EXCEPTION '% is append-only; % is forbidden', TG_TABLE_NAME, TG_OP
    USING ERRCODE = '55000';
END;
$$;

DO $$
DECLARE
  table_name text;
BEGIN
  FOREACH table_name IN ARRAY ARRAY[
    'RoyaltyObligationRecord',
    'RoyaltyEligibilityObservation',
    'RoyaltyDeductionRecord',
    'SettlementAttemptObservationRecord',
    'RoyaltyStatementSnapshotRecord'
  ]
  LOOP
    EXECUTE format('DROP TRIGGER IF EXISTS accounting_append_only_guard ON %I', table_name);
    EXECUTE format(
      'CREATE TRIGGER accounting_append_only_guard BEFORE UPDATE OR DELETE ON %I FOR EACH ROW EXECUTE FUNCTION reject_music_accounting_authority_mutation()',
      table_name
    );
  END LOOP;
END;
$$;
