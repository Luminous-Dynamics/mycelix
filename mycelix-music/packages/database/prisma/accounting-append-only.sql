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

-- Source eligibility observations may only assert externally observable facts.
-- Compiler outcomes such as below_threshold and awaiting_eligibility_evidence
-- are derived projection state and therefore may never enter the evidence log.
ALTER TABLE "RoyaltyEligibilityObservation"
  DROP CONSTRAINT IF EXISTS royalty_eligibility_observable_code;
ALTER TABLE "RoyaltyEligibilityObservation"
  ADD CONSTRAINT royalty_eligibility_observable_code
  CHECK ("code" IN (
    'eligible',
    'awaiting_payee_route',
    'rights_conflict',
    'legal_hold',
    'tax_documentation_required',
    'awaiting_fx_quote',
    'dormant_beneficiary'
  ));

-- Settlement observations are rail evidence with a closed state vocabulary.
ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_observation_state_code;
ALTER TABLE "SettlementAttemptObservationRecord"
  ADD CONSTRAINT settlement_observation_state_code
  CHECK ("state" IN (
    'authorized',
    'submitted',
    'accepted',
    'confirmed',
    'finalized',
    'failed',
    'rejected',
    'reversed',
    'disputed'
  ));

-- Finality is never inferred from a status string alone: every finalized rail
-- observation must retain a non-empty durable receipt reference.
ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_finalized_requires_receipt;
ALTER TABLE "SettlementAttemptObservationRecord"
  ADD CONSTRAINT settlement_finalized_requires_receipt
  CHECK (
    "state" <> 'finalized'
    OR ("railReceiptRef" IS NOT NULL AND btrim("railReceiptRef") <> '')
  );

-- A settlement attempt cannot rely on an eligibility snapshot that did not yet
-- exist when the attempt observation was emitted.
ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_observation_after_eligibility;
ALTER TABLE "SettlementAttemptObservationRecord"
  ADD CONSTRAINT settlement_observation_after_eligibility
  CHECK ("observedAt" >= "eligibilityAsOf");

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
