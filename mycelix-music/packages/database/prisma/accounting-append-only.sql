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

-- Canonical persisted money uses non-negative base-10 minor units. Authority
-- roots are lowercase SHA-256 digests. These checks protect direct SQL paths.
ALTER TABLE "RoyaltyObligationRecord"
  DROP CONSTRAINT IF EXISTS royalty_obligation_canonical_shape;
ALTER TABLE "RoyaltyObligationRecord"
  ADD CONSTRAINT royalty_obligation_canonical_shape
  CHECK (
    "amountMinor" ~ '^(0|[1-9][0-9]*)$'
    AND "obligationRoot" ~ '^[0-9a-f]{64}$'
  );

ALTER TABLE "RoyaltyEligibilityObservation"
  DROP CONSTRAINT IF EXISTS royalty_eligibility_digest_shape;
ALTER TABLE "RoyaltyEligibilityObservation"
  ADD CONSTRAINT royalty_eligibility_digest_shape
  CHECK ("observationRoot" ~ '^[0-9a-f]{64}$');

ALTER TABLE "RoyaltyDeductionRecord"
  DROP CONSTRAINT IF EXISTS royalty_deduction_canonical_shape;
ALTER TABLE "RoyaltyDeductionRecord"
  ADD CONSTRAINT royalty_deduction_canonical_shape
  CHECK (
    "amountMinor" ~ '^(0|[1-9][0-9]*)$'
    AND "deductionRoot" ~ '^[0-9a-f]{64}$'
  );

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

ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_observation_digest_shape;
ALTER TABLE "SettlementAttemptObservationRecord"
  ADD CONSTRAINT settlement_observation_digest_shape
  CHECK (
    "obligationSetRoot" ~ '^[0-9a-f]{64}$'
    AND "eligibilityEvidenceRoot" ~ '^[0-9a-f]{64}$'
    AND "observationRoot" ~ '^[0-9a-f]{64}$'
  );

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

-- Statements are projections, but persisted snapshots must still be incapable
-- of representing malformed roots or non-conserving/non-canonical money. CASE
-- prevents numeric casts from running on malformed text.
ALTER TABLE "RoyaltyStatementSnapshotRecord"
  DROP CONSTRAINT IF EXISTS royalty_statement_canonical_shape;
ALTER TABLE "RoyaltyStatementSnapshotRecord"
  ADD CONSTRAINT royalty_statement_canonical_shape
  CHECK (
    "obligationRoot" ~ '^[0-9a-f]{64}$'
    AND "adjustmentRoot" ~ '^[0-9a-f]{64}$'
    AND "settlementRoot" ~ '^[0-9a-f]{64}$'
    AND "snapshotRoot" ~ '^[0-9a-f]{64}$'
    AND CASE
      WHEN "grossMinor" ~ '^(0|[1-9][0-9]*)$'
       AND "heldMinor" ~ '^(0|[1-9][0-9]*)$'
       AND "deductionMinor" ~ '^(0|[1-9][0-9]*)$'
       AND "netPayableMinor" ~ '^(0|[1-9][0-9]*)$'
       AND "paidMinor" ~ '^(0|[1-9][0-9]*)$'
      THEN
        "grossMinor"::numeric = "heldMinor"::numeric + "deductionMinor"::numeric + "netPayableMinor"::numeric
        AND "paidMinor"::numeric <= "netPayableMinor"::numeric
      ELSE false
    END
  );

ALTER TABLE "RoyaltyStatementSnapshotRecord"
  DROP CONSTRAINT IF EXISTS royalty_statement_period_order;
ALTER TABLE "RoyaltyStatementSnapshotRecord"
  ADD CONSTRAINT royalty_statement_period_order
  CHECK ("periodStart" < "periodEnd");

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
