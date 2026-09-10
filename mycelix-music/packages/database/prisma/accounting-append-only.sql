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

CREATE OR REPLACE FUNCTION enforce_royalty_eligibility_obligation_causality()
RETURNS trigger
LANGUAGE plpgsql
AS $$
DECLARE
  obligation_observed_at timestamp;
BEGIN
  SELECT "observedAt"
    INTO obligation_observed_at
    FROM "RoyaltyObligationRecord"
   WHERE "id" = NEW."obligationId";

  IF FOUND AND NEW."observedAt" < obligation_observed_at THEN
    RAISE EXCEPTION 'royalty_eligibility_after_obligation: eligibility observation % predates obligation %', NEW."id", NEW."obligationId"
      USING ERRCODE = '23514';
  END IF;
  RETURN NEW;
END;
$$;

-- Allocation deduction roots are stored canonically as a strictly increasing
-- JSON array of lowercase SHA-256 strings. Strict ordering also proves uniqueness.
CREATE OR REPLACE FUNCTION canonical_sha256_json_array(value jsonb)
RETURNS boolean
LANGUAGE plpgsql
IMMUTABLE
AS $$
DECLARE
  element jsonb;
  current_value text;
  previous_value text := NULL;
BEGIN
  IF jsonb_typeof(value) <> 'array' THEN
    RETURN false;
  END IF;

  FOR element IN SELECT * FROM jsonb_array_elements(value)
  LOOP
    IF jsonb_typeof(element) <> 'string' THEN
      RETURN false;
    END IF;
    current_value := element #>> '{}';
    IF current_value !~ '^[0-9a-f]{64}$' THEN
      RETURN false;
    END IF;
    IF previous_value IS NOT NULL AND current_value <= previous_value THEN
      RETURN false;
    END IF;
    previous_value := current_value;
  END LOOP;
  RETURN true;
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

-- Rail finality may cover all or part of a deterministic batch. PostgreSQL can
-- prove that finalized rows carry canonical amount/currency evidence and that
-- non-final rows do not. Exact batch comparison is canonical replay authority.
ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_finalized_requires_receipt;
ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_finality_evidence_shape;
ALTER TABLE "SettlementAttemptObservationRecord"
  ADD CONSTRAINT settlement_finality_evidence_shape
  CHECK (
    CASE
      WHEN "state" = 'finalized' THEN
        "railReceiptRef" IS NOT NULL
        AND btrim("railReceiptRef") <> ''
        AND "settledAmountMinor" IS NOT NULL
        AND "settledAmountMinor" ~ '^[1-9][0-9]*$'
        AND "settledCurrency" IS NOT NULL
        AND btrim("settledCurrency") <> ''
      ELSE
        "settledAmountMinor" IS NULL
        AND "settledCurrency" IS NULL
    END
  );

-- A settlement attempt cannot rely on an eligibility snapshot that did not yet
-- exist when the attempt observation was emitted.
ALTER TABLE "SettlementAttemptObservationRecord"
  DROP CONSTRAINT IF EXISTS settlement_observation_after_eligibility;
ALTER TABLE "SettlementAttemptObservationRecord"
  ADD CONSTRAINT settlement_observation_after_eligibility
  CHECK ("observedAt" >= "eligibilityAsOf");

-- Allocation rows preserve an already-created semantic authority. PostgreSQL
-- enforces canonical local shape, residual provenance and discharge consistency.
-- The full conservation theorem is re-proved on canonical replay against the
-- deterministic batch and deduction authorities; batch gross is not duplicated here.
ALTER TABLE "SettlementAllocationRecord"
  DROP CONSTRAINT IF EXISTS settlement_allocation_canonical_shape;
ALTER TABLE "SettlementAllocationRecord"
  ADD CONSTRAINT settlement_allocation_canonical_shape
  CHECK (
    "obligationSetRoot" ~ '^[0-9a-f]{64}$'
    AND "eligibilityEvidenceRoot" ~ '^[0-9a-f]{64}$'
    AND "allocationRoot" ~ '^[0-9a-f]{64}$'
    AND btrim("batchId") <> ''
    AND btrim("beneficiaryId") <> ''
    AND btrim("currency") <> ''
    AND btrim("railReceiptRef") <> ''
    AND canonical_sha256_json_array("deductionRoots")
    AND "creatorPaidMinor" ~ '^[1-9][0-9]*$'
    AND "deductionTotalMinor" ~ '^(0|[1-9][0-9]*)$'
    AND (jsonb_array_length("deductionRoots") > 0 OR "deductionTotalMinor" = '0')
    AND "residualHeldMinor" ~ '^(0|[1-9][0-9]*)$'
    AND "allocatedAt" >= "eligibilityAsOf"
    AND CASE
      WHEN "residualHeldMinor" ~ '^(0|[1-9][0-9]*)$' THEN
        CASE
          WHEN "residualHeldMinor"::numeric = 0 THEN
            "obligationSetDischarged" = true
            AND "residualAuthorityRef" IS NULL
          ELSE
            "obligationSetDischarged" = false
            AND "residualAuthorityRef" IS NOT NULL
            AND btrim("residualAuthorityRef") <> ''
        END
      ELSE false
    END
  );

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

DROP TRIGGER IF EXISTS royalty_eligibility_causality_guard ON "RoyaltyEligibilityObservation";
CREATE TRIGGER royalty_eligibility_causality_guard
BEFORE INSERT ON "RoyaltyEligibilityObservation"
FOR EACH ROW EXECUTE FUNCTION enforce_royalty_eligibility_obligation_causality();

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

DROP TRIGGER IF EXISTS settlement_allocation_append_only_guard ON "SettlementAllocationRecord";
CREATE TRIGGER settlement_allocation_append_only_guard
BEFORE UPDATE OR DELETE ON "SettlementAllocationRecord"
FOR EACH ROW EXECUTE FUNCTION reject_music_accounting_authority_mutation();
