-- Serialized source registry for settlement-allocation lineage checkpoints.
--
-- This file is intentionally applied after Prisma schema deployment, alongside
-- accounting-append-only.sql. The application reads these tables through raw
-- SQL because their role is operational source-completeness evidence, not a new
-- source of economic authority.
--
-- The advisory transaction lock is shared by allocation/link writers and the
-- checkpoint snapshot reader. Cursor assignment uses MAX+1 while holding that
-- lock rather than a free-running sequence, so cursor order equals serialized
-- committed-source order for this registry.

CREATE TABLE IF NOT EXISTS "SettlementAllocationSuccessorLinkRecord" (
  "linkId" text PRIMARY KEY,
  "batchId" text NOT NULL,
  "predecessorAllocationId" text NOT NULL,
  "predecessorAllocationRoot" text NOT NULL,
  "successorAllocationId" text NOT NULL,
  "successorAllocationRoot" text NOT NULL,
  "supersessionEvidenceRef" text NOT NULL,
  "linkedAt" timestamp NOT NULL,
  "linkRoot" text NOT NULL UNIQUE,
  "createdAt" timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  CONSTRAINT settlement_allocation_link_predecessor_fk
    FOREIGN KEY ("predecessorAllocationId") REFERENCES "SettlementAllocationRecord"("allocationId") ON DELETE RESTRICT,
  CONSTRAINT settlement_allocation_link_successor_fk
    FOREIGN KEY ("successorAllocationId") REFERENCES "SettlementAllocationRecord"("allocationId") ON DELETE RESTRICT,
  CONSTRAINT settlement_allocation_link_predecessor_unique UNIQUE ("predecessorAllocationId"),
  CONSTRAINT settlement_allocation_link_successor_unique UNIQUE ("successorAllocationId"),
  CONSTRAINT settlement_allocation_link_distinct_endpoints CHECK ("predecessorAllocationId" <> "successorAllocationId"),
  CONSTRAINT settlement_allocation_link_canonical_shape CHECK (
    btrim("linkId") <> ''
    AND btrim("batchId") <> ''
    AND btrim("supersessionEvidenceRef") <> ''
    AND "predecessorAllocationRoot" ~ '^[0-9a-f]{64}$'
    AND "successorAllocationRoot" ~ '^[0-9a-f]{64}$'
    AND "linkRoot" ~ '^[0-9a-f]{64}$'
  )
);

CREATE INDEX IF NOT EXISTS settlement_allocation_link_batch_time_idx
  ON "SettlementAllocationSuccessorLinkRecord" ("batchId", "linkedAt");

CREATE TABLE IF NOT EXISTS "SettlementAllocationLineageIngestRecord" (
  "ingestSeq" bigint PRIMARY KEY,
  "recordKind" text NOT NULL,
  "recordId" text NOT NULL,
  "batchId" text NOT NULL,
  "recordRoot" text NOT NULL,
  "evidenceAt" timestamp NOT NULL,
  "createdAt" timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  CONSTRAINT settlement_allocation_lineage_ingest_kind CHECK ("recordKind" IN ('allocation', 'successor_link')),
  CONSTRAINT settlement_allocation_lineage_ingest_shape CHECK (
    btrim("recordId") <> ''
    AND btrim("batchId") <> ''
    AND "recordRoot" ~ '^[0-9a-f]{64}$'
  ),
  CONSTRAINT settlement_allocation_lineage_ingest_identity UNIQUE ("recordKind", "recordId")
);

CREATE INDEX IF NOT EXISTS settlement_allocation_lineage_ingest_batch_seq_idx
  ON "SettlementAllocationLineageIngestRecord" ("batchId", "ingestSeq");

CREATE OR REPLACE FUNCTION lock_music_accounting_allocation_lineage_source()
RETURNS void
LANGUAGE plpgsql
AS $$
BEGIN
  PERFORM pg_advisory_xact_lock(hashtextextended('mycelix-accounting-allocation-lineage-v1', 0));
END;
$$;

CREATE OR REPLACE FUNCTION lock_music_accounting_allocation_lineage_writer()
RETURNS trigger
LANGUAGE plpgsql
AS $$
BEGIN
  PERFORM lock_music_accounting_allocation_lineage_source();
  RETURN NEW;
END;
$$;

CREATE OR REPLACE FUNCTION validate_settlement_allocation_successor_link()
RETURNS trigger
LANGUAGE plpgsql
AS $$
DECLARE
  predecessor "SettlementAllocationRecord"%ROWTYPE;
  successor "SettlementAllocationRecord"%ROWTYPE;
  predecessor_root text;
  creator_changed boolean;
  receipt_changed boolean;
BEGIN
  SELECT * INTO predecessor
    FROM "SettlementAllocationRecord"
   WHERE "allocationId" = NEW."predecessorAllocationId";
  IF NOT FOUND THEN
    RAISE EXCEPTION 'settlement_allocation_link_predecessor_missing: %', NEW."predecessorAllocationId"
      USING ERRCODE = '23503';
  END IF;

  SELECT * INTO successor
    FROM "SettlementAllocationRecord"
   WHERE "allocationId" = NEW."successorAllocationId";
  IF NOT FOUND THEN
    RAISE EXCEPTION 'settlement_allocation_link_successor_missing: %', NEW."successorAllocationId"
      USING ERRCODE = '23503';
  END IF;

  IF NEW."batchId" <> predecessor."batchId" OR NEW."batchId" <> successor."batchId" THEN
    RAISE EXCEPTION 'settlement_allocation_link_batch_mismatch' USING ERRCODE = '23514';
  END IF;
  IF NEW."predecessorAllocationRoot" <> predecessor."allocationRoot"
     OR NEW."successorAllocationRoot" <> successor."allocationRoot" THEN
    RAISE EXCEPTION 'settlement_allocation_link_root_mismatch' USING ERRCODE = '23514';
  END IF;
  IF predecessor."obligationSetRoot" <> successor."obligationSetRoot"
     OR predecessor."eligibilityAsOf" <> successor."eligibilityAsOf"
     OR predecessor."eligibilityEvidenceRoot" <> successor."eligibilityEvidenceRoot"
     OR predecessor."beneficiaryId" <> successor."beneficiaryId"
     OR predecessor."currency" <> successor."currency" THEN
    RAISE EXCEPTION 'settlement_allocation_link_authority_identity_mismatch' USING ERRCODE = '23514';
  END IF;
  IF predecessor."obligationSetDischarged" THEN
    RAISE EXCEPTION 'settlement_allocation_link_discharged_predecessor' USING ERRCODE = '23514';
  END IF;
  IF successor."allocatedAt" <= predecessor."allocatedAt" THEN
    RAISE EXCEPTION 'settlement_allocation_link_nonmonotonic_time' USING ERRCODE = '23514';
  END IF;
  IF NEW."linkedAt" < successor."allocatedAt" THEN
    RAISE EXCEPTION 'settlement_allocation_link_predates_successor' USING ERRCODE = '23514';
  END IF;

  IF successor."residualHeldMinor"::numeric >= predecessor."residualHeldMinor"::numeric THEN
    RAISE EXCEPTION 'settlement_allocation_link_residual_not_reduced' USING ERRCODE = '23514';
  END IF;
  IF successor."creatorPaidMinor"::numeric < predecessor."creatorPaidMinor"::numeric THEN
    RAISE EXCEPTION 'settlement_allocation_link_creator_paid_regression' USING ERRCODE = '23514';
  END IF;
  IF successor."deductionTotalMinor"::numeric < predecessor."deductionTotalMinor"::numeric THEN
    RAISE EXCEPTION 'settlement_allocation_link_deduction_regression' USING ERRCODE = '23514';
  END IF;

  FOR predecessor_root IN SELECT jsonb_array_elements_text(predecessor."deductionRoots")
  LOOP
    IF NOT (successor."deductionRoots" ? predecessor_root) THEN
      RAISE EXCEPTION 'settlement_allocation_link_deduction_disappeared: %', predecessor_root
        USING ERRCODE = '23514';
    END IF;
  END LOOP;
  IF successor."deductionTotalMinor"::numeric > predecessor."deductionTotalMinor"::numeric
     AND jsonb_array_length(successor."deductionRoots") <= jsonb_array_length(predecessor."deductionRoots") THEN
    RAISE EXCEPTION 'settlement_allocation_link_deduction_growth_requires_authority' USING ERRCODE = '23514';
  END IF;

  creator_changed := successor."creatorPaidMinor"::numeric <> predecessor."creatorPaidMinor"::numeric;
  receipt_changed := successor."railReceiptRef" <> predecessor."railReceiptRef";
  IF creator_changed <> receipt_changed THEN
    RAISE EXCEPTION 'settlement_allocation_link_receipt_amount_rebinding' USING ERRCODE = '23514';
  END IF;

  RETURN NEW;
END;
$$;

CREATE OR REPLACE FUNCTION assign_and_validate_settlement_allocation_lineage_ingest()
RETURNS trigger
LANGUAGE plpgsql
AS $$
DECLARE
  expected_batch text;
  expected_root text;
  expected_time timestamp;
BEGIN
  PERFORM lock_music_accounting_allocation_lineage_source();

  IF NEW."ingestSeq" IS NOT NULL THEN
    RAISE EXCEPTION 'settlement_allocation_lineage_ingest_cursor_is_database_assigned'
      USING ERRCODE = '23514';
  END IF;

  IF NEW."recordKind" = 'allocation' THEN
    SELECT "batchId", "allocationRoot", "allocatedAt"
      INTO expected_batch, expected_root, expected_time
      FROM "SettlementAllocationRecord"
     WHERE "allocationId" = NEW."recordId";
  ELSIF NEW."recordKind" = 'successor_link' THEN
    SELECT "batchId", "linkRoot", "linkedAt"
      INTO expected_batch, expected_root, expected_time
      FROM "SettlementAllocationSuccessorLinkRecord"
     WHERE "linkId" = NEW."recordId";
  ELSE
    RAISE EXCEPTION 'settlement_allocation_lineage_ingest_unsupported_kind: %', NEW."recordKind"
      USING ERRCODE = '23514';
  END IF;

  IF expected_batch IS NULL THEN
    RAISE EXCEPTION 'settlement_allocation_lineage_ingest_missing_source_record: %/%', NEW."recordKind", NEW."recordId"
      USING ERRCODE = '23503';
  END IF;
  IF NEW."batchId" <> expected_batch OR NEW."recordRoot" <> expected_root OR NEW."evidenceAt" <> expected_time THEN
    RAISE EXCEPTION 'settlement_allocation_lineage_ingest_source_mismatch: %/%', NEW."recordKind", NEW."recordId"
      USING ERRCODE = '23514';
  END IF;

  SELECT COALESCE(MAX("ingestSeq"), 0) + 1
    INTO NEW."ingestSeq"
    FROM "SettlementAllocationLineageIngestRecord";
  RETURN NEW;
END;
$$;

CREATE OR REPLACE FUNCTION register_settlement_allocation_lineage_ingest()
RETURNS trigger
LANGUAGE plpgsql
AS $$
BEGIN
  IF TG_ARGV[0] = 'allocation' THEN
    INSERT INTO "SettlementAllocationLineageIngestRecord"
      ("recordKind", "recordId", "batchId", "recordRoot", "evidenceAt")
    VALUES
      ('allocation', NEW."allocationId", NEW."batchId", NEW."allocationRoot", NEW."allocatedAt")
    ON CONFLICT ("recordKind", "recordId") DO NOTHING;
  ELSIF TG_ARGV[0] = 'successor_link' THEN
    INSERT INTO "SettlementAllocationLineageIngestRecord"
      ("recordKind", "recordId", "batchId", "recordRoot", "evidenceAt")
    VALUES
      ('successor_link', NEW."linkId", NEW."batchId", NEW."linkRoot", NEW."linkedAt")
    ON CONFLICT ("recordKind", "recordId") DO NOTHING;
  ELSE
    RAISE EXCEPTION 'unsupported lineage ingestion trigger kind: %', TG_ARGV[0]
      USING ERRCODE = '23514';
  END IF;
  RETURN NEW;
END;
$$;

DROP TRIGGER IF EXISTS settlement_allocation_lineage_writer_lock ON "SettlementAllocationRecord";
CREATE TRIGGER settlement_allocation_lineage_writer_lock
BEFORE INSERT ON "SettlementAllocationRecord"
FOR EACH ROW EXECUTE FUNCTION lock_music_accounting_allocation_lineage_writer();

DROP TRIGGER IF EXISTS settlement_allocation_successor_link_writer_lock ON "SettlementAllocationSuccessorLinkRecord";
CREATE TRIGGER settlement_allocation_successor_link_writer_lock
BEFORE INSERT ON "SettlementAllocationSuccessorLinkRecord"
FOR EACH ROW EXECUTE FUNCTION lock_music_accounting_allocation_lineage_writer();

DROP TRIGGER IF EXISTS settlement_allocation_successor_link_semantic_guard ON "SettlementAllocationSuccessorLinkRecord";
CREATE TRIGGER settlement_allocation_successor_link_semantic_guard
BEFORE INSERT ON "SettlementAllocationSuccessorLinkRecord"
FOR EACH ROW EXECUTE FUNCTION validate_settlement_allocation_successor_link();

DROP TRIGGER IF EXISTS settlement_allocation_lineage_ingest_assign_guard ON "SettlementAllocationLineageIngestRecord";
CREATE TRIGGER settlement_allocation_lineage_ingest_assign_guard
BEFORE INSERT ON "SettlementAllocationLineageIngestRecord"
FOR EACH ROW EXECUTE FUNCTION assign_and_validate_settlement_allocation_lineage_ingest();

DROP TRIGGER IF EXISTS settlement_allocation_ingest_register ON "SettlementAllocationRecord";
CREATE TRIGGER settlement_allocation_ingest_register
AFTER INSERT ON "SettlementAllocationRecord"
FOR EACH ROW EXECUTE FUNCTION register_settlement_allocation_lineage_ingest('allocation');

DROP TRIGGER IF EXISTS settlement_allocation_successor_link_ingest_register ON "SettlementAllocationSuccessorLinkRecord";
CREATE TRIGGER settlement_allocation_successor_link_ingest_register
AFTER INSERT ON "SettlementAllocationSuccessorLinkRecord"
FOR EACH ROW EXECUTE FUNCTION register_settlement_allocation_lineage_ingest('successor_link');

DROP TRIGGER IF EXISTS settlement_allocation_successor_link_append_only_guard ON "SettlementAllocationSuccessorLinkRecord";
CREATE TRIGGER settlement_allocation_successor_link_append_only_guard
BEFORE UPDATE OR DELETE ON "SettlementAllocationSuccessorLinkRecord"
FOR EACH ROW EXECUTE FUNCTION reject_music_accounting_authority_mutation();

DROP TRIGGER IF EXISTS settlement_allocation_lineage_ingest_append_only_guard ON "SettlementAllocationLineageIngestRecord";
CREATE TRIGGER settlement_allocation_lineage_ingest_append_only_guard
BEFORE UPDATE OR DELETE ON "SettlementAllocationLineageIngestRecord"
FOR EACH ROW EXECUTE FUNCTION reject_music_accounting_authority_mutation();

-- Backfill allocations that predate installation of the serialized source
-- registry without mutating their economic authority rows. The same lock makes
-- bootstrap order and future ingestion order one serialized namespace.
DO $$
DECLARE
  allocation_row record;
BEGIN
  PERFORM lock_music_accounting_allocation_lineage_source();
  FOR allocation_row IN
    SELECT a."allocationId", a."batchId", a."allocationRoot", a."allocatedAt"
      FROM "SettlementAllocationRecord" a
     WHERE NOT EXISTS (
       SELECT 1
         FROM "SettlementAllocationLineageIngestRecord" r
        WHERE r."recordKind" = 'allocation'
          AND r."recordId" = a."allocationId"
     )
     ORDER BY a."createdAt", a."allocationId"
  LOOP
    INSERT INTO "SettlementAllocationLineageIngestRecord"
      ("recordKind", "recordId", "batchId", "recordRoot", "evidenceAt")
    VALUES
      ('allocation', allocation_row."allocationId", allocation_row."batchId", allocation_row."allocationRoot", allocation_row."allocatedAt");
  END LOOP;
END;
$$;
