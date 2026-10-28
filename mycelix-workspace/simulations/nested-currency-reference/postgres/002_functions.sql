-- NCF PostgreSQL transaction functions for the shared-coordinator profile.
-- A caller must supply verifier outputs only after independent verification.
-- This SQL does not perform cryptographic signature or real-world work verification.
BEGIN;

CREATE OR REPLACE FUNCTION ncf.allocate_reservation_slice(
    p_reservation_id uuid,
    p_agreement_id uuid,
    p_community_id text,
    p_instrument_id text,
    p_direction text,
    p_shard_id text,
    p_epoch bigint,
    p_reserved_units numeric,
    p_manifest_digest bytea
) RETURNS jsonb
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
DECLARE
    v_agreement ncf.clearing_agreement%ROWTYPE;
    v_cap ncf.community_instrument_cap%ROWTYPE;
    v_existing ncf.reservation_slice%ROWTYPE;
BEGIN
    IF p_reservation_id IS NULL OR p_agreement_id IS NULL
       OR p_community_id IS NULL OR p_instrument_id IS NULL
       OR p_direction IS NULL OR p_direction NOT IN ('a_to_b', 'b_to_a')
       OR p_shard_id IS NULL OR length(btrim(p_shard_id)) = 0
       OR p_epoch IS NULL OR p_epoch < 0
       OR p_reserved_units IS NULL OR p_reserved_units <= 0
       OR p_reserved_units::text IN ('NaN', 'Infinity', '-Infinity')
       OR p_reserved_units <> trunc(p_reserved_units)
       OR abs(p_reserved_units) >= power(10::numeric, 78)
       OR p_manifest_digest IS NULL OR octet_length(p_manifest_digest) <> 32 THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'invalid_reservation_input');
    END IF;

    SELECT * INTO v_existing
      FROM ncf.reservation_slice
     WHERE reservation_id = p_reservation_id;
    IF FOUND THEN
        IF v_existing.agreement_id = p_agreement_id
           AND v_existing.community_id = p_community_id
           AND v_existing.instrument_id = p_instrument_id
           AND v_existing.direction = p_direction
           AND v_existing.shard_id = p_shard_id
           AND v_existing.epoch = p_epoch
           AND v_existing.reserved_units = p_reserved_units
           AND v_existing.manifest_digest = p_manifest_digest THEN
            RETURN jsonb_build_object('decision', 'replay', 'reservation_id', p_reservation_id);
        END IF;
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'reservation_id_conflict');
    END IF;

    SELECT * INTO v_agreement
      FROM ncf.clearing_agreement
     WHERE agreement_id = p_agreement_id
     FOR SHARE;
    IF NOT FOUND OR v_agreement.state <> 'active' THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'agreement_missing_or_inactive');
    END IF;
    IF v_agreement.instrument_id <> p_instrument_id
       OR p_community_id <> CASE p_direction
            WHEN 'a_to_b' THEN v_agreement.community_a
            ELSE v_agreement.community_b
          END THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'reservation_scope_mismatch');
    END IF;

    SELECT * INTO v_cap
      FROM ncf.community_instrument_cap
     WHERE community_id = p_community_id
       AND instrument_id = p_instrument_id
     FOR UPDATE;
    IF NOT FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'missing_community_instrument_cap');
    END IF;
    IF v_cap.allocated_units + p_reserved_units > v_cap.cap_units THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'reservation_exceeds_unallocated_cap');
    END IF;

    INSERT INTO ncf.reservation_slice(
        reservation_id, agreement_id, community_id, instrument_id, direction,
        shard_id, epoch, reserved_units, used_units, manifest_digest, state
    ) VALUES (
        p_reservation_id, p_agreement_id, p_community_id, p_instrument_id, p_direction,
        p_shard_id, p_epoch, p_reserved_units, 0, p_manifest_digest, 'active'
    );

    UPDATE ncf.community_instrument_cap
       SET allocated_units = allocated_units + p_reserved_units
     WHERE community_id = p_community_id AND instrument_id = p_instrument_id;

    RETURN jsonb_build_object(
        'decision', 'accepted',
        'reservation_id', p_reservation_id,
        'reserved_units', p_reserved_units::text,
        'manifest_digest_sha256', encode(p_manifest_digest, 'hex')
    );
EXCEPTION WHEN unique_violation THEN
    SELECT * INTO v_existing FROM ncf.reservation_slice WHERE reservation_id = p_reservation_id;
    IF FOUND
       AND v_existing.agreement_id = p_agreement_id
       AND v_existing.community_id = p_community_id
       AND v_existing.instrument_id = p_instrument_id
       AND v_existing.direction = p_direction
       AND v_existing.shard_id = p_shard_id
       AND v_existing.epoch = p_epoch
       AND v_existing.reserved_units = p_reserved_units
       AND v_existing.manifest_digest = p_manifest_digest THEN
        RETURN jsonb_build_object('decision', 'replay', 'reservation_id', p_reservation_id);
    ELSIF FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'reservation_id_conflict');
    END IF;
    RAISE;
END;
$$;

CREATE OR REPLACE FUNCTION ncf.admit_obligation(
    p_request_id uuid,
    p_payload_digest bytea,
    p_principal_id text,
    p_community_id text,
    p_instrument_id text,
    p_profile_revision integer,
    p_amount_units numeric,
    p_canonical_work_digest bytea,
    p_agreement_id uuid,
    p_direction text,
    p_reservation_id uuid,
    p_shard_id text,
    p_epoch bigint,
    p_verifier_version text,
    p_verifier_receipt_digest bytea
) RETURNS jsonb
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
DECLARE
    v_binding ncf.request_binding%ROWTYPE;
    v_receipt jsonb;
    v_existing ncf.obligation%ROWTYPE;
    v_agreement ncf.clearing_agreement%ROWTYPE;
    v_cap ncf.community_instrument_cap%ROWTYPE;
    v_slice ncf.reservation_slice%ROWTYPE;
    v_obligation_id bigint;
    v_new_receipt jsonb;
    v_outbox_payload jsonb;
BEGIN
    IF p_request_id IS NULL
       OR p_payload_digest IS NULL OR octet_length(p_payload_digest) <> 32
       OR p_principal_id IS NULL OR length(btrim(p_principal_id)) = 0
       OR p_community_id IS NULL OR p_instrument_id IS NULL
       OR p_profile_revision IS NULL OR p_profile_revision <= 0
       OR p_amount_units IS NULL OR p_amount_units <= 0
       OR p_amount_units::text IN ('NaN', 'Infinity', '-Infinity')
       OR p_amount_units <> trunc(p_amount_units)
       OR abs(p_amount_units) >= power(10::numeric, 78)
       OR p_canonical_work_digest IS NULL OR octet_length(p_canonical_work_digest) <> 32
       OR p_agreement_id IS NULL OR p_direction IS NULL OR p_direction NOT IN ('a_to_b', 'b_to_a')
       OR p_reservation_id IS NULL OR p_shard_id IS NULL OR length(btrim(p_shard_id)) = 0
       OR p_epoch IS NULL OR p_epoch < 0
       OR p_verifier_version IS NULL OR length(btrim(p_verifier_version)) = 0
       OR p_verifier_receipt_digest IS NULL OR octet_length(p_verifier_receipt_digest) <> 32 THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'invalid_or_unverified_input');
    END IF;

    -- Serializes identical canonical-work keys even when callers use different request IDs.
    -- Hash collisions only serialize unrelated work; the unique digest constraint remains authoritative.
    PERFORM pg_advisory_xact_lock(hashtextextended(encode(p_canonical_work_digest, 'hex'), 0));

    SELECT rb, ob.receipt_json
      INTO v_binding, v_receipt
      FROM ncf.request_binding rb
      JOIN ncf.obligation ob ON ob.obligation_id = rb.obligation_id
     WHERE rb.request_id = p_request_id
     FOR UPDATE OF rb;
    IF FOUND THEN
        IF v_binding.payload_digest = p_payload_digest
           AND v_binding.principal_id = p_principal_id
           AND v_binding.community_id = p_community_id
           AND v_binding.instrument_id = p_instrument_id
           AND v_binding.profile_revision = p_profile_revision
           AND v_binding.amount_units = p_amount_units
           AND v_binding.canonical_work_digest = p_canonical_work_digest
           AND v_binding.agreement_id = p_agreement_id
           AND v_binding.direction = p_direction
           AND v_binding.reservation_id = p_reservation_id
           AND v_binding.shard_id = p_shard_id
           AND v_binding.epoch = p_epoch
           AND v_binding.verifier_version = p_verifier_version
           AND v_binding.verifier_receipt_digest = p_verifier_receipt_digest THEN
            RETURN jsonb_build_object(
                'decision', 'replay',
                'obligation_id', v_binding.obligation_id,
                'receipt', v_receipt
            );
        END IF;
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'request_id_payload_or_evidence_conflict');
    END IF;

    -- Canonical digest aliases return the original receipt without a second cap/slice debit.
    SELECT * INTO v_existing
      FROM ncf.obligation
     WHERE canonical_work_digest = p_canonical_work_digest;
    IF FOUND THEN
        IF v_existing.principal_id <> p_principal_id
           OR v_existing.community_id <> p_community_id
           OR v_existing.instrument_id <> p_instrument_id
           OR v_existing.profile_revision <> p_profile_revision
           OR v_existing.amount_units <> p_amount_units
           OR v_existing.agreement_id <> p_agreement_id
           OR v_existing.direction <> p_direction THEN
            RETURN jsonb_build_object('decision', 'rejected', 'reason', 'canonical_work_identity_conflict');
        END IF;

        INSERT INTO ncf.request_binding(
            request_id, payload_digest, principal_id, community_id, instrument_id,
            profile_revision, amount_units, canonical_work_digest, agreement_id, direction,
            reservation_id, shard_id, epoch, verifier_version, verifier_receipt_digest,
            obligation_id, decision, receipt_json
        ) VALUES (
            p_request_id, p_payload_digest, p_principal_id, p_community_id, p_instrument_id,
            p_profile_revision, p_amount_units, p_canonical_work_digest, p_agreement_id, p_direction,
            p_reservation_id, p_shard_id, p_epoch, p_verifier_version, p_verifier_receipt_digest,
            v_existing.obligation_id, 'replay', v_existing.receipt_json
        );
        RETURN jsonb_build_object(
            'decision', 'replay',
            'obligation_id', v_existing.obligation_id,
            'receipt', v_existing.receipt_json
        );
    END IF;

    -- Lock order for all new-admission paths: agreement -> community cap -> reservation slice.
    SELECT * INTO v_agreement
      FROM ncf.clearing_agreement
     WHERE agreement_id = p_agreement_id
     FOR SHARE;
    IF NOT FOUND OR v_agreement.state <> 'active' THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'agreement_missing_or_inactive');
    END IF;
    IF v_agreement.instrument_id <> p_instrument_id
       OR v_agreement.profile_revision <> p_profile_revision
       OR p_community_id <> CASE p_direction
            WHEN 'a_to_b' THEN v_agreement.community_a
            ELSE v_agreement.community_b
          END THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'agreement_instrument_or_direction_mismatch');
    END IF;

    SELECT * INTO v_cap
      FROM ncf.community_instrument_cap
     WHERE community_id = p_community_id
       AND instrument_id = p_instrument_id
     FOR UPDATE;
    IF NOT FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'missing_community_instrument_cap');
    END IF;

    SELECT * INTO v_slice
      FROM ncf.reservation_slice
     WHERE reservation_id = p_reservation_id
     FOR UPDATE;
    IF NOT FOUND
       OR v_slice.agreement_id <> p_agreement_id
       OR v_slice.community_id <> p_community_id
       OR v_slice.instrument_id <> p_instrument_id
       OR v_slice.direction <> p_direction
       OR v_slice.shard_id <> p_shard_id
       OR v_slice.epoch <> p_epoch
       OR v_slice.state <> 'active' THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'reservation_slice_missing_inactive_or_mismatched');
    END IF;

    IF v_cap.outstanding_units + p_amount_units > v_cap.allocated_units
       OR v_cap.outstanding_units + p_amount_units > v_cap.cap_units THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'community_instrument_cap_exceeded');
    END IF;
    IF v_slice.used_units + p_amount_units > v_slice.reserved_units THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'reservation_slice_exhausted');
    END IF;

    UPDATE ncf.community_instrument_cap
       SET outstanding_units = outstanding_units + p_amount_units
     WHERE community_id = p_community_id AND instrument_id = p_instrument_id;
    UPDATE ncf.reservation_slice
       SET used_units = used_units + p_amount_units
     WHERE reservation_id = p_reservation_id;

    INSERT INTO ncf.obligation(
        canonical_work_digest, principal_id, community_id, instrument_id, profile_revision,
        amount_units, agreement_id, direction, reservation_id, shard_id, epoch,
        state, receipt_json
    ) VALUES (
        p_canonical_work_digest, p_principal_id, p_community_id, p_instrument_id, p_profile_revision,
        p_amount_units, p_agreement_id, p_direction, p_reservation_id, p_shard_id, p_epoch,
        'outstanding', '{}'::jsonb
    ) RETURNING obligation_id INTO v_obligation_id;

    v_new_receipt := jsonb_build_object(
        'receipt_version', 'ncf.obligation.receipt.v1',
        'obligation_id', v_obligation_id,
        'request_id', p_request_id,
        'canonical_work_digest_sha256', encode(p_canonical_work_digest, 'hex'),
        'principal_id', p_principal_id,
        'community_id', p_community_id,
        'instrument_id', p_instrument_id,
        'profile_revision', p_profile_revision,
        'amount_units', p_amount_units::numeric(78,0)::text,
        'agreement_id', p_agreement_id,
        'direction', p_direction,
        'reservation_id', p_reservation_id,
        'shard_id', p_shard_id,
        'epoch', p_epoch,
        'verifier_version', p_verifier_version,
        'verifier_receipt_digest_sha256', encode(p_verifier_receipt_digest, 'hex')
    );
    UPDATE ncf.obligation SET receipt_json = v_new_receipt WHERE obligation_id = v_obligation_id;

    v_outbox_payload := jsonb_build_object(
        'event_version', 'ncf.obligation.accepted.v1',
        'obligation_id', v_obligation_id,
        'receipt', v_new_receipt
    );
    INSERT INTO ncf.holochain_outbox(
        obligation_id, event_type, payload_json, payload_sha256, state
    ) VALUES (
        v_obligation_id, 'obligation_accepted', v_outbox_payload,
        sha256(convert_to(v_outbox_payload::text, 'UTF8')), 'pending'
    );

    INSERT INTO ncf.request_binding(
        request_id, payload_digest, principal_id, community_id, instrument_id,
        profile_revision, amount_units, canonical_work_digest, agreement_id, direction,
        reservation_id, shard_id, epoch, verifier_version, verifier_receipt_digest,
        obligation_id, decision, receipt_json
    ) VALUES (
        p_request_id, p_payload_digest, p_principal_id, p_community_id, p_instrument_id,
        p_profile_revision, p_amount_units, p_canonical_work_digest, p_agreement_id, p_direction,
        p_reservation_id, p_shard_id, p_epoch, p_verifier_version, p_verifier_receipt_digest,
        v_obligation_id, 'accepted', v_new_receipt
    );

    RETURN jsonb_build_object(
        'decision', 'accepted',
        'obligation_id', v_obligation_id,
        'receipt', v_new_receipt
    );
EXCEPTION WHEN unique_violation THEN
    -- In PL/pgSQL the failing block is rolled back before this handler runs, so any
    -- provisional counter/slice/outbox changes above are undone on a request-ID race.
    SELECT rb, ob.receipt_json
      INTO v_binding, v_receipt
      FROM ncf.request_binding rb
      JOIN ncf.obligation ob ON ob.obligation_id = rb.obligation_id
     WHERE rb.request_id = p_request_id;
    IF FOUND THEN
        IF v_binding.payload_digest = p_payload_digest
           AND v_binding.principal_id = p_principal_id
           AND v_binding.community_id = p_community_id
           AND v_binding.instrument_id = p_instrument_id
           AND v_binding.profile_revision = p_profile_revision
           AND v_binding.amount_units = p_amount_units
           AND v_binding.canonical_work_digest = p_canonical_work_digest
           AND v_binding.agreement_id = p_agreement_id
           AND v_binding.direction = p_direction
           AND v_binding.reservation_id = p_reservation_id
           AND v_binding.shard_id = p_shard_id
           AND v_binding.epoch = p_epoch
           AND v_binding.verifier_version = p_verifier_version
           AND v_binding.verifier_receipt_digest = p_verifier_receipt_digest THEN
            RETURN jsonb_build_object('decision', 'replay', 'obligation_id', v_binding.obligation_id, 'receipt', v_receipt);
        END IF;
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'request_id_payload_or_evidence_conflict');
    END IF;
    RAISE;
END;
$$;

CREATE OR REPLACE FUNCTION ncf.settle_obligation(
    p_settlement_id uuid,
    p_obligation_id bigint,
    p_settlement_digest bytea
) RETURNS jsonb
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
DECLARE
    v_before ncf.obligation%ROWTYPE;
    v_obligation ncf.obligation%ROWTYPE;
    v_agreement ncf.clearing_agreement%ROWTYPE;
    v_cap ncf.community_instrument_cap%ROWTYPE;
    v_slice ncf.reservation_slice%ROWTYPE;
    v_existing ncf.settlement_receipt%ROWTYPE;
    v_receipt jsonb;
    v_payload jsonb;
BEGIN
    IF p_settlement_id IS NULL OR p_obligation_id IS NULL
       OR p_settlement_digest IS NULL OR octet_length(p_settlement_digest) <> 32 THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'invalid_settlement_input');
    END IF;

    PERFORM pg_advisory_xact_lock(hashtextextended('ncf-settlement-id:' || p_settlement_id::text, 0));
    SELECT * INTO v_existing FROM ncf.settlement_receipt WHERE settlement_id = p_settlement_id;
    IF FOUND THEN
        IF v_existing.obligation_id = p_obligation_id
           AND v_existing.settlement_digest = p_settlement_digest THEN
            RETURN jsonb_build_object('decision', 'replay', 'obligation_id', p_obligation_id, 'receipt', v_existing.receipt_json);
        END IF;
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'settlement_id_conflict');
    END IF;

    SELECT * INTO v_before FROM ncf.obligation WHERE obligation_id = p_obligation_id;
    IF NOT FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'obligation_not_found');
    END IF;

    -- Follow the same lock order as admission to avoid cap/slice/obligation deadlocks.
    SELECT * INTO v_agreement
      FROM ncf.clearing_agreement WHERE agreement_id = v_before.agreement_id FOR SHARE;
    SELECT * INTO v_cap
      FROM ncf.community_instrument_cap
     WHERE community_id = v_before.community_id AND instrument_id = v_before.instrument_id
     FOR UPDATE;
    SELECT * INTO v_slice
      FROM ncf.reservation_slice WHERE reservation_id = v_before.reservation_id FOR UPDATE;
    SELECT * INTO v_obligation
      FROM ncf.obligation WHERE obligation_id = p_obligation_id FOR UPDATE;

    SELECT * INTO v_existing FROM ncf.settlement_receipt WHERE obligation_id = p_obligation_id;
    IF FOUND THEN
        IF v_existing.settlement_id = p_settlement_id
           AND v_existing.settlement_digest = p_settlement_digest THEN
            RETURN jsonb_build_object('decision', 'replay', 'obligation_id', p_obligation_id, 'receipt', v_existing.receipt_json);
        END IF;
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'obligation_already_settled');
    END IF;
    IF v_obligation.state <> 'outstanding' THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'obligation_not_outstanding');
    END IF;
    IF v_cap.community_id IS NULL OR v_slice.reservation_id IS NULL
       OR v_cap.outstanding_units < v_obligation.amount_units
       OR v_slice.used_units < v_obligation.amount_units THEN
        RAISE EXCEPTION 'NCF ledger counters inconsistent before settlement of obligation %', p_obligation_id;
    END IF;

    UPDATE ncf.community_instrument_cap
       SET outstanding_units = outstanding_units - v_obligation.amount_units
     WHERE community_id = v_obligation.community_id AND instrument_id = v_obligation.instrument_id;
    UPDATE ncf.reservation_slice
       SET used_units = used_units - v_obligation.amount_units
     WHERE reservation_id = v_obligation.reservation_id;
    UPDATE ncf.obligation
       SET state = 'settled', settled_at = clock_timestamp()
     WHERE obligation_id = p_obligation_id;

    v_receipt := jsonb_build_object(
        'receipt_version', 'ncf.settlement.receipt.v1',
        'settlement_id', p_settlement_id,
        'obligation_id', p_obligation_id,
        'settlement_digest_sha256', encode(p_settlement_digest, 'hex'),
        'instrument_id', v_obligation.instrument_id,
        'amount_units', v_obligation.amount_units::text,
        'community_id', v_obligation.community_id,
        'direction', v_obligation.direction
    );
    INSERT INTO ncf.settlement_receipt(settlement_id, obligation_id, settlement_digest, receipt_json)
    VALUES (p_settlement_id, p_obligation_id, p_settlement_digest, v_receipt);

    v_payload := jsonb_build_object(
        'event_version', 'ncf.obligation.settled.v1',
        'obligation_id', p_obligation_id,
        'receipt', v_receipt
    );
    INSERT INTO ncf.holochain_outbox(obligation_id, event_type, payload_json, payload_sha256, state)
    VALUES (
        p_obligation_id, 'obligation_settled', v_payload,
        sha256(convert_to(v_payload::text, 'UTF8')), 'pending'
    );

    RETURN jsonb_build_object('decision', 'accepted', 'obligation_id', p_obligation_id, 'receipt', v_receipt);
EXCEPTION WHEN unique_violation THEN
    SELECT * INTO v_existing FROM ncf.settlement_receipt WHERE settlement_id = p_settlement_id;
    IF FOUND THEN
        IF v_existing.obligation_id = p_obligation_id
           AND v_existing.settlement_digest = p_settlement_digest THEN
            RETURN jsonb_build_object('decision', 'replay', 'obligation_id', p_obligation_id, 'receipt', v_existing.receipt_json);
        END IF;
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'settlement_id_conflict');
    END IF;
    RAISE;
END;
$$;


CREATE OR REPLACE FUNCTION ncf.lease_outbox_batch(
    p_worker text,
    p_batch_size integer,
    p_lease_seconds integer
) RETURNS SETOF ncf.holochain_outbox
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
BEGIN
    IF p_worker IS NULL OR length(btrim(p_worker)) = 0
       OR p_batch_size IS NULL OR p_batch_size < 1 OR p_batch_size > 100
       OR p_lease_seconds IS NULL OR p_lease_seconds < 1 OR p_lease_seconds > 600 THEN
        RAISE EXCEPTION 'invalid outbox lease parameters' USING ERRCODE = '22023';
    END IF;

    RETURN QUERY
    WITH selected AS (
        SELECT outbox_id
          FROM ncf.holochain_outbox
         WHERE ((state IN ('pending', 'retry') AND next_attempt_at <= clock_timestamp())
             OR (state = 'leased' AND lease_until <= clock_timestamp()))
         ORDER BY next_attempt_at, outbox_id
         FOR UPDATE SKIP LOCKED
         LIMIT p_batch_size
    )
    UPDATE ncf.holochain_outbox AS o
       SET state = 'leased',
           attempt_count = o.attempt_count + 1,
           lease_owner = p_worker,
           lease_until = clock_timestamp() + make_interval(secs => p_lease_seconds)
      FROM selected
     WHERE o.outbox_id = selected.outbox_id
    RETURNING o.*;
END;
$;

CREATE OR REPLACE FUNCTION ncf.mark_outbox_retry(
    p_outbox_id bigint,
    p_worker text,
    p_error text,
    p_delay_seconds integer
) RETURNS jsonb
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
DECLARE
    v_outbox ncf.holochain_outbox%ROWTYPE;
    v_state text;
BEGIN
    SELECT * INTO v_outbox FROM ncf.holochain_outbox
     WHERE outbox_id = p_outbox_id FOR UPDATE;
    IF NOT FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'outbox_not_found');
    END IF;
    IF v_outbox.state <> 'leased' OR v_outbox.lease_owner <> p_worker THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'outbox_lease_not_owned');
    END IF;
    IF p_delay_seconds IS NULL OR p_delay_seconds < 0 OR p_delay_seconds > 86400 THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'invalid_retry_delay');
    END IF;

    v_state := CASE WHEN v_outbox.attempt_count >= 20 THEN 'dead' ELSE 'retry' END;
    UPDATE ncf.holochain_outbox
       SET state = v_state,
           next_attempt_at = clock_timestamp() + make_interval(secs => p_delay_seconds),
           lease_owner = NULL,
           lease_until = NULL,
           last_error = left(coalesce(p_error, 'unspecified dispatch error'), 2000)
     WHERE outbox_id = p_outbox_id;

    RETURN jsonb_build_object('decision', v_state, 'outbox_id', p_outbox_id, 'attempt_count', v_outbox.attempt_count);
END;
$;

CREATE OR REPLACE FUNCTION ncf.record_holochain_dispatch(
    p_outbox_id bigint,
    p_worker text,
    p_dna_hash text,
    p_action_hash text,
    p_author_agent text,
    p_entry_hash text
) RETURNS jsonb
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
DECLARE
    v_outbox ncf.holochain_outbox%ROWTYPE;
    v_observation_state text := 'authored';
BEGIN
    IF p_dna_hash IS NULL OR length(btrim(p_dna_hash)) = 0
       OR p_action_hash IS NULL OR length(btrim(p_action_hash)) = 0
       OR p_author_agent IS NULL OR length(btrim(p_author_agent)) = 0 THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'invalid_holochain_action_identity');
    END IF;
    SELECT * INTO v_outbox FROM ncf.holochain_outbox
     WHERE outbox_id = p_outbox_id FOR UPDATE;
    IF NOT FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'outbox_not_found');
    END IF;

    IF v_outbox.state = 'published' THEN
        INSERT INTO ncf.holochain_action_observation(
            dna_hash, action_hash, outbox_id, author_agent, entry_hash,
            validation_receipt_count, observation_state
        ) VALUES (p_dna_hash, p_action_hash, p_outbox_id, p_author_agent, p_entry_hash, 0, v_observation_state)
        ON CONFLICT (dna_hash, action_hash) DO NOTHING;
        IF v_outbox.holochain_action_hash = p_action_hash THEN
            RETURN jsonb_build_object('decision', 'replay', 'outbox_id', p_outbox_id, 'action_hash', p_action_hash);
        END IF;
        RETURN jsonb_build_object(
            'decision', 'duplicate_action_observed',
            'outbox_id', p_outbox_id,
            'canonical_action_hash', v_outbox.holochain_action_hash,
            'additional_action_hash', p_action_hash
        );
    END IF;

    IF v_outbox.state <> 'leased' OR v_outbox.lease_owner <> p_worker THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'outbox_lease_not_owned');
    END IF;

    UPDATE ncf.holochain_outbox
       SET state = 'published',
           holochain_action_hash = p_action_hash,
           published_at = coalesce(published_at, clock_timestamp()),
           lease_owner = NULL,
           lease_until = NULL,
           last_error = NULL
     WHERE outbox_id = p_outbox_id;
    INSERT INTO ncf.holochain_action_observation(
        dna_hash, action_hash, outbox_id, author_agent, entry_hash,
        validation_receipt_count, observation_state
    ) VALUES (p_dna_hash, p_action_hash, p_outbox_id, p_author_agent, p_entry_hash, 0, v_observation_state)
    ON CONFLICT (dna_hash, action_hash) DO NOTHING;

    RETURN jsonb_build_object('decision', 'published', 'outbox_id', p_outbox_id, 'action_hash', p_action_hash);
END;
$;

CREATE OR REPLACE FUNCTION ncf.record_validation_receipts(
    p_dna_hash text,
    p_action_hash text,
    p_receipt_count integer,
    p_required_count integer
) RETURNS jsonb
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, ncf
AS $
DECLARE
    v_observation ncf.holochain_action_observation%ROWTYPE;
    v_new_count integer;
    v_state text;
BEGIN
    IF p_receipt_count IS NULL OR p_receipt_count < 0
       OR p_required_count IS NULL OR p_required_count < 1 THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'invalid_validation_receipt_threshold');
    END IF;
    SELECT * INTO v_observation
      FROM ncf.holochain_action_observation
     WHERE dna_hash = p_dna_hash AND action_hash = p_action_hash
     FOR UPDATE;
    IF NOT FOUND THEN
        RETURN jsonb_build_object('decision', 'rejected', 'reason', 'action_observation_not_found');
    END IF;

    v_new_count := greatest(v_observation.validation_receipt_count, p_receipt_count);
    v_state := CASE
      WHEN v_new_count >= p_required_count THEN 'receipts_observed'
      ELSE 'unresolved'
    END;
    UPDATE ncf.holochain_action_observation
       SET validation_receipt_count = v_new_count,
           observation_state = v_state,
           observed_at = clock_timestamp()
     WHERE dna_hash = p_dna_hash AND action_hash = p_action_hash;

    RETURN jsonb_build_object(
        'decision', v_state,
        'dna_hash', p_dna_hash,
        'action_hash', p_action_hash,
        'validation_receipt_count', v_new_count,
        'required_count', p_required_count
    );
END;
$;

REVOKE EXECUTE ON FUNCTION ncf.allocate_reservation_slice(uuid, uuid, text, text, text, text, bigint, numeric, bytea) FROM PUBLIC;
REVOKE EXECUTE ON FUNCTION ncf.admit_obligation(uuid, bytea, text, text, text, integer, numeric, bytea, uuid, text, uuid, text, bigint, text, bytea) FROM PUBLIC;
REVOKE EXECUTE ON FUNCTION ncf.settle_obligation(uuid, bigint, bytea) FROM PUBLIC;
GRANT EXECUTE ON FUNCTION ncf.allocate_reservation_slice(uuid, uuid, text, text, text, text, bigint, numeric, bytea) TO ncf_app;
GRANT EXECUTE ON FUNCTION ncf.admit_obligation(uuid, bytea, text, text, text, integer, numeric, bytea, uuid, text, uuid, text, bigint, text, bytea) TO ncf_app;
GRANT EXECUTE ON FUNCTION ncf.settle_obligation(uuid, bigint, bytea) TO ncf_app;
GRANT EXECUTE ON FUNCTION ncf.lease_outbox_batch(text, integer, integer) TO ncf_app;
GRANT EXECUTE ON FUNCTION ncf.mark_outbox_retry(bigint, text, text, integer) TO ncf_app;
GRANT EXECUTE ON FUNCTION ncf.record_holochain_dispatch(bigint, text, text, text, text, text) TO ncf_app;
GRANT EXECUTE ON FUNCTION ncf.record_validation_receipts(text, text, integer, integer) TO ncf_app;

COMMIT;
