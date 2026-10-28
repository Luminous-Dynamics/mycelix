-- Sequential PostgreSQL contract smoke test. Run after 001_schema.sql and 002_functions.sql.
\set ON_ERROR_STOP on

INSERT INTO ncf.instrument_identity(
  instrument_id, issuer_community_id, instrument_family, denomination_code, unit_scale, transferable
) VALUES ('NCF-TEST-UNIT', 'community-a', 'transferable', 'UNIT', 0, true);

INSERT INTO ncf.instrument_profile_revision(instrument_id, profile_revision, profile_digest, policy)
VALUES ('NCF-TEST-UNIT', 1, decode(repeat('1',64),'hex'), '{"test_profile":true}'::jsonb);

INSERT INTO ncf.clearing_agreement(
  agreement_id, community_a, community_b, instrument_id, profile_revision, state
) VALUES (
  'aa000000-0000-0000-0000-000000000001',
  'community-a', 'community-b', 'NCF-TEST-UNIT', 1, 'active'
);

INSERT INTO ncf.community_instrument_cap(community_id, instrument_id, cap_units)
VALUES ('community-a', 'NCF-TEST-UNIT', 100);

INSERT INTO ncf.agreement_direction_cap(
  agreement_id, direction, community_id, instrument_id, cap_units, allocated_units, outstanding_units
) VALUES
 ('aa000000-0000-0000-0000-000000000001', 'a_to_b', 'community-a', 'NCF-TEST-UNIT', 100, 0, 0),
 ('aa000000-0000-0000-0000-000000000001', 'b_to_a', 'community-b', 'NCF-TEST-UNIT', 100, 0, 0);

SELECT ncf.allocate_reservation_slice(
  'bb000000-0000-0000-0000-000000000001',
  'aa000000-0000-0000-0000-000000000001',
  'community-a', 'NCF-TEST-UNIT', 'a_to_b', 'shard-a', 1, 100,
  decode(repeat('2',64),'hex')
);

DO $$
DECLARE
    v_result jsonb;
    v_replay jsonb;
    v_conflict jsonb;
    v_alias jsonb;
  v_alias_wrong_shard jsonb;
    v_overshoot jsonb;
    v_settle jsonb;
    v_settle_replay jsonb;
    v_fractional jsonb;
    v_nan jsonb;
    v_retry jsonb;
    v_dispatch jsonb;
    v_dispatch_replay jsonb;
    v_duplicate_dispatch jsonb;
    v_receipts jsonb;
    v_receipts_high_water jsonb;
    v_outbox_id bigint;
    v_second_outbox_id bigint;
    v_lease_seen bigint;
    v_observation_count bigint;
    v_obligation_id bigint;
    v_count bigint;
    v_units numeric;
    v_used numeric;
    v_allocated numeric;
    v_outstanding numeric;
    v_slice_used numeric;
    v_edge_allocated numeric;
    v_edge_outstanding numeric;
BEGIN
    v_result := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000001',
      decode(repeat('a',64),'hex'),
      'alice','community-a','NCF-TEST-UNIT',1,60,
      decode(repeat('b',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('c',64),'hex')
    );
    IF v_result->>'decision' <> 'accepted' THEN
      RAISE EXCEPTION 'expected accepted first obligation, got %', v_result;
    END IF;
    v_obligation_id := (v_result->>'obligation_id')::bigint;

    v_replay := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000001',
      decode(repeat('a',64),'hex'),
      'alice','community-a','NCF-TEST-UNIT',1,60,
      decode(repeat('b',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('c',64),'hex')
    );
    IF v_replay->>'decision' <> 'replay' THEN
      RAISE EXCEPTION 'expected exact retry replay, got %', v_replay;
    END IF;

    v_conflict := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000001',
      decode(repeat('d',64),'hex'),
      'alice','community-a','NCF-TEST-UNIT',1,60,
      decode(repeat('e',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('c',64),'hex')
    );
    IF v_conflict->>'decision' <> 'rejected' THEN
      RAISE EXCEPTION 'expected changed-payload request-ID conflict, got %', v_conflict;
    END IF;

    v_alias := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000002',
      decode(repeat('f',64),'hex'),
      'alice','community-a','NCF-TEST-UNIT',1,60,
      decode(repeat('b',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v2',decode(repeat('9',64),'hex')
    );
    IF v_alias->>'decision' <> 'replay'
       OR (v_alias->>'obligation_id')::bigint <> v_obligation_id THEN
      RAISE EXCEPTION 'expected canonical-work alias replay, got %', v_alias;
    END IF;

    -- A canonical-work alias from another shard must not acquire a new replay binding.
    v_alias_wrong_shard := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000005',
      decode(repeat('e',64),'hex'),
      'alice','community-a','NCF-TEST-UNIT',1,60,
      decode(repeat('b',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','non-home-shard',1,
      'smoke-verifier-v2',decode(repeat('8',64),'hex')
    );
    IF v_alias_wrong_shard->>'decision' <> 'rejected'
       OR v_alias_wrong_shard->>'reason' <> 'canonical_work_identity_or_home_shard_conflict' THEN
      RAISE EXCEPTION 'expected cross-shard canonical-work alias rejection, got %', v_alias_wrong_shard;
    END IF;

    v_overshoot := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000003',
      decode(repeat('8',64),'hex'),
      'bob','community-a','NCF-TEST-UNIT',1,50,
      decode(repeat('7',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('6',64),'hex')
    );
    IF v_overshoot->>'decision' <> 'rejected' THEN
      RAISE EXCEPTION 'expected cap/slice rejection, got %', v_overshoot;
    END IF;

    v_fractional := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000004',
      decode(repeat('4',64),'hex'),
      'bob','community-a','NCF-TEST-UNIT',1,1.5,
      decode(repeat('3',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('6',64),'hex')
    );
    IF v_fractional->>'decision' <> 'rejected'
       OR v_fractional->>'reason' <> 'invalid_or_unverified_input' THEN
      RAISE EXCEPTION 'fractional economic units must fail closed, got %', v_fractional;
    END IF;

    v_nan := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000005',
      decode(repeat('5',64),'hex'),
      'bob','community-a','NCF-TEST-UNIT',1,'NaN'::numeric,
      decode(repeat('4',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('7',64),'hex')
    );
    IF v_nan->>'decision' <> 'rejected'
       OR v_nan->>'reason' <> 'invalid_or_unverified_input' THEN
      RAISE EXCEPTION 'non-finite economic units must fail closed, got %', v_nan;
    END IF;

    SELECT count(*), COALESCE(sum(amount_units),0)
      INTO v_count, v_units FROM ncf.obligation
     WHERE instrument_id='NCF-TEST-UNIT';
    SELECT allocated_units, outstanding_units INTO v_allocated, v_outstanding
      FROM ncf.community_instrument_cap
     WHERE community_id='community-a' AND instrument_id='NCF-TEST-UNIT';
    SELECT used_units INTO v_slice_used FROM ncf.reservation_slice
     WHERE reservation_id='bb000000-0000-0000-0000-000000000001';
    SELECT allocated_units, outstanding_units INTO v_edge_allocated, v_edge_outstanding
      FROM ncf.agreement_direction_cap
     WHERE agreement_id='aa000000-0000-0000-0000-000000000001' AND direction='a_to_b';
    IF v_count <> 1 OR v_units <> 60 OR v_allocated <> 100
       OR v_outstanding <> 60 OR v_slice_used <> 60 OR v_edge_allocated <> 100 OR v_edge_outstanding <> 60 THEN
      RAISE EXCEPTION 'admission conservation failure: count %, units %, allocation %, outstanding %, slice-used %',
        v_count, v_units, v_allocated, v_outstanding, v_slice_used;
    END IF;
    IF (SELECT count(*) FROM ncf.holochain_outbox WHERE event_type='obligation_accepted') <> 1 THEN
      RAISE EXCEPTION 'expected one outbox event for one canonical obligation';
    END IF;

    v_settle := ncf.settle_obligation(
      'cc000000-0000-0000-0000-000000000001',
      v_obligation_id,
      decode(repeat('5',64),'hex')
    );
    IF v_settle->>'decision' <> 'accepted' THEN
      RAISE EXCEPTION 'expected settlement, got %', v_settle;
    END IF;
    v_settle_replay := ncf.settle_obligation(
      'cc000000-0000-0000-0000-000000000001',
      v_obligation_id,
      decode(repeat('5',64),'hex')
    );
    IF v_settle_replay->>'decision' <> 'replay' THEN
      RAISE EXCEPTION 'expected settlement replay, got %', v_settle_replay;
    END IF;

    SELECT outstanding_units INTO v_outstanding
      FROM ncf.community_instrument_cap
     WHERE community_id='community-a' AND instrument_id='NCF-TEST-UNIT';
    SELECT used_units INTO v_slice_used FROM ncf.reservation_slice
     WHERE reservation_id='bb000000-0000-0000-0000-000000000001';
    SELECT outstanding_units INTO v_edge_outstanding FROM ncf.agreement_direction_cap
     WHERE agreement_id='aa000000-0000-0000-0000-000000000001' AND direction='a_to_b';
    IF v_outstanding <> 0 OR v_slice_used <> 0 OR v_edge_outstanding <> 0 THEN
      RAISE EXCEPTION 'settlement did not return used capacity: outstanding %, slice-used %', v_outstanding, v_slice_used;
    END IF;
    IF (SELECT count(*) FROM ncf.holochain_outbox WHERE obligation_id=v_obligation_id) <> 2 THEN
      RAISE EXCEPTION 'expected accepted and settled outbox events';
    END IF;

    SELECT leased.outbox_id INTO v_outbox_id
      FROM ncf.lease_outbox_batch('smoke-worker', 1, 30) AS leased
     ORDER BY leased.outbox_id LIMIT 1;
    IF v_outbox_id IS NULL THEN
      RAISE EXCEPTION 'expected outbox lease';
    END IF;
    v_retry := ncf.mark_outbox_retry(v_outbox_id, 'smoke-worker', 'test lost acknowledgement', 0);
    IF v_retry->>'decision' <> 'retry' THEN
      RAISE EXCEPTION 'expected retry state after transient delivery failure, got %', v_retry;
    END IF;

    SELECT count(*) INTO v_lease_seen
      FROM ncf.lease_outbox_batch('smoke-worker-2', 10, 30) AS leased
     WHERE leased.outbox_id = v_outbox_id;
    IF v_lease_seen <> 1 THEN
      RAISE EXCEPTION 'expected retried outbox row to be leased again';
    END IF;

    v_dispatch := ncf.record_holochain_dispatch(
      v_outbox_id, 'smoke-worker-2', 'dna-smoke', 'action-smoke-1', 'agent-smoke', 'entry-smoke'
    );
    IF v_dispatch->>'decision' <> 'published' THEN
      RAISE EXCEPTION 'expected Holochain authored dispatch, got %', v_dispatch;
    END IF;
    v_dispatch_replay := ncf.record_holochain_dispatch(
      v_outbox_id, 'another-worker', 'dna-smoke', 'action-smoke-1', 'agent-smoke', 'entry-smoke'
    );
    IF v_dispatch_replay->>'decision' <> 'replay' THEN
      RAISE EXCEPTION 'exact dispatch acknowledgement must replay, got %', v_dispatch_replay;
    END IF;
    v_duplicate_dispatch := ncf.record_holochain_dispatch(
      v_outbox_id, 'another-worker', 'dna-smoke', 'action-smoke-duplicate', 'agent-smoke', 'entry-smoke'
    );
    IF v_duplicate_dispatch->>'decision' <> 'duplicate_action_observed' THEN
      RAISE EXCEPTION 'second action hash must be recorded as duplicate evidence, not new SQL obligation, got %', v_duplicate_dispatch;
    END IF;

    v_receipts := ncf.record_validation_receipts('dna-smoke', 'action-smoke-1', 1, 3);
    IF v_receipts->>'decision' <> 'unresolved' THEN
      RAISE EXCEPTION 'sub-threshold DHT receipts must not imply replication readiness, got %', v_receipts;
    END IF;
    v_receipts_high_water := ncf.record_validation_receipts('dna-smoke', 'action-smoke-1', 3, 3);
    IF v_receipts_high_water->>'decision' <> 'receipts_observed' THEN
      RAISE EXCEPTION 'receipt threshold should be observed at count 3, got %', v_receipts_high_water;
    END IF;
    v_receipts_high_water := ncf.record_validation_receipts('dna-smoke', 'action-smoke-1', 2, 3);
    IF v_receipts_high_water->>'decision' <> 'receipts_observed'
       OR (v_receipts_high_water->>'validation_receipt_count')::integer <> 3 THEN
      RAISE EXCEPTION 'receipt count must be monotonic, got %', v_receipts_high_water;
    END IF;

    SELECT outbox_id INTO v_second_outbox_id
      FROM ncf.holochain_outbox
     WHERE obligation_id = v_obligation_id AND outbox_id <> v_outbox_id
     ORDER BY outbox_id LIMIT 1;
    IF v_second_outbox_id IS NULL THEN
      RAISE EXCEPTION 'expected second outbox event for settlement';
    END IF;
    v_dispatch := ncf.record_holochain_dispatch(
      v_second_outbox_id, 'smoke-worker-2', 'dna-smoke', 'action-smoke-settle', 'agent-smoke', 'entry-smoke-settle'
    );
    IF v_dispatch->>'decision' <> 'published' THEN
      RAISE EXCEPTION 'expected settlement event dispatch, got %', v_dispatch;
    END IF;
    SELECT count(*) INTO v_observation_count
      FROM ncf.holochain_action_observation
     WHERE outbox_id IN (
       SELECT outbox_id FROM ncf.holochain_outbox WHERE obligation_id = v_obligation_id
     );
    IF v_observation_count <> 3 THEN
      RAISE EXCEPTION 'expected three action observations (two accepted-event attempts and one settlement), got %', v_observation_count;
    END IF;
    IF (SELECT count(*) FROM ncf.obligation WHERE instrument_id='NCF-TEST-UNIT') <> 1
       OR (SELECT outstanding_units FROM ncf.community_instrument_cap WHERE community_id='community-a' AND instrument_id='NCF-TEST-UNIT') <> 0 THEN
      RAISE EXCEPTION 'outbox retries must not create or settle a second SQL obligation';
    END IF;

    BEGIN
      UPDATE ncf.instrument_identity SET unit_scale=1 WHERE instrument_id='NCF-TEST-UNIT';
      RAISE EXCEPTION 'immutable identity trigger did not fire';
    EXCEPTION WHEN SQLSTATE '55000' THEN
      NULL;
    END;
END;
$$;


-- Application sessions must call audited transaction APIs and cannot mutate ledger rows.
SET ROLE ncf_app;
DO $$
DECLARE
    v_read_only_replay jsonb;
    v_denied boolean := false;
BEGIN
    -- The public online API must not mutate Holochain dispatch or receipt observations.
    IF NOT has_function_privilege('ncf_app', 'ncf.admit_obligation(uuid,bytea,text,text,text,integer,numeric,bytea,uuid,text,uuid,text,bigint,text,bytea)', 'EXECUTE')
       OR NOT has_function_privilege('ncf_app', 'ncf.settle_obligation(uuid,bigint,bytea)', 'EXECUTE') THEN
      RAISE EXCEPTION 'ncf_app is missing an intended economic function grant';
    END IF;
    IF has_function_privilege('ncf_app', 'ncf.lease_outbox_batch(text,integer,integer)', 'EXECUTE')
       OR has_function_privilege('ncf_app', 'ncf.mark_outbox_retry(bigint,text,text,integer)', 'EXECUTE')
       OR has_function_privilege('ncf_app', 'ncf.record_holochain_dispatch(bigint,text,text,text,text,text)', 'EXECUTE')
       OR has_function_privilege('ncf_app', 'ncf.record_validation_receipts(text,text,integer,integer)', 'EXECUTE') THEN
      RAISE EXCEPTION 'ncf_app must not control dispatch or DHT validation observations';
    END IF;
    IF NOT has_function_privilege('ncf_dispatcher', 'ncf.lease_outbox_batch(text,integer,integer)', 'EXECUTE')
       OR NOT has_function_privilege('ncf_dispatcher', 'ncf.record_holochain_dispatch(bigint,text,text,text,text,text)', 'EXECUTE')
       OR has_function_privilege('ncf_dispatcher', 'ncf.record_validation_receipts(text,text,integer,integer)', 'EXECUTE') THEN
      RAISE EXCEPTION 'ncf_dispatcher capability boundary is incorrect';
    END IF;
    IF NOT has_function_privilege('ncf_observer', 'ncf.record_validation_receipts(text,text,integer,integer)', 'EXECUTE')
       OR has_function_privilege('ncf_observer', 'ncf.admit_obligation(uuid,bytea,text,text,text,integer,numeric,bytea,uuid,text,uuid,text,bigint,text,bytea)', 'EXECUTE')
       OR has_function_privilege('ncf_observer', 'ncf.record_holochain_dispatch(bigint,text,text,text,text,text)', 'EXECUTE') THEN
      RAISE EXCEPTION 'ncf_observer capability boundary is incorrect';
    END IF;

    v_read_only_replay := ncf.admit_obligation(
      '10000000-0000-0000-0000-000000000001',
      decode(repeat('a',64),'hex'),
      'alice','community-a','NCF-TEST-UNIT',1,60,
      decode(repeat('b',64),'hex'),
      'aa000000-0000-0000-0000-000000000001','a_to_b',
      'bb000000-0000-0000-0000-000000000001','shard-a',1,
      'smoke-verifier-v1',decode(repeat('c',64),'hex')
    );
    IF v_read_only_replay->>'decision' <> 'replay' THEN
      RAISE EXCEPTION 'ncf_app must be able to invoke the audited transaction API, got %', v_read_only_replay;
    END IF;

    BEGIN
      UPDATE ncf.obligation SET amount_units=99 WHERE obligation_id=1;
    EXCEPTION WHEN insufficient_privilege THEN
      v_denied := true;
    END;
    IF NOT v_denied THEN
      RAISE EXCEPTION 'ncf_app must not directly mutate the obligation ledger';
    END IF;
END;
$$;
RESET ROLE;

SELECT 'NCF_POSTGRES_SMOKE_PASS' AS result;
