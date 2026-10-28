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
    v_overshoot jsonb;
    v_settle jsonb;
    v_settle_replay jsonb;
    v_obligation_id bigint;
    v_count bigint;
    v_units numeric;
    v_used numeric;
    v_allocated numeric;
    v_outstanding numeric;
    v_slice_used numeric;
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

    SELECT count(*), COALESCE(sum(amount_units),0)
      INTO v_count, v_units FROM ncf.obligation
     WHERE instrument_id='NCF-TEST-UNIT';
    SELECT allocated_units, outstanding_units INTO v_allocated, v_outstanding
      FROM ncf.community_instrument_cap
     WHERE community_id='community-a' AND instrument_id='NCF-TEST-UNIT';
    SELECT used_units INTO v_slice_used FROM ncf.reservation_slice
     WHERE reservation_id='bb000000-0000-0000-0000-000000000001';
    IF v_count <> 1 OR v_units <> 60 OR v_allocated <> 100
       OR v_outstanding <> 60 OR v_slice_used <> 60 THEN
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
    IF v_outstanding <> 0 OR v_slice_used <> 0 THEN
      RAISE EXCEPTION 'settlement did not return used capacity: outstanding %, slice-used %', v_outstanding, v_slice_used;
    END IF;
    IF (SELECT count(*) FROM ncf.holochain_outbox WHERE obligation_id=v_obligation_id) <> 2 THEN
      RAISE EXCEPTION 'expected accepted and settled outbox events';
    END IF;

    BEGIN
      UPDATE ncf.instrument_identity SET unit_scale=1 WHERE instrument_id='NCF-TEST-UNIT';
      RAISE EXCEPTION 'immutable identity trigger did not fire';
    EXCEPTION WHEN SQLSTATE '55000' THEN
      NULL;
    END;
END;
$$;

SELECT 'NCF_POSTGRES_SMOKE_PASS' AS result;
