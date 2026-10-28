-- Separate fixture for concurrent PostgreSQL calls; run after 900_smoke_test.sql.
INSERT INTO ncf.instrument_identity(
  instrument_id, issuer_community_id, instrument_family, denomination_code, unit_scale, transferable
) VALUES ('NCF-RACE-UNIT', 'community-a', 'transferable', 'UNIT', 0, true);

INSERT INTO ncf.instrument_profile_revision(instrument_id, profile_revision, profile_digest, policy)
VALUES ('NCF-RACE-UNIT', 1, decode(repeat('1',64),'hex'), '{"test_profile":"concurrency"}'::jsonb);

INSERT INTO ncf.clearing_agreement(
  agreement_id, community_a, community_b, instrument_id, profile_revision, state
) VALUES (
  'aa000000-0000-0000-0000-000000000002',
  'community-a', 'community-b', 'NCF-RACE-UNIT', 1, 'active'
);

INSERT INTO ncf.community_instrument_cap(community_id, instrument_id, cap_units)
VALUES ('community-a', 'NCF-RACE-UNIT', 100);

SELECT ncf.allocate_reservation_slice(
  'bb000000-0000-0000-0000-000000000002',
  'aa000000-0000-0000-0000-000000000002',
  'community-a', 'NCF-RACE-UNIT', 'a_to_b', 'shard-race', 1, 100,
  decode(repeat('2',64),'hex')
);
