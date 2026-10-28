-- Isolated workload fixture for pgbench_admit.sql.
-- Use a disposable PostgreSQL database; the row keys are intentionally fixed.
INSERT INTO ncf.instrument_identity(
  instrument_id, issuer_community_id, instrument_family, denomination_code, unit_scale, transferable
) VALUES ('NCF-PGBENCH-UNIT', 'pgbench-community-a', 'transferable', 'UNIT', 0, true);

INSERT INTO ncf.instrument_profile_revision(instrument_id, profile_revision, profile_digest, policy)
VALUES ('NCF-PGBENCH-UNIT', 1, sha256(convert_to('ncf-pgbench-profile-v1','UTF8')), '{"test_profile":"pgbench"}'::jsonb);

INSERT INTO ncf.clearing_agreement(
  agreement_id, community_a, community_b, instrument_id, profile_revision, state
) VALUES (
  'aa000000-0000-0000-0000-000000000010',
  'pgbench-community-a', 'pgbench-community-b', 'NCF-PGBENCH-UNIT', 1, 'active'
);

INSERT INTO ncf.community_instrument_cap(community_id, instrument_id, cap_units)
VALUES ('pgbench-community-a', 'NCF-PGBENCH-UNIT', 1000000000000000000000000);

INSERT INTO ncf.agreement_direction_cap(
  agreement_id, direction, community_id, instrument_id, cap_units, allocated_units, outstanding_units
) VALUES
 ('aa000000-0000-0000-0000-000000000010', 'a_to_b', 'pgbench-community-a', 'NCF-PGBENCH-UNIT', 1000000000000000000000000, 0, 0),
 ('aa000000-0000-0000-0000-000000000010', 'b_to_a', 'pgbench-community-b', 'NCF-PGBENCH-UNIT', 1000000000000000000000000, 0, 0);

SELECT ncf.allocate_reservation_slice(
  'bb000000-0000-0000-0000-000000000010',
  'aa000000-0000-0000-0000-000000000010',
  'pgbench-community-a', 'NCF-PGBENCH-UNIT', 'a_to_b', 'pgbench-shard-1', 1,
  1000000000000000000000000,
  sha256(convert_to('ncf-pgbench-reservation-v1','UTF8'))
);
