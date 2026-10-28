-- One invocation equals one new obligation attempt. Post-run reconciliation
-- requires one accepted canonical obligation for every successful pgbench txn.
\set r1 random(1, 2147483647)
\set r2 random(1, 2147483647)

SELECT ncf.admit_obligation(
  md5('ncf-pgbench:' || :client_id::text || ':' || :r1::text || ':' || :r2::text)::uuid,
  sha256(convert_to('payload:' || :client_id::text || ':' || :r1::text || ':' || :r2::text, 'UTF8')),
  'pgbench-client-' || :client_id::text,
  'pgbench-community-a',
  'NCF-PGBENCH-UNIT',
  1,
  1,
  sha256(convert_to('work:' || :client_id::text || ':' || :r1::text || ':' || :r2::text, 'UTF8')),
  'aa000000-0000-0000-0000-000000000010'::uuid,
  'a_to_b',
  'bb000000-0000-0000-0000-000000000010'::uuid,
  'pgbench-shard-1',
  1,
  'pgbench-verifier-v1',
  sha256(convert_to('verifier:' || :client_id::text || ':' || :r1::text || ':' || :r2::text, 'UTF8'))
)->>'decision';
