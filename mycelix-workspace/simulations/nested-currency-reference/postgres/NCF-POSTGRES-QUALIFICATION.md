# PostgreSQL transactional admission — current qualification record

**Date:** 2026-10-28  
**Architecture:** PostgreSQL shared coordinator + Holochain authored provenance/DHT validation.  
**Status:** parser/runner logic tested locally; live PostgreSQL and Holochain integration remain unqualified.

## Work completed

- Repaired pgbench result parsing to support both PostgreSQL's timed-mode actual-count output and count-mode actual/target output.
- Corrected the TPS regex and latency-unit conversion (per-transaction pgbench time is microseconds).
- Made the output path unique per invocation and implemented the successful-count receipt that the previous runner attempted to read but did not create.
- Fail closed on failed, skipped, malformed, missing, or unaccounted log rows.
- Reconcile three independent transaction counts: successful log rows, pgbench stdout count, and SQL accepted-obligation count.
- Attach exact source SHA-256 values for schema, transaction functions, workload script, runner, parser, and parser tests.
- Added seven offline parser regressions, including the timed-mode output used by this runner.
- Made a new NCF research branch push workflow trigger and moved runner.temp from job-level env into step-level env.

## Local result

Seven parser tests pass in 0.009 seconds with ResourceWarning promoted to an error. Bash syntax validation and Python byte compilation pass. Parser, tests, and runner bytewise match the repository copies according to their Git blob SHA-1 identities.

Exact hashes are in [NCF-POSTGRES-PARSER-TEST-RESULTS.txt](NCF-POSTGRES-PARSER-TEST-RESULTS.txt).

## What is still untested

- PostgreSQL 18.6 schema/function application and the SQL smoke script.
- Real concurrent PostgreSQL sessions for cap contention, canonical-work aliases, and request-ID payload conflicts.
- pgbench performance and three-way database reconciliation.
- PostgreSQL crash/restart/failover or point-in-time recovery.
- Actual Mycelix Finance DNA build, Holochain zome implementation for the NCF claim/settlement contract, and 3–5 conductor Sweettest.
- Cross-host uniqueness under partition or production settlement finality.

The editor environment has no psql, pgbench, PostgreSQL server, cargo/rustc, docker, or podman executable, so these runs could not be executed locally. Prior exact-head attempts had zero jobs. The current workflow definition fixes the step-level runner context and explicitly maps the PostgreSQL service port. Latest exact-head PR workflow run [40741164756](https://github.com/Luminous-Dynamics/mycelix/actions/runs/40741164756) now has four instantiated jobs (SQL/concurrency plus 1/4/16-client load) in queued state. Queued is not PASS; inspect job logs and reconciliation artifacts before promoting any database result.

## Completion gate

The PostgreSQL workflow must complete its SQL smoke and independent-session race jobs. The load job must produce raw pgbench logs, stdout, parser receipt, exact source/runtime identities, and a final SQL reconciliation where all three transaction counts agree. Holochain is a separate gate requiring actual built DNA/WASM hashes and conductor/agent evidence. No backend boundary can be promoted to production solely because parser tests pass.


## SQL installability and privileged-path preflight — 2026-10-28

A source audit caught an exact-head PostgreSQL installation blocker before the queued SQL job ran: all seven PL/pgSQL functions had malformed single-dollar body openers (`AS $`); four of the matching function terminators were also single-dollar. PostgreSQL dollar-quoted function bodies require matching delimiters, so the migration was not installable as written.

Fixed in `postgres/002_functions.sql`:
- All seven function bodies now use matching `AS $$ ... $$;` delimiters.
- All seven `SECURITY DEFINER` functions now set `search_path = pg_catalog, ncf, pg_temp`, putting the temporary schema last as required by PostgreSQL's documented safe pattern.
- Added static regression tests for matched dollar delimiters and the privileged-function search path in `test_postgres_contract_static.py`. The workflow executes those guards before applying SQL.

Exact repository identities after the repair:
- `002_functions.sql` Git blob: `cab4c0c7f671e96bb3b4f5260ad12db10f6e4f71`
- `test_postgres_contract_static.py` Git blob: `c3aca9eaed78982d83185ed4ffe7c1bb3902bccc`

Source-level lexical checks against the fetched exact file confirm seven balanced open/close pairs, zero malformed single-dollar delimiters, and seven secure search-path declarations. This is not a PostgreSQL parser or live SQL PASS; the dedicated workflow must still apply the migration, execute the smoke/concurrency suites, and reconcile benchmark output.

The security-path change follows PostgreSQL's official guidance that `SECURITY DEFINER` functions should put `pg_temp` last to prevent temporary objects from masking intended relations: https://www.postgresql.org/docs/18/sql-createfunction.html.
