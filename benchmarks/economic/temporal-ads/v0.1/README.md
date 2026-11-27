# Temporal ADS benchmark corpus V0.1

This directory contains the first language-neutral conformance corpus for issue [#6798](https://github.com/Luminous-Dynamics/mycelix/issues/6798).

## Files

- `scenario.schema.json` — JSON Schema draft 2020-12 for one scenario.
- `scenarios.jsonl` — one independently adjudicated scenario per line.
- `manifest.json` — version, count, scenario identifiers and intended semantic coverage.

## Current scenario coverage

1. `HINDSIGHT-BACKDATED-REVOCATION-001`: the same F0 prefixes yield the same historical result across replicas with different later suffix visibility; an accepted backdated revocation changes F1 without rewriting F0.
2. `VALIDITY-EPISODE-CROSS-PAIR-002`: two validity episodes for one claim cannot cross-pair endpoints.
3. `MISSING-VALID-TO-NOT-OPEN-003`: missing VALID_TO remains Indeterminate, not OPEN.
4. `OMITTED-OWNER-EMPTY-QUERY-004`: valid proofs over a supplied owner set do not close an omitted owner.
5. `WITHHELD-OWNER-ROSTER-005`: withheld owner is not an empty result and prevents positive roster closure.
6. `ROSTER-EPOCH-STALE-006`: a registry epoch advance invalidates reuse of an earlier owner roster/frontier for the new universe.

## Contract boundaries

The corpus separates three outputs:
- interval query result relative to the exact supplied owner prefixes;
- owner roster/frontier reconciliation;
- source-universe and checkpoint authentication.

The expected values are adjudicated semantics, not claims that any one implementation has passed them. Each implementation must report unsupported inputs as explicit abstention/Indeterminate rather than dropping them.

Scenario fields use owner-local sequence cuts, not a global scalar order. Both query instants and endpoint values use integer seconds plus nanoseconds. Every frontier explicitly binds the registry identity, registry epoch, universe profile, frontier digest and each owner's log-root digest; a frontier from an older roster epoch is not current for a newer owner universe.

## Comparative proof-engine baseline

The benchmark programme now also tracks [#6800 — proof-domain query planning](https://github.com/Luminous-Dynamics/mycelix/issues/6800). This extends the comparison beyond authenticated index structures to general verifiable query execution.

A relevant 2026 comparator is [TruthTable: A Verifiable Query Engine](https://opencryptology.org/2026/1260) (IACR ePrint 2026/1260; verified upload 2026-11-13). Its paper abstract reports 17 of 22 TPC-H queries supported and, on a 1M-row TPC-H database, 55s average proving time, 32ms verification and 24kB proofs. These are paper-reported TPC-H figures, not directly comparable temporal-query measurements. Reproduce/adapt the same query families and report the semantic and trust-boundary differences before making comparisons.

Also include [HMT — Authenticated Data Structures for Dynamic Workloads](https://opencryptology.org/2026/1235) for skew-aware owner membership workloads. HMT membership proofs do not establish complete owner-roster range coverage.

The comparison must distinguish:
- **ADS verification:** authenticate source membership/ranges, log prefixes and interval query results;
- **verifiable query execution:** prove execution of a supported relational query over a committed input;
- **hybrid composition:** authenticate the source snapshot/frontier with ADS proofs, then prove the query plan over those exact authenticated inputs.

A succinct execution proof does not establish real-world truth, source-universe completeness, checkpoint authenticity or semantic interpretation unless those are independently bound and proved. Succinctness also does not imply zero knowledge.

Report query execution, proof generation, verification and end-to-end latency separately, alongside proof/transmission bytes, peak memory, index/update/checkpoint costs, query-plan coverage, privacy leakage and setup/trust assumptions. Pin source commits, corpus manifest, hardware and toolchain; publish raw samples and p50/p95/p99 from at least 30 independent trials after warm-up.

## Use

- The self-contained Go corpus validator is in this directory. Run `go test ./...`, `go test -race ./...` and `go vet ./...`.
- The validator strictly decodes each JSONL line, checks IDs/sequence and owner references, independently evaluates the seed temporal semantics, and checks expected owner-roster/frontier dispositions.
- Validate every JSONL line against `scenario.schema.json` with a JSON Schema draft 2020-12 validator.
- Run the same corpus through the independent Go owner-prefix oracle, an independently authored Rust oracle, each database adapter and each authenticated ADS adapter.
- Keep the held-out adversarial set separate; do not tune an implementation against the held-out expected results.

The corpus is a starting synthetic conformance set, not a statistically representative production workload or a proof of full field coverage.