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

## Use

- The self-contained Go corpus validator is in this directory. Run `go test ./...`, `go test -race ./...` and `go vet ./...`.
- The validator strictly decodes each JSONL line, checks IDs/sequence and owner references, independently evaluates the seed temporal semantics, and checks expected owner-roster/frontier dispositions.
- Validate every JSONL line against `scenario.schema.json` with a JSON Schema draft 2020-12 validator.
- Run the same corpus through the independent Go owner-prefix oracle, an independently authored Rust oracle, each database adapter and each authenticated ADS adapter.
- Keep the held-out adversarial set separate; do not tune an implementation against the held-out expected results.

The corpus is a starting synthetic conformance set, not a statistically representative production workload or a proof of full field coverage.