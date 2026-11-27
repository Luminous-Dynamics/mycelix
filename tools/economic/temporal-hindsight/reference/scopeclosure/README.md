# Temporal owner-universe closure reference (Go)

This is an independent, dependency-free **structural reconciliation oracle** for Mycelix issue [#6795](https://github.com/Luminous-Dynamics/mycelix/issues/6795).

It addresses a gap above the temporal frontier forest: a valid proof over all owners named in a supplied list cannot detect an owner omitted before that list was formed. The checker compares:
1. an independently supplied owner-universe snapshot for one exact scope/profile/registry epoch;
2. a closure manifest committing to that candidate roster and its explicit dispositions; and
3. the verifier's exact owner-frontier bindings.

The checker requires each Required owner to appear exactly once in the frontier with the same stable owner-history identity. Unknown, withheld or unavailable candidates, missing owners, duplicate owners, wrong registry epochs, scope substitution, explicit exclusions appearing in the frontier, and unresolved owner gaps all yield `Indeterminate`.

## Important claim ceiling

The positive structural disposition is `FrontierCoversSuppliedRequiredRoster`, **not** `RosterClosedForDeclaredScope` and not `CompleteForAcceptedFrontier`.

Both `UniverseAuthentication` and `FrontierAuthentication` remain `NotEvaluated`. The implementation does not verify signatures, registry completeness, checkpoint authority, source discovery or source truth. Its expected universe must be supplied by an independently qualified upstream enumerator/verifier. An empty supplied universe is therefore not proof that the actual universe is empty.

This is intentional. A data structure cannot authenticate its own expected set.

## Test commands

```sh
go test ./...
go test -race ./...
go vet ./...
go test -run 'TestValidFrontierOverOmittedOwner|TestManifestCannotSelfDeclareAnIncompleteRoster|TestLaterRegistryEpochDoesNotRewriteHistoricalCertificate' -count=30 ./...
go test -run '^$' -bench BenchmarkScopeFrontierReconciliation -benchmem -benchtime=30x ./...
```

Tests cover omitted owners, manifest self-substitution, Unknown/Withheld/Unavailable candidates, explicit out-of-scope evidence, stale registry epoch, scope and stable-history substitution, duplicate history identities, owner gaps, empty roster, nanosecond scope commitment and later registry-epoch noninterference.

## Benchmarks

The benchmark measures only canonical roster/manifest/frontier reconciliation. It does not include registry queries, cryptographic source proof verification, signature verification, checkpoint verification, owner log replay, projection derivation, interval proofs or network transfer. Raw bounded observations are in `BENCHMARK-OBSERVATIONS.md`. Do not compare these values with temporal query/index benchmarks or present them as SOTA performance.

## Next trust-boundary work

- independently verify the source universe snapshot, registry epoch and enumeration completion;
- bind source-holder coverage, pagination/cursor completion and explicit withheld/unknown states;
- compose the qualified result into #6780/#6781 without changing their narrower proof ceilings;
- run the paired registry/frontier TLA+ model and cross-language Rust/Go vectors.
