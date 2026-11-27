# Temporal owner-universe closure reference (Go)

This is an independent, dependency-free **structural reconciliation oracle** for Mycelix issue [#6795](https://github.com/Luminous-Dynamics/mycelix/issues/6795).

It addresses a gap above the temporal frontier forest: a valid proof over all owners named in a supplied list cannot detect an owner omitted before that list was formed. The checker compares:
1. an independently supplied owner-universe snapshot for one exact scope/profile/registry epoch;
2. a closure manifest committing to that candidate roster and its explicit dispositions; and
3. the verifier's exact owner-frontier bindings.

The checker requires each Required owner to appear exactly once in the frontier with the same stable owner-history identity. The frontier itself binds the exact registry identity, registry epoch and universe profile used by the roster manifest; a frontier from an older registry epoch cannot be reused against a newer roster, even when owner labels happen to be unchanged. Unknown, withheld or unavailable candidates, missing owners, duplicate owners, wrong registry epochs, scope substitution, explicit exclusions appearing in the frontier, and unresolved owner gaps all yield `Indeterminate`.

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

Tests cover omitted owners, manifest self-substitution, Unknown/Withheld/Unavailable candidates, explicit out-of-scope evidence, registry epoch mismatch in both manifest and frontier, registry/profile substitution, scope and stable-history substitution, duplicate history identities, owner gaps, empty roster, nanosecond scope commitment, canonical owner ordering and later registry-epoch noninterference.

## Benchmarks

The benchmark measures only canonical roster/manifest/frontier reconciliation. It does not include registry queries, cryptographic source proof verification, signature verification, checkpoint verification, owner log replay, projection derivation, interval proofs or network transfer. Raw bounded observations are in `BENCHMARK-OBSERVATIONS.md`. Do not compare these values with temporal query/index benchmarks or present them as SOTA performance.

## Next trust-boundary work

- independently verify the source universe snapshot, registry epoch and enumeration completion;
- bind source-holder coverage, pagination/cursor completion and explicit withheld/unknown states;
- compose the qualified result into #6780/#6781 without changing their narrower proof ceilings;
- run the paired registry/frontier TLA+ model and cross-language Rust/Go vectors.


## Structural pagination-chain validator (V1)

The package now includes `enumeration.go`, which validates the internal structure of a source's paginated owner enumeration before its candidate inventory is supplied to `Reconcile`:

- binds source, snapshot, registry identity/epoch, universe profile, query-scope digest and enumeration-predicate digest on every page;
- verifies zero-based contiguous page indices, request/next cursor continuity, cursor-cycle absence, exactly one terminal page at the end, and exact page count;
- recomputes each domain-separated page digest and the aggregate canonical owner-candidate root, rejecting duplicate owners across pages;
- binds first/terminal page digests, expected and observed page counts, the source completion disposition and exact completion-receipt digest into a deterministic manifest/result commitment;
- preserves deterministic failure reasons when several metadata fields are malformed.

The positive disposition is only `EnumerationPageChainWellFormed`. Both `PageChainAuthentication` and `SourceCompletenessAuthentication` remain `NotEvaluated`. A source's declaration that it completed pagination, and a digest-shaped receipt reference, are not proof that the source is honest, authoritative, complete, current, or exhaustive. A separate verifier must qualify the source/checkpoint/receipt and the enumeration semantics before the owner universe can be called closed. The page-chain validator therefore does **not** upgrade the package's `FrontierCoversSuppliedRequiredRoster` result to `RosterClosedForDeclaredScope`.

Adversarial tests cover truncated chains, page-index gaps, cursor discontinuities and cycles, early/missing terminal markers, duplicate owners across pages, foreign snapshots/epochs/predicates, missing completion receipts, Unknown/Incomplete source declarations, page/root tampering, manifest digest substitution, deterministic failures and candidate-order canonicalization.


## Bounded pagination model

The formal directory also contains `TemporalEnumerationContinuity.tla` and a separate truncated-page mutant. The correct finite model requires all three modeled pages to be observed in order, the continuation cursor to close, and exactly the final page to be terminal before `chainWellFormed` can become true. The mutant declares completion after page zero and must violate `CompletePageSet`.

This model checks only the bounded three-page abstraction. It does not authenticate cursor tokens, page signatures, source identity, registry snapshot completeness, or arbitrary-length enumerations. Those trust obligations remain external and are not promoted by the Go structural page-chain result.
