# Temporal owner-universe closure — V0.1

**Status:** design and structural reference draft. The TLA+ model has been authored but not run through TLC. The Go reference tests pass locally; neither source-universe nor frontier authenticity is qualified by this package.

## 1. The missing theorem

The temporal frontier forest can prove that every owner in a supplied roster was queried. It cannot detect an owner omitted before the roster was created.

The intended claim is bounded:

> For this exact verifier, policy, namespace, subject, claim, query profile, valid-time instant, source-universe profile and registry epoch, every owner candidate capable of contributing an in-scope assertion has been accounted for.

This is not a claim about every possible owner in the world. It is a claim relative to an independently enumerated, versioned source universe.

## 2. Separate the universes

There are two distinct sets:

1. **Scope-eligible owner universe:** source owners enumerated under a declared and independently qualified scope/profile/registry snapshot.
2. **Accepted frontier owner set:** stable owner-history identities and owner-local cuts actually accepted by verifier X at frontier F.

The universe must not be inferred from the returned query records or the receipts already found. That would allow the queried data to define its own expected completeness.

The reconciliation is:

`ScopeClosure(S,U,E) + Frontier(F,U) + OwnerLogs(U,F) + Projections(U,F) + IntervalProofs(Q,V)`

where E is the registry epoch. Owner name ordering is only for canonical encoding; it is never causal ordering.

## 3. Dispositions and claim ceilings

The reference checker uses `FrontierCoversSuppliedRequiredRoster` as its only positive structural disposition.

It does **not** report `RosterClosedForDeclaredScope` or `CompleteForAcceptedFrontier`. Those stronger dispositions require upstream proofs the reference package does not verify:
- authentication and completeness of the source universe;
- registry/checkpoint currentness and authority;
- source-holder enumeration and pagination closure;
- signed verifier frontier and owner-history roots;
- complete owner-log prefix replay;
- semantic payload-to-interval projection.

The output therefore always retains:
- `UniverseAuthentication=NotEvaluated`;
- `FrontierAuthentication=NotEvaluated`.

An empty candidate roster can structurally reconcile with an empty frontier, but it is not proof that the actual scope contains no owner. That outcome remains explicitly untrusted until an independently verified closure receipt establishes the source universe.

## 4. Candidate dispositions

Each candidate is one of:
- `Required`: must appear exactly once in the accepted frontier with the exact stable owner-history identity;
- `ExcludedByDeclaredProfile`: excluded by a versioned profile reason and evidence digest, and must not appear in the accepted frontier;
- `Unknown`, `Withheld`, or `Unavailable`: prevents positive reconciliation and remains Indeterminate.

A required owner with zero accepted events must still have a stable owner-history identity and an explicit empty-history binding. Owner absence is not represented by omitting the owner.

The roster binds owner identity separately from owner-history identity. Duplicate owner identities and two required owners sharing one history identity are rejected.

## 5. Structural Go reference

The independent reference lives at `tools/economic/temporal-hindsight/reference/scopeclosure/`. It:
- computes canonical exact-scope and roster commitments;
- binds registry ID, registry epoch, universe profile and source-proof digest;
- compares a closure manifest with a separately supplied expected-universe snapshot;
- verifies exact owner-set equality against the frontier;
- checks owner/history identity equality, duplicate owners and unresolved gaps;
- refuses Unknown/Withheld/Unavailable and unproven profile exclusions;
- emits deterministic failure reasons and certificate digests.

The expected universe is an input from a future independent verifier, not an output of the temporal query. The code validates the consistency of supplied data; it does not authenticate that external verifier.

## 6. Formal model

`TemporalOwnerUniverseClosure.tla` models:
- a source registry epoch and a live owner universe;
- roster enumeration and explicit required/excluded/unknown states;
- a newly registered owner after an earlier frontier;
- exact scope-roster closure and frontier coverage;
- an immutable historical certificate epoch;
- a faulty resolver that closes over its own incomplete candidate list.

The correct model requires `rosterCandidates = liveUniverse` before closure. The mutant intentionally omits this check and should violate `RosterUniverseEquality`. The model and configs are authored but not TLC-checked.

## 7. Benchmark protocol

Correctness targets (proposed, not achieved):

| Measure | Target |
|---|---:|
| Hidden-owner omissions detected on frozen oracle corpus | 100% |
| False roster closure | 0 |
| False frontier coverage | 0 |
| False accepted-frontier completeness | 0 |
| Empty-result promotions under unknown/withheld/unavailable coverage | 0 |
| Owner/history identity substitutions accepted | 0 |
| Historical roster drift under later registry epochs | 0 |
| Independent roster reconciliation parity | 100% |

Scale owner counts at 1 / 10 / 100 / 1k / 10k and assertion counts at 1k / 10k / 100k / 1m. Cross uniform and highly skewed ownership, registry pagination depth, owner churn, alias churn, withholding, delayed visibility and source common-cause clusters.

Measure source-universe enumeration, closure proof construction/verification, roster-frontier reconciliation, owner log proof verification, projection replay and interval proof separately. Report p50/p95/p99, proof bytes, allocations, peak memory, query critical path and stale-roster invalidation latency. Use at least 30 independent timed runs after warm-up for performance distributions and publish raw samples, source/corpus digests and exact commit SHA.

The local reference benchmark in `reference/scopeclosure/BENCHMARK-OBSERVATIONS.md` only measures in-memory structural reconciliation. It is not registry enumeration, cryptographic verification or temporal index performance.

## 8. Industry comparison

- **XTDB:** strong native bitemporal valid-time/system-time semantics, including retrospective correction and "what the database knew" reconstruction. It is the temporal database baseline, not a proof of an independently closed distributed owner universe. See [XTDB time](https://docs.xtdb.com/about/time-in-xtdb.html).
- **SQL Server temporal tables:** mature system-versioned point-in-time history and `AS OF` reconstruction. Benchmark its history semantics and cost, not distributed roster proofs. See [Microsoft temporal tables](https://learn.microsoft.com/en-us/sql/relational-databases/tables/temporal-tables?view=sql-server-ver17).
- **VeriTS:** authenticated aggregate interval trees, minimum-covering-set completeness and sliding-window delta verification. This is a temporal query ADS baseline, not an owner universe discovery theorem. See [VeriTS](https://arxiv.org/abs/2608.28318).
- **VTRQ:** separate temporal interval ADS combined with spatial trajectory proofs. Useful for tree maintenance and cross-dimensional proof composition, but not a source-owner closure mechanism. See [VTRQ](https://arxiv.org/abs/2608.21314).
- **Mycelix:** benchmark the composed path: independently closed scope roster, verifier-relative owner frontier, full owner log, semantic assertion mapping, per-owner interval proofs and historical replay. Only compare systems on identical source universes, updates, query scopes and claim guarantees.

## 9. Composition ownership

Reuse #6705 for open-world source discovery, #1289 for scoped absence, #6441 for the independently verified dependency-closure pattern, #5790 for expected evidence-population completeness, and #6501 for authenticated directory history ingestion. The temporal layer owns only the mapping from the declared query scope to the exact owner universe and the reconciliation of that universe with the accepted frontier. It must not create another owner registry, authorization engine or truth oracle.

## 10. Nonclaims

This work does not prove global discovery, source truth, trusted time, registry honesty, signed checkpoint authenticity, legal effect, financial authorization, settlement, or permission to mutate money/journals/external systems.
