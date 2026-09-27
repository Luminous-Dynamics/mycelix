# MYC-INT-006B — Integral reference-stack evaluation protocol

Status: preregistered evaluation protocol. No architecture winner, production-readiness claim, or Integral endorsement is established.

## Purpose

Evaluate the MYC-INT-006A reference-stack proposal under a common, falsifiable workload before any contributor-facing recommendation is made.

The comparison contains three implementation families:

1. conventional local service stack;
2. Mycelix + Holochain reference node;
3. hybrid conventional-local + Mycelix interoperability/federation profile.

The benchmark is evidence for Integral's own technology-stack deliberation. It is not a mechanism for Mycelix to decide that deliberation.

## Source basis

Observed 2026-09-27:

- Integral Technical Specifications: `https://integralcollective.io/documents/specifications.html`
- Integral Development Guide v0.1: `https://integralcollective.io/documents/integral_devguide_v01.pdf`
- MYC-INT-003A0 source registry: #3144
- MYC-INT-006A reference-stack proposal: #3145 / PR #3149

The benchmark compares implementations **behind the same external semantic contracts**. Holochain is one candidate runtime, not a required protocol identity.

## Reuse existing comparison discipline

Reuse OPS-INTEL-013 / #2945 rather than inventing another comparison ontology.

Evidence classes:

- `DocumentedCapability`
- `LocallyDemonstratedCapability`
- `DirectComparativeMeasurement`

Useful maturity states:

- `Designed`
- `ImplementedUnqualified`
- `QualifiedInternal`
- `MeasuredInternal`
- `Unknown`
- `NotApplicable`

No scalar `platform_score`, architecture winner, or points for unimplemented design.

## Benchmark subject

Use #3119 / MYC-INT-005A's water-system vertical slice as generation I0.

Every candidate receives the same runtime-neutral fixture corpus and must preserve:

```text
Observation / ActorReport
        ↓
Evidence / source provenance
        ↓
Issue
        ↓
Alternatives + Objections
        ↓
Non-executive Recommendation
        ↓
Decision
        ↓
Authorization
        ↓
ImplementationAttempt
        ↓
ImplementationReceipt
        ↓
OutcomeObservation
        ↓
ReviewCandidate
```

No candidate receives credit for a faster path created by collapsing required states.

## I0 canonical corpus contract

The evaluation harness must not let each candidate invent its own interpretation of the water scenario.

Freeze one runtime-neutral manifest with:

```text
corpus_id
corpus_version
source_manifest_ref
schema_manifest_ref
object_manifest
ordered_input_actions
unordered_delivery_sets where applicable
expected semantic dispositions
expected preserved unknowns/conflicts
expected rejection classes
expected lineage edges
export expectations
nonclaims
```

Runtime-specific IDs, timestamps, storage keys, DHT hashes, database row IDs and transport envelopes are excluded from canonical expected identity unless the fixture explicitly marks them as source semantics.

### I0 logical subjects

At minimum freeze distinct identities for:

- water system / resource subject;
- maintenance issue;
- direct observation;
- actor report;
- evidence bundle;
- two competing alternatives;
- one objection with affected-party context;
- one analytical recommendation;
- one governance decision;
- one bounded authorization;
- one implementation attempt;
- one implementation receipt;
- one contradictory or partially adverse outcome observation;
- one review candidate;
- one superseding decision candidate without destructive mutation of history.

### I0 expected dispositions

Every implementation must produce an explicit result for each case:

- `Accepted`
- `Rejected(reason)`
- `Indeterminate(reason)`
- `Unsupported(profile)`

Names are conceptual; exact serialization may differ by implementation. Silent omission is not a valid disposition.

### I0 semantic oracle

The oracle checks relations, not storage layout.

Examples:

```text
recommendation.subject == issue/alternative context
recommendation.authority == none

decision references recommendation/evidence as inputs
but decision != recommendation

authorization references exact decision/scope
but authorization != decision

implementation receipt references an admitted attempt
but receipt != outcome truth

contradictory outcome -> review candidate
not destructive rewrite of prior decision
```

A candidate may use a relational database, Holochain entries, event sourcing, or another runtime and still pass if the same externally meaningful relations and non-equivalences are preserved.

## Mandatory hostile fixtures

All implementations receive the same hostile cases:

1. `RecommendationDirectEffect` — recommendation attempts direct execution;
2. `ForeignDecisionLocalAuthority` — imported decision attempts local authorization;
3. `CredentialAuthorityLaunder` — valid credential/identity attempts to become authority;
4. `DuplicateSemanticDelivery` — same semantic event delivered twice;
5. `UnknownAfterPersistBeforeAck` — receiver persists before sender timeout/ack failure;
6. `StaleSchemaGeneration` — stale source schema/interface version;
7. `ExternalCertificationLocalAcceptance` — external certification presented as locally accepted certification;
8. `DerivedSummaryAsSourceFact` — derived summary presented as source-owned current state;
9. `ExpiredAuthorizationEffect` — expired/revoked authorization presented for a new effect;
10. `OutcomeMutatesDecision` — contradictory outcome attempts direct mutation of historical decision;
11. `ReorderedTransport` — delivery order differs where ordering is not guaranteed;
12. `SameVisibleIdDifferentSchema` — same visible object ID under a different source schema;
13. `SilentTranslationLoss` — translation drops source/authority-relevant semantics without receipt;
14. `ForeignAuthorityLocalAuthority` — foreign authority presented as local authority without explicit recognition/delegation;
15. `PartitionLastArrivalWins` — conflicting partitioned states resolved solely by arrival order;
16. `PredictionAsObservation` — model/prediction output presented as direct observation;
17. `RegistrationAsExecution` — registration/delivery receipt presented as effect receipt;
18. `SuccessAsDesiredOutcome` — successful command completion presented as proof the desired real-world outcome occurred.

## Harness mutation controls

The harness must include deliberately weakened implementations or adapters proving it can detect at least:

- recommendation -> authorization shortcut;
- duplicate delivery -> duplicate logical effect;
- timeout -> `DefinitelyFailed` without evidence;
- stale summary -> current source fact;
- foreign decision -> local authority;
- export strips schema version;
- export strips provenance;
- partition conflict -> last-arrival winner;
- historical decision overwritten in place;
- constant-success implementation that ignores semantic input.

A benchmark incapable of failing these mutations cannot support interoperability claims.

## Fairness theorem

```text
same benchmark label
!= same semantics
```

A candidate that does not implement a required semantic property must report it as unsupported or differently modeled. It may still report raw performance, but the omission remains visible beside that measurement.

A competent conventional implementation is required; do not compare Mycelix against a deliberately weak centralized straw man.

## Candidate A — conventional local service stack

A reasonable I0 reference may use:

```text
Rust or another documented language/runtime
+ relational database
+ explicit HTTP/gRPC/internal interface boundary
+ transactional outbox/inbox where required
+ deterministic fixture/conformance harness
```

A relational database does not imply centralized governance authority. Institutional/federation semantics must be modeled explicitly.

## Candidate B — Mycelix + Holochain reference node

Use the same external fixture with Holochain as the reference persistence/federation profile.

Required separation:

```text
Integral semantic identity
!= Holochain ActionHash / DHT identity
```

Holochain IDs may be provenance/runtime identifiers but cannot be the only identity preserved in export.

## Candidate C — hybrid

Use conventional local services for selected source-owned domain state and an explicit Mycelix interoperability/federation edge.

The ownership map must prevent two canonical owners for the same fact:

```text
source-owned local state
+ federation projection
!= two authorities for the same fact
```

## Environment manifest

Every run freezes:

- benchmark generation ID;
- exact source commits/releases;
- dependency locks;
- fixture corpus commitment;
- host hardware or VM profile;
- OS/runtime/container profile;
- database/runtime versions;
- build mode;
- node count and topology;
- network profile;
- configuration;
- cache state;
- measurement tools;
- start/end timestamps as observational metadata;
- known environmental limitations.

Material environment changes create a new run/generation; old results are never silently refreshed.

## Correctness and semantic fidelity

Report separate pass/fail/indeterminate/unsupported results for:

- source namespace/schema/version preservation;
- semantic object identity preservation;
- source provenance preservation;
- recommendation/decision/authorization/effect separation;
- local/foreign authority separation;
- source-owned-current-state vs derived-summary separation;
- duplicate semantic event idempotency;
- transport attempt vs semantic admission separation;
- timeout/unknown-state preservation;
- historical supersession without destructive mutation;
- translation-loss disclosure;
- export/import identity round trip;
- conflict preservation;
- bounded currentness/staleness semantics used by the profile.

Correctness evidence is not converted into a performance score.

## Partition and disconnection profiles

Do not report one generic `offline-capable` flag.

Run:

- `NET-0` fully connected;
- `NET-1` peer absent before operation;
- `NET-2` partition after local append;
- `NET-3` concurrent conflicting partitioned updates;
- `NET-4` delayed/reordered delivery;
- `NET-5` reconnect/reconciliation;
- `NET-6` prolonged peer absence.

For each profile report:

- locally available operations;
- unavailable operations;
- unknown/indeterminate state;
- queued operations;
- conflicts created/preserved;
- reconciliation steps;
- human/operator intervention required.

## Performance metrics

Keep distributions separate:

- ingest throughput;
- p50/p95/p99 local write/admission latency;
- p50/p95/p99 query/projection latency;
- cross-node propagation/convergence latency;
- partition recovery/reconciliation duration;
- export/import duration;
- cold startup duration;
- restore duration;
- storage bytes/event/object and total growth;
- CPU utilization;
- peak RSS/memory;
- network bytes sent/received.

No weighted aggregate performance score.

## Operations benchmark

Record reproducible procedures and operator effort for:

1. clean install;
2. first-node bootstrap;
3. add second node;
4. ordinary upgrade;
5. incompatible schema migration;
6. backup;
7. restore on a clean host;
8. recovery from deliberately corrupted local non-secret state where supported;
9. diagnose a failed/indeterminate delivery;
10. export all externally meaningful state;
11. remove/replace a runtime component.

For each task record commands/steps, elapsed time, manual decisions, files/configuration touched, expected interruption, observability used, failure points and rollback route.

## Developer ergonomics benchmark

Use bounded tasks rather than subjective ratings:

- add one backward-compatible optional field;
- introduce one incompatible schema revision without rewriting history;
- add one interface/seam field;
- add one validation rule;
- trace one malformed event through logs/provenance;
- add one alternate adapter;
- implement one runtime-neutral export;
- reproduce I0 from a fresh checkout.

Record elapsed time, files/tests changed, rebuild/test duration, errors encountered, runtime-specific concepts required and external documentation consulted.

Founder/developer timings are not universal usability evidence; participant experience and sample size must be disclosed.

## Exit / migration qualification

The external offer is not adoption-neutral until runtime replacement executes.

Target theorem:

```text
Implementation A
      ↓ export
RuntimeNeutralCorpus
      ↓ import
Implementation B
      ↓
Same externally meaningful semantic identities
+ source lineage
+ historical status/supersession
+ unresolved conflict/unknown state
+ declared translation losses
```

Required cases over successive generations:

- Holochain reference node -> conventional implementation;
- conventional implementation -> Holochain reference node;
- hybrid node -> protocol-only consumer;
- old source/schema generation -> explicit successor implementation.

Runtime-specific provenance may differ. It cannot be the sole identity of Integral-facing objects.

## Export corpus minimum

Preserve where applicable:

- source schema namespace/name/version;
- semantic object ID/version;
- source provenance refs/commitments;
- event/decision lineage;
- standing/authority historical refs without manufacturing live authority;
- authorization historical identity/status;
- implementation/effect receipts;
- outcome/review lineage;
- supersession/correction history;
- translation/migration receipts;
- unresolved conflict/unknown state.

Secrets, live bearer capabilities and runtime-private caches are not required to migrate as domain history.

## Scale generations

```text
I0  one vertical slice / tens of objects
I1  ~10^3 semantic objects / ~10^4 events
I2  ~10^5 semantic objects / ~10^6 events
```

Exact fixture sizes are frozen per generation. Proceed upward only after correctness and resource behavior are understood at the previous generation.

## Evaluation generation

Each published run should produce one immutable record containing:

```text
cutoff_date
Integral source-manifest ref
benchmark profile/version
fixture commitment
candidate implementation identities
candidate maturity/evidence classes
environment manifests
semantic-fidelity results
network/partition results
performance distributions
operations results
developer-task evidence
migration/exit evidence
unsupported cases
limitations
artifact/receipt refs
```

A later better result creates a new generation rather than rewriting the old one.

## Cross-project neutrality check

I0 demonstrates Integral compatibility only. Before treating the interoperability waist as domain-neutral, run the same semantic primitives through one materially different non-Integral governance adapter.

Required theorem:

```text
same core semantic waist
+ different governance/economic policy
+ different adapter semantics
-> no core policy rewrite required
```

If the second adapter requires Integral-specific concepts in core, the abstraction boundary has failed and must be revised.

## Outreach gate

Do not convert MYC-INT-006A into an external technology recommendation merely from architecture documentation.

Before a dependency-oriented offer, require at minimum:

- exact status disclosure for every Mycelix/Luminous component named;
- relevant interoperability qualification gates resolved or explicitly presented as unqualified;
- complete I0 semantic-fidelity run;
- measured execution for the candidate stack being offered;
- a competent conventional baseline;
- at least one executed migration/exit direction;
- one second non-Integral adapter neutrality run;
- licensing terms stated consistently with #884's resolved evidence;
- limitations and unresolved Integral decisions stated explicitly.

A contributor-facing note may invite collaboration earlier, but must call the stack a proposal/reference candidate rather than production infrastructure.

## Relationship

- #3145 / PR #3149 — MYC-INT-006A adoption-neutral reference-stack proposal
- #3147 — MYC-INT-006B evaluation issue
- #2945 — shared comparison methodology
- #3119 / MYC-INT-005A — water-system I0 workload
- #3142 — semantic seam/delivery contract
- #3143 — source-owned fact vs derived view
- #3144 — external source registry
- #884 — license/IP reconciliation

## Nonclaims

This protocol does not establish that Mycelix/Holochain, a conventional stack or a hybrid stack is preferable overall. It does not establish Integral adoption, governance legitimacy, production scalability, security certification, legal compliance, or fitness for a real infrastructure deployment. It defines evidence that can inform those later decisions without replacing them.