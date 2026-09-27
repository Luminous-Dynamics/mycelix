# MYC-INT-005B — Integral reference-stack evaluation protocol

Status: preregistered evaluation protocol. No architecture winner, production-readiness claim, or Integral endorsement is established.

## Purpose

Evaluate the MYC-INT-005A reference-stack proposal under a common, falsifiable workload before any contributor-facing recommendation is made.

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

The Development Guide defines interfaces as formal contracts that hide a module/system's internal storage and permit independent evolution. It explicitly treats Holochain as a reference technology/potential component rather than a committed architectural choice.

Therefore the benchmark compares implementations **behind the same external semantic contracts**.

## Reuse existing benchmark discipline

This tranche reuses OPS-INTEL-013 / #2945 methodology rather than inventing another comparison ontology.

Evidence classes:

- `DocumentedCapability`
- `LocallyDemonstratedCapability`
- `DirectComparativeMeasurement`

Useful implementation states:

- `Designed`
- `ImplementedUnqualified`
- `QualifiedInternal`
- `MeasuredInternal`
- `Unknown`
- `NotApplicable`

There is no scalar `platform_score`, architecture winner, or points awarded for unimplemented design.

## Benchmark subject

Use #3119's water-system vertical slice as generation I0.

Every candidate implementation receives the same runtime-neutral fixture corpus and must preserve the same logical sequence:

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

## Mandatory anti-collapse cases

All implementations receive the same hostile fixtures:

1. recommendation attempts direct execution;
2. imported decision attempts to become local authorization;
3. valid credential/identity attempts to become authority;
4. duplicate delivery of one semantic event;
5. delivery timeout after receiver persistence but before acknowledgment;
6. stale source schema/interface version;
7. external certification presented as local certification acceptance;
8. derived summary presented as source-owned current state;
9. expired/revoked authorization presented for a new effect;
10. contradictory outcome attempts direct mutation of historical decision;
11. reordered transport events under a profile with no ordering guarantee;
12. same visible identifier under a different source schema;
13. translation silently drops authority-relevant/source-relevant semantics;
14. foreign authority presented as local authority without explicit local recognition/delegation.

## Fairness theorem

```text
same benchmark label
!= same semantics
```

A candidate that does not implement a required semantic property must report the property as unsupported/differently modeled. It may still report raw performance, but the omission remains visible beside the measurement.

Do not compare a candidate that preserves conflict/unknown states with one that silently chooses a winner as if both performed the same work.

## Candidate A — conventional local service stack

The exact I0 implementation may use a common architecture such as:

```text
Rust or another documented language/runtime
+ relational database
+ explicit HTTP/gRPC/internal interface boundary
+ transactional outbox/inbox where required
+ deterministic fixture/conformance harness
```

This candidate is not a straw man. It should use competent conventional engineering, including transactionality, idempotency and explicit service boundaries.

No requirement exists that it centralize all institutional authority merely because it uses a relational database locally.

## Candidate B — Mycelix + Holochain reference node

Use the same external semantic fixture with Holochain as a reference runtime profile.

Required separation:

```text
Integral semantic identity
!= Holochain ActionHash / DHT identity
```

Holochain-specific IDs may remain provenance/runtime identifiers, but an export must retain runtime-neutral source/schema/object identity.

## Candidate C — hybrid

Use conventional local services for selected domain state while using Mycelix contracts and an explicit federation/interoperability edge.

The benchmark must disclose ownership clearly enough to avoid two competing canonical states.

```text
local service state owner
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

Changing a material coordinate creates a new run/generation; old results are not silently refreshed.

## Correctness and semantic-fidelity metrics

Report independent pass/fail/indeterminate results for:

- source namespace/schema/version preservation;
- object identity preservation;
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

Correctness is prerequisite evidence, not a performance score.

## Partition and disconnection profiles

Do not report one generic `offline-capable` flag.

Run deterministic cases:

### NET-0 — fully connected

Normal baseline.

### NET-1 — peer absent before operation

Record which operations remain locally meaningful and which require remote evidence/authority.

### NET-2 — partition after local append

One node commits a local event but cannot federate it immediately.

### NET-3 — concurrent partitioned conflict

Two nodes produce semantically conflicting observations/updates while disconnected.

### NET-4 — delayed / reordered delivery

Messages arrive outside production order where the interface does not promise ordering.

### NET-5 — reconnect

Measure discovery, reconciliation, conflict exposure and convergence under the exact profile.

### NET-6 — prolonged peer absence

Verify the system does not invent closed-world completeness merely because another node is unreachable.

For each profile report:

- locally available operations;
- unavailable operations;
- unknown/indeterminate state;
- queued operations;
- conflicts created/preserved;
- reconciliation steps;
- human/operator intervention required.

## Performance metrics

Keep each distribution separate:

- ingest throughput;
- p50/p95/p99 local write/admission latency;
- p50/p95/p99 query/projection latency;
- cross-node propagation/convergence latency under the exact profile;
- partition recovery/reconciliation duration;
- export duration;
- import duration;
- cold startup duration;
- recovery/restore duration;
- storage bytes per event/object and total growth;
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
8. recovery from deliberately corrupted local non-secret state where the runtime supports a defined procedure;
9. diagnose a failed/indeterminate delivery;
10. export all externally meaningful state;
11. remove/replace a runtime component.

For each task record:

- commands/steps;
- elapsed time;
- number of manual decisions;
- files/configuration touched;
- expected service interruption;
- observability used;
- failure points;
- cleanup/rollback route.

## Developer ergonomics benchmark

Prefer bounded tasks to subjective star ratings.

Candidate tasks:

- add one backward-compatible optional field;
- introduce one incompatible schema revision without rewriting history;
- add one interface/seam field;
- add one validation rule;
- trace one malformed event through logs/provenance;
- add one alternate adapter;
- implement one runtime-neutral export;
- reproduce I0 from a fresh checkout.

Record:

- elapsed time;
- source files changed;
- tests changed/added;
- rebuild/test duration;
- errors encountered;
- runtime-specific concepts required;
- external documentation consulted.

Founder/developer timings are not universal usability evidence. Contributor sample size and experience must be disclosed.

## Exit / migration qualification

The external offer is not adoption-neutral until at least one runtime-replacement proof executes.

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
+ declared translation losses
```

Required cases over successive generations:

- Holochain reference node -> conventional implementation;
- conventional implementation -> Holochain reference node;
- hybrid node -> protocol-only consumer;
- old source/schema generation -> explicit successor implementation.

Runtime-specific provenance is allowed to differ. It cannot be the sole identity of Integral-facing objects.

## Export corpus minimum

An export should preserve at least, where applicable:

- source schema namespace/name/version;
- semantic object ID/version;
- source provenance refs/commitments;
- event/decision lineage;
- standing/authority refs without manufacturing live authority;
- authorization historical identity/status;
- implementation/effect receipts;
- outcome/review lineage;
- supersession/correction history;
- translation/migration receipts;
- unresolved conflict/unknown state.

Secrets, live bearer capabilities and runtime-private caches are explicitly not required to migrate as domain history.

## Scale generations

Do not jump to a large-scale claim.

```text
I0  one vertical slice / tens of objects
I1  ~10^3 semantic objects / ~10^4 events
I2  ~10^5 semantic objects / ~10^6 events
```

Exact fixture sizes are frozen in each generation.

Proceed to a larger generation only after resource behavior and correctness at the prior generation are understood.

## Mutation controls

The harness must prove it can detect weakened implementations.

Synthetic bad candidates should include at least:

- `Recommendation -> Authorization` shortcut;
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

## Evaluation generation

Each published run should produce one immutable evaluation record containing:

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

Name and serialization remain implementation decisions.

A later better result creates a new generation rather than rewriting an old one.

## Outreach gate

Do not convert MYC-INT-005A into an external technology recommendation merely from architecture documentation.

Before a dependency-oriented offer, require at minimum:

- exact status disclosure for every Mycelix/Luminous component named;
- relevant interoperability qualification gates resolved or explicitly presented as unqualified;
- complete I0 semantic-fidelity run;
- measured execution for the candidate stack being offered;
- a competent conventional baseline rather than a toy comparison;
- at least one executed migration/exit direction;
- licensing terms for offered components stated consistently with #884's resolved evidence;
- limitations and missing Integral decisions stated explicitly.

A contributor-facing note may invite collaboration earlier, but must call the stack a proposal/reference candidate rather than production infrastructure.

## Relationship

- #3145 / PR #3146 — adoption-neutral reference-stack proposal
- #3147 — evaluation issue
- #2945 — shared comparison methodology
- #3119 — water-system I0 workload
- #3142 — semantic seam/delivery contract
- #3143 — source-owned fact vs derived view
- #3144 — external source registry
- #884 — license/IP reconciliation

## Nonclaims

This protocol does not establish that Mycelix/Holochain, a conventional stack or a hybrid stack is preferable overall. It does not establish Integral adoption, governance legitimacy, production scalability, security certification, legal compliance, or fitness for a real infrastructure deployment. It defines evidence that can inform those later decisions without replacing them.
