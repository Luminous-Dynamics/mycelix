# MYC-INT-006A — Adoption-neutral Mycelix reference-stack proposal for Integral

Status: architecture proposal / evaluation aid. No claim of Integral adoption, production readiness, or component qualification.

## Purpose

Prepare a technically falsifiable option for Integral's pending Phase-0 technology-stack decision without asking Integral to adopt Mycelix governance, economics, or implementation choices wholesale.

Integral remains the semantic owner of CDS, OAD, ITC, COS, FRS, their governance rules, contribution/accounting rules, certification rules, ecological rules, and federation policy.

The proposal is intentionally modular:

```text
Integral domain semantics
        |
        v
versioned semantic contracts
        |
        +-------------------------------+
        |                               |
        v                               v
Mycelix interoperability            alternative
reference components               implementation
        |
        v
optional Holochain runtime
        |
        v
optional operations/security tooling
```

Core theorem:

```text
Integral compatibility
!= Mycelix dependency
!= Holochain dependency
!= Luminous-stack dependency
```

## External source basis

Observed 2026-09-27:

- Integral Technical Specifications: `https://integralcollective.io/documents/specifications.html`
- Integral Development Guide v0.1: `https://integralcollective.io/documents/integral_devguide_v01.pdf`
- Integral White Paper v0.1: `https://integralcollective.io/documents/whitepaper.html`
- Integral Decision Record: `https://integralcollective.io/documents/decisions.html`

The builder-facing Technical Specifications currently mark the technology-stack decision `SPEC-STACK-01` as `PENDING`, alongside pending interface specifications for OAD→COS, COS→ITC, and FRS→CDS. The Development Guide names Holochain as a conceptually aligned potential component while explicitly not committing to it.

This document must be revised if those source statuses change.

## Offer profile A — protocol only

Integral retains its own database, transport, runtime, deployment system, and UI.

Mycelix contributes only reusable semantic contracts and conformance tests where they survive independent-domain qualification:

- schema-qualified semantic references;
- evidence/provenance composition;
- recommendation / decision / authorization / execution separation;
- outcome/review lineage;
- foreign-authority firewall;
- source-owned fact versus derived-view discipline;
- versioned semantic seam and delivery semantics;
- loss-aware translation receipts;
- external adapter/conformance fixtures.

This is the lowest-coupling offer and should remain useful even if Integral rejects Holochain or every other Luminous runtime component.

## Offer profile B — reference node

Reference implementation of one Integral node using:

```text
Integral domain modules
+ Mycelix semantic/interoperability contracts
+ Holochain reference persistence/federation profile
+ replaceable application client
```

Holochain is an implementation profile, not the semantic identity of the system.

Required boundary:

```text
Integral object identity
!= Holochain action/hash identity
```

Holochain action/hash identities may provide provenance and runtime addressing, but exported Integral objects must retain transport-neutral semantic identity and source lineage.

Before this profile can be recommended for a pilot, it must demonstrate:

- local node operation under the selected connectivity profile;
- partition/disconnection behavior;
- reconnection and reconciliation;
- backup and recovery;
- schema evolution;
- bounded storage growth;
- debugging and operator observability;
- deterministic import/export of externally meaningful data;
- explicit authority behavior across federated nodes.

## Offer profile C — full optional operations profile

Profile B plus independently qualified optional Luminous components where they materially improve the deployment:

- Xenia security/remote-operations components;
- Nix/NixOS reproducible environments and deployment;
- durable outbox/idempotency/recovery components already owned elsewhere in Mycelix;
- evidence-capture and qualification tooling;
- optional Symthaea analytics.

These are not bundle requirements.

A conforming Integral node must not fail semantic conformance merely because it uses another operating system, transport, UI, analytics engine, or deployment mechanism.

## Symthaea boundary

Symthaea may later provide:

- simulation;
- scenario analysis;
- diagnostic assistance;
- forecasting;
- constraint analysis;
- recommendation generation;
- engineering/model evaluation.

But:

```text
Symthaea output != direct observation
Symthaea output != Integral decision
Symthaea output != authorization
Symthaea output != effect
```

Symthaea should not be required for the first reference node.

## Requirements traceability matrix

This matrix records candidate mappings, not approval or readiness.

| Integral requirement / design direction | Candidate Mycelix/Luminous support | Current classification | Required evidence before external claim |
| --- | --- | --- | --- |
| explicit cross-system seam contracts | MYC-INT semantic references + #3142 seam profile | Designed / partially implemented | executable conformance tests across >=2 Mycelix domains + external mock |
| schema/interface versioning | `SchemaRef`, source registry, translation receipts | ImplementedUnqualified / Designed | #3137 PASS, #3139 composition, drift fixtures |
| append/event idempotency | existing durable-effect/idempotency lines | Mixed existing lineage | exact reusable owner identified + conformance fixture |
| FRS recommendation does not become governance authority | authority anti-collapse line + #3119 | Designed | hostile recommendation→effect rejection |
| OAD retains design authority when consuming FRS evidence | source-domain ownership + #3143 | Designed | source-owned-state fixtures |
| federation without semantic/policy convergence | #3120 federation seam | Designed | two-policy federation test |
| Holochain-style node autonomy | optional Holochain profile | CandidateMapping | node/partition/recovery benchmark |
| meaningful local operation when disconnected | Holochain/local-first candidate profile | CandidateMapping | explicit per-module offline contract and measured test |
| no mandatory central semantic authority | Mycelix heterogeneous federation model | Designed | multi-node fixture with independent local authority |
| tamper-evident evidence/history | existing provenance/evidence owners | Existing but fragmented | exact ancestry and qualification census |
| transport/provider non-authority | existing LEX/integration lines + #3142 | Existing/Designed | selected runtime composition test |
| implementation replaceability | protocol/runtime separation | Designed | second implementation or mock conformer |
| export/migration | SemanticRef + runtime-neutral fixture corpus | Gap/Designed | round-trip export/import fixture |
| licensing clarity | #884 | Gap | reconciled component-by-component license schedule |

Do not convert this table into a scalar readiness score.

## Conventional vs Mycelix/Holochain vs hybrid

The external proposal should compare at least three architectures.

### Conventional local service stack

Example shape:

```text
Rust/another language
+ relational database
+ HTTP/gRPC
+ event/outbox infrastructure
+ conventional deployment
```

Likely strengths:

- common developer skills and tooling;
- mature observability and SQL analysis;
- straightforward local transactions;
- well-understood backup/recovery patterns;
- easier hiring/onboarding.

Questions to test:

- how federation avoids turning one service/database into a de facto authority;
- how independently governed nodes synchronize without a privileged central API;
- how offline operation and later reconciliation work;
- how append-only semantic history and authority lineage are preserved.

### Mycelix + Holochain reference node

Potential strengths to test:

- closer conceptual fit to node autonomy;
- agent-centric/distributed data model;
- reduced dependence on one central database/API;
- natural test bed for local authority and heterogeneous federation semantics.

Risks to measure rather than hand-wave:

- smaller contributor/tooling ecosystem;
- greater conceptual complexity;
- debugging/observability burden;
- operational and recovery maturity;
- schema/data migration difficulty;
- integration cost with conventional infrastructure.

### Hybrid

Example:

```text
conventional local application services
+ explicit Mycelix semantic contracts
+ optional Holochain/federation edge
```

Potential benefit: use familiar local persistence/analytics while keeping federation and cross-domain meaning behind explicit contracts.

Potential cost: two persistence/consistency worlds can increase complexity unless the ownership boundary is extremely clear.

The POC should measure these tradeoffs instead of assuming a winner.

## Replaceability / exit contract

A credible offer requires an exit path before adoption.

Required theorem:

```text
replace runtime component
without redefining Integral domain meaning
```

At minimum:

1. Integral-facing schemas remain versioned and public.
2. Runtime-specific IDs are not the sole external semantic identity.
3. Export includes source namespace, schema version, object identity, lineage/provenance, and relevant status/history.
4. Historical records retain the semantics under which they were created.
5. A second conforming implementation can import the exported corpus without Mycelix private state.
6. Unsupported/lossy conversion is declared, never silently normalized.
7. Replacing Holochain, Xenia, Nix, Leptos, or Symthaea does not require changing Integral governance/economic rules.

## First POC

Use #3119's water-system vertical slice rather than attempting an entire Integral node.

Minimum demonstration:

```text
observation/evidence
-> issue
-> alternatives/objections
-> non-executive analytical recommendation
-> Integral-shaped deliberation adapter
-> decision
-> separate bounded authorization
-> implementation receipt
-> outcome observation
-> review/supersession candidate
```

Run the same semantic substrate with one non-Integral governance adapter.

Required hostile cases:

- recommendation attempts direct execution;
- imported decision attempts local authorization;
- duplicate transport delivery;
- timeout with unknown recipient state;
- stale schema/interface version;
- external certification treated as locally accepted;
- derived summary treated as source-owned current fact;
- partition/reconnect conflict;
- expired/revoked authorization;
- outcome contradiction attempts automatic decision mutation.

## Metrics

Collect evidence for:

- setup steps and setup time;
- build/test cycle complexity;
- source/dependency footprint;
- event/record latency;
- partition and reconnect behavior;
- recovery procedure complexity;
- storage growth;
- memory/CPU footprint on a modest node;
- number of operator actions during upgrade/recovery;
- observability/debugging quality;
- export/import completeness;
- implementation-specific versus portable code percentage.

No claim such as "simpler", "more resilient", or "more decentralized" should be made without an explicit measured or qualified basis.

## External-offer gates

Before asking Integral to depend on the stack rather than merely evaluate it, disclose and resolve as applicable:

- #3137 qualification state;
- #3139 semantic-root composition;
- #3140/#3144 source/schema fixture status;
- #3142 seam/delivery status;
- #3119 E2E qualification;
- #3147 fair evaluation protocol and at least one executed I0 generation;
- second non-Integral adapter;
- selected security/runtime qualification evidence;
- installation/upgrade/recovery documentation;
- #884 licensing/IP reconciliation.

Current repository licensing must not be summarized from one file or one code header. #884 already records inconsistent license surfaces and owns that reconciliation.

## Suggested contributor-facing framing

A future submission to Integral should be framed approximately as:

> Mycelix is offered as a modular reference implementation and interoperability toolkit for evaluation against Integral's Phase-0 requirements. Integral retains ownership of its domain semantics and governance. The proposal can be evaluated in layers, from protocol-only conformance through an optional Holochain reference node, with explicit export/replaceability boundaries and published qualification status for every reused component.

This is not yet the submission itself. It is the internal standard the submission must satisfy.

## Relationship

- #3114 — compatibility contract
- #3119 / MYC-INT-005A — water-system neutral adapter proof
- #3120 — heterogeneous federation
- #3122 — adoption-neutral adapter SDK
- #3139 — composition with existing EPI/provenance roots
- #3140/#3144 — Integral source/schema fixture program
- #3142 — semantic seam/delivery contract
- #3143 — source-owned fact vs derived summary
- #3145 — MYC-INT-006A planning issue for this proposal
- #3147 — MYC-INT-006B fair evaluation protocol
- #884 — licensing/IP reconciliation

## Nonclaims

This document does not establish that Integral should select Mycelix, Holochain, Xenia, Nix, Leptos, or Symthaea; that any component is production-ready; that Holochain is technically superior to a conventional or hybrid stack; that Integral endorses this proposal; or that a pending/draft Integral specification may be treated as ratified.