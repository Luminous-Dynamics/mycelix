# MYC-INT-007A — Integral architecture contribution package

Status: research-backed proposal for external technical contribution. This document does not select Integral policy, governance, economics, or technology stack.

## Purpose

Offer technically useful architecture work to Integral that remains valuable even if Integral never adopts Mycelix, Holochain, Xenia, Symthaea, or any Luminous Dynamics runtime.

The governing boundary is:

```text
helping Integral specify its architecture
!= asking Integral to adopt Mycelix

external proposal
!= Integral decision

prototype
!= evidence

evidence
!= authority
```

## Source status

Research snapshot: 2026-09-27.

Primary public Integral sources consulted:

- Integral Development Guide v0.1;
- Integral Technical Specifications page;
- Integral Decision Record;
- current CDS/OAD/ITC/FRS system descriptions;
- Integral white paper/federation material.

Source families are not collapsed. A white-paper statement, a development-guide proposal, a Technical Specification, a Decision Record entry, and an implementation are different authority/status classes.

```text
white-paper concept
!= development-guide implementation proposal
!= ratified Technical Specification
!= implementation
!= conformance evidence
```

## Highest-value immediate contribution: complete the interface map

The current Development Guide says the minimum architecture contains twelve primary cross-system data contracts. The currently published Technical Specifications page exposes three Phase-0 interface contracts as `PENDING`:

- `SPEC-IF-01` — `OAD -> COS`;
- `SPEC-IF-02` — `COS -> ITC`;
- `SPEC-IF-03` — `FRS -> CDS`.

The other nine Development Guide seams should not be assumed missing by mistake, intentionally deferred, already ratified, or unnecessary. Until Integral assigns statuses, the safe description is `NO_CURRENT_PUBLIC_SPEC_ID_OBSERVED`.

### Twelve-contract coverage matrix

| # | Development Guide contract | Producer | Consumer | Current public SPEC-IF observed | Key semantic boundary |
|---|---|---|---|---|---|
| 1 | Certified Design Package | OAD | COS | `SPEC-IF-01` — PENDING | certified design authorizes/grounds production planning; COS does not redefine certification |
| 2 | Design Intelligence Signal | OAD | ITC | none observed | OAD design intelligence informs ITC valuation; ITC does not become design authority |
| 3 | Design Event Signal | OAD | FRS | none observed | design/certification changes become monitoring input; FRS does not own certification |
| 4 | Operational Recalibration | FRS | OAD | none observed | FRS provides evidence/review triggers; OAD retains design authority |
| 5 | Labor and Materials Record | COS | ITC | `SPEC-IF-02` — PENDING | COS source activity remains distinguishable from ITC weighting/accounting |
| 6 | Operational Signal | COS | FRS | none observed | COS operational facts remain distinct from FRS interpretation |
| 7 | Credit and Access Signal | ITC | FRS | none observed | ITC-owned state is consumed by FRS without FRS reconstructing/owning it |
| 8 | Sensemaking Artifact | FRS | CDS | `SPEC-IF-03` — PENDING | recommendation/evidence remains non-executive governance input |
| 9 | Governance Signal | CDS | FRS | none observed | decision/activity history feeds review; FRS does not retroactively redefine the decision |
| 10 | Design Mandate | CDS | OAD | none observed | CDS authorizes/constrains; OAD performs domain design/certification work |
| 11 | Production Mandate | CDS | COS | none observed | CDS defines authorized envelope; COS self-organizes execution inside it |
| 12 | Policy Signal | CDS | ITC | none observed | CDS sets normative parameters; ITC applies them without silently originating policy |

### Recommended matrix fields

For each seam Integral could maintain a machine-readable and human-readable record containing:

- source family and source version;
- contract name/ID;
- producer;
- consumer;
- data/source owner;
- authority direction;
- object/schema IDs and versions;
- delivery guarantees;
- acknowledgement semantics;
- retry/idempotency semantics;
- currentness/staleness semantics;
- error behavior;
- privacy/minimization requirements;
- compatibility/deprecation rules;
- conformance fixture;
- status (`draft`, `pending`, `ratified`, `deprecated`, etc.);
- unresolved questions.

Exit criterion: every minimal seam is either represented by a versioned specification or explicitly marked unresolved/deferred by Integral itself.

## Recommendation 1 — interface evolution policy

Integral already recognizes that schema evolution can damage history and interoperability. The contribution package should add an explicit compatibility policy rather than relying only on predicting future nullable fields.

Cover at least:

- backward-compatible additive changes;
- incompatible schema generation changes;
- producer/consumer version skew;
- unknown-field behavior;
- deprecation/support windows;
- stale-generation rejection;
- historical replay under original schema;
- translation/migration receipts;
- exact source bytes versus normalized/query projections.

Recommended invariant:

```text
new schema generation
!= silent reinterpretation of old object
```

Future fields that can be predicted should be planned, but the protocol should remain robust when future requirements were not predicted.

## Recommendation 2 — evidence and oracle-integrity profile

The Development Guide explicitly identifies verification/oracle integrity as an unresolved hard problem. This deserves its own architecture profile rather than being left to per-module judgment.

Preserve at minimum:

```text
ActorReport
!= Observation
!= Measurement
!= DerivedFinding
!= Recommendation
!= Decision
```

Every consequential derived finding should be able to carry:

- source references;
- source kind;
- acquisition/observation time;
- source-owner identity where appropriate;
- derivation lineage;
- currentness/staleness;
- confidence semantics defined by the source domain;
- independence/correlation metadata where multiple sources are combined;
- contradictory evidence references;
- known missing evidence.

### Adversarial oracle fixtures

Test:

- biased self-report;
- stale sensor values;
- duplicated evidence represented as independent evidence;
- correlated sources represented as independent;
- selective omission;
- contradictory physical measurements;
- source-owned current state reconstructed from a partial derived view;
- valid cryptographic provenance attached to false content.

Important theorem:

```text
verified provenance
!= truth

source agreement
!= source independence

high confidence label
!= execution authority
```

## Recommendation 3 — bounded emergency authority

The Development Guide explicitly identifies the tension between governance velocity and crisis response and suggests pre-approved emergency mechanisms with sunset behavior.

A useful protocol shape is:

```text
EmergencyCondition
        ↓
pre-authorized trigger profile
        ↓
bounded EmergencyAuthorization
        ↓
ImplementationAttempt(s)
        ↓
receipts + outcome observations
        ↓
automatic expiry
        ↓
mandatory review
        ↓
ratify / supersede / repudiate / remediate
```

Required negative controls:

- emergency detection alone does not mint unlimited authority;
- trigger scope and effect scope are explicit;
- scope cannot silently widen after activation;
- expiry is machine-checkable;
- renewal creates a new authorization lineage;
- later review cannot erase prior emergency effects/history;
- FRS detection/recommendation remains advisory unless Integral explicitly ratifies a different authority rule.

## Recommendation 4 — governance attention budget

Integral correctly identifies democratic/cognitive overload as a hard problem. The safer contribution is to measure and reduce mechanical burden without giving algorithms extra governance authority.

Measure:

- issues introduced per participant/time period;
- median deliberation time burden;
- unresolved blocking-objection age;
- duplicate/redundant issue rate;
- percentage of routine operations inside pre-ratified envelopes;
- frequency of envelope boundary hits requiring CDS review;
- reopened/superseded decision rate;
- proportion of participant time spent on mechanical compilation versus judgment/deliberation.

Preserve:

```text
automation reduces administrative work
!= automation receives governance authority
```

Automate aggregation, formatting, retrieval, routing, and bounded policy evaluation wherever possible. Keep diagnosis, contested interpretation, and normative judgment explicitly visible.

## Recommendation 5 — preregister decision intent and review

Before implementation, a CDS decision should be able to bind:

- intended outcome(s);
- assumptions;
- constraints;
- affected scope;
- known dissent/trade-offs;
- evidence expected later;
- review trigger/window;
- amendment/supersession route.

Then:

```text
later FRS observation
-> evaluate against preregistered decision context
```

rather than:

```text
later outcome
-> redefine what the original decision was supposed to achieve
```

This improves institutional learning without making FRS the authority over historical decisions.

## Recommendation 6 — federation privacy/disclosure budget

Integral's federation model intentionally preserves node autonomy while sharing outward-facing state and diagnostic intelligence. Every federated object should therefore define a disclosure budget.

Record:

- purpose;
- minimum fields;
- granularity;
- retention;
- audience;
- linkability/pseudonymity properties;
- correction/revocation semantics where meaningful;
- aggregation mechanism;
- known inference leakage;
- whether raw participant-level data is prohibited.

Avoid the false theorem:

```text
aggregated or anonymized
== private
```

Privacy claims should be evidence-bearing and mechanism-specific.

## Recommendation 7 — federation recognition is not authority

Cross-node interoperability should preserve distinctions such as:

```text
foreign identity recognized
!= foreign credential accepted
!= foreign certification accepted
!= foreign decision authoritative locally
!= foreign ITC contribution recognized locally
```

For each federated artifact, make explicit:

- who issued it;
- what its original scope was;
- what the receiving node verifies;
- what the receiving node recognizes;
- whether recognition creates any local authority;
- whether a local policy/delegation is required;
- how recognition can be superseded/revoked without rewriting historical evidence.

This preserves node autonomy while still allowing federation.

## Recommendation 8 — participant-facing legibility and contestability

The Development Guide correctly treats black-box behavior as an adoption failure, not merely a UX inconvenience.

A useful `Why did this happen?` contract for CDS/ITC/COS/FRS outputs should expose:

- relevant source inputs;
- rule/policy version;
- computed/derived steps;
- uncertainty or missing evidence;
- which component was advisory;
- who/what had authority;
- what effect occurred;
- what remains unknown;
- how a participant can contest/correct/review it.

Do not treat machine-generated explanations as proof of legitimacy or usability. Test with actual participants and report sample/population limitations.

## Recommendation 9 — adversarial simulation before real-node operation

Before real-world deployment, test at least:

- duplicate/reordered/missing messages;
- persisted-before-ack timeout;
- stale source summaries;
- conflicting node states;
- malicious or biased self-report;
- unavailable peers/source systems;
- authorization-expiry race;
- consensus-input manipulation/collusion;
- emergency-power persistence attempt;
- OAD certification revoked after COS production starts;
- ITC cross-node recognition disagreement;
- FRS false positive / false negative;
- schema-version skew;
- source/current-state reconstruction from incomplete local data;
- privacy inference from aggregated federation output.

Results should be recorded as failures, limitations, unsupported behavior, or evidence—not collapsed into a single architecture score.

## Recommendation 10 — small formal assurance targets

After semantics stabilize, selected machine-checkable invariants could be formally verified or model checked.

Candidate properties:

- recommendation cannot directly create execution authority;
- expired authorization cannot admit a new effect;
- historical decisions are superseded, not destructively rewritten;
- source schema/version survives translation/federation;
- duplicate delivery does not create duplicate logical effect;
- emergency authorization cannot survive expiry without a new explicit authorization;
- foreign artifact cannot gain local authority without explicit local recognition/delegation.

Avoid claiming formal proof of broad social properties such as fairness, democratic legitimacy, desirability, trust, or ecological adequacy.

## What Mycelix can contribute without becoming the stack

Mycelix can contribute reusable artifacts behind these independent Integral contracts:

- schema/version identity primitives;
- evidence/provenance profiles;
- translation receipts;
- authority/non-authority separation;
- conformance corpora;
- neutral candidate protocols;
- federation/admission profiles;
- exact qualification/evidence discipline.

Integral could consume those ideas/artifacts while implementing them in PostgreSQL, another conventional stack, Holochain, or something else.

## What Mycelix can optionally contribute as an implementation

Only after Integral-facing contracts exist independently:

- Mycelix protocol/reference node;
- Holochain persistence/federation profile;
- PostgreSQL conventional/reference service;
- hybrid PostgreSQL projection + Mycelix/Holochain federation profile;
- Xenia secure transport/effects integration where relevant;
- Symthaea simulation/analysis as a strictly advisory FRS-style component.

Important boundary:

```text
Integral contract
!= Mycelix implementation
!= Holochain runtime
!= Luminous-stack dependency
```

## Suggested contribution sequence

1. twelve-contract coverage/traceability matrix;
2. schema/interface evolution policy;
3. evidence/oracle-integrity profile;
4. bounded emergency-authorization profile;
5. synthetic cross-system conformance corpus;
6. federation privacy/recognition profile;
7. participant-legibility contract;
8. adversarial simulation pack;
9. optional Mycelix and conventional reference implementations;
10. comparative evidence and migration/exit proof.

This ordering improves Integral even if step 9 is never accepted.

## Nonclaims

This document does not endorse or oppose Integral's economic or governance choices, does not select a technology stack, does not decide unresolved policy questions, and does not claim Mycelix is the preferred implementation. It identifies technical specification and assurance work that Integral contributors can evaluate through Integral's own decision process.
