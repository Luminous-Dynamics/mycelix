# Sovereign Intelligence Fabric v0.1 — Accountable Access

Status: **experimental architecture / first protocol tranche**

This document defines the first cross-project contract for a privacy-preserving intelligence fabric spanning Mycelix, Xenia, and Symthaea.

## Design objective

Provide useful operational intelligence without making indiscriminate centralized collection the default architecture.

The first enforceable property is **No Invisible Gaze**:

> Any lookup intentionally resolving an identifiable/linkable person creates a durable access receipt and a subject-visible notification. Immediate notice is the default. Delayed notice requires separate authorization and has a mandatory release deadline.

The rule applies to direct identifiers and sufficiently linkable pseudonyms. Systems MUST NOT evade it by translating a person to a plate, device ID, account ID, embedding, face template, or other stable linkable identifier immediately before the query.

## Data-flow boundary

```text
                       CONTROL / AUTHORITY PLANE
        Mycelix identity + policy + governance + subject rights
                              |
                              v
                     QueryCapability (signed)
                              |
                              v
+------------------------- LOCAL DATA DOMAIN --------------------------+
| sensor / hospital / enterprise / civic / infrastructure data         |
|                                                                      |
|  Symthaea/local engine -> predicate or bounded result                 |
|                  |                                                   |
|                  +-> proof / attestation reference                   |
+------------------|---------------------------------------------------+
                   v
             AccessDecision
                   |
                   v
              AccessReceipt <---- Xenia session provenance
                   |
          +--------+---------+
          |                  |
          v                  v
   immediate notice     authorized delayed notice
          |                  |
          +--------+---------+
                   v
          subject access ledger
                   |
          inspect / contest / prove
```

Raw data SHOULD remain in the originating administrative/data domain whenever a bounded query or proof can satisfy the legitimate purpose.

## Cross-project ownership

### Mycelix owns

- subject / requester identity bindings and credentials;
- policy and purpose registries;
- access-receipt persistence adapters;
- notification routing and citizen-facing access ledger;
- delayed-notice governance and threshold approval policy;
- correction / contest / appeal records;
- public aggregate transparency without exposing sensitive case contents.

### Xenia owns

- authenticated session establishment and peer identity;
- replay-resistant sealed transport;
- cryptographic transcript/session provenance;
- operator role and revocation state;
- capability delivery bindings so a capability cannot be replayed in a different session/context unnoticed.

### Symthaea owns

- local/federated query execution and semantic reasoning;
- privacy-preserving predicate evaluation where feasible;
- ZK/verifiable-computation adapters for supported computations;
- anomaly detection over access patterns;
- purpose-drift and inference-risk analysis.

No project may silently assume another layer's responsibility. In particular, a valid Xenia session is **not** authorization; a Mycelix credential is **not** proof that a computation was performed correctly; and a Symthaea inference is **not** permission to disclose its underlying data.

## Rights model

### Right to Know
A subject can determine that a person-linked lookup occurred, who/which organization requested it, its declared purpose and authority, the policy used, the decision, and what class of information was disclosed.

### Right to Inspect
A subject can obtain the subject-visible information and material inferences held about them, subject to narrowly governed temporary withholding.

### Right to Contest
Every delivered notification has a stable route for correction, justification request, appeal, or delegated review.

### Right to Proof
The system can attach verifiable references for authorization, policy decision, session provenance, receipt integrity, and—where supported—computation correctness.

## Immediate versus delayed notification

`Immediate` is the normal mode.

`Delayed` is valid only when all of the following are present:

- machine-readable justification code;
- distinct authority reference;
- one or more accountable approvers, with deployment policy free to require a threshold;
- cryptographic authorization/proof reference;
- creation timestamp;
- scheduled release timestamp;
- hard mandatory-release timestamp.

The durable receipt exists independently of notification visibility. A delayed-notification authorization changes **when the subject sees the receipt**, not whether the receipt exists.

Extensions, if introduced in a later protocol version, MUST themselves be new auditable authorization events and MUST NOT mutate away the original deadline/history.

## Denied and abusive attempts

A denied person-linked query still creates a receipt. It must disclose no subject data to the requester.

Repeated denied searches, unusual requester/subject relationships, excessive use of delay powers, purpose changes, or suspicious query bursts are inputs to an abuse-detection layer. Detection signals MUST NOT automatically become punitive findings without appropriate review.

## Bulk and aggregate analysis

The protocol distinguishes `PersonLinked` and `AggregateOnly` capabilities.

Aggregate analysis is not a bypass mechanism. Aggregate executions:

- have no person-query budget;
- cannot request individual evidentiary artifacts;
- should use minimum cohort/privacy thresholds and differential privacy where appropriate;
- must be promoted into the person-linked path before resolving or disclosing an individual.

Future versions should add explicit privacy-budget, minimum-cohort, and re-identification-risk fields rather than overloading the person query budget.

## Inference rule

A lookup does not cease to be person-linked merely because the requested value is inferred rather than stored.

Examples that should enter the accountable path when tied to a person include risk scores, inferred relationships, predicted affiliation, behavioral similarity, face/voice identity matches, and cross-dataset entity resolution.

A later tranche should model `InferenceReceipt` separately so subjects can distinguish raw facts from machine-generated assertions and challenge the provenance/model behind them.

## Threat model for v0.1

The design explicitly considers:

1. **Authorized insider curiosity** — valid operator identity but illegitimate purpose.
2. **Purpose laundering** — declaring a permitted purpose while using results for another.
3. **Bulk reconstruction** — many individually small queries used to reconstruct population behavior.
4. **Delayed-notice abuse** — secrecy powers used as a general bypass.
5. **Identifier laundering** — converting a person into a stable pseudonym/device/vehicle identifier to avoid subject rights.
6. **Receipt suppression** — successful lookup without durable audit creation.
7. **Disclosure escalation** — returning more fields/artifacts than the capability allows.
8. **Credential/session replay** — reusing authority in a different time/session/context.
9. **Inference opacity** — consequential machine inference with no provenance or contest path.
10. **Ledger privacy failure** — making sensitive lookup metadata globally readable in the name of transparency.

v0.1 provides type-level/local validation against several classes but does not by itself solve persistence, collusion, compromised endpoints, side channels, traffic analysis, malicious models, coercive lawful authority, or cryptographic key compromise.

## Required next tranches

### v0.2 — Mycelix persistence adapter

- Holochain integrity/coordinator types for receipts and notification state;
- subject-indexed private/controlled retrieval rather than public sensitive content;
- append-only receipt commitment / tombstone rules;
- notification delivery queue;
- contest record and response lifecycle;
- multi-agent Sweettest coverage.

### v0.3 — Xenia capability/session binding

- canonical capability digest;
- transcript binding / domain separation;
- expiry and replay enforcement;
- operator revocation check at execution time;
- signed receipt/session provenance fixture shared with Mycelix.

### v0.4 — Symthaea proof adapter

- portable `ComputationAttestationRef` mapping;
- predicate-only query example;
- verifiable-search/ZK adapter where already supported;
- anomaly scoring that cannot itself silently expand disclosure.

### v0.5 — Privacy budgets and aggregate safety

- per-purpose/per-requester query budgets;
- cohort thresholds;
- differential-privacy budget accounting;
- composition/re-identification controls;
- rate-limited entity resolution.

### v0.6 — Citizen access UX

- immediate lookup notice;
- delayed notice after expiry;
- inspect provenance and disclosure;
- contest/correct/delegate review;
- human-readable explanation generated from machine-readable receipt fields without replacing the canonical receipt.

## Acceptance gate for v0.1

Before merge:

- `cargo test --manifest-path crates/mycelix-sovereign-access/Cargo.toml` passes in the repository's supported environment;
- protocol types serialize/deserialize deterministically enough for the selected canonical signing format (JSON roundtrip alone is not a canonical-signing guarantee);
- reviewers confirm no API permits a person-linked receipt to omit a notification policy;
- reviewers confirm delayed notification cannot exceed its mandatory release timestamp;
- documentation continues to state that authority/proof references require external cryptographic verification.

## Non-claim

This architecture is a research design. It is not a certification of compliance with any surveillance, privacy, criminal-procedure, health, intelligence, employment, or data-protection law. Deployments must define jurisdiction-specific authority and subject-rights policy separately, and should obtain independent security, privacy, legal, and civil-liberties review.
