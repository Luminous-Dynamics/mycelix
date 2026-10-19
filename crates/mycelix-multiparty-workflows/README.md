# mycelix-multiparty-workflows

Portable append-only workflow primitives for processes in which independent actors must contribute decisions without mutating one another's records.

## Core rule

> An actor never mutates another actor's record to express their decision.

Requester, issuer, trustee, subject, and execution authority actions are represented as independently authored immutable events. Current state is a deterministic projection over the valid causal event graph.

## Why this exists

Several real workflows are inherently multi-party:

- a credential requester submits a request, but the issuer must independently review, approve/reject, and issue;
- social recovery trustees independently approve or reject a recovery operation;
- the subject may cancel a recovery before execution;
- a narrowly authorized recovery executor applies the final key change only after threshold and timelock requirements are met.

Representing those processes as cross-agent updates creates an author-binding conflict in Holochain: the actor who owns the original entry is not necessarily the actor authorized to make the next decision. This crate replaces that model with immutable event authorship.

## Event graph invariants

`EventEnvelope` carries a process ID, author, timestamp, causal parents, payload, and proof reference.

Projection rejects:

- duplicate event IDs;
- cross-process events;
- self-parenting or duplicate causal parents;
- non-root events without parents;
- references to unknown parents;
- timestamps earlier than causal parents;
- causal cycles/disconnected branches;
- protocol-version drift.

Event arrival order is not authoritative. Concurrently-ready events are ordered deterministically by `(occurred_at_ms, event_id)`, so two nodes receiving the same valid event set derive the same projection.

## Credential workflow

The requester authors `Requested`. The issuer independently authors `ReviewStarted`, `Approved`/`Rejected`, and `CredentialIssued`.

Important bindings:

- request has an explicit issuer, policy reference, exact request digest, and expiry;
- approval must bind the exact original request digest;
- issuance is only valid after approval;
- issuance must name the approval event as a causal parent;
- issued credential carries a non-zero credential digest;
- requester cannot forge issuer decisions by reusing the requester's original record.

## Recovery workflow

A recovery policy binds:

- subject;
- unique trustee set;
- majority-or-stronger threshold;
- minimum timelock;
- explicit execution-authority allowlist;
- policy reference;
- digest of the exact canonical policy document.

Trustees author their own one-time approval/rejection events. The subject may independently cancel. Execution is valid only when:

1. the approval threshold was reached;
2. the minimum timelock expired;
3. the executor is explicitly authorized by policy;
4. the target key matches the initiated recovery target;
5. the execution event causally references at least the threshold number of trustee approval events;
6. the host verifies the execution proof reference.

## Holochain adapter requirements

This crate deliberately contains no HDK/HDI dependency. A Holochain adapter should:

- bind `EventEnvelope.author` to the action author / agent DID in integrity validation;
- make workflow events immutable (reject update/delete);
- verify proof references rather than treating a non-empty string as cryptographic validity;
- validate policy and credential references against their owning zomes;
- index events by process without introducing a mutable shared state entry;
- feed the complete valid event set into the pure projection functions;
- treat ambiguous/missing causal history as failure, not graceful degradation.

## Relationship to institutional core

`mycelix-multiparty-workflows` models safe collaboration mechanics. `mycelix-institutional-core` models institutional authority, jurisdiction, rulebooks, evidence, action requests, challenges, appeals, and remedies. Holochain adapters should eventually map workflow decisions into institutional `ActionEvent` / `Decision` evidence rather than creating a second authority model here.

## Non-goals

- cryptographic signature verification;
- canonical policy serialization;
- Holochain persistence/indexing;
- W3C VC issuance implementation;
- key rotation implementation;
- legal claims about identity or recovery policy.

Those belong to integrating layers. The purpose of this crate is to make the state machine and authorship invariants portable and testable before storage-specific code is introduced.
