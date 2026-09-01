# Sovereign Intelligence Fabric v0.1

Status: **pre-alpha architecture contract**

This document defines the first cross-stack contract for privacy-preserving, person-linked queries across Mycelix, Xenia, and Symthaea.

It is not a legal-compliance claim, law-enforcement product specification, or authorization to deploy surveillance. The goal is to make accountability, minimization, and eventual subject visibility structural properties of the software.

## Constitutional invariants

1. **No invisible gaze.** Every person-linked lookup attempt creates an accountability event, including denied attempts.
2. **Notification by default.** A successful or denied person-linked lookup is subject-visible immediately unless a separately authorized delayed-notification grant is active.
3. **Secrecy can delay, never erase.** Delayed notification has an explicit authority, reason category, approver set, reveal deadline, and hard expiry. The underlying receipt remains committed while hidden and becomes subject-visible at expiry.
4. **Minimum disclosure.** Queries return the least information needed for the declared purpose. Predicate answers and attestations are preferred over raw records when sufficient.
5. **Purpose binding.** Every query is bound to a declared purpose and policy version. Results may not be silently repurposed.
6. **Proof-carrying access.** A receipt binds the query, policy decision, requester/session provenance, disclosure summary, and optional computation proof.
7. **No accessor-controlled erasure.** The requester cannot delete the fact that access occurred. Independent witness commitments are supported.
8. **Raw-data expiry is separate from receipt retention.** Expiring an observation does not erase the accountability record for prior access.
9. **Bulk reconstruction resistance.** Individually valid queries are still subject to query/privacy budgets, anomaly detection, and bulk-query policy.
10. **Right to know, inspect, contest, and prove.** Subjects can learn who queried them and why, inspect attributable records/inferences where policy permits, contest them, and verify the integrity of the access trail.

## Ownership boundaries

### Mycelix

Owns:

- query capabilities and purpose/authority policy;
- subject access receipts and notification state;
- delayed-notification authorization;
- citizen-facing access ledger and challenge/correction workflow;
- independent witness commitments and transparency aggregates;
- query/privacy budgets.

### Xenia

Owns:

- authenticated session and operator provenance;
- cryptographic channel/session identifiers;
- role-scoped authorization and revocation state;
- signed consent/operator ledger bindings;
- transport of bounded SIF envelopes when Xenia is the carrier.

Xenia does **not** own domain-specific surveillance policy.

### Symthaea

Owns:

- local/federated predicate evaluation;
- minimum-disclosure reasoning and result shaping;
- optional verifiable-computation/ZK attestations;
- privacy-preserving anomaly detection and semantic search;
- proof binding to query/policy/result commitments.

Symthaea does **not** decide whether an actor has legal or organizational authority to query a person.

## Core flow

1. A requester obtains a `QueryCapability` scoped to a purpose, authority, subject selector, allowed disclosures, time window, and budget.
2. Xenia (when used) authenticates the operator/session and emits a provenance binding.
3. Mycelix evaluates authorization and budget policy before data access.
4. A local data holder evaluates the predicate. Raw data remains local unless disclosure is explicitly authorized.
5. Symthaea may produce a minimum-disclosure result plus an optional proof/attestation.
6. Mycelix emits a `SubjectAccessReceipt` for **every attempt**, denied or allowed.
7. The receipt is immediately subject-visible unless covered by a valid `DelayedNotificationGrant`.
8. Witness nodes may commit to the receipt digest without learning protected contents.
9. When a delay expires, the receipt becomes visible automatically; expiry cannot be renewed implicitly.

## Receipt fields

A v0.1 receipt binds at least:

- `receipt_id`
- `query_id`
- requester principal/organization commitment
- subject commitment or pseudonymous subject key
- declared purpose
- authority reference
- policy version/digest
- decision (`allowed`, `denied`, `partial`)
- disclosure summary (fields/categories, not necessarily values)
- query/result commitment
- requester/session provenance binding
- optional computation proof binding
- creation time
- notification state and reveal time
- retention class
- previous receipt/witness commitment when used as an append-only chain

## Delayed notification

Delayed notification is a first-class authorization object, not a boolean flag.

It must contain:

- grant id;
- query or narrowly scoped query-class binding;
- authority reference;
- reason category;
- independent approver identities/commitments;
- issuance time;
- `reveal_after`;
- hard expiry;
- policy digest;
- signature/threshold-signature binding.

A grant cannot delete a receipt, suppress witness commitments, or create an indefinite concealment period. Extensions must be explicit new grants linked to the previous grant and subject to policy limits.

## Query/privacy budgets

Authorization is necessary but not sufficient. The policy engine tracks query volume by requester, organization, purpose, subject class, geography, and time window. This is intended to prevent population reconstruction through many individually valid requests.

Budget exhaustion produces a denied receipt and can trigger an anomaly event.

## Minimum disclosure examples

Prefer:

- `match = true` over returning a full historical record;
- `count = 7` with differential privacy where appropriate over exposing seven identities;
- `credential satisfies predicate` over returning the credential;
- `observation existed in bounded interval` over returning unrelated observations around it.

Escalation to raw evidence should require a capability whose allowed-disclosure set explicitly includes that evidence class.

## Citizen rights surface

The subject-facing API should eventually expose four explicit operations:

- **Know** — access receipts concerning the subject.
- **Inspect** — attributable records/inferences and provenance, subject to narrowly defined protections.
- **Contest** — submit a correction/challenge tied to a record, inference, or receipt.
- **Prove** — verify receipt integrity, policy/provenance commitments, and available computation proofs.

A contested inference must not be silently overwritten. Corrections should preserve lineage: original assertion, challenge, adjudication, and current status.

## Threats explicitly in scope

- insider lookup abuse;
- stalking/ex-partner/celebrity lookups;
- silent purpose drift;
- bulk-query population reconstruction;
- unauthorized cross-jurisdiction sharing;
- deletion/tampering of audit history;
- indefinite secrecy orders;
- over-disclosure when a predicate answer is sufficient;
- compromised/revoked operator credentials;
- model/inference errors that materially affect a subject.

## Threats not solved by this document alone

- compromised endpoint hardware/firmware;
- coercive or unlawful policy itself;
- side-channel leakage from implementations;
- malicious sensor fabrication;
- physical-world abuse after legitimate disclosure;
- correctness of legal authority references.

These require separate threat models, independent audits, governance, and deployment controls.

## v0.1 implementation sequence

1. Add shared Mycelix SIF protocol types and deterministic canonical hashing.
2. Add property/unit tests for notification and delay invariants.
3. Add Xenia `SessionProvenanceBinding` adapter without domain policy in `xenia-wire`.
4. Add Symthaea `VerifiableQueryBinding`/minimum-disclosure attestation adapter.
5. Implement an in-memory reference policy engine and receipt ledger.
6. Add a two-agent integration test: allowed lookup, denied lookup, delayed notification expiry, revoked requester, and budget exhaustion.
7. Add independent witness commitment test.
8. Only then build subject and operator UI surfaces.

## Non-goals for v0.1

- nationwide camera-network deployment;
- facial recognition;
- covert bulk surveillance;
- automated punitive decisions;
- claims of compliance with any jurisdiction;
- replacing existing evidentiary/legal processes.
