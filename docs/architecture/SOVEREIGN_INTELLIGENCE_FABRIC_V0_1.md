# Sovereign Intelligence Fabric v0.1 — Reciprocal Accountability

Status: implementation contract for the v0.1 vertical slice.

This document defines the first cross-stack invariant for a privacy-preserving decision-intelligence fabric spanning Mycelix, Xenia, and Symthaea.

## Constitutional invariant: no invisible gaze

Every attempted lookup that resolves, targets, or materially evaluates an identifiable natural person MUST produce a durable accountability receipt before protected data is released.

Notification has exactly two valid modes:

1. **Immediate** — the subject can receive the receipt without avoidable delay.
2. **Delayed** — notice is placed in cryptographic escrow under an independently approved authorization with a mandatory release timestamp.

There is intentionally no permanent-secret notification state.

A delayed notice MAY be used only when a jurisdiction/deployment policy permits the stated reason, the delay is bounded, required approvals are present, and (when configured) at least one approver is outside the requesting organization. An authorization created after the lookup is invalid: secrecy cannot be retroactively blessed.

## Privacy invariant: receipts must not become a surveillance system

The accountability mechanism MUST NOT create a new globally searchable index of citizens.

`PairwiseSubjectId` is therefore a pairwise/pseudonymous routing identifier. Implementations SHOULD derive a different subject handle for each institutional relationship or disclosure domain. Full receipts SHOULD remain private to the subject, the originating institution, and authorized oversight peers. Shared/public transparency surfaces SHOULD contain commitments and aggregate statistics rather than subject identifiers, case identifiers, raw predicates, or disclosed records.

Notification routing information (email address, device token, DID endpoint, etc.) is outside the receipt schema and MUST be stored/sealed separately.

## Cross-domain placement

Reciprocal accountability is not a Civic-only concept. The same primitive must apply to health, identity, finance, commerce, enterprise operations, sensor networks, and future Mycelix domains. The canonical types therefore live in the shared `mycelix-accountability-core` crate. Civic re-exports the types as the first integration surface, but does not own the protocol.

## Receipt lifecycle

```text
requester
   |
   | signed + purpose-bound QueryCapability
   v
policy gate ---- denied -----------------------------+
   |                                                  |
 allowed / partial                                    |
   v                                                  |
local predicate / local model                         |
   |                                                  |
   +--> minimum necessary disclosure                  |
   |                                                  |
   +--> AccessReceipt <-------------------------------+
          |
          +--> commit before protected disclosure
          |
          +--> Immediate -----------------> subject ledger / notification
          |
          +--> DelayedAuthorization ------> sealed escrow
                                             |
                                             +--> mandatory release_at
                                                        |
                                                        v
                                             subject ledger / notification
```

Denied requests create receipts as well. A denied request MUST have `DisclosureKind::None` and cannot use the receipt object to smuggle result data.

## v0.1 schema

The canonical Rust schema and fail-closed validator live in:

`crates/mycelix-accountability-core/src/lib.rs`

The receipt includes:

- pairwise subject identifier;
- requesting organization, actor key/identifier, and role;
- purpose, matter/case reference, scope commitment, and purpose expiry;
- asserted legal/consensual authority and jurisdiction;
- query commitment and policy version;
- allow / partial / deny outcome;
- minimum-disclosure summary;
- query-budget charge;
- optional machine-inference disclosure;
- subject rights advertised for this decision;
- immediate or expiring delayed-notification directive;
- optional proof/attestation reference.

## Subject rights profile

A deployment can configure required rights. The recommended high-accountability profile requires all of:

- **Know** — learn that a person-linked lookup occurred;
- **Inspect** — inspect the accessible data and inference summary involved;
- **Contest** — challenge inaccurate data or an unsupported inference;
- **HumanReview** — request meaningful human review where automated reasoning has significant effect;
- **Appeal** — challenge the authorization or resulting action through the applicable process;
- **ProofOfPolicy** — obtain/verifiably check evidence that the declared policy was applied.

The validator fails closed when a policy-required right is absent from a receipt.

## Delayed-notification escrow

Delayed notice is an exception, never a bypass.

A `DelayedNotificationAuthorization` carries:

- standardized delay reason;
- legal/policy basis;
- commitment to the sealed justification;
- approval time;
- mandatory release time;
- threshold count;
- independent approval records.

The v0.1 validator rejects:

- delay when the deployment disables it;
- reasons not in the policy allow-list;
- missing authorization/basis/justification commitments;
- release at or before the lookup;
- delays beyond the deployment maximum;
- authorizations created after the lookup;
- insufficient or duplicate approvers;
- absence of independent oversight when policy requires it.

When the release timestamp is reached, evaluation returns `DeliverNow`. There is no code path that returns "withhold forever."

## Anti-mosaic / query-budget invariant

Many individually lawful low-disclosure queries can reconstruct a sensitive history. A privacy-preserving fabric therefore needs protection against composition attacks.

`QueryBudgetCharge` lets deployments charge identifiable searches against time-bounded, purpose-scoped budgets. Higher layers should additionally rate-limit by actor, organization, subject cohort, predicate family, and geographic/time scope. An unusual fan-out should be surfaced to oversight and, where safe, to affected subjects.

Aggregate analytics SHOULD use privacy-preserving aggregate mechanisms rather than resolving individuals and issuing millions of nominally independent person-linked requests.

## Inference accountability

The danger is not only access to raw records; secret inference can materially affect a person even when every source datum is correct.

`InferenceDisclosure` therefore carries a stable inference ID, model/version, plain-language summary, canonical integer confidence, explanation commitment, provenance receipt IDs, and whether the inference can have significant effect.

Future versions should support signed correction/contest records without deleting historical evidence: the system should be able to say "this inference existed, was challenged, and was corrected" rather than silently rewriting history.

## Xenia responsibility

Xenia is the secure execution/session plane, not the policy authority.

For SIF traffic it should provide:

- authenticated requester/device sessions;
- PQC-capable sealed transport where configured;
- replay protection and key rotation;
- signed operator/action provenance;
- capability-bound session context;
- delivery acknowledgement for subject notifications;
- cryptographic continuity between the lookup execution and the receipt commitment.

The receipt schema remains transport-independent. Xenia should carry canonical receipt bytes or their commitments without learning data it does not require.

## Symthaea responsibility

Symthaea is the local/federated cognition and verifiable-computation plane, not the authority that decides whether surveillance is lawful.

For SIF it should provide bounded functions such as:

- local predicate evaluation;
- local anomaly/similarity inference;
- federated learning/querying where raw source data remains local;
- minimum-necessary result generation;
- ZK/verifiable attestations that a declared predicate/model/policy computation executed as specified;
- plain-language explanations that remain linked to machine-verifiable provenance.

`AttestationRef` is the v0.1 seam: it can point at a Symthaea ZK proof, an Xenia signed execution transcript, or a composite verifier profile without coupling Mycelix domains to a specific proof backend.

## Commit-before-disclose ordering

Production adapters MUST enforce this order:

1. authenticate requester;
2. validate purpose/scope capability;
3. evaluate local predicate/model;
4. determine the minimum necessary disclosure;
5. build and validate `AccessReceipt`;
6. durably commit receipt or escrow commitment;
7. only then release protected output;
8. deliver/queue subject notice according to `NotificationDisposition`;
9. record delivery acknowledgement or retry state.

If step 5 or 6 fails, step 7 MUST fail closed.

This prevents an outage in the accountability subsystem from degrading into an unlogged-surveillance mode.

## Failure and abuse cases to test next

- requester tries to omit or weaken purpose binding;
- actor fans out a predicate across a population to reconstruct movements;
- administrator attempts to create an unbounded embargo;
- approver signs the same delay twice under multiple identities;
- requesting organization controls all purported oversight identities;
- notification service is offline during disclosure;
- subject is temporarily unreachable;
- query is denied but operator attempts to encode information in receipt metadata;
- model inference is updated after a citizen contests it;
- clock skew affects embargo release;
- receipt commitment exists but full private receipt is lost;
- policy version is revoked while an embargo is active;
- jurisdiction changes during a cross-border query;
- compromised operator key performs high-volume lookups;
- malicious subject attempts to infer protected third-party/source information from notification metadata.

## Next implementation tranches

### v0.1A — current

- cross-domain receipt/notification core;
- Civic re-export as first integration surface;
- fail-closed validator;
- delayed-notification expiry and independent-approval checks;
- subject-rights gating;
- query-budget and attestation seams;
- unit tests of the structural invariants.

### v0.1B — Mycelix persistence

- private subject access ledger;
- receipt commitment entry;
- notification delivery state machine;
- delayed-notice escrow entry + release scheduler;
- contest/correction records;
- oversight aggregate views.

### v0.1C — Xenia binding

- canonical lookup execution transcript;
- receipt/transcript commitment binding;
- sealed receipt delivery transport;
- delivery acknowledgement and replay-safe retry;
- operator/session identity mapping into `RequestingPrincipal`.

### v0.1D — Symthaea proof adapter

- canonical predicate statement;
- minimum-disclosure proof statement;
- policy-execution attestation;
- inference provenance commitment;
- verifier profile registry feeding `AttestationRef`.

### v0.2 — anti-abuse federation

- cross-node query-budget accounting;
- threshold-sensitive searches;
- independent oversight witnesses;
- anomaly detection over access patterns;
- privacy-preserving public transparency metrics;
- challenge/appeal workflows and correction propagation.

## Non-claim

This protocol is a research implementation contract, not a statement that any deployment is legally compliant. Legal bases, notice timing, secrecy restrictions, access rights, retention, court processes, and national-security rules vary by jurisdiction and require independent legal and security review.
