# Sovereign Intelligence Fabric v0.1 — Reciprocal Accountability

Status: implementation contract for the v0.1 vertical slice.

This document defines the first cross-stack invariant for a privacy-preserving decision-intelligence fabric spanning Mycelix, Xenia, and Symthaea.

## Constitutional invariant: no invisible gaze

Every attempted lookup that resolves, targets, or materially evaluates an identifiable natural person MUST produce a durable accountability receipt before protected data is released.

Notification has exactly two valid modes:

1. **Immediate** — the subject can receive the receipt without avoidable delay.
2. **Delayed** — notice is placed in cryptographic escrow under an independently approved authorization with a mandatory release timestamp.

There is intentionally no permanent-secret notification state.

Delayed notice MAY be used only when a jurisdiction/deployment policy permits the reason, the delay is bounded, required approvals are present, and—when configured—at least one approver is outside the requesting organization. Authorization created after the lookup is invalid: secrecy cannot be retroactively blessed.

## Privacy invariant: receipts must not become a surveillance system

The accountability mechanism MUST NOT create a new globally searchable index of citizens.

`PairwiseSubjectId` is a pairwise/pseudonymous routing identifier. Implementations SHOULD derive a different subject handle for each institutional relationship or disclosure domain. Full receipts SHOULD remain private to the subject, originating institution, and authorized oversight peers. Shared/public transparency surfaces SHOULD contain commitments and aggregate statistics rather than subject identifiers, case identifiers, raw predicates, or disclosed records.

Notification routing information (email address, device token, DID endpoint, etc.) is outside the receipt schema and MUST be stored/sealed separately.

## Cross-domain placement

Reciprocal accountability is not a Civic-only concept. The same primitive applies to health, identity, finance, commerce, enterprise operations, sensor networks, and future Mycelix domains. Canonical types live in `crates/mycelix-accountability-core`. Civic re-exports them as the first integration surface but does not own the protocol.

## Evidence is plural

A single signature or ZK proof must not be allowed to stand in for unrelated security claims. v0.1 therefore models evidence by role:

- `ExecutionBinding` — authenticated requester/session/action and signed evidence frontier; Xenia is the natural producer.
- `ComputationProof` — evidence that a declared predicate/model computation produced the committed minimum-necessary result; Symthaea is the natural producer.
- `PolicyProof` — evidence that a declared policy program or policy decision was evaluated as specified.
- `ExternalWitness` — independent ordering/timestamp/oversight evidence strengthening the claim that accountability evidence existed before disclosure.

`AccountabilityPolicy.required_attestation_roles` decides which roles a deployment requires. A high-assurance deployment can require several independently verified roles instead of trusting one omnibus artifact.

## Two-phase receipt lifecycle

The receipt commitment intentionally excludes later proof references. That breaks the otherwise circular dependency in which Xenia/Symthaea would need to prove a receipt that already contains the digest of the proof being created.

```text
requester
   |
   | authenticated, purpose-bound capability
   v
policy gate ---- denied ------------------------------+
   |                                                   |
 allowed / partial                                     |
   v                                                   |
local predicate / model                                |
   |                                                   |
   +--> minimum necessary result                       |
   |                                                   |
   +--> semantic AccessReceipt <-----------------------+
          |
          +--> validate_pre_attestation_receipt()
          |
          +--> pre_attestation_receipt_commitment()
                       |
             +---------+-----------------+
             |                           |
             v                           v
       Xenia execution              Symthaea computation
       binding proof                proof / attestation
             |                           |
             +------------+--------------+
                          |
                     attach refs
                          |
                    validate_receipt()
                          |
             durable commit / witness
                          |
                   protected disclosure
                          |
              immediate notice OR escrow
                          |
                    mandatory release
```

Denied requests create receipts too. A denied request MUST have `DisclosureKind::None` and cannot use receipt metadata to smuggle result data.

## v0.1 schema

Canonical Rust schema and fail-closed validators live in:

`crates/mycelix-accountability-core/src/lib.rs`

The receipt includes:

- pairwise subject identifier;
- requesting organization, actor identifier, role, and authenticated cryptographic source ID;
- purpose, matter/case reference, scope commitment, and purpose expiry;
- asserted legal/consensual authority and jurisdiction;
- query commitment and policy version;
- allow / partial / deny outcome;
- minimum-disclosure summary;
- query-budget charge;
- optional machine-inference disclosure;
- subject rights advertised for this decision;
- immediate or expiring delayed-notification directive;
- zero or more typed proof/attestation references during construction, followed by all policy-required evidence roles before disclosure.

All cross-stack security commitments use fixed 32-byte BLAKE3 commitments in v0.1 rather than free-form digest strings.

## Subject rights profile

A deployment can configure required rights. The recommended high-accountability profile requires:

- **Know** — learn that a person-linked lookup occurred;
- **Inspect** — inspect accessible data and inference summaries involved;
- **Contest** — challenge inaccurate data or unsupported inference;
- **HumanReview** — request meaningful human review where automated reasoning has significant effect;
- **Appeal** — challenge the authorization or resulting action through the applicable process;
- **ProofOfPolicy** — obtain or verifiably check evidence that the declared policy was applied.

The validator fails closed when a policy-required right is absent.

## Delayed-notification escrow

Delayed notice is an exception, never a bypass. `DelayedNotificationAuthorization` carries a standardized reason, legal/policy basis, sealed-justification commitment, approval time, mandatory release time, threshold count, and independent approval records.

The validator rejects disabled/unpermitted delay, missing authorization material, invalid release windows, delays beyond policy maximum, authorization created after lookup, insufficient/duplicate approvers, or missing independent oversight when required. At `release_at_ms`, evaluation returns `DeliverNow`; no code path means “withhold forever.”

## Safe citizen notice projection

The internal receipt and the initial notification are deliberately different objects. `SubjectNotice` can expose the institution, role, purpose, authority category, disclosure summary, rights, safe inference summary, delay history, and commitment-only proof references without copying raw query commitments, matter/case IDs, approver identities, or sealed investigative justification into a push-notification channel.

A deployment may reveal more during authenticated `Inspect`/`Contest` workflows according to law and policy; the initial notification should remain minimum-necessary in both directions.

## Anti-mosaic / query-budget invariant

Many individually lawful low-disclosure queries can reconstruct a sensitive history. `QueryBudgetCharge` lets deployments charge identifiable searches against time-bounded, purpose-scoped budgets. Higher layers should additionally rate-limit by actor, organization, subject cohort, predicate family, and geographic/time scope. Unusual fan-out should be surfaced to oversight and, where safe, affected subjects.

Aggregate analytics SHOULD use privacy-preserving aggregate mechanisms instead of resolving individuals and issuing millions of nominally independent person-linked requests.

## Inference accountability

The danger is not only raw-data access: secret inference can materially affect a person even when every source datum is correct. `InferenceDisclosure` carries a stable inference ID, model/version, plain-language summary, canonical integer confidence, explanation commitment, provenance receipt IDs, and whether the inference can have significant effect.

Future versions should support signed correction/contest records without deleting historical evidence: “this inference existed, was challenged, and was corrected” is preferable to silently rewriting history.

## Xenia responsibility

Xenia is the secure execution/session plane, not the policy authority. For SIF it should provide authenticated requester/device sessions, PQC-capable sealed transport where configured, replay protection/key rotation, signed action provenance, capability-bound session context, delivery acknowledgements, and cryptographic continuity between the authenticated execution and the semantic receipt commitment.

The semantic requester includes an authenticated 32-byte source ID so Mycelix and Xenia can prove they are referring to the same principal without sending citizen identity or case contents into Xenia.

## Symthaea responsibility

Symthaea is the local/federated cognition and verifiable-computation plane, not the authority that decides whether surveillance is lawful. For SIF it should provide bounded local predicate evaluation, anomaly/similarity inference, federated computation where source data remains local, minimum-necessary result generation, verifiable computation/ZK attestations where supported, and explanations linked to machine-verifiable provenance.

Its initial integration target is `AttestationRole::ComputationProof`: bind the same operation/query/result/pre-attestation-receipt commitments used by the other layers without pretending that the proof itself establishes legal authority.

## Commit-before-disclose ordering

Production adapters MUST enforce this order:

1. authenticate requester;
2. validate purpose/scope capability;
3. evaluate the local predicate/model;
4. determine minimum-necessary disclosure;
5. build semantic `AccessReceipt`;
6. run `validate_pre_attestation_receipt`;
7. compute the pre-attestation receipt, purpose, query, policy, and result commitments;
8. obtain required Xenia/Symthaea/policy/witness evidence and attach `AttestationRef`s;
9. run final `validate_receipt` and verify the referenced proof bodies;
10. durably commit or independently witness the receipt/evidence bundle;
11. only then release protected output;
12. deliver or escrow subject notice and record delivery/retry state.

If steps 5–10 fail, disclosure MUST fail closed. A signature alone proves commitment, not wall-clock ordering, so durable or externally witnessed pre-disclosure commitment is part of the high-assurance design.

## Failure and abuse cases to test next

- requester omits or weakens purpose binding;
- actor fans out a predicate across a population to reconstruct movement;
- administrator attempts an unbounded embargo;
- approver signs the same delay twice under multiple identities;
- requesting organization controls all purported oversight identities;
- notification service is offline during disclosure;
- subject is temporarily unreachable;
- denied query encodes information in receipt metadata;
- model inference changes after citizen contest;
- clock skew affects embargo release;
- receipt commitment exists but private receipt is lost;
- policy version is revoked while an embargo is active;
- cross-border jurisdiction changes during a query;
- compromised operator key performs high-volume lookups;
- malicious subject infers protected third-party/source information from notice metadata;
- proof reference is valid but belongs to a different receipt/query/result;
- execution proof exists but computation proof is missing;
- valid signature is produced only after protected output was already disclosed.

## Next implementation tranches

### v0.1A — current Mycelix core

- cross-domain receipt/notification core;
- pairwise identities and safe citizen notice projection;
- two-phase semantic/final validation;
- multi-proof evidence roles;
- delayed-notification expiry and independent-approval checks;
- subject-rights gating;
- query-budget accounting seam;
- canonical domain-separated commitments;
- Civic re-export and focused CI gate.

### v0.1B — Mycelix persistence

- private subject access ledger;
- receipt commitment entry;
- notification delivery state machine;
- delayed-notice escrow + release scheduler;
- contest/correction records;
- oversight aggregate views.

### v0.1C — Xenia binding

- authenticated session/requester/ledger authorization binding;
- receipt/query/purpose/policy/result commitment binding;
- sealed receipt delivery transport;
- delivery acknowledgement and replay-safe retry;
- independently witnessed pre-disclosure ordering.

### v0.1D — Symthaea proof adapter

- canonical computation statement;
- minimum-disclosure result binding;
- model/predicate commitment;
- receipt and Xenia execution-binding linkage;
- verifier-profile adapter feeding `AttestationRole::ComputationProof`.

### v0.2 — anti-abuse federation

- cross-node query-budget accounting;
- threshold-sensitive searches;
- independent oversight witnesses;
- anomaly detection over access patterns;
- privacy-preserving public transparency metrics;
- challenge/appeal workflows and correction propagation.

## Non-claim

This protocol is a research implementation contract, not a legal-compliance certification. Legal bases, notice timing, secrecy restrictions, access rights, retention, judicial process, national-security rules, and evidentiary requirements vary by jurisdiction and require independent legal/security review.
