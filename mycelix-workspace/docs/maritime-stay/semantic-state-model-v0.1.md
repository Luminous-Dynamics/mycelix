# Maritime Stay Protocol v0.1 — semantic state model proposal

**Status:** proposed verifier semantics for review; not yet adopted as a normative protocol amendment.
**Parent:** Mycelix #5918
**Verifier owner:** Mycelix #5921
**Case corpus:** `semantic-cases-v0.1.json`
**Architecture extension:** Mycelix #5920

## 1. Purpose

The v0.1 JSON Schema defines message shape. This note defines the cross-message state semantics that a future independent verifier should evaluate.

The most important model distinction is:

`provider effect state != client observation state`

A provider may durably commit a berth while the response is lost. The client cannot infer that the effect failed, and it cannot claim success until provider-authoritative evidence is reconciled. The provider's physical/contractual state and the network's knowledge of that state must remain separate.

This is a verifier contract proposal, not an implementation or test result.

## 2. Four separate state dimensions

Do not encode all system knowledge in one status field.

### A. Provider-side effect state

| State | Meaning |
|---|---|
| NotApplied | Provider has established that the requested effect was not applied |
| Applied(effect_ref) | Provider has established that the effect was durably applied |
| Rejected(reason) | Provider rejected the request under its authority |
| Conflict(reason) | Provider found a key/payload or resource-state conflict |
| ProviderUnknown | Provider itself cannot establish the outcome |

Only the provider or an independently recognized provider-authorized evidence source can establish these states.

### B. Requester/orchestrator observation state

| State | Meaning |
|---|---|
| NotSent | No request was submitted |
| Sent | Request was sent; no provider outcome is known |
| ResponseObserved | A response was received, but its authority/effect semantics still need validation |
| ReceiptObserved | A provider-attributed receipt was observed and linked to the exact operation |
| NeedsReconciliation | Outcome cannot be resolved from current evidence |
| Reconciled | A new provider-authoritative observation resolved the outstanding operation |

A timeout after sending leaves the observer at NeedsReconciliation, not Failed.

### C. Evidence disposition

For each input/receipt, report separately:
- structurally valid / invalid;
- semantically consistent / inconsistent;
- issuer attributed / not attributed / unknown;
- issuer locally recognized / not recognized / not assessed;
- current / stale / revoked / unknown;
- applicable to the exact subject, operation and purpose / not applicable / unknown.

The v0.1 schema contains a `provider_ref` string and an integrity digest, but no signed issuer envelope, key reference, verification method or revocation evidence. Therefore a v0.1 message-only verifier can check internal consistency and digest shape, but cannot independently prove that the provider actually issued the message. Treat provider attribution as an external prerequisite/disposition, not an inference from the string field.

### D. Aggregate stay view

The `StayManifest` is a derived snapshot at its `generated_at` time. It is not a new provider authority and must not be silently mutated when later evidence arrives. A later reconciliation produces a new manifest/view over a newer evidence frontier.

## 3. Proposed time semantics

### Resource intervals

Resource-use periods are half-open:

`[start, end)`

The resource is occupied at start and not occupied at end. Two consecutive allocations may meet at the boundary without overlapping.

### Offer and availability expiry

**Proposed v0.1 decision for review:** treat `expires_at` and `valid_until` as exclusive upper bounds.

For a provider-authoritative evaluation time (t):

`observed_at <= t < valid_until`

and:

`issued_at <= t < expires_at`

At exact equality with the upper bound, the snapshot or offer is expired. This aligns expiry with half-open interval semantics and prevents a resource/hold from being admitted at the instant it ceases to be valid.

This choice still needs explicit adoption in #5918. Until then, SEM-022 remains `not_evaluable_v0_1`; it must not be treated as an executed test.

### Clock qualification

Timestamps are claims, not proof that clocks were accurate.

- Compare timestamps only under a declared clock/time-source profile.
- Keep provider event time, source observation time, message issue time, webhook/delivery time, client receive time and local evaluation time distinct.
- A verifier must not let a client-supplied clock decide whether a provider hold was still valid at the provider allocation boundary.
- If the time uncertainty interval crosses an expiry boundary, disposition is Unknown/NeedsReconciliation rather than optimistic acceptance.
- Any permitted clock skew must be explicit, bounded and included in the verifier profile; there is no universal assumed tolerance in v0.1.

## 4. Provider operation and idempotency semantics

The protocol does not guarantee globally exactly-once message delivery.

The narrower provider-owned property is:

`same provider + same idempotency scope + same key + same canonical request -> original effect reference`

`same provider + same idempotency scope + same key + different canonical request -> Conflict`

`lost response -> NeedsReconciliation`

### Required operation record (conceptual)

For an operation, a provider implementation should durably bind:
- provider identity and endpoint/security profile;
- authenticated actor or delegated requester;
- operation kind;
- resource and stay-intent/component subject;
- idempotency key and scope;
- canonical request digest under a pinned canonicalization profile;
- original provider effect/receipt reference;
- provider revision/sequence where supplied;
- outcome and event time.

The v0.1 message schema does not carry all these fields. The verifier must identify which checks are possible from v0.1 and which require provider-specific evidence. Do not invent missing actor, signature, key, revision or request-digest facts.

### Crash windows

| Failure point | Provider state | Client observation | Correct disposition |
|---|---|---|---|
| Request not durably applied | NotApplied if provider can establish this | timeout or error | Reconcile; do not assume |
| Provider durably commits; response lost | Applied | NeedsReconciliation | Unknown until provider receipt/state recovered |
| Response created; delivery lost | Applied | NeedsReconciliation | Retry same key; retrieve original effect |
| Client receives response; manifest update crashes | Applied | ReceiptObserved | Replay local projection idempotently |
| Provider is unavailable after restart | Unknown to client | NeedsReconciliation | Stay/component Unknown |
| Provider returns original durable effect receipt | Applied | Reconciled | Derive current component/manifest state from receipt and currentness |

The retry path must not create a second provider effect for the same provider-scoped operation. A client log or HTTP 2xx alone is not evidence of a durable provider-side effect.

## 5. Component state and aggregate manifest derivation

Each manifest component retains its own state and provider evidence. Aggregate state must be derived from mandatory components; optional components remain visible but do not block mandatory completion.

### Proposed aggregate precedence

Evaluate at the manifest's declared as-of time and exact evidence frontier.

1. **Disputed** — a mandatory component has an unresolved dispute that affects whether the commitment is valid.
2. **Unknown** — a mandatory component's current state, issuer attribution, commitment validity or required receipt cannot be established.
3. **Confirmed** — every mandatory component has a current, provider-attributed, semantically linked commitment and required effect evidence.
4. **PartiallyCancelled** — at least one previously committed mandatory component is cancelled/released while other mandatory components remain committed.
5. **PartiallyConfirmed** — at least one mandatory component is committed and at least one other mandatory component is known pending, rejected, failed or not yet committed.
6. **Cancelled** — all mandatory components are cancelled/released and no mandatory commitment remains active.
7. **Expired** — all mandatory components ended because their required holds/offers expired before sufficient commitments were established.
8. **Pending** — no mandatory component is committed and at least one required provider operation remains in flight.
9. **Draft** — no provider operation has been requested.

This is a proposal to resolve precedence; #5918 must adopt/revise it before the verifier treats it as normative.

### Optional components

- An optional component's failure/unknown state does not block aggregate Confirmed if all mandatory components are confirmed.
- The optional component's actual state must still be displayed.
- An optional component must never be silently relabelled mandatory or have a requirement removed from the stay intent.

### Historical manifests

A Confirmed manifest means only: all mandatory components satisfied the declared confirmation predicate at that manifest's as-of time and evidence frontier.

It does not prove:
- current berth availability after manifest generation;
- navigation safety, port-entry permission, pilotage or border clearance;
- physical service delivery;
- payment capture/settlement;
- absence of later cancellation, disruption, dispute or revocation.

## 6. Price, invoice and payment are separate state machines

`quote != accepted obligation != invoice != payment instruction != payment finality`

The v0.1 Money object represents a quoted/committed amount in integer minor units. A matching ProviderCommitment price does not prove payment was made. An EffectReceipt with operation_kind=refund is not automatically proof that a payment processor finalized a refund.

The verifier should:
- recompute line-item arithmetic under an explicit tax/discount/rounding profile;
- require one currency within each arithmetic total;
- compare accepted and committed quote revisions;
- preserve amendments and fee disclosure;
- report payment/refund as Unknown until a recognized payment-provider/accounting finality reference is supplied.

Do not turn a stay into Confirmed/paid merely because a price field exists. Reservation confirmation and payment state remain distinct attributes.

## 7. Capacity and concurrency boundary

For v0.1, the provider's own allocation boundary is the only canonical authority for scarce capacity.

For a capacity-one resource and two overlapping requests, at most one may be committed by that provider revision/allocation authority.

This does not prove that multiple disconnected provider replicas can safely allocate the same resource. Offline multi-writer reservation requires a separately qualified, non-overlapping allocation-right/lease scheme with fencing and expiry. That belongs to #5920 and Symtropy's resource owners, not this v0.1 verifier.

Compound resources such as connection count + instantaneous W + cumulative Wh also require the versioned resource-bundle profile; do not overload the current single-dimension AvailabilitySnapshot.

## 8. Cross-standard and provider-event boundaries

A provider webhook or external port-call event can be:
- transport-authenticated;
- schema-conforming;
- correctly translated;
- current or stale;
- locally recognized or not;
- useful evidence for a decision.

These are separate checks.

In particular:
- a signed webhook notification is not a fresh availability response;
- S-131 service metadata is not a live berth inventory;
- DCSA Port Call 2.0 conformance is not berth availability or port authority;
- IAPH PCO planned timestamps are not DCSA actual timestamps;
- a provider's marina reservation is not Port Control permission.

## 9. Verifier implementation split

### Message-only checks available in v0.1

- JSON/schema shape (under a pinned JSON Schema implementation);
- timestamp syntax and internal ordering relationships;
- message cross-reference existence and type;
- provider/resource/intent/hold/commitment/receipt linkage as asserted in supplied messages;
- integer minor-unit arithmetic;
- manifest completeness and aggregate-state consistency under frozen rules;
- duplicate identity and payload conflicts when the relevant identifiers/inputs are present.

### Checks requiring external or provider context

- issuer signature/authentication and key status;
- local recognition/capability and endpoint authorization;
- current provider inventory/commitment revision;
- provider clock truth and clock uncertainty;
- real-world resource availability/physical capacity;
- actual payment/refund finality;
- fulfilment/physical delivery;
- legal permission, port-entry, navigation and safety authority.

The verifier should emit a typed disposition per claim rather than one `verified=true` flag.

## 10. Proposed decision checklist for #5918

Before promoting these semantics into a normative v0.1 verifier:
- adopt or revise exclusive `valid_until`/`expires_at` boundaries;
- define the trusted provider time and permitted clock-skew profile;
- freeze which provider workflows require a ReservationHold;
- define amendments and supersession versus cancellation/rebooking;
- freeze canonical request serialization and idempotency scope;
- define the exact issuer-authentication/recognition input boundary;
- approve the aggregate state precedence above;
- decide which price/fee/tax rounding rules belong in v0.1 versus a later finance profile.

**Execution status:** design proposal only. No Rust verifier, model checker, protocol conformance or runtime behavior has been executed by this document.
