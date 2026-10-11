# Payment Adapter Qualification Matrix v1

**Status:** reference specification and effects-disabled failure corpus  
**Created:** 2026-10-11  
**Related work:** [FIN-ISO20022-001 / #2432](https://github.com/Luminous-Dynamics/mycelix/issues/2432), [MYC-ECO-004 / #3345](https://github.com/Luminous-Dynamics/mycelix/issues/3345), [MYC-ECO-005 / #3348](https://github.com/Luminous-Dynamics/mycelix/pull/3348), [banking control plane / #4466](https://github.com/Luminous-Dynamics/mycelix/pull/4466)

## 1. Decision

Use one provider-neutral qualification corpus for the external-payment boundary, then bind every deployment to an exact provider/product and versioned profile. Do not create a universal `PaymentRail` that assumes card networks, bank messaging, instant-payment systems, and internal mutual-credit settlement share the same semantics.

This artifact complements the existing ISO 20022 airlock, settlement-finality model, and Finance conformance profile. It is deliberately narrower than those architecture documents: it freezes failure sequences and expected dispositions that a deterministic simulator or adapter test harness can execute later.

The companion machine-readable corpus is [payment-adapter-qualification-v1.json](./payment-adapter-qualification-v1.json). It contains 27 scenarios and requires no live credentials or real-money effects.

## 2. Canonical boundary

Every payment-related adapter must preserve the distinction between:

```text
Economic intent
  -> authority / authorization
  -> execution attempt
  -> external observation
  -> rail-specific finality evaluation
  -> reconciliation
  -> economic outcome projection
```

These stages are separate. In particular:

- parsing a message does not authenticate or authorize its sender;
- authorizing a payment does not prove that it was submitted;
- submission or provider acceptance does not prove booking or settlement;
- a provider status is evidence interpreted through a versioned rail profile, not universal finality;
- reconciliation must compare amount, unit, instrument, source/destination, fees, identity, and causal references;
- a return, reversal, dispute, or correction is a successor event. It does not erase the source event;
- a multi-rail workflow is not atomic unless the rail capabilities and coordinator prove the requested atomic semantics.

The canonical financial journal remains authoritative only within the source of authority it explicitly represents. Mycelix must not claim authority over a bank's actual books merely because it stores an observation of them.

## 3. Adapter families and integration boundaries

| Adapter family | First contract to qualify | Required non-claim |
|---|---|---|
| Bank payment messaging (ISO 20022 / SWIFT-connected institution) | Exact message definition, usage profile, sender/authentication context, correlation IDs, status/return mapping, and bank-specific reconciliation evidence | Message validity or positive status alone does not prove settlement finality |
| Visa Direct payout | Exact enrolled Visa Direct product profile, request and callback schema, reference mapping, status/return handling, query/recovery behavior | A payout notification is not a generic card-purchase lifecycle or universal finality proof |
| Card acceptance / acquirer / processor | Authorization, capture, clearing/settlement evidence, refund, dispute, and chargeback lifecycle | Authorization is not capture; capture is not necessarily settled proceeds; chargeback does not by itself prove fraud |
| Regional or instant-payment rail | Named country, rail, participant/provider, supported currencies, eligibility rules, actual API or message contract, and finality evidence | A regional initiative, political grouping, or announced interconnection is not an executable endpoint |
| Mycelix internal or mutual-credit settlement | Instrument identity, obligation direction, authorization, conservation, idempotency, causal evidence, and reconciliation | Internal credit, reputation, SAP, TEND, bank deposits, and external money are not interchangeable |
| Unresolved named product (including “PandaCard”) | Verified product identity, legal provider, official technical documentation, jurisdiction, settlement counterparties, and a test/sandbox contract | A product name alone does not establish that a card network, issuer, processor, API, or settlement rail exists |

SWIFT and Visa are not adapters by themselves: each deployment needs the applicable service/product, institutional access, and the exact versioned technical profile. Likewise, do not treat “BRICS” as one payment API. Integrate an actual participant system or provider connection at a time, and record the legal and operational context for each route.

## 4. Time-sensitive ISO 20022 guard

The ISO 20022 catalogue is versioned and should be consulted for the exact message definition and associated usage guidance selected by a deployment.

As checked on **2026-10-11**, Swift's guidance for the November 2026 milestone states that after **14 November 2026**, unstructured postal addresses will no longer be accepted in the cross-border space; hybrid or fully structured addresses are required for the relevant Swift CBPR+ profile. This is intentionally represented as a profile-scoped test rather than a global rule for every ISO 20022 implementation.

The adapter must not “fix” an address by inventing structure. Invalid or insufficiently structured input must be rejected or held for correction before dispatch, with the selected rule/profile version and validation evidence recorded. A date-sensitive policy must use an injected test clock in deterministic fixtures rather than the wall clock.

References:
- [ISO 20022 message definitions catalogue](https://www.iso20022.org/iso-20022-message-definitions)
- [Swift: call to action for November 2026](https://www.swift.com/standards/iso-20022/iso-20022-bytes/call-action-november-2026)

## 5. Visa Direct and card lifecycle guard

Visa Direct Connect documents product-specific APIs for supported payout flows and includes payout status and return notifications for certain products. Qualification must pin the actual product, endpoint, request and callback versions, authentication method, identifier constraints, and recovery/query behavior used by the integration. Re-read the relevant provider reference at implementation time; do not freeze an undocumented assumption from a display label.

Visa's published [error-code reference](https://developer.visa.com/capabilities/visa-direct-connect/error-codes) distinguishes a Client Reference ID that conflicts with a previously processed payout and a request whose amount/currency differs from that earlier transaction. Scenario `PAY-VISA-005` binds this to the idempotency boundary: preserve the original operation identity, reject the conflicting payload, and query/reconcile the original provider state rather than silently treating the changed request as another payment.

**Provider idempotency keys are not universally permanent.** Stripe documents that keys may be pruned after at least 24 hours and that reusing a pruned key can initiate a new request. Adyen documents company-account scope, a 7–14 day retention window, and no duplicate-key check across simultaneous regional endpoints. These contracts differ, so an adapter must pin key namespace, retention, concurrent-request behavior, and query/recovery support for the exact provider/product. The new scenario `PAY-IDEM-001` forbids dispatching under a fresh key while the old operation remains unresolved after the provider's key-retention window expires or is unknown. A persistent Mycelix operation record does not extend the provider's deduplication guarantee.

The corpus explicitly tests:

- a transport timeout after a provider may have accepted a payout;
- repeated delivery of the same authenticated status notification;
- a return arriving after an earlier status;
- provider-reference mapping that would require invalid or colliding identifiers;
- an authorization that has not yet been captured;
- a capture timeout;
- a dispute/chargeback after a previously recorded transaction.

For unknown outcomes, first query or reconcile the original logical operation using the provider's supported reference and recovery contract. Do not issue a new logical payment merely because the caller did not receive a response.

References:
- [Visa Direct Connect](https://developer.visa.com/capabilities/visa-direct-connect)
- [Visa Direct Connect API reference](https://developer.visa.com/capabilities/visa-direct-connect/reference)
- [Stripe idempotent requests](https://docs.stripe.com/api/idempotent_requests)
- [Adyen API idempotency](https://docs.adyen.com/development-resources/api-idempotency)

## 6. Required durable evidence

Every adapter-generated receipt or event should reference, as applicable:

- exact source system, provider/product, jurisdiction and profile/version;
- stable logical operation ID and idempotency key identity (the reusable secret/key itself must not be exposed);
- request/message content digest and external transaction/reference ID;
- exact asset/instrument identity, currency, unit scale, instructed and observed amounts;
- authorization/policy scope and the evidence used to validate it;
- authenticated provider event, timestamp and freshness/replay decision;
- provider lifecycle state, finality profile, and the narrow claim supported by the evidence;
- reconciliation comparison, discrepancies, correction links, and unresolved status;
- causal links between original event, return, reversal, refund, dispute or compensation.

Do not place API secrets, bearer tokens, private keys, seed phrases, full payment-card numbers, or other reusable authentication material in test fixtures, receipts, logs, or portable evidence bundles. Store hashes/references where appropriate, with access-controlled protected material outside the corpus.

## 7. Qualification workflow — no CI wait required

1. **Freeze profile inputs.** Pin provider/product, message/API version, jurisdiction, currency/instrument profile, capability profile, finality profile, and policy snapshot.
2. **Run effects-disabled cases.** Use deterministic fake provider responses and an injected clock. Do not call a live endpoint, and do not use credentials.
3. **Assert dispositions.** Check expected state, dispatch eligibility, event cardinality, amount/instrument preservation, and forbidden claims. A logged warning is not a pass.
4. **Inject crash and retry points.** Include response loss before acknowledgement, after possible acceptance, after partial multi-leg execution, and after receiving duplicate/out-of-order callbacks.
5. **Independently inspect evidence.** Validate the corpus and receipts with a verifier that does not reuse the producer's decision result as its own proof.
6. **Qualify sandbox behavior.** Separately obtain partner/provider sandbox access and verify signatures, actual payloads, status queries, returns, timeouts, idempotency and reconciliation against the exact contract.
7. **Gate production per product.** Require security review, credential scope, operational monitoring, incident/recovery runbook, legal/compliance review and provider approval. A mock or sandbox pass is not production qualification.

The JSON corpus is a reference artifact. This branch also includes the standard-library-only verifier at `tools/finance/verify_payment_adapter_corpus.py`. From the repository root, run `python tools/finance/verify_payment_adapter_corpus.py --self-test` to validate the corpus and exercise thirteen negative mutation tests. The verifier checks schema, unique case IDs, required fields, HTTPS source references, and critical timeout/finality/return/partial-effect plus payment-claim concurrency, sender isolation, and Treasury recovery contracts. A separate effects-disabled in-memory reference runner is provided at `tools/finance/payment_adapter_lifecycle_smoke.py`; run `python tools/finance/payment_adapter_lifecycle_smoke.py` to exercise twelve deterministic lifecycle scenarios: lost response after provider acceptance, status-versus-finality, partial multi-rail effects, return lineage, and conflicting idempotency payloads, 64-way exact-claim replay, conflicting concurrent fingerprints, Treasury receipt replay after a simulated status-write failure, rejection of unanchored effect IDs, and 40 concurrent attempts near the `u64::MAX` balance boundary and same-key isolation across two sender identities, and same-key recovery after a simulated Holochain chain-head-moved rollback. These scripts do not call a provider, exercise the production Rust coordinator, or move money. The new lightweight Payment Adapter Qualification workflow runs syntax, corpus mutation, and lifecycle/concurrency checks automatically for changes to these artifacts; the simulations are still not wired to a Rust test target and are not production integration tests. The additional concurrency and chain-head cases are thread-safe in-memory models only; they do not establish Rust/Holochain transaction isolation, SweetConductor/Tryorama behavior, or provider-side guarantees. Those remain distinct follow-on qualification stages.

## 8. Exit criteria

Do not call the adapter qualified until:

- all applicable corpus cases have explicit, reviewed dispositions;
- duplicate retries cannot create duplicate value-moving effects;
- external accept/timeout ambiguity resolves through query or reconciliation rather than blind resubmission;
- amount, currency, instrument and reference mapping preserve exact semantics;
- returns, reversals and disputes retain causal history;
- partial multi-rail effects remain visible and recoverable;
- status-to-finality claims are limited to the bound finality profile;
- the independent verifier can reproduce each claimed disposition from frozen inputs and evidence;
- production claims are backed by provider-specific evidence rather than this reference corpus.

**Claim ceiling:** this artifact defines adapter qualification expectations. It does not establish a live SWIFT, Visa, BRICS-related, PandaCard, bank, processor or central-bank integration; legal settlement finality; licensing; regulatory compliance; custody; solvency; or production readiness.
