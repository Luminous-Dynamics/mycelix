# South African Request-to-Pay Profile v1

**Status:** synthetic reference and qualification contract; no live provider integration, payment execution or legal conclusion.

## Purpose

Model a payment request as a distinct message and consent lifecycle. A request asks a payer to consider initiating a payment; it is not itself a payment instrument, debit or settlement event. The benchmark binds request presentation and payer acceptance to an immutable digest of the exact request terms.

## Fundamental state separation

`created → delivered → displayed → accepted | declined | expired | cancelled`

Only a valid payer acceptance that matches the exact immutable request digest, request version, beneficiary, amount, currency, reference, fee disclosure and expiry may create a payment instruction.

`payer acceptance → payment instruction → provider outcome → recipient availability → settlement/finality qualification`

Each is a distinct evidence transition. A request delivered, viewed, accepted, declined or expired is not proof of funds movement. Even when a provider describes payment as occurring immediately on acceptance, Mycelix must preserve the independently observable payment lifecycle and must not infer settlement/finality.

## South African source-version finding

Capitec's current dedicated product page, observed 4 November 2026, lists R100 per request and up to two requests per 24 hours with an R200 combined cap. The page describes request acceptance followed by payment. It does not clearly state the effective date for the displayed limits.

Capitec's 19 February 2025 blog explainer states a different R500 per-request ceiling and older fee bands. Both snapshots are preserved separately. The evaluator must not silently combine them or treat the old explainer as current. Where the effective date is unknown or a product-term conflict affects admission, quarantine and requalify with the provider.

## International design comparators

- EPC SEPA Request-to-Pay v4.0 is a messaging functionality, not a payment instrument. It has been effective since 5 October 2025 and provides separate payee/provider, inter-provider and payer/provider message flows using ISO 20022 implementation guidelines.
- Pay.UK Request to Pay is a secure request and communication overlay: the bill payer may pay by their preferred method, query the biller, pay part, pay later or decline. The framework says a response does not change legal obligations.
- NPCI directed UPI P2P Collect to stop by 1 October 2025. Do not use P2P Collect as an unqualified current competitor. Merchant flows and other UPI request mechanisms must be separately sourced.

These comparators inform architecture; they do not establish South African participant permissions, limits or product availability.

## Data and consent binding

Every acceptance binds request id/version, requester, payer, beneficiary, amount, currency, reference, purpose, fee disclosure, expiry, channel and a digest of what was presented. A change to any field invalidates prior acceptance. Replay of an exact acceptance can resolve to one existing operation; changed fingerprint must be rejected. A decline, expiry or cancellation before acceptance creates no payment instruction.

## Benchmark

Hard gates:
- 0 payment instructions without valid, exact payer acceptance;
- 0 post-display mutations accepted under the old digest;
- 0 duplicate instructions from replay or concurrent accept callbacks;
- 0 delivered/viewed/accepted requests reported as recipient funds available;
- 0 declines, expiries and pre-acceptance cancellations that create instructions;
- 0 stale product limits or source conflicts silently admitted as current;
- 100% request and acceptance accounting with explicit unknown outcomes;
- deterministic replay of the frozen corpus.

Measures:
- request creation, delivery and view latency;
- view-to-decision duration and acceptance/decline/expiry/cancellation;
- acceptance-to-instruction latency;
- instruction-to-recipient-availability latency separately;
- limit-rejection rate and request count/value headroom;
- wrong-payee prevention, stale alias resolution and mutation rejection;
- fee disclosure, payer/merchant cost and usability;
- complaints, spam and accessibility.

Never maximise acceptance through dark patterns, concealing requester identity, suppressing decline, or making rejection difficult.

## Claim ceiling

The corpus is synthetic. It establishes no Capitec integration, live participant permission, effective-date certainty, fee guarantee, payment settlement or real-world superiority. Current product and account terms must be verified with the participant before operational use.


## 2026-11-04 Capitec fee-incidence and requester-limit clarification

A further current FAQ snapshot makes the cost incidence explicit but also reveals an internal documentation conflict:

- Capitec says sending a PayShap Request is free.
- It says the fee is charged to the person receiving/approving the request; the FAQ's bands are R0–R100 free, R100.01–R1,000 R1, and R1,000.01–R3,000 R3.50.
- The same FAQ says the request maximum is R100 and the account may make two requests per day / within 24 hours to any South African bank.
- The FAQ says a request includes the sender's name/reference, may be declined without money being taken, a pending request can be cancelled by the sender, and the default expiry is one day. Once approved, its FAQ describes the payment as immediate and irrevocable.

Source: [Capitec PayShap Request FAQ](https://www.capitecbank.co.za/were-here-to-help/faqs/), observed 4 November 2026. The page does not state the effective date of these terms.

**Qualification rule:** retain the two-request limit as scoped to the requester account because the FAQ says "you can make two requests in 24 hours to anyone". Preserve the payer-fee bands but mark the schedule internally conflicting because two of the three fee bands exceed the same FAQ's R100 request cap. Do not generalise the Capitec fee/limit to other participants. The 2025 blog remains a separate historical profile with R500 maximum and its own fee snapshot.

This confirms why the outcomes benchmark must track requester and payer fees separately: a free request sent by a merchant need not mean a free payment for the payer. The fee schedule is a public product-term observation, not a matched realised-cost result.


## 2026-11-04 canonical request commitment contract

Every request is bound by a recomputed digest, not by equality of unverified digest strings.

`SHA-256("MYCELIX-SA-PAYMENT-RTP-REQUEST-V1" || NUL || canonical_request_bytes)`

The canonical payload is the complete request object with only `request_digest` removed. It includes request ID/version, requester, payer and beneficiary identities, amount/currency, reference, purpose, fee, expiry, issue time, channel and profile.

`MYCELIX-CANONICAL-JSON-V1` uses compact UTF-8 JSON, recursively lexicographically sorted object keys, preserved array order, standard JSON string escaping and integer-only numeric values. Floating-point values are prohibited in this v1 payload. The domain separator is ASCII followed by a single NUL byte. Digest comparison is byte-exact.

The Rust evaluator recomputes the digest for each request and rejects any request whose declared digest differs. Acceptance and payment-instruction events must bind that recomputed value. A field change without a new version/digest and new payer acceptance is invalid.

A reference vector is stored in `request_digest_contract.vectors` and checked by unit tests. This is a protocol-specific canonicalization profile; it must not be silently replaced by a different JSON serializer or digest scope.


## Identity and authenticity boundary (verified 4 November 2026)

The Capitec FAQ says a PayShap Request displays the sender's name and reference, can be sent/approved only by a logged-in user in its app, and can be declined where the recipient does not know who sent it or why. This proves only the documented user-interface and access-control claims. A displayed sender name is not proof of legal-person identity, authority to collect for a beneficiary, invoice validity, or absence of compromised sender credentials.

The European Payments Council's Verification of Payee (VOP) scheme is a useful but bounded comparator. VOP v1.1 became effective 20 September 2026 and lets a payer PSP ask a payee PSP to compare an account identifier/name, returning match, close match, no match or unable-to-check. EPC explicitly says VOP messaging cannot be relied upon to identify a private or legal person. Its matching guidance v2.0, published 3 November 2026, addresses character sets, natural/legal person name differences, initials, phonetic and commercial-name variants. This improves account/name mismatch detection; it still does not establish that an invoice is genuine or a requester is authorised to collect it.

Thus keep five independent claims: sender account/session authenticated; displayed counterparty/account matched; sender authorised to request for the beneficiary; underlying commercial claim supported; payer voluntarily accepts the exact immutable request. Never collapse them into a single `verified` flag.

## Capitec amount/fee inconsistency

The current FAQ has a R100 per-request cap but also lists payer fee bands for R100.01–R3,000. Requests above R100 must therefore be rejected/quarantined according to the stated per-request cap; the fee bands above R100 are preserved as contradictory tariff text, not evidence that those request amounts are admissible. The FAQ says two requests per 24 hours but does not state a combined daily value ceiling; that separate monetary field remains unknown.


## South African participant capability comparison (observed 4 November 2026)

Do not model a universal PayShap Request limit. Limits are participant-, route-, direction- and counterparty-specific in the public product material.

| Provider profile | Amount limits | Request-count / counterpart limits | Expiry | Partial payment |
|---|---|---|---|---|
| Capitec current FAQ/product snapshot | R100 per request | 2 outbound requests per 24 hours to any SA bank | 1-day default; sender can select expiry; maximum unknown | FAQ says only the requested amount can be paid |
| Absa current page/FAQ snapshot | On-us account-number R5m; on-us ShapID R3,000; off-us R50,000 subject to receiving-bank lower limit | Unlimited aggregate sent/received; at most 5 per same recipient per day outbound and 5 from same sender per day inbound | Absa-sent requests expire after 45 days | Partial/reduced amount when sender enables AMI |
| Nedbank current FAQ snapshot | Up to R50,000 per transaction per day; other banks may have lower limits | No daily limit on number sent or paid | 14-day default, selectable from today through day 45 | Not established by this FAQ snapshot |

Sources: [Capitec FAQ](https://www.capitecbank.co.za/were-here-to-help/faqs/), [Absa PayShap Request](https://www.absa.co.za/self-service/digital-banking-services/digital-ways-to-pay/payshap-request/), [Nedbank PayShap Request FAQs](https://personal.nedbank.co.za/bank/digital-banking/needs/payments/payshap/payshap-request-faqs.html). These are current public-source observations, not effective-date guarantees or a universal scheme rule. Off-us limits can be constrained by the other participant. Fees remain participant/product-specific and must be qualified separately.

### Evaluator obligations

- Resolve the route scope before applying a maximum (on-us account, on-us ShapID, off-us, or generic).
- Never infer a daily count cap from a per-transaction amount limit.
- A rate-limit rejection is only admissible when the participant profile publishes a finite counter.
- A partial-payment option is only admissible under the participant's qualified policy and an immutable AMI field in the request commitment.
- Expiry duration is checked against the selected participant profile; a request's chosen expiry does not modify the profile.
- Decline and cancellation terminality is participant-specific; do not inherit Nedbank's no-reopen rule to other participants without source evidence.
