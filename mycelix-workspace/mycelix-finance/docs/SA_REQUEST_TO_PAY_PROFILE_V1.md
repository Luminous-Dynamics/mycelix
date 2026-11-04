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
