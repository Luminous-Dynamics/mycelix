# Payment Initiation Assurance Contract v1

**Status:** synthetic research contract; no provider integration or live payment execution.

## Scope

Treat QR, barcode and digital-link payloads as untrusted initiation data. QR+ Standard 1.2 is a published South African initiation/data-exchange standard. The underlying selected rail executes the payment; the QR+ payload itself does not.

A correct system separates parsing, payee authentication/resolution, payer presentation, user consent, dispatch, provider acceptance, settlement, recipient availability and finality/legal discharge.

## PaymentIntentV1 concept

The future implementation should define a canonical, versioned and domain-separated intent containing:
- stable intent ID and one-time nonce;
- payer authority and authenticated session/key context;
- selected PSP and underlying rail;
- QR+ profile version and raw payload digest;
- provider-authenticated canonical payee reference and display result;
- exact integer amount, currency/instrument and scale;
- payer/merchant fee and total debit, each in explicit units;
- payment reference and merchant order ID where applicable;
- creation/expiry time and payee-resolution freshness;
- exact fields shown to the payer; and
- digest of the exact intent authorised by the payer.

No values may silently change after consent. A changed payee, amount, currency, scale, fee, merchant order, PSP or rail requires a new intent and fresh payer authorisation.

## Payee verification and privacy

A payee result must bind the provider/directory identity, audience, canonical beneficiary, display name/masked identity, timestamp, expiry and signing-key profile. The client verifies it under the provider's trusted key/profile; a QR-embedded display name is informational only.

Limit payee-resolution requests and disclosed fields to reduce account enumeration. Stale/unavailable directories produce an unverified/indeterminate result, not fallback to untrusted QR text.

## Idempotency and uncertainty

An exact retry uses the original operation identity and exact intent fingerprint. Reusing an identity with a changed fingerprint conflicts. After a timeout when an external effect may have occurred, preserve the original operation ID and query/reconcile that effect. Never retry with a new key merely because the response is unknown.

## Lifecycle rule

`Parsed != PayeeResolved != PayeeVerified != Presented != Authorised != Dispatched != ProviderAccepted != Settled != RecipientAvailable != Final`.

A valid signature or directory result does not imply user consent. User consent does not imply provider acceptance. Provider acceptance does not imply settlement. Settlement does not automatically prove recipient availability or legal finality.

## QR+ and South African timeline

- SARB published QR+ Standard 1.2 on 7 July 2026.
- QR+ supports QR codes, barcodes and digital links; v1.2 adds a PayShap payload for structured initiation-data exchange to the underlying rail.
- The Q1 2027 operational pilot is a separate milestone and cannot be inferred from standard publication.
- Provider/merchant implementation and acceptance require participant-specific evidence.

Source: [SARB QR+ Standard 1.2](https://www.resbank.co.za/en/home/publications/publication-detail-pages/other-publications/2026/qr-1-2-overview).

## Benchmark goals

Conformance: 100% rejection of seeded payee/provider/amount/currency/scale/audience/expiry/replay mutations; zero dispatches after mutation or cancellation; zero parse-only promotion; exact retry reuse and changed-fingerprint conflict; zero timeout reissue under a new identity before reconciliation.

Operational measures: scan-to-payee-resolution p50/p95/p99; confirmation-to-dispatch p50/p95/p99; initiation-to-recipient-available p50/p95/p99; wrong-payee prevention, false-warning and task-completion rates; directory freshness/availability; provider coverage; enumeration resistance; all-in cost, support burden, reversals and disputes.

Safety gates are non-compensable and are not averaged against latency or cost.

## Claim ceiling

This contract does not implement QR+ parsing, payee resolution, user authentication, provider integration, PayShap access or settlement. The corpus evaluator uses synthetic fixtures only.


## Provider-authoritative Verification of Payee / Confirmation of Payee

This contract treats verification of the beneficiary's account name/type as a distinct evidence claim between proxy resolution and payer consent.

`proxy resolution != account-name verification != requester identity != collection authority != commercial-claim legitimacy != payer consent`

The EU Instant Payments Regulation overview states that the payer's PSP must offer VoP for standard and instant euro credit transfers, free to the payer. The ECB identifies four result classes: match, close match, no match and other. The euro-area deadline was 9 October 2025; the non-euro-area deadline is 9 July 2027. Source: [ECB IPR overview](https://www.ecb.europa.eu/paym/retail/instant_payments/html/instant_payments_regulation.es.html).

Pay.UK's January 2026 Faster Payments Principles v11 describes CoP checks against both the name and personal/business account type held by the payee PSP, with match, close match, no match and unavailable outcomes. Source: [Pay.UK FPS Principles v11](https://www.wearepay.uk/wp-content/uploads/2026/02/Pay.UK-Faster-Payments-System-Principles-v-11-Jan-2026.pdf), section 10.3, page 26.

SARB's 2025 PEM Position Paper proposes future NPU proxy-resolution and confirmation-of-payee capabilities. This is programme intent, not evidence of a currently operational nationwide South African VoP service. Source: [SARB PEM Position Paper](https://www.resbank.co.za/content/dam/sarb/publications/other-publications/2025/pem-position-paper.pdf).

The new 14-case synthetic extension tests:
- exact, close, no-match, unavailable and other outcomes;
- stale result, wrong audience and cross-subject replay;
- personal/business account-type mismatch;
- alias resolution without available name verification;
- name match incorrectly promoted to collection authority;
- beneficiary mutation after query;
- client-side fuzzy-match promotion;
- excessive payer-visible disclosure.

The evaluator qualifies declared provider-result classes, source/profile references, issuer/participant/audience and exact opaque subject bindings. `provider_signature_state` is a fixture input; this evaluator does not cryptographically verify a provider response signature or establish live issuer key trust. A real adapter must consume a separately qualified signature/JWS verifier and freshness/key-status profile. The evaluator does not calculate approximate name matches. A positive name match does not establish requester authority, debt legitimacy or payer consent. Unavailable, other and stale remain distinct non-match dispositions. A proposed South African NPU capability is recorded as proposal-only, not a live provider result. All new fixtures keep dispatch disabled.

## 2026-11-04 SRTP technical-documentation comparator update

The EPC published SRTP Trust & Security Model v1.1.0 (authentication, message integrity and JWS electronic sealing) and SRTP OpenAPI v4.0.1 on 27 October 2026. They align with SRTP Rulebook v4.0; they may be used voluntarily from publication and formally take effect 1 May 2027, when EPC Directory Service use becomes mandatory for all scheme participants. These are European scheme comparators, not South African PayShap Request requirements.

Sources: [EPC publication](https://www.europeanpaymentscouncil.eu/news-insights/news/epc-publishes-updated-technical-documentation-for-the-srtp-scheme), [Trust and Security Model](https://www.europeanpaymentscouncil.eu/document-library/guidance-documents/sepa-request-pay-srtp-trust-and-security-model-0), [OpenAPI v4.0.1](https://www.europeanpaymentscouncil.eu/document-library/implementation-guidelines/sepa-request-pay-srtp-openapi-specifications).


The complete synthetic corpus now contains 33 unique QR+/payee verification cases. The focused negative-mutation campaign targets 28 independent semantic failure modes.
