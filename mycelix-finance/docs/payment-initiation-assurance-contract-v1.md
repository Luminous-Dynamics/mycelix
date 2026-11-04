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
