# Payment Pre-initiation Assurance Certificate v1

**Status:** synthetic reference contract; effects and dispatch are disabled.

## Purpose

Compose independently qualified upstream payment intent, payee-resolution, VoP/CoP, provider-JWS trust, participant limit/fee, payer presentation and consent evidence into an inspectable pre-initiation certificate. This evaluator does not implement a QR parser, fuzzy name matcher, JOSE verifier, provider directory, payment rail, receipt signer or payment executor.

## Composes with

- #5742/#5743 and #5769: immutable PaymentIntent and request-digest evidence;
- #5807: provider-authoritative VoP/CoP result semantics;
- #5810: provider-JWS trust result (synthetic fixture only in this evaluator);
- #1215 typed evidence and later lifecycle/recourse work.

## Admission states

- `ready_for_payer_decision`: required pre-presentation evidence is valid and fresh; payer decision remains separate.
- `ready_with_warning`: a profile-specific Close Match, No Match, unavailable or not-supported result was accurately shown; payer choice remains separate.
- `needs_payer_reconfirmation`: intent, evidence set, fee or presentation changed after consent.
- `quarantined_unknown`: source, profile, freshness or override policy is unknown or indeterminate.
- `ineligible_under_profile`: a source-backed participant/profile rule or limit rejects the intent.
- `rejected_evidence`: evidence is invalid, mismatched, replayed, unauthenticated or materially incomplete.

The separate `prevalidated_for_psp_review` disposition means only that evidence and payer-choice records are internally consistent for the PSP to consider. It does not submit an instruction or confer payment authority.

## Non-collapsible claims

`name match != legal identity != merchant legitimacy != authority to collect != debt validity`

`signature valid != issuer authorised != subject correct != payer consent != payment executed`

`certificate issued != payment submitted != provider accepted != recipient available != settled != final`

The public corpus uses synthetic subject commitments, not raw account identifiers, phone numbers, national IDs, statements or bearer credentials.

## Source and policy boundaries

The source register distinguishes CPMI's 20 November 2025 payment pre-validation brief; EPC's VoP rulebook and no-match/unavailable payer-warning guidance; UK PSR Confirmation of Payee policy; and SARB's proposal-only NPU payee-verification context and published QR+ 1.2 initiation standard.

South African NPU payee verification is **proposal-only**, not an operational nationwide API. QR+ 1.2 is an initiation data-exchange standard, not a payee-verification response or payment execution. The EU fixture permits continuation after No Match/unavailable with an explicit warning under the cited profile guidance. UK override policy remains unknown in this fixture. Foreign policy is not transferred to South Africa.

## Digests

Presentation: `SHA-256("MYCELIX-PAYER-PRESENTATION-V1\\0" || canonical_json(payload))`.

Evidence set: `SHA-256("MYCELIX-EVIDENCE-SET-V1\\0" || canonical_json({profile_id,intent_digest,evidence_refs}))`.

Certificate output: `SHA-256("MYCELIX-PREINITIATION-CERTIFICATE-V1\\0" || canonical_json(certificate_payload))`.

Canonical JSON is UTF-8 compact JSON, recursively sorted object keys, preserved array order, integer-only numbers and standard JSON escaping. The upstream intent digest is consumed by reference, not reimplemented here.

## Hard gates

- 0 wrong issuer/audience/key-trust, stale, replayed or cross-subject evidence admitted;
- 0 No Match/unavailable/not-supported results converted to Match;
- 0 Close Match relabelled Exact Match;
- 0 payee match promoted to legal identity, merchant legitimacy, debt or collection authority;
- 0 changed intent or incomplete presentation accepted under stale consent;
- 0 unknown jurisdictional capability promoted to supported;
- 0 consent bound to a different presentation/evidence-set digest;
- 0 certificates marked submitted, accepted, recipient-available, settled or final;
- 100% deterministic reason codes and certificate digests on frozen inputs.

Quality measures include false-ready and false-quarantine rates, freshness and unknown rate, provider evidence latency/availability, warning comprehension, reconfirmation rate, evaluation latency, certificate size, accessibility and channel parity. Compare incumbent, conventional orchestration and Mycelix on identical evidence sets and matched case mix; never trade trust or consent gates for speed.

## Claim ceiling

A synthetic PASS does not establish live South African VoP coverage, live JWS verification, legal payee identity, requester authority, debt validity, payment authorisation, dispatch, settlement or legal finality.


## Independent timestamp freshness check

A freshness label is not sufficient by itself. Every certificate scenario binds a frozen `evaluation_time`. The evaluator independently checks:
- payment intent issue/expiry window;
- payee resolution observation, expiry and profile maximum age;
- VoP result observation/expiry and profile maximum age;
- signed provider evidence issue/expiry and profile maximum age;
- limit/fee evidence observation, expiry and profile maximum age;
- payer decision time relative to presentation, evaluation and intent expiry.

The synthetic evaluator uses UTC RFC 3339 timestamps to make replay deterministic. A provider label of `fresh` cannot override an expired, future-dated or over-age timestamp. Unknown freshness bounds are quarantined, not guessed.
