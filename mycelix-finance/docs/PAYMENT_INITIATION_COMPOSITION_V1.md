# Payment Initiation Composition Assurance v1

**Status:** synthetic, effects-disabled integration contract. Not a provider integration, legal debt determination or payment authorisation.

## Purpose

Compose already-qualified outputs from message authentication, requester identity, VoP/CoP, collection authority, commercial evidence, fraud intelligence, local participant policy, payer decision and payment-effect observation. This evaluator does not duplicate the JWS verifier, request digest canonicaliser, payee matcher, truth adjudicator, human-decision experiment or recourse evaluator.

## Paradigm: an assurance vector, not a trust score

The result preserves separate claims: message authenticity; requester identity; beneficiary/account match; authority to collect for that beneficiary on that product/channel; source-authentic commercial evidence; immutable payment intent and payer decision; purpose-bound fraud intelligence and local policy disposition; observed payment effect.

Every claim is independently bound to its subject digest, issuer, audience, profile, time window and evidence maturity. A positive result in one dimension never fills an unknown or negative result in another.

## Non-transitivity

A valid JWS authenticates a message to a configured trust boundary; it does not prove the request is legitimate. A VoP match compares account/name data under the applicable scheme; the EPC explicitly says it cannot identify a private or legal person. Requester identity does not prove collection authority. Authenticated invoice content does not adjudicate legal debt. Human comprehension does not supply institutional authority. Payer acceptance does not prove execution, recipient availability, settlement or finality.

## Standards context

SARB QR+ 1.2 is an initiation-data standard. It can carry structured data to a selected rail but does not execute payment or establish settlement.

EPC SRTP Trust & Security Model v1.1.0, published 27 October 2026, specifies JWS-based authentication, message integrity and electronic sealing. EPC says the updated technical documentation is voluntary immediately and formally takes effect on 1 May 2027. Do not backdate or apply it as South African rules.

EPC VoP v1.1 became effective 20 September 2026. Its matching-process recommendations v2.0 were published 3 November 2026 and address character sets and name variants. EPC says VoP messaging cannot identify a private or legal person. These are SEPA comparators, not South African participant mandates.

## Synthetic evaluation policy

The harness permits an instruction only when the exact request digest is verified; message authenticity is trusted for the exact request/audience/profile; requester identity binds the exact requester; beneficiary match is Match or Close Match with explicit payer warning acknowledgement; collection authority binds exact requester, beneficiary, product and channel; required commercial evidence binds payer, amount and currency; a current local participant policy allows the exact request; and payer acceptance is current, exact, pre-expiry and acknowledges all material fields.

Decline, query, pay-later or no decision creates no instruction. Unknown/stale required evidence quarantines. Invalid subject joins and explicit no-match reject. Fraud signals remain advisory/contested/unadjudicated inputs; only a separate local policy decision can affect the instruction path. This synthetic policy is not a universal bank policy.

## Effect ladder

`payer decision → payment instruction → provider outcome → recipient availability → settlement observation → profile-specific finality qualification`

Each step requires its own event, authority and evidence. The evaluator never derives a later state from an earlier one.

## Qualification gates

- 0 cross-subject joins across request, requester, beneficiary, authority, invoice, payer and instruction.
- 0 unknown/stale/expired required assertions silently promoted to verified.
- 0 wrong-issuer or wrong-audience assertions accepted.
- 0 name matches promoted to identity, collection authority or invoice legitimacy.
- 0 unadjudicated fraud signals promoted to confirmed fact or automatic denial.
- 0 instructions without exact payer acceptance and local policy permission.
- 0 instruction/acknowledgement events promoted to recipient availability, settlement or finality.
- 100% conflict/correction lineage preserved; deterministic replay of frozen inputs.

Measure false-ready and false-quarantine against independently adjudicated labels, cross-subject mismatch detection, stale/wrong-audience rejection, material-field comprehension, unresolved evidence share, appeal/correction burden and human review time. Do not optimise by quarantining everything.

## Industry comparison

Use matched frozen component outputs for incumbent bank initiation/manual exception workflow, conventional API orchestration/risk integration, and compositional Mycelix assurance-vector evaluation. Measure semantic preservation, false-ready, false-quarantine, correction propagation, evidence completeness and operator effort. Synthetic results are not real-world fraud reduction or provider superiority.

## Claim ceiling

Passing tests proves only that this reference evaluator rejects seeded composition and evidence-laundering errors. It does not prove a requester is honest, an invoice is legally enforceable, payer consent is legally sufficient, a fraud claim is true or a payment settled. All records are fictional; external effects are disabled.
