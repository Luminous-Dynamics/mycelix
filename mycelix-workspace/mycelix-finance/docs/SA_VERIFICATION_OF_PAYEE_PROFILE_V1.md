# South African Verification of Payee / Confirmation of Payee Profile v1

**Status:** synthetic qualification contract and international design comparison. This is not a live South African bank service, legal identity certificate, account-ownership proof or payment authority.

## Purpose and boundary

Qualify a payer-visible beneficiary-name verification response as a narrowly scoped pre-payment fact. A ShapID or account alias resolution establishes an alias-to-account binding under its own profile; Verification of Payee compares a payer-submitted name against a participant's permitted representation of the beneficiary. Those are distinct checks.

`alias resolves != name matches != payer consents != payment authorised != funds available`

A Match is not portable proof of a person's legal identity, account ownership, entitlement to funds, or the legitimacy of an invoice/debt. It only records the responding provider's matching outcome for the exact account/name input, profile, participant, audience and currentness window.

## South African position

SARB's PEM position paper proposes Verification of Payee / proxy-resolution capability as part of the future national payments ecosystem. QR+ Standard 1.2 standardises payment-initiation data exchange; it is not itself a universal account-name verification scheme. The current South African participant coverage, response taxonomy, provider endpoints, service levels, match criteria, fallback rules and fees remain **unknown until exact participant/product evidence exists**.

The ZA profile in this corpus is therefore deliberately unqualified. It must not inherit EPC VOP or UK CoP rules.

## International comparators (observed 4 November 2026)

- EPC VOP Rulebook v1.1 and API Security Framework v2.1 are effective from 20 September 2026; Inter-PSP API Specifications v1.1.1 are also effective from that date.
- EPC Matching Recommendations v2.0 were published 3 November 2026. They discuss natural and legal person matching, spelling, transposition, initials, phonetic variation, commercial/trading names and abbreviations. Responding PSPs remain responsible for their own matching criteria.
- The European Central Bank describes VOP under the EU Instant Payments Regulation for standard and instant euro credit transfers, free to the payer and returned before payment authorisation. This is an EU comparator, not a South African rule.
- UK PSR PS26/2 removed the expiry date from Specific Direction 17 in October 2026, retaining CoP requirements for most PSPs. The PSR reports over 320 organisations and more than two million checks per day. Scale does not establish accuracy or effectiveness.

EPC VOP must not be described as a universal identity service. A matching response does not prove legal identity or authority.

## Typed response outcomes

- `match`: provider reports a match under the exact participant/matching profile.
- `close_match`: provider reports a bounded discrepancy and may offer a privacy-permitted suggestion. It is not a Match and cannot authorize payment.
- `no_match`: provider reports mismatch. Do not silently route to the account.
- `verification_not_possible`: provider could not return an authoritative result, for example due to outage, unsupported coverage, throttling or character-set limitation.
- `not_requested`: this path did not perform verification; never describe it as passed.
- `unknown` / `stale`: response authenticity, scope, key, currentness or alias binding cannot be established.

The receiving PSP owns its matching criteria and outcome. Mycelix validates that a response is typed, scoped, current and bound to the submitted request; it does not implement a universal fuzzy matcher or second-guess a provider's private matching rules.

## Exact response binding

Every check binds:
- check ID, provider/participant, profile and version;
- beneficiary account commitment and alias binding generation/state;
- payer-entered name commitment and character-set/normalisation profile;
- payer audience and request commitment;
- provider response check ID, profile, participant, account, alias generation, name input, matching-profile version, charset, audience and request commitment;
- provider signature state, key currentness/revocation and provider authority state;
- response observed time, validity end, reason code and optional Close Match suggestion commitment.

A response with the wrong participant, account, alias generation, submitted-name commitment, profile, audience or request commitment is unbound. An invalid/unknown signature, revoked/unknown key, unauthorised provider or expired response cannot be treated as Match.

## Payer decision and instruction

A payment instruction can be created only when:
1. the final check is a fresh, correctly bound Match;
2. provider signature is valid, key is active, provider is authorised and audience matches;
3. alias binding is current (or direct account verification explicitly uses `not_applicable`);
4. payer's exact decision binds the final request commitment, beneficiary account commitment and submitted-name commitment;
5. payer authority is separately valid.

Close Match requires a new input commitment, a new check and a fresh decision. No Match, Verification Not Possible, Not Requested, Unknown and Stale do not create instructions under this v1 contract. An explicit bypass is recorded as unverified and has no implicit green status. VOP does not authorize a payment or prove settlement.

## South African linguistic and privacy fairness

Use provider-specific character-set/normalisation profiles. Preserve the submitted string and the provider's permitted matching representation separately. Tests include UTF-8, case/space/punctuation/diacritics, transliteration loss, phonetic similarity, trading-name mapping, bidi controls, homoglyphs, zero-width and malformed inputs.

Do not force South African names through an English-only or ASCII-only normalizer. Similar-sounding people must not be automatically conflated. A Close Match suggestion should disclose only the minimum permitted correction. Raw names/accounts remain absent from public benchmark fixtures; commitments are synthetic identifiers, not unsalted hashes of real data.

## Benchmarks

Hard gates:
- 0 Close Match, No Match, Verification Not Possible, Not Requested, Unknown or Stale promoted to Match;
- 0 instruction from wrong provider/audience, account, name, profile, alias generation, revoked key or expired response;
- 0 Close Match instruction without a new check and fresh exact payer decision;
- 0 Match treated as legal identity, account ownership, debt validity or payment authority;
- 100% response binding across check ID, participant, account, alias generation, name input, profile, charset, audience and currentness;
- 100% No Match/VNP/Unknown/Stale reason categories remain distinct;
- 0 raw customer names/accounts or public stable PII hashes in the corpus;
- deterministic replay of frozen responses and decisions.

Measure false Match (critical), false No Match, Close Match precision, Verification Not Possible rate, response p50/p95/p99, stale response rate, alias-currentness mismatch rate, false-warning cancellation, appeal/correction rate, task comprehension and accessibility parity. Publish complete confusion matrices and cohort denominators. Do not optimise only Match rate or check volume.

## Industry comparison

Use EPC VOP v1.1 / API v1.1.1 / Security Framework v2.1 / Matching Recommendations v2.0 as the scheme/API comparator; EU IPR as the regulatory model; UK SD17 as the mature-scale comparator. The first South African benchmark must be labelled coverage-unknown until a named participant's current API, product scope, response taxonomy, terms and currentness profile are sourced.

## Claim ceiling

A passing synthetic evaluator proves only that the frozen response/authority/intent contract rejects modeled binding and status-laundering failures. It proves no live South African VOP service, no bank coverage, no account ownership, no legal identity equivalence, no fraud-reduction result, no SARB endorsement and no live-payment safety.


## Transaction-intent and payer-subject binding

A check envelope is transaction-scoped even though the underlying provider matching API may only compare account/name fields. Mycelix binds the provider response to a payer-subject commitment and an immutable payment-intent commitment, then requires the payer decision and any instruction to preserve the same commitments, amount, currency and reference. This is a local orchestration/evidence binding; it does not claim the EPC provider's VOP API natively consumes payment amount or invoice data.

A valid name-match result for one payer or payment intent cannot be replayed into another. The profile's matching-algorithm version and allowed character-set profile are also bound to the exact check. Synthetic fixture commitments are opaque test identifiers, not hashes of real customer data.
