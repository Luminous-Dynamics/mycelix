# Maritime Commons Monetary Interoperability Profile v0.1

**Status:** research proposal; not normative Stay Protocol v0.1 change  
**Tracking:** Mycelix #5948  
**Parent:** Global Interoperability #5945  
**Composes:** Stay Protocol #5918, verifier #5921, benchmark #5932

## 1. Objective

Define a deterministic, internationally portable representation for prices, charges, credits, invoice line items, cancellation adjustments, refund requests and external settlement evidence.

The current Stay Protocol v0.1 uses integer amount_minor and a three-letter currency code. That avoids binary floating-point arithmetic, but the integer is not globally interpretable unless the currency scale and metadata revision are pinned. ISO 4217 currency conventions are not universally two decimal places.

This profile does not change the v0.1 wire contract. It defines a proposed versioned monetary context and the migration questions for review.

## 2. Governing principles

1. Canonical monetary amounts are exact integers, never binary floating-point values.
2. Every amount is bound to a currency metadata snapshot and applicable minor-unit exponent.
3. Historical amounts retain their original scale interpretation even when registry metadata changes.
4. Locale formatting changes presentation only, never accounting meaning.
5. Quote currency, invoice currency and settlement currency are separately identified.
6. Foreign exchange is an explicit, sourced transformation, not an implicit formatting/conversion operation.
7. Refund request, refund attempt and externally evidenced refund finality are different states.
8. A signed protocol receipt is not proof that a bank, card network, payment provider or cash operator completed settlement.
9. Tax treatment and regulatory compliance are referenced to a reviewed jurisdiction profile; they are not inferred by the generic currency layer.

## 3. Proposed MoneyAmount profile

A future money object should bind:

- profile identifier and version;
- ISO 4217 alphabetic and numeric currency identifiers;
- currency metadata standard and exact source artifact/release;
- source artifact digest and retrieval time;
- minor-unit exponent applicable at the transaction's effective date;
- effective/retirement interval for historic or replaced currency codes;
- non-negative integer amount in the declared minor unit;
- semantic role (quote, charge, invoice, credit, refund request, refund effect, settlement evidence);
- line-item breakdown and arithmetic;
- explicit rounding policy and evidence of any applied rounding;
- transaction effective time and relevant jurisdiction/price policy references.

The companion JSON Schema describes the structural proposal. Semantic rules still require an independent verifier.

### Why metadata must be pinned

A string such as currency=XXX plus amount_minor=1234 does not fully identify a major-unit amount until the applicable exponent is known. The exponent must come from a pinned, issuer-qualified currency metadata artifact, not a mutable latest alias or a UI locale.

The test corpus therefore refers to exponent-zero, exponent-two and exponent-three fixture classes. At execution, the test harness must bind these classes to the exact ISO 4217 metadata artifact and digest used. Do not hardcode a remembered internet table as the authoritative fixture.

## 4. Amounts, line items and sign semantics

Prefer a non-negative total plus explicit line-item direction/category rather than allowing an unexplained negative total.

Line items need:
- stable line-item identity and revision;
- semantic category (base charge, tax/levy, mandatory fee, optional fee, discount, credit, cancellation charge, refund adjustment);
- signed integer minor-unit value;
- currency-context reference;
- mandatory/optional disclosure status;
- source/provider and quote/invoice revision;
- tax inclusion/exclusion declaration where applicable.

The verifier must define which categories may be negative and which aggregate totals are valid. A negative line item must not silently turn a payment into a refund or create an external financial effect.

Refund request, refund authorization, refund attempt, refund execution and external finality require distinct typed evidence. A requested refund is not a completed refund.

## 5. Quote, invoice and settlement

Keep these distinct:

- provider quote and expiry;
- accepted contractual price/obligation;
- invoice/credit note issued by the responsible provider;
- payment instruction to an external payment provider;
- payment attempt/authorization/capture;
- settlement/finality assertion from the responsible external system;
- reversal, chargeback or refund evidence.

Maritime Commons may carry references and reconcile statements, but does not hold payment-card data, create a payment rail or become a merchant of record by virtue of this profile.

A provider's signed assertion that an invoice is paid is only a provider claim unless the contract explicitly recognizes that provider as the finality authority for the relevant payment method. The recognized external finality source and evidence type must be pinned.

## 6. Foreign exchange

Never convert quote to settlement currency implicitly.

An FX conversion record should bind:
- exact source and target currency contexts;
- source and target amount values;
- rate as an exact rational or otherwise explicitly specified decimal representation;
- direction/base/quote convention;
- rate source/issuer and quote/reference identity;
- observed-at/effective time and validity window;
- fees/spread and who bears them;
- rounding mode and rounding stage;
- any external payment/settlement evidence.

A stale, reversed, unrecognized or ambiguous FX rate is not a valid conversion. The generic Commons does not operate an FX data feed or execute conversion.

## 7. Rounding, tax and locale

Rounding must be explicit: mode, precision, stage, input value, result and profile/version. Distinguish accounting rounding from cash tender rounding. Never silently round an amount to fit a display.

Tax/levy fields reference a jurisdiction pack and responsible provider/authority. This profile does not decide whether a tax applies or whether an invoice is legally valid.

CLDR is for locale presentation. Formatting an amount for English, Arabic, Hindi, Chinese or another locale must never modify the canonical integer amount, currency code, exponent, tax classification or signed evidence.

## 8. Compatibility with Stay Protocol v0.1

Before implementation, review the following options:

A. Add a separately versioned currency metadata reference to a future compatible message profile, retaining amount_minor only where an exact scale profile is present.

B. Introduce a new versioned money object with currency metadata, exponent, effective date and amount semantics.

C. Define a narrowly scoped v0.1 interpretation only where the provider explicitly supplies a scale context, and return Unknown/NotEvaluable where the context is absent.

Do not retroactively guess the exponent for existing v0.1 fixtures. Existing messages must not be silently rewritten or reinterpreted. The protocol owner must decide the migration and compatibility rule in #5918.

## 9. Acceptance and claim ceiling

- Every canonical amount has a pinned currency metadata artifact and applicable exponent.
- Integer arithmetic is deterministic, bounded and overflow-checked.
- Quote, invoice, credit, payment attempt and external settlement evidence remain distinct.
- Quote currency and settlement currency cannot be silently conflated.
- FX conversion has exact rate source, direction, time, fee and rounding lineage.
- Locale formatting cannot mutate canonical values.
- Refund finality cannot be inferred from refund request or ordinary signed message.
- Historical replay retains the original currency metadata interpretation.
- Positive controls pass; the verifier cannot pass by rejecting every currency except two-decimal examples.

Passing the synthetic corpus establishes only arithmetic and metadata behavior for the exact pinned fixture and implementation. It does not establish tax/legal compliance, provider invoice validity, regulated payment status, bank settlement, actual refund completion, FX execution or financial compliance.
