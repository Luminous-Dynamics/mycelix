# IG-007U0 — Treasury asset/unit observed profile

## Purpose

IG-007U0 freezes the source-visible Treasury asset/unit semantics that motivate P0 #1222.

It is an **observational profile**, not a live transaction test and not a successor implementation.

The central distinctions are:

```text
integer arithmetic safety
!= canonical currency identity
!= canonical quantity/base-unit authority
!= Treasury consumption of that authority
```

## Evidence identity

```text
profile   mycelix-treasury-asset-unit-observed-fca2c107-v1
revision  1
authority ObservedSourceBound
SHA-256   58280f735acecd84071b71876c5b38d12d6776b8505016b22a020a05cca2ffb5
```

Semantic production subject:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Evidence-authoring parent:

`feb30a89257e96592fdbd40b249258581d42fde7`

Exact bound source:

```text
Treasury coordinator
mycelix-finance/zomes/treasury/coordinator/src/lib.rs
blob 840e66bcb6fdedb27fe2451752511d1317eb50b8

Treasury integrity
mycelix-finance/zomes/treasury/integrity/src/lib.rs
blob 5ee9b72f7c138a5283818b873ceae36dff3305d4

Canonical Finance domain types
mycelix-finance/types/src/lib.rs
blob 354a377c88a57e8a5612a518d8c8b8b519fb67e2
```

## Canonical Finance currency semantics already exist

The HDK-free `mycelix_finance_types` crate is explicitly the shared economic type-definition layer. It defines:

```text
Currency::Mycel
Currency::Sap
Currency::Tend
```

with source-visible semantic differences:

```text
MYCEL -> non-transferable reputation substrate
SAP   -> transferable circulation medium
TEND  -> transferable mutual credit
```

This is a positive control: Mycelix already has a canonical currency namespace.

However, the frozen Treasury plane does **not** consume that type. Treasury, Contribution, Allocation and SavingsPool use free-form `currency: String` fields instead.

Therefore the observed problem is not simply “no currency model exists.” It is:

```text
canonical Finance Currency enum exists
!= Treasury consumes canonical Currency enum
!= canonical per-currency quantity semantics exist
```

The frozen source also exposes no general canonical quantity/base-unit type in the shared Finance domain crate.

## Treasury asset identity

The observed Treasury model stores:

```text
currency: String
balance: u64
```

`CreateTreasuryInput` accepts the currency string directly.

Observed create/update integrity does not establish use of `mycelix_finance_types::Currency`, a canonical base-unit scale, or an immutability theorem for Treasury currency.

A Treasury ID therefore does not, from the frozen integrity theorem alone, imply one immutable typed asset/unit identity.

## Contributions

`ContributeInput` carries its own `currency: String`; the recorded Contribution stores that caller-supplied string while:

```text
credit_treasury(treasury_id, amount)
```

mutates only the numeric Treasury balance.

No source-visible Contribution→Treasury currency equality theorem is established.

## Ordinary allocations

`ProposeAllocationInput` likewise carries caller-supplied `currency: String`; later:

```text
execute_allocation
  -> debit_treasury(&alloc.treasury_id, alloc.amount)
```

uses only Treasury ID and numeric amount at the debit boundary.

No source-visible Allocation→Treasury currency equality theorem is established before debit, and Allocation integrity does not reconstruct asset/currency equality or immutability.

## DKG / large-value path

The Treasury coordinator defines:

```text
DKG_THRESHOLD_AMOUNT = 10_000_000_000
```

and documents the amount as **micro-SAP**.

Its signed subject is shaped as:

```text
treasury_allocation:{treasury_id}:{amount}:{recipient_did}
```

which explicitly binds Treasury ID, numeric amount and recipient but does not explicitly bind:

```text
Currency / asset id
base-unit scale
policy revision
```

The resulting Allocation records:

```text
currency: treasury.currency.clone()
```

Thus the high-value path assumes micro-SAP threshold semantics while the Treasury itself retains a free-form string currency field that is not source-bound to the canonical `Currency::Sap` authority.

## Qualification result

The frozen source supports:

```text
CanonicalFinanceCurrencyEnumExists = true
TreasuryUsesCanonicalFinanceCurrencyType = NotEstablished
CanonicalTreasuryAssetUnitBinding = NotEstablished
DkgSignedSubjectAssetUnitPolicyBinding = NotEstablished
```

This does **not** mean integer arithmetic is unsafe.

## Positive controls

Preserve separately:

1. Treasury balances are integer `u64` values;
2. credits use `checked_add`;
3. debits use `checked_sub`;
4. insufficient Treasury balance fails closed;
5. the DKG threshold is explicitly documented in micro-SAP;
6. the pure Finance domain crate already defines canonical `Currency::{Mycel,Sap,Tend}` semantics.

## Successor design direction

Do **not** introduce a competing asset-ID namespace if `mycelix_finance_types::Currency` can be evolved safely.

The successor should extend the existing pure domain layer with typed quantity/value semantics. Because the three currencies have materially different economic semantics, avoid a universal `u64 + Currency` type that implies they are interchangeable.

A better shape is conceptually a tagged family such as:

```text
TransferValue {
  Sap(SapAmountBaseUnits),
  Tend(TendCreditUnits),
}
```

with MYCEL excluded from transferable-value APIs unless a separate, explicitly non-transferable reputation quantity is required.

Exact names/types remain reviewable. The core theorem is:

```text
Treasury asset identity is canonical and typed
+ quantity representation is valid for that asset
+ base-unit scale is explicit where applicable
+ mutations/thresholds/authorizations consume the same typed value subject
```

Historical free-form records require explicit version/migration semantics rather than silent reinterpretation.

## Relationship to #1134 / #1151 / #1247 / #1248

The large-value theorem must classify a qualified typed value, not a naked integer. A successor authorization should bind that same typed value inside #1247's canonical action subject, be authorized through #959/#960, and be consumed under #1248's execution/idempotency semantics.

## Child evidence

U1 / #1241 remains blocked on executable exact-head U0 qualification. Its deterministic receipts should preserve both the positive existence of the canonical Currency enum and the negative observation that Treasury does not consume it.

## Non-claims

No live cross-currency transfer, stolen funds, deployment exploit/currentness, or Treasury safety verdict is claimed.
