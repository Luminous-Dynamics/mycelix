# MoneyAmountV1 wire contract

**Status:** shared representation primitive; not yet a persisted-entry migration.

## Wire rules

- Amounts are canonical non-negative base-10 integer strings in atomic/minor units.
- JSON numbers, decimals, signs, exponents, whitespace and leading zeroes are rejected.
- Every amount carries an immutable instrument profile ID; scale is resolved from an explicitly admitted registry, never inferred from the label or value.
- Arithmetic is checked and allowed only inside the same profile.
- The current SAP profile describes the internal ledger's micro-SAP representation only; it is not ZAR, external custody, a settlement rail or legal finality.
- TEND is syntactically representable but intentionally has no admitted scale in v1. No SAP/TEND equivalence is implied.

## Historical migration

Existing Holochain Trade entries that serialize f64 are legacy schema v1 and remain readable as legacy data. Do not reinterpret or rewrite their bits in place. Migration must produce an append-only v2 successor with source action hash, exact old serialized digest, chosen instrument profile, conversion proof, explicit rounding/excess-precision disposition, independent reconciliation and an authorisation record. A value that is not exactly representable under the selected profile is quarantined or marked indeterminate, not rounded.

The TypeScript form is string-based so values above JavaScript's exact integer range stay exact. Never pass minor_units through Number().

## Qualification

money-amount-golden-v1.json defines canonical, negative and arithmetic vectors. Rust tests cover the shared primitive. Rust and TypeScript now share the checked-in golden vectors; the SDK helper uses decimal-string digit arithmetic without Number() or BigInt. Holochain dual-read/append-only migration remains separate work.
