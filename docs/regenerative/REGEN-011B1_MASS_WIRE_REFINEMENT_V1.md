# REGEN-011B1 — Biomass Mass/Wire Refinement v1

Status: refinement of REGEN-011B only. This document supersedes the numeric-width sentence in REGEN-011B section 4 while preserving the rest of the REGEN-011B executable biomass contract unchanged.

## 1. Correction

REGEN-011B originally proposed:

```rust
pub struct MassMg(u128);
```

For v1, use:

```rust
pub struct MassMg(u64);
```

This refinement changes no mass basis, ecological-allocation, assessment, reservation, evidence, rights/custody, or authority semantics.

## 2. Why `u64` is sufficient

The maximum `u64` value is:

```text
18,446,744,073,709,551,615 mg
= 18,446,744,073,709.551615 kg
≈ 18.446 billion metric tonnes
```

That is far above a reasonable single biomass lot, material-state snapshot, ecological allocation, or process-input reservation while retaining exact non-negative integer arithmetic.

If a future application needs one value larger than that, it should justify a separately reviewed scalar revision rather than silently widening the wire type.

## 3. Wire-format reason

The first biomass core should remain friendly to ordinary serde/JSON and cross-language consumers.

`u64` has substantially more conventional wire interoperability than unconstrained `u128` numeric serialization while still providing enormous physical range.

Therefore v1 freezes:

```text
exact internal mass scalar = unsigned 64-bit integer milligrams
```

and not:

```text
arbitrary JSON number
floating-point kilograms
implementation-dependent 128-bit JSON number
```

## 4. Arithmetic rules remain unchanged

All REGEN-011B arithmetic rules still apply:

- checked addition only;
- checked subtraction only;
- overflow is an explicit error;
- no saturating arithmetic;
- no negative values;
- no implicit unit conversion;
- no implicit mass-basis conversion;
- `AsReceived != DryMatterEquivalent`;
- reservation sum must not exceed available same-basis mass.

## 5. PEF remains the analytical-value layer

Milligram ledger precision is not an analytical measurement precision claim.

Fine-grained laboratory measurements, moisture fractions, uncertainty, and derived quantities continue to live in PEF evidence.

The biomass mass primitive exists for physical lot/allocation/reservation accounting, not to replace environmental/scientific measurement values.

## 6. Serde theorem

If the first biomass crate enables serde, `MassMg` should encode/decode as the canonical non-negative `u64` integer representation used by that wire format and must revalidate all enclosing domain invariants after deserialization.

No deserialization path may:

- coerce a floating value to integer mass;
- accept a negative quantity;
- silently saturate an overflowing quantity;
- reinterpret a quantity under a different `BiomassMassBasis`.

## 7. Qualification delta

The future REGEN-011 executable campaign should add explicit tests for:

1. `MassMg::new(0)` where zero is structurally permitted;
2. `MassMg::new(u64::MAX)` round trip;
3. checked addition overflow at `u64::MAX + 1`;
4. checked subtraction underflow;
5. serde round trip for `0`, ordinary values, and `u64::MAX` if serde is enabled;
6. rejection of negative/floating/noncanonical wire values where the chosen wire format exposes them;
7. exact same-basis reservation arithmetic near the upper bound.

## 8. Deliberate non-claims

This scalar refinement establishes no real biomass quantity, measurement accuracy, weighing-device calibration, ecological allocation validity, legal right, process eligibility, market value, or execution authority.

It is a deterministic accounting/wire-format choice only.
