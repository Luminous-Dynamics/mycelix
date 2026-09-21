# FIN-MKT-001 Canonical Market Order Intent V1

## Scope

This profile defines deterministic identity for one immutable provider-neutral market-order intent.

```text
canonical market-order intent
!= financial authority
!= buying power / capacity
!= provider support
!= credential scope
!= broker order
!= provider acceptance
!= fill
!= settlement
```

It is deliberately narrower than FIX/provider order execution and narrower than FIN-SYNC settlement.

## Dependencies

V1 reuses qualified FIN-ECO-001 `AssetAmount` / `AssetId` only for exact non-negative atomic quantities.

`AssetId` is opaque. The string does not establish listing identity, asset registry authority, share scale, currency semantics, issuer, custody, or legal rights.

Therefore every quantity and price also binds an exact profile reference describing the intended external/unit semantics. Provider adapters must separately prove how their API values map to those profiles.

## Identifier roles

FIN-MKT deliberately separates protocol vocabulary from domain/provider identifiers.

### `MarketProtocolIdV1`

Used for profile IDs and semantic idempotency IDs. V1 permits only ASCII alphanumeric bytes plus:

```text
. _ : / -
```

Maximum: 128 bytes. No case folding or normalization.

### `MarketExternalIdV1`

Used for subject/instrument identifiers owned by domains/providers/adapters. V1 preserves exact UTF-8 bytes, rejects controls, and caps at 256 bytes.

```text
protocol identifier syntax
!= external identifier syntax
```

The kernel does not perform URI, DID, Unicode, ticker, provider-symbol, or identifier-scheme equivalence. An adapter/profile owns any such mapping.

This split intentionally aligns with the semantic-kernel hardening principle that canonical protocol vocabulary and external/domain identity are different roles.

## Canonical transcript

All integers are unsigned big-endian. All text is exact UTF-8 with no normalization and is encoded as:

```text
u32 byte_length || exact UTF-8 bytes
```

Every digest is raw 32 bytes in authority bytes. JSON evidence uses exactly 64 lowercase hexadecimal characters.

V1 transcript:

```text
"MYCELIX_FIN_MKT_ORDER_INTENT_V1\0"
u32 commitment_profile_revision = 1
intent_subject
account_subject
instrument
u8 side
quantity_spec
order_terms
u8 time_in_force
execution_profile
semantic_idempotency
optional upstream_economic_effect_commitment
```

### Profile reference

```text
protocol_id profile_id
u32 revision
[32] digest
```

The digest is opaque. FIN-MKT does not assume how the referenced profile artifact was hashed, resolved, trusted, or adopted.

### Subject reference

```text
profile_ref
external_id subject_id
```

### Instrument reference

```text
profile_ref
external_id instrument_id
```

### Side tags

```text
0 AcquireLong
1 ReduceLong
```

V1 intentionally does not claim complete short-sale, option position-effect, or multi-leg strategy semantics.

### Quantity specification

```text
0 Units
1 Notional
```

followed by:

```text
unit_profile_ref
u64 atomic_units
text FIN-ECO-001 AssetId
```

The quantity profile owns scale/lot/fraction semantics. `Units` does not mean whole shares by itself.

### Order terms

```text
0 Market
1 Limit
2 Stop
3 StopLimit
```

Each included price is:

```text
pricing_profile_ref
u64 quote_atomic_units
text FIN-ECO-001 quote AssetId
```

The pricing profile defines the denominator/base-lot and scale semantics. A quote amount alone is not a market-data observation.

For `StopLimit`, V1 requires both trigger prices to use the exact same pricing profile and quote `AssetId`. A provider-specific successor may impose additional ordering/tick-size rules.

### Time in force

```text
0 Day
1 GoodTilCanceled
2 ImmediateOrCancel
3 FillOrKill
```

A provider adapter must separately prove that its current semantics support the selected class. No silent downgrade is permitted.

### Semantic idempotency reference

```text
idempotency_profile_ref
protocol_id semantic_id
```

This identity is independent of provider order IDs and provider client-order IDs.

### Optional upstream economic effect

```text
0                              // absent
1 || [32] exact commitment     // present
```

It is correlation/binding input only. Its presence does not grant authority.

## Validation / positive-type boundary

V1 rejects:

- malformed protocol IDs;
- empty/oversized/control-containing external IDs;
- zero quantity/notional;
- zero limit or stop price;
- stop-limit prices whose price profile or quote asset differs;
- non-canonical uppercase/non-hex digest wire input;
- unknown top-level JSON fields.

Raw input does **not** expose a public unchecked canonical-byte function. The canonical transcript is publicly obtainable only from a successfully constructed `CanonicalMarketOrderIntentV1`.

`CanonicalMarketOrderIntentV1` is serializable but deliberately not deserializable. Portable bytes are reconstruction inputs, not a caller-controlled positive theorem.

## Frozen independent vector

Fixture: `test-vectors/order-intent-v1.json`

```text
canonical preimage bytes = 671
SHA-256 = 37d7854bb7012da68bf12cbd76f9a8ca46d3e8364140a6d275168da2f0721f71
```

The r2 hardening changes type/admission boundaries only; the canonical transcript and vector for already-valid inputs are unchanged.

The independent stock-Python oracle in `tests/reference/order_intent_v1_reference.py` reconstructs the transcript directly from the fixture without importing Rust code.

## Identity sensitivity

Changing any effect-significant field changes the commitment, including account, instrument/profile, side, quantity/notional, quantity profile, order terms, price/profile, TIF, execution profile, semantic idempotency identity, or bound upstream effect.

Cancel/replace does not mutate these bytes in place. FIN-MKT-002+ owns explicit order-event/successor lineage.

## Nonclaims

A V1 PASS would establish only deterministic bounded identity for the exact supplied semantic candidate under this profile. It does not establish account ownership, instrument authenticity, market-data truth, suitability, authorization, capacity, provider capability/currentness, credential validity, best execution, order submission, fill, position ownership, settlement, legal/regulatory compliance, tax/accounting correctness, or autonomous Symthaea authority.
