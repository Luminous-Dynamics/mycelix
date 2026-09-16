# Mycelix Finance settlement commitment format v1

This document specifies the language-neutral canonical byte encoding used by
`mycelix-finance-settlement` for FIN-ECO-002A.

It defines deterministic commitments and a pure settlement-qualification
theorem. It does **not** authenticate provider truth, establish generic
Holochain finality, prove legal/commercial discharge, or turn caller-supplied
time into trusted current time.

## Cryptographic profile

All v1 commitments use SHA-256 over the canonical bytes defined below.

No JSON serialization, Rust enum discriminant, locale-specific rendering,
native-endian integer, floating-point value, action-hash ordering, or debug
string participates in a canonical hash.

Changing field meaning, field order, encoding, or a domain prefix requires a
new commitment profile revision/domain. Old bytes are never reinterpreted
under new semantics.

## Primitive encodings

- `u8`: one byte.
- `u16`: two bytes, unsigned big-endian.
- `u32`: four bytes, unsigned big-endian.
- `u64`: eight bytes, unsigned big-endian.
- `Digest32`: exactly 32 raw bytes.
- UTF-8 string/reference: `u32 byte_length || utf8_bytes`.
- execution-attempt reference: reference encoding of its canonical
  `ReferenceId`.
- ordered reference set: deduplicate exact reference values, sort ascending by
  the raw UTF-8 bytes of each reference value, encode the count as `u32`, then
  encode each already-sorted reference as `u32 byte_length || utf8_bytes`.
  The length prefix does not participate in ordering.
- ordered digest set: `u32 count || sorted raw 32-byte digests`.
- `AssetAmount`: asset-id UTF-8 string followed by atomic-units `u64`.

Lengths count UTF-8 bytes. The reference-set ordering rule is additionally
illustrated by `REFERENCE_SET_ORDERING_V1.md` and
`test-vectors/reference-ordering-v1.json`.

## Finality profile commitment v1

Domain prefix, including terminal NUL:

`MYCELIX_FINANCE_SETTLEMENT_FINALITY_PROFILE_V1\0`

Fields, in exact order:

1. commitment profile revision — `u16`; v1 requires `1`;
2. finality profile ID — reference;
3. finality semantic revision — `u64`, non-zero;
4. rail — reference;
5. network — reference;
6. required evidence kinds — canonical ordered reference set;
7. minimum distinct sources — `u16`, non-zero;
8. maximum observation age in milliseconds — `u64`, non-zero;
9. reversal model — `u8`:
   - `0x00`: `MayReverse`;
   - `0x01`: `DeclaredTerminalByProfile`.

The digest stored in `FinalityProfileRef` is:

`SHA256(canonical_profile_bytes)`

`FinalityProfile::new` derives this digest. `validate()` recomputes it, so
changing any qualification-significant profile field while preserving the old
digest fails closed.

## Evaluation-context commitment v1

Domain:

`MYCELIX_FINANCE_SETTLEMENT_EVALUATION_CONTEXT_V1\0`

Fields:

1. commitment profile revision — `u16 = 1`;
2. context class — `u8`:
   - `0x00`: `DeterministicSupplied`;
   - `0x01`: `HistoricalReplay`;
3. evaluation Unix milliseconds — `u64`;
4. temporal profile ID — reference;
5. temporal profile revision — `u64`, non-zero;
6. temporal context/evidence digest — `Digest32`.

The commitment is:

`SHA256(canonical_evaluation_context_bytes)`

There is deliberately no caller-constructible `TrustedCurrent` tag. A future
runtime theorem may authenticate a temporal context and produce a stronger
currentness proof. The pure kernel proves only qualification relative to the
exact committed evaluation context.

## Evidence commitment v1

Domain:

`MYCELIX_FINANCE_SETTLEMENT_EVIDENCE_V1\0`

Fields:

1. commitment profile revision — `u16 = 1`;
2. evidence ID — reference;
3. settlement subject ID — reference;
4. operation ID — reference;
5. operation revision — `u64`;
6. observation ID — reference;
7. evidence kind — reference;
8. evidence source — reference;
9. evidence payload/source digest — `Digest32`.

The commitment is:

`SHA256(canonical_evidence_bytes)`

Operation-specific evidence is therefore bound to the exact observation and
revision it supports. Evidence for revision 1 cannot silently become evidence
for revision 2.

If a rail needs revision-independent prerequisite evidence, that must use a
separately versioned evidence profile and explicit join theorem.

## Observation commitment v1

Domain:

`MYCELIX_FINANCE_SETTLEMENT_OBSERVATION_V1\0`

Fields:

1. commitment profile revision — `u16 = 1`;
2. observation ID — reference;
3. settlement subject ID — reference;
4. opaque financial-effect commitment — `Digest32`;
5. execution attempt — reference;
6. rail — reference;
7. network — reference;
8. operation ID — reference;
9. operation revision — `u64`;
10. asset ID — UTF-8 string;
11. current operation amount in atomic units — `u64`;
12. observed state — `u8`:
    - `0x00`: `Applied`;
    - `0x01`: `Pending`;
    - `0x02`: `Rejected`;
    - `0x03`: `Unknown`;
    - `0x04`: `Reversed`;
13. observed-at Unix milliseconds — `u64`;
14. canonical evidence commitments — ordered digest set.

The commitment is:

`SHA256(canonical_observation_bytes)`

### Current-amount semantics

`SettlementObservation.amount` is the exact **current amount represented by the
operation at this revision**, not an incremental event delta.

For example:

```text
op-1 rev 1 Applied 30
op-1 rev 2 Applied 70
op-1 rev 3 Applied 100
```

qualifies the current operation as `100`, not `200`.

A delta-emitting rail must normalize its provider history into this current
revisioned state, or use another registered observation profile.

### Immutable observation identity

Within v1, one `observation_id` identifies one canonical observation body and
one canonical evidence set. Reusing the same observation ID with changed
evidence changes the observation commitment and is a conflict.

Evidence commits to the semantic observation ID/revision rather than the
observation digest, avoiding a commitment cycle.

## Selected-evidence frontier v1

Domain:

`MYCELIX_FINANCE_SETTLEMENT_FRONTIER_V1\0`

Fields:

1. commitment profile revision — `u16 = 1`;
2. settlement subject ID — reference;
3. opaque financial-effect commitment — `Digest32`;
4. execution attempt — reference;
5. rail — reference;
6. network — reference;
7. finality profile commitment — `Digest32`;
8. subject asset ID — UTF-8 string;
9. subject atomic units — `u64`;
10. evaluation-context commitment — `Digest32`;
11. selected current observation commitments — ordered digest set.

The selected evidence frontier is:

`SHA256(canonical_frontier_bytes)`

This value is derived by `qualify_settlement`. It is never accepted as a
caller-selected frontier label.

An external provider, DHT, archive, or evidence-store frontier may be carried
as separate provenance in a later profile, but cannot substitute for the
canonical selected-evidence frontier.

## Qualification order

FIN-ECO-002A v1 uses this fail-closed order:

```text
validate canonical finality-profile commitment
-> validate evaluation-context commitment
-> validate subject/profile/rail/network/attempt/effect
-> group observations by operation
-> select highest revision
-> reject same-revision state/amount disagreement
-> evaluate freshness for every evidence-bearing current observation
-> validate evidence subject/operation/revision/observation binding
-> validate evidence identity/kind/source requirements
-> derive evidence and observation commitments
-> derive selected-evidence frontier
-> sum exact current Applied operation amounts once
-> require exact equality with settlement subject
-> construct constructor-controlled QualifiedSettlement
```

One fresh duplicate cannot make stale same-revision evidence fresh. A stale
current observation fails closed.

`Pending` and `Unknown` remain explicit unresolved states. `Rejected` and
`Reversed` do not contribute settled value. Partial and over-settlement are
distinct failures.

## Financial-effect boundary

The opaque `financial_effect_commitment` is an identity seam only.

FIN-ECO-002 does not define reservation or Business authority. A successor
FIN-ECO-003 theorem derives the exact financial-effect commitment from the
Business/Finance reservation binding and verifies that the settlement proof
carries the same commitment.

Therefore:

```text
same attempt + same amount + same finality profile
!= same financial effect
```

## Invalidation

A historical `QualifiedSettlement` is not rewritten.

`derive_invalidation` accepts only a fresh, higher-revision observation bound
to the same subject, financial effect, attempt, rail/network, asset and
canonical finality profile. The invalidating observation must satisfy the same
evidence kind/source requirements and exact observation/revision evidence
bindings.

A contradictory higher-revision result yields an append-only
`SettlementQualificationInvalidation`. A higher revision with the same
`Applied` amount does not invalidate the prior qualification.

This proves explicit contradiction lineage only; it does not establish that a
provider or court has legally reversed an obligation.

## Golden vector

The machine-readable v1 fixture is:

`test-vectors/settlement-v1.json`

It binds exact canonical bytes and SHA-256 commitments for:

- one finality profile;
- one deterministic evaluation context;
- one operation-specific evidence item;
- one applied observation;
- one selected-evidence frontier.

The multi-reference ordering conformance fixture is:

`test-vectors/reference-ordering-v1.json`

It binds an ordered-reference-set case that distinguishes raw-reference
ordering from sorting complete length-prefixed encodings.

Rust tests load the checked-in fixtures and reproduce their bytes/hashes. Strong
qualification should additionally reconstruct them in an independent
implementation, without calling production Rust.

## Nonclaims

A v1 PASS does not establish provider/bank truth, generic Holochain finality,
global double-spend resistance, legal discharge, commercial satisfaction,
trusted wall-clock time, accounting correctness, tax/regulatory correctness,
or autonomous Symthaea authority. It establishes exact settlement
qualification relative to the canonical policy, evidence set, financial-effect
identity, and explicit evaluation context supplied to the theorem.
