# IG-007T0 — Treasury/Credit downstream observed profile

## Purpose

IG-007T0 freezes the source-visible governance-to-Treasury/Credit boundary that motivates P0s #1085, #1086 and #1134.

It is an **observational profile**, not a vulnerability exploit and not a successor design.

The central distinctions are:

```text
route continuity
!=
DHT mutation authority
!=
global large-value policy continuity
```

The frozen source provides positive fail-closed containment at disconnected call boundaries, useful coordinator controls on ordinary/DKG paths, weak decentralized Treasury/Allocation update predicates, and no established theorem that the DKG threshold applies across every Treasury debit path.

## Evidence identity

```text
profile   mycelix-treasury-credit-downstream-observed-fca2c107-v1
revision  1
authority ObservedSourceBound
SHA-256   9a7094cf377e1275f65e59976013390898f456207a27a0236836a535d1c72ef6
```

Semantic production subject:

```text
fca2c107a1ea5108823ce617ba4111b6f7f77230
```

Evidence-authoring head:

```text
feb30a89257e96592fdbd40b249258581d42fde7
```

The authoring head is asserted source-equivalent only for the exact bound files. No repository-wide tree-equivalence claim is made.

## Closed Treasury debit surface

On the exact bound Finance Treasury coordinator, `debit_treasury(...)` has exactly two call sites in addition to its helper definition:

```text
execute_allocation
execute_dkg_allocation
```

The profile therefore records a finite two-path debit surface for this frozen source. This is stronger than an open-ended claim about “relevant entrypoints” and gives IG-007T2 / #1151 an exact universe over which to qualify large-value policy continuity.

## Shared call-helper binding

The profile binds:

```text
mycelix-governance/crates/governance-utils/src/lib.rs
blob 282888816cc101c2743ef5c5905119defc3fee6d
```

Both helper paths matter:

```text
call_local
  call()/transport failure -> propagated Err
  NetworkError             -> Err
  unexpected response      -> Err

call_role
  transport failure        -> Err
  NetworkError             -> Err
  unexpected response      -> Err
```

The exact-head qualifier checks both helper sections independently. Neither containment theorem is inferred from the other, and neither uses the best-effort wrappers.

## Plane A — legacy execution dispatch

The observed execution action is:

```text
TransferCredits { from, to, amount:f64 }
```

with source-visible checks for non-empty `from`/`to` and a positive finite amount.

Its local dispatch target is:

```text
governance_bridge::transfer_credits
```

The exact-head qualifier scans the complete bound eight-module governance-bridge coordinator census and requires there to be no exported `transfer_credits` entrypoint.

The source-visible call chain is therefore fail-closed on a missing target. This is positive containment, not an unauthorized-transfer result.

## Plane B — explicit governance→Finance bridge API

The visible bridge API targets:

```text
finance role / treasury::execute_governance_transfer
```

The qualifier requires `execute_approved_transfer` to remain visible while requiring no exported `execute_governance_transfer` entrypoint in the exact frozen Treasury coordinator.

Again, this is fail-closed route disconnection, not a successful transfer.

## Plane C — Treasury allocation coordinator and DKG continuity

The ordinary allocation path has useful controls:

```text
approve_allocation
  Proposed only
  caller DID binding
  approver must be Treasury manager
  manager-majority -> Approved

execute_allocation
  Approved only
  checked-sub debit
  optimistic reread/retry
  DKG threshold guard: none observed
```

The same coordinator declares:

```text
DKG_THRESHOLD_AMOUNT = 10_000_000_000 micro-SAP
```

and provides a separate `execute_dkg_allocation` path for amounts above that threshold. That path targets:

```text
governance / threshold_signing::verify_threshold_signature
```

while the exact bound threshold-signing coordinator:

```text
mycelix-governance/zomes/threshold-signing/coordinator/src/lib.rs
blob 3449df8b03a4dd1774a5f22756d06931c72855b2
```

contains no exported `verify_threshold_signature` entrypoint. Therefore the special DKG path is disconnected/fail-closed on the frozen source.

At the same time, ordinary `execute_allocation` and shared `debit_treasury` contain no observed DKG threshold/signature predicate. Because the exact debit surface has only those two callers, the source-contract result is precise:

```text
special DKG debit path -> missing verifier -> fails closed
ordinary debit path    -> no DKG threshold guard observed
closed debit surface   -> exactly these two callers

therefore:
GlobalDkgLargeAllocationEnforcement is not established
```

This is P0 #1134. It is not a claim that a live DKG bypass transfer occurred.

## Plane D — Treasury DHT integrity

The frozen integrity code is materially weaker than the coordinator policy:

```text
validate_update_treasury
  ReserveRatioFinite
  ReserveRatioUnitInterval

validate_create_allocation
  RecipientDidShape
  StringLengthBounds
  AmountPositive

validate_update_allocation
  AmountPositive
```

The profile therefore records no observed reconstruction of Treasury-update author authority, balance/manager-change authority, Allocation transition authority, `approved_by` authority, proposal authorization, or immutable transfer-subject/value fields.

That is the separate P0 #1086 theorem.

## Why the split matters

A disconnected canonical route can fail safely while still being operationally incomplete.

A coordinator can enforce sensible manager rules while the DHT validator remains too weak to reconstruct the same authority theorem.

And a special high-assurance path can be fail-closed while its sibling debit path does not consume the same high-value policy.

IG-007T0 therefore rejects all three shortcuts:

```text
missing call target == funds vulnerable       // unsupported
coordinator checks == decentralized authority // unsupported
safe DKG path == global DKG enforcement       // unsupported
```

## Positive containment registry

The profile keeps six local positive observations explicit:

1. legacy missing target fails closed through bound `call_local`;
2. bridge→Finance missing target fails closed through bound `call_role`;
3. legacy amount is positive and finite;
4. Treasury debit uses checked subtraction;
5. ordinary coordinator approval is manager-majority;
6. the DKG entrypoint fails closed when signature verification cannot be established, including the frozen missing-verifier target.

The sixth item is intentionally scoped to the DKG entrypoint and must not be promoted into a global threshold claim.

## Claim ceiling

IG-007T0 does not establish:

- `AuthorizedGovernanceValueTransfer`;
- `GovernanceToTreasuryAuthorityContinuity`;
- `DecentralizedTreasuryMutationAuthority`;
- `DecentralizedAllocationTransitionAuthority`;
- `GlobalDkgLargeAllocationEnforcement`;
- Treasury/Credit deployment currentness;
- Treasury/Credit governance safety.

## Child evidence discipline

IG-007T1 / #1092 remains preregistered as six deterministic source-contract fixtures for #1085/#1086.

IG-007T2 / #1151 owns the separate large-value policy-continuity theorem and must quantify over the closed two-path debit surface.

No child corpus commitment should be frozen while T0's parent profile is still changing. Historical qualified evidence must remain immutable after repairs; successor evidence should show which historical counterexamples intentionally stop reproducing.
