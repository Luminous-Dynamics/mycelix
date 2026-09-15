# IG-007T0 — Treasury/Credit downstream observed profile

## Purpose

IG-007T0 freezes the source-visible governance-to-Treasury/Credit boundary that motivates P0s #1085 and #1086.

It is an **observational profile**, not a vulnerability exploit and not a successor design.

The central distinction is:

```text
route continuity
!=
DHT mutation authority
```

The frozen source currently provides positive fail-closed containment at two disconnected call boundaries while separately exposing weak decentralized Treasury/Allocation update predicates.

## Evidence identity

```text
profile   mycelix-treasury-credit-downstream-observed-fca2c107-v1
revision  1
authority ObservedSourceBound
SHA-256   77db81e40ebe7b0ec9278f8c9aad7584faa08b6784cde4779393d1d6b88cf58d
```

Semantic production subject:

```text
fca2c107a1ea5108823ce617ba4111b6f7f77230
```

Evidence-authoring head:

```text
feb30a89257e96592fdbd40b249258581d42fde7
```

The latter is only asserted source-equivalent for the exact bound files. No repository-wide tree-equivalence claim is made.

The evidence also binds:

```text
mycelix-governance/crates/governance-utils/src/lib.rs
blob 282888816cc101c2743ef5c5905119defc3fee6d
```

because the cross-role containment theorem depends on `governance_utils::call_role` propagating transport, network, and unexpected-response failures as `Err` rather than using the separate best-effort helper.

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

The exact-head qualifier scans the complete bound eight-module governance-bridge coordinator census and requires there to be **no exported `pub fn transfer_credits(...)`**.

The historical profile therefore preserves:

```text
missing target
=> call error
=> TransferCredits action fails closed
```

This is positive containment. It is not an unauthorized-transfer result.

## Plane B — explicit governance→Finance bridge API

The visible bridge API is instead:

```text
execute_approved_transfer(ApprovedTransferInput {
    proposal_hash,
    recipient_did,
    amount_sap,
    purpose,
})
```

which targets:

```text
finance role / treasury::execute_governance_transfer
```

The exact-head qualifier requires `execute_approved_transfer` to remain present while requiring **no exported `pub fn execute_governance_transfer(...)`** in the exact frozen Treasury coordinator.

The call uses the exact bound `governance_utils::call_role` helper. That helper returns `Err` on transport errors, network errors and unexpected zome responses, so the missing-target observation is legitimately fail-closed rather than an assumed property of the caller.

Again, this is modeled as fail-closed route disconnection, not a successful transfer.

## Plane C — actual Treasury allocation coordinator

The source also contains a distinct, more explicit allocation path:

```text
propose_allocation
  proposal_id: Option<String>

approve_allocation
  Proposed only
  caller DID binding
  approver must be Treasury manager
  manager-majority -> Approved

execute_allocation
  Approved only
  checked-sub debit
  optimistic reread/retry
```

This is positive evidence for coordinator-level safeguards.

It does not establish a decentralized integrity theorem by itself.

A separate DKG-gated large-allocation path is retained as a distinct mechanism, including fail-closed cross-role signature verification. It is not treated as an alias for the legacy `TransferCredits` route.

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

The profile therefore records no observed reconstruction of:

- Treasury-update author authority;
- balance-change authority;
- manager-change authority;
- Allocation transition authority;
- `approved_by` authority;
- proposal authorization;
- immutable transfer-subject/value fields.

That is the separate P0 #1086 theorem.

## Why the split matters

A disconnected canonical route can be safer than a permissive route because it fails closed, while still being operationally incomplete.

Conversely, a coordinator can enforce sensible manager rules while the underlying DHT validator remains too weak to reject publications from a modified client.

IG-007T0 therefore refuses either simplification:

```text
missing call target == funds vulnerable       // false claim
coordinator checks == decentralized authority // also false claim
```

## Positive containment registry

The profile requires all six observations to remain explicit:

1. legacy missing target fails closed;
2. bridge→Finance missing target fails closed through the bound `call_role` helper;
3. legacy amount is positive and finite;
4. Treasury debit uses checked subtraction;
5. ordinary coordinator approval is manager-majority;
6. DKG path fails closed on signature-verification failure.

## Claim ceiling

IG-007T0 does **not** establish:

- `AuthorizedGovernanceValueTransfer`;
- `GovernanceToTreasuryAuthorityContinuity`;
- `DecentralizedTreasuryMutationAuthority`;
- `DecentralizedAllocationTransitionAuthority`;
- Treasury/Credit deployment currentness;
- Treasury/Credit governance safety.

## Child

IG-007T1 / #1092 converts these observations into six deterministic source-contract fixtures with frozen corpus commitment:

```text
89b795cea150c1d0f18aa63bc6adfe2ffd31389ef9396bbde810f647540695de
```

Historical T0/T1 evidence must remain immutable after a repair. Successor evidence should show which old counterexamples intentionally stop reproducing.
