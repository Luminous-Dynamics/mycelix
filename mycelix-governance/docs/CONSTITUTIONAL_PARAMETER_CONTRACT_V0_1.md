# Constitutional Parameter Contract v0.1

Status: **InertContractPendingQualifiedAuthority / experimental / unqualified**  
Tranche: **MYC-CONST-003D1D-P0**  
Tracks: **#1626**  
Parent: **MYC-CONST-003D1D0** semantic head `7bb43d0af6590f3794956d12c47d6fa376b59821`

## Purpose

D1D0 established that the current `GovernanceAction::UpdateParameter` path is not a safe constitutional effect provider.

The legacy caller sends only:

```text
{ parameter, value }
```

while the constitution coordinator accepts:

```text
UpdateParameterInput {
    parameter,
    value,
    proposal_id?,
}
```

and requires a proposal ID when the parameter already exists. A missing parameter can enter the create path without one.

The current integrity zome checks that the stored value parses as JSON, but it does not prove that the writer or `changed_by_proposal` is constitutionally authorized and it accepts `ParameterIndex` link creation without authority binding. Current lookup then chooses the latest index link by timestamp.

P0 therefore does **not** patch the live zome. It first freezes the provider semantics a later P1 integrity implementation must preserve.

## Split

The parameter repair lane is explicitly split:

```text
P0 — inert value / request / revision / retry contract
P1 — Holochain integrity + coordinator activation
```

P0 is safe to implement before B4/CR1 qualification because it creates no Holochain entry type, link type, coordinator extern, or execution dispatch change.

## Stable identities

P0 reuses D1C identities rather than inventing a provider-local identity system:

```text
P0 operation_id              <- ConstitutionalOperation.operation_id
P0 proposal_id               <- ConstitutionalOperation.proposal_id
P0 claim_binding_commitment  <- ConstitutionalOperation.claim_binding
P0 action_id                 <- ActionIntent.action_id
P0 action_commitment         <- ActionIntent.action_commitment
P0 publisher_did             <- provider-specific authority input
```

`action_id` is the provider retry/idempotency key.

`parameter_name` is the stable parameter namespace key.

P0 does not regenerate either identity.

The ClaimBinding value remains opaque until the B4/CR1 refinement chain is qualified and integrated.

## Typed value profile

P0 does not use an arbitrary JSON string as constitutional parameter identity.

The exact v1 value types are:

```text
Integer(i64)
Decimal(decimal-string-v1)
Percentage(percentage-decimal-string-v1)
DurationMillis(u64)
Boolean
String(UTF-8)
```

### Why decimals are strings

Constitutional numeric identity should not depend on IEEE-754 behavior.

P0 therefore forbids exponent notation in Decimal/Percentage and normalizes lexical forms:

```text
001.2300   -> 1.23
-000.000   -> 0
0005       -> 5
```

The following are not accepted as decimal-string-v1:

```text
1e3
1E3
+1.25
NaN
Infinity
```

Decimal and Percentage canonical JSON is a JSON string plus an independent type tag. This means:

```text
Decimal("1.25") != String("1.25")
```

Duration is explicitly milliseconds rather than an unversioned textual duration convention.

P0 does not claim that this is a backwards-compatible reinterpretation of every legacy parameter value. Legacy migration is a separate future theorem.

## Request identity

A `ParameterMutationRequest` commits to:

```text
schema_version
operation_id
action_id
proposal_id
claim_binding_commitment
action_commitment
publisher_did
parameter_name
typed canonical value
expected prior revision
```

Generated commitments are domain-separated BLAKE3-256 values encoded:

```text
blake3-256:<64 lowercase hexadecimal digits>
```

Wall-clock time is not request identity.

## Compare-and-set predecessor rule

The first revision requires:

```text
CreateOnly
```

Every later revision requires:

```text
Exact {
    revision: current_revision,
    revision_commitment: current_revision_commitment,
}
```

So a new action cannot silently write against stale state.

A stale predecessor produces:

```text
StaleRevision
```

and does not mutate history.

The caller must reconcile/replan from the current revision rather than blindly retrying the stale request.

## Append-only revision chain

Revision numbering is:

```text
1, 2, 3, ...
```

without gaps.

Each revision after revision 1 commits to the exact previous revision commitment.

A stored revision also reconstructs the exact request commitment that should have produced it from:

```text
revision number
predecessor commitment
authority binding
parameter name
typed canonical value
```

This is important: an attacker cannot replace the inner request commitment and merely recompute the outer revision commitment to create a self-consistent-looking forged revision.

`committed_at_unix_ms` is audit metadata and deliberately excluded from revision identity.

## Historical action idempotency

P0 maintains two independent indexes:

```text
revision number -> ParameterRevision
action_id       -> exact revision number
```

The action index is historical, not just a pointer to current state.

Example:

```text
action A -> revision 1
action B -> revision 2
action C -> revision 3
```

If action A is redelivered after revision 3, P0 resolves it to revision 1:

```text
same action A + same request
    -> ExistingSame(revision 1)
```

It does not create revision 4, and it is not rejected merely because the parameter advanced.

This makes retry identity independent from current projection state.

## Conflicting duplicate

If one `action_id` is reused with different request semantics:

```text
same action_id
+ different value / proposal / ClaimBinding / predecessor / authority
    -> IntegrityConflict
    -> preserve existing history
    -> halt this parameter history
```

There is no:

```text
Overwrite
Replace
LastWriteWins
```

P0 intentionally defines no recovery-from-halt authority. Recovery deserves a separate governed protocol.

## Current state

The current parameter value is defined as the highest valid contiguous revision in the revision chain.

It is **not**:

```text
latest link timestamp
latest wall-clock timestamp
last record returned by an unconstrained index
```

P1 must preserve this distinction at the DHT integrity/query boundary.

## Current legacy findings retained from D1D0

Exact runtime subject:

```text
15b9c89adf0ac3c6c5a73681614d6bfcd368820a
```

Source-visible findings:

- execution dispatch sends `{parameter,value}` and omits `proposal_id`;
- existing-parameter mutation requires `proposal_id` only in coordinator logic;
- missing parameters can enter the create path without proposal authority;
- integrity only checks that parameter value parses as JSON;
- execution does not canonicalize the logical parameter value before dispatch;
- `set_parameter` appends a new parameter entry and a new `ParameterIndex` link;
- `get_parameter` selects the latest index link by timestamp;
- parameter entry integrity does not bind `changed_by_proposal` to qualified authority;
- parameter entry integrity does not bind the writer to qualified governance authority;
- `ParameterIndex` creation is accepted without parameter-authority validation;
- Phi config synchronization is best-effort and therefore a derived projection.

P0 does not rewrite those historical observations.

## Phi projection semantics

The constitution parameter revision is constitutional truth.

Bridge Phi configuration is downstream projection/cache state:

```text
durable qualified parameter revision
        ↓
optional Phi config synchronization
```

Therefore:

```text
Phi sync success   != parameter authorization
Phi sync failure   != parameter revision rollback
Phi cache freshness != constitutional completion
```

A future projection reconciler may make cache state more observable, but it must not become the authority source for the parameter.

## P1 activation requirements

P1 may add Holochain persistence only after it can establish at least:

1. qualified B4 ClaimBinding semantics;
2. qualified CR1 refinement mapping;
3. exact qualified D1C operation/action identity;
4. exact-head P0 qualification;
5. DHT action-author to `publisher_did` binding;
6. exact proposal/ClaimBinding authorization at integrity validation;
7. immutable append-only `ConstitutionalParameterRevision` entries;
8. integrity-validated parameter-name, revision and action indexes;
9. direct-DHT stale predecessor rejection;
10. direct-DHT duplicate/conflicting action tests;
11. authoritative query by parameter revision and action identity;
12. exact-head P1 qualification.

Only after those gates should `GovernanceAction::UpdateParameter` be redirected.

## P0 tests

The pure crate covers:

- integer lexical normalization;
- deterministic decimal normalization;
- exponent/plus rejection;
- typed Decimal/String identity separation;
- deterministic string JSON escaping;
- CreateOnly genesis rule;
- revision-1 commit;
- same-action/same-request idempotency;
- delayed historical retry after newer revisions;
- conflicting same-action halt;
- stale compare-and-set no mutation;
- exact compare-and-set predecessor chaining;
- revision-to-request commitment reconstruction;
- wall-clock independence;
- proposal and ClaimBinding participation in identity;
- predecessor participation in request identity;
- malformed/uppercase commitment rejection;
- action-index drift detection;
- predecessor-chain drift detection;
- structural D1C operation/action identity reuse without regeneration.

## Non-claims

P0 does not establish:

- Holochain parameter persistence;
- a callable parameter provider;
- qualified ClaimBinding verification;
- qualified proposal/writer authority;
- safe legacy parameter migration;
- live execution repair;
- Phi projection delivery/currentness;
- deployment currentness;
- P1 qualification.

Those omissions are deliberate. P0 freezes deterministic parameter semantics before an authority-bearing write surface exists.
