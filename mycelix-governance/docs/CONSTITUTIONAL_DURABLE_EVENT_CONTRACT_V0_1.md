# Constitutional Durable Event Contract v0.1

Status: **InertContractPendingQualifiedClaimBinding / experimental / unqualified**  
Tranche: **MYC-CONST-003D1D-E0**  
Contract revision: **3**  
Tracks: #1627  
Predecessor: **MYC-CONST-003D1D0**

## Purpose

D1D0 established that the current `GovernanceAction::EmitEvent` path is an ephemeral `emit_signal` projection whose return value is discarded, while the existing durable `GovernanceBridgeEvent` path is not yet constitutional-authority-grade because its integrity rules permit updates and do not authority-bind the event index.

D1D-E therefore separates two concepts that must never again be conflated:

```text
constitutional event truth
        !=
client signal delivery
```

E0 makes the first half compile-testable without exposing a new Holochain write surface before ClaimBinding/refinement authority is qualified and integrated.

## Why E0 is inert

The current execution action contains only:

```text
EmitEvent {
    event,
    payload,
}
```

It does not carry:

- operation identity;
- action identity;
- proposal identity at the provider boundary;
- ClaimBinding commitment;
- action commitment.

Registering a new DHT event entry immediately would therefore create a durable write surface before the integrity zome could prove that the write was authorized by the exact constitutional action.

E0 intentionally avoids that mistake.

It adds only a Rust contract crate and a machine-readable contract profile. It does **not** add an entry type, link type, coordinator extern, or execution dispatch change.

## Durable event identity

The provider key is:

```text
action_id
```

The semantic event commitment includes:

```text
schema_version
operation_id
action_id
proposal_id
claim_binding_commitment
action_commitment
publisher_did
event_name
payload_commitment
```

It deliberately excludes:

```text
committed_at_unix_ms
projection attempt identity/time/outcome
```

This means a retry five seconds or five days later does not become a second constitutional event merely because wall-clock time changed.

## D1C identity continuity

E0 must not invent a second operation/action identity system beside the D1C effect ledger.

The mapping is exact:

```text
E0 operation_id              <- ConstitutionalOperation.operation_id
E0 proposal_id               <- ConstitutionalOperation.proposal_id
E0 claim_binding_commitment  <- ConstitutionalOperation.claim_binding
E0 action_id                 <- ActionIntent.action_id
E0 action_commitment         <- ActionIntent.action_commitment
E0 publisher_did             <- provider-specific authority input
```

The event-provider crate therefore has a **dev-only** dependency on `constitutional-effect-ledger`. Its integration tests first validate the D1C operation and `ActionIntent`, then construct E0 authority directly from those exact fields.

The tests specifically establish that:

- E0 does not regenerate `operation_id`;
- E0 does not regenerate `action_id`;
- E0 does not regenerate `action_commitment`;
- D1C `claim_binding` is the opaque value carried as E0 `claim_binding_commitment`;
- an action commitment that fails `ActionIntent::validate_against_operation` is not accepted as a valid mapping;
- retry wall-clock time does not turn the mapped D1C action into a second E0 event.

This is a **structural compatibility** result only. E0 does not inherit a qualification claim from D1C, and the opaque ClaimBinding value remains blocked from positive authority claims until the qualified 003B4/CR1 chain is integrated.

## Canonical payload and commitments

The payload is converted to deterministic compact JSON:

- object keys are recursively sorted;
- whitespace is removed by serialization;
- array order is preserved because array order is semantic;
- payload size is bounded to 64 KiB;
- the resulting bytes receive a domain-separated BLAKE3-256 commitment.

The durable event receives a second domain-separated BLAKE3-256 commitment over the full event semantics.

Both generated commitments are self-describing:

```text
blake3-256:<64 lowercase hexadecimal digits>
```

This avoids depending on out-of-band knowledge of the digest algorithm or width. Validation requires the exact algorithm tag and lowercase 256-bit encoding, then re-parses and re-canonicalizes stored payload text and recomputes both commitments. A mutated payload, malformed commitment encoding, or commitment mismatch therefore fails contract validation.

This canonicalization is the E0 contract's own defined JSON encoding; it is not presented as a claim of RFC 8785/JCS equivalence.

## Duplicate semantics

E0 defines a closed publish decision:

```text
no existing action_id
        -> Created

same action_id
same semantic event commitment
        -> ExistingSame

same action_id
different semantic event commitment
        -> IntegrityConflict
```

There is intentionally no:

```text
Overwrite
Replace
LastWriteWins
```

A conflicting event under the same constitutional action identity is evidence of an integrity problem, not a newer version of the truth.

Changing any of these under the same action ID changes the event commitment:

- proposal;
- operation;
- ClaimBinding commitment;
- action commitment;
- publisher DID;
- event name;
- canonical payload.

## Projection semantics

`SignalProjectionAttempt` references:

```text
projection_attempt_id
operation_id
action_id
durable_event_commitment
attempt_ordinal
attempted_at_unix_ms
outcome
```

with outcomes:

```text
Delivered
Failed { error_class }
UnknownOutcome
```

Projection attempts must bind to the exact durable event, but their outcome is not part of constitutional completion.

Projection diagnostics are nevertheless held to append-only evidence discipline. A projection history must have:

```text
unique projection_attempt_id
attempt_ordinal = 1, 2, 3, ... without gaps
attempted_at_unix_ms non-decreasing
```

So signal delivery remains non-authoritative while its operational evidence cannot silently contain duplicate attempt identities, missing attempts, or backwards time ordering.

Therefore:

```text
Durable event valid
        ↓
projection history coherent
        ↓
constitutional event complete
        ↓
projection attempt fails or is unknown
        ↓
constitutional event STILL complete
        ↓
projection may be retried as next diagnostic attempt
```

This is the desired semantics for WebSocket/UI/client signals: they are disposable, replayable views of durable truth, while their diagnostics remain auditable.

## Authority ceiling

E0 carries both:

```text
claim_binding_commitment
action_commitment
```

but does not claim that those opaque strings have yet been verified against qualified runtime authority.

That positive claim remains blocked on at least:

```text
003B4 ClaimBinding qualification
003CR1 refinement-crosswalk qualification
integration of exact binding verification at the DHT integrity boundary
```

The `publisher_did` is structurally required to be a DID, but E0 does not claim that a future DHT action author is already bound to it. That is an E1 integrity responsibility.

## E1 activation gate

Only E1 may add a Holochain entry/link/write surface.

E1 must establish all of the following before `GovernanceAction::EmitEvent` can be redirected:

1. a dedicated immutable constitutional-event entry;
2. an action-key index whose base/target relationship is integrity-validated;
3. action author / authority binding that survives malicious coordinator bypass;
4. qualified ClaimBinding/refinement verification;
5. same-key/same-payload idempotent lookup;
6. same-key/different-payload fail-closed conflict handling;
7. durable event commit before signal projection;
8. authoritative lookup by action identity;
9. exact-head tests and adversarial mutation controls.

The existing mutable `GovernanceBridgeEvent` remains a general bridge event facility and is not silently promoted into constitutional authority.

## Tests in E0

The Rust crate covers:

- recursive JSON key canonicalization;
- array-order preservation;
- self-describing lowercase BLAKE3-256 commitment encoding;
- timestamp independence of event identity;
- first publish -> `Created`;
- duplicate semantic retry -> `ExistingSame`;
- changed payload under same action -> `IntegrityConflict`;
- changed ClaimBinding under same action -> `IntegrityConflict`;
- changed publisher under same action -> `IntegrityConflict`;
- payload commitment tampering;
- event commitment tampering;
- malformed/uppercase commitment encoding;
- rejection of non-canonical stored payload text;
- DID-shaped publisher requirement;
- projection failure not revoking constitutional completion;
- projection retry without creating a new constitutional event;
- duplicate projection attempt IDs rejected;
- projection ordinal gaps rejected;
- projection time regression rejected;
- rejection of projection references to another action/event;
- rejection of lookup state keyed to the wrong action;
- direct structural identity mapping from validated D1C operation/action records;
- rejection of a D1C action commitment that does not match its operation plan.

## Non-claims

D1D-E0 does not establish:

- Holochain DHT event persistence;
- a callable durable-event provider;
- qualified ClaimBinding verification;
- qualified constitutional authority;
- live `EmitEvent` repair;
- deployment currentness;
- exactly-once signal delivery;
- D1D-E1 qualification.

Those omissions are deliberate. E0 exists so the event semantics can be tested and frozen before a new authority-bearing persistence surface exists.
