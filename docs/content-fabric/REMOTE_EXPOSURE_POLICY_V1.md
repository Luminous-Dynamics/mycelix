# Mycelix Content Fabric — Remote Exposure Policy v0.1

Status: CF-07A executable draft.

## Purpose

CF-07 deliberately remains loopback-only. CF-07A defines the authority boundary that must exist before a later adapter may expose Nix cache content beyond loopback.

The central rule is:

> local cache exposure is not remote publication authority.

A `NixCacheCatalogV1` entry means only that the local CF-07 facade may expose the object on its contained loopback interface. It MUST NOT be treated as proof that the object may be disclosed on LAN, WAN, public HTTP, or another authenticated edge network.

## Two projection surfaces

CF-07A has two deliberately different outputs.

`RemoteExposureSnapshotV1` is the general deterministic replay/diagnostic projection. It may represent partial evidence or a non-strict policy and MUST NOT be used directly as remote serving authority.

`RemoteServingSnapshotV1` is the strict, time-bounded artifact intended for a future non-loopback adapter.

```text
CF-07 NixCacheCatalogV1
        +
exact grant/revocation evidence set
        +
explicit evaluation time
        +
authority coverage assertion
        +
strict remote policy
        +
refresh interval
        |
        v
project_remote_serving_snapshot_v1(...)
        |
        +--> deterministic RemoteExposureSnapshotV1
        |        (internal serving projection; replay API remains separately available)
        |
        +--> serving-horizon derivation from the SAME evidence slices
        |
        v
RemoteServingSnapshotV1
        |
        v
future authenticated/non-loopback adapter
```

### Atomic serving projection invariant

Public serving artifacts MUST be produced with `project_remote_serving_snapshot_v1(...)`.

The lower-level `RemoteServingSnapshotV1::from_projection(...)` constructor is crate-private. This is intentional: a caller must not be able to project current authority using one revocation set and then omit known scheduled revocations while deriving a longer serving horizon.

The exact grant/revocation slices supplied to the public serving function therefore determine both:

1. whether an object is remotely exposable at the evaluation time; and
2. how long that result may safely be served.

## Strict serving admission

Serving construction requires all of the following:

1. projection and policy bind the same endpoint/policy ID;
2. evidence coverage is `CompleteForAuthority`;
3. the policy itself requires complete authority coverage;
4. the policy minimum grant assurance is exactly `CryptographicallyVerified`;
5. the refresh interval is non-zero; and
6. every exposed grant is present in the supplied evidence set.

A future operator-managed or otherwise weaker remote mode must use a separately named type/constructor. It MUST NOT weaken `RemoteServingSnapshotV1`.

## Authority-bound endpoint scope

Every `ExposureEndpointIdV1` carries both:

- a logical endpoint stable ID; and
- the `PartyIdV1` publication authority that owns that endpoint.

The stable ID is domain-separated from authority + canonical endpoint label. Wire/storage reconstruction requires authority + label + claimed ID and recomputes the commitment. A copied endpoint ID therefore cannot be rebound to another authority or label.

Grant construction requires:

```text
grant.issuer == endpoint.authority
```

and projection defensively checks the same authority relation.

## Assurance provenance

`ExposureAssuranceV1` distinguishes:

- `SelfClaimed`;
- `OperatorVerified`;
- `CryptographicallyVerified`.

These are input labels. CF-07A enforces them but does not mint or cryptographically prove them.

Likewise, `CompleteForAuthority` is supplied by the upstream evidence adapter for the concrete `policy.authority()`. It is not global distributed-system finality.

The intended later path is:

```text
signature / Xenia / operator attestation
        |
        v
verified authority adapter
        |
        v
truthfully labeled grant/revocation evidence + authority coverage
        |
        v
CF-07A
```

## Exact Nix publication binding

Remote grants bind `NixExposureObjectIdV1`, which commits:

- exact store path;
- NAR digest algorithm and bytes;
- NAR size;
- ordered references;
- deriver state/value;
- ordered existing `Sig:` values; and
- optional `CA:` state/value.

Changing `.narinfo` trust/dependency metadata therefore requires a new exposure grant even if the raw NAR bytes remain identical.

## Causal replay and grant validity

Projection never reads the system clock. The caller supplies `evaluation_time_unix_ms`.

Evidence authored after the evaluation time is removed before deduplication, conflict handling, diagnostics, and exposure evaluation. Future-authored evidence therefore cannot alter an earlier replay.

Grant validity is half-open:

```text
valid_from <= evaluation_time < valid_until
```

Policy additionally bounds maximum grant lifetime and maximum grant-evidence age.

## Revocation semantics

Revocation is part of v0.1.

- below-policy revocation evidence is outside the trusted authority universe and is inert;
- foreign-issuer revocations are ignored/diagnosed;
- revocations authored before the grant are ignored/diagnosed;
- a sufficiently assured same-authority revocation removes the grant once `effective_at <= evaluation_time`;
- policy-admissible revocation evidence weaker than the original grant becomes `RevocationUnresolved` at its effective time and fails closed;
- already-authored future-effective revocations are visible but do not revoke early.

Ignored revocations must also be ignored by serving-horizon derivation.

## Serving horizon

`RemoteServingSnapshotV1` is a time-bounded authority artifact, not a permanent capability.

Its exclusive `serve_until_unix_ms` is:

```text
min(
    evaluation_time + refresh_interval,
    every exposed grant valid_until,
    every exposed grant authored_at + policy.max_evidence_age,
    every visible, same-authority, policy-admissible scheduled revocation effective_at
)
```

A below-policy, foreign-issued, or pre-grant revocation does not shorten the horizon because it does not affect the projected authority state.

The serving snapshot receives a separate domain-separated ID committing:

- projection ID;
- policy ID;
- configured refresh interval; and
- derived serving deadline.

## Time-bearing serving lookup

Serving code MUST use:

```text
entry_for_reader_at(store_hash, reader, now_unix_ms)
```

The lookup fails before returning an entry when:

```text
now_unix_ms < evaluation_time
or
now_unix_ms >= serve_until_unix_ms
```

Thus stale serving authority cannot be used merely because a handler forgot a separate freshness check.

A future HTTP adapter should map stale/not-yet-valid serving snapshots to service-unavailable/refresh-required behavior. Authorization denial and object absence should remain externally indistinguishable where existence privacy matters.

## Audience scope

The v0.1 audience model is:

- `Public`;
- `AuthenticatedPrincipal(PartyIdV1)`; and
- `AuthenticatedGroup(StableIdV1)`.

`RemoteReaderV1` uses sealed constructors so zero principals/groups and non-canonical group sets cannot be created through the public API.

CF-07A does not authenticate requests. A future edge adapter must map transport-authenticated identity into validated reader facts before querying the serving snapshot.

## Stable identities

CF-07A uses explicit domain-separated `StableIdV1::derive` commitments for:

- authority-bound endpoint identity;
- exact Nix exposure object;
- grant;
- revocation;
- projection policy;
- effective projection; and
- serving-safe snapshot.

It does not hash arbitrary JSON/Serde output.

These IDs are replay/audit commitments, not proof that upstream assurance or coverage labels were honestly assigned.

## Required future server invariant

A future non-loopback Nix server MUST NOT have constructors equivalent to:

```text
new(cas, NixCacheCatalogV1, bind_addr)
```

or:

```text
new(cas, RemoteExposureSnapshotV1, bind_addr)
```

Its authority-bearing constructor must require `RemoteServingSnapshotV1` plus an authenticated-reader adapter appropriate to the configured audience modes, and handlers must use the time-bearing lookup.

## What CF-07A is not

CF-07A is not a network listener, TLS/mTLS implementation, bearer-token service, Holochain authority, Xenia verifier, grant signer, publication-governance engine, billing/lease layer, CF-03 integrity replacement, or Nix software-trust root.

## Deferred

- cryptographic grant signer/format;
- Xenia verification adapter;
- Holochain publication-authority coordination;
- TLS/mTLS termination;
- HTTP Basic/Bearer/netrc compatibility;
- authenticated principal/group extraction;
- public DNS/certificate policy;
- non-loopback socket implementation;
- dynamic snapshot refresh;
- remote fill;
- settlement/lease coupling.
