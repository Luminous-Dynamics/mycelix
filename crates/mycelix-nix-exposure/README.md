# mycelix-nix-exposure

`mycelix-nix-exposure` is the CF-07A fail-closed authority layer between the loopback-only CF-07 Nix cache and any future non-loopback edge server.

It opens **no socket** and mints **no authorization evidence**.

## Boundary

```text
CF-07 local catalog
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
      +--> deterministic replay projection
      |
      +--> serving horizon from SAME evidence
      |
      v
RemoteServingSnapshotV1
      |
      v
future authenticated/non-loopback adapter
```

A future server should accept `RemoteServingSnapshotV1`, never `NixCacheCatalogV1` or the general `RemoteExposureSnapshotV1` directly.

## Atomic serving projection

`RemoteExposureSnapshotV1` remains public for replay and diagnostics, including intentionally partial/non-strict views.

Remote serving authority is different. Public code must call `project_remote_serving_snapshot_v1(...)`. The lower-level promotion constructor is crate-private so callers cannot project with one revocation set and then omit a scheduled revocation while deriving a longer serving deadline.

The same evidence slices therefore answer both:

1. what may be exposed now; and
2. how long that result may be served.

## Strict serving type

`RemoteServingSnapshotV1` requires:

- exact policy/endpoint binding;
- `CompleteForAuthority` coverage;
- a policy that itself requires complete coverage;
- `CryptographicallyVerified` minimum assurance;
- a non-zero refresh interval; and
- grant evidence for every exposed grant.

An operator-managed or weaker remote mode, if ever required, should use a separately named type rather than weakening this one.

## Bounded serving lifetime

The exclusive serving deadline is:

```text
min(
    evaluation_time + refresh_interval,
    every exposed grant valid_until,
    every exposed grant authored_at + policy.max_evidence_age,
    every visible policy-admissible scheduled revocation effective_at
)
```

Below-policy, foreign-issued, and pre-grant revocations are ignored consistently by projection and horizon derivation.

The public serving lookup is time-bearing:

```text
entry_for_reader_at(store_hash, reader, now_unix_ms)
```

It refuses to return entries before the projection time or at/after `serve_until`.

## Authority-bound endpoint

`ExposureEndpointIdV1` binds one canonical endpoint label to one `PartyIdV1` publication authority. Wire/storage reconstruction takes authority + label + claimed ID, recomputes the commitment, and rejects rebinding.

A grant issuer must equal the endpoint authority.

## Exact Nix publication binding

`NixExposureObjectIdV1` commits the complete CF-07 publication object:

- store path;
- raw-NAR digest and size;
- references;
- deriver;
- existing `Sig:` values; and
- optional `CA:`.

Changing those facts requires a new remote grant even when the raw NAR bytes are unchanged.

## Assurance and coverage provenance

`SelfClaimed`, `OperatorVerified`, and `CryptographicallyVerified` are labels supplied by an upstream evidence adapter. CF-07A enforces but does not mint them.

`CompleteForAuthority` is likewise an assertion about the concrete policy authority, not global finality.

A later Xenia/operator verifier should be the component that establishes these facts before constructing CF-07A evidence.

## Revocation

Revocation is append-only and causal:

- future-authored evidence is invisible to earlier replay;
- already-authored future-effective revocations remain inert until their boundary but shorten the serving horizon;
- below-policy evidence is inert;
- foreign or pre-grant evidence is ignored/diagnosed;
- sufficiently assured same-authority evidence revokes at `effective_at`;
- policy-admissible evidence weaker than the original grant becomes `RevocationUnresolved` and fails closed at its effective boundary.

## Readers

The v0.1 audiences are public, authenticated principal, and authenticated group.

`RemoteReaderV1` has sealed constructors. CF-07A does not authenticate network traffic; the future edge adapter must turn authenticated transport identity into these reader facts.

Unauthorized and absent content should remain externally indistinguishable where existence privacy matters.

## Stable commitments

Domain-separated IDs cover endpoint, publication object, grant, revocation, policy, replay projection, and serving-safe snapshot. The serving ID additionally commits refresh configuration and the derived serving deadline.

These are audit/replay commitments, not proof that upstream assurance labels were honestly assigned.
