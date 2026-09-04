# Mycelix Content Fabric — Xenia Reader Bridge v0.1

Status: CF-07C2 executable draft.

## Purpose

CF-07C2 is the narrow bridge between Xenia's authenticated application-channel evidence and Mycelix Content Fabric's application reader identity.

It closes this substitution gap:

```text
cryptographically opened request bytes from peer A
        +
reader identity asserted separately as Party B
        x
one authenticated reader request
```

The bridge instead consumes Xenia's sealed `OpenedPeerApplicationPayloadV1` directly and maps the exact authenticated hybrid peer keys through one explicitly pinned CF-07C1 enrollment registry snapshot.

## Upstream Xenia proof required

CF-07C2 depends on exact Xenia review commit:

```text
2125d0db2a5b7a995d129646e851adacb1c9f1fa
```

from stacked Xenia PR #281.

That proof type already composes:

1. real Ed25519 + ML-DSA-65 peer authentication;
2. exact owned transport used by that handshake;
3. stable transport/pre-session/availability profiles around receipt;
4. one configured Xenia application payload domain;
5. ChaCha20-Poly1305 authentication; and
6. replay-window acceptance.

CF-07C2 does not reimplement or weaken those checks.

The git dependency is pinned by commit rather than branch/tag. Any future Xenia evidence-surface change therefore requires an explicit dependency-review update in this crate. Dedicated CI also inspects resolved Cargo metadata and requires the resolved `xenia-peer-core` source to end at the exact review SHA above.

## Content Fabric Xenia domain

CF-07C2 reserves:

```text
CONTENT_FABRIC_READER_PAYLOAD_TYPE_V1 = 0xCF
```

inside Xenia's `0x30..=0xff` application range.

A sealed Xenia proof from any other application payload type is rejected before enrollment lookup.

This byte is **not** the Content Fabric request schema. It provides cryptographic/replay stream separation only. Plaintext inside the successfully opened payload still needs a versioned, bounded, typed Content Fabric request decoder before resource authority is evaluated.

## Registry commitment pin

The bridge requires:

```text
OpenedPeerApplicationPayloadV1
        +
ReaderEnrollmentRegistryV1
        +
expected_registry_id
```

and checks:

```text
registry.id() == expected_registry_id
```

before performing identity lookup.

The expected registry commitment must come from higher-level trusted deployment/policy state. CF-07C2 does not make that ID self-authenticating.

The purpose of this check is narrower: a server that intends to use registry snapshot `R` cannot silently be handed snapshot `R2` and map the same authenticated Xenia keys to a different principal/group set.

## Exact hybrid enrollment

After payload-domain and registry-pin checks, CF-07C2 reconstructs the exact CF-07C1 hybrid credential from Xenia's authenticated:

- Ed25519 public key; and
- ML-DSA-65 public key.

It then calls `ReaderEnrollmentRegistryV1::lookup_keys(...)`.

The lookup is exact on both keys. There is no:

- Ed25519-only fallback;
- ML-DSA-only fallback;
- key-fingerprint-to-PartyId derivation;
- transcript-hash-to-PartyId derivation; or
- caller-supplied PartyId/group override.

The selected enrollment supplies the non-zero `PartyIdV1` and canonical group set used to construct `RemoteReaderV1`.

## Output evidence

`EnrollmentBoundOpenedPayloadV1` internally binds:

```text
AEAD/replay-opened Content Fabric-domain plaintext
        +
RemoteReaderV1 from exact enrollment
        +
registry snapshot ID
        +
enrollment ID
        +
hybrid credential ID
        +
Xenia transcript generation
        +
negotiated context commitment
        +
carrier receive sequence
```

The type is privately constructed and deliberately has no `Debug`, `Clone`, `Copy`, `Serialize`, `Deserialize`, `Default`, or `From` constructor surface.

### Authority remains sealed before schema validation

CF-07C2 intentionally does **not** expose public `plaintext()` or `reader()` accessors.

Both the authenticated plaintext and `RemoteReaderV1` remain crate-private because the plaintext has not yet passed the Content Fabric request schema. External code can inspect only non-sensitive audit commitments/generation metadata:

- registry ID;
- enrollment ID;
- hybrid credential ID;
- transcript hash;
- negotiated context hash; and
- carrier receive sequence.

CF-07C3 must live in this crate, consume `EnrollmentBoundOpenedPayloadV1`, parse exactly one bounded request schema, and only then release a public typed operation together with the already-bound reader.

This prevents a downstream caller from extracting `RemoteReaderV1` early and pairing it with a different resource operation.

## What the output proves

A successful CF-07C2 output proves the following composition was accepted:

```text
real Xenia hybrid authentication
        ↓
same peer-bound carrier
        ↓
AEAD + replay acceptance
        ↓
Content Fabric Xenia payload domain 0xCF
        ↓
exact pinned enrollment registry snapshot
        ↓
exact hybrid-key enrollment
        ↓
explicit Mycelix PartyIdV1 + groups
        ↓
reader + plaintext remain sealed together
```

## What the output does NOT prove

It does not prove:

- the plaintext is a syntactically valid Content Fabric request;
- the request schema version is supported;
- requested operation/path/store hash is valid;
- the bound reader may access the requested object;
- CF-07A serving authority is current;
- the bytes exist in local CAS;
- the bytes pass CF-03 verification; or
- Nix trusts the artifact's signature/key.

Those remain later checks.

## Required next composition

The next layer should consume `EnrollmentBoundOpenedPayloadV1` internally and perform exactly one request-schema parse:

```text
EnrollmentBoundOpenedPayloadV1
        |
        +-- exact bounded Content Fabric request schema parse
        +-- semantic validation
        |
        v
public typed reader request
        |
        +-- bound RemoteReaderV1 released only now
        +-- requested store/object identity
        |
        v
CF-07A RemoteServingSnapshotV1::entry_for_reader_at(...)
```

A failed schema decode must not trigger guessing another authority-bearing protocol or payload domain.

## Failure ordering

CF-07C2 evaluates:

1. exact Xenia application payload domain;
2. exact pinned registry commitment;
3. exact hybrid enrollment;
4. CF-07A reader construction.

This ordering means plaintext does not get to choose its own principal or registry snapshot, and the constructed reader cannot escape publicly before typed request validation.

## Tests required for v0.1

The executable model pins:

- exact hybrid enrollment projects expected principal/groups internally;
- canonical groups survive into the sealed `RemoteReaderV1`;
- wrong Xenia application payload domain fails;
- wrong expected registry commitment fails;
- same Ed25519 + different ML-DSA does not enroll;
- same ML-DSA + different Ed25519 does not enroll;
- group membership comes only from the exact pinned snapshot; and
- audit links preserve registry/enrollment/credential/transcript/context/receive-generation identity.

## Deferred

- signed/provenanced source of the expected registry commitment;
- dynamic enrollment refresh/retirement/revocation ceremony;
- typed Content Fabric Xenia request schema;
- typed response schema;
- direct CF-07A serving-snapshot lookup;
- listener/TLS/DNS policy;
- long-lived Xenia application-channel rekey;
- stock Nix proxy adaptation; and
- Nix signature/trusted-key policy.
