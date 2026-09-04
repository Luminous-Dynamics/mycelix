# Mycelix Content Fabric — Xenia Typed Reader Request v0.1

Status: CF-07C3 executable draft stacked on CF-07C2.

## Purpose

CF-07C3 is the first boundary that releases public resource-read authority from the Xenia reader chain.

It consumes `EnrollmentBoundOpenedPayloadV1`, whose authenticated plaintext and enrolled `RemoteReaderV1` remain crate-private, and does not expose either independently. Instead it:

1. decodes exactly one fixed Content Fabric request schema;
2. validates the exact Nix store hash with the existing `NixStoreHashV1` parser;
3. evaluates the still-sealed reader against a current `RemoteServingSnapshotV1` at a trusted server-supplied request time; and
4. releases `AuthorizedNixReadV1` only if CF-07A returns the exact entry for that reader/hash.

The authority chain is therefore:

```text
Xenia hybrid peer authentication
        ↓
same owned carrier
        ↓
AEAD + replay acceptance
        ↓
Content Fabric Xenia domain 0xCF
        ↓
pinned CF-07C1 enrollment
        ↓
sealed reader + plaintext
        ↓
exact CF-07C3 request parse
        ↓
current CF-07A serving lookup
        ↓
operation + deadline bound AuthorizedNixReadV1
```

## Fixed wire schema

The v1 plaintext request is exactly 40 bytes:

```text
offset  size  field
0       4     family magic = ASCII "MCFR"
4       2     schema_version = u16 big-endian, exactly 1
6       1     operation = 1 NarInfo | 2 Nar
7       1     reserved = 0
8       32    Nix store hash ASCII text
```

`MCFR` identifies the Content Fabric reader-request protocol family. The separate u16 field is the sole schema-version authority, so future schema versions do not inherit a contradictory version digit from the framing magic.

There are no variable-length fields.

The decoder rejects:

- any length other than 40 bytes;
- any magic other than `MCFR`;
- any schema version other than `1`;
- any operation other than `1` or `2`;
- any non-zero reserved byte;
- invalid UTF-8 in the store-hash field; and
- any store hash rejected by `NixStoreHashV1::parse`.

Trailing bytes are not ignored. Future semantics require a new schema version rather than smuggling meaning into an old decoder.

## Authority facts intentionally absent from plaintext

The request contains no:

- `PartyIdV1`;
- group ID;
- enrollment ID;
- hybrid credential ID;
- enrollment-registry ID;
- Xenia transcript hash;
- serving snapshot/projection/policy ID;
- authority principal;
- evaluation timestamp;
- arbitrary URL/path; or
- artifact trust assertion.

Those facts come from authenticated server-side state, not client plaintext.

## Trusted request time

`authorize_bound_nix_read_v1(...)` requires `request_time_unix_ms` as a separate server-side input.

The request cannot choose the time at which serving authority is evaluated. CF-07A checks:

```text
evaluation_time <= request_time < serve_until
```

before returning any entry.

A stale or not-yet-valid serving snapshot is an error, not an authorization miss.

## Non-enumerating lookup

CF-07A deliberately maps both:

```text
entry absent
```

and:

```text
entry exists but reader lacks audience authority
```

to `Ok(None)`.

CF-07C3 preserves that behavior. It does not add an oracle that distinguishes private-object existence from absence.

## Operation- and deadline-bound output

`AuthorizedNixReadV1` is privately constructed and deliberately has no `Debug`, `Clone`, `Copy`, serialization, `Default`, or public conversion constructor.

It internally binds:

- exact `NixReaderOperationV1`;
- exact CF-07A-authorized `NixCacheEntryV1`;
- serving snapshot ID;
- serving projection ID;
- exposure policy ID;
- trusted authorization time;
- exclusive `serve_until_unix_ms` deadline;
- CF-07C1 registry/enrollment/credential IDs;
- Xenia transcript generation;
- negotiated context commitment; and
- same-carrier receive sequence.

There is no public generic `entry()` accessor.

For current `NarInfo` authority:

```text
render_narinfo_at(now) -> Some(body)
nar_sha256_digest_at(now) -> None
nar_size_at(now) -> None
```

For current `Nar` authority:

```text
render_narinfo_at(now) -> None
nar_sha256_digest_at(now) -> Some(exact SHA-256)
nar_size_at(now) -> Some(exact size)
```

At or after the exclusive serving deadline, all three representation-capability methods return `None`.

This prevents a metadata-only authorization from becoming a raw-NAR capability and prevents the ordinary public API from yielding representation facts after the evidence horizon has expired.

A large raw-NAR transfer may span time. The next byte-serving layer must therefore continue checking the same deadline during the stream rather than only before opening the CAS object.

## Defense-in-depth returned-entry check

`RemoteServingSnapshotV1::entry_for_reader_at` is already keyed by the parsed `NixStoreHashV1`.

CF-07C3 nevertheless rechecks:

```text
returned_entry.store_path().hash() == parsed_request.store_hash
```

before releasing `AuthorizedNixReadV1`.

If future CF-07A internals are refactored incorrectly, a mismatched entry fails closed instead of becoming authority.

## Nix trust remains independent

CF-07C3 answers only:

> May this authenticated/enrolled reader retrieve this representation of this exact store-hash entry from this current serving-authority snapshot, and until when?

It does **not** answer:

> Should Nix trust/install this artifact?

Nix signature/trusted-key policy remains independent. A malicious or compromised authorized cache path must not become a software trust root merely because transport, identity, and disclosure authority are valid.

## Next serving boundary

The next narrow tranche should adapt `AuthorizedNixReadV1` to bytes without broadening authority:

```text
NarInfo
    → deadline-checked exact rendered metadata body

Nar
    → deadline-checked exact SHA-256 + exact size
    → CF-03 verified CAS open
    → continuously enforce serve-until while streaming
    → stream only the verified immutable bytes
```

The serving adapter must not accept an arbitrary digest/path in addition to `AuthorizedNixReadV1`, because that would reintroduce resource substitution after authorization.

## Qualification expectations

Focused tests pin:

- exact 40-byte canonical encoding;
- trailing-byte rejection;
- closed-world magic/schema/operation/reserved-byte decoding;
- reuse of the existing exact Nix store-hash parser;
- successful reader+hash lookup produces operation/snapshot/deadline/audit-bound authority;
- NarInfo cannot expose raw-NAR retrieval facts;
- representation facts disappear at the exclusive serving deadline;
- unauthorized and absent store hashes both remain `None`; and
- stale serving snapshots fail before authority is released.

Keep this tranche draft until exact-head Rust 1.96 and Rust 1.94 CI execute successfully.
