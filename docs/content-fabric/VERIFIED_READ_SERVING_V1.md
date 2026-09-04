# Mycelix Content Fabric — Verified Read Serving v0.1

Status: CF-07C4 executable draft stacked on CF-07C3.

## Purpose

CF-07C4 is the first layer allowed to turn a CF-07C3 `AuthorizedNixReadV1` into representation bytes.

It preserves the authority chain:

```text
Xenia hybrid authentication
        ↓
same peer-bound carrier
        ↓
AEAD + replay acceptance
        ↓
pinned CF-07C1 enrollment
        ↓
exact CF-07C3 typed request
        ↓
current CF-07A serving authority
        ↓
AuthorizedNixReadV1
        ↓
CF-07C4
   ├─ NarInfo → one-shot bounded writer
   └─ Nar     → exact authorized SHA-256 + size
                    ↓
              CF-03 open_verified
                    ↓
              same verified file handle
                    ↓
              sequential bounded reader
```

CF-07C4 does not accept a second caller-supplied digest, size, store hash, filesystem path, URL, reader identity, or operation alongside `AuthorizedNixReadV1`.

## Operation preservation

The two CF-07C3 operations remain distinct.

`NarInfo` authority can only become `AuthorizedNarInfoV1`.

`Nar` authority can only become `AuthorizedNarReaderV1`.

There is no generic representation/body/file conversion that erases the operation distinction.

## Exact CAS binding

For a `Nar` authorization, CF-07C4 obtains the raw-NAR SHA-256 and exact byte size only from the still-current `AuthorizedNixReadV1` capability.

It constructs exactly:

```text
BlobDescriptorV1 {
    digest.algorithm = sha256,
    digest.bytes     = authorized NAR SHA-256,
    size_bytes       = authorized NAR size,
    media_type       = none,
}
```

and passes that descriptor to CF-03 `LocalCasV1::open_verified`.

CF-03 then:

1. rejects an unexpected filesystem object/symlink;
2. opens the immutable digest path with no-follow semantics on Unix;
3. checks the opened file size against the authorized size;
4. hashes the exact opened file handle;
5. requires the exact authorized SHA-256;
6. rewinds that same handle; and
7. returns that handle.

CF-07C4 never reconstructs or reopens a path after verification.

## NarInfo one-shot output

`AuthorizedNarInfoV1` keeps the rendered metadata bytes private.

The only representation operation is a consuming `write_to(...)` call. It writes bounded chunks and checks the inherited serving horizon before and after each write.

There is no public `String`, `Vec<u8>`, clone, serialization, or reusable body accessor.

## Raw NAR reader

`AuthorizedNarReaderV1` owns the exact file handle returned by CF-03.

It deliberately exposes no:

- `Seek`;
- `Clone`;
- `into_file`;
- raw filesystem path; or
- arbitrary descriptor/digest replacement.

Reads are sequential, limited to at most 64 KiB per call, and bounded by the exact authorized byte count.

Unexpected early EOF terminalizes the reader with an integrity error. CF-03's size+digest verification already prevents a valid prepared reader from containing extra trailing bytes outside the authorized object.

## Dual-clock serving horizon

CF-07C4 inherits the exclusive CF-07A `serve_until_unix_ms` horizon carried by CF-07C3 authority.

During preparation, the remaining Unix-time horizon is converted to a private monotonic `Instant` deadline only after any expensive CF-03 full-file verification.

Every bounded read/write checks both:

- the private monotonic deadline; and
- current Unix time against the original `serve_until_unix_ms`.

The two checks address different clock failures:

- monotonic time prevents a wall-clock rollback from extending authority; and
- the Unix check makes a forward wall-clock jump expire authority immediately.

Clock read/conversion failures fail closed.

For raw NAR reads, expiry discovered after a file read zeros the newly read portion of the caller buffer before returning the deadline error and terminalizes the stream.

## Precise release guarantee

CF-07C4 guarantees that **it will not release new representation bytes after its serving authority has expired**.

That is intentionally not phrased as "the remote peer cannot receive bytes after the deadline." Once bytes have already been released to a caller-provided buffer or `Write` implementation, a synchronous library cannot prevent that caller or a buffering transport from delaying their eventual transmission.

Therefore a later socket-owning transport adapter must preserve the same authority horizon through actual network writes and must not prebuffer an entire representation outside the authority-bearing transport path.

Expiry cannot retroactively revoke bytes already disclosed before the deadline.

## Audit facts are not authority

`PreparedNixReadAuditV1` is cloneable for evidence/logging, but intentionally excludes:

- reader principal/groups;
- NAR digest;
- NAR size;
- CAS path;
- file handle;
- NarInfo body; and
- representation bytes.

It carries only operation/store-hash and the serving/enrollment/Xenia commitment chain. Cloning audit evidence does not clone serving capability.

## Terminal behavior

`AuthorizedNarReaderV1` becomes terminal on:

- serving-horizon expiry;
- trusted-clock failure;
- underlying file I/O failure; or
- unexpected early EOF.

After terminalization it cannot resume ordinary reads.

A successfully exhausted reader returns EOF normally and is not treated as a fault.

## Explicit non-claims

CF-07C4 does **not**:

- expose a listener or bind address;
- authenticate a transport;
- mint reader enrollment;
- mint remote-exposure grants;
- refresh a serving snapshot;
- reauthorize a different object;
- fetch missing content remotely;
- compress/rechunk the NAR;
- sign Nix metadata;
- decide Nix `trusted-public-keys`; or
- prove that a buffering/network writer transmitted a byte before the authority deadline.

Nix software trust remains independent of Content Fabric delivery authority.

## Qualification expectations

The focused C4 lane must run:

- Rust 1.96 formatting, all-target check, strict Clippy, tests, and rustdoc;
- Rust 1.94 MSRV check/tests;
- the exact pinned Xenia dependency-source assertion inherited from CF-07C2/C3; and
- CF-03 `mycelix-content-node` tests, because C4's same-handle verified-serving claim depends directly on that implementation.

Focused C4 tests pin one-shot NarInfo behavior, expiry refusal, sequential exact-length NAR reads, bounded read size, early-EOF terminalization, and no disclosure on a pre-read expired deadline.

Keep this tranche draft until exact-head GitHub Actions execute successfully.
