# SSF Canonical Wire v0.1

Provider-neutral byte-level grammar for future SSF canonical semantic encoders.

This crate defines only wire primitives and domain framing. It does **not** yet claim that the canonical replay-evidence decision subject is fully encodable, and it performs no hashing, signing, verification, evidence qualification, replay authorization, or effect authorization.

## v0.1 rules

- fixed protocol magic is emitted verbatim;
- wire version is an unsigned 16-bit big-endian integer;
- semantic domains use explicit stable unsigned 16-bit tags;
- integers have fixed widths and big-endian encoding;
- fixed-size commitments are emitted verbatim at profile-fixed widths;
- optional values use explicit `None = 0` / `Some = 1` tags;
- booleans use explicit `False = 0` / `True = 1` tags;
- variable byte strings use a u32 big-endian length prefix;
- Rust memory layout, `Debug`, native endianness, enum declaration order, and implicit enum discriminants are never canonical wire format.

The first closed domain tag is reserved for `EvidenceFitnessForAtMostOneReplay`. Later semantic encoding profiles must assign explicit field order and explicit tags for every nested enum they encode.

A future crypto adapter may implement `CanonicalWireSinkV1` directly over a transcript or hasher, but this crate itself computes no digest. Deterministic bytes and cryptographic authentication remain separate claims.
