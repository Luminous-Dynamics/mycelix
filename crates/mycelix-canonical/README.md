# mycelix-canonical

Deterministic, domain-separated bytes for signatures, proofs, receipts, and cross-language conformance.

## Why

Security-sensitive Mycelix protocols increasingly bind **exact** actions, policies, receipts, capabilities, and evidence digests. Those digests should not depend on incidental behavior of a JSON serializer, MessagePack struct layout, map ordering, whitespace, or language runtime.

`mycelix-canonical` defines a deliberately restrictive signing format that can be implemented identically in Rust, TypeScript, Python, Xenia, Holochain zomes, and external verifier services.

## Canonical v1

A document contains:

1. fixed magic/version bytes;
2. an ASCII protocol domain;
3. non-zero schema version;
4. strictly increasing numbered fields;
5. explicit type tag;
6. explicit big-endian payload length;
7. exact typed payload bytes.

Supported field types in v0.1:

- machine ASCII;
- byte-exact UTF-8;
- `u16`, `u32`, `u64` big-endian integers;
- boolean;
- 32-byte digest;
- raw bytes;
- ordered ASCII list;
- nested canonical document.

There is intentionally **no generic map and no floating-point field type**. Security protocols that need a numeric quantity should choose an integer unit (basis points, milliseconds, micro-units, etc.) in their schema.

## Domain separation

The domain is inside the signed bytes. Identical payload fields under `governance.execution` and `identity.recovery` therefore produce different bytes and different digests.

This prevents a valid signature from one protocol from being reinterpreted as authorization in another protocol.

## Field evolution

Field tags are schema identifiers, not serialization accidents.

Rules:

- tag `0` is reserved;
- tags must appear once and in strictly increasing order;
- an existing tag must never silently change meaning;
- semantic changes require a new schema version;
- every implemented schema should publish at least one exact-byte + digest conformance vector.

Optional fields are omitted only when that omission is explicitly defined by the schema. A schema must not treat omitted and empty fields as interchangeable unless the protocol says so.

## Text rules

Machine identifiers should use `field_ascii`, which rejects Unicode and control characters.

`field_utf8` is explicitly byte-exact. The canonical layer does not silently normalize Unicode because hidden normalization can itself create interoperability surprises. A higher-level protocol that permits human text must specify any required normalization before calling the canonical writer.

## Digest

v0.1 publishes SHA-256 over the complete canonical bytes.

SHA-256 was selected for the interoperability layer because it is widely available in browsers, operating systems, HSMs, programming languages, standards tooling, and verifier environments. Protocols may carry other cryptographic constructions around this digest, but they should not silently replace the canonical byte contract.

## Published conformance vector

Schema:

- domain: `governance.execution`
- schema version: `1`
- field 1: ASCII `MIP-42`
- field 2: digest32 = 32 bytes of `0x11`
- field 3: u64 = `7`

Canonical bytes (hex):

`4d5943454c49582d43414e4f4e4943414c00010014676f7665726e616e63652e657865637574696f6e0001000101000000064d49502d3432000207000000201111111111111111111111111111111111111111111111111111111111111111000303000000080000000000000007`

SHA-256:

`00019268e6e8cd9b78f98588655e000a14653eecf7a10d3c8d93f1b991c7d3ec`

Any conforming implementation should reproduce both values exactly.

## Planned consumers

The first useful migrations are:

- governance exact action-plan authorization;
- institutional authority grants and rulebooks;
- multi-party credential/recovery event envelopes;
- sovereign-access receipts/capabilities;
- Xenia capability/session transcript bindings;
- SCITT-style evidence/receipt adapters;
- W3C VC proof-domain adapters where a Mycelix-specific canonical payload is required.

The goal is not to invent a replacement for external standards. It is to ensure that Mycelix-specific security boundaries have one small, portable definition of **what bytes were actually authorized**.
