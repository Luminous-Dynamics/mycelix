# GOVSYS-003A-R Rust Root-A Conformance v0.1

Status: **production semantic-port qualification contract**

This tranche ports the already-qualified GOVSYS-003A Root-A semantic/identity theorem into a
small Rust `no_std + alloc` crate without changing the constitutional contract.

Qualified source theorem:

`235834fc726c1a79467e6f910a2cf39fb9abd2e3`

The governing requirement is cross-language equality:

```text
same exact Root-A semantics
        ↓
qualified Python oracle
        ==
Rust canonical implementation
```

A discrepancy is a contract defect. Rust is not permitted to silently choose a different
normalization, stricter identity transcript, or broader accepted semantic domain.

## Crate boundary

`crates/mycelix-constitutional-root`

The production crate is deliberately small:

- Rust 1.98.1;
- `#![no_std]` plus `alloc`;
- no Holochain/HDK/network/runtime dependency;
- no persistence or wall clock;
- no serde dependency;
- SHA-256 through pinned direct dependency `sha2 = =0.10.9` with default features disabled;
- no authority/currentness/effect machinery.

Transport parsing is outside the production semantic core. In particular, JSON hex case is a
representation issue; the Rust semantic types carry raw 32-byte digests.

## Exact frozen profiles

Root identity:

`mycelix-constitutional-trust-root-v1-sha256-framed-semantic`

Source descriptor:

`mycelix-constitutional-root-source-descriptor-v1-sha256-framed-semantic`

Rotation authority:

`mycelix-constitutional-root-rotation-authority-v1-sha256-framed-semantic`

All domain separators, field order, little-endian u64 framing, optional-value encoding, set
sorting and digest semantics reproduce the qualified Python oracle exactly.

## Positive-origin boundary

`ConstitutionalRoot`, `Rulebook`, and `AuthorizedPolicyScope` are **unqualified semantic input**.
They are intentionally constructible by ordinary callers.

Only local `qualify_root()` may create `QualifiedConstitutionalRootA`.

The qualified type:

- owns the exact validated root;
- stores locally recomputed complete Root-A identity;
- stores locally recomputed source-descriptor identity;
- stores the locally recomputed optional rotation-authority identity;
- has private fields;
- has no serde dependency or deserialization path;
- exposes read-only getters; and
- explicitly grants neither currentness nor effect authority.

This boundary is needed so downstream #848/#846 can consume locally recomputed Rust semantics
without treating caller-supplied digest assertions as qualified facts.

## Exact semantic parity

Rust reproduces the v1 rules for:

- exact protocol version;
- nonempty bounded UTF-8 text;
- leading/trailing ASCII-space rejection;
- ASCII-control-byte rejection;
- nonzero 32-byte semantic digests;
- constitutional and provider rulebook shape;
- generation-zero predecessor absence;
- successor predecessor presence;
- exact bootstrap-mode registry;
- bounded authorized policy-scope count;
- policy-profile self-authorization denial;
- duplicate semantic scope denial;
- duplicate `(policy_identity_profile, policy_registry_namespace)` provider-key denial;
- expiry strictly after `valid_from_ms`;
- exact rotation-mode registry;
- immutable root carries neither rotation profile nor rotation-authority anchor; and
- predecessor-authorized root carries both a valid profile and nonzero anchor.

The Rust input model uses native `u64`, so Python's JSON integer type/range checks become a
transport-decoding concern before semantic qualification. Canonical semantic values remain exact.

## Canonical authorized-policy set

Authorized policy scopes are an unordered semantic set for Root-A identity.

Rust encodes every validated scope independently, sorts the encoded byte strings lexically, then
commits the count and concatenated sorted encodings exactly as Python does.

Caller ordering cannot change Root-A identity.

Two entries sharing the same policy-profile/registry-namespace key remain invalid even if their
provider details differ. Multi-provider policy currentness requires a separately named profile.

## Derived trust-role separation

Source descriptor commits only:

```text
authoritative_root_source_ref
root_coverage_profile
root_source_verification_profile
root_source_anchor_digest
```

Rotation authority commits only:

```text
rotation_profile
rotation_authority_anchor_digest
```

Therefore:

```text
source descriptor != rotation authority
```

Changing one anchor changes only its derived role identity while the complete Root-A identity
changes because it commits both roles.

## Golden cross-language identities

Original qualified Root-A fixture:

```text
root
c3f9ba9b323f20c2d2ebd597e424857e8f3459d31393886fd6a7f835a6a93d6d

source descriptor
f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126

rotation authority
cf86718a53410b1e5e38cf5c552b3e4448772da97533fbd08c7b9bbfbee909ae
```

Exact #839 transition predecessor:

```text
root
b7a0c7cb28f182d06367d4bf1d3cc4f7d82094701d955eeaa7ec53153d2080a6

source descriptor
f97a96e20ce6dd0c86c67e1590252abc678dfd6401128ab44ced4a66ec088126

rotation authority
db3bf05b04063ea2e9626e9ea63def4920f205cb36c43d64a046fd04e87b8304
```

Exact #839 transition successor:

```text
root
c970bfc0957efc00946d08a957d7617b85815f47d98fcc98aa43f6f6a6ced963

source descriptor
3f867aab08093b08f8f4088a540e8cffa85dc25ab7c7cfb34ac65c8bc071a3fe

rotation authority
ee35844f067306e66ec8fcb40442b3564e9f7f5bd824a3e2a71265ea318aebea
```

The checked-in Rust conformance example prints these nine identities. Hosted qualification runs
the exact qualified Python oracle over the same semantic fixtures and byte-diffs Python output
against Rust output.

## Hosted gate

The dedicated gate must establish:

- exact ancestry from qualified Root-A head `235834fc...`;
- exact child review surface and no parent-oracle/vector modification;
- exact Python Root-A oracle re-execution;
- Python conformance output for original + transition-pair fixtures;
- Rust 1.98.1 exact toolchain;
- rustfmt;
- all-target unit tests;
- warnings-denied Clippy;
- wasm32 `no_std` check;
- Rust conformance example output;
- exact Python-vs-Rust output diff;
- immutable checkout; and
- final fail-closed aggregation.

The semantic test corpus covers ordering invariance, duplicate/ambiguous scopes, self-authorizing
profiles, generation/predecessor relations, invalid validity windows, zero digests, exact rotation
shape, source/rotation independence, and text/control-byte constraints.

## Dependency-evidence boundary

The direct hashing dependency is exactly pinned to `sha2 0.10.9`, but this first semantic port does
not yet claim a checked-in exact transitive Cargo dependency graph. A clean dependency-closure
child should freeze the isolated Cargo lock before production promotion, following CORE-LINEAGE
#849 rather than conflating semantic conformance with package-resolution evidence.

## Nonclaims

`QualifiedConstitutionalRootA` proves only that the exact semantic value satisfies the frozen
Root-A contract and that the three Root-A identities were locally recomputed.

It does **not** establish:

- bootstrap provenance;
- predecessor transition authorization;
- historical lineage;
- global uniqueness;
- source coverage/currentness;
- legal/democratic legitimacy;
- ordinary policy authority;
- actor authority;
- execution authority; or
- external-effect authority.

The network remains infrastructure for institutions. It is not the sovereign.
