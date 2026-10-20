# FIN-SYNC-001 canonical settlement graph format v1

Status: source specification for the FIN-SYNC-001 candidate. A source commit or review does not establish executable qualification.

## Claim boundary

This format identifies one bounded multi-leg settlement graph over one exact economic effect.

It does **not** establish business or institutional authority, scarce-capacity reservation, rail/provider capability, dispatch, per-leg settlement, PvP/DvP synchronization, legal finality, commercial satisfaction, or compensation authority.

The V1 graph is therefore a semantic object, not a bearer capability.

## Important V1 design decisions

### Graph identity is derived, not caller supplied

V1 uses the full SHA-256 graph commitment as `graph_id`. A caller cannot preserve a friendly graph identifier while mutating an amount, asset, party, rail, profile, dependency, role, or group.

### Leg identity is derived in graph context

A leg commitment binds the exact economic-effect commitment, graph profile, adapter profile, rail/network, typed leg role, ordered source/destination subjects, exact FIN-ECO-001 asset/amount, asset/unit profile, finality profile, semantic idempotency reference, optional purpose profile, and optional delivery/asset subject.

The construction alias is deliberately excluded.

```text
same visible transfer + different economic effect
!= same settlement-leg identity

provider transaction ID
!= Mycelix settlement-leg identity
```

### Source -> destination is the V1 direction

V1 does not carry a redundant direction enum. The ordered `source_subject -> destination_subject` pair is authoritative. A future direction taxonomy may be added only through a new profile if it adds information not already represented by ordered subjects, role and purpose profile.

### Typed leg roles prevent structural label laundering

V1 uses a closed role vocabulary:

```text
1 = Payment
2 = Delivery
3 = Auxiliary
```

Role-local rules:

- `Payment` must not carry `delivery_asset_subject`;
- `Delivery` must carry `delivery_asset_subject`;
- `Auxiliary` must carry an explicit `purpose_profile`.

These rules establish only graph semantics. They do not prove that an underlying rail can execute the role.

### Construction aliases are non-authority

Leg aliases and group aliases exist only to author a graph before semantic IDs are known. They are dropped from the positive graph and never enter commitment bytes. Renaming aliases while preserving the exact semantic graph therefore preserves identity.

### Single-leg settlement stays in FIN-ECO

FIN-SYNC V1 requires at least two legs. A single external settlement operation remains owned by FIN-ECO-002.

### Profile identity does not imply profile support

This pure crate commits exact adapter, asset/unit, finality, graph, coordination, predicate and temporal profile references. It does not maintain a live registry of operational support. FIN-SYNC-003 owns rail-capability/profile compatibility.

```text
exact profile reference
!= profile currently supported
```

## Common encoding rules

All canonical hashes use SHA-256. All domains are ASCII bytes including the trailing NUL. All integers use unsigned big-endian representation.

- commitment-profile revision: `u16`;
- profile revision: `u64`;
- atomic amount: `u64`;
- collection/string lengths: `u32`.

Text is encoded as:

```text
u32 byte_length || UTF-8 bytes
```

`BoundedText` is non-empty, at most 256 UTF-8 bytes and contains no control characters.

A `Commitment32` is exactly 32 raw bytes in canonical encodings and lower-case 64-character hexadecimal in JSON.

A semantic profile reference is:

```text
text(profile_id)
|| u64(profile_revision)
|| raw_32_byte_profile_digest
```

An optional value is `0x00` for absent or `0x01 || encoded_value` for present.

Collections encode `u32(count) || elements` after the semantic ordering rule for that collection is applied.

## Leg commitment

Domain:

```text
MYCELIX_FIN_SYNC_LEG_V1\0
```

Bytes:

```text
domain
|| u16(1)
|| economic_effect_commitment
|| graph_profile
|| adapter_profile
|| text(rail)
|| text(network)
|| u8(role)
|| text(source_subject)
|| text(destination_subject)
|| text(amount.asset)
|| u64(amount.atomic_units)
|| asset_unit_profile
|| required_finality_profile
|| text(semantic_idempotency_ref)
|| optional(purpose_profile)
|| optional(text(delivery_asset_subject))
```

`leg_id = SHA256(bytes)`.

The construction alias is absent. Zero-amount legs are rejected.

## Dependency encoding and ordering

Dependencies use a closed typed byte representation:

```text
1 = Requires
2 = Before
3 = ConditionalOnEvidence
```

`Requires`:

```text
0x01 || leg_id || prerequisite_id
```

Its graph edge is `prerequisite -> leg`.

`Before`:

```text
0x02 || before_id || after_id
```

`ConditionalOnEvidence`:

```text
0x03 || leg_id || predicate_profile
```

Conditional evidence does not create an ordering edge.

The complete canonical dependency byte strings are sorted lexicographically as unsigned bytes. Duplicate canonical dependency byte strings are rejected.

This rule is deliberately independent of Rust enum declaration order, derived `Ord`, serde layout, display strings, or source-code refactoring.

## Coordination groups

Domain:

```text
MYCELIX_FIN_SYNC_GROUP_V1\0
```

Class tags:

```text
1 = PvP
2 = DvP
3 = AllOrNone
4 = Saga
```

Bytes:

```text
domain
|| u16(1)
|| graph_profile
|| u8(class)
|| u32(member_count)
|| member_leg_ids sorted ascending by raw digest bytes
|| coordination_profile
```

`group_id = SHA256(bytes)`.

V1 cardinality/composition:

- PvP: exactly two `Payment` legs;
- DvP: exactly one `Payment` leg and one `Delivery` leg;
- AllOrNone: 2..64 legs;
- Saga: 2..64 legs.

A PvP/DvP class is therefore not a free label over arbitrary members. It still remains structural metadata only:

```text
well-formed PvP/DvP group
!= underlying synchronization primitive qualified
!= synchronized settlement occurred
```

V1 allows each leg to belong to at most one coordination group, preventing ambiguous overlapping group semantics before a later theorem defines safe overlap.

Each leg must carry a graph-unique Mycelix semantic idempotency reference. Provider bearer tokens and provider-native request IDs remain outside this object.

PvP, DvP and AllOrNone are strongly coupled structural groups. Saga is not.

## Residual dependency DAG

Ordinary `Requires` and `Before` dependencies must be acyclic after strongly coupled groups are collapsed to one structural node.

An ordinary dependency whose endpoints collapse into the same strong group is rejected. Coupling must be represented by the group rather than by a fake ordering cycle.

A mutual requirement that truly means all-or-none belongs in an explicit coordination group, not in `Requires(A,B) + Requires(B,A)`.

## Connectivity

After resolving ordinary leg-to-leg dependencies and coordination-group membership, all legs must belong to one connected undirected semantic component. This prevents an unrelated bag of transfers from acquiring one graph identity merely because the caller submitted them together.

`ConditionalOnEvidence` constrains one leg but does not connect otherwise independent legs.

## Graph commitment

Domain:

```text
MYCELIX_FIN_SYNC_GRAPH_V1\0
```

Bytes:

```text
domain
|| u16(1)
|| economic_effect_commitment
|| graph_profile
|| optional(temporal_profile)
|| u32(leg_count)
|| leg_ids sorted ascending by raw digest bytes
|| u32(dependency_count)
|| canonical dependency byte strings sorted lexicographically
|| u32(group_count)
|| group_ids sorted ascending by raw digest bytes
```

`graph_commitment = SHA256(bytes)` and V1 sets `graph_id = graph_commitment`.

The graph encoder sorts semantic collections itself even though the positive constructor also normalizes them. Canonical identity therefore does not depend on incidental in-memory collection order.

## Resource limits

V1 admits at most 64 legs, 256 dependencies, 32 coordination groups, 64 members per group and 256 UTF-8 bytes per bounded text field.

Inputs outside these bounds fail before graph construction. The crate forbids unsafe Rust and its graph builder has no intentional panic path for representable bounded input.

## Frozen independent vector

`test-vectors/settlement-graph-v1.json` is independently derived from the format above.

Expected commitments:

```text
USD leg:
a0fca8a0301e14743716ec3c280c0179dca915310e90818b40db69f38930cd86

EUR leg:
3c00858264be8a13449bea08c8ca2ba592b799d490795300b264e7f1b4c7b709

PvP group:
cf5254fed98de1d0f8dd8922f280130d69feada95e1820a73fbe064643977f5e

graph:
3cac9d88d535025a357b2cbfc18d48606e726dec7fa3ccd57b81b11f345a662d
```

A qualification lane must reconstruct these values independently rather than using the Rust canonicalization code as its own oracle.
