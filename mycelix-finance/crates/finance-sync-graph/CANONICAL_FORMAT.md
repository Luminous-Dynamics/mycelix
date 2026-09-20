# FIN-SYNC-001 canonical settlement graph format v1

Status: source specification for the FIN-SYNC-001 candidate. A source commit or
review does not establish executable qualification.

## Claim boundary

This format identifies one bounded multi-leg settlement graph over one exact
economic effect.

It does **not** establish:

- business or institutional authority;
- scarce capacity reservation;
- rail/provider capability;
- dispatch;
- per-leg settlement;
- PvP/DvP synchronization;
- legal finality;
- commercial satisfaction;
- compensation authority.

The V1 graph is therefore a semantic object, not a bearer capability.

## Important V1 design decisions

### Graph identity is derived, not caller supplied

V1 uses the full SHA-256 graph commitment as `graph_id`.

A caller cannot preserve a friendly graph identifier while mutating an amount,
asset, party, rail, profile, dependency, or group.

### Leg identity is derived in graph context

A leg commitment binds:

- the exact economic-effect commitment;
- the exact graph profile;
- adapter profile;
- rail and network;
- ordered source and destination subjects;
- exact FIN-ECO-001 asset ID and atomic amount;
- exact asset/unit profile;
- required finality profile;
- semantic idempotency reference;
- optional purpose profile;
- optional delivery/asset subject.

The construction alias is deliberately excluded.

Therefore:

```text
same visible transfer + different economic effect
!= same settlement-leg identity
```

and:

```text
provider transaction ID
!= Mycelix settlement-leg identity
```

### Source -> destination is the V1 direction

V1 does not carry a redundant `direction` enum. The ordered
`source_subject -> destination_subject` pair is authoritative.

A future semantic direction taxonomy may be added only through a new profile if
it adds information that is not already represented by ordered subjects and
purpose profile.

### Construction aliases are non-authority

Leg aliases and group aliases exist only to author a graph before semantic IDs
are known. They are dropped from the qualified graph and never enter commitment
bytes.

Renaming aliases while preserving the exact semantic graph therefore preserves
identity.

### Single-leg settlement stays in FIN-ECO

FIN-SYNC V1 requires at least two legs. A single external settlement operation
does not need a synchronization graph and remains owned by FIN-ECO-002.

### Profile identity does not imply profile support

This pure crate commits exact adapter, asset/unit, finality, graph,
coordination, predicate and temporal profile references.

It does not maintain a live registry of which profiles are operationally
supported. FIN-SYNC-003 owns rail-capability/profile compatibility.

```text
exact profile reference
!= profile currently supported
```

## Common encoding rules

All canonical hashes use SHA-256.

All domains are ASCII bytes including the trailing NUL.

All integers use unsigned big-endian representation.

- commitment-profile revision: `u16`;
- profile revision: `u64`;
- atomic amount: `u64`;
- collection/string lengths: `u32`.

Text is encoded as:

```text
u32 byte_length || UTF-8 bytes
```

`BoundedText` is non-empty, at most 256 UTF-8 bytes and contains no control
characters.

A `Commitment32` is exactly 32 raw bytes in canonical encodings and lower-case
64-character hexadecimal in JSON.

A semantic profile reference is:

```text
text(profile_id)
|| u64(profile_revision)
|| raw_32_byte_profile_digest
```

An optional value is:

```text
0x00
```

for absent, or:

```text
0x01 || encoded_value
```

for present.

Collections encode:

```text
u32(count) || element_0 || ... || element_n
```

after applying the semantic ordering rule below.

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

The construction alias is absent.

Zero-amount legs are rejected.

## Dependency encoding

Dependencies are canonicalized as a set and duplicate exact dependencies are
rejected.

Tags:

```text
1 = Requires
2 = Before
3 = ConditionalOnEvidence
```

`Requires` bytes:

```text
0x01 || leg_id || prerequisite_id
```

Its graph edge is `prerequisite -> leg`.

`Before` bytes:

```text
0x02 || before_id || after_id
```

`ConditionalOnEvidence` bytes:

```text
0x03 || leg_id || predicate_profile
```

Conditional evidence does not create an ordering edge.

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
|| sorted_member_leg_ids
|| coordination_profile
```

`group_id = SHA256(bytes)`.

V1 cardinality:

- PvP: exactly 2 legs;
- DvP: exactly 2 legs;
- AllOrNone: 2..64 legs;
- Saga: 2..64 legs.

V1 deliberately allows each leg to belong to at most one coordination group.
This prevents ambiguous overlapping group authority before a later theorem
defines safe overlap semantics.

Each leg must also carry a graph-unique Mycelix semantic idempotency reference.
This prevents two distinct semantic legs from accidentally sharing one replay
identity. Provider bearer tokens remain outside this object.

PvP, DvP and AllOrNone are strongly-coupled structural groups. Saga is not.

A group declaration alone does not prove that the requested coordination
primitive exists or executed.

## Residual dependency DAG

Ordinary `Requires` and `Before` dependencies must be acyclic after
strongly-coupled groups are collapsed to one structural node.

An ordinary dependency whose endpoints collapse into the same strong group is
rejected. Coupling must be represented by the group rather than by a fake
ordering cycle.

A mutual requirement that truly means all-or-none belongs in an explicit
coordination group, not in a `Requires(A,B) + Requires(B,A)` cycle.

## Connectivity

After resolving ordinary leg-to-leg dependencies and coordination-group
membership, all legs must belong to one connected undirected semantic component.

This prevents an unrelated bag of transfers from acquiring one graph identity
merely because the caller submitted them together.

`ConditionalOnEvidence` constrains one leg but does not connect otherwise
independent legs.

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
|| dependencies sorted by their typed semantic ordering
|| u32(group_count)
|| group_ids sorted ascending by raw digest bytes
```

`graph_commitment = SHA256(bytes)`.

V1 sets:

```text
graph_id = graph_commitment
```

## Resource limits

V1 admits at most:

- 64 legs;
- 256 dependencies;
- 32 coordination groups;
- 64 members per group;
- 256 UTF-8 bytes per bounded text field.

Inputs outside these bounds fail before graph construction.

The crate forbids unsafe Rust and its graph builder has no intentional panic
path for representable bounded input.

## Frozen independent vector

`test-vectors/settlement-graph-v1.json` is independently derived from the
format above.

Expected commitments:

```text
USD leg:
59ecbd931e2d2d4fbb49708dda90e630ad2c0b41e1adbfb23b905b3310fb1e3d

EUR leg:
dc97fa0902800488dc6b2a256fd24808ceafa7cb0998778bf60dce4f168638e1

PvP group:
eb83cae7a1017a5a147512563df0d12c336c7c0550e34d30c53ec49880e4149c

graph:
041bcb420a4f263b05b014db7ab785bfa5e01f19047ee4953595e89eb15c9537
```

A qualification lane should reconstruct these values independently rather than
using the Rust canonicalization code as its own oracle.
