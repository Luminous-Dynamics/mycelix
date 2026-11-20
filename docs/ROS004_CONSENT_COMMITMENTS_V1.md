# ROS-004 consent commitments v1

**Status:** draft owner API, stacked on ROS-007 content-bound consent identity PR #3692. This is an immutable binding tuple, not a live authorization decision or issuer signature.

## API

`Consent::commitments_v1()` validates the record's content-bound `ConsentId` and the canonical scope representation before returning `ConsentCommitmentsV1`. Its fields are private; no raw-digest constructor is exposed.

The tuple contains:
- the exact `ConsentId` used to derive it;
- a consent-profile digest over the complete immutable `Consent::canonical_bytes()`;
- a purpose digest over the typed, tagged canonical `ConsentPurpose`;
- a data-class digest over the complete canonical data-class set.

Each digest uses BLAKE3 over a distinct versioned domain, followed by an unsigned 64-bit little-endian payload length and the canonical payload. The data-class set retains the ROS-004 enum tags, including a length-delimited custom tag. Data-class order is canonicalized by `DisclosureScope::new`; manually assembled noncanonical scope values are rejected by the commitment API.

The consent-profile source bytes already begin with `mycelix.consent-content.v1\0`, which carries the immutable content schema version. The commitment domains are:
- `mycelix.ros004.consent-profile.v1\0`
- `mycelix.ros004.purpose.v1\0`
- `mycelix.ros004.data-class-set.v1\0`

No trimming, case folding, Unicode normalization, prefix matching or wildcard semantics are inferred for purpose or field strings. The current typed purpose is committed as its exact owner-defined tag and UTF-8 bytes.

## Lifecycle boundary

The commitments are invariant across status/event changes because they bind immutable grant content. A revoked or expired consent may still have the same commitment tuple; consumers must call `Consent::authorize()` against current lifecycle state and the exact request before relying on consent. The tuple is not permission to disclose, delegate, invoke a Holochain function, or execute a host effect.

## Known-answer vector

The unit test `consent_commitments_v1_match_known_answer_vectors` freezes three digests for the repository's deterministic test consent fixture:
- Consent profile: `85048ebd83b33fb6719026d6899deefea25843ee25b139f9e72357f765af857a`
- Purpose: `30956c7b670cd14aea3bba594fc2d370041e9af6f2988805006d0193db1bc9fa`
- Data classes: `e9b846aab504f88778ca2ecbeaf96b555aeab1eadc6e0ec7abce78f301c4b295`

These vectors lock this v1 encoding. Any semantic or byte-format change requires a new schema/domain version and explicit cross-consumer migration.

## Qualification scope

The dedicated exact-head workflow pins Rust 1.96.1 and runs formatting, tests and strict Clippy for `mycelix-core-types`. This PR is stacked on #3692; it does not qualify ROS-005 parent-chain semantics, Holochain 0.7.1, DHT currentness, signatures, or downstream Xenia enforcement.


## Independent known-answer fixture

The frozen fixture uses:

- relationship: `RelationshipId::derive("test", b"relationship")`;
- grantor: participant namespace `did`, identifier `alice`;
- audience: participant namespace `did`, identifier `bob`;
- purpose: `ConsentPurpose::Named("relationship-support")`;
- disclosure mode: `Disclose`;
- data classes: `[Contact]`;
- field set: `["email"]`;
- granted-at: `100`;
- expires-at: `Some(200)`;
- authority epoch: `7`.

All hexadecimal strings below are lowercase, with no separators. Integer lengths are unsigned 64-bit little-endian values, except the relationship epoch/time fields, which follow the existing ROS-004 v1 canonical encoding.

**Consent-profile payload** (the exact output of `Consent::canonical_bytes()`, 190 bytes):

```text
6d7963656c69782e636f6e73656e742d636f6e74656e742e763100
96af77b8e32e0179ce20562595a4f82c078dceb742d50449498266c46b173c7c
0300000000000000646964
0500000000000000616c696365
0300000000000000646964
0300000000000000626f62
00140000000000000072656c6174696f6e736869702d737570706f7274
0101000000000000000201000000000000000500000000000000656d61696c
640000000000000001c8000000000000000700000000000000
```

**Purpose payload** (the existing typed purpose tag plus length-prefixed UTF-8 value):

```text
00140000000000000072656c6174696f6e736869702d737570706f7274
```

**Data-class-set payload** (one class, tag `Contact = 0x02`):

```text
010000000000000002
```

Applying the domain + payload-length framing specified above must produce the three digest vectors in the previous section. These payloads and digests are intended to be copied into independently implemented known-answer tests; changes require a new commitment schema/domain version.
