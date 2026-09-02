# Mycelix Content Fabric v1 Wire Invariants

These invariants are normative for CF-00 and the CF-01..CF-07 implementation tranche.

- **CF-INV-01** Bulk content MUST NOT enter the Holochain DHT.
- **CF-INV-02** Clients MUST verify immutable content cryptographically.
- **CF-INV-03** Provider availability is not equivalent to content integrity.
- **CF-INV-04** Hard policy constraints MUST precede optimization.
- **CF-INV-05** Symthaea produces proposals, not authority.
- **CF-INV-06** Cache nodes are not automatically software trust roots.
- **CF-INV-07** Private content is encrypted before untrusted storage.
- **CF-INV-08** Commercial settlement is distinct from technical delivery.
- **CF-INV-09** Provider identity alone does not establish failure-domain independence.
- **CF-INV-10** Transport-specific chunking MUST NOT define global object identity.

## Digest contract

Wire-visible digests are algorithm-tagged. Unknown algorithms MUST fail closed unless a negotiated/versioned extension explicitly supports them. Digest byte length MUST be validated for the selected algorithm.

Initial algorithms:

- `blake3-256`
- `sha256`

Verified aliases MAY bind multiple digest algorithms to the same byte sequence, but an alias MUST only be emitted after recomputation/verification over those bytes.

## Manifest contract

`ObjectManifestV1` describes composition of immutable blobs. It MUST NOT contain mutable provider availability, marketplace terms, planner scores, lease state, or transport-specific chunk graphs.

`PublicationRecordV1` carries logical naming/versioning/provenance separately from immutable byte identity.

## Placement contract

`PlacementRequirementsV1` contains hard constraints. `PlacementPreferencesV1` contains soft optimizer preferences. A provider failing any hard constraint MUST NOT enter the ranking set.

## Envelope contract

Infrastructure service lifecycle is versioned as:

`CapabilityEnvelope<T> -> OfferingEnvelope<T> -> LeaseEnvelope<T> -> ReceiptEnvelope<T>`

Required validation properties:

- deterministic/stable IDs;
- schema-version rejection for unsupported versions;
- expiry ordering;
- offer does not exceed referenced capability;
- lease does not exceed referenced offering;
- receipt references a valid lease;
- zero/overflow resource units rejected where nonsensical;
- canonical serialization fixtures are stable.

## Compatibility rule

New fields in v1 records should be additive/optional only when old readers can safely ignore them. Semantic changes that can alter validation, identity, authorization or digest computation require a new schema version.
