# SIF v0.1 Canonical Commitments

Status: normative interoperability profile for the draft Sovereign Intelligence Fabric v0.1 accountability slice.

The purpose of this profile is to ensure Mycelix, Xenia, Symthaea, browser clients, auditors, and future implementations in other languages derive the same public accountability statement from the same semantic data.

## Hash construction

All v0.1 accountability commitments use BLAKE3-256 over:

```text
domain || 0x00 || "mycelix-accountability-canonical-v1" || 0x00 || canonical_body
```

The domain is commitment-specific:

- receipt: `mycelix:accountability-receipt:pre-attestation:v1`
- purpose: `mycelix:accountability-purpose:v1`
- policy: `mycelix:accountability-policy:v1`

The receipt commitment deliberately excludes `AccessReceipt.attestations`. Evidence providers bind this pre-attestation digest and their references are attached afterward, avoiding a receipt/proof hash cycle.

## Primitive encoding

Canonical bodies are not Serde, JSON, CBOR, protobuf, or `bincode` encodings.

- `u8`: one byte.
- `u16`, `u32`, `u64`: unsigned big-endian.
- `bool`: `0x00` false, `0x01` true.
- `Commitment32`: exactly 32 raw bytes.
- UTF-8 string: `u64` byte length followed by exact UTF-8 bytes.
- option: one-byte tag (`0` absent, `1` present), then the value when present.
- vector: `u64` element count followed by canonical element encodings.

Enum tags are fixed by protocol version and MUST NOT be inferred from a language/compiler representation.

## Set semantics

Several schema fields are represented as vectors for ergonomic serialization but are semantic sets for commitment purposes. Implementations MUST normalize them before canonical encoding:

- `AccessReceipt.rights`: ascending protocol enum tag, duplicates removed.
- `DisclosureSummary.data_classes`: lexical UTF-8 string order, duplicates removed.
- `InferenceDisclosure.provenance_receipt_ids`: lexical UTF-8 string order, duplicates removed.
- delayed-notice approvals: ascending `(organization_id, approver_id, approval_digest)` tuple.
- `AccountabilityPolicy.permitted_delay_reasons`: ascending protocol enum tag, duplicates removed.
- `AccountabilityPolicy.required_subject_rights`: ascending protocol enum tag, duplicates removed.
- `AccountabilityPolicy.required_attestation_roles`: ascending protocol enum tag, duplicates removed.

This prevents insertion order from changing a commitment to an otherwise identical policy or receipt.

## Enum tags

### AuthorityType

`Consent=0`, `Statute=1`, `CourtOrder=2`, `Warrant=3`, `Emergency=4`, `Contractual=5`, `Other=6`.

### LookupOutcome

`Allowed=0`, `PartiallyAllowed=1`, `Denied=2`.

### DisclosureKind

`None=0`, `PredicateOnly=1`, `RedactedRecord=2`, `FullRecord=3`.

### SubjectRight

`Know=0`, `Inspect=1`, `Contest=2`, `HumanReview=3`, `Appeal=4`, `ProofOfPolicy=5`.

### DelayReason

`ActiveInvestigation=0`, `PreventCrime=1`, `ProtectVictimOrWitness=2`, `PublicSafety=3`, `NationalSecurity=4`.

### AttestationRole

`ExecutionBinding=0`, `ComputationProof=1`, `PolicyProof=2`, `ExternalWitness=3`.

## Receipt field order

The pre-attestation receipt body is encoded in this exact order:

1. `protocol_version`
2. `receipt_id`
3. pairwise subject ID
4. requester: organization ID, actor ID, role, authenticated source commitment
5. purpose: code, plain-language purpose, optional matter ID, scope commitment, expiry
6. legal authority: enum tag, authority ID, jurisdiction
7. query commitment
8. policy version
9. occurrence time
10. lookup outcome tag
11. disclosure: kind, normalized data classes, item count, optional result commitment
12. notification directive and, when delayed, authorization fields plus normalized approvals
13. normalized subject rights
14. optional query-budget charge
15. optional inference disclosure with normalized provenance receipt IDs

No attestation reference is encoded in this body.

## Policy field order

The policy body is encoded in this exact order:

1. maximum delay milliseconds
2. minimum delay approvals
3. independent-organization approval requirement
4. normalized permitted delay reasons
5. normalized required subject rights
6. query-budget-charge requirement
7. normalized required attestation roles

## Interoperability requirement

An implementation claiming SIF v0.1 commitment compatibility MUST be able to consume shared golden vectors for canonical bytes and final BLAKE3 digests without calling Rust serialization code. Xenia and Symthaea adapters MUST treat the Mycelix pre-attestation receipt digest as opaque public input and MUST NOT independently reinterpret or reserialize the receipt.

A future incompatible canonical format requires a new codec/profile version and new commitment domain/version; silent changes to v0.1 encoding are forbidden.
