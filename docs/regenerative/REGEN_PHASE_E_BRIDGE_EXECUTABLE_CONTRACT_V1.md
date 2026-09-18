# REGEN Phase-E Bridge Executable Contract v1

Status: implementation-exact preregistration only

Program: Luminous-Dynamics/mycelix#940

Parent conformance contract: #1564 / `1cad5fc7ae5bb011e5408e0b16f1023f3102b00c`

Cross-repository ownership contract: #1523

Related Mycelix semantics: REGEN-040..047, especially #1431 / REGEN-042 and #1519 / REGEN-042A

Related Symthaea implementation family: Luminous-Dynamics/symthaea#3817 and REGEN-042S successors

## 1. Purpose

Freeze the first executable producer-side Phase-E bridge waist before Rust code is authored.

The bridge has one narrow job:

> convert one exact, bounded, already-selected Mycelix Phase-E assessment snapshot into one canonical, content-addressed, offline-replayable payload whose semantics can be independently reproduced by Symthaea.

It does not select authoritative state, query live Holochain state, decide policy, run a resilience simulation, authenticate an issuer, or authorize any physical action.

Core theorem:

```text
validated bounded semantic envelope
+ frozen canonical binary grammar
+ deterministic ordering
+ explicit quantity / time / epistemic semantics
+ SHA-256 domain-separated commitment
+ independent known-answer oracle
= canonical Phase-E bridge payload v1
```

not:

```text
canonical payload
= source truth
= current evidence
= valid authority
= model correctness
= real resilience
= execution authority
```

## 2. First executable crate

Proposed standalone crate:

```text
crates/mycelix-regen-phase-e-bridge/
    Cargo.toml
    src/lib.rs
    tests/known_answer.rs
    tests/rejection_matrix.rs
    vectors/v1/*.json or *.txt
```

The crate should be independently buildable from its own manifest.

It should not require a root workspace.

Initial direct dependency budget:

```text
sha2
```

No Holochain, Tokio, database, HTTP, Mycelix runtime, Symthaea runtime, LLM, RNG, clock, filesystem, signing, compression, or dynamic plugin dependency belongs in the semantic core.

The core should use:

```rust
#![forbid(unsafe_code)]
```

The first implementation should avoid serde in the normative binary path. Human-readable fixture source may be handled in tests/tooling, but serde field order must never define scientific identity.

## 3. Protocol identity

The v1 commitment domain prefix is frozen as the exact ASCII byte sequence:

```text
mycelix.regen.phase-e.bridge/v1\0
```

Normative values:

```text
protocol_id               = 1
schema_version            = 1
canonicalization_version  = 1
commitment_suite          = SHA-256
```

A future semantic change does not silently reuse these values.

## 4. Canonical commitment

The v1 bridge commitment is:

```text
SHA-256(
    b"mycelix.regen.phase-e.bridge/v1\0"
    || canonical_payload_v1
)
```

The domain prefix is part of the hashed input but is not duplicated inside `canonical_payload_v1`.

The output is exactly 32 bytes.

A hex rendering, when used in logs/receipts, is lowercase 64-character hexadecimal.

```text
matching commitment
=> matching canonical bytes under this exact v1 grammar
```

not authentication or truth.

## 5. Integer encoding

All unsigned integers use fixed-width big-endian encoding.

```text
u8   = 1 byte
u16  = 2 bytes, big-endian
u32  = 4 bytes, big-endian
u64  = 8 bytes, big-endian
```

Signed decimal mantissas use exactly 16-byte two's-complement big-endian `i128` encoding.

No platform-native width, varint implementation, host endianness, or text-number rendering belongs in v1 canonical identity.

## 6. Byte-string framing

Every variable-length byte string is encoded as:

```text
u32 byte_length
|| exact bytes
```

All v1 identifiers and human-readable protocol strings are UTF-8 and MUST also satisfy their field-specific limits.

Invalid UTF-8 is rejected before semantic construction.

No Unicode normalization is performed.

Therefore two different UTF-8 byte sequences remain different identifiers even if a display system might render them similarly.

## 7. Canonical identifier grammar

All semantic IDs use one narrow normalization-free grammar.

Limits:

```text
1..=128 UTF-8 bytes
ASCII only in v1
```

Allowed bytes:

```text
a-z
A-Z
0-9
-
_
.
:
/
```

No leading/trailing whitespace exists because whitespace is not in the grammar.

The bridge does not lowercase, uppercase, trim, Unicode-normalize, URL-normalize, or otherwise rewrite an ID.

```text
Service:A != service:a
```

unless an upstream contract explicitly maps them before bridge construction.

## 8. Exact reference

The base external identity type is conceptually:

```rust
pub struct ExactRef {
    pub namespace: CanonicalId,
    pub id: CanonicalId,
    pub revision: CanonicalId,
    pub content_sha256: [u8; 32],
}
```

All four fields are required in v1.

A mutable name without immutable revision/content identity is not a valid `ExactRef`.

Canonical field order is exactly:

```text
namespace
id
revision
content_sha256
```

where each identifier uses the byte-string framing above and the digest is 32 raw bytes with no length prefix.

## 9. Resolution state

Scientific unknowns are not represented by field omission.

V1 freezes:

```rust
pub enum Resolution<T> {
    Known(T),
    Unresolved(CanonicalId),
    Unknown,
    NotApplicable,
}
```

Canonical tags:

```text
0x00 Known
0x01 Unresolved
0x02 Unknown
0x03 NotApplicable
```

For `Known`, the encoded value immediately follows the tag.

For `Unresolved`, one canonical reason ID immediately follows the tag.

`Unknown` and `NotApplicable` carry no additional bytes.

No unknown enum tag is accepted.

## 10. Availability state

V1 availability is deliberately small:

```rust
pub enum Availability {
    Available,
    Degraded,
    Unavailable,
}
```

Canonical tags:

```text
0x00 Available
0x01 Degraded
0x02 Unavailable
```

Epistemic uncertainty is represented by `Resolution<Availability>`, not a fourth availability value.

Therefore:

```text
Unknown availability
!= known Unavailable
```

## 11. Canonical decimal

A v1 exact decimal is:

```rust
pub struct CanonicalDecimal {
    mantissa: i128,
    scale: u8,
}
```

with mathematical value:

```text
mantissa * 10^-scale
```

Constraints:

```text
scale <= 18
```

Canonical normalization:

- zero MUST be encoded as `mantissa = 0, scale = 0`;
- nonzero values with `scale > 0` MUST NOT have a mantissa divisible by 10;
- normalization may reduce scale by removing trailing decimal zeros;
- normalization MUST NOT increase scale or change mathematical value.

Canonical bytes are exactly:

```text
16-byte big-endian two's-complement mantissa
u8 scale
```

Examples:

```text
1.0    -> mantissa 1, scale 0
1.50   -> mantissa 15, scale 1
0.000  -> mantissa 0, scale 0
```

NaN, infinity, signed zero, locale formatting, binary floating-point identity, and implicit rounding do not exist in this representation.

## 12. Quantity

A v1 quantity is:

```rust
pub struct Quantity {
    value: CanonicalDecimal,
    unit_id: CanonicalId,
    basis_id: CanonicalId,
}
```

Canonical field order:

```text
value
unit_id
basis_id
```

`basis_id` is always explicit, including when the upstream domain uses a neutral basis such as `absolute`.

No implicit unit or basis conversion occurs inside the bridge.

## 13. Campaign time

The deterministic campaign time type is:

```rust
pub struct Tick(pub u64);
```

A horizon is:

```rust
pub struct Horizon {
    start: Tick,
    end_exclusive: Tick,
    time_basis_ref: ExactRef,
}
```

V1 requires:

```text
start < end_exclusive
```

The interval is always half-open:

```text
[start, end_exclusive)
```

A campaign tick is not Unix time.

No wall-clock conversion is implied by the bridge unless the `time_basis_ref` explicitly defines one upstream.

## 14. Service input

The initial service projection is conceptually:

```rust
pub struct ServiceInput {
    service_id: CanonicalId,
    service_profile_ref: ExactRef,
    delivered: Resolution<Quantity>,
    adopted_floor: Resolution<Quantity>,
    scenario_demand: Resolution<Quantity>,
    dependency_ids: Vec<CanonicalId>,
}
```

The three quantity fields remain distinct:

```text
delivered
!= adopted_floor
!= scenario_demand
```

If two or more of these are `Known`, v1 requires exact `unit_id` and `basis_id` equality before they may participate in the same service comparison.

The bridge itself does not decide whether the adopted floor is legitimate.

## 15. Dependency input

The initial dependency projection is:

```rust
pub struct DependencyInput {
    dependency_id: CanonicalId,
    dependency_kind: CanonicalId,
    authority_ref: Resolution<ExactRef>,
    availability: Resolution<Availability>,
    nominal_capacity: Resolution<Quantity>,
    usable_capacity: Resolution<Quantity>,
    committed_capacity: Resolution<Quantity>,
    failure_domain_ids: Vec<CanonicalId>,
    evidence_refs: Vec<ExactRef>,
}
```

When all three capacities are known and comparable:

```text
committed <= usable <= nominal
```

must hold exactly.

An unavailable dependency does not erase nominal design capacity; availability and capacity remain separate fields.

## 16. Stock input

The initial stock projection is:

```rust
pub struct StockInput {
    stock_id: CanonicalId,
    total: Resolution<Quantity>,
    protected: Resolution<Quantity>,
    committed: Resolution<Quantity>,
    evidence_refs: Vec<ExactRef>,
}
```

When all quantities are known and comparable:

```text
protected + committed <= total
```

must hold using checked exact decimal arithmetic.

No negative physical stock is accepted in v1 known stock fields.

## 17. Failure-domain input

The initial failure-domain projection is:

```rust
pub struct FailureDomainInput {
    failure_domain_id: CanonicalId,
    domain_kind: CanonicalId,
    availability: Resolution<Availability>,
    evidence_refs: Vec<ExactRef>,
}
```

Shared membership is represented by dependencies referencing the same exact failure-domain ID.

The bridge must not infer independence from distinct dependency IDs.

## 18. Object manifest

The envelope also carries a manifest of authoritative objects that are referenced but not fully projected into model fields.

Conceptually:

```rust
pub struct ManifestEntry {
    object_kind: CanonicalId,
    object_ref: ExactRef,
}
```

The manifest is not a bag of URLs.

Every entry binds immutable revision/content identity.

A future richer object bundle can be layered around this manifest without changing the first execution-state projection unless its semantics require a new bridge version.

## 19. Assessment envelope v1

The first complete semantic object is:

```rust
pub struct AssessmentEnvelopeV1 {
    envelope_id: CanonicalId,
    producer_repository: CanonicalId,
    producer_commit: CanonicalId,
    snapshot_revision: CanonicalId,
    campaign_ref: ExactRef,
    evidence_snapshot_ref: ExactRef,
    horizon: Horizon,
    services: Vec<ServiceInput>,
    dependencies: Vec<DependencyInput>,
    stocks: Vec<StockInput>,
    failure_domains: Vec<FailureDomainInput>,
    manifest: Vec<ManifestEntry>,
}
```

Protocol/schema/canonicalization version are carried in the canonical payload header rather than repeated as mutable semantic fields.

`producer_commit` is an opaque exact identifier. V1 does not assume every producer uses Git SHA-1 forever, but the adopted profile must define the namespace if another identity is used.

## 20. Canonical payload header

`canonical_payload_v1` starts with exactly:

```text
u16 protocol_id               = 1
u16 schema_version            = 1
u16 canonicalization_version  = 1
```

followed by the envelope fields in the exact order frozen below.

## 21. Canonical top-level field order

After the 6-byte header:

```text
envelope_id
producer_repository
producer_commit
snapshot_revision
campaign_ref
evidence_snapshot_ref
horizon
services
dependencies
stocks
failure_domains
manifest
```

No field tags are included in v1.

This means field order is protocol, not implementation convenience.

## 22. Collection encoding

Every vector/set-like collection is encoded as:

```text
u32 element_count
|| element_1
|| ...
|| element_n
```

V1 bounds element counts before allocation.

Semantic collections are canonicalized by bytewise ascending canonical ID/reference key as specified below.

A decoder of canonical bytes MUST reject out-of-order or duplicate semantic identities rather than silently sorting malformed input.

An authoring constructor MAY accept unordered semantic inputs, but it must:

1. detect duplicate identities before map/set collapse;
2. validate the full semantic object;
3. sort deterministically;
4. only then emit a `ValidatedEnvelope` capable of canonical encoding.

## 23. Canonical collection order

V1 sorting keys:

```text
services         -> service_id bytes
dependencies     -> dependency_id bytes
stocks           -> stock_id bytes
failure_domains  -> failure_domain_id bytes
manifest         -> object_ref(namespace,id,revision,digest) canonical bytes
```

Nested ID lists such as service dependency IDs and dependency failure-domain IDs are sorted by exact ID bytes.

Evidence refs are sorted by their full canonical `ExactRef` bytes.

Canonical sort order creates no priority, authority, dispatch order, or preference.

## 24. Referential closure

Validation occurs before commitment.

At minimum:

- every service dependency ID MUST identify one dependency in the envelope;
- every dependency failure-domain ID MUST identify one failure domain;
- every duplicate service/dependency/stock/failure-domain ID is rejected;
- every `ExactRef` must satisfy the exact-ref grammar;
- the campaign and evidence-snapshot refs must be immutable exact refs;
- an unresolved authority ref remains unresolved and cannot become a known authority merely because the dependency is otherwise feasible.

The first implementation does not perform a live network lookup to close references.

## 25. Structural limits profile v1

The first executable profile freezes conservative hard maxima:

```text
MAX_CANONICAL_PAYLOAD_BYTES      = 4_194_304
MAX_ID_BYTES                     = 128
MAX_SERVICES                     = 1_024
MAX_DEPENDENCIES                 = 16_384
MAX_STOCKS                       = 8_192
MAX_FAILURE_DOMAINS              = 4_096
MAX_MANIFEST_ENTRIES             = 32_768
MAX_DEPENDENCIES_PER_SERVICE     = 2_048
MAX_FAILURE_DOMAINS_PER_DEP      = 1_024
MAX_EVIDENCE_REFS_PER_OBJECT     = 2_048
```

The limits are part of bridge v1 qualification.

A future change to these normative maxima creates a new limits/profile revision and must be reflected in receipts even if the binary grammar remains otherwise compatible.

Over-limit input is invalid bridge input.

No silent truncation, graph pruning, first-N behavior, or lossy summarization is permitted.

## 26. Validation order

The first implementation should use a deterministic fail-fast order so error fixtures are stable:

```text
1. protocol/schema/canonicalization support
2. top-level structural limits
3. identifier/exact-ref syntax
4. duplicate identities
5. collection/nested limits
6. horizon validity
7. referential closure
8. quantity canonicality and sign rules
9. unit/basis comparability
10. capacity ordering
11. stock partition arithmetic
12. canonical payload-size bound
13. commitment
```

The library may report more than one error in a future diagnostic API, but the normative v1 constructor should have one stable first-error result.

## 27. Error taxonomy

The first public error enum should remain structured and stable enough for the cross-repo rejection corpus.

Conceptually:

```rust
pub enum BridgeError {
    UnsupportedProtocol,
    UnsupportedSchema,
    UnsupportedCanonicalization,
    LimitExceeded,
    InvalidIdentifier,
    InvalidExactRef,
    DuplicateService,
    DuplicateDependency,
    DuplicateStock,
    DuplicateFailureDomain,
    DuplicateManifestEntry,
    DanglingDependencyReference,
    DanglingFailureDomainReference,
    InvalidHorizon,
    InvalidDecimal,
    NegativePhysicalQuantity,
    QuantityUnitMismatch,
    QuantityBasisMismatch,
    QuantityOverflow,
    CapacityOrderingViolation,
    StockPartitionViolation,
    NonCanonicalOrder,
    MalformedEncoding,
    TrailingBytes,
    CommitmentMismatch,
}
```

Exact variant naming may be refined once code is authored, but semantic categories should not be collapsed into free-form strings.

## 28. Decoder theorem

A canonical decoder consumes exact bytes plus an expected or embedded v1 header and returns only a validated envelope.

Conceptually:

```rust
pub fn decode_canonical_v1(bytes: &[u8]) -> Result<ValidatedEnvelopeV1, BridgeError>;
```

It must reject:

- truncation;
- invalid lengths;
- over-budget lengths before allocation;
- invalid UTF-8/ID bytes;
- unknown enum tags;
- non-normalized decimal values;
- noncanonical collection order;
- duplicates;
- dangling refs;
- semantic invariant failures;
- trailing bytes.

Parsing success alone must not produce an unvalidated semantic object.

## 29. Encoder theorem

Only a validated envelope may produce normative canonical bytes.

Conceptually:

```rust
impl ValidatedEnvelopeV1 {
    pub fn canonical_bytes(&self) -> Vec<u8>;
    pub fn commitment(&self) -> [u8; 32];
}
```

A raw/deserialized struct must not expose `commitment()` before validation.

## 30. Decoder / encoder identity

For every accepted canonical payload:

```text
decode(bytes).canonical_bytes() == bytes
```

must hold exactly.

This is stronger than semantic equality after permissive parsing.

A decoder that accepts noncanonical bytes and then re-encodes different canonical bytes fails the v1 conformance theorem.

## 31. Independent Python oracle

The first qualification campaign must include a small Python standard-library oracle that independently implements the frozen byte grammar and SHA-256 commitment using `hashlib`.

The oracle MUST NOT call the Rust library or parse Rust debug output.

For every positive known-answer vector:

```text
Rust canonical bytes
== Python canonical bytes

Rust SHA-256 commitment
== Python SHA-256 commitment
```

The Python oracle is evidence support, not a production runtime dependency.

## 32. Positive known-answer corpus

The first campaign freezes at least six synthetic fixtures:

### P1 — direct dependency

One service with one known available direct dependency and exact known delivered/floor/demand quantities.

### P2 — explicit unresolved dependency

One service whose dependency availability is `Unresolved(reason)`.

The unresolved state must survive bytes and decode unchanged.

### P3 — shared failure domain

Primary and fallback dependencies reference the same failure-domain ID.

The decoded graph must preserve the overlap.

### P4 — finite stock partition

One stock has known total/protected/committed quantities satisfying the exact partition invariant.

### P5 — floor distinct from demand

Known adopted floor and larger known scenario demand use the same unit/basis but remain separate committed fields.

Changing either field changes the commitment.

### P6 — half-open campaign horizon

A deterministic REGEN-042A campaign ref with an explicit `[start,end)` horizon.

Changing `end_exclusive` changes the commitment.

Every positive fixture freezes:

- semantic source fixture;
- canonical byte length;
- canonical bytes or byte-file digest;
- expected bridge SHA-256 commitment;
- expected decoded field census.

## 33. Negative/rejection corpus

The first Rust and Python/consumer-compatible rejection campaign should include at least:

```text
N01 unsupported protocol
N02 unsupported schema
N03 unsupported canonicalization
N04 invalid identifier byte
N05 overlong identifier
N06 duplicate service
N07 duplicate dependency
N08 duplicate stock
N09 duplicate failure domain
N10 dangling service->dependency
N11 dangling dependency->failure-domain
N12 reversed/empty horizon
N13 decimal scale > 18
N14 noncanonical decimal trailing zero
N15 negative known stock quantity
N16 known service quantity unit mismatch
N17 known service quantity basis mismatch
N18 committed capacity > usable
N19 usable capacity > nominal
N20 protected + committed stock > total
N21 over-limit service count
N22 over-limit nested dependency refs
N23 noncanonical service ordering in encoded bytes
N24 unknown availability tag
N25 unknown resolution tag
N26 truncated byte string
N27 declared length exceeds remaining bytes
N28 trailing bytes
N29 valid payload with wrong expected commitment
N30 floor/demand field aliasing mutation
```

A later shared cross-repo corpus may add more cases without weakening these first 30.

## 34. Metamorphic properties

The first implementation should test at least:

```text
unordered authoring insertion order
-> identical canonical bytes

canonical encode -> decode -> encode
-> byte identity

semantic field mutation
-> commitment mutation

nonsemantic source-fixture whitespace
-> same canonical bytes after fixture parsing
```

Semantic mutations include:

- service ID;
- dependency ID;
- failure-domain membership;
- floor;
- demand;
- delivered quantity;
- unit/basis;
- resolution state;
- campaign ref;
- evidence snapshot ref;
- horizon;
- producer commit;
- snapshot revision.

## 35. No equality-by-hash shortcut during validation

Semantic validation must not accept two objects as equivalent merely because the caller supplies equal digest strings.

The implementation computes commitments from validated canonical bytes itself.

Caller-supplied commitment fields are only compared after local recomputation.

## 36. Import boundary to Symthaea

The Mycelix bridge crate owns only the producer-side canonical protocol implementation.

Symthaea should implement an independent consumer/parser or independently validated compatibility layer.

Dependency direction:

```text
Mycelix authoritative/adopted snapshot
-> canonical Phase-E bridge bytes
-> Symthaea import validator
-> Symthaea immutable model state
```

not:

```text
Symthaea depends on Mycelix internal Rust structs
```

The consumer should echo both:

```text
bridge_input_commitment
resulting_model_state_commitment
```

in an import receipt.

## 37. Shared IDs remain opaque

The bridge transports exact service/dependency/stock/failure-domain/profile identities.

Symthaea may index these internally but must retain a lossless mapping back to the exact bridge IDs.

The bridge does not create a second namespace merely for model convenience.

## 38. Evidence class boundary

A bridge payload is a transport/evidence package, not an observation class upgrade.

If an upstream field is modeled, inferred, scenario, forecast, unresolved, or otherwise non-observed, the adapter must preserve that semantic state through the exact referenced evidence/profile objects.

The first narrow execution projection does not attempt to duplicate the entire PEF evidence model inside the bridge.

## 39. Freshness boundary

The bridge commitment binds which evidence snapshot was used.

It does not make that evidence fresh.

```text
new envelope commitment
!= current evidence
```

Any freshness/currentness evaluation remains an explicit upstream profile result or separately bound field in a future bridge revision.

## 40. Authority boundary

`authority_ref: Known(...)` means only that the envelope contains an exact upstream authority/profile reference selected for the assessment.

It does not mean the bridge itself verified every legal/social condition that reference represents.

`Unresolved` remains unresolved.

Symthaea feasibility cannot upgrade it.

## 41. Authentication boundary

V1 commitment is unkeyed SHA-256 content identity.

It is not an issuer signature.

Future detached authentication may bind:

```text
bridge commitment
+ signer/key identity
+ algorithm suite
+ credential/lifecycle evidence
```

through REGEN-Q002/Xenia-compatible semantics.

The core bridge remains independently replayable without private keys or network access.

## 42. Compression boundary

Compression is outside canonical identity in v1.

If a transport compresses payload bytes:

```text
canonical payload
-> optional transport compression
```

The receiver must recover the exact canonical bytes before checking the bridge commitment.

Compressed representation bytes are not the v1 scientific identity.

## 43. Storage boundary

The bridge crate defines bytes and validation only.

It does not choose database schema, Holochain entry type, file layout, retention policy, or replication policy.

Those adapters store the canonical payload/commitment without changing the semantic theorem.

## 44. Privacy boundary

The first synthetic/executable bridge should not include personal identifying data.

The schema prefers opaque exact refs where Symthaea does not need raw source content.

A later domain-specific bridge extension must justify any sensitive field as analysis-required rather than technically convenient.

## 45. Qualification sequence

Recommended exact sequence:

```text
implementation-exact contract (this subject)
-> authored standalone Rust producer crate
-> pinned Rust 1.96 preparation
-> generated Cargo.lock retained in capsule
-> known-answer Rust/Python agreement
-> strict Clippy
-> fuzz/property corpus where practical
-> exact-byte ProductFrozen promotion
-> exact-head ProductFrozen qualification
-> Q001 receipt
-> independent Symthaea consumer implementation
-> independent Symthaea qualification
-> exact-pair cross-repo conformance campaign
```

A producer PASS does not manufacture a consumer PASS.

## 46. First Rust regression propositions

At least these propositions should be named directly in tests:

1. canonical identifiers preserve exact bytes without normalization;
2. invalid identifiers reject;
3. zero decimal has one canonical encoding;
4. nonzero trailing-zero decimal rejects as noncanonical on decode;
5. exact quantity unit mismatch rejects where comparability is required;
6. exact quantity basis mismatch rejects where comparability is required;
7. duplicate IDs reject before ordered-map collapse;
8. dangling dependency refs reject;
9. dangling failure-domain refs reject;
10. empty horizon rejects;
11. service floor and scenario demand remain independent fields;
12. unavailable and unknown remain distinct;
13. unresolved reason survives roundtrip;
14. capacity ordering is exact and checked;
15. stock partition is exact and checked;
16. shared failure-domain identity survives roundtrip;
17. unordered authoring input canonicalizes identically;
18. canonical bytes roundtrip byte-identically;
19. noncanonical ordering is rejected by decoder;
20. trailing bytes reject;
21. over-budget collection rejects before materialization where practical;
22. commitment changes when campaign revision changes;
23. commitment changes when evidence snapshot changes;
24. commitment changes when floor changes;
25. commitment changes when scenario demand changes;
26. commitment changes when failure-domain membership changes;
27. Rust/Python bytes match for every positive vector;
28. Rust/Python SHA-256 commitments match for every positive vector;
29. wrong expected commitment rejects;
30. no API emits a bridge commitment for an invalid raw envelope.

## 47. Fuzz targets

After deterministic tests are green, useful bounded fuzz targets include:

- decoder never panics;
- arbitrary bytes never allocate beyond configured limits;
- accepted bytes always re-encode identically;
- accepted envelopes always satisfy structural invariants;
- one-bit mutation either rejects or produces a different commitment unless it changes no bytes;
- malformed length prefixes fail without integer overflow;
- unknown enum tags fail closed.

Fuzz success does not prove complete protocol correctness.

## 48. Mutation targets

Mutation testing should especially attack:

- endianness;
- field order;
- collection sort direction;
- duplicate checks;
- decimal normalization;
- resolution/availability tags;
- horizon inequality;
- capacity ordering;
- stock partition arithmetic;
- domain prefix;
- digest comparison;
- limits checks;
- trailing-byte rejection.

A surviving mutation in one of these protocol-defining paths blocks a strong qualification claim until understood.

## 49. ProductFrozen boundary

A later ProductFrozen bridge subject should contain only the exact prepared semantic crate/test bytes, checked-in Cargo.lock, qualification workflow, and exact qualified receipt validator required by its preregistered release profile.

Preparation/promoter history should not enter product ancestry.

Any product-byte change after preparation requires a new preparation lineage.

## 50. Deliberate non-claims

This contract establishes no executable PASS today.

It does not establish:

- real service sufficiency;
- current or truthful source evidence;
- valid rights/ecology/quality determinations;
- hazard probability;
- disaster readiness;
- model validity beyond future tested import semantics;
- authentication or authorized issuer identity;
- emergency policy;
- procurement authority;
- infrastructure safety;
- physical actuation authority.

Its proposition is narrow:

> the first Mycelix→Symthaea Phase-E bridge can be implemented as a small, bounded, exact binary protocol whose semantic object, canonical bytes, content commitment, rejection behavior, and independent oracle are frozen before executable evidence is gathered.
