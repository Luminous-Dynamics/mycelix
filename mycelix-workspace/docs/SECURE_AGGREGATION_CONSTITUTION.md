# MYC-SA-000R — Secure Aggregation Constitution

## Status

Documentation-only semantic/security contract before a canonical secure-aggregation
runtime or cryptographic backend is implemented.

The current TypeScript `SecureAggregator` and FHE client are useful prototypes and
migration evidence. They are not the production theorem.

## Governing theorem

```text
encrypted values
+ an aggregation result
!=
qualified secure aggregation
```

A secure-aggregation claim requires an exact protocol profile, participant/roster
semantics, contribution validity rules, transcript binding, key/decryption model,
cryptographic backend identity, failure/dropout semantics and output receipt.

Likewise:

```text
secure aggregation != differential privacy
secure aggregation != Byzantine robustness
secure aggregation != participant trust
secure aggregation != consent
secure aggregation != representative sampling
secure aggregation != authority to act on the result
```

## Current prototype findings

### FHE provider is explicitly simulated

The current TypeScript FHE client states that its default provider is a simplified
demonstration simulation.

The simulated ciphertext contains the JSON-encoded plaintext after a random prefix,
and simulated homomorphic operations internally decode values, compute in plaintext
and re-encode them.

Therefore no result produced through that provider may be represented as
cryptographically private FHE evidence.

Required rule:

```text
SimulatedFHE
    cannot emit
QualifiedCryptographicAggregation
```

### One `threshold` currently conflates distinct concepts

The prototype config has one `threshold` described as a minimum participant count
for reconstruction.

Canonical semantics must separate at least:

```text
collection threshold
    !=
decryption/reconstruction threshold
    !=
roster size
    !=
minimum statistical sample size
```

These may happen to be numerically equal in a profile, but they are not one concept.

### Threshold reached currently causes early automatic aggregation

`submitValue()` triggers aggregation as soon as the collection threshold is met.

This makes arrival order consequential: early contributors may determine the
aggregate while later valid participants are excluded.

Canonical v1 should treat:

```text
threshold reached
    -> eligible to seal
```

not automatically:

```text
threshold reached
    -> round sealed/completed
```

A profile must define the exact sealing policy: explicit seal, deadline, fixed
roster completion, or another qualified deterministic rule.

### Pre-encrypted and plaintext submission paths differ

Current `submitValue()` auto-aggregates at threshold while `submitEncrypted()` does
not.

Equivalent semantic contribution paths must not produce different lifecycle
transitions merely because encryption happened inside or outside the coordinator.

### Duplicate participant submissions silently replace prior values

The prototype stores contributions in a map keyed by participant ID. A later
submission from the same ID replaces the earlier value before sealing.

Canonical semantics must choose an explicit policy:

```text
RejectDuplicate
ExplicitSupersessionBeforeSeal
VersionedContribution
```

Silent replacement is not acceptable for an evidence-bearing protocol.

### No roster/authentication binding is currently established

The prototype does not bind each ciphertext to:

- an exact roster;
- participant authentication/signature;
- round identity inside an authenticated contribution envelope;
- key epoch;
- sequence/nonce/replay state;
- input shape/domain;
- contribution commitment/proof.

A map key supplied by a caller is not participant authentication.

### `completed` currently means multiple different things

The round status becomes `completed` after producing an encrypted sum/count, before
`finalizeResult()` decrypts it.

Canonical state must distinguish:

```text
encrypted aggregate constructed
    !=
decryption completed
    !=
result released
    !=
result accepted by a downstream domain
```

### Current finalization uses a central secret key

`finalizeResult()` calls the same FHE client used by the aggregator, which holds a
secret key generated at initialization.

This is a central-key FHE architecture, not threshold decryption.

A Shamir helper existing in the same source file does not make FHE decryption
threshold-distributed.

### Shamir commitment/verification is intentionally placeholder

The current secret-sharing helper creates random bytes as a `commitment`, unrelated
to the share contents, and `verifyShare()` only checks that share and commitment
byte arrays are nonempty.

Therefore:

```text
verifyShare() == true
    !=
cryptographic share verification
```

This helper must not be promoted into a production VSS/DKG/threshold-decryption
claim.

### Final verification hash is too narrow for protocol evidence

The prototype hashes only the decrypted sum and count JSON.

That does not bind the exact:

- round plan;
- roster;
- contribution set;
- rejected contributions;
- cryptographic profile;
- key epoch;
- aggregate ciphertext;
- decryption evidence;
- sealing policy;
- transcript ordering/canonicalization.

Canonical secure aggregation needs a transcript/receipt commitment, not merely an
output hash.

### Failure states are too coarse

The prototype collapses timeout, cancellation, aggregation errors and other
conditions into `failed`.

The authority/evidence path must preserve why and when the protocol stopped.

## Protocol planes

Canonical secure aggregation should separate these planes.

### 1. Plan / policy plane

Defines what round is allowed to occur.

### 2. Participant / roster plane

Defines who may contribute under what participant identity/authentication profile.

### 3. Contribution plane

Defines one exact contribution envelope and validity requirements.

### 4. Cryptographic compute plane

Defines how admitted encrypted/masked contributions combine.

### 5. Decryption/reconstruction plane

Defines who/what can reveal or reconstruct output and under what threshold/profile.

### 6. Transcript/evidence plane

Records enough exact evidence to reconstruct protocol disposition without exposing
protected plaintext inputs.

### 7. Release/consumption plane

Controls whether/how a result may leave the aggregation protocol and be consumed by
a domain. This plane is not implied by successful decryption.

## Profile families

Do not pretend one mechanism provides every security model.

Initial profile registry should allow at least:

```text
CentralKeyFheV1
MaskedSecureAggregationV1
ThresholdFheV1
```

The first implementation need not support all three.

### CentralKeyFheV1

Potential first real FHE profile:

- participants encrypt to one qualified aggregation public key;
- ciphertexts are aggregated without plaintext exposure to the aggregation path;
- an explicitly identified key holder/decryption service can decrypt;
- the profile does **not** claim threshold-distributed decryption authority.

This is useful while keeping its trust assumption explicit.

### MaskedSecureAggregationV1

A future secure-sum profile can use pairwise/additive masking or another qualified
secure-aggregation construction where appropriate.

This may be a better fit than FHE for some high-volume FL/federated-statistics sums,
but it requires its own dropout/recovery/threat theorem.

### ThresholdFheV1

A later threshold-FHE profile requires an actual distributed key-generation and
threshold-decryption construction qualified for the chosen FHE scheme.

Do not infer this from generic Shamir secret sharing, Feldman VSS, or the presence
of multiple key shares.

## SecureAggregationPlanV1

The round plan should bind, at minimum, equivalents of:

```rust
SecureAggregationPlanV1 {
    protocol_id,
    protocol_profile,
    round_id,
    roster_commitment,
    roster_size,
    collection_threshold,
    sealing_policy,
    contribution_policy,
    aggregation_function,
    value_domain,
    shape_profile,
    crypto_profile,
    key_epoch,
    decryption_profile,
    decryption_threshold,
    deadline_evidence,
    output_release_profile,
}
```

Fields may be split into typed nested objects. The important property is that the
semantic plan is exact and immutable once collection begins.

## Threshold separation

Required invariant:

```text
collection_threshold
    controls whether enough admissible contributions exist to permit sealing

decryption_threshold
    controls cryptographic reconstruction/decryption under the selected backend

roster_size
    defines the admitted participant universe for the round
```

No generic `threshold` field should silently mean all three.

## Roster semantics

A roster entry should bind enough information to identify the admitted participant
under the selected authentication profile without forcing unnecessary public
identity disclosure.

Conceptually:

```rust
RosterEntryV1 {
    participant_ref,
    authentication_profile,
    contribution_limit,
    eligibility_evidence_ref,
}
```

Secure aggregation does not decide whether a participant *should* be eligible;
it consumes a roster established by the owning domain/policy.

## ContributionEnvelopeV1

Each accepted contribution should bind at least:

```rust
ContributionEnvelopeV1 {
    protocol_id,
    round_id,
    roster_commitment,
    participant_ref,
    contribution_sequence,
    nonce,
    crypto_profile,
    key_epoch,
    ciphertext_or_masked_value_commitment,
    shape_profile,
    value_domain_profile,
    validity_proof_ref,
    authentication_evidence,
}
```

The exact cryptographic encoding is backend-specific; the semantic envelope is not.

## Replay and duplicate rules

At minimum reject/quarantine:

- wrong round;
- wrong protocol/profile;
- wrong roster;
- wrong key epoch;
- participant not on roster;
- nonce replay;
- sequence rollback;
- duplicate terminal contribution under `RejectDuplicate`;
- conflicting same-sequence/different-commitment contribution;
- contribution after seal.

An explicit supersession profile may allow replacement before seal, but it must
preserve predecessor identity and cannot silently overwrite history.

## Contribution validity

Cryptographic privacy alone does not prove the encrypted value is admissible.

Required distinction:

```text
valid ciphertext
    !=
valid plaintext domain
```

For example, an adversary can submit an encryption of an extreme value that
poisons a sum while preserving perfect ciphertext validity.

Depending on the profile, contribution validity may require evidence for:

- plaintext knowledge;
- vector/scalar shape;
- integer/fixed-point encoding range;
- clipping/range bound;
- categorical membership;
- schema/domain correctness;
- model/version binding for FL contributions.

This can later compose ZK/range proofs where justified. MYC-SA-000R does not select
a universal proof system.

## State machine

Avoid one `collecting/aggregating/completed/failed` lifecycle.

Reference lifecycle:

```text
Planned
   |
   v
Collecting
   |
   +--> ThresholdReached        (still collecting unless policy seals)
   |
   v
CollectionSealed
   |
   v
Aggregating
   |
   v
EncryptedAggregateBound
   |
   +--> ReleasedWithoutDecryption  (profile-specific)
   |
   v
DecryptionPending
   |
   v
DecryptedResultBound
   |
   v
Finalized
```

Typed terminal/non-success states should include equivalents of:

```text
CancelledBeforeSeal
TimedOutBeforeThreshold
SealedBelowRequiredThreshold   // normally invalid/fail closed
AggregationRejected
AggregationFailed
DecryptionUnavailable
DecryptionRejected
DecryptionFailed
IndeterminateAggregation
IndeterminateDecryption
```

Exact names may differ; the distinctions may not be collapsed into one `failed`.

## Seal policy

A round is not sealed merely because a threshold was observed unless its exact
profile says so.

Reference policies may include:

```text
ExplicitSeal
DeadlineSeal
FullRosterSeal
ThresholdAutoSeal   // allowed only as an explicit profile with known fairness semantics
```

If `ThresholdAutoSeal` exists, its dependence on arrival order must be part of the
profile/nonclaims.

## Transcript as a security/evidence object

Define a canonical transcript identity over semantic protocol events.

Conceptually:

```rust
SecureAggregationTranscriptV1 {
    plan_commitment,
    roster_commitment,
    accepted_contributions,
    rejected_contributions,
    seal_event,
    aggregate_commitment,
    decryption_events,
    release_event,
    terminal_disposition,
}
```

Where event order is semantically irrelevant, canonicalize as a set keyed by stable
identity rather than arrival order. Where order is relevant, bind it explicitly.

The transcript should not contain plaintext participant inputs unless the selected
profile deliberately requires them (which would usually defeat the purpose).

## Accepted/rejected contribution evidence

A rejected contribution should preserve a bounded reason code and commitment where
safe, for example:

```text
ParticipantNotInRoster
WrongRound
WrongKeyEpoch
Replay
Duplicate
ShapeMismatch
ValueProofFailed
AuthenticationFailed
AfterSeal
UnsupportedProfile
MalformedCiphertext
```

Do not echo sensitive plaintext or secrets into diagnostic receipts.

## Backend interface

Keep the protocol theorem independent from concrete crypto libraries.

Conceptually:

```rust
trait SecureAggregationBackend {
    type Contribution;
    type Aggregate;
    type DecryptionEvidence;

    fn validate_contribution(...);
    fn aggregate(...);
    fn bind_aggregate(...);
}
```

Decryption may live behind a separate trait because not every secure-aggregation
profile uses the same reconstruction model.

```rust
trait AggregateDecryptionBackend {
    fn request_decryption(...);
    fn verify_decryption_evidence(...);
}
```

No backend method returning success may by itself create protocol authority.

## Key epochs

Every cryptographic round must bind an exact key epoch/profile.

Required distinction:

```text
same public key bytes
    !=
same key-authority epoch unless profile says so
```

A key rotation cannot reinterpret old contributions under the new key epoch.

## DKG / threshold-key boundary

Generic DKG/VSS capability elsewhere in Mycelix is valuable prior art but is not
automatically a DKG for the selected threshold-FHE scheme.

Required theorem:

```text
Feldman/Shamir/VSS implementation exists
    !=
threshold-FHE key generation qualified
```

A future ThresholdFheV1 must bind the exact scheme-specific distributed key
creation, evaluation-key generation, decryption-share proof/verification and
reconstruction semantics.

## Output semantics

A secure aggregate should expose an exact mathematical output type, for example:

```text
Sum
Count
VectorSum
Histogram
DomainDefinedFunction(profile)
```

Do not assume arbitrary computation merely because the cryptographic backend can
technically perform more operations.

The aggregation function and encoding domain must be part of the plan.

## Fixed-point / encoding semantics

Many cryptographic schemes operate on integers/rings while application domains use
real-valued quantities.

A production profile must bind:

```text
encoding scale
signedness
modulus/wrap behavior
range bound
rounding rule
overflow policy
vector packing/slot semantics
```

`encrypted average` is not defined merely by multiplying an encrypted integer sum
by a host-language floating-point reciprocal.

## SecureAggregationReceiptV1

The final receipt should bind at least equivalents of:

```rust
SecureAggregationReceiptV1 {
    protocol_id,
    round_id,
    plan_commitment,
    roster_commitment,
    transcript_commitment,
    crypto_profile,
    key_epoch,
    collection_count,
    accepted_contribution_set_commitment,
    rejected_contribution_set_commitment,
    aggregate_commitment,
    decryption_profile,
    decryption_evidence_commitment,
    output_commitment,
    terminal_disposition,
    implementation_qualification_ref,
}
```

Do not use one `verificationHash` over output values as the protocol receipt.

## Decryption evidence

Different profiles produce different evidence.

### Central key

Bind at least:

- exact decryption service/key epoch;
- aggregate commitment;
- output commitment;
- authenticated result evidence/profile.

This still has a central key-holder trust assumption.

### Threshold decryption

Bind at least:

- decryption-share identities/commitments;
- threshold/profile;
- share proof/verification results where the scheme supplies them;
- exact aggregate commitment;
- reconstruction/output commitment.

Do not expose raw secret shares in ordinary receipts.

## Release policy

Successful decryption does not automatically authorize release.

A release profile may bind:

```text
recipient/audience
purpose
minimum cohort policy
DP release requirement
retention policy
result sensitivity classification
```

These semantics usually belong to domain/privacy policy and are composed rather
than invented by the secure-aggregation core.

## Relationship to differential privacy

A secure sum can reveal sensitive information from the aggregate, especially with
small cohorts or repeated queries.

Therefore:

```text
inputs hidden during computation
    !=
output safe to disclose
```

A later federated analytics pipeline may require:

```text
secure aggregation receipt
+
qualified DP release receipt
+
consent/purpose/authority evidence
```

Each remains independently verifiable.

## Relationship to FL

Secure aggregation hides individual updates from the aggregation path under its
threat model. It does not make those updates honest or robust.

A federated-learning round may compose:

```text
roster / participation evidence
+
secure aggregation
+
robust aggregation semantics (where compatible)
+
DP release
+
model qualification
```

But some robust aggregators require visibility of individual updates/distances and
are not directly compatible with simple secure-sum protocols. This is a real
architecture constraint, not an implementation nuisance.

Profiles must state what computation is possible under the chosen privacy model.

## Relationship to scientific inference

Secure aggregation only addresses a computational privacy/trust boundary.

It does not establish:

- representative sampling;
- observation independence;
- correct measurement;
- causal identification;
- calibration;
- statistical significance;
- scientific validity.

A scientific receipt must compose these separately.

## Browser / Leptos relationship

The canonical secure-aggregation protocol remains UI independent.

A browser adapter may later execute expensive cryptography through
`mycelix-browser-worker` and expose state to Leptos.

Required distinction:

```text
browser worker operational
    !=
secure aggregation qualified
```

The UI should present exact protocol truth such as:

```text
Collection: 47/50 accepted
Seal: explicit / sealed
Crypto profile: CentralKeyFheV1
Decryption model: central key holder
Input validity: range proof unavailable / not claimed
DP release: not yet applied
Transcript: bound
```

rather than one generic green `Secure` badge.

## Canonical language rule

Protocol state machine, transcript/receipt semantics, cryptographic profile
validation and authority-bearing contribution validation should live in canonical
Rust.

TypeScript remains useful as:

- legacy/prototype reference;
- browser/SDK compatibility adapter;
- cross-language fixture implementation where independently useful.

It should not remain the sole owner of production secure-aggregation semantics.

## Migration disposition for current TypeScript prototype

Classify current `sdk-ts/src/fhe/secure-aggregation.ts` as:

```text
Prototype / historical oracle
```

Preserve tests and useful behavior during migration, but explicitly prevent it
from producing qualified secure-aggregation receipts.

Classify the default simulated FHE provider as:

```text
DevelopmentSimulation
```

and fail closed if an authoritative profile attempts to use it.

## MYC-SA-001A — protocol core child

The first executable child should implement **no real FHE yet**.

Build the crypto-independent protocol theorem first:

```text
SecureAggregationPlanV1
RosterV1
ContributionEnvelopeV1
RoundStateMachine
TranscriptV1
ReceiptV1
Backend traits
reason/error codes
```

Use a deterministic toy/mock backend solely to qualify protocol transitions, marked
as non-cryptographic.

This prevents cryptographic library details from defining protocol semantics.

## MYC-SA-001AQ — protocol qualification

Minimum corpus:

1. invalid plan/profile;
2. collection threshold > roster size;
3. invalid decryption threshold for selected profile;
4. participant not on roster;
5. wrong round;
6. wrong roster commitment;
7. wrong crypto profile;
8. wrong key epoch;
9. replayed nonce;
10. sequence rollback;
11. duplicate contribution rejected under RejectDuplicate;
12. explicit supersession preserves predecessor under supersession profile;
13. contribution after seal rejected;
14. shape mismatch;
15. value-domain/proof failure;
16. malformed ciphertext/envelope;
17. threshold reached does not auto-seal under ExplicitSeal;
18. deterministic seal membership independent of transport ordering where profile requires it;
19. timeout below threshold;
20. cancellation before seal;
21. aggregation failure after seal;
22. decryption unavailable;
23. indeterminate decryption retained distinctly;
24. stale key epoch rejected;
25. aggregate substitution rejected;
26. transcript mutation changes/rejects transcript identity;
27. roster mutation changes/rejects plan identity;
28. simulated backend cannot emit qualified cryptographic disposition;
29. output receipt does not imply DP/consent/authority;
30. restart/recovery does not silently reopen a sealed/finalized round;
31. exact-head immutable qualification.

## MYC-SA-002A — first real cryptographic backend

After the protocol core qualifies, add exactly one real backend profile.

A reasonable first target is a narrowly scoped `CentralKeyFheV1` or a separately
well-specified masked secure-sum profile, chosen after implementation/operational
tradeoff review.

Do not combine central FHE, masked aggregation and threshold FHE in one first PR.

## MYC-SA-003+ — later children

Potential later tranches:

```text
MYC-SA-003  contribution validity/range-proof profile
MYC-SA-004  masked secure aggregation + dropout recovery
MYC-SA-005  actual threshold-FHE DKG/decryption profile
MYC-SA-WASM browser-worker cryptographic adapter
MYC-SA-HC   Holochain orchestration/evidence adapter
MYC-SA-LEP  Leptos protocol-status UX
MYC-SA-FED  federated analytics composition
```

Names are provisional; keep each theorem independently qualifiable.

## Operational recovery

Protocol state that affects accepted contribution membership, sealing, aggregate
identity or decryption disposition must have explicit recovery semantics.

A process crash cannot permit:

- accepting a second conflicting contribution after a previously sealed round;
- changing the roster;
- changing key epoch;
- silently repeating decryption under a new profile;
- fabricating `DefinitelyNotAggregated` from missing process memory.

Persisted/reconstructed evidence requirements can be a later implementation child,
but the state machine must define the expected outcomes now.

## Time semantics

Wall-clock time is evidence/profile input, not semantic truth by itself.

Distinguish:

```text
configured deadline
local timer fired
trusted deadline evidence
round actually sealed
```

A JavaScript `setTimeout` firing is not independently verifiable protocol time.

The first pure core should accept explicit time/deadline evidence/context rather
than reading ambient time inside the semantic transition function.

## Cryptographic agility

Every crypto backend/profile must carry exact algorithm/scheme/parameter identity.

Never infer security level from:

- key/ciphertext byte length;
- scheme name alone;
- provider brand;
- `supportsWASM`;
- successful decrypt call.

Algorithm/profile upgrades create new profile identities. Historical receipts retain
the profile under which they were produced.

## Resource bounds

All protocol inputs are potentially adversarial.

Profiles should bound at least:

```text
roster size
contribution count
contribution bytes
vector dimension
proof bytes
participant/profile identifier lengths
transcript event count
decryption-share count
```

Do not allow a malformed round to become an unbounded browser/native memory or CPU
sink.

## Privacy metadata caution

Even when plaintext values are hidden, transcript metadata can expose:

- participation;
- timing;
- cohort size;
- rejection reasons;
- repeated participant pseudonyms;
- output/query patterns.

Receipts should minimize participant-identifying metadata while retaining enough
semantic evidence for the selected audit profile.

`encrypted payload` does not mean `private protocol metadata`.

## Nonclaims

MYC-SA-000R does not establish production cryptography, threshold FHE, malicious
security, dropout tolerance, anonymity, differential privacy, consent, Holochain
integrity, model quality, statistical validity, legal compliance or authority to
act on any aggregate.

It establishes the semantic/security contract future secure-aggregation
implementations must not weaken.