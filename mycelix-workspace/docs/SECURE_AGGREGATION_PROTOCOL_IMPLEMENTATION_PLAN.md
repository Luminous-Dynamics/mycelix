# MYC-SA-001A — Secure Aggregation Protocol Core Implementation Plan

## Status

Planning child of MYC-SA-000R / draft PR #1800.

This tranche deliberately implements the **protocol theorem before real
cryptography**. A deterministic toy backend may exercise transitions, but it must
be explicitly incapable of producing a qualified cryptographic-security claim.

## Objective

Create a small canonical Rust core for:

```text
plan identity
roster identity
contribution admission
round state transitions
sealing
aggregate binding
decryption/reconstruction evidence binding
transcript identity
final receipt
```

without choosing FHE, masked aggregation, MPC, threshold FHE, Holochain or Leptos
as the semantic owner.

## Proposed crate

Recommended new crate:

```text
mycelix-workspace/crates/mycelix-secure-aggregation-core/
```

If repository naming conventions strongly favor another location/name, preserve
the semantic boundary rather than the literal path.

Proposed minimal modules:

```text
src/
  lib.rs
  ids.rs
  plan.rs
  roster.rs
  contribution.rs
  state.rs
  transcript.rs
  receipt.rs
  backend.rs
  validation.rs
  error.rs
```

Do not add FHE-library dependencies in MYC-SA-001A.

## Dependency rule

Keep the pure core close to:

```text
serde
thiserror
stable commitment/profile primitives already canonical in Mycelix
```

Avoid:

```text
web-sys
Leptos
Holochain HDK
TFHE/OpenFHE bindings
network clients
ambient clocks
random UUID generation
persistent databases
```

The caller supplies semantic IDs/time evidence/profile references.

## Stable identities

Introduce typed newtypes/equivalents for at least:

```rust
ProtocolId
RoundId
RosterId
ParticipantRef
ContributionId
KeyEpochId
TranscriptId
AggregateId
ReceiptId
ProfileRef
CommitmentRef
```

Do not use interchangeable plain `String` values for every identity at the
canonical boundary.

Validation should bound lengths and reject empty/noncanonical forms according to
the selected profile.

## Plan

Conceptual shape:

```rust
pub struct SecureAggregationPlanV1 {
    pub protocol_id: ProtocolId,
    pub round_id: RoundId,
    pub protocol_profile: ProfileRef,
    pub roster: RosterRefV1,
    pub collection_threshold: u32,
    pub seal_policy: SealPolicyV1,
    pub contribution_policy: ContributionPolicyV1,
    pub aggregation_function: AggregationFunctionV1,
    pub value_domain: ValueDomainProfileV1,
    pub shape: ShapeProfileV1,
    pub crypto_profile: ProfileRef,
    pub key_epoch: KeyEpochId,
    pub decryption_profile: DecryptionProfileV1,
    pub release_profile: ProfileRef,
    pub deadline: Option<DeadlineEvidenceV1>,
}
```

The exact implementation may factor nested objects differently.

### Plan invariants

At construction validate at least:

```text
roster size > 0
collection threshold > 0
collection threshold <= roster size
profile IDs are admitted/nonempty
shape/value-domain profile structurally valid
seal-policy requirements internally consistent
decryption threshold valid for selected decryption profile
```

Once collection begins the plan is immutable.

## Threshold separation

Represent collection and decryption/reconstruction thresholds independently.

Example:

```rust
pub enum DecryptionProfileV1 {
    None,
    CentralKey { service: ParticipantRef },
    Threshold { threshold: u32, committee: RosterRefV1 },
    BackendDefined { profile: ProfileRef },
}
```

The pure core validates structural relationships only. It does not pretend a
Threshold variant establishes cryptographic threshold decryption without a
qualified backend.

## Roster

Conceptual shape:

```rust
pub struct RosterV1 {
    pub roster_id: RosterId,
    pub participants: Vec<RosterEntryV1>,
    pub profile: ProfileRef,
}

pub struct RosterEntryV1 {
    pub participant: ParticipantRef,
    pub authentication_profile: ProfileRef,
    pub max_contributions: u32,
    pub eligibility_evidence: Option<CommitmentRef>,
}
```

### Roster invariants

Reject:

- duplicate participant refs;
- empty roster;
- zero contribution allowance where profile forbids it;
- roster size beyond resource profile;
- conflicting participant/profile duplicates.

Sort/canonicalize set-like roster identity independently from caller insertion
order.

## Contribution policy

Freeze duplicate/supersession behavior explicitly:

```rust
pub enum ContributionPolicyV1 {
    RejectDuplicate,
    ExplicitSupersessionBeforeSeal,
}
```

Do not implement multiple submissions per participant in v1 unless required.

For explicit supersession require predecessor contribution identity and preserve
both events in transcript history.

## Contribution envelope

Conceptual shape:

```rust
pub struct ContributionEnvelopeV1 {
    pub contribution_id: ContributionId,
    pub protocol_id: ProtocolId,
    pub round_id: RoundId,
    pub roster_id: RosterId,
    pub participant: ParticipantRef,
    pub sequence: u64,
    pub nonce: NonceV1,
    pub crypto_profile: ProfileRef,
    pub key_epoch: KeyEpochId,
    pub payload_commitment: CommitmentRef,
    pub shape_profile: ProfileRef,
    pub value_domain_profile: ProfileRef,
    pub validity_evidence: Option<CommitmentRef>,
    pub authentication_evidence: CommitmentRef,
    pub supersedes: Option<ContributionId>,
}
```

The pure core need not contain ciphertext bytes. It can bind an exact payload
commitment supplied by the backend/adapter.

## Contribution admission

Admission order should be explicit and fail closed:

```text
round state accepts contributions?
protocol/round/roster match?
participant on roster?
profile/key epoch match?
sequence/nonce replay valid?
duplicate/supersession policy valid?
shape/domain profile match?
authentication evidence structurally present?
backend validity result admitted?
resource bounds satisfied?
```

A backend validation result is typed input to the state machine, not a generic
boolean called `verified`.

Conceptually:

```rust
pub enum ContributionValidationV1 {
    Admitted {
        backend_evidence: CommitmentRef,
    },
    Rejected {
        reason: ContributionRejectReasonV1,
        evidence: Option<CommitmentRef>,
    },
    Indeterminate {
        reason: ContributionIndeterminateReasonV1,
    },
}
```

## Replay state

Maintain bounded per-round replay state sufficient for:

```text
participant latest sequence
seen nonces
accepted contribution identity
supersession predecessor
```

Replay state belongs to the round theorem. A restart adapter must reconstruct it
from durable transcript/evidence before accepting more authoritative
contributions.

## Seal policy

Reference enum:

```rust
pub enum SealPolicyV1 {
    Explicit,
    FullRoster,
    Deadline,
    ThresholdAuto,
}
```

MYC-SA-001A should implement at least `Explicit` and optionally `FullRoster`.

`Deadline` requires explicit deadline/time evidence injected by the caller rather
than `Instant::now()`/`Date.now()` inside pure semantics.

`ThresholdAuto` should remain deferred unless there is a compelling use case,
because arrival order becomes membership-significant.

## State machine

Recommended authoritative states:

```rust
pub enum RoundStateV1 {
    Planned,
    Collecting,
    CollectionSealed,
    Aggregating,
    EncryptedAggregateBound,
    DecryptionPending,
    DecryptedResultBound,
    Finalized,
    TerminalFailure(TerminalFailureV1),
}
```

Do not create a separate `ThresholdReached` stored state unless it has semantic
consequence. It may be a derived predicate while remaining `Collecting`.

### Transition examples

```text
Planned -> Collecting
Collecting -> CollectionSealed
CollectionSealed -> Aggregating
Aggregating -> EncryptedAggregateBound
EncryptedAggregateBound -> DecryptionPending
EncryptedAggregateBound -> Finalized          // no-decryption profile
DecryptionPending -> DecryptedResultBound
DecryptedResultBound -> Finalized
```

No backward transition in v1.

## Terminal failures

Use typed terminal failures such as:

```rust
pub enum TerminalFailureV1 {
    CancelledBeforeSeal,
    TimedOutBeforeThreshold,
    InvalidSeal,
    AggregationRejected,
    AggregationFailed,
    AggregationIndeterminate,
    DecryptionUnavailable,
    DecryptionRejected,
    DecryptionFailed,
    DecryptionIndeterminate,
}
```

Keep input-level contribution rejection distinct from terminal round failure.

## Errors versus protocol dispositions

Rust `Err` should represent malformed calls/invariants that prevent a transition
from being evaluated.

Protocol-level negative outcomes should generally remain typed evidence:

```text
contribution rejected
round aggregation failed
indeterminate decryption
```

Do not discard them as exceptions without transcript evidence.

## Backend boundary

Separate aggregation and decryption concerns.

Conceptual traits:

```rust
pub trait SecureAggregationBackend {
    type ContributionArtifact;
    type AggregateArtifact;

    fn profile(&self) -> &ProfileRef;
    fn validate_contribution(
        &self,
        plan: &SecureAggregationPlanV1,
        envelope: &ContributionEnvelopeV1,
        artifact: &Self::ContributionArtifact,
    ) -> BackendContributionValidationV1;

    fn aggregate(
        &self,
        plan: &SecureAggregationPlanV1,
        admitted: &[AdmittedContributionRefV1],
    ) -> BackendAggregationResultV1<Self::AggregateArtifact>;
}

pub trait AggregateDecryptionBackend<A> {
    fn profile(&self) -> &ProfileRef;
    fn decrypt_or_reconstruct(
        &self,
        plan: &SecureAggregationPlanV1,
        aggregate: &A,
    ) -> BackendDecryptionResultV1;
}
```

Exact associated types may differ.

The pure core converts backend outputs into state transitions only after checking
profile/plan/aggregate binding.

## Deterministic toy backend

MYC-SA-001A may ship a test-only backend such as:

```text
DeterministicCommitmentSumTestBackendV1
```

but it must be impossible to confuse it with cryptography.

Recommended controls:

- type/module under `#[cfg(test)]` or explicit `test-backend` feature;
- profile name contains `test`/`non-cryptographic`;
- receipt classification `DevelopmentSimulation` or `TestOnly`;
- canonical qualification logic refuses `QualifiedCryptographic` for it.

The toy backend exists solely to exercise protocol state/transcript semantics.

## Aggregation result binding

Backend aggregation result should bind at least:

```text
exact plan ID/commitment
accepted contribution-set commitment
backend profile
aggregate artifact commitment
backend disposition/evidence
```

Reject result substitution from another round/profile/input set.

## Decryption result binding

Similarly bind:

```text
aggregate commitment
key epoch
decryption profile
decryption evidence commitment
output commitment
```

A successful decryption result for aggregate A cannot finalize aggregate B.

## Transcript events

Prefer typed events rather than one mutable status object.

Reference events:

```rust
PlanActivated
ContributionAccepted
ContributionRejected
ContributionSuperseded
CollectionSealed
AggregationStarted
AggregateBound
DecryptionRequested
DecryptionEvidenceBound
OutputBound
RoundFinalized
RoundFailed
```

Each event binds exact predecessor/state context where required.

## Transcript identity

The transcript commitment should be deterministic for the same semantic history.

Set-like collections such as accepted contributions at seal time should be
canonicalized by stable semantic identity rather than transport arrival order
unless the selected profile explicitly makes order meaningful.

Do not commit unstable local paths/log IDs/timestamps unless the time evidence is
part of the semantic profile.

## Receipt

Conceptual shape:

```rust
pub struct SecureAggregationReceiptV1 {
    pub protocol_id: ProtocolId,
    pub round_id: RoundId,
    pub plan_commitment: CommitmentRef,
    pub roster_commitment: CommitmentRef,
    pub transcript_commitment: CommitmentRef,
    pub crypto_profile: ProfileRef,
    pub key_epoch: KeyEpochId,
    pub accepted_set_commitment: CommitmentRef,
    pub rejected_set_commitment: CommitmentRef,
    pub aggregate_commitment: Option<CommitmentRef>,
    pub decryption_profile: DecryptionProfileV1,
    pub decryption_evidence: Option<CommitmentRef>,
    pub output_commitment: Option<CommitmentRef>,
    pub terminal_disposition: RoundTerminalDispositionV1,
    pub implementation_qualification_ref: Option<CommitmentRef>,
}
```

The receipt explicitly does not contain a generic `secure: true` field.

## Resource bounds

Add a plan/runtime resource profile or hard first-version constants covering:

```text
max roster size
max participant/profile ID length
max transcript events
max accepted/rejected contribution count
max contribution commitment length/profile
max proof/evidence reference count
max sequence/nonce state
```

Backend artifacts may add their own byte/dimension limits later.

## Canonicalization

Avoid hashing arbitrary serde JSON as protocol identity.

If Mycelix already has a qualified typed commitment/framing primitive, compose it.
Otherwise MYC-SA-001A should define an explicit versioned canonical byte encoding
for the small core identities or keep commitments abstract until a registered
canonical profile is available.

Required rule:

```text
same Rust struct value
    !=
stable protocol commitment unless canonical encoding is specified
```

## Time

Pure core transitions accept time evidence/context as values.

Example:

```rust
DeadlineEvidenceV1 {
    deadline_profile,
    deadline_value,
    evaluation_time,
    evidence_ref,
}
```

The first v1 can avoid deadline-based sealing entirely to stay deterministic.

## Persistence boundary

MYC-SA-001A defines recovery semantics but does not need to implement storage.

Expose enough transcript/state reconstruction to support a later durable adapter.

Required invariant:

```text
missing process memory
    !=
round never accepted contributions
```

A recovery API should rebuild state from authenticated/bound transcript evidence
rather than initializing a fresh round with the same ID.

## Authentication boundary

The core consumes typed authentication evidence/disposition from an external
qualified verifier.

It does not own DID, Xenia signature, credential or Holochain identity mechanics.

Required distinction:

```text
authenticated participant
    !=
eligible participant
    !=
valid contribution
```

Roster policy, authentication and backend contribution validity remain separate.

## MYC-SA-001AQ qualification corpus

Pure/state-machine fixtures must cover at least:

1. valid plan construction;
2. empty roster rejection;
3. duplicate roster participant rejection;
4. collection threshold zero rejection;
5. collection threshold > roster rejection;
6. structurally invalid decryption threshold rejection;
7. wrong protocol ID contribution rejected;
8. wrong round ID rejected;
9. wrong roster ID rejected;
10. participant absent from roster rejected;
11. wrong key epoch rejected;
12. wrong crypto profile rejected;
13. replayed nonce rejected;
14. sequence rollback rejected;
15. duplicate participant rejected under `RejectDuplicate`;
16. valid explicit supersession preserves predecessor;
17. missing/wrong supersession predecessor rejected;
18. contribution after seal rejected;
19. backend validation rejection preserved in transcript;
20. backend indeterminate validation does not become admission;
21. threshold reached does not seal an Explicit-seal round;
22. explicit seal below threshold rejected;
23. explicit seal at threshold succeeds;
24. deterministic seal accepted-set identity across transport permutations;
25. no backward state transitions;
26. aggregate result from wrong plan rejected;
27. aggregate result from wrong accepted-set rejected;
28. decryption evidence for wrong aggregate rejected;
29. decryption evidence for wrong key epoch rejected;
30. aggregation failure distinct from indeterminate aggregation;
31. decryption unavailable distinct from failed and indeterminate;
32. transcript mutation changes/fails transcript commitment;
33. roster mutation changes/fails plan commitment;
34. test backend cannot produce cryptographic-qualified disposition;
35. final receipt does not imply DP/consent/authority;
36. reconstruction from same transcript yields same state/receipt;
37. truncated/missing required transcript evidence fails closed;
38. resource limits enforced;
39. unknown future profile rejected rather than downgraded;
40. exact-head locked qualification and checkout immutability.

## Property tests

Useful bounded properties:

```text
accepted_count <= roster_size
one accepted active contribution per participant under RejectDuplicate
sealed round accepts no new contribution
finalized round has exactly one terminal disposition
state rank never decreases
same canonical event history reconstructs same state
permuting transport order before explicit seal does not change accepted-set commitment
```

## Mutation tests

Add targeted mutations to prove critical checks matter:

- remove round check;
- remove roster check;
- remove key-epoch check;
- remove nonce replay check;
- allow post-seal contribution;
- allow aggregate substitution;
- allow decryption aggregate substitution;
- promote test backend to qualified cryptographic.

At least one qualification lane should demonstrate those mutations are caught,
without shipping mutation variants in production code.

## First code-stack suggestion

Keep implementation reviewable:

```text
MYC-SA-001A1  IDs + plan + roster validation
MYC-SA-001A2  contribution admission + replay/supersession
MYC-SA-001A3  state machine + seal semantics
MYC-SA-001A4  backend traits + test-only backend
MYC-SA-001A5  transcript reconstruction + receipt
MYC-SA-001AQ  exact-head qualification/adversarial corpus
```

These labels are provisional. Fewer PRs are fine if each diff remains narrow.

## First real backend selection gate

Only after protocol qualification choose MYC-SA-002A.

Evaluate separately:

### CentralKeyFheV1

Good for proving encrypted computation with an explicit central decryption trust
assumption.

### MaskedSecureAggregationV1

Potentially simpler/faster for sum/vector-sum but requires dropout/recovery and
malicious-participant threat analysis.

Do not select solely on implementation convenience.

Threshold FHE remains later because it requires scheme-specific distributed key
generation/decryption proof semantics rather than generic secret sharing.

## DP composition

A later release pipeline should compose receipts rather than merge semantics:

```text
SecureAggregationReceipt
    + PrivacyReleaseReceipt
    + purpose/consent/authority evidence
    -> domain-specific release decision
```

No secure-aggregation state may directly decrement or reset DP accounting unless a
separate composition theorem explicitly owns that integration.

## FL composition

Simple secure sum/vector sum cannot directly implement robust aggregators that
need per-update distances/ordering.

The architecture must therefore support profile declarations such as:

```text
secure-sum FedAvg-compatible
not compatible with Krum visibility requirements
```

rather than pretending every FL defense composes transparently with every privacy
mechanism.

## Browser relationship

The pure core must compile independently of browser APIs.

Later `MYC-SA-WASM` may:

- bind actual cryptographic backend to WASM;
- execute expensive operations in `mycelix-browser-worker`;
- expose reactive state to Leptos.

The UI consumes typed round/receipt states; it does not own them.

## Holochain relationship

A Holochain adapter may persist/share plans, roster evidence, contribution
commitments or receipts according to domain design.

Required rule:

```text
valid Holochain entry/action
    !=
secure aggregation theorem satisfied
```

Integrity/coordinator zomes should validate every theorem that can be checked from
DHT-visible data without relying on a trusted UI.

## Nonclaims

Passing MYC-SA-001A would establish only the frozen protocol state/transcript
semantics using a non-cryptographic qualification backend.

It would **not** establish FHE security, threshold cryptography, malicious-secure
MPC, dropout tolerance, anonymity, differential privacy, consent, scientific
validity, Holochain deployment correctness, or authority to act on an output.