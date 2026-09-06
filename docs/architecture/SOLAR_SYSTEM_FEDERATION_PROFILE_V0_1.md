# Mycelix Solar-System Federation Profile v0.1

Status: **Draft / architecture profile**

This profile defines how Mycelix federations should preserve security, autonomy, epistemic integrity, and eventual reconciliation under extreme communication delay and disruption, including planetary, cislunar, deep-space, and interplanetary deployments.

It is intentionally transport-agnostic at the Mycelix semantic layer. Interplanetary carriage SHOULD use standardized Delay/Disruption Tolerant Networking (DTN), Bundle Protocol Version 7 (BPv7), and Bundle Protocol Security (BPSec) rather than duplicating those transport-layer security services.

This profile does **not** claim that current Mycelix/Holochain deployments are already interplanetary-qualified. It freezes the invariants that future implementation and qualification must satisfy.

## 1. Design premise

Interplanetary operation invalidates several terrestrial assumptions:

- continuous end-to-end connectivity;
- globally fresh membership or revocation state;
- low-latency consensus across all participants;
- globally current clocks;
- synchronous federated-learning rounds;
- immediate human or remote-operator intervention;
- instantaneous software or policy propagation.

A secure interplanetary Mycelix deployment therefore MUST treat communication partitions as a normal operating state rather than an exceptional failure.

The profile is based on one central rule:

> **Evidence may cross federation boundaries. Authority does not.**

A remote federation may supply authenticated evidence, claims, learned defenses, model candidates, revocation information, or governance proposals. A receiving federation MUST independently evaluate those artifacts under local policy before they can create local authority or effects.

## 2. Security planes

The profile separates six planes:

1. **Observation plane** — local telemetry, application events, device evidence, machine provenance, threat observations.
2. **Learning plane** — private/federated learning, Byzantine-robust aggregation, threat-model adaptation, acquired immunity.
3. **Science plane** — holdouts, privacy evaluation, backdoor evaluation, lineage separation, calibration, contraindication evidence.
4. **Epistemic plane** — claims, evidence, contradictions, challenges, uncertainty, reproducibility.
5. **Authority plane** — identity, federation membership, policy, delegation, revocation, Holochain integrity, local governance.
6. **Effect plane** — capability restriction, quarantine, rollback, reconfiguration, execution fencing, requalification.

The following distinctions are normative:

```text
observation != inference
inference != evidence
agreement != truth
reputation != truth
evidence != authority
authority != execution
execution != confirmed outcome
foreign evidence != local authority
```

No component MAY collapse those transitions merely because a preceding object is signed or high-confidence.

## 3. Federation topology

The profile assumes a hierarchy of independently sovereign security domains, for example:

```text
process
  -> machine
    -> habitat / vehicle
      -> settlement / station
        -> regional or orbital federation
          -> planetary federation
            -> interplanetary evidence federation
```

The hierarchy is descriptive rather than mandatory. Implementations MAY use different topologies, but each federation MUST have an explicit scope and authority root.

A federation MUST NOT depend on another planet, settlement, or spacecraft being online in order to perform actions required for immediate local safety and survival that were already authorized by local policy.

Conversely, disconnection MUST NOT silently expand local authority.

## 4. Federation identity and generations

Security-bearing cross-federation artifacts MUST bind at least:

```text
federation_id
federation_epoch
authority_root_commitment
policy_commitment
issuer_identity
issuer_authority_generation
artifact_type
artifact_schema_version
artifact_commitment
causal_predecessor_or_parent
```

Logical identity is insufficient. The same federation name, node identifier, validator identifier, session UUID, round number, or model name MUST NOT make different authority generations interchangeable.

The receiving federation MUST treat a different `federation_epoch`, authority root, policy digest, or authority generation as a different security context.

## 5. Local sovereignty theorem

For any foreign artifact `F` and local consequential action `A`:

```text
ForeignArtifact(F)
    -> verify provenance
    -> verify integrity/authentication
    -> classify evidence quality
    -> evaluate local freshness policy
    -> evaluate local scientific/epistemic requirements
    -> evaluate local authority policy
    -> LocalAuthority(A)
```

There MUST NOT exist a direct transition:

```text
ForeignArtifact(F) -> LocalAuthority(A)
```

This applies even when the foreign federation is normally trusted.

## 6. Disconnected autonomy budgets

Every federation that may operate while disconnected SHOULD have an explicit `AutonomyBudget`.

A conceptual `AutonomyBudgetV1` contains:

```text
federation_scope
policy_digest
issued_authority_generation
validity_basis
allowed_consequence_classes
maximum_blast_radius
maximum_delegation_depth
maximum_resource_commitment
maximum_disconnected_duration
maximum_evidence_age_by_class
required_local_quorum_by_class
required_machine_health_level
required_software_generation
emergency_overrides
reconciliation_obligations
```

The budget MUST obey authority attenuation:

```text
child_autonomy <= parent_authority
```

and disconnection MUST NOT increase any authority dimension.

Where policy chooses degradation under prolonged isolation, the intended relationship is:

```text
less externally verifiable freshness
    -> narrower permitted consequence
    -> stronger local corroboration requirements
```

not:

```text
less connectivity -> unrestricted emergency power
```

## 7. Consequence-dependent evidence

The amount and independence of evidence required MUST increase with potential consequence.

A profile MAY define consequence classes such as:

```text
C0  observe / log
C1  request challenge / increase monitoring
C2  lower learning weight / defer candidate
C3  reject one contribution / restrict one capability
C4  temporary local workload quarantine
C5  machine or service isolation
C6  federation-level revocation / critical infrastructure action
C7  irreversible or life-critical action
```

High consequence actions MUST NOT be justified solely by reputation, a single adaptive model, a foreign claim, or a stale authority snapshot.

Uncertainty SHOULD reduce blast radius before it increases coercive authority.

## 8. Time evidence

Interplanetary authority MUST NOT assume access to a globally current wall clock.

Implementations SHOULD distinguish at least:

```text
TrustedAbsoluteTime
LocalMonotonicTime
CausalGeneration
ExternallyWitnessedTime
UnknownOrDegradedTime
```

An authority object's validity SHOULD be derived from the strongest available time/freshness evidence and MUST bind its time-quality assumptions.

Where multiple upstream validity boundaries exist:

```text
valid_authority <= minimum(all upstream validity boundaries)
```

A stale membership generation, expired key, expired machine attestation, stale revocation snapshot, expired policy, or expired scientific qualification MUST NOT be hidden by a still-valid downstream lease.

## 9. Revocation under partition

Global instantaneous revocation is impossible under long-delay networking.

Therefore:

1. revocation state MUST be generation-based and monotonic;
2. disconnected authority MUST have bounded lifetime or bounded consequence;
3. high-consequence authority SHOULD require fresher revocation evidence than low-consequence authority;
4. a node MUST record which revocation generation it knew when an action was authorized;
5. later receipt of a newer revocation generation MUST NOT rewrite history; it MUST create reconciliation evidence;
6. local policy MUST define which actions become invalid for future execution after new revocation information arrives.

The profile distinguishes:

```text
historically authorized under known state
```

from:

```text
currently authorized under latest locally verified state
```

## 10. Delay/disruption-tolerant carriage

Mycelix interplanetary semantics SHOULD be carried using standardized DTN mechanisms such as BPv7 and BPSec.

Mycelix MUST NOT duplicate Bundle Protocol security merely to provide integrity/confidentiality at the same scope.

The layering is:

```text
Mycelix semantic artifact
  -> optional Xenia semantic-authority envelope
    -> BPv7 bundle
      -> BPSec integrity/confidentiality as required
        -> DTN store-carry-forward transport
```

BPSec protects bundle data according to DTN security policy. Xenia/Mycelix remain responsible for the higher-level question of whether the authenticated semantic artifact carries recognized authority for a particular action.

Transport authentication MUST NOT be treated as proof of semantic authority.

## 11. Cross-federation artifact classes

The first interplanetary profile SHOULD support only a small set of explicit artifact classes:

```text
FederationCheckpoint
MembershipSnapshot
RevocationSnapshot
PolicySnapshot
ThreatObservationSummary
QualifiedThreatAssessment
AcquiredImmunityCapsule
ContraindicationEvidence
SoftwareGenerationEvidence
MachineHealthEvidence
ResponseOutcomeReceipt
ReconciliationStatement
```

Each artifact MUST be immutable, versioned, domain-separated, provenance-bound, and federation-scoped.

An artifact MUST NOT contain an untyped free-form field that can silently substitute for authority-critical identity, generation, policy, or evidence commitments.

## 12. Acquired immunity across planets

Federated learning MUST NOT require a synchronous Solar-System-wide round.

Instead, federations SHOULD learn locally and exchange qualified artifacts asynchronously.

A conceptual `AcquiredImmunityCapsuleV1` SHOULD bind:

```text
origin_federation
origin_epoch
threat_family
parent_model_commitment
candidate_defense_commitment
training_evidence_commitment
privacy_policy_commitment
holdout_commitment
lineage_policy_commitment
backdoor_evaluation_commitment
safety_evaluation_commitment
estimated_effectiveness_bounds
known_failure_modes
contraindication_commitment
qualification_policy_commitment
qualification_evidence_root
issued_authority_generation
validity_basis
```

Receipt of a capsule MUST create only foreign evidence.

Local adoption requires a separate locally qualified object, for example:

```text
ForeignImmunityCapsule
    -> provenance verification
    -> local compatibility checks
    -> local privacy policy
    -> local holdout / safety evaluation
    -> local promotion policy
    -> LocallyQualifiedImmunity
```

No remote federation may directly promote a local defensive model by sending a capsule.

## 13. Contraindication memory

The interplanetary immune system MUST preserve negative knowledge as first-class evidence.

Examples include:

- false-positive detector regimes;
- defenses that damage legitimate workloads;
- model-family failures;
- privacy leakage from learned representations;
- reputation heuristics exploited by collusion;
- mitigations that create dangerous operational side effects.

A receiving federation SHOULD evaluate both immunity and contraindication evidence before adoption.

This reduces the risk of cyber autoimmunity propagating across federations.

## 14. Private learning data

Compression, hyperdimensional representation, or aggregation readiness MUST NOT be treated as confidentiality.

Raw gradients, HDC/hypervector representations, sensitive telemetry, and client-local training evidence SHOULD remain off globally replicated DHT surfaces unless a separate privacy analysis explicitly authorizes publication.

The preferred split is:

```text
PRIVATE LEARNING PLANE
local observation
 -> private representation
 -> Xenia-authenticated private carriage
 -> secure/private federation aggregation

PUBLIC / REPLICATED EVIDENCE PLANE
round manifest
contribution commitment
aggregation transcript commitment
qualification certificate
promotion certificate
revocation
outcome receipt
```

## 15. Holochain role

A Holochain DNA used under this profile SHOULD represent deterministic local federation constitution and immutable evidence transitions.

The integrity layer MUST NOT depend on mutable remote truth such as:

- the currently reachable Earth authority;
- the globally latest revocation state;
- a live remote reputation query;
- a live foreign model server;
- current remote network health;
- current remote wall-clock values.

Instead, integrity validation SHOULD consume explicitly referenced immutable snapshots and deterministic dependencies.

The target theorem is:

> Even if a coordinator zome is replaced by a malicious implementation, it cannot cause an authority-bearing interplanetary claim to become valid DHT state without the immutable evidence required by the integrity constitution.

## 16. Partition-safe effect authority

Adaptive learning and threat inference MUST NOT directly execute consequential effects.

The intended transition is:

```text
ThreatObservation
 -> QualifiedThreatAssessment
 -> ResponseProposal
 -> LocalAuthorityEvaluation
 -> AuthorizedResponseLease
 -> EffectFence
 -> Effect
 -> ResponseOutcomeReceipt
```

The `AuthorizedResponseLease` MUST bind the exact federation scope, authority generation, evidence commitment, policy commitment, consequence ceiling, freshness basis, and effect target.

Effect systems SHOULD consume move-only or otherwise non-replayable authority tokens where practical.

## 17. Outcome ambiguity

Network or storage ambiguity MUST remain representable.

A consequential operation SHOULD use a closed-world result such as:

```text
Confirmed
ProvenNotApplied
OutcomeUnknown
```

`OutcomeUnknown` MUST NOT become an ordinary retry.

The exact operation identifier and authority lineage MUST be retained until reconciliation establishes whether the prior action occurred.

This requirement applies to remote governance actions, revocations, software rollout, quarantine, model promotion, resource commitments, and other external effects.

## 18. Reconnection and reconciliation

Reconnection MUST NOT mean "replace local state with the remote latest state."

Each federation SHOULD exchange authenticated causal history and reconcile explicitly.

A reconciliation procedure SHOULD be able to determine:

```text
what evidence each federation possessed
which authority generation was active
which revocation generation was known
which policy version governed the action
which software/machine generation executed it
what action was attempted
whether the outcome was confirmed or ambiguous
what later evidence changes future authority
```

Historical actions that were valid under then-known state MUST remain historically represented as such. New evidence may change future authority or trigger review without falsifying the recorded past.

## 19. Forks and ordering

Critical security history MUST NOT infer global linearizability from DHT retrieval ordering, link ordering, arrival time, or wall-clock timestamps alone.

Security-critical histories SHOULD use:

```text
explicit sequence / generation
explicit predecessor commitment
fork detection
checkpoint commitments
independent witnesses where required
```

Conflicting histories are evidence and MUST remain representable rather than being silently resolved by whichever item is returned last.

## 20. Emergency operation

Emergency policy MUST be explicit before disconnection where possible.

An emergency mode MUST define:

- trigger evidence;
- maximum consequence;
- local quorum or responsible authority;
- maximum duration or causal generation;
- delegation restrictions;
- mandatory audit evidence;
- mandatory later reconciliation;
- conditions that terminate emergency authority.

Emergency authority MUST attenuate from a recognized root. An emergency MUST NOT mint arbitrary new sovereignty from the absence of a remote controller.

For life-critical systems, local survival authority MAY intentionally exceed ordinary remote-dependency constraints, but that authority MUST be predeclared, tightly scoped, locally auditable, and independently reviewable after reconnection.

## 21. Initial normative invariants

The first implementation tranche SHOULD freeze the following invariants as qualification vectors:

- **SSF-001** Foreign evidence never directly creates local authority.
- **SSF-002** Disconnection never increases authority.
- **SSF-003** Child autonomy never exceeds parent authority.
- **SSF-004** High-consequence actions require stronger and fresher evidence than low-consequence actions.
- **SSF-005** Different federation epochs or authority generations are never interchangeable.
- **SSF-006** Stale upstream prerequisites cannot produce fresh downstream authority.
- **SSF-007** Remote revocation delay remains explicit in action evidence.
- **SSF-008** Adaptive inference never directly determines DHT validity or executes effects.
- **SSF-009** Reputation never substitutes for evidence or authority.
- **SSF-010** Transport security never substitutes for semantic authority.
- **SSF-011** Private learning representations are not presumed safe for public replication.
- **SSF-012** Synchronous global FL rounds are not required for interplanetary learning.
- **SSF-013** Outcome ambiguity is represented explicitly and never blindly retried.
- **SSF-014** Reconnection performs causal reconciliation, not last-writer-wins replacement.
- **SSF-015** DHT/link retrieval order is never treated as an authoritative security chronology.
- **SSF-016** Emergency authority is pre-scoped and cannot self-expand.
- **SSF-017** Contraindication evidence travels with or alongside acquired-immunity evidence.
- **SSF-018** A malicious coordinator cannot forge constitutionally valid authority-bearing state.

## 22. Qualification scenarios

At minimum, an implementation claiming this profile SHOULD demonstrate:

1. Earth and Mars federations operating independently through a prolonged simulated partition.
2. Local low-consequence operation continuing without remote connectivity.
3. High-consequence remote-dependent authority narrowing or expiring as designed.
4. A revocation created remotely during partition and correctly reconciled later without rewriting historical action evidence.
5. Foreign acquired-immunity evidence arriving late and requiring local qualification before adoption.
6. A malicious foreign federation sending a syntactically valid but scientifically unqualified defense and failing to obtain local promotion.
7. Conflicting foreign histories being preserved as fork evidence.
8. DTN bundle duplication/reordering/delay without semantic replay of authority-bearing operations.
9. `OutcomeUnknown` on an external effect followed by explicit reconciliation without duplicate execution.
10. Loss of trusted absolute time without converting uncertainty into fresh authority.
11. Compromise of an adaptive threat model without direct capability or effect authority.
12. Replacement of a coordinator zome with a malicious implementation without violation of integrity invariants.

## 23. Implementation sequence

The recommended implementation order is deliberately conservative:

```text
SSF-PR0  normative invariant corpus and canonical types
SSF-PR1  federation epoch + immutable snapshot contracts
SSF-PR2  autonomy-budget and consequence-class model
SSF-PR3  time-evidence + freshness attenuation model
SSF-PR4  cross-federation immutable artifact envelopes
SSF-PR5  DTN/BPv7/BPSec adapter prototype
SSF-PR6  asynchronous acquired-immunity capsule flow
SSF-PR7  effect-authority / outcome-reconciliation integration
SSF-PR8  two-federation partition simulator and adversarial qualification
```

The DTN adapter SHOULD remain downstream of the semantic contracts. A transport implementation must not define Mycelix security semantics.

## 24. Non-claims

This v0.1 profile does not establish:

- flight qualification;
- radiation tolerance;
- real-time control suitability;
- human-rated life-support suitability;
- formal correctness of the current Holochain FL zomes;
- empirical proof of acquired immunity;
- malicious-secure MPC merely because private aggregation boundaries exist;
- a completed zkSTARK system merely because proof-related types exist;
- trusted interplanetary time;
- global instantaneous revocation;
- a single globally synchronous DHT or consensus domain.

Those require independent implementation and qualification programs.

## 25. Target property

The long-term target is a system in which a federation may remain disconnected for an extended period and later provide enough authenticated causal evidence for another federation to determine:

```text
what it knew
what it did not know
what authority it possessed
what policy constrained it
what software and machine generation acted
what evidence justified each consequential action
what outcomes were confirmed or uncertain
what new information arrived later
```

without requiring either federation to surrender local sovereignty.

That is the intended meaning of **interplanetary federated security** in Mycelix.
