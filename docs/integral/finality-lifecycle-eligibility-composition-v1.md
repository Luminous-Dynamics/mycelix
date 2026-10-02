# Finality/lifecycle eligibility composition v1

Status: **ReferenceModelOnly**

D6N and D6O establish two distinct evidence layers:

    D6N = relationship of an observation to the external effect
    D6O = lifecycle eligibility of that observation

D6P composes them without collapsing evidence into authority.

The core rule is:

    D6N CorroboratingIndependent
        +
    D6O EligibleCurrent
        +
    exact join
        =
    current-finality-eligible witness

## Why the composition boundary exists

D6N can report two independent corroborating observations, but it cannot know from the observation set alone whether each observer is still eligible under current lifecycle state.

D6O can establish that an observation was eligible under an observer generation and dependency snapshot, but that does not say whether the observation corroborates the target external effect.

Therefore neither layer is sufficient by itself.

    observer count != current eligible witness count
    lifecycle eligibility != external finality
    D6N finality evidence != actuation authorization

D6P is a join, not a new source of truth.

## Evidence identity is single-valued at the join

D6P consumes D6N evidence by observation ID, so the D6N provenance boundary treats
a duplicate supplied identity as ambiguous input rather than applying last-write-wins.
This applies even when only one observation ID is required: a conflicting second
representation cannot be hidden by matching the cardinality check.

The assessment constructor follows the same rule and returns insufficient evidence
for duplicate observation identities. This keeps source selection deterministic and
independent of input order.

## Exact D6N artifact

D6P consumes D6N ObservationSetAssessmentV1 as an exact reference-model artifact.

The assessment must bind:

- observation-set ID;
- D6N assessment commitment;
- every observation ID in the observation set;
- each D6N ObservationClassificationV1.

The set-level D6N assessment commitment remains a reference-model artifact:

    assessment_commitment == "assessment:" + set_commitment

Each per-observation D6N assessment item now has a separate canonical SHA-256
commitment over its complete assessment payload, including the exact D6M
observation commitment. D6P carries and verifies this per-item commitment rather
than treating the set-level commitment as sufficient identity.

D6P does not recompute a second conflict taxonomy.

## D6M observation identity

D6M observations now carry a canonical, versioned SHA-256 commitment:

    MYCELIX-INTEGRAL-D6M-OBSERVATION-V1\0

The commitment covers the complete observation payload: effect and lineage identity,
lifecycle generation, route/provider identity, provider outcome identity, request and
idempotency bindings, semantic environment, observation frontier, observed state,
source, evidence root, and claim ceiling.

This is an integrity identity, not a provenance oracle. A caller can recompute a
forged observation commitment after changing a field; authoritative D6N reconstruction
must still compare the observation to the effect, route, current frontier, and
observation-set semantics.

The D6N assessment item carries that exact D6M commitment and then commits the full
assessment item under:

    MYCELIX-INTEGRAL-D6N-ASSESSMENT-V1\0

The resulting chain is:

    D6M observation
        -> canonical observation commitment
        -> D6N assessment item
        -> canonical assessment-item commitment
        -> D6P witness commitment

This makes semantic substitution visible at each boundary without conflating
cryptographic integrity with source authority. The conformance tests also mutate
each committed D6M observation field and each committed D6N assessment field,
ensuring the reference commitment cannot silently omit a newly relevant field.

## Exact D6O artifact

D6P consumes D6O EvidenceEligibilityReceiptV1.

A current witness requires:

- EligibleCurrent disposition;
- Live provenance;
- exact observation ID;
- exact observer identity;
- exact observer generation;
- matching observation profile;
- matching semantic environment;
- exact observation frontier;
- exact current frontier;
- current_generation_id equal to observer_generation_id;
- lifecycle profile matching the requested D6O lifecycle profile;
- D6N classification matching the eligibility receipt classification.

A HistoricalOnly receipt is evidence of historical eligibility, not a current-finality witness.

## Profile namespaces

The D6N finality qualification profile and D6O lifecycle profile are distinct semantic namespaces.

D6N binds:

    observation_set.qualification_profile_id

D6O binds:

    eligibility_receipt.qualification_profile_id

D6P therefore carries both values and requires the caller to supply the exact D6O lifecycle profile ID. It does not incorrectly require the two IDs to be equal.

## Witness commitment

Each D6P witness has a canonical commitment computed over the complete witness
payload, with the commitment field itself excluded from the payload before hashing.
This binds the D6N/D6O join's semantic fields together, including observer identity,
generation and lifecycle bindings, dependency snapshot, classifications, frontiers,
and claim ceiling.

This commitment provides integrity and exact-object identity; it does not establish
that the witness was produced by authoritative D6N/D6O sources. That provenance is
checked separately by the qualified D6P reconstruction path.

## Independence counts are observer-distinct

An independent-witness threshold counts independent observers, not merely
distinct observation IDs. D6N therefore treats two observations from the same
observer identity as sharing a dependency even when their evidence and custody
roots differ.

This prevents one observer from inflating a multi-observer threshold by emitting
multiple independently classified observations. Distinct observers remain subject
to the existing evidence/custody/upstream dependency checks as well.

## Cross-layer semantic join binding

D6P does not treat shared observation IDs as sufficient evidence that the D6N and D6O objects describe the same underlying observation.

Each FinalityWitnessEligibilityV1 now carries the exact D6N assessment-item commitment in addition to the D6N observation-set commitment. The qualified D6P reconstruction verifies the witness against the specific D6N assessment item, the specific D6N evidence object, the selected D6O receipt, the observation set, and the requested current frontier.

The join therefore requires all of the following to agree:

- observation identity;
- observer identity;
- D6N observation-set identity and commitment;
- D6N per-observation assessment commitment;
- exact D6M observation commitment carried by that assessment;
- D6N classification and evidence/custody roots;
- D6N observation-to-set semantic fields;
- D6O generation, eligibility, disposition, and dependency-snapshot identities;
- observation frontier;
- current frontier;
- lifecycle profile.

Conflicting duplicate evidence or receipt representations for the same observation ID are rejected rather than resolved by map insertion order. This makes source selection deterministic and prevents a valid object from one revision being silently paired with a valid object from another revision.

This is an integrity/semantic-binding boundary, not a provenance authority oracle. The authoritative D6N/D6O reconstruction remains responsible for proving that the selected objects actually originate from their respective authoritative sources. This separation is consistent with provenance models that distinguish identified entities/versions from the derivation and generation relationships that establish how an object was produced.

## Recomputed witness count

D6N exposes an independent_count. D6P does not trust that count as current-finality authority.

Instead, for every D6N CorroboratingIndependent observation:

    exact D6O EligibleCurrent receipt
        -> count 1

Anything else:

    -> count 0

The final current witness count is derived from these exact joins.

This prevents a historical D6N assessment from remaining sufficient after observer revocation, generation rotation, dependency change, or other lifecycle invalidation.

## Failure semantics

D6P distinguishes binding failures from insufficient current witnesses.

Examples:

- observation ID mismatch -> BlockedBinding;
- D6N assessment/set commitment mismatch -> BlockedBinding;
- lifecycle profile mismatch -> BlockedProfile;
- frontier mismatch -> BlockedCurrentness;
- lifecycle-blocked receipt -> BlockedLifecycle;
- dependency-blocked receipt -> BlockedDependency;
- continuity-blocked receipt -> BlockedContinuity;
- archive provenance -> BlockedArchive;
- missing/non-current receipt without a more specific binding failure -> InsufficientEligibleWitnesses.

If multiple distinct current-witness failure causes exist, D6P does not select one as the winner. It returns InsufficientEligibleWitnesses.

This preserves arrival-order and failure-order independence.

## Contest preservation

D6N contradictory classifications remain first-class historical evidence.

If a D6N observation is:

    ContradictoryIndependent
    or
    ContradictoryDependent

D6P increments preserved_contradictory_count and returns Contested.

A later lifecycle revocation does not turn a contradiction into a non-event.

The distinction is:

    lifecycle ineligibility
        !=
    evidence falsity

The underlying D6N observation remains retained.

## Generation continuity

A superseded observer generation cannot count under the same observer ID.

D6P accepts only the exact D6O receipt's current generation binding. If the receipt refers to a historical generation or has a mismatched current generation, it does not contribute to the current witness count.

This composes D6I no-resurrection and D6O generation continuity without rewriting old evidence.

## Dependency freshness

D6O may downgrade current eligibility after a dependency snapshot changes.

D6P consumes the resulting D6O disposition rather than attempting to reconstruct dependency semantics itself.

Therefore:

    old observation
        +
    later dependency change
        ->
    historical evidence retained
        +
    current witness count reduced

No historical D6N record is mutated.

## Archive boundary

Archived D6O eligibility cannot satisfy current finality.

Archive evidence may preserve:

- historical observer state;
- historical eligibility;
- reconstruction inputs;
- audit context.

It cannot silently become a live current witness.

## Witness registry semantics

The ledger's `witnesses` map is a single-view convenience registry keyed by observation ID.

It intentionally rejects a second witness version for the same observation rather than
silently replacing an earlier frontier-bound witness. Historical frontier versions are
retained in immutable D6P compositions, whose canonical commitments include their
frontier-bound witness material.

Consumers that need to select from the ledger must use an explicit expected frontier:

    witness_at_frontier(observation_id, expected_frontier_root)

This selection check does not establish that the supplied frontier is authoritative.
The caller must still obtain the authoritative frontier from its trust domain.

## Terminal receipt semantics

FinalityEligibilityLedgerV1 treats only EligibleCurrent composition receipts as terminal for an effect.

These remain non-terminal:

- InsufficientEligibleWitnesses;
- Contested;
- BlockedBinding;
- BlockedProfile;
- BlockedCurrentness;
- BlockedLifecycle;
- BlockedDependency;
- BlockedContinuity;
- BlockedArchive;
- BlockedAuthorization.

This permits later evidence to establish a new qualified current composition without rewriting earlier insufficient or contested receipts.

## No authority promotion

D6P creates qualification evidence only.

A CurrentFinalityEligibilityReceiptV1 cannot:

- authorize actuation;
- mint authority;
- mint consent;
- mint capacity;
- create a new external effect;
- rewrite D6N observation history;
- rewrite D6O lifecycle history.

The helper current_finality_receipt_is_non_authorizing is intentionally explicit.

## Symthaea boundary

Symthaea may assemble candidate D6N/D6O joins and identify missing lifecycle evidence.

It may not:

- promote HistoricalOnly to EligibleCurrent;
- invent an eligibility receipt;
- choose a lifecycle failure winner;
- replace an exact D6N assessment;
- replace an exact D6O receipt;
- resolve D6N contradictions;
- authorize actuation.

Mycelix remains the semantic root and qualification authority.

## Adversarial corpus

The module contains source-level tests for:

1. eligible independent witness counts;
2. historical-only witness does not count;
3. lifecycle-blocked witness does not count;
4. dependency-blocked witness does not count;
5. continuity-blocked witness does not count;
6. archived eligibility does not count;
7. observation ID mismatch;
8. observer-generation mismatch;
9. D6N assessment commitment mismatch;
10. semantic environment mismatch;
11. frontier mismatch;
12. lifecycle profile mismatch;
13. two eligible witnesses satisfy a threshold;
14. many ineligible witnesses cannot substitute;
15. contradictory D6N evidence remains contested;
16. superseded generation cannot count under the same observer ID;
17. dependency invalidation does not rewrite observation history;
18. archive mirror cannot count as current independent evidence;
19. divergent duplicate witness identity is rejected;
20. conflicting terminal receipts are rejected;
21. non-terminal receipt can be followed by a qualified receipt;
22. out-of-order D6N/D6O delivery converges;
23. Symthaea composition remains non-authoritative;
24. composition cannot authorize actuation;
25. lifecycle evidence alone cannot establish finality;
26. arrival order cannot change the eligible witness count;
27. cross-frontier witness replacement is rejected and explicit witness selection requires the expected frontier;
28. duplicate D6N evidence identities are rejected rather than resolved by last-write-wins;
29. canonical D6M observation mutations and D6N assessment-item substitutions are rejected at their authoritative boundaries.
30. multiple observations from the same observer cannot inflate the independent-witness threshold.

## Claim ceiling

**ReferenceModelOnly.**

D6P establishes deterministic composition semantics for the exact D6N and D6O reference-model inputs.

It does not establish:

- external physical truth;
- observer trust;
- cryptographic authenticity;
- revocation infrastructure;
- production finality;
- Byzantine consensus;
- legal/accounting settlement;
- durable storage;
- production compensation;
- actuation safety.

Source-level tests are authored evidence only. They become execution evidence only after a functioning CI or independent local run produces a verifiable receipt.
