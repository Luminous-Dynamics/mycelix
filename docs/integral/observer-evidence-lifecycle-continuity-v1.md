# Observer and evidence lifecycle continuity v1

Status: **ReferenceModelOnly**

D6N establishes that an observer's identity does not prove independence and that copied/dependent evidence cannot be counted as independent corroboration. D6O adds the missing temporal layer.

The central distinction is:

    independent at frontier F1
        !=
    independent forever

Observer status, semantic profile, dependency roots, custody, and observer generations can change. Those changes must affect future qualification without rewriting historical observations.

## Design boundary

D6O remains a small deterministic reference model.

It uses:

- logical frontier sequences and roots;
- immutable generation records;
- append-only lifecycle transitions;
- immutable dependency snapshots;
- explicit predecessor/successor rotation certificates;
- D6N ObservationClassificationV1;
- evidence eligibility receipts.

It does not use wall-clock time as authority and does not implement:

- revocation infrastructure;
- cryptographic key verification;
- physical observer trust;
- Byzantine consensus;
- durable storage guarantees;
- production finality;
- actuation.

The architectural boundary remains:

    Mycelix  = semantic root
    Symthaea = analysis / proposal
    Xenia    = cryptographic mechanism / verification

## Observer generations

ObserverGenerationV1 makes the semantic basis of an observer immutable for one generation.

A generation binds:

- observer identity;
- logical generation sequence;
- predecessor generation, if any;
- observer role;
- observation method;
- provider relationship;
- evidence root;
- custody root;
- upstream observer/evidence dependencies;
- semantic environment;
- observation profile;
- creation frontier;
- initial status;
- generation commitment.

A later semantic-basis change is not a mutation of the old generation.

Instead:

    observer-A / generation-1
             |
             | qualified rotation
             v
    observer-A / generation-2

The same observer ID is therefore insufficient to select the current generation.

Generation sequence numbers are logical identifiers, not a currentness oracle. Current continuity follows explicit predecessor/successor links and rotation certificates rather than "largest generation wins".

## Lifecycle status

ObserverStatusV1 contains:

- Active
- Suspended
- Revoked
- Retired
- Superseded

Only Active is currently eligible.

Status changes are append-only ObserverStatusTransitionV1 records. A transition binds:

- transition identity;
- observer identity;
- predecessor generation;
- successor generation for supersession;
- prior status;
- new status;
- effective logical frontier root/sequence;
- the generation's fixed semantic profile/dependency roots;
- reason;
- qualification transition;
- transition commitment.

A transition cannot return a generation to Active. Reactivation is represented by a new generation.

This prevents a historical generation from regaining present eligibility by mutating one status flag.

## Causal completeness and arrival order

Lifecycle transitions are ordered by their logical frontier sequence, not delivery order.

A later transition may arrive before its predecessor. It can be retained as temporarily incomplete when its status edge is intrinsically possible.

For example:

    Active --F2--> Suspended --F3--> Revoked

If the F3 transition arrives first:

- it may be retained;
- status at F3 remains unknown/incomplete;
- currentness cannot be derived;
- once F2 arrives, both ledgers converge to Revoked.

Impossible edges such as Revoked -> Suspended are rejected immediately.

Two distinct transitions at the same logical frontier are conflicts. Neither wins because it arrived later.

status_at_frontier returns no state until the transition chain to that frontier is causally complete.

## Dependency snapshots

D6N detects correlation using evidence, custody, and upstream dependency roots. D6O makes those dependencies temporal with EvidenceDependencySnapshotV1.

A snapshot binds:

- observer generation;
- observation profile;
- semantic environment;
- evidence root;
- custody root;
- upstream observer IDs;
- upstream evidence roots;
- independence declaration;
- effective logical frontier;
- predecessor snapshot;
- snapshot commitment.

Dependency changes do not mutate a generation.

For example:

    F1: evidence-root-A, independent
    F2: evidence-root-B, newly shared dependency

Evidence at F1 retains its original evidence identity. Evidence at F2 is evaluated against the F2 dependency snapshot and can no longer silently inherit the F1 independence basis.

A later dependency snapshot may arrive before its predecessor. It is retained but remains unavailable to eligibility queries until its snapshot chain is complete.

## Rotation continuity

ObserverRotationCertificateV1 is the explicit bridge between observer generations.

A valid certificate binds:

- exact predecessor generation;
- exact successor generation;
- same observer identity;
- predecessor/successor environment roots;
- predecessor/successor observation profiles;
- predecessor/successor evidence roots;
- predecessor supersession transition;
- exact effective frontier root/sequence;
- qualification transition;
- continuity root;
- certificate commitment.

The successor's creation frontier must equal the predecessor supersession transition frontier exactly.

This means:

    same observer ID
        !=
    same observer generation
        !=
    semantic continuity

A profile/environment/dependency/key-root change without an explicit qualified rotation cannot create current continuity.

A rotation certificate does not itself authorize actuation or mint authority/capacity/consent.

## Evidence eligibility

EvidenceEligibilityReceiptV1 is an evidence join, not an execution permit.

It binds:

- observation identity;
- observer identity;
- exact observer generation;
- observation profile;
- semantic environment;
- dependency snapshot;
- observation frontier;
- current frontier;
- current generation where known;
- qualification profile;
- live/archive provenance;
- D6N observation classification;
- lifecycle transition references;
- eligibility disposition;
- commitment.

Eligibility is evaluated against the lifecycle ledger rather than accepting a caller-supplied current flag.

The reference model distinguishes:

- EligibleCurrent
- HistoricalOnly
- BlockedLifecycle
- BlockedProfile
- BlockedDependency
- BlockedContinuity
- BlockedCurrentness
- BlockedArchive
- Contested
- InsufficientEvidence
- BlockedAuthorization

## Historical preservation

A valid observation is not retroactively erased when its observer is later suspended, revoked, retired, or superseded.

For historical use:

    eligible at observation frontier
        +
    later lifecycle change
        ->
    HistoricalOnly

For current-finality reuse:

    historical eligibility
        !=
    current eligibility

Current reuse requires:

- the observer generation to remain the explicit continuous generation;
- the observer to be Active at the current frontier;
- the dependency snapshot to remain current;
- the semantic profile/environment to match;
- the requested frontier to be current.

If any of these conditions no longer hold, current eligibility is conservatively blocked.

## D6N composition

D6O does not introduce a second conflict taxonomy.

It consumes D6N ObservationClassificationV1:

- CorroboratingIndependent can proceed to lifecycle checks;
- CorroboratingDependent is dependency-blocked;
- ContradictoryIndependent remains contested;
- ContradictoryDependent remains contested;
- Stale is currentness-blocked;
- Superseded is lifecycle-blocked;
- Incomparable and InsufficientEvidence remain insufficient.

Thus:

    D6N = what the observation says about evidence relationship
    D6O = whether that evidence remains lifecycle-eligible now

Neither layer manufactures semantic authority.

## Archive boundary

Archived lifecycle/evidence material is historical or reconstruction input.

An archived eligibility observation cannot become current merely because:

- its contents are complete;
- its frontier equals the current frontier;
- its generation ID is still known;
- its observer ID is still present.

Archive provenance is explicit and current-finality use is blocked.

## Symthaea boundary

Symthaea may:

- detect observer lifecycle anomalies;
- compare generations;
- correlate dependency changes;
- identify possible rotation paths;
- propose requalification;
- surface historical/current discrepancies.

Symthaea may not:

- choose the current generation by numeric ID;
- invent a missing rotation certificate;
- manufacture observer independence;
- mutate a generation's semantic profile;
- turn a lifecycle proposal into an authoritative transition;
- promote historical eligibility into current eligibility;
- authorize actuation;
- mint authority, consent, or capacity.

Only a qualified Mycelix lifecycle transition can establish the modeled continuity.

## Non-authority helpers

lifecycle_proposal_is_authoritative returns false by construction.

lifecycle_transition_can_authorize_actuation returns false.

lifecycle_transition_can_mint_authority_capacity_or_consent returns false.

These helpers are intentionally explicit so downstream code cannot mistake lifecycle evidence for an authorization primitive.

## Adversarial corpus

The module contains source-level tests for:

1. active observer eligibility;
2. revocation before an observation frontier;
3. suspension before a new observation;
4. historical preservation after revocation;
5. refusal to reuse historical evidence for current qualification;
6. explicit rotation continuity;
7. rotation without continuity evidence;
8. same-ID successor not silently current;
9. dependency-root change;
10. semantic-environment rotation binding;
11. observation-profile rotation binding;
12. provider-role mismatch;
13. archive-mirror boundary;
14. reactivated-generation no-resurrection;
15. exact generation binding in eligibility receipts;
16. stale lifecycle state;
17. currentness revalidation after revocation;
18. D6N contradiction preservation;
19. divergent lifecycle transitions;
20. divergent duplicate transition identity;
21. out-of-order lifecycle transition delivery;
22. non-authoritative Symthaea proposal;
23. actuation boundary;
24. authority/capacity/consent boundary;
25. archived lifecycle evidence;
26. out-of-order dependency snapshots;
27. impossible status regression.

The source corpus is deterministic and contains no wall-clock or network dependency.

## Claim ceiling

**ReferenceModelOnly.**

The implementation demonstrates only deterministic reference semantics under the exact modeled inputs.

It does not establish:

- real-world observer independence;
- key authenticity or revocation infrastructure;
- physical observation correctness;
- legal or accounting authority;
- statistical confidence;
- Byzantine fault tolerance;
- distributed consensus;
- durable storage;
- production external finality;
- production compensation;
- actuation safety.

Source-level tests are authored evidence only. They become execution evidence only after a functioning CI or independent local run produces a verifiable receipt.
