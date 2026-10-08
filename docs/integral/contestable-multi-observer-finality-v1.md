# Contestable multi-observer external finality v1

Status: **ReferenceModelOnly**

D6M establishes a boundary between provider-reported outcomes and independently qualified external finality. D6N adds the evidence-contest layer needed when more than one observation exists.

The central distinction is:

    multiple observations != automatic truth
    observer agreement != semantic authority
    conflict resolution != winner selection

## Observer identity and independence

\`ExternalObserverProfileV1\` binds observer identity to:

- observation method;
- provider relationship;
- evidence root;
- custody root;
- upstream observer dependencies;
- upstream evidence dependencies;
- semantic environment;
- observation profile;
- explicit independence declaration;
- independence commitment.

Identity alone never proves independence.

Two observers sharing an evidence root, custody root, upstream observer, or upstream evidence root are treated as dependent for the reference model. This prevents copied evidence from being counted repeatedly.

Provider-reported observations remain subject to an explicit qualification-profile switch. The conservative default is that provider reports do not satisfy an independent-observer threshold.

Archive mirrors and derived observers are not independent merely because they have distinct observer IDs.

## Observation sets

\`ExternalObservationSetV1\` binds all observations to:

- the exact Mycelix effect;
- lineage and lifecycle generation;
- provider route;
- provider identity and operation;
- provider semantic profile;
- semantic environment;
- observation frontier;
- qualification profile;
- target finality state.

An observation concerning another effect, route, operation, lifecycle generation, provider profile, or semantic environment is incomparable and cannot settle the target.

## Evidence classifications

The reference model preserves:

- CorroboratingIndependent;
- CorroboratingDependent;
- ContradictoryIndependent;
- ContradictoryDependent;
- Stale;
- Superseded;
- Incomparable;
- InsufficientEvidence.

Unknown or contested external state remains insufficient evidence.

A contradictory independent observation makes the set contested. A large number of correlated observations cannot override an independence threshold.

## Qualification

\`FinalityQualificationProfileV1\` explicitly defines:

- allowed observation sources;
- required independent-observation count;
- current-frontier requirement;
- whether provider reports may satisfy independence;
- whether explicit conflict resolution is permitted;
- semantic environment and profile commitment.

The reference model never interprets observer count as a standalone truth criterion.

A set becomes qualified only when the declared profile's independence threshold is met and no unresolved independent contradiction remains.

## Conflict resolution

\`FinalityResolutionReceiptV1\` is a new semantic evidence transition. It binds:

- target effect/lineage/generation;
- route/provider/provider operation;
- provider profile;
- semantic environment;
- observation-set identity and commitment;
- qualification profile;
- resolved external state;
- required independence threshold;
- observation frontier;
- resolver identity;
- explicit qualification transition;
- resolution commitment.

The resolver cannot be one of the observations being used as its own independent evidence.

Resolution never rewrites the underlying observations. Contradictory evidence remains part of the historical evidence set.

A conflict may be resolved only when the qualification profile explicitly permits conflict resolution and the resolution contains an explicit qualification transition.

## No last-write-wins

The assessment is invariant to delivery order.

It does not choose a result based on:

- newest timestamp;
- latest arrival;
- lexicographic observer ID;
- provider preference;
- observer count alone;
- archive availability;
- Symthaea confidence.

This is intentional: semantic authority must come from an explicit Mycelix-qualified transition, not from a convenient ordering heuristic.

## Currentness

For current-finality profiles:

    resolution profile == target profile
    environment == target environment
    observation frontier == current frontier
    target generation == live generation

A stale resolution may remain historical evidence but cannot become current finality.

A tombstoned generation cannot acquire current finality through later observer evidence.

## Archive boundary

Archive evidence remains historical.

\`archive_resolution_is_historical_only\` prevents an archived resolution from being treated as a current semantic authority. Matching the current frontier does not change this boundary.

This preserves D6J/D6K:

    reconstructable history != current authority

## Actuation boundary

A finality resolution is evidence about an already-bound external effect.

It cannot:

- create a new effect;
- authorize a new actuation;
- mint authority;
- mint consent;
- mint capacity;
- change the predecessor effect;
- resurrect a retired lifecycle generation.

External execution remains subject to Mycelix's existing authority, consent, capacity, and consequence-enforcement layers.

## Compensation boundary

D6M compensation can reference a qualified finality resolution without rewriting the predecessor.

A contested or insufficient resolution cannot be used as the causal basis for compensation qualification.

The compensation remains a distinct semantic effect with its own identity and lifecycle.

## Evidence ledger

\`ObserverEvidenceLedgerV1\` tracks:

- observer profiles;
- observed evidence;
- observation sets;
- finality resolutions;
- terminal resolution per effect;
- contests.

The ledger rejects divergent duplicate identities and incompatible terminal resolutions.

It is an in-memory reference ledger only.

## Symthaea / Mycelix boundary

Symthaea may:

- correlate observations;
- detect dependence and conflict;
- classify evidence;
- propose a reconciliation candidate;
- surface unresolved evidence;
- recommend a qualification transition.

Symthaea may not:

- manufacture independence;
- turn observer count into authority;
- choose a conflict winner;
- promote stale evidence to currentness;
- turn an archive into current authority;
- authorize actuation;
- mutate predecessor history;
- resurrect a tombstoned generation.

Mycelix remains the semantic root and qualifies the transition.

## Reference tests

The module contains **27 source-level tests** covering:

- genuinely independent corroboration;
- provider self-report exclusion from independent thresholds;
- shared evidence-root dependence;
- shared custody-root dependence;
- independent contradiction;
- correlated contradiction;
- stale frontier;
- wrong effect binding;
- wrong provider operation;
- tombstoned generation;
- explicit conflict resolution;
- resolver/observer separation;
- arrival-order invariance;
- actuation boundary;
- archive historical boundary;
- compensation qualification;
- terminal-resolution conflict;
- divergent observer identity;
- Symthaea non-authority;
- equal visible state with distinct evidence roots;
- archive-mirror dependence;
- unknown independence;
- upstream dependency;
- malformed resolution;
- current-frontier requirement;
- explicit conflict-transition requirement.

## Claim ceiling

This is deterministic reference semantics only.

It does not establish:

- real-world observer independence;
- physical external truth;
- provider honesty;
- cryptographic authenticity;
- durable observation;
- statistical confidence;
- legal or accounting settlement;
- Byzantine consensus;
- production finality;
- production compensation safety;
- physical-world safety.

Evidence, custody, and commitment roots are opaque reference-model fields.

Source-level tests are authored but are not execution evidence until a functioning CI or independent local run produces a verifiable receipt.
