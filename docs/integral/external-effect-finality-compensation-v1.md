# External-effect finality and compensation lineage v1

Status: **ReferenceModelOnly**

D6L preserved one Mycelix semantic effect across provider substitution. D6M closes the next boundary: what happens after a provider operation has been dispatched but before Mycelix can truthfully describe the external consequence as settled.

The central distinctions are:

    provider-reported outcome
    !=
    independently qualified external state

    external finality evidence
    !=
    actuation authorization

    compensation/reversal
    !=
    mutation of the predecessor effect

## External observations

An ExternalEffectObservationV1 binds an observation to:

- the stable Mycelix effect ID and lineage;
- lifecycle generation;
- exact provider route and provider operation;
- provider semantic profile;
- exact provider outcome ID;
- request and idempotency commitments;
- semantic environment;
- observation frontier;
- observed external state;
- evidence source and evidence root.

The observed state can be Unknown, NotApplied, Applied, Reversed, or Contested.

ProviderReported is intentionally a distinct observation source. A provider can report success without that report becoming independent external finality.

## Finality

ExternalFinalityProfileV1 defines the exact effect and semantic environment for which a finality claim is meaningful. It also defines:

- the allowed independent observation sources;
- the required finality state;
- whether current-finality use requires the observation to be at the current Mycelix frontier.

ExternalFinalityReceiptV1 binds the finality claim to the exact observation, provider operation, outcome, route, profile, effect lineage, lifecycle generation, and observation frontier.

The assessment is fail-closed:

- provider-only evidence cannot establish independent finality;
- Unknown or Contested external state remains unresolved;
- NotStarted/RejectedWithoutEffect cannot prove Applied;
- Succeeded cannot prove NotApplied;
- mismatched routes, operations, profiles, effects, environments, or frontiers are rejected;
- a current-finality claim requires exact current-frontier binding;
- a lifecycle tombstone blocks finality for the retired generation.

Historical finality evidence can be retained as historical evidence. It does not become current authority merely because the archive is still available.

## Finality does not authorize execution

The reference model exposes CurrentActuationAuthorization as an explicit blocked purpose.

A finality receipt is evidence about an already-bound effect. It cannot:

- mint a new effect;
- authorize an external operation;
- create authority, consent, or capacity;
- upgrade an archive into current authority;
- replace a current Mycelix semantic transition.

Any external execution remains subject to the existing Mycelix authority, consent, capacity, and consequence-enforcement layers.

## Compensation and reversal

CompensationEffectLinkV1 represents a new semantic effect caused by an observed consequence.

The link binds:

- predecessor effect, lineage, generation, route, and provider operation;
- new compensation effect, lineage, and generation;
- exact causal observation;
- optional exact finality receipt;
- causal provider identity/profile;
- reason: Reversal, Refund, Remediation, or Correction;
- explicit conserved-capacity coverage;
- accounting and link commitments.

The compensation effect must have a distinct effect ID and distinct lineage. This prevents a refund or reversal from becoming a disguised successor or resurrection of the original effect.

The predecessor is passed into the assessment both before and after the compensation transition. They must be byte-for-byte equal in the reference model. A compensation therefore cannot silently rewrite the predecessor's amount, resource, authority, consent, generation, or identity.

## Conservation

Compensation cannot manufacture conserved capacity merely because a new effect exists.

The reference conservation transition requires:

- the predecessor effect remains present;
- exactly one new compensation effect is added;
- existing capacity claim IDs are preserved exactly;
- authority and consent claim sets are not silently removed;
- covered capacity claims are explicit and belong to the predecessor's existing capacity set;
- overlapping coverage for the same predecessor is rejected by the external-effect ledger.

Capacity creation, release, or reallocation remains a separate semantic transition. This model deliberately avoids inventing arithmetic semantics for amounts represented as strings.

## Lifecycle and tombstones

D6I's no-resurrection rule remains active.

A tombstoned predecessor can still be the historical cause of a legitimate compensation, provided the compensation is a genuinely new effect with a new lineage. What cannot happen is:

1. reuse the retired effect ID;
2. reuse the retired lineage/generation as the compensation effect;
3. replay an old provider operation;
4. use the tombstone to manufacture current finality.

Thus a legitimate post-retirement correction remains possible without reviving the retired semantic effect.

## Archive boundary

D6K/D6J reconstruction evidence is explicitly kept separate.

ArchiveRecoveryBindingV1 may identify the historical effect and reconstructed state, but assess_archive_finality_boundary permits only historical use. Current external finality and current actuation authorization are blocked unconditionally.

This remains true even when:

    archive source frontier == current frontier

Equal frontiers establish a matching observation boundary, not a permission boundary.

## Ledger

ExternalEffectLedgerV1 is an in-memory reference ledger for evidence identity.

It records:

- observations by observation ID;
- one terminal finality receipt per effect;
- compensation links by link ID.

It rejects:

- exact observation/finality/compensation duplicates;
- conflicting finality for one effect;
- reuse of a compensation effect ID;
- overlapping compensation coverage for the same predecessor.

The ledger does not claim durable persistence, distributed agreement, or exactly-once external execution.

## Symthaea / Mycelix boundary

Symthaea may:

- correlate provider and external observations;
- detect unresolved outcomes;
- identify possible semantic drift;
- propose finality candidates;
- propose compensation or remediation candidates;
- surface conservation conflicts.

Symthaea may not:

- turn provider success into independent truth;
- invent missing observations;
- promote stale evidence to current finality;
- authorize external actuation;
- mutate predecessor effects;
- resurrect tombstoned generations;
- create conserved capacity;
- select a conflicting finality as authoritative without a Mycelix-qualified transition.

Mycelix remains the semantic root and the authority for qualifying current transitions.

## Reference tests

The module contains 19 source-level tests covering:

- provider success without independent finality;
- accepted independent current finality;
- stale-frontier rejection;
- exact route/operation binding;
- profile drift;
- unresolved external state;
- finality-as-authorization rejection;
- tombstone finality rejection;
- distinct compensation effect and predecessor preservation;
- explicit cause binding;
- predecessor mutation rejection;
- compensation double-counting;
- tombstoned predecessor compensation without resurrection;
- archive finality boundary;
- archive effect mismatch;
- same visible state with distinct effect lineages;
- conflicting finality in the ledger;
- finality receipts not minting effects;
- claim-bounded archive finality witness.

## Claim ceiling

This is deterministic reference semantics only.

It does not establish:

- real-world external state truth;
- provider honesty;
- cryptographic authenticity;
- durable observation;
- exactly-once delivery;
- settlement or legal/accounting finality;
- Byzantine agreement;
- production compensation safety;
- physical-world correctness.

The evidence and commitment roots are opaque fields in this reference model. Source-level tests are authored but are not execution evidence until a functioning CI or independent local run produces a verifiable receipt.
