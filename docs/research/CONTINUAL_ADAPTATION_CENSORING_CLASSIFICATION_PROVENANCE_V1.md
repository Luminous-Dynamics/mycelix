# Censoring Classification Provenance Research v1

Status: research specification and dependency-light fixture model only.

This seam extends #4667 and #4696 by making the censoring/selection classification itself claim-local, content-bound, temporally ordered evidence. It does not create a second qualification authority.

## Why this is the next causal boundary

Target-trial methodology emphasizes alignment of eligibility, treatment assignment, and follow-up at time zero, because selection and misclassification can create immortal-time and other design biases. More recent longitudinal target-trial work also treats loss to follow-up, artificial censoring, and observation processes as explicit components of the design and analysis. These controls can prevent self-inflicted design bias, but they do not by themselves identify an underlying causal model.

Research references:

- Hernan & Robins: https://pmc.ncbi.nlm.nih.gov/articles/PMC5124536/
- Target-trial framework: https://pmc.ncbi.nlm.nih.gov/articles/11936718/
- Operational TTE framework: https://pmc.ncbi.nlm.nih.gov/articles/PMC13230876/
- Dynamic-regime censoring/IPW: https://pubmed.ncbi.nlm.nih.gov/16611197/

The preceding #4696 seam therefore records censoring class, basis, and timing. The remaining attack is semantic provenance:

    label-only classification
        <
    immutable Classification C1
      -> classifies Attempt A1
      -> frozen_by Policy P1
      -> supported_by Basis B1
      -> content commitment C1
      -> exact policy blob identity P1
      -> temporal ordering before outcome

## Evidence model

A CensoringClassification is a graph-addressable evidence object with:

- exactly one attempted episode target;
- exactly one classification-policy provenance edge;
- exactly one classification-basis provenance edge;
- an explicit censoring class;
- classification and policy-freeze epochs;
- the exact policy Git blob identity;
- a revision number;
- a content commitment over the semantic identity fields.

The current fixture dialect uses explicit relations:

    classifies
    frozen_by
    supported_by
    supersedes
    invalidated_by

The classification graph is reachable from the claim through:

    Claim
      -> AttemptCensus
      -> Attempt
      -> CensoringClassification
      -> Policy / Basis / lineage

Unrelated graph growth is not allowed to strengthen a claim or satisfy a missing classification requirement.

## Content binding

The classification commitment covers:

    attempt_id
    censoring_reason
    classification_epoch
    frozen_epoch
    policy_blob_sha
    basis_id
    revision

The verifier recomputes this commitment rather than trusting the declared value.

Base-revision history is additionally anchored by an immutable fixture-level commitment. Thus changing the historical class, basis, or policy identity without creating a new revision fails closed.

## Temporal boundary

The fixture uses a frozen logical epoch order:

    t0 < t1 < t2

The policy must be frozen before the episode outcome.

The classification record must occur at or before the outcome.

These are deliberately separate conditions:

    policy freeze time
    !=
    classification-record time

A later reclassification cannot rewrite an earlier claim merely by changing a current field. A genuine replacement is a new classification entity linked with `supersedes` and a new content commitment.

The verifier does not claim that wall-clock timestamps are trustworthy. It consumes logical temporal provenance within this restricted research dialect and composes with the existing trusted-time/currentness machinery.

## Revision and supersession

Revision `0` is a base classification and is externally anchored.

Revision `n > 0` must explicitly supersede revision `n-1`.

The superseding classification must classify the same attempt.

The policy rejects:

- revision without supersession;
- skipped revision numbers;
- multiple predecessor edges;
- supersession cycles;
- multiple active revisions for one attempt;
- supersession recorded after the outcome.

Historical entities remain immutable. A correction creates a new lineage rather than rewriting an old node.

This follows the same general provenance principle used by W3C PROV: revisions are represented as distinct entities and linked through derivation/revision relationships; provenance validity includes consistency constraints over the resulting history.

## Anti-self-evidence boundary

A classification cannot use a `Result` as its supporting evidence.

This blocks the circular pattern:

    result
      -> favorable claim
      -> classification of censoring
      -> result

The policy-liveness campaign deliberately disables this rule and checks that the expected verdict transition occurs. The resulting mutation is therefore a live test of the policy semantics rather than a prose-only assertion.

## Current fixed attack corpus

The fixed fixture covers 22 cases across:

1. valid claim-local classification;
2. class mismatch;
3. late policy freeze;
4. late classification record;
5. policy identity substitution;
6. missing policy provenance;
7. missing basis provenance;
8. duplicate classification edge;
9. one classification reused across attempts;
10. result-backed classification;
11. forged commitment;
12. valid superseding revision;
13. post-outcome supersession;
14. result-backed superseding revision;
15. classification hidden from the claim-local projection;
16. active classification invalidation;
17. history rewrite;
18. isolated late policy-freeze revision;
19. isolated late-recording revision;
20. nonzero revision without supersession;
21. restricted-dialect non-ASCII input;
22. revision-level class mismatch.

The ordering of these checks deliberately distinguishes content forgery from missing/ambiguous provenance:

    forged/incompatible content -> unqualified
    missing/late/ambiguous proof -> unresolved

Neither state is treated as a positive qualification result.

## Generated campaign

`generate_censoring_classification_properties.py` creates 128 deterministic mutations from seed `0x43505601` across:

- representation invariance;
- content/identity binding;
- structural and claim-local projection rejection;
- revision/supersession integrity;
- anti-self-evidence;
- compositional attacks.

The generated corpus is regenerated twice in CI and compared byte-for-byte before either reference evaluator is run.

This makes deterministic construction itself part of the research evidence rather than assuming that a generator is correct because it is deterministic in one run.

## Differential implementation

Two dependency-light verifiers are maintained:

- `verify_censoring_classification.py`;
- `verify_censoring_classification.mjs`.

Both implement the restricted fixture dialect independently.

CI compares the complete serialized reports, not only the verdict counts.

During hardening, the cross-language campaign exposed the same kind of subtle failure previously found in the main evidence kernel: locale-dependent ordering was unsuitable for evidence identity. The Node verifier now uses explicit lexical ordering and rejects non-ASCII input in this restricted dialect.

## Policy liveness

`CONTINUAL_ADAPTATION_CENSORING_CLASSIFICATION_POLICY_LIVENESS_FIXTURES.json` contains six one-rule mutations:

- class/attempt reason compatibility;
- result-as-support prohibition;
- claim-local active-classification requirement;
- historical immutability anchor;
- classification-before-outcome requirement;
- supersession requirement for nonzero revisions.

For policy mutations, the liveness harness rebinds the temporary policy blob and recomputes the affected classification commitments. Historical anchors are only rebound when the policy itself legitimately changes; a dedicated history mutation preserves the original anchor. This separation prevents policy change from being mistaken for historical rewriting.

## Claim ceiling

A green result establishes only that this synthetic fixture dialect detects the specified classification-provenance attacks.

It does not establish:

- that a real-world censoring class is causally correct;
- validity of any censoring-adjustment model;
- valid inverse-probability weights;
- absence of unmeasured confounding;
- physical efficacy;
- deployment safety;
- cognition, consciousness, or autonomous authority.

No production behavior or autonomous update authority is introduced.

## Composition

This seam composes with:

- #4570 action-dependent observation/distribution shift;
- #4571 attempt completeness and negative evidence;
- #4599 measurement invariance;
- #4617 claim-local evidence ledger;
- #4618 evidence dependence/shared ancestry;
- #4652 policy identity binding;
- #4661 policy liveness;
- #4696 selection/survival/informative censoring.

The resulting boundary is:

    attempt census
      +
    selection/censoring integrity
      +
    classification provenance
      +
    estimand identity
      !=
    automatic causal truth

## Omission closure follow-up

Follow-up issue #4740 identified two omission classes not covered by the original corpus:

1. an immutable base anchor could disappear without being checked, allowing a replacement revision-0 classification to become active;
2. claim-local projection could begin from an implicit string root even when no Claim entity existed.

The closure hardening now requires an actual Claim root plus its claim-to-AttemptCensus entry edge, requires every declared base anchor to be present as the revision-0 classification it anchors, and adds fixed/generated node-omission mutations.

The research distinction remains:

    missing provenance -> unresolved
    forged/incompatible provenance -> unqualified

A green result still establishes only bounded synthetic verifier behaviour.
