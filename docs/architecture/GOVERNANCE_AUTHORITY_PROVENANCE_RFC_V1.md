# Governance Authority Provenance RFC v1

Status: architecture candidate / documentation only

Related work:

- Human Agency Kernel HAK-001 / HAK-002 / HAK-003
- Mycelix #292 — proof verification / trusted verifier boundary
- Mycelix #309 — baseline civic standing vs consciousness-derived influence
- Mycelix #317 — exact approved/signed authority before execution
- Mycelix #319 — proposal status transitions require exact decision evidence
- Mycelix #323 — final tally requires closed voting lineage + fixed policy
- Mycelix #325 — ethics disclosures need authenticated provenance before binding effect

## 1. Purpose

Mycelix governance already contains many valuable mechanisms:

- DID/author binding;
- one-agent-one-vote protections;
- proposal lifecycles;
- weighted, Phi-weighted, delegated, and quadratic voting;
- raw vote counts in richer tallies;
- proposal-tier quorum and approval rules;
- absolute quorum floors;
- timelocks;
- threshold-signing infrastructure;
- guardian vetoes;
- concentration / narrow-margin analysis;
- collective-mirror circuit breakers;
- ethics disclosures;
- cross-zome execution.

The current audit found that the largest remaining risk is not absence of mechanisms. It is **authority discontinuity between mechanisms**.

Several boundaries currently accept a representation that looks like the desired state without proving the event that legitimately creates that state.

Examples:

```text
structurally valid proof container
    != trusted verified eligibility claim

valid proposal-status enum transition
    != evidence-backed governance transition

caller-computed tally marked final
    != qualified final decision

proposal ID mentioned in a signed description
    != signature over exact executable proposal/actions

threshold-signing service unavailable
    != permission to execute without signing

caller-supplied ethics verdict
    != governance-binding ethics authority
```

This RFC defines the end-to-end provenance chain future patches should compose into.

It does not introduce runtime types or change governance behavior.

## 2. Core theorem

A governance state is authoritative only when the exact transition into that state is reconstructible from its legitimate source evidence.

```text
StateLabel != AuthorityEvidence
```

More generally:

```text
RepresentationOfAuthority != SourceOfAuthority
```

The system should be able to answer, for every consequential transition:

```text
what exact subject changed?
which governance lineage does it belong to?
what policy allowed the transition?
what evidence proves the policy conditions?
who/what legitimately issued the transition?
what later artifact consumes it?
```

## 3. End-to-end governance authority chain

The intended authority pipeline is:

```text
ProposalDraft
    ↓
ProposalVotingPolicy
    ↓
OpenVotingLineage
    ↓
CanonicalBallots
    ↓
QualifiedVotingClosure
    ↓
QualifiedTally
    ↓
EvidenceBoundProposalTransition
    ↓
ApprovedProposalSubject
    ↓
VerifiedProposalSignature
    ↓
AuthorizedTimelock
    ↓
AuthorizedExecution
    ↓
ExternalSideEffects
    ↓
ExecutionReceipt
    ↓
OutcomeEvidence
```

Supporting analysis flows beside this authority chain rather than silently becoming it:

```text
Consciousness / reputation / expertise / affectedness
    ↓
DeliberativeSignal
    ↓
explicit process policy, if selected
    ↓
ProcessSpecificWeight / review requirement
```

and:

```text
EthicsAssessment
    ↓
AdvisoryEthicsDisclosure
    ↓
explicit trusted ethics-governance policy
    ↓
QualifiedEthicsFinding
    ↓
bounded process restriction
```

## 4. Governance lineage identity

Every proposal decision needs a stable lineage identity that survives representation changes while distinguishing incompatible voting episodes.

Candidate conceptual identity:

```text
GovernanceDecisionLineageV1 {
    community_or_constitution_lineage
    proposal_id
    proposal_version_or_action_hash
    voting_round_or_generation
    voting_policy_id
    voting_policy_version
}
```

The exact representation remains domain-owned.

### 4.1 New voting lineage

Start a new lineage when authority-relevant premises change after voting has begun, including where applicable:

- proposal actions/content;
- proposal version;
- weighting policy;
- quorum policy;
- approval policy;
- electorate/eligibility derivation policy;
- voting window in a way not already authorized by the existing policy.

Do not reinterpret existing ballots under changed authority semantics.

## 5. ProposalVotingPolicyV1 candidate

Before the first ballot, the process should bind the rules that determine what ballots mean.

Conceptual fields:

```text
ProposalVotingPolicyV1 {
    schema_version
    policy_id
    policy_version
    governance_lineage

    proposal_id
    proposal_version_or_action_hash

    eligibility_policy
    weighting_policy
    quorum_policy
    approval_policy

    voting_starts
    voting_ends

    delegation_policy
    tally_policy
    circuit_breaker_policy
    ethics_effect_policy

    integrity_binding
}
```

### 5.1 Weighting policy

Possible explicit modes include:

```text
EqualBaseline
Quadratic
Conviction
PhiExperimental
ReputationExperimental
CompositeExperimental
```

No implicit policy should be inferred merely because a certain vote type or person-level score exists.

### 5.2 Quorum policy must name units

Avoid naked numeric quorum values whose unit is ambiguous.

Candidate semantic forms:

```text
EligibleVoterFraction(f64)
AbsoluteVoterCount(u64)
EligibleVotingMassFraction(f64)
AbsoluteVotingMass(f64)
CompositeQuorum { ... }
```

A percentage must not be compared directly with an absolute weighted sum unless the conversion is explicitly part of the policy.

## 6. OpenVotingLineage

Opening voting should produce or identify the exact decision lineage under which ballots are accepted.

Required properties:

- proposal content/actions frozen or explicitly versioned;
- voting policy frozen;
- voting window known;
- eligibility derivation known;
- weighting semantics known;
- later ballot validation can reconstruct the same policy.

Candidate theorem:

```text
FirstAcceptedBallot -> AuthorityRelevantVotingPolicyFrozen
```

Any later policy change begins a new lineage rather than silently changing earlier ballots.

## 7. Canonical ballots

A ballot should bind at least:

```text
proposal decision lineage
voter identity / privacy-preserving eligibility subject
choice
voting policy identity
weighting inputs or verifier-owned weight result where applicable
timestamp / sequence
integrity provenance
```

### 7.1 Baseline standing

Under `EqualBaseline`:

```text
eligible direct voter -> canonical civic mass 1.0
```

Changing Phi, reputation, stake, participation, or model assessment alone cannot change that baseline mass.

### 7.2 Experimental weighting

If an explicit policy uses assessment/reputation/stake inputs, the ballot or derived weight must retain enough provenance to reproduce the result.

The existence of weighted experiments does not make weighted standing the constitutional default.

## 8. QualifiedVotingClosureV1 candidate

A final decision needs proof that the ballot-accrual phase is closed.

Conceptual fields:

```text
QualifiedVotingClosureV1 {
    schema_version
    closure_id
    governance_lineage
    proposal_id
    proposal_version_or_action_hash
    policy_id
    voting_started
    voting_closed
    closure_reason
    ballot_acceptance_cutoff
    verifier_or_policy_source
    integrity_binding
}
```

Possible legitimate closure sources include:

- deterministic expiry under the precommitted voting window;
- a constitutionally authorized early-close rule;
- an emergency procedure explicitly represented in policy.

A public caller invoking `tally()` is not itself closure evidence.

## 9. QualifiedTallyV1 candidate

A final tally should be verifier-owned and reproducible from the closed lineage.

Conceptual fields:

```text
QualifiedTallyV1 {
    schema_version
    tally_id

    governance_lineage
    proposal_id
    proposal_version_or_action_hash

    closure_id
    voting_policy_id
    voting_policy_version
    eligibility_policy_id
    weighting_policy_id
    quorum_policy_id
    approval_policy_id

    ballot_set_commitment
    ballot_count
    eligible_set_commitment_or_derivation

    raw_for
    raw_against
    raw_abstain

    canonical_for
    canonical_against
    canonical_abstain
    canonical_total

    quorum_reached
    approved

    analytics
    verifier_identity
    integrity_binding
}
```

### 9.1 Analytics are not authority inputs by default

Useful analytics may include:

- Phi provenance / coverage;
- HHI concentration;
- narrow-margin risk;
- turnout distribution;
- delegation concentration;
- expertise/affectedness views;
- collective-mirror diagnostics.

Unless the precommitted policy explicitly says otherwise, those remain analysis outputs rather than authority to change the canonical result.

### 9.2 Advisory tally

A caller may request hypothetical analysis using alternate thresholds or policies.

That output must be typed separately:

```text
AdvisoryTally != QualifiedTally
```

It cannot advance proposal state.

## 10. EvidenceBoundProposalTransitionV1 candidate

Proposal status should be a projection of verified transition evidence, not a freely writable authority label.

Candidate transition subjects:

```text
Draft -> Active
    source: author submission + current proposal eligibility policy

Active -> Ended
    source: QualifiedVotingClosure

Ended -> Approved
    source: QualifiedTally(approved=true)

Ended -> Rejected
    source: QualifiedTally(approved=false)

Approved -> Signed
    source: VerifiedProposalSignature

Signed -> Executed
    source: ExecutionReceipt(success)

Signed -> Failed
    source: ExecutionReceipt(failure)
```

The exact failure/cancellation/veto state machine can remain richer than this simplified chain.

### 10.1 Generic status mutator

A generic operation like:

```text
update_proposal_status(new_status)
```

must not be capable of manufacturing authority-bearing states solely because an enum edge is structurally allowed.

Prefer transition-specific verifier-owned APIs/artifacts.

## 11. ApprovedProposalSubjectV1 candidate

Before signing or timelocking, canonicalize the exact thing governance approved.

Conceptual subject:

```text
ApprovedProposalSubjectV1 {
    governance_lineage
    proposal_id
    proposal_version_or_action_hash
    qualified_tally_id
    canonical_actions_digest
    execution_policy_id
    required_signing_policy_id
    required_timelock_policy_id
}
```

This becomes the object whose digest is signed.

It prevents:

```text
approved proposal A
+ caller-supplied actions B
-> executable timelock
```

## 12. VerifiedProposalSignatureV1 candidate

A threshold signature must prove authorization over the exact approved subject, not merely mention a proposal ID in human-readable text.

Conceptual fields:

```text
VerifiedProposalSignatureV1 {
    approved_subject_digest
    signing_policy_id
    committee_id
    committee_policy_digest
    committee_scope
    threshold
    signer_set
    signature
    verified_at
    verifier_identity
    integrity_binding
}
```

Required theorem:

```text
SignedDescriptionContainsProposalId
    != SignedExactApprovedSubject
```

### 12.1 Committee policy failure

If committee scope/policy is required for authorization:

```text
committee policy unavailable/unknown
    -> NoAuthority
```

not permissive fallback.

## 13. AuthorizedTimelockV1 candidate

A timelock should be constructed from the approved subject, not independent caller-supplied actions.

Conceptual fields:

```text
AuthorizedTimelockV1 {
    governance_lineage
    approved_subject_digest
    verified_signature_id
    canonical_actions_digest
    started_at
    not_before
    timelock_policy_id
    veto_policy_id
    status
    integrity_binding
}
```

Creation requires the exact approved/signed subject and policy.

A `Pending` timelock is not execution authority.

## 14. AuthorizedExecutionV1 candidate

All authority checks should complete before the first external side effect.

Conceptual preflight result:

```text
AuthorizedExecutionV1 {
    governance_lineage
    approved_subject_digest
    timelock_id
    signature_id
    executor_identity
    execution_policy_id
    resource_scope
    canonical_actions_digest
    authorized_at
    expires_at_or_generation
    integrity_binding
}
```

Required theorem:

```text
ExternalSideEffect must not precede CurrentExecutionAuthority
```

### 14.1 Availability

If a required authorization dependency is unavailable:

```text
AuthorizationDependencyUnavailable -> NoAuthority
```

A deployment that intentionally does not require threshold signing must encode that as a distinct explicit execution policy, not infer it from service absence.

## 15. ExecutionReceiptV1 candidate

After execution, record what actually happened.

Conceptual fields:

```text
ExecutionReceiptV1 {
    execution_id
    governance_lineage
    authorized_execution_id
    canonical_actions_digest
    executor_identity
    per_action_results
    external_receipts_or_effect_ids
    started_at
    completed_at
    success_state
    failure_boundary
    integrity_binding
}
```

Proposal `Executed` / `Failed` states should derive from this evidence.

### 15.1 Partial effects

The current action executor can perform multiple cross-zome operations sequentially.

A failure after some actions creates partial external state.

Future execution policy should explicitly choose among:

- transactional execution where supported;
- compensating actions / saga semantics;
- idempotent per-action receipts;
- preflight all resources then commit;
- explicit `PartialSuccess` with recovery workflow.

Do not hide partial effects behind a single Boolean success field.

## 16. OutcomeEvidence

Execution authorization and outcome quality remain distinct.

```text
AuthorizedAction -> bad outcome
```

may indicate bad policy, bad prediction, external change, or implementation failure.

```text
UnauthorizedAction -> good outcome
```

does not become retroactively authorized.

Therefore:

```text
GoodOutcome != RetroactiveAuthorization
BadOutcome  != AutomaticProofOfPriorUnauthorizedness
```

Outcome evidence feeds future policy/science, not historical authority rewriting.

## 17. QualifiedEthicsFindingV1 candidate

Ethics/model analysis should default to advisory status.

```text
EthicsAssessment -> AdvisoryEthicsDisclosure
```

A binding governance effect requires an explicit transformation:

```text
AdvisoryEthicsDisclosure
+ TrustedAssessmentProvenance
+ GovernanceEthicsPolicy
-> QualifiedEthicsFinding
```

Conceptual fields:

```text
QualifiedEthicsFindingV1 {
    governance_lineage
    proposal_id
    proposal_version_or_action_hash
    assessment_model_or_reviewer_identity
    assessment_version
    evidence_or_input_digest
    verdict
    justification_digest
    governance_ethics_policy_id
    issued_at
    expires_at_or_review_generation
    contestability_state
    integrity_binding
}
```

Only a qualified finding can alter canonical process requirements.

## 18. Consciousness / reputation / expertise boundary

The baseline-standing RFC separately proposes:

```text
eligible member -> baseline civic standing
```

Assessment/reputation/expertise signals enter the governance authority chain only through an explicitly selected policy.

Default:

```text
Assessment -> DeliberativeSignal
```

Optional experiment:

```text
DeliberativeSignal
+ ExplicitWeightingPolicy
+ GovernanceLineage
-> ProcessSpecificVoteWeight
```

This preserves governance experimentation without making model assessment an implicit source of universal civic worth.

## 19. Proof verification boundary

Any privacy-preserving eligibility or weighting proof must preserve independent states:

```text
ContainerStructurallyValid
ProofCryptographicallyVerified
VerifierIdentityVerified
VerifierTrustedForPolicy
ClaimCurrent
ClaimBoundToSubject/Proposal/Policy
```

Only the complete policy-qualified result may feed eligibility/weighting.

This is the boundary tracked in #292.

## 20. Authority provenance graph

The resulting pipeline can be viewed as:

```text
Membership / Eligibility Evidence
        │
        ▼
BaselineCivicStanding
        │
        ├──────────────┐
        │              │
        ▼              ▼
Direct Ballot      Explicit Delegation
        │              │
        └──────┬───────┘
               ▼
       Canonical Ballot Set
               │
ProposalVotingPolicy + QualifiedVotingClosure
               │
               ▼
        QualifiedTally
               │
               ▼
EvidenceBoundProposalTransition
               │
               ▼
      ApprovedProposalSubject
               │
               ▼
   VerifiedProposalSignature
               │
               ▼
       AuthorizedTimelock
               │
               ▼
      AuthorizedExecution
               │
               ▼
       ExternalSideEffects
               │
               ▼
        ExecutionReceipt
               │
               ▼
          OutcomeEvidence
```

Advisory channels join only at explicit policy points.

## 21. No authority by availability fallback

This RFC freezes a general governance invariant:

```text
RequiredAuthorityDependencyUnavailable
    -> Hold / Deny / Retry / Escalate
```

never implicitly:

```text
RequiredAuthorityDependencyUnavailable
    -> SkipCheckAndProceed
```

Availability policy can choose a safer degraded mode, but cannot silently widen authority.

## 22. No side effect before authority preflight

Before the first cross-zome/external mutation:

```text
all required authority checks complete
```

This includes:

- proposal subject exact;
- decision lineage current;
- tally qualified;
- signature exact/current/trusted;
- timelock satisfied;
- veto state clear;
- executor/resource scope valid;
- action digest exact.

Then and only then:

```text
execute external action
```

For multi-action execution, define transactional/saga semantics explicitly.

## 23. Preserve strong existing mechanisms

The redesign should reuse, not discard, existing good work where semantically correct:

- proposal author binding;
- voter-author binding;
- one-agent-one-vote links;
- delegation author binding and cycle detection;
- vote immutability;
- raw counts in Phi-weighted tallies;
- Phi provenance categories;
- absolute quorum floors;
- HHI concentration analysis;
- narrow-margin detection;
- collective-mirror diagnostics;
- circuit-breaker concepts;
- timelocks;
- guardian vetoes;
- threshold-signing infrastructure;
- fail-closed cross-zome calls already used in some paths.

The objective is to connect these with exact provenance.

## 24. Avoid derived-status oracles

A consumer should be able to reconstruct why a proposal is `Approved`, `Signed`, or `Executed` from source evidence.

Prefer:

```text
ApprovedStatusProjection(QualifiedTally)
```

over:

```text
stored status == Approved therefore authorized
```

Status may remain useful for indexing/UI, but it should not be the sole authority artifact.

## 25. Candidate cross-artifact bindings

At minimum:

```text
ProposalVotingPolicy
    binds proposal version/actions

QualifiedVotingClosure
    binds policy + proposal lineage

QualifiedTally
    binds closure + ballot set + policy

EvidenceBoundProposalTransition
    binds QualifiedTally

ApprovedProposalSubject
    binds transition/tally + exact actions

VerifiedProposalSignature
    binds ApprovedProposalSubject digest

AuthorizedTimelock
    binds VerifiedProposalSignature + exact actions

AuthorizedExecution
    binds timelock + signature + executor/resource scope

ExecutionReceipt
    binds AuthorizedExecution + actual effects
```

No step should have to trust an unbound human-readable description from the previous step.

## 26. Candidate independent tranches

Keep each authority boundary separately reviewable and qualifiable.

```text
GOV-AUTH-001  governance decision lineage + ProposalVotingPolicy semantics
GOV-AUTH-002  QualifiedVotingClosure
GOV-AUTH-003  QualifiedTally / canonical ballot-set binding
GOV-AUTH-004  evidence-bound proposal transitions
GOV-AUTH-005  canonical ApprovedProposalSubject
GOV-AUTH-006  exact threshold-signature subject + committee policy
GOV-AUTH-007  AuthorizedTimelock construction
GOV-AUTH-008  AuthorizedExecution preflight / remove availability fallbacks
GOV-AUTH-009  ExecutionReceipt + partial-effect semantics
GOV-AUTH-010  QualifiedEthicsFinding / advisory split
GOV-AUTH-011  end-to-end authority-provenance property/integration tests
```

Cross-cutting but independent:

```text
#292       trusted proof-verification path
#309/#312  baseline standing + explicit weighting-policy legitimacy
```

## 27. Dependency recommendation

A safe build order is:

```text
GOV-AUTH-001
    ↓
GOV-AUTH-002
    ↓
GOV-AUTH-003
    ↓
GOV-AUTH-004
    ↓
GOV-AUTH-005
    ↓
GOV-AUTH-006
    ↓
GOV-AUTH-007
    ↓
GOV-AUTH-008
    ↓
GOV-AUTH-009
    ↓
GOV-AUTH-011
```

`GOV-AUTH-010` can proceed alongside 001-004 once proposal lineage identity is defined.

#292 must be resolved before any proof-bearing cognition/eligibility path is relied upon.

#309/#312 can evolve in parallel because it asks what should establish baseline civic legitimacy, not whether evidence artifacts are cryptographically valid.

## 28. End-to-end properties

### GOV-AUTH-P1 — no caller-created final authority

No public caller can produce a final tally/Approved/Signed/Executed authority state merely by supplying fields that satisfy structural validation.

### GOV-AUTH-P2 — exact proposal subject

Every authority artifact after proposal closure binds the exact proposal version/action subject.

### GOV-AUTH-P3 — policy immutability

Once ballots begin, authority-relevant voting policy cannot change within that voting lineage.

### GOV-AUTH-P4 — closure before finality

No final tally exists before qualified voting closure.

### GOV-AUTH-P5 — tally reproducibility

A qualified tally is reproducible from the committed ballot set + fixed policy.

### GOV-AUTH-P6 — exact signature subject

Signature over subject A cannot authorize subject B, including changed actions/version/tally lineage.

### GOV-AUTH-P7 — availability non-authority

Loss of verifier/signing/policy infrastructure cannot widen executable authority.

### GOV-AUTH-P8 — no pre-authority side effects

No external effect occurs before AuthorizedExecution exists.

### GOV-AUTH-P9 — receipt binding

Executed/Failed status is derivable from the exact execution receipt.

### GOV-AUTH-P10 — advisory non-authority

Changing only an advisory ethics/cognition/reputation signal cannot change canonical authority unless a precommitted policy explicitly consumes that signal.

### GOV-AUTH-P11 — delegation conservation

Under equal-baseline governance, delegation cannot create more civic mass than the delegator possessed.

### GOV-AUTH-P12 — weighted transparency

Every weighted qualified tally retains raw unweighted counts.

## 29. Machine-readable future audit manifest

HAK-003 proposes an audit manifest rather than a runtime oracle.

Governance could eventually record entries such as:

```text
AuthorityTransformationAuditV1 {
    transformation_kind
    governance_lineage
    source_artifact
    destination_artifact
    policy_id
    legitimacy_source
    disposition
    evidence_dependencies
}
```

This should help tooling locate broken edges without becoming the source of authority itself.

## 30. Non-claims

This RFC does not claim:

- every current governance feature is unsafe;
- weighted voting should be deleted;
- all governance must use equal voting;
- all ethics findings must be human-authored;
- all external calls can be globally transactional;
- Holochain's integrity model is insufficient;
- every status field should disappear;
- a universal HAK runtime type should replace domain-owned governance types;
- current Mycelix governance is deployed as a sovereign public government.

The narrower conclusion is:

> Governance authority should be reconstructible as an exact provenance chain from legitimate source evidence to bounded execution, with no semantic step supplied merely by a caller, fallback, status label, or unverified model output.

## 31. Review gate

Before runtime implementation, reviewers should answer:

1. What exact fields define a governance decision lineage?
2. Which voting-policy fields must freeze before the first ballot?
3. What is the canonical electorate/eligibility-set derivation?
4. Which tally analytics are advisory vs policy-binding?
5. What exact digest should threshold signatures cover?
6. Which committee-scope policies must fail closed?
7. What transactional/saga semantics are required for multi-action execution?
8. Which ethics effects are advisory vs binding?
9. What historical migration marker identifies legacy implicit tallies/statuses?
10. Which artifacts need durable replay/generation barriers?

Until those questions are reviewed:

```text
architecture only
no claim of qualification
no silent reinterpretation of historical records
```
