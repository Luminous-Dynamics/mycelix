# Governance Authority Conservation Laws v1

Status: architecture candidate / documentation only

Companion to:

- `GOVERNANCE_AUTHORITY_PROVENANCE_RFC_V1.md`
- Mycelix #309 / PR #312 — baseline civic standing
- Mycelix #323 — qualified tally
- Mycelix #319 — evidence-bound proposal transitions
- Mycelix #317 — exact execution authority
- Mycelix #325 — qualified ethics findings
- Mycelix #330 — voice-credit issuance and monotonic spend
- Mycelix #331 — delegated civic-mass conservation
- Mycelix #332 — delegated ballot lineage / uniqueness

## 1. Purpose

Authority provenance alone is not enough.

A system can preserve provenance perfectly and still accidentally multiply authority while transforming it.

Examples:

```text
one voter
-> two overlapping 60% delegations
-> 120% delegated civic mass
```

```text
one voice-credit budget
-> two concurrent/forked spends
-> two ballots each claiming the same credits
```

```text
one civic unit
-> direct vote + delegated vote
-> same source mass exercised twice
```

```text
one authorization grant
-> compatibility fallback
-> broader execution authority
```

This document defines candidate conservation laws for Mycelix governance.

The laws are architectural constraints, not yet runtime types or qualified implementation claims.

## 2. Core theorem

For any authority-preserving transformation, output authority must not exceed the legitimate source authority available to that transformation.

```text
AuthorityOut <= AuthorizedSourceBudget
```

Where a transformation legitimately creates new authority, it must name a new legitimacy source and a new lineage.

```text
AuthorityOut > AuthorityIn
    requires
NewLegitimacySource + ExplicitPolicy + NewAuthorityLineage
```

Therefore:

```text
RepresentationChange != AuthorityMint
Routing != AuthorityMint
Delegation != AuthorityMint
Fallback != AuthorityMint
Retry != AuthorityMint
Fork != AuthorityMint
Replay != AuthorityMint
```

## 3. Conservation is scope-aware

Authority is rarely a single global scalar.

A conservation check must be evaluated against an exact scope.

Candidate scope identity:

```text
AuthorityScopeV1 {
    constitutional_or_community_lineage
    governance_decision_lineage
    proposal_or_domain
    policy_mode
    time_or_generation
    resource_or_subject
}
```

Two grants may each validly use 100% of a source if they are provably non-overlapping.

For example:

```text
100% delegation for Energy proposals
100% delegation for Music proposals
```

is not necessarily amplification if the same decision can never satisfy both scopes.

But:

```text
60% delegation for all proposals
60% delegation for Constitutional proposals
```

must be treated as overlapping for a Constitutional decision.

Conservation therefore requires **scope intersection**, not merely summing every grant globally.

## 4. Baseline civic mass

For equal-baseline governance, normalize one eligible person's baseline direct civic mass as:

```text
BaselineCivicMass(person, lineage) = 1.0
```

This is not a claim that every governance mechanism must use equal weighting.

It is the source budget for the equal-baseline layer proposed by #309 / PR #312.

A process-specific weighting policy may transform that mass into another influence measure, but it must not silently redefine the baseline source itself.

```text
BaselineCivicMass
+ ExplicitWeightingPolicy
-> ProcessSpecificInfluence
```

not:

```text
ProcessSpecificInfluence
-> retroactive redefinition of BaselineCivicMass
```

## 5. Delegated civic-mass conservation

For a person `p`, governance scope `s`, and time/generation `t`:

```text
RetainedMass(p,s,t)
+ Sum(EffectiveOutgoingDelegatedMass(p,s,t))
<= BaselineCivicMass(p,s,t)
```

Under a normalized equal-baseline source:

```text
retained + delegated <= 1.0
```

### 5.1 Decay

Delegation decay may reduce authority but must never increase it.

```text
MassAfterDecay <= MassBeforeDecay
```

### 5.2 Transitivity

Every transitive edge must be attenuating or neutral relative to its parent source.

```text
ChildRoutedMass <= ParentAvailableMass
```

The sum of child branches from one parent must also remain within that parent budget.

```text
Sum(ChildBranchMass) <= ParentAvailableMass
```

### 5.3 Cycles

Cycle detection is necessary but not sufficient.

A valid cycle rule must satisfy:

```text
CycleTraversal cannot increase total source mass
```

The safest default is for cyclic authority paths to contribute zero unless a domain-specific policy defines a deterministic non-amplifying resolution.

### 5.4 Fan-out

Multiple outgoing delegates must reserve from one shared source budget.

A per-edge rule:

```text
0 < percentage <= 1
```

is insufficient.

Conservation is a property of the **set of overlapping effective grants**, not each grant independently.

## 6. Direct-vote / delegated-vote conservation

The same baseline civic unit must not be exercised both directly and through delegation in one decision lineage.

Required theorem:

```text
DirectExercisedMass
+ DelegatedExercisedMass
<= SourceCivicMass
```

Possible explicit policy models include:

### 6.1 Reserved delegation

```text
active delegated fraction
-> unavailable for direct exercise
```

### 6.2 Vote-time split

```text
direct ballot exercises retained fraction only
```

### 6.3 Direct-vote supersession

```text
direct vote
-> revokes/supersedes applicable delegation before ballot qualification
```

Any model is acceptable if explicit and reproducible.

Implicit double use is not.

## 7. Ballot uniqueness conservation

Authentication and ballot uniqueness are distinct properties.

```text
AuthenticatedVoter != UniqueBallot
```

For a source civic unit `u` and voting lineage `L`:

```text
Count(QualifiedExercises(u,L)) <= policy_allowed_exercises(u,L)
```

For ordinary one-ballot governance:

```text
Count(QualifiedExercises(u,L)) <= 1
```

Retries, reconnections, duplicated links, forked coordinator calls, or alternate vote endpoints must not increase this count.

### 7.1 One canonical admission key

Direct, delegated, quadratic, verified/ZK, and other ballot modes should converge on a canonical admission identity sufficient to prevent cross-endpoint double exercise.

Conceptually:

```text
BallotExerciseKeyV1 {
    governance_lineage
    source_civic_identity_or_private_commitment
    source_authority_lineage
    exercise_class
}
```

The exact privacy representation remains domain-owned.

## 8. Voice-credit issuance conservation

Quadratic voting uses a separately minted budget.

Therefore voice-credit issuance needs its own legitimacy source.

```text
VoiceCreditPolicy
+ QualifiedIssuerAuthority
+ AllocationEpoch
-> VoiceCreditGrant
```

For a grant `g`:

```text
Allocated(g) = Spent(g,t) + Remaining(g,t)
```

at every valid state.

Across time:

```text
Spent(g,t+1) >= Spent(g,t)
Remaining(g,t+1) <= Remaining(g,t)
Allocated(g,t+1) = Allocated(g,t)
```

unless a new authorized grant lineage is created.

### 8.1 No implicit refill

An update, retry, alternate branch, expiry transition, or compatibility path must not refill a grant.

```text
UpdateExistingGrant != NewAllocationAuthority
```

### 8.2 Allocation-epoch uniqueness

If policy grants one budget per epoch:

```text
EffectiveGrantCount(owner, epoch, policy_lineage) <= 1
```

unless the policy explicitly defines additive grants.

## 9. Voice-credit spend conservation

A quadratic ballot should not merely declare a number of credits spent.

The spend must be tied to an exact source grant lineage.

Candidate flow:

```text
QualifiedVoiceCreditGrant
-> VoiceCreditSpendReceipt
-> QualifiedQuadraticBallot
```

For each spend receipt:

```text
AmountSpent <= PriorRemaining
NewRemaining = PriorRemaining - AmountSpent
NewSpent = PriorSpent + AmountSpent
```

### 9.1 Fork/concurrency rule

Two concurrent spends must not each consume the same source remaining mass.

Conceptually:

```text
Sum(CanonicalChildSpendAmounts(source_state)) <= source_state.remaining
```

A DHT fork is not authority to duplicate a consumable budget.

### 9.2 Ballot/spend binding

A qualified quadratic ballot should reference the exact spend receipt or equivalent verifier-owned spend proof.

```text
ClaimedCreditsSpent != ProvenCreditsSpent
```

## 10. Weighted influence is not conserved civic mass

Some voting mechanisms intentionally transform source civic standing into a nonlinear influence quantity.

Quadratic voting is the clearest example:

```text
voice credits -> sqrt(credits) influence
```

The system should therefore distinguish:

```text
SourceAuthorityMass
ProcessBudget
ProcessSpecificInfluence
```

These are not interchangeable units.

Conservation applies within each unit system and across explicit transformation functions.

For example:

```text
VoiceCreditsSpent is conserved against VoiceCreditGrant
QuadraticWeight = deterministic_transform(VoiceCreditsSpent)
```

It is meaningless to require:

```text
QuadraticWeight <= BaselineCivicMass
```

unless a policy explicitly normalizes those units.

## 11. Proposal policy conservation

Once voting begins, authority-relevant policy cannot be mutated while preserving the same ballot lineage.

```text
FirstAcceptedBallot
-> VotingPolicyFrozenForLineage
```

Changing any of the following starts a new lineage unless the original policy explicitly authorized the transformation:

- eligibility semantics;
- weighting semantics;
- quorum units/threshold;
- approval threshold;
- delegation semantics;
- electorate derivation;
- voting window;
- ethics/circuit-breaker binding effects.

This prevents policy mutation from effectively minting or destroying influence after ballots already exist.

## 12. Proposal-state authority conservation

A status transition may summarize authority but cannot create it.

```text
StateTransitionAuthorityOut
<= EvidenceAuthorityIn
```

Therefore:

```text
Ended -> Approved
```

cannot carry more authority than the exact `QualifiedTally` proving approval.

And:

```text
Approved -> Signed
```

cannot carry more authority than the exact verified signature over the approved subject.

## 13. Signature authority conservation

A signature authorizes only its exact signed subject and policy scope.

```text
SignatureAuthority
<= SignedSubject x SignerPolicyScope
```

Therefore:

```text
signature over proposal A
!= authority for proposal B
```

```text
signature over action digest X
!= authority for action digest Y
```

```text
valid signer key
!= unlimited governance authority
```

A broader interpretation requires a separately legitimate policy source.

## 14. Timelock / execution conservation

Timelocks delay authority; they do not create it.

```text
AuthorizedTimelockAuthority
<= ApprovedSignedSubjectAuthority
```

Execution preflight may narrow further:

```text
AuthorizedExecution
<= AuthorizedTimelock
```

A missing dependency cannot widen this set.

```text
DependencyUnavailable -> NoAdditionalAuthority
```

Compatibility fallback may reduce service or availability, but must not broaden execution rights.

## 15. Restriction conservation

Restrictions are negative authority and need monotonic handling too.

If an active restriction blocks some action set `R`, a representation change must not silently drop it.

```text
ActiveRestrictionBefore
-> equivalent-or-stronger restriction after transformation
```

unless a legitimate revocation/expiry source is proven.

Examples include:

- vetoes;
- revocations;
- expired credentials;
- cooling periods;
- safety blocks;
- revoked delegation;
- spent voice-credit budget.

A cache eviction, missing service, stale read, alternate API, or migration path must not resurrect authority that an active restriction removed.

## 16. Advisory signals do not carry conserved authority by default

Model outputs, ethics assessments, reputation signals, collective-mirror analysis, and other deliberative information may be copied and combined freely as information.

They do not carry governance authority unless an explicit policy converts them.

```text
AdvisorySignal
+ ExplicitGovernancePolicy
-> BoundedAuthorityEffect
```

The resulting authority is sourced from the governance policy, not intrinsically from the model output.

This distinction prevents accidental claims such as:

```text
high-confidence model output
-> high political authority
```

## 17. Candidate machine-checkable invariants

Future property tests / model checking should target at least:

```text
GOV-CONS-001
sum overlapping active delegation mass <= source civic mass

GOV-CONS-002
transitive child mass <= parent available mass

GOV-CONS-003
direct + delegated exercise <= source civic mass

GOV-CONS-004
one-ballot lineage cannot contain multiple qualified exercises of one source civic unit

GOV-CONS-005
voice-credit spent is monotonic non-decreasing

GOV-CONS-006
voice-credit remaining is monotonic non-increasing

GOV-CONS-007
voice-credit allocation is immutable within one grant lineage

GOV-CONS-008
canonical concurrent spend sum <= source remaining budget

GOV-CONS-009
quadratic ballot requires exact spend lineage

GOV-CONS-010
policy change after first ballot creates new voting lineage

GOV-CONS-011
proposal status cannot carry authority absent exact transition evidence

GOV-CONS-012
signature interpretation cannot exceed signed subject + signer policy scope

GOV-CONS-013
execution authority cannot exceed approved/signed/timelocked subject

GOV-CONS-014
availability/compatibility failure cannot increase authority

GOV-CONS-015
active restriction cannot disappear without legitimate expiry/revocation
```

## 18. Suggested implementation order

Do not begin by building one generic conservation engine.

Recommended domain-owned tranches:

```text
GOV-CONS-001A  voice-credit grant issuer/policy binding
GOV-CONS-001B  monotonic grant spend + immutable grant fields
GOV-CONS-001C  exact spend receipt / quadratic-ballot binding

GOV-CONS-002A  delegation source-budget semantics
GOV-CONS-002B  overlap-aware grant admission
GOV-CONS-002C  direct/delegated exercise rule
GOV-CONS-002D  transitive fan-out conservation properties

GOV-CONS-003A  canonical ballot exercise key
GOV-CONS-003B  direct/delegated/quadratic/verified admission convergence

GOV-CONS-004   end-to-end property/model tests
```

Only after at least two authority domains demonstrate materially identical conservation semantics should HAK extract shared runtime primitives.

## 19. Desired end-state

The governance pipeline should be able to prove two independent things.

First, provenance:

```text
Where did this authority come from?
```

Second, conservation:

```text
Did any transformation create more authority than the legitimate source supplied?
```

Together:

```text
LegitimateSource
+ ExactLineage
+ ExplicitTransformationPolicy
+ ConservedSourceBudget
-> QualifiedAuthorityArtifact
```

This is the intended foundation for governance that remains composable as Mycelix gains richer delegation, quadratic voting, AI-assisted deliberation, privacy proofs, and automated execution.