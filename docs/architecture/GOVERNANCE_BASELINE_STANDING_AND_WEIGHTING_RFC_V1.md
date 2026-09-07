# Governance Baseline Standing & Explicit Weighting RFC v1

Status: architecture candidate / documentation only

Related:

- Human Agency Kernel HAK-001 / HAK-002 / HAK-003
- Mycelix issue #292 — consciousness ZKP verification security blocker
- Mycelix issue #309 — baseline civic standing vs consciousness-derived influence

## 1. Purpose

Mycelix governance currently combines several useful ideas:

- one-agent-one-vote deduplication;
- DID/author binding;
- Sybil resistance;
- proposal-tier-specific quorum and approval thresholds;
- timelocks;
- delegation with expiry/decay;
- threshold signing;
- circuit breakers;
- quadratic voting;
- Phi/consciousness metrics;
- reputation/trust signals;
- stake exposure;
- domain reputation;
- participation history.

The problem is not the existence of these signals or mechanisms.

The problem is that some current paths combine them into **baseline civic eligibility and vote influence** without first naming which source of legitimacy is intended to authorize that influence.

Current code contains the live chain:

```text
Symthaea / consciousness evidence
    -> consciousness gate
    -> voting eligibility
    -> PhiWeight
    -> weighted direct/delegated vote
```

This RFC decomposes that chain.

## 2. Constitutional invariant

```text
ModelAssessmentOfPerson != AuthorityOverPerson
```

For governance:

```text
CognitiveMetric     != BaselineCivicStanding
Reputation          != HumanWorth
Expertise           != CitizenshipWeight
Stake               != ConstitutionalVoice
Participation       != InherentPoliticalWorth
SocialTrust         != BasicCivicExistence
```

These signals may still have legitimate roles when those roles are explicit, scoped, contestable, and reviewable.

## 3. Authority kinds

Governance must distinguish at least five concepts.

### 3.1 BaselineCivicStanding

The ordinary standing of an eligible member/citizen to participate in collective governance.

Candidate default:

```text
eligible member -> one equal direct baseline vote
```

Eligibility may require domain-owned membership and Sybil-resistance checks.

It must not require an inferred cognitive/consciousness/reputation score merely to possess the baseline vote.

### 3.2 ScopedRoleQualification

Evidence that an actor is qualified for a specialized bounded role.

Examples:

- safety officer;
- treasury signer;
- infrastructure operator;
- domain reviewer;
- emergency coordinator;
- constitutional auditor;
- technical maintainer.

This may legitimately use competence/reputation/evidence because the authority is function-specific.

### 3.3 DelegatedCivicAuthority

A member explicitly delegates some portion of their own current civic authority to another actor.

This derives from:

```text
delegator baseline authority
+ explicit delegation
+ scope
+ temporal bounds
+ revocation
```

not from the delegate being assigned a higher human-worth score.

### 3.4 DeliberativeSignal

Non-binding information used to improve collective reasoning.

Examples:

- Phi/consciousness research metrics;
- domain expertise;
- prediction calibration;
- argument quality;
- affectedness;
- historical contribution;
- reputation;
- stake exposure;
- value-alignment analysis;
- minority/dissent evidence.

A deliberative signal may influence human judgment without directly multiplying baseline civic authority.

### 3.5 OperationalCapability

Permission to execute a concrete governance outcome against a resource/system.

Examples:

- transfer treasury funds;
- deploy configuration;
- revoke infrastructure access;
- publish a constitutional amendment;
- rotate a signing committee.

Operational capability must remain downstream of explicit governance decisions and execution policy.

## 4. Proposed direct-vote policy model

Introduce an explicit proposal/process weighting policy.

Conceptual type:

```text
VoteWeightPolicyV1 {
    EqualBaseline,
    Quadratic { credit_policy_id },
    Conviction { policy_id },
    ReputationExperimental { policy_id },
    PhiExperimental { policy_id },
    CompositeExperimental { policy_id },
}
```

The exact Rust shape is not proposed in this documentation tranche.

### 4.1 EqualBaseline

```text
direct eligible vote weight = 1.0
```

This becomes the constitutional/default mode unless a community constitution explicitly selects another mechanism for a bounded process.

### 4.2 Experimental weighted modes

Weighted modes remain possible, but must be:

- explicit in proposal/process metadata;
- visible before voting starts;
- bound to a governance lineage/policy version;
- inspectable by clients;
- reversible/amendable according to constitution;
- accompanied by raw unweighted tallies;
- unable to masquerade as the universal default.

## 5. Separate eligibility from weighting

Current governance conflates questions that should be answered independently.

### Question A — may this identity participate?

Candidate inputs:

```text
membership
identity binding
Sybil policy
jurisdiction/process scope
suspension/eligibility rules
```

Output:

```text
BaselineEligibility
```

### Question B — how is this process tallying eligible votes?

Input:

```text
VoteWeightPolicyV1
```

Output:

```text
WeightForThisProcess
```

### Question C — may this actor execute the decision?

Inputs:

```text
approved proposal
execution policy
threshold signatures / role qualification
resource scope
current authority
```

Output:

```text
OperationalCapability
```

None of these questions should be silently answered by the same person-level scalar.

## 6. Replace consciousness gating for baseline voting

Current action gates use consciousness thresholds for Basic participation, ProposalSubmission, Voting, and Constitutional actions.

For baseline civic participation, replace this with a dedicated eligibility boundary.

Conceptual shape:

```text
BaselineEligibilityV1 {
    member_id
    governance_lineage
    membership_current
    identity_current
    sybil_policy_satisfied
    suspended
    jurisdiction_scope
}
```

A cognitive/consciousness metric is not a required field.

### 6.1 High-impact decisions

High-impact processes still need stronger safeguards.

Use process hardening such as:

- higher quorum;
- supermajority;
- longer timelock;
- independent review;
- threshold signatures;
- affected-party review;
- adversarial/dissent analysis;
- execution separation;
- rollback/reversion plan;
- emergency-stop semantics.

Do not use a higher person-level cognitive threshold as the only or primary safety mechanism.

## 7. Preserve consciousness metrics as research/deliberative signals

This RFC does not delete Symthaea consciousness work.

It changes the semantic role.

Possible uses:

### 7.1 Voluntary self-reflection

A participant may privately use Symthaea to inspect:

- uncertainty;
- emotional state;
- cognitive load;
- coherence;
- possible biases;
- argument structure.

No civic penalty follows from the result.

### 7.2 Group-level research

With consent/privacy protection, aggregate signals may support research into collective deliberation quality.

They remain measurement outputs, not direct constitutional authority.

### 7.3 Advisory expertise layer

A community may expose domain-qualified or cognitive signals as non-binding metadata:

```text
vote: For
baseline_weight: 1.0
advisory_signals:
  domain_expertise: ...
  prediction_calibration: ...
  affected_party: ...
```

Clients may use these signals to understand deliberation without changing the canonical tally unless the process explicitly opted into a weighted experiment.

### 7.4 Explicit governance experiments

A community may deliberately test Phi-weighted governance.

That experiment should have:

- explicit policy identity;
- preregistered hypotheses;
- raw/unweighted comparison tally;
- sunset/expiry;
- outcome evaluation;
- no claim that Phi represents human worth;
- no silent transfer into other communities or governance lineages.

## 8. Preserve Sybil resistance without person scoring

Removing consciousness as a baseline vote gate must not mean removing Sybil resistance.

Use direct mechanisms such as:

- agent-key binding;
- membership proofs;
- uniqueness/personhood mechanisms appropriate to the community;
- rate/issuance constraints;
- anti-replay;
- maturation periods where constitutionally legitimate;
- fraud evidence;
- graph/community checks as investigation signals rather than automatic human-worth scores;
- privacy-preserving eligibility proofs.

Important distinction:

```text
prove one eligible participant
```

is not the same as:

```text
score how worthy this participant is
```

## 9. Delegation redesign

Delegation should preserve baseline civic semantics.

Candidate rule:

```text
DelegatedWeight
    = delegator process weight
    * explicit delegation percentage
    * decay/expiry policy
```

The delegator's baseline vote should not become stronger merely because the delegate has a higher Phi/reputation score.

If the governance process intentionally uses an experimental weighted policy, that policy must explicitly define whether weighting occurs:

- before delegation;
- after delegation;
- on delegator attributes;
- on delegate attributes;
- or not at all.

No implicit rule.

### 9.1 Recommended default

For `EqualBaseline`:

```text
one member's total delegable civic mass <= 1.0
```

regardless of delegate identity.

Delegation changes routing, not total constitutional mass.

## 10. Expertise and affectedness

One-person-one-baseline-vote does not imply all governance knowledge is equal.

Mycelix should represent expertise and affectedness structurally.

Possible process:

```text
baseline member votes
+ expert review report
+ affected-party report
+ forecast panel
+ dissent report
+ execution feasibility review
```

The reports are separately visible and provenance-bearing.

This is richer than compressing all of those dimensions into a single weighted ballot.

## 11. Collective-intelligence layer

Symthaea can improve governance without becoming the governor.

Recommended pipeline:

```text
independent participant judgments
    -> evidence/argument graph
    -> expertise discovery
    -> affected-party discovery
    -> dissent preservation
    -> Symthaea synthesis
    -> explicit human deliberation
    -> baseline or explicitly selected voting policy
    -> decision
    -> execution authorization
    -> outcome evidence
    -> postmortem / policy learning
```

This creates more intelligence around the vote rather than concentrating authority inside a model score.

## 12. Raw-tally dual reporting

Every non-equal weighting experiment should produce both:

```text
canonical policy tally
raw equal-person tally
```

Example:

```text
PolicyTally:
  for_weight = 148.2
  against_weight = 121.5

RawTally:
  for = 109
  against = 117
```

A divergence is important governance evidence and should never be hidden.

This supports:

- contestability;
- scientific analysis;
- detection of weighting-system capture;
- future constitutional review.

## 13. Weighting-policy provenance

Every weighted tally should bind:

```text
policy_id
policy_version
governance_lineage
proposal_id
eligible-voter-set commitment or derivation rule
weighting algorithm
parameter set
input provenance requirements
start/end validity
raw tally
weighted tally
```

A tally without its weighting-policy identity is not independently interpretable.

## 14. Migration plan

### Phase 0 — security blocker

Resolve #292 before relying on proof-bearing consciousness eligibility.

```text
StructureValid != ProofVerified != TrustedVerifier
```

This is independent from the constitutional migration below.

### Phase 1 — introduce explicit policy vocabulary

Add process metadata capable of distinguishing equal and experimental weighted modes.

No current wire fields are deleted.

### Phase 2 — add pure equal-baseline weight function

Candidate invariant:

```text
compute_baseline_vote_weight(eligible_direct_voter) == 1.0
```

Tests should demonstrate that changing Phi, reputation, stake, participation, or domain reputation does not change baseline weight.

### Phase 3 — separate eligibility verifier

Create a baseline governance eligibility function that does not depend on consciousness metrics.

Consciousness gates remain available only for explicitly named experimental/scoped policies during migration.

### Phase 4 — proposal-level policy binding

Each proposal/process binds the selected weighting policy before voting begins.

Changing the policy after the first vote requires a new proposal/voting lineage rather than reinterpretation of existing ballots.

### Phase 5 — delegated-vote migration

Route delegated equal-baseline civic mass without applying delegate person scores.

Experimental weighted modes remain separately specified.

### Phase 6 — deprecate implicit Phi weighting

Deprecate paths where ordinary direct voting implicitly calls `calculate_vote_weight()` without explicit proposal policy.

Do not rewrite historical records.

### Phase 7 — constitutional review

Review `ConsciousnessProfile`, `SovereignCredential`, tier thresholds, and constitutional-envelope assumptions under the new distinction:

```text
score robustness != legitimacy of score-based civic authority
```

## 15. Backward compatibility

Historical vote entries remain historical evidence.

Do not reinterpret old `weight` fields as equal baseline weights.

Instead record the policy semantics used at creation whenever known.

For legacy entries without explicit policy identity:

```text
policy = LegacyImplicitWeightedV1
```

or an equivalent migration marker.

This preserves provenance.

## 16. Candidate tests

### GOV-HAK-P1 — cognitive independence of baseline standing

For any two otherwise eligible members A and B:

```text
Phi(A) != Phi(B)
```

does not imply:

```text
BaselineVoteWeight(A) != BaselineVoteWeight(B)
```

### GOV-HAK-P2 — reputation independence

Changing K-trust/reputation alone does not change baseline direct vote weight.

### GOV-HAK-P3 — stake independence

Changing stake alone does not change baseline direct vote weight.

### GOV-HAK-P4 — delegation conservation

Under equal-baseline policy:

```text
sum(delegated civic mass originating from member M) <= 1.0
```

### GOV-HAK-P5 — explicit experiment binding

A Phi-weighted vote is invalid unless the proposal/process explicitly selected a Phi-weighted policy before voting began.

### GOV-HAK-P6 — raw tally availability

Every weighted final tally contains or references the raw equal-person tally.

### GOV-HAK-P7 — policy immutability during vote lineage

After the first accepted ballot:

```text
weighting_policy_id cannot change
```

without starting a new voting lineage.

### GOV-HAK-P8 — process hardening independent of cognitive score

Constitutional/Emergency safety checks remain enforceable when consciousness weighting is disabled.

## 17. Candidate code tranches

Keep changes reviewable and independently qualifiable.

```text
GOV-HAK-001  weighting-policy types / docs / wire semantics
GOV-HAK-002  equal-baseline pure weight computation
GOV-HAK-003  baseline eligibility verifier
GOV-HAK-004  proposal binds weighting policy
GOV-HAK-005  equal-baseline direct vote path
GOV-HAK-006  delegation conservation under equal baseline
GOV-HAK-007  raw + weighted dual tally
GOV-HAK-008  deprecate implicit weighted legacy path
GOV-HAK-009  migrate consciousness metrics to deliberative signals
GOV-HAK-010  constitutional-envelope review
```

Security issue #292 remains a separate prerequisite for any retained ZKP-gated policy.

## 18. What remains valuable from the existing weighted architecture

The current implementation contains useful ideas worth preserving:

- explicit Phi provenance (`Attested`, `Snapshot`, `Unavailable`);
- refusal to fabricate Phi when unavailable;
- raw vote counts in Phi-weighted tallies;
- anti-plutocracy caps;
- delegation cycle detection;
- delegation decay and expiry;
- tiered quorum floors;
- proposal-type-specific thresholds;
- timelocks;
- fail-closed critical bridge calls;
- provenance-bearing attestations;
- constitutional anti-capture intent.

The redesign should retain these where semantically appropriate.

## 19. What changes philosophically

Old tendency:

```text
better measured participant
    -> more governance authority
```

New default:

```text
member
    -> baseline civic standing

expertise / evidence / cognition / affectedness
    -> richer deliberation and scoped roles

explicit community experiment
    -> optional bounded weighting mechanism
```

This creates a constitutional floor beneath experimentation.

## 20. Non-claims

This RFC does not claim:

- one-person-one-vote is optimal for every specialized institution;
- expertise should never matter;
- quadratic voting should be removed;
- reputation systems have no value;
- stake exposure is always irrelevant;
- consciousness research should be removed;
- every community must choose identical governance;
- Mycelix is currently deployed as sovereign public government;
- current governance authors intended human-worth scoring.

The narrow rule is:

> Baseline civic authority should not silently depend on a model-derived assessment of the person.

## 21. Review gate

Before code work begins, review should answer:

1. What establishes membership/baseline eligibility in each governance deployment model?
2. Which existing weighted modes should remain supported as explicit experiments?
3. Which high-impact safeguards replace cognitive gating for Constitutional/Emergency decisions?
4. How should legacy vote records identify their implicit weighting policy?
5. How should delegation preserve civic mass across policy modes?
6. Which consciousness/reputation/expertise outputs should become deliberative signals rather than authority inputs?
7. Which constitutional-envelope invariants remain valid after baseline standing is separated from score-based tiers?

Until these are reviewed:

```text
no silent reinterpretation of existing votes
no deletion of weighted experiments
no claim that the redesign is qualified
```
