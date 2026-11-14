# LARP-EPISTEMIC — Scientific Evidence and Reward Contract v2.0

**Status:** research contract profile; no live reward system or scientific adjudication claimed  
**Lineage:** historical Mycelix-DeSci EpistemicBounty Pact v1.1

## 1. Purpose

This contract governs how a scientific contribution may become eligible for recognition or a bounded reward, how reviewer evidence is qualified, and how corrections, retractions and challenges are handled.

It does not decide scientific truth from a vote count, reputation score, semantic similarity or AI confidence. It does not execute treasury transfers or impose sanctions.

The historical E4/N3 threshold and materiality schedule are retained as historical policy examples only. Any live threshold must be explicitly adopted by the relevant scientific institution and versioned in its local constitution/policy. There is no universal E4/N3 authorization.

## 2. Core separation

Keep five objects distinct:

1. **Scientific claim:** the proposition and its declared scope.
2. **Evidence artifact:** data, method, provenance, uncertainty and reproducibility.
3. **Review attestation:** what a reviewer evaluated, using which competence and conflict declarations.
4. **Institutional assessment:** a versioned determination under an identified local evidence policy.
5. **Reward request:** the exact bounded recipient, amount, purpose and funding source submitted to the treasury owner for a separate authorization.

The reviewer does not authorize the reward. The assessment service does not execute it. The treasury does not decide scientific truth.

## 3. Eligibility conditions

A reward request may be prepared only when all locally required predicates are satisfied:
- the claim is within the pact's scientific scope;
- evidence satisfies the applicable method and reproducibility requirements;
- evidence owners and provenance are identified;
- reviewer conflicts are disclosed and resolved under the local policy;
- the required independent evidence threshold is met;
- known common dependencies are represented and do not invalidate the declared independence requirement;
- the claim's materiality and reward ceiling are bound to the current contract version;
- the assessment is reproducible from a pinned evidence snapshot;
- participant rights, attribution, privacy and applicable consent conditions are satisfied.

A reviewer count is not an independence proof. Different reviewers may share an employer, funder, upstream dataset, instrument, model, service provider or appointment authority.

## 4. Semantic similarity and collusion

Semantic similarity is a **review signal**, not a misconduct finding.

- Similar reviews can be independently correct.
- Dissimilar reviews can be coordinated.
- Similarity may trigger a request for method-level explanation, source lineage or independent review.
- No competence decay, slashing, blacklist, quarantine or grant revocation follows from similarity alone.
- Collusion findings require independently corroborated evidence, conflict-screened review, notice, reasons, proportionality and appeal.
- An unresolved suspicion may put a reward request on hold or trigger a challenge process; it cannot become a permanent penalty by inaction.

Retire the historical automatic “laziness penalty” and “thermodynamic quarantine” execution semantics. If a temporary protective hold is necessary, it must be narrowly scoped, time-bounded, reviewable and unable to silently become punishment.

## 5. Retraction, correction and appeal

Retraction and correction are first-class events, not deletion.

A retraction record includes:
- original claim and evidence digests;
- reason and supporting artifact;
- effective time and knowledge time;
- whether the correction changes current applicability;
- whether any previous reward request or assessment is affected;
- notice and participant response;
- review and appeal outcome;
- any restitution or correction action authorized by the local process.

An honest retraction does not automatically imply incompetence or misconduct. A grace period may be adopted locally, but it must be a transparent policy parameter rather than a hard-coded universal assumption. Proven misconduct requires separate adjudication and due process.

## 6. Reward lifecycle

```text
ClaimSubmitted
  → EvidenceAssembled
  → ReviewsRecorded
  → IndependenceAndConflictAssessed
  → LocalAssessmentIssued
  → RewardRequestPrepared
  → TreasuryOwnerAuthorization
  → TreasuryDispatch
  → ProviderAcknowledgement
  → EffectObserved
  → Reconciled
```

The reward request is bound to:
- claim and assessment digests;
- contract and constitutional digests;
- recipient identity and locally verified entitlement;
- exact amount, currency, ceiling and schedule;
- purpose and funding source;
- idempotency / logical-operation identity;
- expiry and revocation state.

Unknown payment outcomes remain unknown until reconciled. No blind retry. A duplicate already-applied logical operation is denied. Treasury authorization is always separate from scientific assessment.

## 7. Decision outcomes

- **Permit:** the assessment or bounded reward request meets the declared local contract.
- **Deny:** a known prohibition exists, such as unauthorized recipient substitution, invalid funding scope or a sanction based solely on a score.
- **Hold:** evidence is stale, a local threshold is not yet met, or treasury/effect state is unknown.
- **Indeterminate:** independence, provenance or a required source inventory is incomplete.
- **ChallengeRequired:** an adverse finding or material control-concentration signal needs independent review.

The result includes stable reason codes, source digests, local policy version, relevant authority epoch and effective/knowledge time. It is not itself an authorization to transfer funds or punish a participant.

## 8. Rights and safeguards

- No automatic reputation or competence penalty.
- No automatic blacklisting or permanent exclusion.
- No reward denial solely because a researcher disputes a reviewer.
- Reviewers can challenge inaccurate conflict or competence records.
- Researchers can correct, retract and appeal.
- Public reproducibility must not override privacy, consent, security or legitimate data sovereignty.
- Reward decisions must disclose criteria, reasons and appeal routes.
- Historical records are retained with correction/supersession links.

## 9. Benchmark programme

The companion input corpus contains 12 synthetic cases with a separate oracle. It includes:
- benign high-similarity agreement;
- low-similarity coordinated behavior;
- uncorroborated cartel suspicion;
- shared upstream evidence dependencies;
- declared conflict and quorum failure;
- honest retraction;
- unresolved misconduct allegation;
- successful appeal and correction;
- missing treasury authorization;
- unknown payment result;
- exact recipient/amount substitution;
- duplicate payout after effect.

Primary metrics:
- reward eligibility false accept/deny;
- reviewer conflict classification;
- independence false-claim rate;
- benign-similarity sanction rate (hard target zero);
- corroborated coordination detection and false-positive rate;
- appeal access and remedy completion;
- historical replay correctness;
- duplicate/unknown treasury effect handling;
- reviewer and researcher burden.

No single score ranks a scientist. Report each metric and its denominator separately.

## 10. External comparison boundary

Compare the scientific-review dimensions with peer-review and research-integrity methods; compare the reward/authorization subset with owner-native Mycelix finance and exact-operation controls. Do not claim to improve scientific truth merely by using more cryptography, a reputation system or more reviewers.

NICE is not a scientific competence standard. Use discipline-specific, locally recognized expertise evidence for scientific review; use NIST NICE only for applicable cybersecurity duties such as securing evidence infrastructure, incident response or key custody.

## 11. Validation status

This profile and its synthetic cases are a proposed contract. They do not establish that a claim is true, that a reward is legally owed, or that an external treasury action has occurred.
