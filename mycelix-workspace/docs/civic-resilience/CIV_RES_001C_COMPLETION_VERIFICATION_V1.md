# CIV-RES-001C — Completion evidence / verification / outcome separation v1

Status: semantic composition contract only  
Parent: CIV-RES-001B / `c7b6f3ac3113d2eba83b01eeb12ab66c3598ee9f`  
Tracking issue: #2023  
Program: #2006

## Purpose

Freeze the Civic Resilience theorem that an externally owned public/community commitment, a claim that it was completed, evidence supplied for that claim, verification of an exact completion predicate, and downstream outcome observations are distinct artifacts with distinct meanings.

CIV-RES-001C is intentionally narrower than a workflow, obligation, execution, payment, procurement, settlement, or effect engine.

## Existing semantic owners

Repository work already defines stronger adjacent concepts that CIV-RES must reuse rather than duplicate where applicable:

- response proposals that deliberately grant no execution authority (draft #483);
- execution receipts and outcome lineage with external authority resolution (draft #484);
- Business obligations, observations, reconciliation and closure semantics (draft #165);
- governance/integration/effect lines for external execution and outcome ambiguity.

These are architectural references only in this tranche. Their presence does not create qualification inheritance or executable ancestry.

## Core chain

```text
ExternalCommitmentRef
        |
        v
CompletionClaim
        |
        v
CompletionEvidenceBundle
        |
        v
CompletionVerificationEvidence
        |
        v
OutcomeObservationRef(s)
```

This is an evidence/provenance chain.

It is not:

```text
commitment -> automatic execution authority
claim -> completed
completion evidence -> verified completion
verified completion -> outcome improvement
outcome improvement -> causal effect
```

## External commitment ownership

CIV-RES-001C does not define a universal commitment/obligation state machine.

`ExternalCommitmentRef` is an opaque typed reference to a commitment owned by another legitimate domain/process, such as a municipal/public-institution decision, Business/Commerce obligation, Governance resolution, Commons work programme, or other domain-specific system.

```text
ExternalCommitmentRef != CommitmentValid
CommitmentValid != CurrentAuthority
CommitmentValid != ExecutionOccurred
Commitment != AuthorityToExecute
```

The adapter that consumes a commitment must validate the exact referenced subject under its owning semantics before making a stronger claim.

## 1. CompletionClaim

A `CompletionClaim` means:

> a bounded assertion that one exact external commitment was fulfilled for one declared completion predicate/scope.

Semantic refs:

```text
completion_claim_id
external_commitment_ref
claimant_or_source_ref
claimed_completion_predicate_ref
claimed_scope_ref
claimed_time_ref
statement
limitations[]
```

A completion claim contains no caller-selectable `verified`, `official`, `paid`, `accepted`, `successful`, or `outcome_improved` boolean.

```text
Commitment != CompletionClaim
CompletionClaim != CompletionEvidence
```

## 2. CompletionEvidenceBundle

A `CompletionEvidenceBundle` means:

> one frozen evidence cut supplied in support of one exact completion claim.

Semantic refs:

```text
completion_evidence_bundle_id
completion_claim_ref
external_commitment_ref
input_evidence_refs[]
collection_or_source_refs[]
time_evidence_refs[]
location_or_scope_refs[]
method_refs[]
limitations[]
```

The bundle does not decide whether its evidence is authentic, sufficient, current, independent, representative, or relevant enough to prove the completion predicate.

```text
CompletionEvidence != CompletionVerification
PhotoOrDocument != Completion
ProviderAcknowledgement != Completion
Payment != Completion
```

A photo, document, provider acknowledgement, payment record, sensor reading, external reference, resident report, or execution receipt may be evidence. None is a universal completion oracle.

## 3. CompletionVerificationEvidence

`CompletionVerificationEvidence` means:

> evidence that one explicitly identified verification process evaluated one exact completion predicate against one exact input evidence cut under one exact method/scope.

Semantic refs:

```text
completion_verification_evidence_id
completion_claim_ref
completion_evidence_bundle_ref
external_commitment_ref
verification_predicate_ref
verification_method_ref
verifier_ref
verifier_qualification_ref
verifier_independence_profile_ref
input_evidence_cut_ref
verification_result_ref
verification_time_ref
limitations[]
```

There is deliberately no universal `completed=true` or `independent=true` field.

### Predicate scope is load-bearing

Verification can establish only the proposition it actually evaluated.

Examples of different predicates include:

```text
physical presence at one location/time
quantity delivered
function observed at verification time
specified quality-profile conformance
specified accessibility requirement
beneficiary acceptance
specified durability/recovery condition
```

These are not interchangeable.

```text
Verification != QualityBeyondVerifiedPredicate
Verification != BeneficiaryAcceptance
```

unless the exact predicate itself evaluated those propositions.

## Independence is not an identity comparison

A different DID, actor ID, organization name, device, or verifier label does not by itself establish independent verification.

```text
DifferentVerifierId != IndependentVerification
```

A stronger independence claim requires an externally defined `VerifierIndependenceProfileRef` that exposes relevant shared fault/provenance domains. CIV-RES-001C does not define one universal independence score.

## Execution receipt boundary

A generic/domain `ExecutionReceiptRef` may establish evidence about what an executor reports or what an execution subsystem qualified under its own theorem.

It does not automatically prove the civic completion predicate.

```text
ExecutionReceipt != CompletionVerification
```

A completion verifier may consume an execution receipt as one evidence input while retaining independent method/predicate/evidence-cut semantics.

## Payment boundary

Payment, invoice, settlement, procurement and completion are separate authority/evidence stages.

```text
Payment != Completion
CompletionVerification != PaymentAuthority
VerifiedCompletion != PaymentDue
```

The last relation may be established only by the owning contract/procurement/payment semantics and exact policy, never by CIV-RES-001C alone.

## Outcome boundary

CIV-RES-001A already owns `OutcomeObservation` semantics. 001C therefore references `OutcomeObservationRef` rather than defining another outcome object.

A verified completion may be associated with later outcome observations for evaluation, but:

```text
CompletionVerification != OutcomeObservation
VerifiedCompletion != OutcomeImprovement
OutcomeObservation != CausalEffect
verified completion + better outcome != causal effect
```

Causal attribution belongs in the future SYM-CIVIC scientific line.

## Required non-equivalences

The v1 closed registry is:

```text
Commitment != AuthorityToExecute
Commitment != CompletionClaim
CompletionClaim != CompletionEvidence
CompletionEvidence != CompletionVerification
CompletionVerification != OutcomeObservation
VerifiedCompletion != OutcomeImprovement
OutcomeObservation != CausalEffect
ExecutionReceipt != CompletionVerification
ProviderAcknowledgement != Completion
Payment != Completion
PhotoOrDocument != Completion
DifferentVerifierId != IndependentVerification
Verification != BeneficiaryAcceptance
Verification != QualityBeyondVerifiedPredicate
CompletionVerification != PaymentAuthority
VerifiedCompletion != PaymentDue
```

## Opaque dependency references

The v1 semantic waist uses opaque refs for stronger external semantics:

```text
ExternalCommitmentRef
EvidenceRef
CollectionSourceRef
TimeEvidenceRef
LocationScopeRef
MethodRef
VerificationPredicateRef
VerificationMethodRef
VerifierQualificationRef
VerifierIndependenceProfileRef
InputEvidenceCutRef
VerificationResultRef
OutcomeObservationRef
ExecutionReceiptRef
AuthorityDecisionRef
```

```text
opaque ref != referenced proposition valid
```

## No universal completion score

CIV-RES-001C defines no scalar completion confidence, quality score, contractor trust score, resident trust score, verifier reputation score, or automatic pass percentage.

Domain-specific quantitative measurements may be evidence, but their interpretation must remain bound to the exact predicate/method/policy that gives them meaning.

## Adversarial cases a later executable layer must cover

At minimum:

- claim references the wrong commitment revision;
- photo/evidence reused across two commitments;
- evidence collected outside the declared location/time scope;
- provider acknowledgement exists but physical result is absent;
- payment exists but work is incomplete;
- execution receipt exists but completion predicate differs;
- verifier is the claimant under a profile requiring independence;
- two nominal verifier identities share one relevant fault domain;
- verification method checks presence but UI labels it quality-complete;
- later evidence contradicts earlier verification;
- completion is verified but target outcome worsens;
- target outcome improves before completion or in a control area too;
- stale verification is presented as current state.

Passing such tests would establish only the tested semantic/refusal properties, not real-world truth.

## Runtime ownership

Runtime ownership remains deferred until generic evidence/currentness and the adjacent obligation/execution lines converge enough to avoid duplicate authority/evidence machinery.

A later implementation may add a narrow CIV-RES completion-verification adapter, but it should consume external commitment/evidence/execution/outcome references rather than own those generic systems.

## Continuation

```text
CIV-RES-002A  service-issue lifecycle composition
CIV-RES-002B  offline signed/idempotent submission queue
CIV-RES-002C  executable projection/mosaic defenses
SYM-CIVIC-000A civic study/evidence-cut bridge
```

## Qualification claim ceiling

A PASS may establish only that this exact contract preserves external commitment ownership, completion-claim/evidence/verification separation, predicate-scoped verification, nontrivial independence, execution/payment non-substitutability, outcome/causal separation, deferred runtime ownership and nonclaims.

It does not establish runtime commitment storage, obligation semantics, execution authority, evidence authenticity, completion truth, verifier independence, service quality, beneficiary acceptance, payment authority, outcome improvement, causal effect, municipal legitimacy, Johannesburg deployment, or deployment readiness.
