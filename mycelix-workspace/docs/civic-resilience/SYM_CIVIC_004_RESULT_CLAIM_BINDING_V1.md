# SYM-CIVIC-004 — result-artifact / scientific-claim binding v1

Status: synthetic research provenance contract only

Parent: SYM-CIVIC-003 study-execution evidence / `f6df4a700b236fe3fb982a8be5e14fd83d01fbac`

Tracking issue: #3846

## Purpose
Prevent a valid execution receipt or immutable result artifact from being silently promoted into a stronger proposition.

The four planes remain distinct:

`ExecutionEvidence != ResultArtifact != ScientificClaim != CivicAuthority`

A result artifact records an output. A scientific claim interprets that output within explicit scope, provenance, derivation, uncertainty, and limitations. Neither establishes civic or operational authority.

## Required binding
`result_artifact + study_execution_evidence_ref + upstream_evidence_cut + result_semantics + claim_scope + claim_proposition + supporting_result_refs + derivation_refs + uncertainty_ref + limitations + alternative_explanations + output_disposition + claim_provenance`

## Required non-equivalences
- ExecutionEvidence != ResultArtifact
- ResultArtifact != ScientificClaim
- Estimate != CausalTruth
- ConfidenceInterval != Guarantee
- Prediction != Observation
- Interpretation != Observation
- ScientificClaim != CivicAuthority
- ModelRecommendation != Authorization
- ValidExecution != ValidCausalConclusion

## Adversarial qualification target
Reject result-digest mismatch, post-binding mutation, mutable provenance, scope widening, uncertainty-to-guarantee leakage, unsupported heterogeneous collapse, missing derivation edges, invalidated-source survival, aggregate-to-person inference, recommendation-to-authorization leakage, and missing execution binding.

Accept correctly bound descriptive results, scoped causal estimates with explicit uncertainty/limitations, inconclusive dispositions, and explicitly distributional stochastic replication.

## Qualification ceiling
PASS establishes only that this synthetic benchmark preserves the result/claim/authority distinction and detects the declared provenance failures.

It does not establish real-world causal effect, scientific truth, clinical validity, public safety, municipal legitimacy, policy efficacy, or deployment readiness.

## Provenance alignment
W3C PROV supplies the entity/activity/agent/derivation vocabulary. RO-Crate supplies interoperable workflow provenance patterns in which actions reference inputs as `object` and outputs as `result`. This contract uses those concepts without creating a competing provenance substrate.

No runtime implementation is proposed by this tranche.
