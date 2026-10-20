# Scientific Authority Matrix

This document defines authority boundaries for Mycelix DeSci scientific information.

The governing rule is simple:

> Derived analysis can explain, summarize, rank, predict, or criticize canonical scientific state, but it cannot silently become canonical evidence or an attestation.

## Authority classes

| Class | Meaning | May directly become evidence? | May directly become attestation? |
|---|---|---:|---:|
| `CanonicalEvent` | Event admitted through governed append-only history | No | No |
| `ExternalEvidence` | Content-addressed scientific artifact | Yes, through governed admission | No |
| `HumanAttestation` | Signed human/institutional scientific assertion | No | Yes, through governed admission |
| `DerivedProjection` | Deterministic/statistical projection over canonical inputs | No | No |
| `ModelInference` | Model/reasoner-generated relationship, hypothesis, or conclusion | No | No |
| `AdvisoryAssessment` | Non-binding critique, prioritization, or review assistance | No | No |

## Module guidance

The following existing modules are useful derived/advisory layers unless and until a governed event explicitly records a human or institutional act based on them:

- `citation.rs`: bibliometric/discovery projection
- `reproducibility.rs`: reproducibility summary projection
- `inference.rs`: model/rule-derived relationship analysis
- `bayesian.rs`: probabilistic projection
- ranking, reputation, or trust-derived scientific summaries

None of these may increase scientific maturity merely because their computed value changes.

## Promotion rule

There is no implicit promotion path:

```text
DerivedProjection -> EvidenceArtifact        forbidden
ModelInference    -> EvidenceArtifact        forbidden
ModelInference    -> Attestation             forbidden
AdvisoryAssessment -> Attestation            forbidden
```

A human or institution may independently create a new governed attestation after reviewing a derived product. That new attestation is a separate signed act and must preserve provenance to the advisory/derived input where relevant.

Likewise, an externally produced artifact may be admitted as evidence only through the normal governed evidence path. The fact that a model emitted bytes does not make those bytes scientific evidence.

## Cross-system boundary

Symthaea experimental output must arrive through an adapter/import boundary. `RunEvidence` or model inference in Symthaea is not, by itself, a Mycelix scientific conclusion. Cross-system identities must use stable cryptographic content hashes and signed/versioned manifests rather than process-local or non-cryptographic fingerprints.
