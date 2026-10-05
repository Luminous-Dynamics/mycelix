# Fabrication security hardening

## Epistemic cache boundary

The verification coordinator caches Knowledge-hApp epistemic classifications only as an
execution optimization. A cache entry is identified by a versioned SHA-256 digest over:

1. the claim type key;
2. the exact claim text;
3. explicit field-length delimiters.

A claim therefore cannot reuse another claim's classification merely because both claims
share the same SafetyClaimType.

The cache is bounded to 128 live entries. Expired or future-dated entries are not reused,
and the oldest entry is evicted when the bound is reached. Cache identity is not an
evidence identity and must never be treated as a cryptographic attestation of the
Knowledge hApp result.

## CI coverage

The repository-level Fabrication hApp CI workflow directly covers the nested
mycelix-workspace/happs/fabrication Cargo workspace with formatting, native tests,
WASM compilation, and clippy. This closes the previous coverage gap where the root
Mycelix CI path filters did not select this nested Holochain workspace.

The fabrication workspace currently targets Holochain 0.6. Changes to its admission
or provenance semantics should therefore be qualified by this dedicated workflow
before being treated as executable evidence.


## Epistemic availability and provenance boundary

Safety-claim admission fails closed with respect to epistemic evidence rather than
with respect to claim existence. An unavailable, unauthorized, or malformed Knowledge
response is represented explicitly and is never converted into a synthetic classification
or inserted into the cache.

The stored SafetyClaim carries an explicit provenance state:

- KnowledgeClassified — a finite, bounded Knowledge classification was accepted.
- KnowledgeUnavailable — classification could not be obtained.
- KnowledgeMalformed — a response was present but failed decoding or semantic validation.
- LegacyUnattributed — a historical record lacked a persisted provenance discriminator.

Only KnowledgeClassified claims with an attached classification contribute to current
epistemic aggregates. The other states remain queryable but contribute zero. The aggregate
also exposes an explicit evidence_status plus classified_claims/total_claims counts.
Consumers MUST NOT interpret a numeric zero as negative epistemic evidence when
evidence_status is NoClassifiedEvidence; it means there was no Knowledge classification
available to contribute. PartialClassifiedEvidence similarly means the aggregate is
incomplete and must not be read as a complete Knowledge assessment.

This provenance is an application-level source state, not a cryptographic attestation of
Knowledge correctness. The actual inter-hApp transport remains a separate architecture
boundary tracked in issue #4227.
