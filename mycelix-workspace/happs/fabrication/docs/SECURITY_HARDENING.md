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

Safety-claim admission is fail-closed with respect to the Knowledge epistemic dependency.
An unavailable, unauthorized, or malformed Knowledge response is an error and must not
be converted into a synthetic classification or inserted into the cache.

The stored SafetyClaim now carries an explicit epistemic provenance state:

- KnowledgeClassified — the coordinator accepted a finite, bounded classification response.
- LegacyUnattributed — a historical record lacked persisted provenance.

LegacyUnattributed records remain queryable, but they are excluded from current
Knowledge-sourced epistemic aggregates. This prevents historical fallback or otherwise
unattributed scores from silently becoming current evidence.

This distinction is source provenance, not an attestation that the external Knowledge
system itself is correct. The actual inter-hApp transport remains a separate architectural
boundary and is tracked in issue #4227; the current Fabrication-only hApp must not claim
that it has a working direct Knowledge-hApp call.
