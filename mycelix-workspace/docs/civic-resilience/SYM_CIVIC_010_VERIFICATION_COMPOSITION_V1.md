# SYM-CIVIC-010 — multi-verifier result composition and conflict semantics v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-009 / `21b98a116a34088539dc782a1dfc886e5f4dc35b`

Tracking issue: #3869

## Purpose

Qualify how independently authenticated verification results compose without silently collapsing provenance, conflicts, temporal state, policy incomparability, or authority boundaries.

The current in-toto SVR predicate permits multiple SVRs for the same subject. SVR records verifier identity, policy references, evaluation time, and verified properties, but it does not define a universal composition or conflict-resolution rule.

The research separation is:

`VerificationResult != EvidenceComposition != ConflictResolution != PolicyDecision != ScientificTruth != CivicAuthority`

## Contract

A composition is admissible only when:

- every participating result remains individually authenticated and bound to the same target subject;
- verifier identity/version and policy identity/version remain attached to each result;
- expired results cannot silently dominate current results;
- out-of-scope results cannot satisfy a broader requested scope;
- duplicate result identities with conflicting content cannot collapse into one identity;
- exact duplicate replay is idempotent rather than creating a new semantic result;
- agreement preserves the provenance identities of all agreeing results;
- contradictory assertions remain an explicit conflict rather than selecting a winner;
- negative assertions are preserved as evidence rather than silently discarded;
- aggregation order does not change the composed evidence;
- incomparable policies remain incomparable unless an explicit external ordering exists;
- quorum/count metadata does not become authorization;
- conflicting scientific interpretations remain interpretations, not resolved fact.

The corpus intentionally includes a small research-only assertion extension for negative/conflicting values because SVR's standard `properties` field represents passing properties rather than a general positive/negative assertion algebra. This extension is not claimed as an in-toto schema.

## Typed dispositions

- `REJECT_COMPOSITION_PROVENANCE`: authentication, subject, policy, scope, temporal, identity, retention, ordering, or authority-boundary failure.
- `COMPOSITION_AGREEMENT`: current valid results agree and every provenance identity is retained.
- `COMPOSITION_CONFLICT`: current valid results make incompatible assertions and the conflict is explicitly preserved.
- `COMPOSITION_INCOMPARABLE`: valid results cannot be ordered under their declared policy relationship and are preserved as incomparable evidence.

## Corpus

M-01 exact agreement
M-02 contradictory properties
M-03 same verifier with different policy versions
M-04 different verifiers with overlapping compatible scope
M-05 expired result plus current result
M-06 stale result incorrectly dominates current
M-07 conflicting results collapsed
M-08 first-result/order-dependent composition
M-09 negative assertion discarded
M-10 out-of-scope verifier included
M-11 duplicate result identity with conflicting content
M-12 historical result reused as current
M-13 quorum threshold treated as authorization
M-14 conflicting scientific interpretations resolved as fact
M-15 agreement preserves both provenance identities
M-16 explicit conflict state preserves both valid results
M-17 order-independent monotonic positive-property composition
M-18 incomparable policy result remains incomparable
M-19 exact duplicate replay is idempotent
M-20 incompatible scopes remain separate rather than winning

The qualifier derives dispositions from semantic predicates and computes a canonical composition receipt. Fixture files contain no expected verdicts.

## Qualification ceiling

PASS establishes only that this synthetic benchmark preserves result provenance and declared composition semantics.

It does not establish truth of the underlying verified properties, correctness of any policy, security of a real verifier implementation, authorization legitimacy, or deployment readiness.

No runtime implementation is proposed.

## Standards alignment

The current in-toto Attestation Framework is v1.2. The SVR predicate is v0.2. SVR permits multiple results for the same subject, requires `verifier.policies`, and records point-in-time verification properties. The framework's general parsing rules recommend monotonic policy consumption, while the Bundle layer requires order-independent processing and notes that the bundle itself is not authenticated as a whole.

References:
- https://github.com/in-toto/attestation/blob/main/spec/predicates/svr.md
- https://github.com/in-toto/attestation/blob/main/spec/v1/README.md
- https://github.com/in-toto/attestation/blob/main/spec/v1/bundle.md
