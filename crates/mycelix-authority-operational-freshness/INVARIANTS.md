# Operational Authority Freshness v0.1 — Normative Invariants

Status: **pure qualification candidate; no runtime provider or DNA provisioning**

This layer proves currentness for ordinary operational authority subjects after the applicable operational policy context has itself been proven current.

The required path is:

`current operational policy context (#116)`
→ `non-authoritative fresh probe (#114)`
→ `exact #96 source/witness coverage`
→ `exact #91 covered append-only state projection`
→ `current-only #74 freshness receipt`
→ `qualified operational current authority`.

It never selects a latest record, never infers currentness from DHT absence, and never turns historical validity into live authority.

## 1. Current operational context is mandatory

`qualify_operational_subject_freshness` accepts a non-deserializable `QualifiedOperationalPolicyContext` from #116.

That context must still be current at `now_ms`.

The target subject is taken from that qualified context; callers cannot substitute another target after policy qualification.

## 2. Probe must bind the exact target and exact policy context

The supplied `VerifiedCoverageChallenge` must bind:

- the exact target `AuthoritySubjectRef`;
- the exact context-policy digest; and
- the exact coverage-policy digest

from the current operational policy context.

A valid probe for another subject or another policy context is rejected before state projection.

The probe remains evidence collection only. Its author/provenance is not institutional authority.

## 3. Full context/source/witness coverage is requalified

A positive path calls #96 `qualify_context_bound_coverage` using the exact policy receipts retained by the qualified operational context.

The call also consumes the exact probe receipt, source-head receipt, witness receipts and witness trust bindings.

Therefore current policy qualification does not replace dynamic source coverage, and dynamic source coverage does not replace current policy qualification.

Both are required.

## 4. Complete append-only current projection is mandatory

The #96 result is projected into `VerifiedAuthorityStateCoverage` and passed with the exact transition receipts to #91 `project_current_authority_state`.

The transition chain must therefore be complete through the freshly covered source head, contiguous from generation 1, fork-free, parent-linked and source-consistent.

A valid prefix is not current authority.

## 5. Historical state cannot become live freshness

This layer calls only `project_current_authority_state` followed by `to_current_freshness_receipt`.

It does not call or accept the historical/as-of projection path.

Later revocation can block new live authority without erasing historically legitimate actions.

## 6. Operational context lease caps current freshness

The resulting freshness receipt is narrowed to:

- `verified_at_ms = max(state/source verification, operational-context verification)`; and
- `lease_until_ms = min(state/source lease, operational-context current lease)`.

The narrowed receipt is revalidated before positive qualification.

A policy/root rotation therefore cannot be bypassed by a longer-lived old source proof.

## 7. Stable authority identity and dynamic evidence identity are separate

This distinction is normative.

`authority_digest` commits only stable current-authority semantics:

- exact current operational-policy-context qualification identity;
- exact current state-projection identity; and
- exact current freshness snapshot identity.

It intentionally does **not** commit the fresh challenge/context-coverage proof instance.

Therefore re-probing and re-verifying the exact same policy context and exact same current authority generation does not mint a new stable authority domain.

`evidence_digest` separately commits:

- the stable `authority_digest`; and
- the exact #96 context-bound coverage digest.

A new challenge/source/witness verification therefore changes evidence provenance while preserving stable current-authority identity when semantics and generation are unchanged.

## 8. Verification ref is evidence metadata, not authority identity

The projected `VerifiedAuthorityFreshness.verification_ref` is rewritten to a deterministic ref derived from `evidence_digest`.

This preserves the exact current proof lineage without contaminating the stable freshness snapshot or stable operational authority identity.

## 9. Policy or generation changes change stable authority identity

Changing any of the following changes `authority_digest`:

- the qualified operational policy context;
- target policy-currentness bundle;
- current state projection/lineage; or
- current freshness snapshot/generation/state.

Thus a policy rotation or authority generation change cannot silently inherit the previous operational authority identity.

## 10. Closed-set composition re-runs #74

`qualify_operational_freshness_set` requires an explicit non-empty required subject set and an exactly same-sized set of qualified operational proofs.

Every projected freshness receipt is revalidated and then passed through #74 `qualify_current_freshness`.

Missing, unexpected, ambiguous, stale, revoked or superseded subjects deny.

## 11. One bootstrap-root epoch per closed set

All qualified subjects in one operational freshness set must bind the exact same bootstrap-root qualification digest/profile.

This prevents a single execution domain from mixing authority proofs from different constitutional/root epochs.

Different subjects may legitimately use different operational policy contexts under that same root.

## 12. Set identity also commits policy-qualified authority identities

#74's freshness bundle commits current snapshots. That is necessary but not sufficient for the final operational qualification identity because two valid policy paths could potentially yield the same authority snapshots.

The stable operational set digest therefore additionally commits every per-subject stable `authority_digest`, canonically ordered by the exact subject identity digest.

Input order cannot change the set identity.

A policy-context change cannot hide behind an unchanged snapshot bundle.

## 13. Duplicate subject proofs deny

After canonical subject-identity ordering, duplicate subject identities are rejected explicitly.

This is defense in depth above #74's exact required-set validation.

## 14. No DHT/currentness heuristic

This crate contains no HDK/Holochain calls and cannot establish currentness from:

- highest observed generation;
- latest timestamp;
- DHT arrival order;
- absence of later records;
- local cache state;
- source reputation;
- Phi/consciousness/reputation; or
- model output.

Currentness is inherited only through the qualified policy context plus #96 covered source head plus #91 complete lineage.

## 15. No execution authority by existence

`QualifiedOperationalSubjectFreshness` and `QualifiedOperationalFreshnessSet` are serializable but not deserializable into positive authority.

A runtime adapter may project a qualified subject to the existing `VerifiedAuthorityFreshness` ABI only after the pure qualifier succeeds.

Neither object itself executes actions, signs threshold decisions, creates grants, claims lifecycle work, or performs external effects.

## 16. Pre-production qualification

At minimum:

- rustfmt;
- native tests;
- warnings-denied Clippy;
- downstream compile of #116/#96/#91/#74;
- wrong target probe denial;
- wrong policy-context probe denial;
- hidden later revocation denial;
- source-head omission denial;
- fork/gap denial;
- stale operational context denial;
- root/policy rotation lease denial;
- same-generation new-probe test proving stable authority digest unchanged while evidence digest changes;
- generation-change test proving stable authority digest changes;
- closed-set input-order invariance;
- mixed-root denial; and
- revoked subject denial through #74.

No `authority_current_freshness_verifier` runtime should be provisioned until this pure chain and its upstream dependencies have executable green evidence.
