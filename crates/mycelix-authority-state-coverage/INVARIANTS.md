# Mycelix Authority State Coverage v0.1 — Normative Invariants

Status: **pure coverage qualification kernel; not an authority-state source, trust-domain registry, or Holochain runtime**

This crate qualifies whether one exact authority-state source head has enough fresh, independently verified coverage evidence to become the `VerifiedAuthorityStateCoverage` input consumed by `mycelix-authority-state-source`.

It exists because:

> A locally complete transition prefix is not proof that no later revocation exists.

## 1. Coverage is independent authority evidence

`AuthoritySourceHeadAttestation`, witness observations, policy objects, DHT records, timestamps, and non-empty proof strings are candidate evidence.

A positive `QualifiedAuthorityStateCoverage` may be constructed only by `qualify_authority_state_coverage` from:

- an independently verified institution-adopted `AuthorityCoveragePolicy`;
- an independently verified exact source-head attestation;
- when policy requires it, independently verified witness observations; and
- one fresh non-zero verifier challenge.

The pure crate never verifies signatures, chooses institutional policy, generates randomness, or declares a trust domain by itself.

## 2. Exact institution-adopted coverage policy

The policy binds:

- one namespace;
- a closed set of authority-subject kinds;
- one logical authoritative source identity;
- one exact source verification identity/profile;
- DirectSource or WitnessQuorum mode;
- source/witness freshness ceilings;
- maximum reusable coverage lease;
- exact validity interval; and
- exact institutional authority/proof that adopted the policy.

A policy is not current merely because it deserializes. Its host receipt must independently verify the exact policy proof and institutional authority.

## 3. DirectSource is explicit concentrated trust

`CoverageMode::DirectSource` is permitted only when the adopted policy explicitly selects it.

It means a fresh, correctly verified response from the configured logical source is sufficient for coverage. It optimizes liveness while concentrating source-compromise risk.

Witness evidence supplied to DirectSource mode is rejected instead of being silently ignored.

## 4. Witness quorum requires verified independence

`CoverageMode::WitnessQuorum` binds:

- minimum distinct verified observers;
- minimum distinct verified trust domains; and
- maximum accepted observations from any one trust domain.

Distinct observer IDs alone are not evidence of independence.

Every witness receipt must independently echo the exact observer ID and exact trust-domain ID used by the observation. A proof/reference cannot be rebound to a different observer or trust domain.

The pure kernel never infers trust-domain identity from network address, key count, organization name, caller input, reputation, Phi, stake, or model output.

## 5. Fresh challenge is mandatory

Every qualification uses one exact non-zero `challenge_digest` supplied by the verifier runtime.

The source head must bind that exact challenge, and every witness must bind the same challenge and exact source-head identity.

A runtime adapter must generate the challenge unpredictably and must not accept caller-selected stale challenges as freshness proof.

v0.1 source-head identity currently binds the challenge and source semantics. A follow-on context layer SHOULD additionally bind the challenge to the exact coverage-policy identity / witness trust-policy identity before production provisioning, preventing cross-policy replay of an otherwise fresh head response.

## 6. Exact source-head identity

The source response binds:

- exact authority subject;
- exact logical authoritative source;
- exact configured source verification identity/profile;
- bounded generation (1..=256);
- exact head transition digest;
- exact head status-record reference;
- source publication/source-chain head reference;
- exact challenge;
- response/expiry time; and
- exact source proof reference.

The source verifier must verify the proof against the exact configured source identity.

## 7. Source freshness is bounded and policy-relative

The source response must:

- have been produced inside the active coverage policy interval;
- already exist when host verification occurs;
- remain unexpired at qualification time; and
- be no older than the policy's maximum source age.

Policy adoption cannot retroactively bless a source response produced before that policy became effective.

## 8. Witnesses bind the exact source head

A witness observation must bind the same:

- subject;
- logical source;
- source-head identity digest;
- generation;
- transition digest;
- status-record reference; and
- challenge.

A witness saying only “I observed generation N” is insufficient.

Witness observation time cannot predate the source response it claims to observe.

## 9. Observer uniqueness and ambiguity

One observer contributes at most one canonical observation.

Exact duplicate evidence for the same observer/head is harmless.

Different qualified observations from the same observer in one qualification are ambiguous and deny rather than selecting one by timestamp or input order.

## 10. Trust-domain concentration fails closed

The count contributed by any trust domain may not exceed `max_per_trust_domain`.

Meeting `min_witnesses` without meeting `min_trust_domains` denies.

Meeting `min_trust_domains` without meeting `min_witnesses` denies.

No score or confidence value can compensate for insufficient independent coverage.

## 11. Coverage policy must be feasible

Witness-policy values are bounded and internally coherent before evidence is processed.

v0.1 permits at most 64 witness receipts and 32 trust-domain classes in one qualification. This bounds adversarial fan-in and arithmetic.

A runtime trust-policy layer may impose a much smaller accepted trust-domain universe; the pure v0.1 kernel's global maxima are not themselves a trust registry.

## 12. Coverage identity is deterministic

The stable coverage digest commits:

- exact semantic coverage-policy digest;
- exact source-head identity digest;
- exact challenge; and
- canonical witness identity digests.

Witnesses are canonicalized by verified observer identity. Input ordering cannot change the coverage identity.

Dynamic verifier invocation references, verification timestamps, and reusable lease horizons remain evidence and are not part of stable coverage identity.

## 13. Reusable lease can only shrink

The returned coverage lease is bounded by the minimum of:

- source-head expiry;
- policy expiry;
- the policy's maximum coverage lease from the current qualification time; and
- every accepted witness expiry.

Re-verification may refresh evidence only if the underlying head/policy/current authority still qualify.

## 14. Output is exactly the #91 coverage ABI

A successful result can produce `VerifiedAuthorityStateCoverage` for `mycelix-authority-state-source`.

The projection binds the exact:

- subject;
- logical authoritative source;
- head generation;
- head transition digest;
- head status-record reference;
- coverage proof identity;
- verification identity/time; and
- lease horizon.

The state-source projector must still prove that the supplied transition lineage endpoint equals this covered head exactly.

Coverage does not replace generation-chain validation.

## 15. Coverage is not current positive authority

A covered source head may legitimately be Revoked or Superseded.

Coverage proves which source head is authoritative enough to project; it does not make the head Active.

`mycelix-authority-state-source` reconstructs the state, and `mycelix-authority-freshness` decides whether that exact current state can contribute to positive current authority.

## 16. No DHT completeness assumption

A later Holochain adapter MUST NOT manufacture positive coverage merely because a DHT query returned no later records.

Local observation of “nothing newer” is not completeness.

Production coverage must come from an independently verified source-head mechanism satisfying this contract (and, for WitnessQuorum, the adopted independent-observer policy).

## 17. Trust-policy binding remains an explicit follow-on

The v0.1 witness receipt verifies exact observer and trust-domain IDs, but this crate does not itself define the external trust framework that decides which trust-domain verifier is accepted.

Before production provisioning, a follow-on trust-context layer must bind qualification to an institution-adopted trust-domain verification policy/profile, so a witness cannot be reclassified under a different verifier policy while preserving otherwise identical head evidence.

That follow-on should also bind the fresh challenge to the exact coverage-policy/trust-policy context.

Until that layer is qualified, this crate remains a pure draft primitive rather than production coverage authority.

## 18. Failure semantics

Missing or mismatched policy/source/witness proof, stale source/witness evidence, wrong challenge, wrong source identity, observer/trust-domain rebinding, ambiguous observer evidence, excessive fan-in, insufficient quorum/diversity, concentration above policy, expired lease, malformed profile, or unknown subject class is denial.

There is no best-effort coverage fallback.

## 19. Advisory systems have no coverage authority

Phi, consciousness, reputation, stake, Guardian labels, caller identity, optimizer output, or Symthaea/model recommendations cannot:

- select a source head;
- declare an observer independent;
- satisfy witness quorum;
- repair a stale challenge;
- override source mismatch; or
- convert incomplete DHT visibility into coverage.

They may inform a governed policy-adoption process, but only verified policy/source/witness evidence enters this kernel.

## 20. External effects remain disabled

This crate does not provision a Holochain zome, mutate authority state, create lifecycle claims, or execute external effects.

No runtime should treat coverage as effect permission.
