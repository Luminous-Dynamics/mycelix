# Mycelix Authority State Coverage Context v0.1 — Normative Invariants

Status: **pure policy/challenge/trust composition layer; not a randomness source, trust registry, signature verifier, or Holochain runtime**

This crate closes the two trust-context gaps intentionally left by the lower-level source-head coverage kernel:

1. a fresh challenge must belong to one exact institution-adopted coverage-policy context; and
2. witness observer/trust-domain classification must belong to one exact institution-adopted trust policy and exact witness observation.

## 1. Exact trust-context policy

`CoverageTrustContextPolicy` binds:

- exact institution / optional jurisdiction / rulebook references;
- exact `AuthorityCoveragePolicy` digest/profile;
- when witnesses are used, exact witness-trust policy digest/profile;
- exact logical witness-trust verifier identity;
- maximum challenge lifetime;
- exact context validity interval; and
- exact institutional authority/proof adopting this context.

A context object is candidate data. `VerifiedCoverageTrustContextPolicy` must independently verify the exact authority and proof before the pure qualifier accepts it.

## 2. DirectSource and WitnessQuorum have different trust contexts

If the referenced coverage policy is `DirectSource`, witness-trust policy/verifier state is forbidden and trust-binding evidence is rejected.

If the referenced coverage policy is `WitnessQuorum`, an exact witness-trust policy and verifier are mandatory.

Changing from DirectSource to WitnessQuorum therefore changes the coverage-policy identity and cannot reuse the same context silently.

## 3. Fresh challenge is bound to exact policy context

`CoverageChallenge` commits:

- exact context-policy digest/profile;
- exact coverage-policy digest/profile;
- exact authority subject;
- exact nonce digest;
- exact randomness-proof reference;
- issue/expiry time; and
- exact challenge issuer.

The source/witness coverage layer receives only this challenge's exact identity digest.

A challenge generated for another context, policy, subject, nonce, randomness proof, or issuer cannot qualify this coverage attempt.

## 4. Randomness verification is exact

`VerifiedCoverageChallenge` must echo the exact:

- nonce digest;
- randomness-proof reference; and
- challenge issuer.

A host/runtime may not prove random material A and attach that proof to nonce B.

The pure layer does not claim a random source is trustworthy merely because a proof string is non-empty. The verifier boundary that constructs the receipt must independently validate the approved randomness source/proof.

## 5. Challenge lifetime is authority-bounded

Challenge issuance must occur inside the exact context-policy validity interval.

Challenge expiry may not exceed context-policy expiry or `max_challenge_lifetime_ms`.

The verifier cannot qualify a challenge before it exists or after it expires.

A long-lived challenge is not current-state coverage evidence.

## 6. Witness trust policy is identity-bearing

A witness trust policy is an exact `ProfiledDigest`, not a label.

Changing trust-framework rules, accepted issuers/registries, classification semantics, independence rules, or policy bytes must change the trust-policy digest/profile and therefore the context identity.

The logical `witness_trust_verifier_ref` is also identity-bearing context.

## 7. Trust classification binds one exact witness observation

`WitnessTrustBinding` commits:

- exact context-policy digest/profile;
- exact witness-trust policy digest/profile;
- exact challenge digest;
- exact witness observation identity digest;
- exact observer ID;
- exact trust-domain ID/ref; and
- exact classification proof reference.

A classification for one observation cannot be replayed onto a different observation from the same observer.

A classification under one trust policy cannot be replayed under another coverage context.

## 8. Trust verifier itself is explicit

Every `VerifiedWitnessTrustBinding` must come from the exact logical trust verifier named by the adopted context.

A different verifier cannot silently reclassify the same observer into another independence domain while preserving current coverage authority.

The runtime adapter must call that authoritative verifier directly; caller-constructed receipt bytes are not self-authenticating authority.

## 9. Witness and trust-binding sets are exact

Witness observations and trust bindings form an exact closed set keyed by observer identity.

The qualifier rejects:

- duplicate witness observers;
- duplicate trust bindings;
- missing bindings;
- extra bindings;
- mismatched observation digest;
- mismatched observer ID;
- mismatched trust-domain ID/ref;
- wrong trust policy/context/challenge; and
- stale trust classification.

Input ordering cannot create or remove a binding.

## 10. Base coverage still enforces quorum/diversity

This layer does not replace the lower-level coverage kernel.

After trust-context qualification, it still calls `qualify_authority_state_coverage`, which independently enforces:

- exact source identity/head;
- exact fresh challenge;
- witness head equivalence;
- observer uniqueness;
- witness count;
- trust-domain diversity; and
- per-domain concentration limits.

Trust classification proves what a witness/domain means; the base coverage kernel proves the adopted quorum is actually satisfied.

## 11. Context-bound coverage identity

The final stable context-coverage digest commits:

- exact context-policy digest;
- exact challenge digest;
- exact base coverage digest; and
- canonical exact trust-binding digests.

Dynamic verification timestamps and leases are not semantic identity.

Any change to context policy, challenge, source head, witness set, witness classification, or underlying coverage policy changes current coverage identity.

## 12. Context can only shrink reusable validity

The final reusable lease is no later than the minimum of:

- base source/witness coverage lease;
- context-policy expiry;
- challenge expiry; and
- every trust-binding validity horizon.

Refreshing proof evidence can never extend authority beyond a semantic dependency's own validity.

## 13. Output remains compatible with the state-source kernel

Successful context qualification outputs the same `VerifiedAuthorityStateCoverage` ABI consumed by the append-only authority-state projector.

Its proof/verification references are replaced with deterministic context-coverage references, and its verification/lease window is conservatively narrowed.

The state-source projector still requires the transition lineage endpoint to equal the covered head exactly.

## 14. Historical validity remains separate from live coverage

This crate produces live coverage evidence only.

Historical/as-of state verification must use the historical source/proof semantics defined by the state-source lineage and may not convert an expired historical challenge or trust classification into new current authority.

## 15. No DHT or local-observation shortcut

A runtime must not bypass this context because it has locally observed the DHT, source chain, or witness records.

`No newer record observed` remains insufficient proof of current authority.

The coverage proof must remain bound to the exact adopted context and fresh challenge.

## 16. No advisory authority

Phi, consciousness, reputation, stake, Guardian labels, model recommendations, caller identity, observer count alone, timestamps, or local network topology cannot:

- mint a challenge;
- qualify randomness;
- choose a trust verifier;
- classify trust domains;
- alter witness policy;
- repair missing evidence; or
- override a failed coverage qualification.

## 17. Runtime requirements before provisioning

A future runtime adapter must independently provide:

- institution-authorized context-policy verification;
- approved unpredictable challenge generation and exact proof binding;
- direct verification of the configured source identity;
- direct witness-proof verification;
- direct witness trust-domain classification under the configured trust policy;
- bounded leases/current-state checks; and
- fail-closed provider outage/ambiguity semantics.

Only then may it invoke this pure composition layer.

## 18. External effects remain disabled

Coverage context establishes freshness/completeness evidence only. It does not create grants, proposals, execution authority, lifecycle claims, or external effects.
