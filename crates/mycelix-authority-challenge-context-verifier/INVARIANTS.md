# Authority Challenge Context Verifier v0.1 — Normative Invariants

Status: **pure composition kernel; no policy discovery, no Holochain authority source, no DNA provisioning, no external effects**

This kernel exists solely to answer:

> Given exact independently verified semantic policy/issuer receipts and exact current generation-bound freshness evidence, is one subject currently allowed to obtain a freshness challenge from one exact issuer?

It does not decide which policy exists, which grant exists, which generation is current, or whether a signature/proof is valid. Those facts must be verified before entering this boundary.

## 1. Semantic policy and current policy are different facts

A semantically valid/unexpired policy is insufficient live authority.

Positive challenge context requires `Active` current freshness for the exact:

- `AuthorityCoveragePolicy`;
- `CoverageTrustContextPolicy`;
- challenge issuer `AuthorityGrant`; and
- in WitnessQuorum mode, `WitnessTrustPolicy`.

A revoked/superseded required subject denies even when semantic `valid_until` has not elapsed.

## 2. Exact canonical policy identity

Coverage and context freshness subjects are derived from the policy kernels' exact canonical digest/profile, not textual IDs alone.

Same-ID mutation or profile substitution therefore creates a different authority subject.

Witness-trust identity is exact digest/profile. Because the current context format carries no independent witness-policy ID, v0.1 derives a deterministic subject ID from that exact digest; the digest/profile remain the authority identity.

## 3. Stable shared subject kinds

This kernel consumes the additive shared freshness kinds from PR #100:

- `AuthorityCoveragePolicy`;
- `CoverageTrustContextPolicy`; and
- `WitnessTrustPolicy`.

It does not define local replacement enums/codes.

## 4. Coverage policy must equal context policy reference

The exact recomputed `AuthorityCoveragePolicy` digest/profile must equal the policy identity committed by `CoverageTrustContextPolicy`.

Matching IDs or equivalent-looking fields are insufficient.

## 5. Witness mode is closed

`DirectSource` requires no witness-trust policy authority and rejects supplied witness-trust authority.

`WitnessQuorum` requires:

- exact witness-trust policy digest/profile from the context;
- exact institution/jurisdiction/rulebook binding;
- exact trust-verifier identity named by the context;
- bounded semantic verification; and
- current/Active witness-policy freshness.

A proof from another trust-verifier domain cannot be rebound to the same policy digest.

## 6. Challenge issuer authority is explicit

Caller identity does not confer issuance authority.

The issuer receipt must represent an independently verified exact `AuthorityGrant` and bind:

- exact issuer DID;
- institution/jurisdiction/rulebook;
- capability `authority_state.challenge_issue`;
- exact grant freshness subject;
- exact authority/proof refs; and
- bounded validity.

The same exact grant subject must independently qualify as current/Active through freshness.

This crate intentionally does not invent a second grant format or infer authority from policy ownership.

## 7. One closed freshness bundle

`qualify_current_freshness` must receive exactly the required current subjects for the selected mode.

Missing, unexpected, ambiguous, revoked, superseded, or stale freshness denies.

The final context identity commits the resulting freshness bundle digest, so any required generation change changes current challenge authority.

## 8. Re-verification is not semantic mutation

Refreshing semantic proof metadata or freshness leases for unchanged exact policy/grant generations may update dynamic verification timestamps/horizons without changing the stable current context identity.

Changing a semantic policy, issuer grant, witness policy, required generation, subject, or issuer changes the identity.

## 9. Conservative lease

The reusable challenge-context horizon is the minimum of all required live dependencies:

- current freshness bundle lease;
- context semantic validity;
- coverage-policy semantic validity;
- issuer-grant validity; and
- witness-trust policy authority validity when applicable.

The verification time is the maximum of required verification times.

No child context may outlive any authority it depends on.

## 10. Non-deserializable positive authority

`QualifiedChallengeContext` derives `Serialize` but not `Deserialize`.

Callers may not deserialize a positive authority object from arbitrary bytes and bypass qualification.

A runtime adapter may project the exact public fields into the #99 wire receipt only after this kernel succeeds.

## 11. #99 wire compatibility

The positive result exposes exactly the semantics required by the current challenge runtime:

- protocol `mycelix-authority-state-challenge-context-v0.1`;
- exact subject;
- exact context policy digest/profile;
- exact coverage policy digest/profile;
- max challenge lifetime;
- context semantic expiry;
- exact challenge issuer DID;
- deterministic verification ref;
- verification time; and
- conservative live horizon.

Extra audit fields such as freshness/context/issuer-grant identities may be retained by the provider but are not substitutes for those wire fields.

## 12. Historical state cannot mint a live challenge

Historical/as-of policy or grant state is not accepted by this live qualifier.

Later revocation must stop new challenges without rewriting legitimately issued historical challenges.

Historical audit remains a separate protocol.

## 13. No authority from advisory systems

Phi, reputation, stake, consciousness, model outputs, Guardian labels, timestamps, code presence, DHT presence, non-empty refs, or caller identity cannot satisfy this kernel.

They may be evidence considered by a governed institutional process before an authority receipt exists; they are not authority here.

## 14. Runtime containment

A later `authority_state_context_policy_verifier` Holochain coordinator must compose independently verified providers and this pure kernel.

Provider absence, decode failure, stale receipts, unavailable state coverage, wrong trust verifier, wrong issuer grant, or freshness ambiguity must deny.

The verifier must remain unprovisioned in the binding DNA until its semantic-policy, grant, and current-state providers are independently qualified.

No external effect is enabled by this kernel.
