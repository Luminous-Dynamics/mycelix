# Authority-State Context Policy Verifier Runtime v0.1 — Normative Invariants

Status: **unprovisioned Holochain composition adapter; no policy registry, no authority-state source, no external effects**

This coordinator exists only to compose already-qualified providers into the pure PR #103 challenge-context kernel.

## 1. No local policy truth

This zome MUST NOT persist or mutate coverage policy, context policy, witness-trust policy, issuer grants, current freshness, or lifecycle authority.

It is not a registry and has no `current=true` state.

## 2. Explicit provider split

Positive context requires explicit local provider calls for:

- context policy: `authority_coverage_context_verifier::resolve_context_policy`;
- coverage policy: `authority_coverage_policy_verifier::resolve_coverage_policy`;
- challenge issuer grant: `authority_challenge_issuer_grant_verifier::resolve_challenge_issuer_grant`;
- witness trust policy when required: `authority_witness_trust_policy_verifier::resolve_witness_trust_policy`; and
- current freshness: `authority_current_freshness_verifier::resolve_current_freshness`.

Missing zome, missing function, non-Ok response, decode failure, or no required semantic authority MUST deny.

## 3. Subject-specific resolution only

The runtime never probes providers using a synthetic/sentinel subject merely to report status or infer provider availability.

`context_verifier_runtime_status` is declarative and always reports `operational = false` in this unprovisioned tranche.

Only `resolve_challenge_context(exact_subject)` may attempt authority qualification.

## 4. Freshness provider does not choose the authority set

The adapter deterministically derives a preliminary requested freshness set from exact semantic receipts.

That request is convenience, not authority.

The returned freshness receipts remain candidate evidence. The adapter MUST pass them into PR #103 `qualify_challenge_context`, which reconstructs the required set independently and denies:

- missing subjects;
- unexpected subjects;
- ambiguous current snapshots;
- inactive generations;
- stale leases; or
- exact identity mismatches.

The freshness provider cannot broaden authority by returning extra receipts.

## 5. Required policy identities

The preliminary request uses the same exact canonical identities expected by the pure kernel:

- `AuthorityCoveragePolicy` using canonical coverage-policy digest/profile;
- `CoverageTrustContextPolicy` using canonical context-policy digest/profile;
- exact issuer `AuthorityGrant` subject supplied by the independent issuer-grant verifier; and
- when present, `WitnessTrustPolicy` using the exact trust-policy digest/profile.

Textual policy IDs alone are never current authority.

## 6. Witness provider is conditional but not advisory

If the context names witness-trust policy, the witness policy provider is required.

If semantic coverage mode/context disagree, the final pure qualifier denies even if the adapter happened to fetch a witness receipt.

The adapter cannot make DirectSource/WitnessQuorum policy decisions independently of the pure kernel.

## 7. Exact pure qualification is mandatory

The only positive path is:

semantic provider receipts
→ exact current freshness receipts
→ `qualify_challenge_context`
→ projection into the #99 wire DTO.

The adapter MUST NOT construct `VerifiedChallengeContextWire` directly from provider fields without a successful pure `QualifiedChallengeContext`.

## 8. Wire projection cannot expand authority

The #99 wire DTO contains only the subset needed by the challenge issuer:

- protocol;
- exact subject;
- exact context/coverage policy identities;
- bounded challenge lifetime;
- context semantic expiry;
- exact issuer DID;
- deterministic verification ref;
- verification time; and
- conservative validity horizon.

Projection may discard audit detail but cannot lengthen lifetime or replace identity.

## 9. Local calls are not trust equivalence

Providers are called through `CallTargetCell::Local` for deployment composition.

Co-location in one cell does not merge semantic authority. Each provider remains independently responsible for its own proof/trust boundary.

## 10. No runtime provisioning yet

This coordinator is added to the governance Cargo workspace only.

It MUST NOT appear in `mycelix-governance/dna/dna.yaml` in this tranche.

Its upstream semantic/current providers are also intentionally absent, so challenge issuance remains unavailable.

## 11. No external effects

This zome may not:

- execute governance actions;
- create lifecycle claims;
- call external services;
- mutate constitutional state;
- issue credentials; or
- cause any real-world effect.

Its output is evidence for the later private-entropy challenge boundary only.

## 12. Fail closed

Any provider outage, malformed subject, stale receipt, semantic mismatch, policy revocation, wrong grant, wrong trust verifier, ambiguous freshness, or pure qualification failure returns an error.

No fallback to expiry-only policy checks, cached context, caller identity, Phi, reputation, Guardian status, model output, or DHT presence is permitted.
