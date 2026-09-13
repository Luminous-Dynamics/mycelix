# Administrative Review Policy Identity v0.1 Invariants

This crate adds canonical semantic identity for ADMIN-003 administrative review policies without changing the ADMIN-003 review/finality kernel.

## Core theorem

`review-policy locator != review-policy semantic content != verified content identity != institutional adoption != current governing review policy != review authority != external-effect authority`

A successful `QualifiedAdministrativeReviewPolicyIdentity` proves only that one supplied `AdministrativeReviewPolicy` carries the registered digest/profile for its own semantic review obligations.

It does not prove that the policy was adopted, is current, is legally valid, that a reviewer is competent under it, that no timely challenge exists, that a decision is administratively final, or that any external effect is permitted.

## Canonical profile

Profile:

`mycelix-administrative-review-policy-v1-blake3-framed-semantic`

Domain:

`mycelix/administrative-review/policy/v1`

The digest binds, in fixed order:

1. identity profile;
2. ADMIN-003 protocol version;
3. procedure profile;
4. source institution;
5. optional source jurisdiction;
6. exact source rulebook ID/version/digest;
7. review forum;
8. optional review jurisdiction;
9. exact review rulebook ID/version/digest;
10. review capability;
11. stay capability;
12. remedy capability;
13. exact accepted-review-role set in canonical byte order;
14. challenge window;
15. appeal window;
16. finality delay;
17. independent-reviewer requirement; and
18. exact allowed-remedy-type set in canonical byte order.

Role and remedy vector order is non-semantic. Duplicate values remain invalid under ADMIN-003 and are rejected before hashing.

## Deliberately excluded

`policy_ref` is provenance/locator metadata. Mirroring or moving the same policy record does not alter the semantic policy identity.

`policy_digest` and `policy_digest_profile` are the identity claim being checked. They cannot be inputs to their own digest.

## ADMIN-003 remains authoritative for structural validity

Canonicalization does not create alternate encodings for structurally invalid ADMIN-003 policies. The identity layer first reuses `AdministrativeReviewPolicy::validate()` with a non-zero sentinel digest and the registered profile substituted only for the circular identity-claim fields.

Therefore malformed identifiers, invalid rulebooks, invalid windows, duplicate review roles, duplicate remedy types, invalid profile fields, and other ADMIN-003 structural errors remain errors rather than becoming hashable variants.

## Positive typestate

`qualify_administrative_review_policy_identity()` requires the registered profile plus the exact recomputed semantic digest.

The resulting token is deliberately not Clone/Serialize/Deserialize. Persisted policy claims must be requalified from exact content.

The token grants neither review authority nor external-effect authority.

## Adoption and currentness remain separate

A matching digest proves only semantic identity.

A later institutional-adoption theorem must bind this exact digest/profile to independently verified adoption authority/evidence. A later currentness theorem must then prove which adopted identity is governing for the relevant scope and time.

Thus:

`digest matches content != institution adopted content != content is current`.

This mirrors the existing authority-policy proof-domain separation without making public administration depend on the authority-operational runtime subsystem.

## Closed-world finality boundary

ADMIN-003 correctly refuses to infer unchallenged administrative finality from an empty local challenge list. This identity tranche does not change that.

Before an uncontested-finality theorem can exist, it must bind at least:

- an exact qualified review-policy identity;
- independently qualified policy adoption/currentness;
- the exact decision identity;
- the policy-defined challenge-window origin and deadline;
- an authoritative challenge-registry namespace/state; and
- positive closed-world coverage through the deadline.

`no local challenge observed != no timely challenge exists`.

## Challenge-clock limitation

ADMIN-003 v0.1 currently calculates challenge timeliness from `decision.decided_at_ms`. It does not prove a service-of-final-decision clock.

No later layer may silently reinterpret the v0.1 review policy as service-based. A service-aware challenge clock requires an explicit later profile/theorem rather than changing the meaning of this identity profile.

## Parent preservation

This tranche is rooted directly at ADMIN-003 clean candidate `34da42444383de796dd50437d9470856f3f1b565`.

CI pins the exact administrative-procedure manifest, ADMIN-003 semantic implementation, hardened review facade, and hardened public facade blobs. Review-policy identity is additive evidence; it cannot silently rewrite the review/finality theorem beneath it.

The parent self-verifying repair passed ADMIN-003 tests and warnings-denied Clippy before producing the product repair. The ordinary read-only exact-head ADMIN-003 workflow is still the promotion gate and remains independently required.
