# Authority Witness Verifier v0.1 — Normative Invariants

Status: **pure qualification candidate; not provisioned**

## 1. Observation authentication is not trust classification

One exact `AuthorityHeadWitnessObservation` is candidate evidence only.

`VerifiedWitnessObservationProof` may prove only:

- exact observation identity digest/profile;
- exact observer identity;
- exact observation proof reference; and
- bounded verifier timing.

It MUST NOT select or certify an institutional trust domain.

`VerifiedWitnessTrustClassificationProof` may prove only the observer-to-domain classification under one exact witness-trust policy and designated trust verifier.

It MUST NOT authenticate the witness observation or claim source-head/currentness authority.

## 2. Classification is independent of observation semantics

The trust classifier receives/classifies an observer under the exact current trust policy. It does not need authority over the source head, challenge or witness observation bytes.

The pure composer locally binds the independently verified classification to the exact observation through #96's `WitnessTrustBinding` identity.

Thus:

`observation authenticity ≠ observer trust classification ≠ quorum acceptance`.

## 3. Exact source-head and challenge binding

Before compatibility projection, the pure kernel recomputes and requires exact agreement on:

- observation subject;
- authoritative source ref;
- source-head digest;
- head generation;
- head transition digest;
- status-record ref;
- challenge digest; and
- causal `source responded <= witness observed` ordering.

The observation cannot be rebound to another authenticated source head or challenge.

## 4. Trust policy and verifier are exact

The context policy must contain an exact witness-trust policy and designated verifier.

The classification receipt must bind exactly that policy and verifier plus the same observer and trust-domain ID carried by the authenticated observation.

The trust-domain reference is supplied by the classification verifier and becomes part of the locally constructed #96 trust binding.

## 5. Compatibility projection cannot widen verifier leases

The legacy `VerifiedAuthorityHeadWitness` ABI carries the signed observation expiry but no independent observation-verifier or classification-verifier expiry.

Therefore positive projection is forbidden if:

`observation.expires_at_ms > min(observation_proof.valid_until_ms, classification_proof.valid_until_ms)`.

A compatibility adapter may preserve or shorten an evidence lease; it must never lengthen it.

## 6. #96 trust binding is constructed locally

Neither verifier may return `VerifiedWitnessTrustBinding`.

After exact proof qualification the kernel locally constructs `WitnessTrustBinding` from:

- current context-policy digest/profile;
- current witness-trust policy;
- exact challenge digest;
- exact observation digest;
- exact observer ID;
- independently classified trust-domain ID/ref; and
- exact classification proof ref.

It then projects `VerifiedWitnessTrustBinding` with a validity horizon equal to the conservative minimum of all relevant proof/observation/challenge/context horizons.

## 7. Stable identity vs dynamic evidence

Stable witness qualification commits the exact context, challenge, authenticated source head, observation and local trust binding.

Dynamic evidence additionally commits observation-verifier and trust-classification-verifier references/timestamps/horizons.

Refreshing proofs may change evidence identity without minting a new witness authority domain.

## 8. Quorum remains #94/#96's job

This kernel qualifies one witness only.

It does not count witnesses, classify independence across the full set, enforce trust-domain concentration, or decide whether quorum is sufficient.

#94/#96 remain authoritative for exact witness-set closure, minimum witnesses, minimum independent trust domains and maximum contribution per trust domain.

## 9. Positive result is non-deserializable

`QualifiedWitnessEvidence` derives `Serialize` but not `Deserialize`.

The existing deserializable #94/#96 receipts are compatibility projections only after successful local pure qualification.

## 10. Containment

This crate contains no HDK calls, DHT reads, persistence, discovery, policy-currentness lookup, lifecycle mutation, external effect, reputation, Phi or model-score authority.

Before runtime provisioning, candidate discovery, observation-proof verification and trust-classification verification must remain independent roles and the composer must run this kernel locally.
