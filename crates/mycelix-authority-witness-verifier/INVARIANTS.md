# Authority Witness Verifier v0.2 — Normative Invariants

Status: **pure qualification candidate; not provisioned**

## 1. Observation authentication is not trust classification

`VerifiedWitnessObservationProof` proves only one exact observation/observer/proof relation with bounded verifier timing. It MUST NOT classify an institutional trust domain.

`VerifiedWitnessTrustClassificationProof` proves only the observer-to-domain relation under one exact witness-trust policy and designated verifier. It MUST NOT authenticate the observation or claim source-head/currentness authority.

Thus:

`observation authenticity ≠ observer trust classification ≠ quorum acceptance`.

## 2. Exact source-head and challenge binding

The pure kernel recomputes and requires exact observation subject, authoritative source, source-head digest, generation, transition digest, status-record ref, challenge digest and causal `source responded <= witness observed` ordering.

The observation cannot be rebound to another authenticated source head or challenge.

## 3. Trust policy and verifier are exact

The context must contain an exact witness-trust policy and designated verifier. The classification receipt must bind that policy/verifier plus the same observer and trust-domain ID carried by the authenticated observation.

## 4. Qualification preserves the true verifier horizon

The non-deserializable `QualifiedWitnessEvidence` carries:

`valid_until = min(observation expiry, observation-proof horizon, classification-proof horizon, challenge expiry, context expiry)`.

A shorter verifier horizon does **not** invalidate an otherwise correct witness qualification at the current instant. Instead the shorter horizon remains explicit dynamic evidence.

This separates two theorems:

- exact witness qualification is valid now; and
- a particular transport/projection can safely preserve that validity horizon.

Legacy transports that omit the horizon must still deny lossy projection. Lease-complete transports may carry it explicitly.

## 5. #96 trust binding is constructed locally

Neither verifier may return `VerifiedWitnessTrustBinding`. The kernel locally constructs the exact binding from current context, witness-trust policy, challenge, observation identity, observer, independently classified domain and exact classification proof.

The projected trust binding carries the conservative `valid_until_ms` directly.

## 6. Stable identity vs dynamic evidence

Stable witness qualification commits exact context, challenge, authenticated source head, observation and local trust binding. Dynamic evidence additionally commits verifier identities/timestamps/horizons.

Refreshing proof evidence may change evidence identity and lease without minting a new semantic witness authority domain.

## 7. Quorum remains #94/#96

This kernel qualifies one witness only. It does not decide minimum count, trust-domain diversity, concentration or closed-set sufficiency.

## 8. Positive result is non-deserializable

`QualifiedWitnessEvidence` derives `Serialize` but not `Deserialize`. Compatibility receipts are projections only after successful local qualification.

## 9. Containment

Pure Rust only. No HDK/DHT, persistence, discovery, policy-currentness lookup, lifecycle mutation, external effects, reputation, Phi or model-score authority.
