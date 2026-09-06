# Authority-State Witness Verifier Runtime v0.2 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Source authentication remains external

Both endpoints accept the established `VerifiedCoverageChallenge + VerifiedAuthoritySourceHead` request. This runtime exact-binds those inputs but does not claim to authenticate the source itself. `source_head_authenticated_here = false` remains explicit.

## 2. Context and witness discovery are candidate-only

Context lookup is constrained by the exact context digest/profile already committed by the verified challenge. Witness discovery returns raw `AuthorityHeadWitnessObservation` candidates only. Neither discovery role grants authority.

## 3. Observation authentication and trust classification remain separate

Each observation is locally digested before the independent observation-proof verifier runs. Trust classification receives only observer ID, exact witness-trust policy and designated verifier; it receives no requested trust-domain ID/ref and no observation bytes.

## 4. #168 qualification is mandatory and local

Only after both proof calls does the coordinator sample host time and run `qualify_witness_evidence`. #168 returns a non-deserializable `QualifiedWitnessEvidence` carrying its exact dynamic horizon.

## 5. Legacy projection remains fail-closed

`verify_witness_evidence` preserves the historical unleased ABI. For every qualified witness it denies if:

`observation.expires_at_ms > qualified.valid_until_ms()`.

Thus an old consumer can never silently reuse a witness longer than the proof evidence carried by the old ABI can represent.

## 6. Leased projection preserves short verifier horizons

`verify_witness_evidence_leased` returns:

`LeasedEvidence<VerifiedWitnessEvidenceBundle>`.

The bundle lease is the monotone intersection of the base challenge/source/context window and every `QualifiedWitnessEvidence` lease. Verification time only moves later; validity only moves earlier.

For DirectSource, no witness provider is contacted and the envelope is conservatively bounded by challenge, source-attestation and context semantic horizons. The independently authenticated source-proof lease remains a separate source-head envelope to be intersected by the caller.

## 7. Trust binding remains local; quorum remains downstream

Neither proof verifier can construct `VerifiedWitnessTrustBinding`; it is projected only from local #168 qualification. This runtime does not decide quorum sufficiency, trust-domain diversity or concentration. #94/#96 remain authoritative for those rules.

## 8. Lease envelope is dynamic evidence, not authority

Deserializing the leased bundle grants no currentness. The consuming current-freshness runtime must still run #94/#96/#91 and cap reusable authority to the envelope lease.

## 9. Fail closed across composition time

The base lease is revalidated after witness proof calls. Each qualified witness lease is created and intersected using a final host-time sample. If any dependency expires while the bundle is being composed, the leased endpoint denies rather than returning already-stale evidence.

## 10. Deliberately unprovisioned

Workspace-only, absent from `dna.yaml`, no external effects and `operational = false`.
