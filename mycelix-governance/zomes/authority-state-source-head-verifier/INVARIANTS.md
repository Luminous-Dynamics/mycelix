# Authority Source-Head Runtime v0.2 — Normative Invariants

Status: **implemented adapter candidate; deliberately unprovisioned in the binding governance DNA**

This zome composes three independent runtime roles around the pure #130 kernel: private-entropy probe re-verification, source response acquisition, and cryptographic source-proof verification. It does not decide institutional source trust or completeness.

## 1. Caller input is a probe reference, not positive verification

Both public verification endpoints accept only `VerifySourceHeadRequest { probe_action }`. Caller-supplied `VerifiedCoverageChallenge` is forbidden. The coordinator reconstructs it through `authority_state_challenge::verify_issued_authority_state_probe` before contacting the responder.

## 2. Positive challenge verification is not exposed to the responder

The source responder receives only the underlying immutable `CoverageChallenge`, never the surrounding positive `VerifiedCoverageChallenge`.

## 3. Responder output is candidate data

`authority_state_source_responder::resolve_source_head_attestation` returns candidate `AuthoritySourceHeadAttestation` bytes only. A source cannot certify its own authentication.

## 4. Cryptographic proof verification is separate

The exact locally reverified challenge digest and exact candidate attestation identity are sent to `authority_source_proof_verifier::verify_source_head_proof`. That verifier returns `VerifiedSourceHeadProof` only.

## 5. Pure qualification is shared by both projection surfaces

Both endpoints must pass through one `qualify_request` path:

`probe -> #114 -> candidate response -> independent source proof -> #130 qualification`.

The coordinator never constructs `VerifiedAuthoritySourceHead` directly.

## 6. Legacy projection remains fail-closed

The historical `verify_source_head` endpoint returns only `VerifiedAuthoritySourceHead`, whose ABI cannot encode #130's independent verifier horizon.

It MUST continue to deny when `attestation.expires_at_ms > qualified.valid_until_ms()`.

Existing unleased callers therefore retain the old no-validity-widening guarantee.

## 7. Leased projection preserves the exact dynamic horizon

`verify_source_head_leased` returns:

`LeasedEvidence<VerifiedAuthoritySourceHead>`.

Its `EvidenceLease` MUST be constructed from exactly:

- `qualified.verified_at_ms()`; and
- `qualified.valid_until_ms()`.

The leased endpoint MAY transport a semantic attestation whose own expiry is later than the verifier horizon because the tighter horizon is no longer discarded. Any downstream reusable authority MUST remain bounded by the envelope lease.

This is not authority widening: semantic identity remains unchanged while dynamic proof validity is carried separately.

## 8. Qualification time follows evidence production

The `now_ms` supplied to #130 and to `EvidenceLease::new` is sampled only after probe verification, source response, and source-proof verification complete.

## 9. Authentication is not institutional trust or completeness

This runtime receives no `AuthorityCoveragePolicy`, performs no witness quorum and no transition projection, and cannot assert current authority or absence of later revocation. #94/#96/#91 remain responsible for those facts.

## 10. Lease envelope is not authority

`LeasedEvidence` is transportable. Deserialization proves nothing. A consuming runtime must still run its domain-specific pure qualification and use the lease only as an additional upper bound.

## 11. Status is declarative only

Status reports old projection fail-closed, leased endpoint available, exact-horizon preservation, source trust/completeness false, external effects false, and `operational = false`. It performs no provider probes.

## 12. Deliberately unprovisioned

The zome remains workspace-only and absent from `dna.yaml`. No external effect is enabled.
