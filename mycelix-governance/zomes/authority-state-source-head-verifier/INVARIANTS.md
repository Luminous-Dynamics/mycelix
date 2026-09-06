# Authority Source-Head Runtime v0.1 — Normative Invariants

Status: **implemented adapter candidate; deliberately unprovisioned in the binding governance DNA**

This zome composes three independent runtime roles around the pure #130 kernel:

1. private-entropy probe re-verification through #114;
2. source response acquisition; and
3. cryptographic source-proof verification.

It does not decide institutional source trust or source-head completeness.

## 1. Caller input is a probe reference, not positive verification

The public `verify_source_head` boundary accepts only one `probe_action: ActionHash`.

The caller MUST NOT be able to submit `VerifiedCoverageChallenge` as positive authority-shaped input.

The coordinator MUST call:

`authority_state_challenge::verify_issued_authority_state_probe`

and obtain the `VerifiedCoverageChallenge` locally before contacting the source responder.

A structurally self-consistent serialized challenge receipt is insufficient. The private-entropy/provenance verifier must reconstruct it from the referenced probe.

## 2. Positive challenge verification is not exposed to the responder

The source responder receives only the underlying immutable `CoverageChallenge` from the locally reverified receipt.

It MUST NOT receive the surrounding `VerifiedCoverageChallenge` or treat positive probe verification as source authority.

The responder needs only enough information to answer the exact challenge. Whether the probe provenance was positively verified is a fact owned by this coordinator/#114, not by the source responder.

## 3. Responder output is candidate data

`authority_state_source_responder::resolve_source_head_attestation` may return one `AuthoritySourceHeadAttestation` candidate only.

The responder MUST NOT return `VerifiedAuthoritySourceHead` or `VerifiedSourceHeadProof` as authoritative output.

A source is not allowed to certify its own authentication merely because it authored the response.

## 4. Cryptographic proof verification is separate

The exact locally reverified challenge digest, exact attestation digest/profile, and exact full attestation are sent to:

`authority_source_proof_verifier::verify_source_head_proof`.

That verifier returns `VerifiedSourceHeadProof` only.

Its output still does not decide institutional source trust or completeness.

## 5. Pure qualification is mandatory

The only positive path is:

`probe ActionHash`
→ #114 `verify_issued_authority_state_probe`
→ `VerifiedCoverageChallenge`
→ plain `CoverageChallenge` to responder
→ candidate `AuthoritySourceHeadAttestation`
→ independent `VerifiedSourceHeadProof`
→ `qualify_source_head_authentication`
→ projection-safety check
→ `VerifiedAuthoritySourceHead`.

The coordinator MUST NOT assemble `VerifiedAuthoritySourceHead` directly from provider fields.

## 6. Validity horizons are monotone under projection

#130 computes an effective validity horizon equal to the minimum of:

- challenge expiry;
- source-attestation expiry; and
- cryptographic-verifier validity.

The legacy `VerifiedAuthoritySourceHead` ABI does not carry that independent `valid_until_ms`; it carries only the source attestation, including `attestation.expires_at_ms`.

Therefore this runtime MUST NOT project a qualified result when:

`attestation.expires_at_ms > qualified.valid_until_ms()`.

Doing so would discard a tighter challenge/verifier horizon and allow a downstream composer to reuse the projected source head after the authentication evidence that justified it had expired.

Until a future ABI carries the explicit verifier horizon, lossy projection MUST deny rather than widen authority.

General rule:

> A composer or projection may preserve or shorten an evidence lease; it must never lengthen it.

## 7. Qualification time follows evidence production

The coordinator MUST sample the `now_ms` supplied to #130 only after #114 probe verification, source response acquisition and source-proof verification have completed.

A timestamp captured before those calls would make freshly produced `verified_at_ms` or `responded_at_ms` values appear to be in the future and could spuriously deny valid evidence.

This ordering is part of the fail-closed causal contract, not an implementation detail.

## 8. Authentication is not institutional trust

This runtime deliberately receives no `AuthorityCoveragePolicy`.

The authenticated source identity and logical source reference are compared against current institutional policy only later in #94/#96.

A cryptographically valid response from an untrusted source must still be rejected later.

## 9. Authentication is not completeness

This runtime does not assert that the authenticated head is the latest or complete authority state.

It contains no DHT lookup, no latest-record selector, no transition projection and no witness quorum logic.

#94/#96/#91 remain responsible for coverage and exact endpoint equality.

Probe-verifier failure, source refusal, network failure, responder outage, proof-verifier outage, decode failure, failed pure qualification, or lossy validity projection MUST deny.

## 10. Exact challenge is reconstructed once and carried without authority leakage

After #114 returns `VerifiedCoverageChallenge`, the coordinator recomputes the exact challenge identity.

Only its underlying `CoverageChallenge` is sent to the responder. The proof-verifier request binds the exact challenge digest, attestation digest/profile and full candidate attestation. #130 then rechecks the challenge receipt, attestation challenge binding, subject, source identity, proof ref and causal time ordering.

No provider may substitute another challenge after local probe verification.

## 11. No provider self-declaration of currentness

Neither the probe verifier, responder nor proof verifier can claim:

- current policy authority;
- source completeness;
- no later revocation;
- witness independence;
- lifecycle authority; or
- external-effect permission.

## 12. Status is declarative only

`source_head_runtime_status` performs no synthetic provider call and reports:

- caller-supplied verified challenge accepted = false;
- probe reverified locally = true;
- verified challenge exposed to responder = false;
- source response candidate-only = true;
- proof verifier separate = true;
- validity horizon widening allowed = false;
- institutional source trust decided here = false;
- completeness decided here = false;
- external effects enabled = false; and
- operational = false.

## 13. Deliberately unprovisioned

The zome is included in the Rust workspace only for native/Clippy/WASM qualification.

`mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_state_source_head_verifier` in this tranche.

The probe verifier, source responder and source-proof verifier are not production-qualified here.

## 14. Required qualification

Before provisioning:

- rustfmt;
- native check and warnings-denied Clippy;
- WASM check;
- static proof that `VerifySourceHeadRequest` cannot contain `VerifiedCoverageChallenge`;
- static proof that #114 probe verification precedes responder/proof verification;
- static proof that responder receives plain `CoverageChallenge`, not positive verification;
- static proof that qualification time is sampled after all evidence-producing calls;
- static proof that a tighter #130 validity horizon cannot be discarded during projection;
- forged caller-supplied challenge receipt rejection;
- probe-verifier outage denial;
- responder outage denial;
- proof-verifier outage denial;
- wrong challenge/attestation/source identity/proof denial;
- causal-order tests from #130;
- proof that source policy is not decided here;
- proof that DHT absence/completeness is not decided here; and
- end-to-end integration with #129 under adversarial source/witness conditions.

No external effects are enabled.
