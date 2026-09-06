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

## 2. Responder output is candidate data

`authority_state_source_responder::resolve_source_head_attestation` may return one `AuthoritySourceHeadAttestation` candidate only.

The responder MUST NOT return `VerifiedAuthoritySourceHead` or `VerifiedSourceHeadProof` as authoritative output.

A source is not allowed to certify its own authentication merely because it authored the response.

## 3. Cryptographic proof verification is separate

The exact locally reverified challenge digest, exact attestation digest/profile, and exact full attestation are sent to:

`authority_source_proof_verifier::verify_source_head_proof`.

That verifier returns `VerifiedSourceHeadProof` only.

Its output still does not decide institutional source trust or completeness.

## 4. Pure qualification is mandatory

The only positive path is:

`probe ActionHash`
→ #114 `verify_issued_authority_state_probe`
→ `VerifiedCoverageChallenge`
→ candidate `AuthoritySourceHeadAttestation`
→ independent `VerifiedSourceHeadProof`
→ `qualify_source_head_authentication`
→ `VerifiedAuthoritySourceHead`.

The coordinator MUST NOT assemble `VerifiedAuthoritySourceHead` directly from provider fields.

## 5. Qualification time follows evidence production

The coordinator MUST sample the `now_ms` supplied to #130 only after #114 probe verification, source response acquisition and source-proof verification have completed.

A timestamp captured before those calls would make freshly produced `verified_at_ms` or `responded_at_ms` values appear to be in the future and could spuriously deny valid evidence.

This ordering is part of the fail-closed causal contract, not an implementation detail.

## 6. Authentication is not institutional trust

This runtime deliberately receives no `AuthorityCoveragePolicy`.

The authenticated source identity and logical source reference are compared against current institutional policy only later in #94/#96.

A cryptographically valid response from an untrusted source must still be rejected later.

## 7. Authentication is not completeness

This runtime does not assert that the authenticated head is the latest or complete authority state.

It contains no DHT lookup, no latest-record selector, no transition projection and no witness quorum logic.

#94/#96/#91 remain responsible for coverage and exact endpoint equality.

Probe-verifier failure, source refusal, network failure, responder outage, proof-verifier outage, decode failure, or failed pure qualification MUST deny.

## 8. Exact challenge is reconstructed once and carried unchanged

After #114 returns `VerifiedCoverageChallenge`, the coordinator recomputes the exact challenge identity and sends that exact receipt to the source responder.

The proof-verifier request binds the exact challenge digest, attestation digest/profile and full candidate attestation. #130 then rechecks the challenge receipt, attestation challenge binding, subject, source identity, proof ref and causal time ordering.

No provider may substitute another challenge after local probe verification.

## 9. No provider self-declaration of currentness

Neither the probe verifier, responder nor proof verifier can claim:

- current policy authority;
- source completeness;
- no later revocation;
- witness independence;
- lifecycle authority; or
- external-effect permission.

## 10. Status is declarative only

`source_head_runtime_status` performs no synthetic provider call and reports:

- caller-supplied verified challenge accepted = false;
- probe reverified locally = true;
- source response candidate-only = true;
- proof verifier separate = true;
- institutional source trust decided here = false;
- completeness decided here = false;
- external effects enabled = false; and
- operational = false.

## 11. Deliberately unprovisioned

The zome is included in the Rust workspace only for native/Clippy/WASM qualification.

`mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_state_source_head_verifier` in this tranche.

The probe verifier, source responder and source-proof verifier are not production-qualified here.

## 12. Required qualification

Before provisioning:

- rustfmt;
- native check and warnings-denied Clippy;
- WASM check;
- static proof that `VerifySourceHeadRequest` cannot contain `VerifiedCoverageChallenge`;
- static proof that #114 probe verification precedes responder/proof verification;
- static proof that qualification time is sampled after all evidence-producing calls;
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
