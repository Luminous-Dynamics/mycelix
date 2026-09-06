# Authority Source-Head Runtime v0.1 — Normative Invariants

Status: **implemented adapter candidate; deliberately unprovisioned in the binding governance DNA**

This zome composes two independent runtime roles around the pure #130 kernel:

1. source response acquisition; and
2. cryptographic source-proof verification.

It does not decide institutional source trust or source-head completeness.

## 1. Responder output is candidate data

`authority_state_source_responder::resolve_source_head_attestation` may return one `AuthoritySourceHeadAttestation` candidate only.

The responder MUST NOT return `VerifiedAuthoritySourceHead` or `VerifiedSourceHeadProof` as authoritative output.

A source is not allowed to certify its own authentication merely because it authored the response.

## 2. Cryptographic proof verification is separate

The exact challenge digest, exact attestation digest/profile, and exact full attestation are sent to:

`authority_source_proof_verifier::verify_source_head_proof`.

That verifier returns `VerifiedSourceHeadProof` only.

Its output still does not decide institutional source trust or completeness.

## 3. Pure qualification is mandatory

The only positive path is:

`VerifiedCoverageChallenge`
→ candidate `AuthoritySourceHeadAttestation`
→ independent `VerifiedSourceHeadProof`
→ `qualify_source_head_authentication`
→ `VerifiedAuthoritySourceHead`.

The coordinator MUST NOT assemble `VerifiedAuthoritySourceHead` directly from provider fields.

## 4. Authentication is not institutional trust

This runtime deliberately receives no `AuthorityCoveragePolicy`.

The authenticated source identity and logical source reference are compared against current institutional policy only later in #94/#96.

A cryptographically valid response from an untrusted source must still be rejected later.

## 5. Authentication is not completeness

This runtime does not assert that the authenticated head is the latest or complete authority state.

It contains no DHT lookup, no latest-record selector, no transition projection and no witness quorum logic.

#94/#96/#91 remain responsible for coverage and exact endpoint equality.

Source refusal, network failure, responder outage, proof-verifier outage, decode failure, or failed pure qualification MUST deny.

## 6. Exact challenge passes through unchanged

The caller supplies an already verified `VerifiedCoverageChallenge`.

The coordinator recomputes its exact identity and uses that digest in the proof-verifier request. #130 then rechecks the challenge receipt, attestation challenge binding, subject, source identity, proof ref and causal time ordering.

## 7. No provider self-declaration of currentness

Neither the responder nor proof verifier can claim:

- current policy authority;
- source completeness;
- no later revocation;
- witness independence;
- lifecycle authority; or
- external-effect permission.

## 8. Status is declarative only

`source_head_runtime_status` performs no synthetic provider call and reports:

- source response candidate-only = true;
- proof verifier separate = true;
- institutional source trust decided here = false;
- completeness decided here = false;
- external effects enabled = false; and
- operational = false.

## 9. Deliberately unprovisioned

The zome is included in the Rust workspace only for native/Clippy/WASM qualification.

`mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_state_source_head_verifier` in this tranche.

The source responder and source-proof verifier are not production-qualified here.

## 10. Required qualification

Before provisioning:

- rustfmt;
- native check and warnings-denied Clippy;
- WASM check;
- responder outage denial;
- proof-verifier outage denial;
- wrong challenge/attestation/source identity/proof denial;
- causal-order tests from #130;
- proof that source policy is not decided here;
- proof that DHT absence/completeness is not decided here; and
- end-to-end integration with #129 under adversarial source/witness conditions.

No external effects are enabled.
