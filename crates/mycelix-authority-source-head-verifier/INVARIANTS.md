# Authority Source-Head Verifier v0.1 — Normative Invariants

Status: **pure qualification candidate; not yet review-ready**

This layer answers only:

> Did this exact source identity authenticate this exact challenged source-head attestation?

It does not decide that the source identity is institutionally trusted and does not prove global completeness/currentness.

## 1. Exact challenge binding

The qualifier MUST recompute the exact `CoverageChallenge.identity_digest()` from a `VerifiedCoverageChallenge` and require both:

- `AuthoritySourceHeadAttestation.challenge_digest`; and
- `VerifiedSourceHeadProof.challenge_digest`

to equal it.

The attestation subject must equal the challenge subject.

A source proof from another challenge or subject cannot be rebound.

## 2. Exact attestation authentication

`VerifiedSourceHeadProof` MUST bind:

- exact `AuthoritySourceHeadAttestation.identity_digest()`;
- registered `SOURCE_HEAD_IDENTITY_PROFILE`;
- exact source identity digest/profile;
- exact challenge digest; and
- exact source-proof reference.

The proof verifier's echoed proof ref must equal the attestation's `source_proof_ref`.

## 3. Authentication is not source-policy authority

The pure source verifier authenticates the identity carried by the attestation.

It MUST NOT decide that this identity is the institution-approved source.

Later #94/#96 MUST compare:

- `authoritative_source_ref`; and
- exact source identity digest/profile

against the current accepted `AuthorityCoveragePolicy`.

This permits cryptographic verification to stay separate from institutional trust selection.

## 4. Authentication is not completeness

A source can truthfully authenticate an old or selectively disclosed head.

Therefore a positive `QualifiedSourceHeadAuthentication` MUST NOT mean:

- no later transition exists;
- no revocation exists;
- local DHT state is complete; or
- the source head is current authority by itself.

#94/#96 coverage plus #91 endpoint equality are still mandatory. Witness quorum may additionally be required by policy.

Source refusal or partition fails closed; the verifier must not synthesize a head from local absence.

## 5. Causal time ordering is mandatory

Before this tranche is review-ready, implementation and tests MUST enforce:

1. challenge verification is no earlier than challenge issuance;
2. source response is no earlier than challenge issuance;
3. source response occurs before challenge expiry;
4. cryptographic source-proof verification is no earlier than source response;
5. all reusable proof/attestation/challenge windows remain live at qualification time; and
6. resulting validity is the conservative minimum of challenge, source-response, and proof-verifier horizons.

This is a pre-PR blocker, not an optional follow-up.

## 6. Stable semantic identity vs dynamic verifier time

The stable qualification digest commits:

- exact challenge identity;
- exact source-head attestation identity/profile; and
- exact source identity digest.

Verifier timestamps and lease metadata are not authority identity. Re-verifying the same exact attestation under the same exact challenge and source identity may refresh proof metadata without creating a different semantic authentication domain.

## 7. Non-deserializable positive result

`QualifiedSourceHeadAuthentication` is serializable but not deserializable as positive authority.

Runtime adapters may transport candidate attestation/proof receipts, but must call `qualify_source_head_authentication` locally before projecting to `VerifiedAuthoritySourceHead`.

## 8. No policy, DHT, or execution authority

This crate is pure Rust and contains no:

- HDK/Holochain calls;
- DHT lookup;
- policy discovery;
- trust-domain classification;
- transition projection;
- persistence;
- lifecycle mutation; or
- external effects.

Phi, reputation, model output, caller identity, and source self-assertion cannot make the source institutionally trusted.

## 9. Required tests

Before opening/merging the tranche, tests must cover at least:

- exact challenged head succeeds;
- wrong challenge denies;
- wrong subject denies;
- changed attestation bytes deny;
- wrong source identity denies;
- wrong source proof ref denies;
- source response before challenge denies;
- proof verification before source response denies;
- challenge verification before issuance denies;
- expired challenge/source/proof denies; and
- same semantic head with refreshed verifier time preserves qualification identity.

No completeness claim may appear in this layer.
