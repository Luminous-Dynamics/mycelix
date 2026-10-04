# Mycelix RATS Attestation/Appraisal Contract v0.1

**Status:** executable reference contract  
**Claim ceiling:** `ReferenceModelOnly`

This artifact turns the RATS separation in the enclave profile into an executable semantic qualification boundary.

## Security model

The qualified flow is:

```
Attester
  -> Evidence
  -> Verifier
  -> Attestation Result
  -> Relying Party
  -> local authorization
  -> Policy Enforcement Point
```

The important invariant is:

```
raw Evidence != Attestation Result != Authorization
```

RFC 9334 defines the Verifier as the party that appraises Evidence under an appraisal policy and produces Attestation Results. The Relying Party then applies its own appraisal policy to those results for application-specific decisions, including authorization.

## What this artifact actually verifies

The contract fixes a synthetic E1 verifier profile with:

- one security domain (`E1`);
- one verifier profile identifier;
- one trust-anchor identifier;
- one measurement-profile identifier;
- explicit challenge nonce and audience binding;
- bounded evidence and result freshness;
- replay detection at the semantic boundary;
- exact subject, device, and workload binding;
- exact policy-version binding;
- explicit local authorization checks;
- explicit deny on cross-domain substitution;
- explicit deny when required bindings are absent.

The booleans representing signature validity are **upstream cryptographic verification oracles**. This harness does not parse TPM quotes, implement a TPM, or claim cryptographic validation. It verifies the security semantics that must hold after those lower-layer checks are available.

## Stronger result binding

A positive Attestation Result is not treated as a portable bearer authorization.

The relying-party test requires the result to match the **current access request** for:

- audience;
- nonce;
- subject;
- device;
- workload;
- security domain;
- policy version.

It then evaluates the current local resource/purpose/releasability/export-control/delegation authorization state.

This deliberately prevents the common semantic collapse:

```
valid attestation
   -> permanent permission
```

Instead:

```
attestation result
AND current request
AND current local policy
-> authorization decision
```

## Determinism

The fixture fixes a test clock at `2026-10-04T12:00:00Z`. No network, randomness, external service, or third-party Python dependency is required.

Run:

```bash
python3 scripts/security/verify_mycelix_rats_v0_1.py
```

The command exits non-zero if any vector fails.

## Qualification corpus

There are **28 executable vectors**:

- 13 evidence appraisal vectors;
- 15 relying-party/local-authorization vectors.

The corpus includes signature, trust-anchor, nonce, audience, freshness, future-time, measurement-profile, domain, required-claim, verifier-profile, replay, result-substitution, policy-downgrade, resource authorization, purpose authorization, export-control authorization, delegation, and key-order invariants.

The key-order permutation vector is intentionally positive: JSON member ordering must not alter semantic appraisal.

## Standards relationship

The contract uses RFC 9334 terminology rather than defining a parallel attestation model. NIST SP 800-207 is used as an architectural reference for separating policy decision from policy enforcement. The hardware layer is intentionally outside the semantic verifier so a future selected TPM 2.0 profile can bind real Evidence to this contract without making the Mycelix application layer responsible for TPM correctness.

## Qualification ceiling

A green run establishes only that the **synthetic reference-model semantics** implemented by this harness satisfy the stated vectors.

It does not establish:

- real TPM quote verification;
- measured-boot correctness;
- firmware or kernel integrity;
- hardware security;
- cryptographic-module validation;
- personnel/facility authorization;
- CMMC status;
- classified authorization;
- CDS approval;
- export-control legal compliance;
- superiority over SIPRNet/NIPRNet.

The next implementation step is therefore to replace the synthetic Evidence fixture with a selected real hardware/attestation profile while preserving the same Verifier -> Attestation Result -> Relying Party boundary.
