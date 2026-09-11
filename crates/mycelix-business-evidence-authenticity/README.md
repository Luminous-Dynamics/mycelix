# Mycelix Business Evidence Authenticity

`mycelix-business-evidence-authenticity` is a reference-only contract for consuming evidence-authenticity verification performed by an external authority domain such as Xenia or Identity.

Business does **not** verify signatures, credentials, certificates, or revocation here.

## What the reference binds

A `VerifiedEvidenceAuthenticityRef` binds:

- exact evidence/subject digest;
- claimed issuer;
- verifier domain and verifier receipt digest;
- verification method and policy digest;
- verifier epoch and receipt sequence;
- credential identity and credential epoch;
- revocation frontier digest; and
- verification validity window.

The metadata is itself digest-bound so it cannot be relabeled without invalidating the local reference.

## Execution-time revalidation

`validate_at` fails closed when verifier epoch, credential epoch, or revocation frontier has changed, or when the receipt is not yet valid/has expired.

A changed frontier does not mean the credential was necessarily revoked; it means the old receipt is no longer sufficient and must be revalidated by the external verifier.

## Non-claims

This crate does not establish external physical truth, legal authority, institutional permission, or cryptographic validity by itself. It only preserves the output of an independently authoritative verification step so Business can compose it safely.
