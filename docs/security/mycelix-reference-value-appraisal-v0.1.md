# Mycelix Reference Value Appraisal v0.1

RATS treats Reference Values as a separate input to the Verifier's appraisal process; the Relying Party ultimately consumes the resulting Attestation Result rather than trusting a bundle-local reference file by itself. citeturn521910search0turn521910search28

For `ReferenceModelOnly`, this implementation uses a reviewed static registry. The registry authorizes an exact `(version, sha256)` pair.

The appraisal result is:

    exact reviewed set → PASS
    approved version + different bytes → DENY
    unknown version → INDETERMINATE

This is deliberately weaker than a live Reference Value Provider trust chain. It proves only that the exact fixture is one of the reference sets approved by the reviewed registry.

A future provider-backed profile can replace the static registry with an authenticated provider statement, freshness, revocation, and policy binding.
