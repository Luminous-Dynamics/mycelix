# Mycelix AK/EK Lineage v0.1

This profile is the next trust-boundary theorem after physical event-log reconstruction.

## Claim

The theorem establishes parentage and TPM association for an attestation key (AK) under an observed endorsement key (EK), without confusing any individual artifact for the whole claim.

The Qualified Name check is:

QN_AK = 0x000b || SHA-256(EK_QN || AK_NAME)

The TPM specification computes a Qualified Name from the parent's Qualified Name and the object's Name, and current tpm2-tools documentation exposes the Qualified Name as evidence of object parentage.

## Three required gates

**1. Qualified Name parentage**

The verifier independently recomputes the AK Qualified Name from the observed EK Qualified Name and AK Name. A substituted parent, AK Name, or Qualified Name is therefore DENY.

**2. Non-migratable AK**

Both fixedTPM and fixedParent must be SET. The current TCG EK credential profile likewise requires non-duplicable EKs to use those attributes.

**3. Credential activation**

TPM2_MakeCredential binds a credential to EK public material and AK Name; TPM2_ActivateCredential then asks the TPM to recover the credential through the credentialed AK and credential key. tpm2-tools documents this association as an attestation mechanism for establishing that the AK belongs to the TPM with a qualified parent key.

## Deliberate boundary

The static verifier does not accept a bundle-stored activation receipt as self-authenticating live proof. Offline evidence remains INDETERMINATE; LiveVerifierSession is reserved for a future verifier that directly performs the TPM activation.

The AK public key and AK Name are required to come from the same TPM ReadPublic context observation. The lineage verifier now invokes the dedicated AK public-attributes verifier as a separate process, supplies its exact verifier input, and requires the generated public-area digest and Name to match the AK identity. This closes the previous false-green path where caller-supplied fixedTPM/fixedParent booleans could disagree with the actual TPMT_PUBLIC bytes.

The exact TPMT_PUBLIC body digest is kept distinct from the broader AK public-artifact digest. This avoids treating differently encoded representations as if their SHA-256 values had the same semantics.

The profile ceiling remains ReferenceModelOnly. No hardware qualification is introduced by this theorem.

## Result

The deterministic corpus contains one canonical PASS, twenty-six adversarial mutations, and a key-order permutation check. DENY outranks INDETERMINATE, which outranks PASS.

The public-name prerequisite is source/input/output bound: a serialized PASS, substituted verifier source, or alternate public-area input cannot satisfy lineage without re-execution of the exact public-name verifier.
