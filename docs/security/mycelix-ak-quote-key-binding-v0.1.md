# Mycelix AK Quote Key Binding v0.1

The TPM Quote signature can be mathematically valid under a public key without proving that the public key is the same AK object whose Name and Qualified Name were qualified elsewhere. This theorem closes that object-identity seam.

The verifier accepts the exact captured AK public-key representation plus the exact TPMT_PUBLIC bytes. OpenSSL canonicalizes the captured RSA public key to DER SubjectPublicKeyInfo. Independently, the verifier parses the RSA modulus and exponent from TPMT_PUBLIC and reconstructs the same canonical SPKI encoding. The byte sequences must match.

The verifier then derives the TPM Name from the exact public-area bytes and requires the supplied AK Name and existing lineage binding to match the same public-area digest. A caller-supplied Quote-key digest is not trusted as a substitute for this reconstruction.

The theorem deliberately does not prove the Quote signature itself, TPM residency, AK/EK parentage, or freshness. Those remain separate propositions.

ReferenceModelOnly is the qualification ceiling. Offline/live origin is INDETERMINATE.
