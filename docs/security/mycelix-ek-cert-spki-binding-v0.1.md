# Mycelix EK Certificate SPKI Binding v0.1

This theorem proves only that the leaf EK certificate contains the same public key as the exact TPM EK public area already captured elsewhere in the evidence chain.

The current TCG EK Credential Profile Version 2.7 defines an X.509 EK credential and requires SubjectPublicKeyInfo for the EK. For RSA, the profile specifies rsaEncryption with a NULL AlgorithmIdentifier parameter and RSAPublicKey containing modulus and publicExponent. citeturn888214search4turn888214search24

The verifier uses OpenSSL to parse the certificate and extract the leaf public key, then canonicalizes that public key to DER SubjectPublicKeyInfo. Independently, it parses the RSA modulus and exponent from TPMT_PUBLIC and constructs the canonical RSA SubjectPublicKeyInfo DER. The SPKI byte sequences must match.

An SPKI match does not establish that the certificate was issued by a trusted EK CA, is currently valid, is not revoked, or vouches for a genuine manufacturer-compliant TPM. Those propositions remain separate.

Offline and live execution modes remain INDETERMINATE. ReferenceModelOnly is only a deterministic key-identity model.

OpenSSL documents the x509 public-key extraction path and DER output controls used by the verifier. citeturn888214search0