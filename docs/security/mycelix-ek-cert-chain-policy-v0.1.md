# Mycelix EK Certificate Chain Policy v0.1

This theorem validates an exact EK certificate path against an explicitly authorized trust anchor at a deterministic verification time, applies the TCG EK leaf policy boundary, requires explicit revocation state, and composes the prior EK SPKI key-identity theorem.

## Reference corpus provenance

The six binary fixtures previously declared as “frozen” are not present in the current Git tree and have no retrievable path history. They are therefore not represented as restored artifacts. The reference corpus is now deterministic-generated-synthetic instead: a committed fixture recipe plus a committed standard-library-only generator produce the exact DER/CRL outputs into a temporary CI directory, and the generator checks every resulting SHA-256 against the committed recipe.

That makes the bytes reproducible without treating an unverifiable historical fixture as authoritative. The synthetic RSA material and synthetic root are reference-only test material; they do not represent a manufacturer key or trust root.

The reference corpus is still ReferenceModelOnly. Reproducibility proves that the reference model is repeatable; it does not establish manufacturer authenticity, hardware provenance, firmware state, or a real-world CA authorization.

## Exact-byte boundary

Caller-supplied certificate criticality overrides are not accepted. Certificate extensions are authoritative only when parsed from the exact DER certificate bytes.

The parser now rejects duplicate Extensions wrappers, duplicate extension OIDs, non-canonical INTEGER/OID encodings, invalid BIT STRING padding, explicit FALSE extension-critical BOOLEAN encodings, malformed Authority Key Identifier structures, structurally incomplete TCG SubjectAltName data, empty Extensions, malformed recognized AIA/CRL Distribution/Subject Directory Attributes values, and invalid CertificatePolicies criticality/structure. SAN criticality is checked against the subject-empty condition.

For the leaf EK, the local 2.7 boundary includes critical BasicConstraints with CA=FALSE, critical Key Usage with RSA keyEncipherment, mandatory non-critical AKI with keyIdentifier, TCG SAN structure, non-critical SKI when present, and non-critical AIA/CRL Distribution/Subject Directory Attributes when present.

## Composed execution

The EK chain does not consume child PASS metadata as authority. It re-executes the trust-anchor appraiser, SPKI verifier, path verifier, and an independent certificate/CRL cryptographic witness against exact reconstructed inputs, binding verifier source, input/output file digests, semantic output content digests, and execution-policy state.

The independent cryptographic witness verifies the exact DER-encoded TBS certificate/CRL bytes and signature bytes with the exact issuer public keys. The composed theorem then requires its object, TBS, signature, and issuer hashes to agree with the in-process structural witness and with the OpenSSL path verifier's exact input-object hashes. A mismatch is fail-closed.

The path theorem executes an explicit OpenSSL full-chain CRL policy at the deterministic verification time, including strict RFC 5280 checks and explicit trust-anchor self-signature checking. Runtime OpenSSL evidence remains evidence about verifier execution, not manufacturer trust.

## Deliberate scope boundary

Conditional EKCredentialAlgorithmList / TPMPQCVersion requirements remain outside this v0.1 theorem because they depend on issuer-issued PQC credentials and firmware-at-manufacturing/current-firmware evidence that the synthetic reference corpus does not contain.

The theorem also does not prove manufacturer authenticity, measured boot, AK/EK lineage, certificate revocation freshness beyond the supplied CRL state, or real-world manufacturer identity.
