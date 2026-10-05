# Mycelix EK Certificate-Chain NV Capture v0.1

This is a provenance-only source boundary. It preserves the exact bytes found in the TCG-defined EK certificate-chain NV range without parsing, reordering, or assigning issuer trust.

TCG EK Credential Profile 2.7 defines the optional EK certificate-chain NV area at indices 0x01c00100 through 0x01c001ff. Certificates are stored as concatenated DER X.509 objects, beginning at 0x01c00100 and overflowing into the next numeric index. The root CA must not be included in that NV-resident chain.

The capture helper therefore inventories NV handles, reads only populated indices in that exact range with tpm2_nvread, records every per-index digest and command receipt, and preserves a byte-for-byte concatenated artifact. It deliberately does not parse the DER chain or infer certificate order beyond the NV index order.

Network retrieval is not part of this source class. Absence of populated chain indices is represented as INDETERMINATE. The later EK certificate-chain policy verifier is responsible for parsing the chain, validating signatures, applying trust-anchor policy, checking time and revocation, and composing the prior SPKI key-identity result.

Claim ceiling remains ReferenceModelOnly.
