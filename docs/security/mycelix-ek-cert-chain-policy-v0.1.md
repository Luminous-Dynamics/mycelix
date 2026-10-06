# Mycelix EK Certificate Chain Policy v0.1

This theorem validates an exact EK certificate path against an explicitly authorized trust anchor at a deterministic verification time, applies the TCG EK leaf policy boundary, requires explicit revocation state, and composes the prior EK SPKI key-identity theorem.

The trust anchor in the deterministic corpus is synthetic. Its digest is a fixture authorization value, not a real manufacturer trust root. A physical deployment therefore requires an independently authorized real root and provenance.

The theorem does not prove manufacturer authenticity, measured boot, AK/EK lineage, certificate revocation freshness beyond the supplied CRL state, or real-world manufacturer identity.


Caller-supplied certificate criticality overrides are not accepted. Certificate extensions are authoritative only when parsed from the exact DER certificate bytes.

The trust-anchor child boundary is source- and output-bound: the chain verifier records the exact trust-anchor appraiser source SHA, exact output-file SHA, and verifier-computed semantic content SHA, and rejects substitutions of any of the three.

The trust-anchor child boundary is source- and output-bound: the chain verifier records the exact trust-anchor appraiser source SHA, exact output-file SHA, and verifier-computed semantic content SHA, and rejects substitutions of any of the three.

The EK chain no longer consumes SPKI PASS metadata as authority. It re-executes the SPKI verifier over the exact chain certificate/public-wire inputs and binds the verifier source, input/output file digests, and semantic output content digest. The contract adds four explicit SPKI composition regressions.


The certificate-path theorem is now an independent execution boundary. The chain verifier derives the exact DER/CRL/time input, re-executes `mycelix.tpm.ek-cert-path-validation.v0.1`, and binds verifier source, input/output file hashes, semantic output hash, and a canonical execution-policy binding. The reference claim ceiling remains `ReferenceModelOnly`; OpenSSL runtime provenance is evidence of the verifier execution, not manufacturer trust.
