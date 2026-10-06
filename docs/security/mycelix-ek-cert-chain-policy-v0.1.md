# Mycelix EK Certificate Chain Policy v0.1

This theorem validates an exact EK certificate path against an explicitly authorized trust anchor at a deterministic verification time, applies the TCG EK leaf policy boundary, requires explicit revocation state, and composes the prior EK SPKI key-identity theorem.

The trust anchor in the deterministic corpus is synthetic. Its digest is a fixture authorization value, not a real manufacturer trust root. A physical deployment therefore requires an independently authorized real root and provenance.

The theorem does not prove manufacturer authenticity, measured boot, AK/EK lineage, certificate revocation freshness beyond the supplied CRL state, or real-world manufacturer identity.


Caller-supplied certificate criticality overrides are not accepted. Certificate extensions are authoritative only when parsed from the exact DER certificate bytes.
