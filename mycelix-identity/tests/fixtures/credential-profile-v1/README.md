# Mycelix credential hash profile V1 interoperability fixture

This deterministic fixture is independently signed using OpenSSL-compatible
Ed25519 (the corresponding private seed is not included). It locks the exact
canonical UTF-8 JSON payload, the V1 domain-separated BLAKE2b-256 digest, a
public verification key, and the signature.

The coordinator unit tests:
- reconstruct the sample typed credential;
- remove only `proofValue`;
- recursively sort JSON object keys and compare the exact serialized payload
  bytes with `fixture.json`;
- compare the BLAKE2b-256 digest against the fixed vector;
- verify the externally generated Ed25519 signature with RustCrypto
  `ed25519-dalek`;
- apply 20 one-field mutations and require the independent signature check to
  fail for every mutation.

A second verification path uses the OpenSSL command-line implementation:

```sh
sh verify-openssl.sh
```

Expected output: `Signature Verified Successfully`.

The vector demonstrates only the Mycelix-internal
`mycelix-credential-hash-v1` profile. It is **not** a W3C Data Integrity
`eddsa-rdfc-2022` vector and does not demonstrate RDF Dataset Canonicalization
or external VC conformance. It also does not resolve the broader handling of
unmodeled JSON-LD extension properties; V1 coverage is the typed model plus
flattened credential-subject claims.
