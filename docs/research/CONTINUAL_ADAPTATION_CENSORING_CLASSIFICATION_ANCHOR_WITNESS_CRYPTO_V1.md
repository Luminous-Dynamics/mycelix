# Anchor witness cryptographic authentication research v1

Status: research-only.

This layer closes the authentication boundary above the semantic witness non-equivocation research in #4848. The important semantic change is that each witness signs its own complete observation; quorum is evaluated only after each observation has been independently authenticated.

```text
witness-specific observation
        -> canonical signing input
        -> domain-separated Ed25519 signature
        -> key / version validity
        -> quorum
        -> non-equivocation
        -> qualification decision
```

A cryptographically valid contradictory witness observation is therefore classified as `equivocation`. It is not mislabeled as a signature failure. Conversely, altering a signed claim, algorithm, domain, or serialization produces an authentication failure before quorum semantics are considered.

## Signed observation boundary

Each witness attestation binds:

- witness identity and registered identity commitment;
- registry identifier and version;
- pinned trust-root reference;
- authority identifier;
- manifest version;
- manifest commitment;
- predecessor commitment;
- key identifier, signature algorithm, and signing domain.

The signed payload is canonical JSON with recursively sorted object keys and compact separators. The signature profile is Ed25519 with base64url encoding without padding and a fixed domain string. The repository contains only public witness keys and public signatures; private witness keys are not fixtures and are not committed.

## Key lifecycle

Keys are valid for explicit manifest-version intervals. Rotation is strict: a successor begins after its predecessor's validity interval, and the successor/predecessor relationship must be symmetric. Active keys cannot carry historical end or revocation bounds. Retired and revoked keys are bounded historical keys. A mathematically valid historical signature can therefore remain cryptographically verifiable while being rejected as inadmissible for a newer manifest version.

The registry also rejects public-key reuse across witness identities because the cryptographic layer cannot establish witness independence when multiple registered witnesses share key material.

## Adversarial campaign

The 22-case campaign covers:

- baseline quorum, single-witness loss, and one-step forward append;
- same-version and predecessor forks using independently valid signatures;
- wrong-key, wrong-manifest, and wrong-predecessor signatures;
- cross-domain reuse and canonicalization mismatch;
- algorithm substitution;
- revoked / expired historical-key replay and rotation rollback;
- duplicate signatures and signature wrapping;
- trust-root substitution and threshold weakening;
- public-key reuse, overlapping rotation, broken successor lineage, and active-key bounds.

The expected distinction is:

```text
valid signatures + conflicting claims  -> equivocation
invalid signature material             -> signature-invalid
inadmissible key/version               -> key lifecycle failure
invalid registry                       -> registry failure
invalid trust-root pin                 -> root failure
```

## Independent implementation check

Python and Node.js implement the same verifier semantics independently and their generated evidence reports are required to be byte-identical. Both implementations also verify the RFC 8032 Ed25519 known-answer test vector before evaluating the campaign.

## Claim ceiling

This demonstrates cryptographic witness authentication relative to a repository-controlled, pinned research registry. It does not establish secure private-key custody, HSM or secure-element protection, independent organizational governance of witnesses, compromise resistance, network-level availability, an append-only witness history, cross-observer consistency proofs, or SCITT protocol interoperability.

The signature envelope is intentionally a research profile rather than a claim of COSE/SCITT wire compatibility. SCITT requires a cryptographically verifiable append-only, non-equivocating and replayable VDS; the next layer is therefore an append-only witness history with explicit cross-observer consistency proofs.

No hosted PASS is claimed until the exact-head workflow completes successfully.
