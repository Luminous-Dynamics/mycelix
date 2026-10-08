# Authenticated inclusion receipts research v1

Status: research-only.

This layer adds the registration/inclusion proof boundary described by SCITT: a transparency service receipt states that a particular signed statement was recorded in a VDS, while the relying party verifies the receipt and the underlying VDS proof separately.

The research pipeline is:

    signed witness observations
        ->
    append-only VDS
        ->
    authenticated witness tree-head quorum
        ->
    transparency-service receipt
        ->
    Merkle inclusion proof
        ->
    qualified registration evidence

Each research receipt binds:

- transparency-service identity and key;
- receipt schema and domain;
- VDS identifier;
- statement identifier and leaf hash;
- manifest version;
- tree size and leaf index;
- VDS root;
- authenticated witness-head quorum digest.

The receipt is signed by a dedicated transparency-service key. That key is intentionally separate from the anchor witness keys.

## Proof boundaries

The verifier does not accept a receipt merely because its signature is valid.

It independently checks:

    receipt signature
        +
    exact statement binding
        +
    reconstructed VDS root
        +
    inclusion proof
        +
    authenticated witness-head quorum

This preserves several distinct failure classes:

    invalid receipt signature      -> signature-invalid
    wrong statement / leaf         -> statement-binding
    wrong tree root/head            -> receipt-head-binding
    malformed proof                -> inclusion-proof-invalid
    invalid witness head            -> head authentication failure
    divergent witness heads        -> head equivocation

A valid receipt over one statement cannot silently be repurposed for another statement because the statement hash, leaf index, tree size, root, and quorum digest are all inside the signed claims.

## Research corpus

The 16-case campaign covers:

- valid receipts for multiple leaves;
- wrong statement hash;
- out-of-range leaf index;
- wrong tree size;
- wrong root;
- mutated, truncated, extended, and empty inclusion proofs;
- VDS identifier substitution;
- transparency-service registry substitution;
- transparency-service identity substitution;
- witness-head quorum digest substitution;
- unknown receipt key;
- receipt domain substitution.

Python and Node independently perform the same verification and emit byte-identical evidence reports.

## Relationship to SCITT

SCITT defines Receipts as signed proofs of VDS properties and requires Receipt Profiles to support inclusion proofs; consistency proofs are an additional proof type. RFC 9942 defines the COSE receipt framework and ties receipt verification to registered VDS and proof profiles.

This research implementation deliberately stops short of claiming COSE/CBOR wire interoperability. The receipt is a canonical-JSON research profile that exercises the semantic security boundary first.

## Claim ceiling

This demonstrates cryptographically authenticated inclusion evidence anchored to an already authenticated witness-head quorum.

It does not yet prove production Transparency Service operation, COSE receipt interoperability, network delivery guarantees, replay/freshness policy, inclusion-proof service availability, or independent organizational custody of the transparency-service signing key.

The next boundary is to make the receipt format wire-compatible with COSE Receipts and then test multiple independent observers exchanging signed heads, receipts, inclusion proofs, and consistency proofs across delayed or adversarial views.
