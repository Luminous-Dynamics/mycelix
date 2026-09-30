# Integral interoperability: canonical commitment consumption boundary

Status: ReferenceModelOnly.

## Purpose

D6S-CANON-1 emits SHA-256 commitments as exactly 64 lowercase hexadecimal characters. The implementation now exposes that textual invariant separately from the older ReferenceModelOnly compatibility path.

There are intentionally two boundaries:

- D6X projection compatibility: QualifiedNodeV1::commitment_matches() and QualifiedEdgeV1::commitment_matches() still accept legacy opaque symbolic commitments so older ReferenceModelOnly fixtures can remain readable.
- D6W consumption: InputCommitmentV1::from_projection() requires selected node and edge commitments to both match their semantic bindings and be canonical 64-character lowercase SHA-256 strings.

This prevents a legacy symbolic commitment from crossing the stronger D6W input boundary while avoiding a broad compatibility-breaking rewrite of every historical reference fixture.

## Exact predicate

is_canonical_sha256_commitment() accepts only:

- exactly 64 bytes;
- ASCII 0-9;
- lowercase ASCII a-f.

Uppercase hexadecimal and symbolic identifiers are not canonical D6S commitment encodings.

The predicate is about representation, not about proving an external object exists. The source DKG snapshot commitment remains an opaque identity at this ReferenceModelOnly boundary.

## Security boundary

A projection can therefore be internally understandable to the legacy D6X reference model without being consumable by D6W.

The intended flow is:

1. D6X may read legacy ReferenceModelOnly fixtures for compatibility.
2. Selected node/edge bindings must match their carried semantic fields.
3. Before D6W creates C_input, selected graph commitments must additionally use the canonical commitment representation.
4. A symbolic or stale selected commitment fails closed.

This makes compatibility a localized migration mechanism rather than an implicit acceptance rule at every downstream boundary.

Cryptographic hashes are used here as integrity commitments; they do not confer truth, authority, causality, current-finality, or actuation semantics. NIST describes cryptographic hash functions as fixed-length message digests and discusses their use for integrity and digital signatures.

## Migration path

The remaining migration work is to convert all ReferenceModelOnly projection fixtures that still use symbolic selected commitments to canonical commitments, then narrow or remove legacy acceptance from the D6X compatibility layer itself.

The current tranche deliberately stops one boundary earlier: D6W is already fail-closed, while old D6X fixtures remain loadable.

## Non-goals

This change does not:

- define or verify the external DKG snapshot object;
- turn SHA-256 into a truth oracle;
- claim conformance to an external Integral wire schema;
- change D6S-CANON-1 numeric semantics;
- authorize downstream actuation.
