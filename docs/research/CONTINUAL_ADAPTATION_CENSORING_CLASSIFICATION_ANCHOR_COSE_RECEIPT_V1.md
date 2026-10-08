# RFC 9942 COSE receipt profile research v1

Status: research-only.

This layer replaces the earlier canonical-JSON research receipt with the COSE receipt envelope and VDS proof representation defined by RFC 9942, using the RFC 9162 SHA-256 Merkle Tree profile.

## Wire representation

The fixture contains tagged `COSE_Sign1` objects (CBOR tag 18).

Protected headers:

- `alg` (label 1): EdDSA (`-8`);
- `kid` (label 4): the transparency-service signing-key identifier;
- `vds` (label 395): `1`, the RFC 9162 SHA-256 VDS algorithm.

The unprotected VDP header (label 396) contains one proof:
- inclusion proof label `-1`, encoded as a CBOR byte string containing `[tree-size, leaf-index, inclusion-path]`;
- consistency proof label `-2`, encoded as a CBOR byte string containing `[old-tree-size, new-tree-size, consistency-path]`.

The COSE payload is detached (`null` in the envelope). The associated payload is the binary Merkle root. The verifier requires RFC 8949 core deterministic CBOR encoding for the envelope, protected header, and embedded proof content; overlong lengths, duplicate map keys, indefinite-length items, and non-deterministic protected headers fail closed.

## Verification order

RFC 9942 Section 5.2.1 requires an inclusion verifier to apply the proof to the candidate entry first; the resulting Merkle root is then the COSE_Sign1 payload whose signature is verified.

RFC 9942 Section 5.3.1 specifies the inverse order for a consistency receipt: verify the signature on the newer tree root first, then verify the consistency proof against the previous authenticated root.

The implementation follows those different orderings explicitly. It also authenticates the prior witness tree-head quorums and checks that the COSE receipt's root and tree size correspond to those heads.

The candidate VDS entry is encoded as deterministic JSON bytes in this research application profile. RFC 9162 hashes the exact entry bytes; a different application using a different entry serialization must not reuse this profile without changing the content-binding definition.

## Fixture and corpus

The fixture has one inclusion receipt for entry index 2 in the size-7 tree and one consistency receipt for size 4 to size 7. Both signatures verify under a dedicated public-only fixture registry. The one-time fixture signing key was created ephemerally and was not persisted; no private key appears in repository fixtures.

The 22-case corpus covers:
- successful inclusion and consistency receipts;
- signature bit flips;
- altered candidate entry;
- mutated inclusion/consistency paths;
- invalid leaf index and tree sizes;
- algorithm, VDS, and key-ID substitution;
- attached payload where detached is required;
- missing COSE tag;
- extra unprotected labels;
- empty proof path;
- non-deterministic protected-header encoding;
- wrong proof-type label;
- TS registry-key substitution;
- receipt-root substitution.

Python and Node.js independently decode the CBOR, check deterministic encodings, verify COSE_Sign1 EdDSA signatures, reconstruct roots, and emit byte-identical evidence reports.

## Standards boundary

This adopts the RFC 9942 COSE receipt representation and its RFC 9162 SHA-256 proof labels. It does not yet claim complete SCITT deployment interoperability: the witness registry, tree-head quorum policy, key-governance ceremonies, and the application's canonical-JSON entry encoding remain Mycelix research-profile choices. End-to-end interoperability still needs to be exercised against an independent standards implementation.

## Claim ceiling

Research-only. This demonstrates the COSE receipt wire format and the inclusion/consistency verification boundary over deterministic fixtures. It does not demonstrate production transparency-service operations, private-key custody, external interoperability certification, gossip completeness, or hosted PASS.

References:
- RFC 9942, Sections 4.4, 5.2.1, and 5.3.1: https://www.rfc-editor.org/rfc/rfc9942.html
- RFC 9052, Sections 4.2–4.3: https://www.rfc-editor.org/rfc/rfc9052.html
- RFC 8949, Section 4.2: https://www.rfc-editor.org/rfc/rfc8949.html
- RFC 9162, Sections 2.1.3–2.1.4: https://www.rfc-editor.org/rfc/rfc9162.html
