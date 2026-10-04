# SYM-CIVIC-018 — sealing-key discovery trust-chain preflight v1

Status: design/research preflight only; v2 repair candidate — not qualified.

Parent subject: `fe82a004c30165affa77b9172d6b44fcd80a41f3`

## Boundary

018 advances one boundary above 017:

`register origins + Agreement-declared Authority Origin -> signed Authorised-Origin Document -> register signing key -> authorised server origin + sealing-key-set signer -> signed sealing COSE Key Set -> JWK-thumbprint addressed sealing key`

This is a synthetic research preflight. It does not implement production COSE, CBOR, HTTP retrieval, Web PKI, or cryptographic signature verification.

ARP-04 §7.5.3 defines a signed key set as a COSE_Sign1 carrying a deterministically encoded COSE_KeySet, with each key carrying `arp-key-status` and `arp-key-validity`. Its Authorised-Origin Document is a COSE_Sign1 whose payload is a bytewise-origin-sorted array of three-element arrays containing the server authority origin, the sealing-key-set signing key identifier, and the operator-key-set signing key identifier. Every addressed register must publish and sign that authorization document from its register key set. citeturn638291view0turn638291view1

The relying-party chain is reconstructed from the Output rather than from possession of Bilateral Register Agreements: register origin -> register key -> Authorised-Origin Document -> authorised authority origin and sealing-key-set signer -> signed sealing key set -> sealing key. ARP-04 states that resolving a key is insufficient because Web PKI proves control of an origin, not entitlement to seal outputs for the addressed register set. citeturn638291view0

## Representation boundary

018 pins one `kid` representation across the synthetic model:

- payload `kid`: base64url text;
- COSE `kid`: UTF-8 bytes of exactly that text;
- single-key URL path: exactly that text;
- Sealing-Key Identifier: `[Authority Origin, kid]`.

ARP-04 makes this equality rule explicit. citeturn638291view0

The synthetic JWK uses the RFC 7638 EC required members `crv`, `kty`, `x`, and `y`. Its thumbprint is computed from those members in canonical lexicographic JSON order, hashed with SHA-256, then base64url-encoded without padding. RFC 7638 explicitly excludes optional JWK members from the thumbprint. citeturn104151search0turn104151search4

The JWK values are deterministic synthetic public material. Passing this part establishes only thumbprint representation mechanics, not possession or cryptographic validity of a private key.

## Temporal key-status boundary

Each synthetic published key carries status and validity metadata. The model covers `active`, `retired`, and `revoked`, including a required revocation time on revoked entries and historical evaluation of the sealing signature against the reconciliation timestamp. Keys outside their validity interval are rejected, and historical keys are not substituted away through deletion.

ARP-04 explicitly requires revoked keys to carry a revocation time, limits retired-key use to the validity interval, rejects sealing after revocation, and prohibits removing a key while outputs it sealed may still be relied upon. citeturn638291view1turn638291view2

ARP-04 also requires the sealing key set to be published and notarised with a Publication Timestamp and republished at the ledger-head notarisation interval. 018 does not claim that mechanism; it uses the key-set publication timestamp only as a clearly labelled synthetic temporal anchor for its key-set signer check. citeturn638291view1

## Corpus

Exactly 34 adversarial cases; no expected-verdict fields.

The v2 repair adds fail-closed malformed-origin cases at nested authorization, key-set, and register-origin boundaries, plus an explicit byte-order case for the Output's Addressed-Registers Identifier Set.

Coverage includes authorization mismatch and omission, signed-document binding drift, bytewise ordering, signer resolution, key-set signer binding, sealing-key deletion, JWK-thumbprint drift, payload/COSE/path `kid` drift, Sealing-Key Identifier origin drift, not-yet-valid and expired keys, retired-key acceptance and boundary rejection, revoked-key historical acceptance and boundary rejection, missing revocation time, key-set signer validity failure, duplicate addressed registers, and RFC 3986-equivalent origin spellings.

Two logically separate validation paths are compared. Any disagreement becomes `SEALING_KEY_CHAIN_UNRESOLVED`; the reference path does not consume the primary verdict.

## Ceiling

GREEN establishes only a synthetic, self-consistent trust-chain relation over a closed fixture.

It does not establish actual COSE_Sign1 verification, deterministic CBOR interoperability, HTTP or Web PKI behavior, live retrieval behavior, real-world authority, private-key possession, Transparency Service notarisation, deployment completeness, operator independence, policy correctness, or operational safety.

RFC 3986 treats scheme and host as case-insensitive normalization targets but still defines a syntactic port component; malformed authority data is therefore invalid input and must become rejection rather than an uncaught verifier exception. citeturn759800search0

RFC 9052 independently requires unique COSE header-map labels and rejection of duplicate labels. 018 leaves duplicate-CBOR-map parsing to a future encoding-focused boundary instead of pretending the JSON fixture already implements it. citeturn104151search1

## v2 repair rule

`Malformed origin data is untrusted input: parser failure becomes REJECT, never an uncaught verifier exception; canonical output ordering is checked as part of the representation boundary.`

## Design rule

`Never accept a sealing key merely because a host serves it: derive the authority chain from the Output, verify every signed binding, pin one kid representation, and evaluate key status at the relevant historical timestamp.`

References:
- draft-hillier-scitt-arp-04 §§7.5.3–7.5.5
- RFC 7638
- RFC 9052
- RFC 8949
- RFC 3986
