# mycelix-hardware-kicad-gateway

Server-neutral, read-only projection of verified Mycelix hardware component records into KiCad's HTTP Library response contract.

This crate deliberately does **not** run an HTTP server, access Holochain, hold access tokens, or become a second component database. A transport adapter can wrap the catalog with Axum (or another HTTP stack) after authenticating KiCad's documented `Authorization: Token ...` header.

The protocol core exposes only the information KiCad's current read-only HTTP Library needs:

- endpoint discovery (`categories`, `parts`);
- categories;
- parts-by-category summaries;
- detailed part records.

All projected scalar values are strings, matching KiCad's protocol. Symbols and footprints are references to ordinary KiCad libraries; this gateway does not transport or invent symbol/footprint definitions.

## Identity and authority

KiCad part identity is the canonical Mycelix `SemanticId`, not a manufacturer part number. The catalog accepts an entry only after a caller-provided `ComponentRecordVerifier` confirms that the supplied source-record digest belongs to exactly that `ComponentIdentity`.

```text
component visible in KiCad
!= approved for this design

symbol/footprint reference exists
!= symbol/footprint verified

component discovered
!= substitute qualified

lifecycle metadata displayed
!= availability guaranteed

HTTP response
!= canonical Mycelix record
```

Manufacturer and MPN are descriptive fields, so identical MPN strings from different manufacturers do not collide. Reserved KiCad/Mycelix fields cannot be overwritten through custom fields.

## Deliberate omissions

The first slice does not flatten evidence confidence, substitution qualification, supplier stock, pricing, or private organization metadata into ordinary KiCad fields. Those values are time-, profile-, or authority-sensitive and should remain available through richer Mycelix surfaces rather than masquerading as static part facts.

The network adapter remains responsible for token authentication, request-size/rate limits, TLS/deployment policy, and mapping catalog errors to HTTP status codes.
