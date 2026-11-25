# Mycelix Economic OCEL projection v0.1

A dependency-free Rust reference exporter for a deliberately narrow OCEL 2.0 JSON projection profile.

## Contract

- FIN-EFFECT operation/effect identities and domain-owned records remain canonical.
- OCEL is a derived analytical representation, never an authority or settlement system.
- Every event requires a valid RFC 3339 source event time and a nonempty time-evidence reference.
- Every object-attribute update requires a separate source attribute-time assertion and provenance reference.
- Record time, ingestion time, owner sequence, event arrival and exporter wall clock are rejected as substitutes.
- OCEL root output contains only the four standard collections: eventTypes, events, objectTypes and objects.
- Time evidence is exported separately to MYCELIX-OCEL-TIME-PROVENANCE-V1; the canonical owner-scoped causal graph remains a different sidecar.
- Monetary/quantity values use integer attributes only. Values outside the interoperable IEEE-754 exact integer range are rejected rather than silently rounded by common JSON consumers.
- Types, IDs, declarations, attributes and E2O/O2O references are checked before serialization.
- Collections and relationships are deterministically sorted before encoding. Input permutation must not change output bytes.

## Run

    cargo test --manifest-path tools/economic/ocel-projection/Cargo.toml
    cargo run --manifest-path tools/economic/ocel-projection/Cargo.toml --example minimal

## Limits

This is a pure reference serializer, not a source-system adapter or complete OCEL validator. It does not authenticate source event times, establish source completeness, validate legal authority, prove physical delivery, settlement, title or finality, or encode event-event causal order in OCEL. The emitted time-provenance sidecar records caller-supplied evidence references but does not authenticate them.
