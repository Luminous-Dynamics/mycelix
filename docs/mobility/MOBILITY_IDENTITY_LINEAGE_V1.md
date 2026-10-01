# Mobility identity and lineage

## MOBILITY-COMMONS-024: configuration applicability transitions

Configuration supersession and physical applicability are independent lineage facts.

If configuration revision `config-r2` supersedes `config-r1`, that fact does **not** implicitly transfer `config-r1 AppliesTo artifact-a` to `config-r2`.

A transition may contain an explicit successor-to-predecessor `Supersedes` edge, a predecessor `AppliesTo` edge, and an optional independently validated successor `AppliesTo` edge.

Three states remain distinct:

1. predecessor applies to an artifact; successor has no applicability assertion;
2. predecessor and successor explicitly apply to the same artifact;
3. predecessor and successor explicitly apply to different artifacts.

The predecessor applicability remains historical in all three cases. A successor never rewrites the predecessor relation.

This is a provenance/lifecycle rule only. It does not establish physical conformance, equivalence, safety, certification, regulatory validity, or measurement truth.

The Rust `ApplicabilityTransition` tests reject implicit inheritance and silently retargeted predecessor applicability while accepting explicit same-artifact and explicit artifact-transition cases.
