# MOBILITY-COMMONS-020: Historical EvidenceState Projection Preservation

This layer tests whether a witness revision can silently rewrite the semantic state produced by a predecessor projection. It requires exact witness identity binding and compares complete EvidenceState snapshots.

The important invariant is that **history is a value, not merely a pointer**: once a predecessor projection has produced a state, a successor witness does not retroactively mutate that state.

Qualification remains semantic/provenance-only. It does not establish physical correctness, safety, certification, regulatory approval, physical equivalence, measurement truth, or authorship truth.

Holochain validation is deterministic and dependency-aware; dependencies must be explicitly addressable and unavailable dependencies are unresolved rather than silently interpreted. citeturn0search0turn0search4 NIST's digital-thread work similarly emphasizes persistent identifiers and lifecycle traceability across heterogeneous engineering, manufacturing, and quality data. citeturn0search2turn0search3turn0search5
