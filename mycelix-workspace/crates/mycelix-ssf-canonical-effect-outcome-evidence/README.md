# SSF Canonical Effect Outcome Evidence v0.1

Consumes one canonical post-operation result and produces a compact, copyable audit envelope.

The envelope binds the exact durable journal provenance, exact operation attempt, unmodified provider receipt, low-level interpretation, stricter canonical classification, pre/post trusted-time evidence, and any post-operation ambiguity that prevented terminal qualification.

The constructor checks that these views agree. A terminal canonical state is permitted only when the low-level terminal result, qualified receipt, post-operation evidence, and canonical classification are mutually consistent. A provider-reported terminal result may remain preserved as raw evidence while the canonical classification is `OutcomeUnknown`.

This crate adds no execution path or retry permission. It preserves provenance after the one-shot execution lineage has been consumed so later storage does not need to recreate a pre-operation typestate.
