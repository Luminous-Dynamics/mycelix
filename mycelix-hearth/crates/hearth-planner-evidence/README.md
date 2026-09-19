# hearth-planner-evidence

Evidence-qualified history adapter for the deterministic Hearth household planner.

The v1 planner accepts `verified_recent_care_minutes` directly for compatibility. This crate defines the v2 boundary: callers provide current member facts **without a history field** plus a `CareDigestV3` evidence capsule. The adapter derives historical minutes only from Digest-v3-qualified actual completion evidence before invoking the existing deterministic planner.

Unbound legacy work, orphan/conflicted completion evidence, performer mismatches, and unknown actual duration remain visible in provenance but cannot be converted into historical workload minutes.

This crate is pure Rust/Serde and has no HDK/HDI dependency.
