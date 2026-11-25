# TUF/Rekor Comparator Slice — LISE Assurance Invalidation

**Corpus:** `LISE-METADATA-COMP-18-v1`  **Owner:** [#6219](https://github.com/Luminous-Dynamics/mycelix/issues/6219)
**Status:** public synthetic development/calibration material; not official TUF/Rekor conformance.

## Files
- `scenarios.v1.json`: 18 scenarios in nine A/B minimal pairs.
- `oracle.v1.json`: separate 18-row owner-scoped oracle.
- `scenario.schema.json` and `oracle.schema.json`: strict Draft 2020-12 schemas.

## TUF cases
1. Timestamp metadata version advance vs rollback below the trusted version.
2. Unexpired timestamp/snapshot metadata vs correctly signed but expired metadata.
3. Snapshot/targets digest coherence vs mix-and-match metadata.
4. Active delegated targets role vs delegation removed by current parent metadata.
5. Sequential root rotation satisfying old/new thresholds vs invalid version/threshold transition.

## Rekor cases
1. Valid inclusion proof vs proof/root mismatch.
2. Consistency proof extending a previously persisted checkpoint vs missing proof.
3. Independent witnesses agree on checkpoint vs same-size different-root split view.
4. Verified inclusion of one expected event vs a missing search result without event-submission completeness proof.

The oracle preserves historical artifact/signature validity separately from current metadata/log qualification. It does not infer event absence from a search miss and does not treat a signed checkpoint as proof of consistency with a previously trusted checkpoint.

## Comparator claim boundaries
- TUF is a strong reference for metadata version, expiry, delegation and snapshot coherence semantics; it does not supply domain-specific semantic applicability for every institutional claim.
- Rekor supplies transparency-log inclusion and consistency mechanisms; an inclusion proof establishes inclusion of a specific entry at a checkpoint, not the completeness of all external events that should have been submitted.
- Neither component by itself establishes Mycelix authority legitimacy, exact external-effect completion or remedies.

Sources:
- TUF specification v1.0.37: https://theupdateframework.github.io/specification/latest/
- TUF security analysis: https://theupdateframework.io/docs/security/
- Rekor overview and monitoring: https://docs.sigstore.dev/logging/overview/
- Rekor specification: https://github.com/sigstore/architecture-docs/blob/main/rekor-spec.md

All scenario and oracle files are public calibration material, not a blind holdout or official protocol conformance suite.
