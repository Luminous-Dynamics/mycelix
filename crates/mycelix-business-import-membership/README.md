# mycelix-business-import-membership

Exact, read-only membership evidence for observations used by Mycelix Business shadow qualification.

This layer closes the `forecast-actual-campaign-membership-unverified` limitation without rewriting earlier evidence.

## Proof path

For each registered extraction campaign it:

1. requires the full exact source-file set;
2. reruns each file through the same delimited adapter with the original ingestion time;
3. requires the regenerated diagnostic manifest to equal the registered manifest;
4. mirrors duplicate-event rejection semantics from the diagnostic scanner;
5. requires every claimed forecast actual to match an exact deterministically normalized `MetricObservation` from those files;
6. rejects missing matches, duplicate matches, source-file substitutions, manifest drift, and observation-identity collisions.

The evidence stores only campaign/file/observation digests and opaque observation references. It does not copy raw source rows into qualification evidence.

## Immutable limitation discharge

The inner derived hospitality report remains unchanged and still records:

`limitation:forecast-actual-campaign-membership-unverified:v1`

A successful stronger verification emits a separate `LimitationDischarge` referencing the exact campaign and membership-evidence digest. This preserves evidence lineage instead of mutating an older report until it appears stronger than it originally was.

The unrelated `limitation:upstream-export-completeness-unverified:v1` remains unresolved: proving that an actual is in an exported file does not prove that the upstream POS exported every real transaction.

## Safety boundary

This crate is read-only. It carries no provider credentials, grants no authority, creates no business mutation, and cannot promote a model beyond the shadow/field qualification gates already defined by the Business Fabric.
