# LISE OPA decision-receipt and effect-binding reference

**Status:** synthetic interoperability reference. This is not an OPA plugin, PEP, collector implementation, provider adapter, or production trust verifier.

OPA decision logs describe policy queries. The official log fields include decision ID, trace/span context, bundle revision, policy path, query, input, result and timestamp. That is strong evidence about a policy decision, but it is not proof that a protected external resource changed.

This reference composes three separately qualified objects:

1. accepted intent (principal/action/purpose/resource/exact operation/policy);
2. collected OPA decision log and a separate collection receipt binding its decision ID, trace ID, exact input digest, bundle revision and observation time;
3. separate effect observation bound to the same decision and exact operation.

## Result semantics

- \`POLICY_DENIED_UNRECONCILED\`: OPA denied, but no effect observation exists. Absence of observation is **not** proof of no external effect.
- \`POLICY_DENIED_NO_EFFECT_CONFIRMED\`: an independent owner observation confirms that the denied operation was not applied.
- \`POLICY_ALLOW_ONLY\`: OPA allowed the operation, but no external effect was independently observed.
- \`ACKNOWLEDGED_NOT_RECONCILED\`: provider acknowledgement exists, but independent observation is absent or shares a common-cause dependency.
- \`DENIED_WITH_UNRECONCILED_ACK\`: a provider acknowledgement followed a denial; this is a reconciliation signal, not proof of effect.
- \`DENIED_WITH_UNRECONCILED_EVIDENCE\`: evidence exists after a denial but independence/common-cause exclusion is insufficient.
- \`DENIED_EFFECT_UNKNOWN_REQUIRES_RECONCILIATION\`: independent observation is available but the external outcome remains unknown.
- \`EFFECT_RECONCILED\`: exact decision/operation binding plus independent resource observation confirms application.
- \`EFFECT_CONFIRMED_NOT_APPLIED\`: independent resource observation confirms no effect after an allow.
- \`PARTIAL_EFFECT_REQUIRES_RECONCILIATION\`: a partial effect was independently observed.
- \`UNAUTHORIZED_EFFECT\`: a protected effect was independently observed after the policy decision denied the operation.
- \`INDETERMINATE\`: masked/erased inputs, invalid/stale collector receipt, unknown cutoff or missing policy bundle correspondence.
- \`EFFECT_BINDING_MISMATCH\`: effect evidence refers to a different decision or exact operation.
- \`DECISION_SCOPE_MISMATCH\`: OPA policy path or input differs from accepted intent.

The report always sets \`dispatch_eligible=false\`.
## Run

    cargo fmt --manifest-path lise-benchmarks/opa-effect-binding-reference/Cargo.toml --check
    cargo check --all-targets --manifest-path lise-benchmarks/opa-effect-binding-reference/Cargo.toml
    cargo test --all-targets --manifest-path lise-benchmarks/opa-effect-binding-reference/Cargo.toml
    cargo run --manifest-path lise-benchmarks/opa-effect-binding-reference/Cargo.toml -- \
      lise-benchmarks/opa-effect-binding-reference/fixtures/opa-effect-fixture.json \
      /tmp/opa-effect-binding-report.json

## Interoperability notes

- OPA's \`decision_id\`, \`trace_id\`, \`span_id\`, \`bundles[*].revision\`, \`path\`, \`input\`, \`result\` and \`timestamp\` are consumed using the documented decision-log structure.
- The input digest is calculated over compact serde_json serialization of the frozen fixture input. A production adapter needs a versioned canonicalization contract and must account for collector redaction/masking.
- If OPA marks input fields erased or masked, this reference refuses to claim exact-operation coverage.
- The OPA collector receipt is separate from the OPA log. It is synthetic and untrusted here; a production integration requires authenticated collector identity, durable delivery semantics, gap/cursor accounting and independent integrity verification.
- A provider acknowledgement is not an independent observation. A resource-owner read or independently controlled reconciliation source is required.
- Common-cause dependencies are not counted as independent corroboration.

## Claim ceiling

The bundled fixture is synthetic. Its collector receipt and owner observation are not cryptographically authenticated. This crate does not connect to OPA, perform a policy query, execute an external operation, verify a provider, or establish real-world source independence.
