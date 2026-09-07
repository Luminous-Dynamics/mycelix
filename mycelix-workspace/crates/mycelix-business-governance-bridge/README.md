# Mycelix Business Governance Resolution Bridge v0.1

This crate owns one explicit cross-domain mapping:

```text
Governance AuthorizedResolutionOutcomeV1
        ↓ bridge v1
Business terminal expectation binding
```

It is intentionally separate from both Governance and the generic Business qualification crates.

## What the bridge proves

Given:

- one typed `AuthorizedResolutionOutcomeV1`;
- the exact Governance `QualifiedInputRef` attributed to that outcome by an owning verifier;
- the current Business `QualificationCut`;

the bridge verifies:

- the source domain is exactly `governance`;
- the source semantic profile/version exactly matches the typed outcome kind;
- the source is present in the current qualification cut;
- refund material maps deterministically into one Business `CommittedIntent`;
- retained exception domain/id maps exactly into one `DomainExceptionRef`.

## Bridge-owned refund mapping

Governance does not name Finance execution profiles.

Bridge v1 deliberately owns this mapping:

```text
governance.authorized-refund-resolution@1
        ↓ business.bridge.governance-resolution-expectation@1
finance.refund@1
```

The Business logical intent ID is the exact Governance-authorized refund `effect_id`.

The Business operation commitment is the exact `RefundResolutionV1::canonical_material_v1()` value.

Changing this mapping requires a new bridge version rather than silently reinterpreting historical v1 evidence.

## What the bridge does NOT prove

This crate does not fetch Governance state and cannot prove that a caller-supplied typed outcome is actually the payload stored at the supplied source record.

That relation remains the responsibility of the Governance runtime verifier tracked separately in issue #321.

The intended production chain is:

```text
exact Governance record/action
        ↓ Governance-owned fetch + verification
verified AuthorizedResolutionOutcomeV1
        ↓ this bridge
CompensationExpectationBinding / ExceptionExpectationBinding
        ↓ Business terminal expectation qualification
actual terminal effect
```

The bridge also does not decide whether the Governance decision was legitimate, whether a refund has settled, or whether an exception should remain admissible. Those semantics remain with their owning domains.
