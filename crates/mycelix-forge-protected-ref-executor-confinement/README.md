# mycelix-forge-protected-ref-executor-confinement

FORGE-009I closes the principal-confinement gap between the exact gittuf marker policy and the exact FORGE-009F Git transaction plan.

The positive `ExecutorConstrainedGitRefTransactionPlanV1` requires:

1. the exact FORGE-009H marker-policy evidence;
2. the exact FORGE-009G policy-qualified FORGE-009F plan;
3. the exact FORGE-009F plan evidence commitment;
4. a closed-world principal set equal to every principal applicable to the marker ref;
5. one principal-to-executor binding for every authorized principal;
6. every executor bound to the exact same transaction-plan commitment;
7. an independent verifier accepting every binding.

This is deliberately stronger than merely naming a preferred executor. An extra authorized principal or a missing principal fails closed.

The result still does **not** prove hostile-host resistance or that an executor process cannot bypass its declared interface. A later sealed-executor theorem must establish that the concrete runtime exposes only the exact transaction capability.

## Qualification

Pinned Rust 1.96.0:

- scoped rustfmt;
- warnings-denied all-target/all-feature Clippy;
- all-feature tests.

The crate performs no repository mutation.
