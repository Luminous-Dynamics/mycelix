# mycelix-business-coordination

`mycelix-business-coordination` builds on `mycelix-business-core` with versioned action and objective contracts plus coordination-envelope validation.

It remains non-authoritative. It does not mint capability, authority, money, inventory, work schedules, property rights, or legal state. It only validates that a prepared action satisfies the declarative requirements of an externally governed contract.

## Scope

The crate provides:

- Action Contracts with required reservation domains, freshness bounds, aggregate-policy keys, reversibility class, and maximum prepared-action lifetime;
- Coordination Envelopes that bind a prepared action to those contract requirements;
- Objective Contracts separating desired outcomes, diagnostic metrics, protected constraints, resilience floors, and evaluation horizon;
- strict unknown-outcome retry policy as the safe default.

This is a contract layer, not a workflow engine or distributed transaction coordinator.
