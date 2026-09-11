# mycelix-business-core

`mycelix-business-core` is a dependency-light contract crate for the Mycelix Business Fabric.

It does **not** own Commerce, Finance, Supply Chain, Praxis, Commons/Property, Governance/Xenia, Identity, Justice, or Symthaea state. It contains only cross-domain references, preparation/coordination envelopes, bounded delegation helpers, autonomy-degradation helpers, and invariant identifiers.

The crate is intentionally incapable of granting institutional authority. A capability descriptor says what can be done; an authority lease reference says what an external authority domain has granted. The crate validates bindings between those references but does not mint authority.

The normative architecture contract is `docs/architecture/BUSINESS_SAFETY_CONSTITUTION_V0_1.md`.

## Design rules

- capability is not authority;
- observation, estimate, forecast, proposal, authorized intent, execution attempt, receipt, and reconciliation remain semantically distinct;
- consequential actions bind observation, policy, and authority frontiers;
- prepared actions are short-lived and fenced;
- delegation may only attenuate authority;
- sibling delegated budgets share a conserved envelope;
- degraded epistemic state may preserve or reduce autonomy but never increase it;
- circuit breakers contract authority and require explicit re-arm;
- unknown external outcomes remain unknown until reconciled;
- the crate owns no vertical industry enums, currencies, countries, payment providers, or legal forms.

## Qualification status

This crate is an architectural contract surface. Its tests establish local type/invariant behavior only. They do not establish field qualification, jurisdictional correctness, or autonomous-business safety.
