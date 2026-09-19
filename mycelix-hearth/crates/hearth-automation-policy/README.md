# hearth-automation-policy

Deterministic authority evaluation for Hearth household automation.

This crate consumes `hearth-automation-types` contracts and an evidence-bearing
`AuthoritySnapshot`. It does not query Holochain or execute devices.

## Policy floor

- restrictions override capability grants;
- A3 reversible actions may use an existing Hearth autonomy capability;
- A4 consequential actions require a scoped grant, explicit approval, or a
  household decision — a legacy capability string alone is intentionally too
  broad;
- A5 critical actions inherit the ontology requirement for explicit approval,
  household decision, or prohibition;
- `ManualOverride` inhibits A3+ side effects immediately;
- approvals are unique by approver and bound to an exact intent and capability;
- expired grants/approvals/decisions never authorize execution;
- scoped grants bind semantic target, operation, maximum consequence and time.

## Existing Hearth compatibility

`AuthoritySnapshot::from_legacy_profile` maps the current
`AutonomyProfile.capabilities` / `restrictions` string lists into policy facts.
That preserves existing low-risk delegation while allowing new automation
policy to become stricter for consequential actions without a breaking
migration of the autonomy entry schema.

The eventual Holochain adapter should fetch the latest autonomy profile and bind
its action hash as `profile_ref`; policy evaluation then emits that reference in
`AuthorityEvidence`.

## Not in this crate

- DHT entry definitions;
- guardian/decision lookup;
- signature verification;
- device execution;
- approval UI;
- emergency automation semantics beyond manual override.

Those remain explicit later boundaries.
