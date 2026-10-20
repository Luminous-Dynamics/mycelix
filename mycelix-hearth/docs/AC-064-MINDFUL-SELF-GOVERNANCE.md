# AC-064: Mindful Self-Governance and Personal Agency

## Purpose

Hearth should help people deliberate about their own choices without turning inner life into a governance data source.

AC-064 therefore defines a privacy-preserving self-governance envelope. It records commitments and decision lineage, not reflective prose or inferred psychological state.

## Core principle

**Self-governance precedes collective governance.**

A client may help a person:

1. pause;
2. clarify intention;
3. inspect evidence;
4. consider alternatives;
5. preview consequences;
6. confirm or revise the choice.

These are optional supports. The kernel must not require meditation, emotional disclosure, agreement with a recommendation, or a particular mental state.

## Privacy boundary

The `SelfGovernanceEnvelope` contains:

- decision identity;
- an opaque intention commitment;
- optional commitments for declared values, alternatives, expected consequences, and reflection;
- evidence references;
- reversibility class;
- disclosure boundary;
- supersession/outcome lineage.

The actual reflective material may remain local/private and is represented by a cryptographic commitment.

The envelope therefore proves **that a commitment was made**, not **what a person secretly thought**.

## Non-authority invariant

Self-governance data must never be interpreted as:

- reputation;
- voting weight;
- economic entitlement;
- access privilege;
- civic eligibility;
- a wellness/compliance score;
- evidence of moral worth;
- evidence of psychological health;
- evidence that a person truly values something because a model inferred it.

Refusing to disclose reflective material must not lower a person's civic standing.

## Reversibility

Every envelope records a coarse reversibility class:

- `EasilyReversible`
- `ReversibleWithCost`
- `DifficultToReverse`
- `Irreversible`

This is decision-support metadata. It does not impose a universal risk threshold.

## Deliberative connection

Hearth decisions remain the collective authority boundary. Self-governance is upstream of collective choice:

`personal reflection → voluntary commitment → collective deliberation → governance decision`

A later outcome can be linked without rewriting the original choice.

## Append-only lineage

Self-governance envelopes use explicit `supersedes_ref` and `outcome_ref` lineage rather than destructive rewriting. Changing one's mind becomes historical information rather than a hidden contradiction.

## Scope

The first executable tranche lives in `hearth-types` so Hearth zomes and clients can share one neutral contract before any DHT mutation surface is introduced.

No DNA change is made in this tranche.

No mental-state inference or behavioral scoring is introduced.

## Qualification

Repository CI is the qualification source. No local cargo test pass is claimed from an environment without the repository checkout.
