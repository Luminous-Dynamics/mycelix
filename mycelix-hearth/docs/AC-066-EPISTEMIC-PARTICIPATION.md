# AC-066: Epistemic Abstention, Dissent, and Reconsideration

## Purpose

A healthy self-governing system must make room for uncertainty.

A person should be able to say "I do not know", "I need more evidence", "I dissent", or "I changed my mind" without those states being silently converted into consent, bad faith, or a civic penalty.

## Participation states

The shared Hearth contract defines:

- `Support { option }`
- `Oppose { option }`
- `Abstain`
- `RequestMoreEvidence`
- `DissentWithoutBlock`
- `ReconsiderationRequested`

Only Support and Oppose contain a substantive option index.

These states are deliberately separate from the existing Vote type. No abstention, evidence request, dissent, or reconsideration request is automatically assigned vote weight.

## Core invariants

- Abstention is not consent.
- Silence is not agreement.
- Requesting evidence is not obstruction.
- Dissent does not imply bad faith.
- Changing one's mind does not invalidate the historical record.
- A model recommendation cannot override a human dissent by itself.
- Participation frequency or reflection state must not become a moral or civic-worth score.

## Reconsideration

`DecisionReconsideration` binds a historical decision to explicit evidence references.

It does not rewrite the prior decision.

A future coordinator may use that evidence to open a successor decision or deliberation phase according to the applicable governance rules.

## Why this belongs in Hearth

Self-governance is not only the capacity to choose. It is also the capacity to withhold judgment responsibly.

AC-064 provides private, voluntary reflection and commitment support.

AC-066 provides an external civic language for epistemic humility:

`reflect → deliberate → know / doubt → investigate → choose → act → reconsider`

## Boundary

This contract does not establish a universal abstention threshold, mandate participation, infer sincerity, or decide when reconsideration is politically justified.

Those remain explicit governance choices.

## Qualification

Repository CI is the qualification source. No local cargo test pass is claimed from an environment without the repository checkout.
