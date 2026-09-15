# AC-003 — Institutional Relationship Graph Contract

Status: draft implementation contract

Parent: AC-002 capture observation contract

## Purpose

AC-003 defines the typed, provenance-bearing graph that later anti-capture analysis can inspect. It represents authority, delegation, ownership, influence, procurement participation, decision authorship, conflict disclosure, appeals, and audits without treating those relationships as proof of wrongdoing.

The graph is descriptive infrastructure, not an accusation engine.

## Core rules

1. **Opaque references, minimal attributes.** Graph nodes contain identifiers and node classes only; they do not carry profiles, ideology, race, wealth, reputation, addresses, or other broad personal attributes.
2. **Private people are credentials, not profiles.** A natural person needed for beneficial-ownership analysis can appear as an opaque `PrivatePersonCredential` reference.
3. **Influence is legitimate by default.** Lobbying, submissions, advisory work, campaigning, and other influence are represented as typed relationships. Their existence is not itself a corruption finding.
4. **Public power cannot be fully opaque.** Authority, delegation, public influence, decision authorship, conflict disclosures, appeals, audits, and key procurement relationships cannot use the fully confidential disclosure tier.
5. **Beneficial ownership balances oversight and privacy.** Beneficial-ownership edges may be public or accessible on a legitimate-interest basis, but cannot be hidden as confidential credentials from all oversight.
6. **Every edge has provenance.** Missing or malformed provenance fails closed.
7. **Corroboration means plural sources.** An edge marked `Corroborated` requires at least two distinct source references.
8. **Challenges remain visible.** An edge marked `Challenged` requires challenge provenance rather than silently replacing the original assertion.
9. **Temporal lineage is explicit.** If both validity bounds are supplied, the end must be strictly later than the start.
10. **Topology is typed.** Authority must target public institutions/offices; influence must target public decision-making; ownership must target organizations/legal entities/assets; procurement participation must target procedures/contracts.

## Disclosure classes

- `PublicMetadata` — relationship metadata is publicly inspectable.
- `LegitimateInterest` — relationship metadata is available to qualified oversight under a legitimate-interest basis.
- `ConfidentialCredential` — the system stores only a confidential credential/reference.

AC-003 rejects `ConfidentialCredential` for relationships where total opacity would defeat public accountability.

## Assertion states

- `Declared`
- `Corroborated`
- `Challenged`

There is intentionally no `True`, `False`, `Guilty`, or `Corrupt` relationship status.

## Initial relationship vocabulary

- `Authority`
- `Delegation`
- `Ownership`
- `Influence`
- `ProcurementParticipation`
- `DecisionAuthorship`
- `ConflictDisclosure`
- `Appeal`
- `Audit`

This is the minimum graph needed for later concentration, conflict, and provenance analysis.

## Privacy and transparency asymmetry

AC-003 starts to operationalize the AC-001 principle that transparency obligations should increase with public power.

It does this narrowly. It does **not** publish private-person profiles. Instead it distinguishes between public metadata, legitimate-interest access, and confidential credentials, while preventing authority-bearing relationships from disappearing behind the strongest secrecy class.

## Beneficial ownership

Ownership edges support:

- direct or beneficial interests;
- known ownership shares expressed in basis points;
- unknown shares (`None`) without silently treating unknown as zero;
- privacy-aware access using `LegitimateInterest`.

Shares above 100% are invalid.

## Influence

Influence edges require a disclosure reference and support:

- meetings;
- written submissions;
- advisory roles;
- campaign finance;
- public communication;
- digital campaigns;
- named custom channels.

The model is intentionally neutral about whether the influence is beneficial, ordinary, excessive, or improper. That judgment belongs in later analysis and human/legal review.

## Corroboration and challenge

A single source can establish a declared graph assertion. It cannot label itself corroborated.

`Corroborated` requires at least two distinct source references.

`Challenged` requires one or more challenge references so disagreement is preserved as evidence rather than erased.

## Non-goals

AC-003 does not:

- calculate a capture score;
- infer corruption;
- expose unrestricted private-person identity data;
- decide which lobbying positions are legitimate;
- define legal disclosure obligations for every jurisdiction;
- adjudicate conflicts of interest;
- impose sanctions;
- replace BODS, OCDS, or jurisdictional registers.

Downstream adapters should map external standards into these typed relationships rather than replacing those standards.

## Qualification gate

Before AC-003 is qualified infrastructure:

1. `cargo test -p civic-types` passes on the exact subject;
2. warnings-denied Clippy passes;
3. mutation tests prove that provenance, corroboration, challenge, topology, and visibility guards fail closed;
4. privacy review confirms private-person nodes cannot accumulate profile attributes through this API;
5. standards review maps ownership to BODS-compatible identifiers and procurement relationships to OCDS-compatible identifiers where applicable;
6. independent review verifies that graph assertion states cannot be interpreted as adjudicated wrongdoing.

## Next tranche

AC-004 should consume this graph plus AC-002 observations to calculate interpretable institutional metrics such as concentration, opacity, contestability, and evidence deficits. AC-004 must output observations/signals only, never sanctions or guilt labels.
