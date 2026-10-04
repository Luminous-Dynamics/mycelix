# AC-019 — Governed Substrate Boundaries

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-019 closes a specific anti-corrosion gap in AC-017:

> a truthful ledger with a captured boundary is still a captured system.

A boundary determines when an economic activity is considered to have consumed too much substrate. Therefore the boundary itself must have provenance, versioning, authority, evidence, and an explicit change process.

The core rule is:

boundary relaxation is not ordinary configuration.

## Why it matters

The UN System of Environmental-Economic Accounting requires clearly defined accounting units and boundaries at suitable scales, and its research agenda explicitly considers attribution of ecosystem degradation and enhancement to economic units. SEEA also supports physical accounting without requiring every ecological quantity to be assigned a monetary price.

The planetary-boundaries literature likewise frames safe operating space in terms of scientifically informed limits rather than a single optimization score.

Those sources answer important measurement questions, but an operational civic/economic protocol still needs to answer:

> Who may change a boundary, using what evidence, with what delay, and under what conflict-of-interest rules?

AC-019 is the reference-model answer.

## Boundary revision model

A boundary revision carries:

- scope;
- governed subject;
- substrate dimension;
- native unit;
- boundary direction;
- boundary value;
- warning buffer;
- hard/soft status;
- revision identity;
- predecessor identity;
- proposal/effective timestamps;
- authority references;
- independent authority references;
- evidence references;
- rationale.

The predecessor chain prevents silent replacement.

Historical revisions remain part of the record.

## Tighten / relax asymmetry

The reference model intentionally makes safety asymmetric.

### Tightening

Making a boundary more protective can become effective without the relaxation cooling period.

Examples:

- increasing a minimum safety floor;
- decreasing a maximum tolerated quantity;
- increasing a warning buffer;
- converting a soft boundary to a hard boundary.

### Relaxation

Making a boundary less protective requires:

1. explicit relaxation or evidence-correction semantics;
2. evidence;
3. at least one independent authority under the default policy;
4. a cooling period before effectiveness;
5. a non-retroactive effective timestamp.

Examples:

- lowering an ecological minimum;
- raising a maximum pollution threshold;
- reducing the warning buffer;
- converting a hard boundary to a soft boundary.

This creates a safety ratchet without claiming that boundaries can never change.

Better evidence is allowed to change a boundary. It simply cannot make a safety reduction instantaneous and invisible.

## Conflict-of-interest rule

The subject governed by a boundary cannot be the sole independent authority for relaxing that boundary.

This is intentionally narrow.

The protocol does not attempt to infer every real-world conflict. It establishes a machine-checkable minimum condition that prevents the most direct self-relaxation loop.

Future governance layers can add stronger requirements:

- multi-house approval;
- affected-community approval;
- sectoral review;
- scientific review;
- constitutional review;
- judicial/ombuds review.

## No retroactive rewrite

A new revision cannot become effective before it was proposed or before its predecessor was effective.

This prevents a newly favorable boundary from rewriting the historical interpretation of older economic activity.

The economic ledger therefore becomes:

past state -> past boundary -> observed impact

rather than:

current boundary -> rewritten history.

## Deterministic current view

Only one revision may become effective for the same scope and dimension at a given timestamp.

Current-boundary lookup is therefore deterministic.

Submission order is not analytical authority.

## Relationship to AC-017 and AC-018

AC-017:

> protect the substrate state.

AC-018:

> preserve impact, attribution, and restoration obligations.

AC-019:

> protect the definition of the safety boundary itself.

Together:

boundary governance -> substrate measurement -> impact attribution -> restoration obligation -> renewed productive capacity.

## Anti-capture theorem sketch

A direct attack requires the adversary to defeat at least one of:

1. measurement provenance;
2. boundary authority;
3. independent approval;
4. relaxation delay;
5. non-retroactive history;
6. substrate action gating.

The system therefore moves from a single point of failure toward a layered defense.

This is not a proof of real-world capture resistance. It is a compositional design target that can be tested adversarially.

## Important limitation

AC-019 does not claim that any particular threshold is scientifically correct.

The protocol separates:

- who and how a boundary changes;
from
- what value the boundary should have.

Scientific validity, local legitimacy, legal authority, and community consent remain separate evidence domains.

That separation is intentional because a technically perfect governance process around a scientifically wrong boundary is still wrong.

## Next qualification tranche

The strongest tests should include:

1. self-relaxation;
2. hidden authority aliasing;
3. duplicate effective timestamps;
4. retroactive boundary changes;
5. mixed tightening/relaxation changes mislabeled as tightening;
6. evidence correction that actually relaxes a boundary;
7. concurrent proposals extending the same predecessor;
8. boundary changes during an active substrate breach;
9. governance authority revocation between proposal and effective time;
10. current-boundary lookup under arbitrary insertion order.

The final scenario is particularly important:

> a financially stressed system attempts to lower its own safety threshold before distributing surplus.

The protocol should make that a visible governance event, not a silent configuration mutation.
