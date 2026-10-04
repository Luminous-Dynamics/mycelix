# AC-018 — Substrate Impact and Reciprocity

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-018 adds the causal/reciprocity layer missing from AC-017.

AC-017 answers:

> What condition is the productive substrate in?

AC-018 answers:

> What action affected it, who can be responsibly attributed, what remains uncertain, and what restoration obligation follows?

The governing rule is:

unknown attribution is not zero impact.

An unresolved impact remains observable. It does not become harmless merely because a causal assignment is incomplete.

## Why this layer is necessary

Market pricing can correct some externalities when the external cost is made visible and assigned to the responsible party. The European Commission describes the Polluter Pays Principle as a response to market failure where prices do not fully reflect pollution costs; the principle shifts prevention, control, and remediation costs toward those responsible.

However, a price is not the same thing as causal truth or restoration.

The UN SEEA framework explicitly supports ecosystem accounting in physical terms and tracks degradation and enhancement. Its current research agenda includes attribution of degradation/enhancement to economic units and treatment of externalities.

TNFD's current implementation work likewise identifies data gaps, attribution complexity, and methodological consistency as practical barriers to identifying and disclosing nature-related impacts.

AC-018 therefore keeps three things distinct:

1. measurement of the impact;
2. attribution of responsibility;
3. remediation/restoration obligation.

None is silently substituted for another.

## Current Mycelix alignment

AC-018 composes existing infrastructure rather than replacing it:

- AC-001 through AC-016: institutional anti-capture, provenance, identity qualification, procurement robustness, and composition invariance;
- DKG: evidence, provenance, contradiction handling, and confidence;
- Identity: actor binding and revocation;
- Attribution: dependency, usage, and reciprocity;
- Finance: SAP/TEND economics and commons reserves;
- Commons: resource-local governance and inalienable substrate;
- PoG: physical grounding and infrastructure attestations;
- Temporal Economics: long-horizon commitments and future-generational/ecological covenants;
- Governance: dispute handling and constitutional authority.

AC-018 becomes the bridge between:

action -> impact -> responsibility -> restoration -> renewed capacity.

## Reference model

The SDK module economics::impact provides:

### ImpactDirection

Two semantic directions:

- Depletion
- Regeneration

A direction is retained separately from the numeric magnitude so a future adapter cannot accidentally reverse the meaning of a signed quantity.

### AttributionBasis

The reference model distinguishes:

- Direct
- Contractual
- Contributory
- SharedChain
- Unknown

Unknown is valid as an epistemic state but cannot be used to close an impact.

This is deliberate: an incomplete causal model should remain incomplete rather than being converted into a false 100% assignment.

### ImpactStatus

Impacts move through:

Open -> Challenged -> Attributed -> Remediating -> Restored

with Inconclusive available when evidence cannot support a resolution.

Evidence is preserved throughout the lifecycle.

### Explicit attribution shares

Attribution is represented in basis points rather than floating point.

10,000 bps means the full measured impact has been attributed.

Anything below 10,000 leaves an explicit residual rather than silently assigning the remainder to somebody or to nobody.

This also makes attribution composition deterministic.

### Restoration obligation

A depletion impact that reaches full attribution opens a restoration obligation in the native unit of the affected substrate.

The reference implementation does not convert that obligation to SAP or another currency.

That separation matters. Monetary settlement is a policy/market mechanism; restoration is an empirical/resource-state mechanism.

A future policy layer may use monetary instruments to satisfy or finance restoration, but the obligation itself remains anchored to the substrate.

## Anti-corrosion gate

AC-018 gates economic actions based on unresolved impact obligations:

- Maintenance may proceed so the substrate can be repaired.
- Restoration may proceed so an obligation can be satisfied.
- Discretionary extraction is blocked by outstanding restoration obligations.
- Discretionary extraction is also blocked when a depletion impact lacks sufficient attribution.
- Emergency activity may require explicit escalation when blocking obligations exist.

This creates a closed feedback loop:

economic extraction -> measured impact -> attribution -> obligation -> restoration -> renewed extraction capacity.

## No universal damage score

AC-018 intentionally does not create:

- a global impact score;
- a moral score;
- a single ecological/social conversion factor;
- a universal monetary price for nature;
- a guilt probability;
- a sanction recommendation.

Those would create a new optimization target and invite metric capture.

The preferred representation is a set of auditable records and constraints.

## Shared responsibility without false precision

A multi-actor impact can be represented explicitly:

- Actor A: 7,500 bps
- Actor B: 2,500 bps

The obligation remains tied to the measured impact as a whole.

The reference model does not automatically turn those shares into debt instruments. That final step should require an explicit jurisdictional, contractual, or governance rule.

This avoids embedding an accidental liability regime into a generic protocol.

## Disputes

Challenging an impact does not delete its evidence and does not erase an already-created obligation.

This is intentional.

Otherwise an actor could make a challenge and obtain the equivalent of:

challenge -> disappearance -> extraction resumes.

A production implementation will need a separate dispute-resolution state machine that can affirm, narrow, supersede, or vacate an obligation with explicit evidence and authority.

## Determinism and anti-manipulation invariants

The reference model establishes:

1. duplicate impact identifiers fail closed;
2. duplicate attribution actors fail closed;
3. attribution arrays are canonically ordered;
4. unknown attribution cannot masquerade as resolved attribution;
5. partial attribution cannot close a depletion impact;
6. a second obligation cannot be created by re-attribution when the original obligation already exists;
7. restoration progress is capped at the obligation amount;
8. restoration changes lifecycle state without deleting the original impact;
9. regenerative impacts do not appear as unresolved remediation exposure;
10. all accounting quantities use integers in the reference model.

These are deliberately composition-oriented safeguards.

## Research-derived architectural rule

The central architectural rule is:

> measurement, attribution, obligation, and payment must remain separable layers.

This is stronger than simply "internalize externalities."

Internalization may use:

- tax;
- fee;
- bond;
- insurance;
- reserve;
- procurement condition;
- contractual liability;
- direct remediation.

The protocol must not assume one instrument is universally correct.

What it should make difficult is:

> receive private benefit while the resulting impact remains unobserved, unattributed, or indefinitely externalized.

## Relationship to AC-017

AC-017 protects the substrate state.

AC-018 protects the causal feedback loop around that state.

Together:

substrate state + impact lineage + attribution + restoration obligation

provide a much stronger anti-corrosion foundation than either a reputation score or a market price alone.

## Next qualification tranche

The next tests should focus on:

1. repeated identical impact submissions;
2. concurrent/conflicting attribution proposals;
3. challenge -> re-resolution without obligation duplication;
4. partial responsibility across a supply chain;
5. impact records whose measurement changes after new evidence;
6. restoration evidence that is itself challenged;
7. privacy-preserving attribution using zero-knowledge proofs;
8. jurisdiction-specific remediation rules;
9. procurement decisions where a cheaper award creates a larger substrate obligation;
10. randomized operation order and canonical serialization invariance.

The most important adversarial scenario is:

> financially superior option + lower immediate price + hidden substrate damage.

The system should be able to make the hidden cost visible without inventing certainty, and should then compare options using explicit policy rather than a secret or universal morality score.

## Research grounding

UN SEEA Ecosystem Accounting:
https://seea.un.org/en/Introduction-to-Ecosystem-Accounting

UN SEEA Ecosystem Accounting methodology:
https://seea.un.org/en/methodology/ecosystem-accounting

UN SEEA research on attribution, degradation/enhancement, and externalities:
https://seea.un.org/en/content/seea-eea-revision-research-areas

EU Polluter Pays Principle:
https://environment.ec.europa.eu/economy-and-finance/ensuring-polluters-pay_en

TNFD 2026 Status Report:
https://tnfd.global/publication/tnfd-2026-status-report/

TNFD discussion of dependencies, impacts, data gaps, and attribution complexity:
https://tnfd.global/publication/dependencies-and-impacts-in-financial-portfolios/

Ostrom Nobel Prize lecture:
https://www.nobelprize.org/prizes/economic-sciences/2009/ostrom/lecture/

## Thesis

AC-017 says:

> Do not consume the foundations that make production possible.

AC-018 adds:

> When consumption does damage those foundations, do not let the damage disappear simply because responsibility, price, or evidence is inconvenient.

The resulting economic model is not anti-market.

It is an attempt to make market activity compatible with the continued existence of the substrate on which the market depends.
