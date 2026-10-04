# AC-017 — Economic Substrate Integrity

Status: implementation candidate; reference-model tests not yet independently executed in CI.

## Purpose

AC-017 extends the anti-capture series from institutional integrity to economic substrate preservation.

The core proposition is:

> An economic system should distinguish productive surplus from consumption of the stocks that make future production possible.

The design is deliberately not a universal "flourishing score." It introduces dimension-specific accounts, explicit safety boundaries, append-only change events, evidence completeness, and action gating.

The governing rule is:

healthy dimensions cannot compensate for a breached hard boundary in another dimension.

## Research grounding

The design is informed by four bodies of work:

1. **Commons governance.** Elinor Ostrom's design principles include resource boundaries, congruence between benefits and provision, user participation, community monitoring, graduated sanctions, conflict resolution, recognition of self-governance, and nested enterprises. AC-017 therefore keeps monitoring local/accountable and treats the substrate itself as something that is monitored, not merely the behavior of users.
   Source: Nobel Prize lecture, https://www.nobelprize.org/uploads/2018/06/ostrom_lecture.pdf

2. **Natural-capital accounting.** The UN System of Environmental-Economic Accounting explicitly represents ecosystem extent, condition, services, and changes in ecosystem assets, including degradation and enhancement. It also notes that monetary valuation is not required: physical information can support decision-making without pricing nature. AC-017 therefore stores native units and does not require a single monetary valuation.
   Sources:
   https://seea.un.org/en/Introduction-to-Ecosystem-Accounting
   https://seea.un.org/en/methodology/ecosystem-accounting

3. **Inclusive wealth.** Inclusive-wealth approaches treat produced, human, and natural capital as part of the productive base on which future well-being depends. AC-017 translates that insight into a local, machine-readable substrate boundary model without collapsing all capital into one compensating score.
   Source: https://www.nature.com/articles/s41599-024-02659-5

4. **Metric corruption / Goodhart-Campbell effects.** High-stakes metrics can become targets and thereby distort behavior or be manipulated. AC-017 therefore makes substrate state primarily a constraint and audit signal, not a reward leaderboard or optimization target.
   Sources:
   https://doi.org/10.1111/j.1740-9713.2018.01205.x
   https://www.sciencedirect.com/science/article/pii/S2666389923002210

The OECD's 2024 trust survey provides an additional institutional design signal: trust is strongly associated with evidence-based decision-making, balancing current and future generations, political voice, accountability, fairness, and responsiveness. AC-017 therefore treats evidence completeness and governance response as part of the substrate boundary rather than optional reporting metadata.
Source: https://www.oecd.org/en/publications/oecd-survey-on-drivers-of-trust-in-public-institutions-2024-results_9a20554b-en/full-report/the-public-governance-drivers-and-personal-characteristics-shaping-trust-in-public-institutions_b27d32e4.html

## Current Mycelix alignment

AC-017 does not duplicate existing mechanisms:

- Finance already has an inalienable reserve, demurrage, commons composting, counter-cyclical TEND limits, and anti-reflexivity guards.
- PoG already binds some economic concepts to physical infrastructure.
- Temporal Economics already supports long-horizon commitments and covenants.
- DKG already supports provenance, attestations, contradiction detection, and time-sensitive confidence.
- Attribution already models dependency, usage, and reciprocity.
- Governance already has timelocks, voting, ethics disclosures, emergency limits, and anti-tyranny work.
- The AC-001 through AC-016 series already addresses capture, identity ambiguity, procurement concentration, evidence lineage, and adversarial composition.

The new primitive is the missing connective tissue: a common representation for the stocks whose depletion should constrain discretionary economic extraction.

## Reference model

The SDK module economics::substrate adds:

### SubstrateDimension

Six independent dimensions:

- Financial
- Physical
- Ecological
- Social
- Epistemic
- Institutional

The list is intentionally finite for the reference model but not presented as an eternal ontology.

### SubstrateAccount

Each account contains:

- native unit
- baseline/reference value
- current value
- boundary direction
- boundary value
- warning buffer
- hard/soft classification
- update timestamp

The model supports both:

- minimum boundaries, where lower values are dangerous;
- maximum boundaries, where higher values are dangerous.

### SubstrateEvent

An append-only event records:

- dimension
- signed delta
- event kind
- actor
- timestamp
- optional evidence reference

Boundary breaches are not rejected as invalid events. Hiding a bad measurement would corrupt the feedback system. Instead, the state records the breach and the action gate reacts to it.

### SubstrateGate

Actions are classified as:

- Discretionary
- Maintenance
- Restoration
- Emergency

Rules:

- missing required dimensions -> InsufficientEvidence;
- discretionary action during hard breach -> Blocked;
- maintenance/restoration during breach -> remains allowed, with warning;
- emergency during hard breach -> explicit escalation required.

This avoids the failure mode where "protect the substrate" accidentally prevents repairing the substrate.

## Non-compensation invariant

The aggregate report intentionally uses worst-case logic:

hard breach + healthy other dimensions = breached system.

There is no weighted average and no "offset" mechanism.

This is deliberate because a system should not be able to claim:

> strong financial performance offsets collapse of the water system.

The same principle applies to social trust, institutional legitimacy, and epistemic integrity.

## Evidence-completeness invariant

A missing substrate account is not equivalent to a healthy account.

For a policy that declares several dimensions required:

missing evidence -> insufficient evidence -> no discretionary authorization.

This prevents actors from selectively omitting the dimension in which their performance is weakest.

## Anti-Goodhart invariant

Substrate condition is not itself a leaderboard.

Do not use:

substrate_score -> reward -> greater substrate_score

because this creates a reflexive gaming loop.

Instead:

measurement -> boundary check -> accountability/correction.

Rewards may eventually consider demonstrated restoration or stewardship, but those mechanisms must remain separate from the safety boundary itself.

## Expected next integration

AC-017 currently stops at a pure reference model. The next layers should be implemented in this order:

1. Bind SubstrateEvent to DKG provenance and identity.
2. Add explicit measurement-verification classes without equating source reputation to truth.
3. Create adapters for existing Finance commons reserves, PoG physical assets, Attribution reciprocity, and Temporal commitments.
4. Extend Metabolic Oracle so substrate breaches influence policy recommendations as constraints, not as a composite score.
5. Add a governance action type for boundary changes with stronger evidence requirements than ordinary parameter changes.
6. Add deterministic multi-agent tests covering missing data, conflicting measurements, stale observations, malicious actors, and deliberate boundary gaming.
7. Add municipal/procurement fixtures where a financially attractive option degrades a protected substrate dimension, and verify that the economically attractive option is not automatically authorized.

## Safety properties to qualify

A future qualified implementation should establish at least:

1. Hard substrate breaches cannot be hidden by healthy dimensions.
2. Missing required measurements cannot be interpreted as healthy.
3. Maintenance and restoration remain available during breach.
4. Emergency paths cannot silently bypass hard boundaries.
5. Append-only evidence survives adverse outcomes.
6. Reordering observations does not change the result.
7. Repeated submissions cannot manufacture a healthier substrate state.
8. Boundary definitions are provenance-bound and cannot be changed by the subject being evaluated without the required governance process.
9. Reputation cannot recursively increase the power to define or rewrite the same substrate boundary.
10. A metric becoming consequential does not make the metric itself the only source of truth.


## Determinism note

The reference model uses integer substrate quantities (i128) rather than floating-point
state transitions. This is intentional: accounting state should not depend on IEEE-754
rounding or processing order. Floating-point normalization, when needed for presentation,
belongs at the reporting edge rather than inside the state transition model.

## Important limitation

AC-017 is a systems-design and reference-model step, not evidence that a particular real economy, institution, company, municipality, or ecosystem is healthy or unhealthy.

It provides a protocol for representing and constraining decisions under explicit local policy. The quality of the observations and the legitimacy of the boundary definitions remain separate empirical and governance questions.

## Thesis

The intended end state is not "central planning by score."

It is:

> decentralized economic agency operating inside explicit, observable, locally governed constraints that preserve the stocks required for continued agency.

That makes the economic system more like a metabolism:

production -> use -> observation -> maintenance/restoration -> renewal -> continued production.
