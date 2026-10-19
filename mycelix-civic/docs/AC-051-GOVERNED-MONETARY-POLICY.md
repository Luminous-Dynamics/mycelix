# AC-051 — Governed Monetary Policy Application

## Purpose

AC-051 separates a monetary-policy recommendation from the authority event that
actually changes network parameters.

The existing Metabolic Oracle is useful as a deterministic policy recommender:
it observes network vitality and emits a bounded adjustment. That recommendation
is not, by itself, a sufficient economic governance event.

AC-051 adds an explicit `GovernedPolicyAdjustment` envelope containing:

- a unique decision ID;
- references to the observations considered;
- a policy/rule reference;
- the authority reference;
- the recommended adjustment;
- the authorization timestamp;
- a deterministic SHA-256 decision fingerprint.

The new `apply_governed_adjustment` path validates and records the decision
before mutating policy state.

## Economic rationale

### Real constraints remain separate from policy

Modern Monetary Theory emphasizes that monetary sovereignty changes the nature
of nominal financing constraints but does not remove real-resource constraints.
The relevant ceiling remains productive capacity, labor, materials, ecological
capacity, imports, and other non-financial limits.

Therefore the oracle's vitality score must remain a measurement/recommendation
primitive rather than being treated as a complete theory of inflation or
resource capacity.

Reference: Levy Economics Institute, *Fiscal Reform to Benefit State and Local
Governments: The Modern Money Theory Approach*.

### Commons governance requires explicit authority

Ostrom's institutional analysis emphasizes clear boundaries, participation in
rule making, monitoring, graduated sanctions, conflict-resolution mechanisms,
and rules congruent with local conditions.

The policy envelope makes the decision boundary explicit: an observation can
inform a recommendation, but an authority reference and policy rule are still
required before the recommendation changes shared state.

Reference: Ostrom Workshop, *How to Use the IAD Framework*.

### Information integrity is not economic truth

Stiglitz's information-economics work emphasizes that imperfect and asymmetric
information changes market outcomes.

AC-051 therefore records the evidentiary basis rather than claiming that the
oracle's recommendation is objectively correct. Provenance answers "what did
this decision rely on?" It does not answer "was the economics correct?"

### Institutions can become extractive

Acemoglu's institutional work emphasizes broad participation, constraints on
political power, and the danger of elites capturing institutions.

AC-051 adds a governance hook, but does not solve capture. The next governance
layers must make authority contestable, auditable, and subject to the local
constitution rather than merely accepting a non-empty authority reference.

### Compatibility with a conventional financial-regulatory lens

Scott Bessent's 2026 Treasury agenda emphasizes financial-system integrity,
modernized regulation, payments fraud controls, digital-asset infrastructure,
and market-based innovation.

The same separation between recommendation and authorized state transition is
useful under that lens: policy changes become attributable, reviewable events
rather than silent parameter mutation.

## What AC-051 guarantees

1. A governed policy decision must identify its observation basis.
2. A governed decision must identify the policy/rule basis.
3. A governed decision must identify the authority.
4. A decision ID cannot be applied twice to the same oracle instance.
5. Policy adjustment factors must be finite and non-negative on the governed path.
6. The decision has a deterministic content fingerprint.
7. The recommendation path remains available for simulation and analysis.
8. No claim is made that the vitality score alone measures inflation, output gaps,
   employment, external balance, or ecological carrying capacity.

## What AC-051 does not guarantee

AC-051 does not establish:

- that the named authority is legally legitimate;
- that the observation references are truthful;
- that the underlying observations are sufficient for macroeconomic policy;
- that the adjustment is optimal;
- that a SHA-256 fingerprint is a signature;
- that local/community currency policy has the same policy space as a sovereign
  fiat issuer.

Those claims belong to higher-level identity, evidence, governance, and
macroeconomic models.

## Next research boundary

A natural AC-052 target is a **real-resource policy observation layer** that
keeps capacity, labor, price, ecological, and external-balance observations
separate from the composite vitality score.

That layer should remain measurement-first: no universal price index, inflation
target, or automatic policy rule should be hard-coded until the relevant
jurisdiction and monetary regime are explicit.

## Research references

- Levy Economics Institute: Modern Money Theory and monetary sovereignty
  https://www.levyinstitute.org/publications/fiscal-reform-to-benefit-state-and-local-governments-the-modern-money-theory-approach/
- Ostrom Workshop: IAD Framework and design principles
  https://ostromworkshop.indiana.edu/doc/teaching/how-to-use--iad-framework-slides.pdf
- Nobel Prize: Joseph E. Stiglitz, information asymmetries
  https://www.nobelprize.org/prizes/economic-sciences/2001/stiglitz/facts/
- Nobel Prize: 2024 Economics Prize, inclusive institutions
  https://www.nobelprize.org/prizes/economic-sciences/2024/popular-information/
- U.S. Treasury: 2026 G20 Finance Track priorities
  https://home.treasury.gov/news/press-releases/sb0398
