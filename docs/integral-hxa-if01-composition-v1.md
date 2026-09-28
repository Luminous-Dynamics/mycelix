# HXA / IF01 Composition Assurance v1

## Purpose

This specification defines the bounded composition boundary between:

- HXA human-experience assurance;
- IF01 OAD -> COS interface assurance; and
- the existing COS semantic/provenance boundaries.

The purpose is not to create a new authority layer. It is to prevent individually
valid artifacts from becoming an invalid stronger claim when composed.

## Composition chain

A consequential path must remain explicit:

**recommendation -> interpretation -> explicit authorization -> semantic admission -> execution -> outcome -> contest/reversal**

No step may be inferred merely because a neighboring step succeeded.

### Required distinctions

| Boundary | Must remain distinct |
|---|---|
| Symthaea recommendation | human/community authorization |
| explanation | source evidence |
| authorization | execution |
| transport receipt | semantic admission |
| semantic admission | production authority |
| observed result | qualification |
| local observation | foreign recognition |
| uncertainty | certainty |
| decision | appeal/reversal |
| machine witness | participant evidence |

## Cross-layer invariants

The executable reference model in
`integral_hxa_if01_composition.rs` checks that:

1. AI recommendation cannot become authority by composition.
2. Execution cannot skip explicit authorization.
3. IF01 authorization references cannot launder AI authority.
4. Provenance-breaking transformations block composition.
5. Loss of uncertainty blocks composition.
6. Contestability cannot depend on the recommending AI.
7. Human override remains available before execution.
8. Missing IF01 semantic admission blocks execution.
9. Passing machine witnesses does not establish human outcomes.

## Assurance levels

### A0 — Architectural statement

The separation is documented.

### A1 — Reference-model conformance

HXA and IF01 executable witnesses pass independently and the cross-layer
composition witnesses pass.

### A2 — Production refinement

Production code demonstrates that the same boundaries survive implementation,
including exact build/source identity.

### A3 — Human behavioral evidence

Participants demonstrate appropriate reliance, independent contestability,
uncertainty interpretation, error detection, and effective override.

### A4 — Longitudinal evidence

Repeated use demonstrates whether the system preserves agency and remains
usable under changing contexts, incentives, workload, and system evolution.

A higher level never retroactively proves a stronger claim at a lower level;
each level has its own evidence obligation.

## Trust calibration

The target is **appropriate reliance**, not maximum trust.

Recent 2026 research reports that confidence displays can increase agreement
with AI while simultaneously increasing agreement with incorrect outputs, and
that uncertainty cues can have counterintuitive behavioral effects.
Accordingly, the Integral boundary should test what people *do*—accept,
reject, verify, override, appeal—not merely what they report believing.
citeturn0search2turn0search3

This also supports treating uncertainty as a first-class provenance property
rather than as decorative interface text.

## Claim ceiling

Even a fully passing A1 reference model does not establish:

- participant comprehension;
- appropriate real-world reliance;
- satisfaction;
- happiness;
- flourishing;
- legitimacy of Integral as a social system; or
- production implementation correctness.

Those require the appropriate higher-level evidence.

## Design consequence

This composition layer makes a useful architectural promise explicit:

> **Symthaea can make a decision easier to understand without becoming the reason the decision is legitimate.**

And the corresponding Mycelix promise:

> **Mycelix can make an action auditable without making auditability equivalent to consent, authorization, or human endorsement.**
