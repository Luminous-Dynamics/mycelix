# D6B — Leptos reference-node cockpit

D6B turns the D6A deterministic cockpit projection into an actual browser UI using Leptos CSR.

## Architecture

```
Integral semantics
      ↓
Mycelix reference trace
      ↓
trace validation
      ↓
D6A deterministic cockpit projection
      ↓
Leptos presentation
      ↓
human inspection / review / appeal
```

The UI is intentionally downstream of the authoritative reference-model trace. It does not create evidence, mint authority, reinterpret provenance, or convert a recommendation into a decision.

## Human-facing contract

The cockpit answers:

1. What happened?
2. Who produced it?
3. What evidence supports the statement?
4. What authority was actually exercised?
5. What remains uncertain?
6. Is this a recommendation or a decision?
7. What can be changed, reversed, challenged, or appealed?
8. Which generation and origin does the evidence belong to?

Progressive disclosure is represented by Summary, Rationale, Assurance, and Technical levels.

## Scenario corpus

The initial UI exposes eight deterministic scenarios:

- normal flow;
- recommendation declined;
- uncertain observation;
- conflicting evidence;
- foreign evidence;
- stale design;
- appealed outcome;
- no-Symthaea fallback.

The scenario selector is presentation-only at this stage; the machine-readable trace remains the source of truth.

## Technology

The UI uses Leptos 0.8.21 CSR and Trunk. Leptos supports client-side rendering with its `csr` feature, and its documented Trunk workflow uses a WASM target and an `index.html` shell.

This is a reference/demo surface, not a claim of production readiness, Integral ratification, economic validity, security/privacy compliance, or human-outcome improvement.

## Next increment

D6C should connect the scenario selector directly to executable adversarial fixtures so each scenario is validated or rejected by the same reference-model rules before being rendered. That will turn the current visual scenario corpus into a deterministic UI conformance harness.
