# Integral reference-node demo: domain seam v1

## Purpose

This PR begins the Integral demo as a **bounded reference implementation**, rather than a mock UI or a claim that Mycelix implements Integral.

The demo needs one explicit domain seam before we connect the five Integral systems:

- proposal/design;
- community decision;
- explicit authorization;
- execution intent;
- observed work/result;
- assessment/recommendation;
- outcome;
- appeal/recovery.

The seam is intentionally compatible with the assurance work already present in IF01 and HXA.

## Five-system composition target

| Integral system | Demo role | Mycelix assurance boundary |
|---|---|---|
| OAD | versioned design/proposal | generation and provenance |
| CDS | human/community decision | decision is distinct from recommendation |
| COS | authorized execution + observation | authorization, admission, execution, observation remain distinct |
| ITC | contribution accounting projection | projection is not source observation |
| FRS | feedback/assessment | assessment/recommendation does not become authority |

Symthaea is an assistive layer across the human boundary: explanation, simulation, clarification, uncertainty presentation, and alternative views. It does not mint authority.

## Core invariant

The consequential path is:

`proposal -> design -> decision -> authorization -> semantic admission -> execution intent -> observation -> assessment/recommendation -> outcome -> appeal`

A shortcut is not treated as equivalent merely because the artifacts are individually valid.

In particular:

- recommendation != decision;
- decision != authorization;
- authorization != execution;
- transport/semantic admission != production authority;
- observation != qualification;
- assessment != source observation;
- foreign evidence retains foreign origin;
- explanation != evidence;
- AI assistance != human authorization.

## Demo claim ceiling

Passing the reference-model tests establishes only that these encoded boundary rules hold for the model.

It does **not** establish:

- Integral ratification;
- production correctness;
- economic validity;
- manufacturing qualification or safety;
- real-world node operation;
- security certification;
- human comprehension, satisfaction, happiness, or flourishing.

Those require the appropriate later evidence layers.

## Next PRs

1. **D1 — Domain seam** (this PR)
2. **D2 — OAD -> CDS -> COS vertical slice**
3. **D3 — ITC projection**
4. **D4 — FRS feedback loop**
5. **D5 — Symthaea/HXA human-facing boundary**
6. **D6 — reference-node cockpit UI**
7. **D7 — reproducible demo + evidence package**

The implementation should compose existing IF01/HXA/COS models rather than fork their semantics.
