# Farmer UX-001: Field-ready decision-to-outcome workspace

**Status:** Research and contract draft. Not an implementation or field-validation claim.  
**Tracking:** [FARMER-UX-001](https://github.com/Luminous-Dynamics/mycelix/issues/5194)  
**First vertical slice:** report an irrigation problem and reach a farmer-decided service outcome.

## Product thesis

Make the interface a **reliable bridge from a farmer's intention to a real-world outcome**, not a dashboard that requires the farmer to translate charts into action.

Primary product metric: the proportion of eligible farmer requests that reach a farmer-accepted outcome within the farmer-declared useful window, with all failures and unresolved cases retained in the denominator.

Do not use registrations, clicks, recommendations emitted, scheduled tasks, map coverage, or model confidence as substitutes for farmer benefit.

## First navigation hypothesis

Test this navigation with farmers rather than assuming it is correct:

- **Today:** urgent issues, deadlines, pending decisions, and resumable work.
- **Land:** fields/assets and time-scoped spatial/condition evidence.
- **Work:** tasks, people, equipment, maintenance and status.
- **Services:** requests, quotes, bookings, service delivery, complaints and rework.
- **Sales:** harvest/stock, offers, orders, fulfilment and payments.

Show only relevant features for each role and context. The interface must not force livestock keepers through crop workflows or smallholders through commercial-fleet dashboards. Use action-first cards and short task flows; maps and charts are supporting views.

## Recommendation card contract

Every material recommendation or warning should expose:

1. **Action:** plain-language action or explicit abstention.
2. **Reason and timing:** why it matters now, deadline/useful window, and consequences of waiting where known.
3. **Evidence:** source/date, observation versus forecast/model/report, freshness, coverage and relevant disagreement.
4. **Limitations:** unknowns, confidence meaning, missing inputs, and what the model does *not* establish.
5. **Resources and trade-offs:** estimated cost/time/water/parts when supported; units and source; separate facts from estimates.
6. **Agency:** accept, decline, defer, correct input, ask a person, or report a problem.
7. **Provenance:** durable receipt for user actions and clear local-save/sync/confirmed state.

No confidence number should be displayed without calibration semantics and a user-understandable meaning. Do not present a model suggestion as a diagnosis, quote, booking, completed job, regulatory approval or physical observation.

## First workflow

```
observe problem
  -> save a local receipt + opaque site reference
  -> inspect relevant field/asset evidence and freshness
  -> choose app, phone, agent, cooperative or another supported channel
  -> receive and inspect a complete quote
  -> explicitly accept or decline it
  -> follow scheduled, delayed, unavailable, cancelled or completed states
  -> accept, dispute or request rework
  -> export/correct the record or raise a complaint
```

A forecast is not stock; an offer is not an accepted quote; a quote is not a booking; a booking is not completed work; provider completion is not farmer acceptance; submission is not external authority acceptance. Keep these distinctions visible in the UI and machine-readable model.

## Field constraints

- Offline is a normal state, not an error page. Support local capture, previously synced data, task preparation, and a visible outbox.
- Every record state must be explicit: local-only, pending sync, confirmed, stale, conflict, rejected, unknown, cancelled, completed-not-yet-accepted, accepted, disputed or rework requested as applicable.
- Never silently overwrite a confirmed commitment or silently discard an offline event.
- Preserve human mediation and existing trusted service channels.
- Hide precise farm coordinates and sensitive commercial data by default; expose sharing purpose and recipient; support corrections, export and a clear exit path.
- Localize content with real language review. Voice/photo inputs are optional paths, not mandatory; respect permissions, connectivity and data minimization.
- For outdoor primary controls, aim for 44 x 44 CSS px targets where feasible, exceeding WCAG 2.2 AA's 24 x 24 minimum criterion. Meet WCAG 2.2 AA across supported surfaces.

## Shared subsystem ownership

- **Mycelix:** purpose-bound consent, shared commitments, corrections, disputes and portable records.
- **Symtropy:** task/workflow composition, resource scenarios and bounded service planning.
- **Symthaea:** evidence-aware reasoning, competing hypotheses, uncertainty and abstention.
- **Sol-Atlas:** place/asset references and source/version/coverage-bound spatial evidence.

Use existing service lifecycle and route contracts. This UX work must not add a second farm identity, route engine, inventory authority, trust score, quote/booking lifecycle or payment authority. See the issue above for the current cross-repository references.

## Proposed gates

### Synthetic / implementation gates

- 100% of locally acknowledged events survive process-kill/restart, retry and reconnection cases in the declared fault suite.
- Zero duplicate booking/payment/inventory/completion effects on replay.
- Zero booking without a complete, unexpired, explicitly accepted quote.
- Zero route time inferred from geodesic distance alone.
- Zero stale/unknown/conflicting evidence silently promoted to a verified fact.
- 100% of critical paths preserve explicit no-service, cancelled, late, disputed and unresolved outcomes.
- Deterministic golden cases are checked against a separately written oracle; do not have the production validator generate its own expected results.

### Usability / accessibility targets

These are proposed targets to evaluate, not measured results:

- At least 90% of pilot participants complete each critical task without facilitator intervention after brief orientation.
- Median routine observation capture under 30 seconds on an already-configured farm profile; report P90, omissions and error rate too.
- At least 90% can distinguish measured, forecast/modelled, pending-sync and confirmed-commitment states in comprehension checks.
- WCAG 2.2 AA conformance for supported UI surfaces; target 44 x 44 CSS px primary touch controls where feasible.
- Measure time to first useful action, initial payload, sync bytes, recovery after interruption, battery cost where feasible, and task completion under a declared low-bandwidth/low-end device profile.

### Field-outcome gates

Before a pilot, define the farm task, season, eligible-request denominator, baseline, comparator, outcome window, inclusion criteria, safety guardrails and analysis plan. Report service acceptance within useful windows, time/cash cost, no-provider/declined/cancelled/no-show/late/rework/disputed outcomes, workload, agency/trust, and exclusions. Do not make universal yield, income or water-saving claims without local evidence and suitable comparison.

## Research anchors

- Gatti et al. (2026), *Journal of Rural Studies*, “Not-so-digital platforms?” https://doi.org/10.1016/j.jrurstud.2026.104210
- “Field-Ready HCI” (2026), *Applied Sciences*. https://doi.org/10.3390/app16146985
- FAO Digital Services Portfolio: https://www.fao.org/digital-services/about/
- W3C, Web Content Accessibility Guidelines 2.2: https://www.w3.org/TR/WCAG22/

These sources motivate design choices; they do not establish local user needs, South African field outcomes, route accuracy, or production readiness. Validate with farmers, extension workers, service providers and cooperatives before locking the information architecture.

## Supporting machine-readable artifacts

The same review branch provides supporting artifacts:

- [Synthetic workflow corpus](farmer-ux-001-cases-v1.json) and [its JSON Schema](farmer-ux-001-cases-v1.schema.json). The corpus now contains 22 cases, including unauthorized delegated requests, quote-scope mismatch, modelled-versus-observed route semantics, precision leakage, digest-without-source-authentication, ambiguous fee translations, provider-capacity withdrawal, export/sync-state fidelity, deliberate deferral, and no-show denominator retention.
- [Role/task matrix](farmer-ux-001-role-task-matrix-v1.json) and [its JSON Schema](farmer-ux-001-role-task-matrix-v1.schema.json). Nine proposed role profiles use the same underlying interaction primitives. Initial P0 roles are smallholder crop farmers, extension officers/agronomists, and irrigation/water service providers. P0/P1/P2/P3 are sequencing hypotheses to validate—not a market-size claim.
- [State and microcopy deck](farmer-ux-001-copy-v1.json) and [its JSON Schema](farmer-ux-001-copy-v1.schema.json). It now proposes 19 English user-visible states, including a dedicated `OutcomeUnknown` state for a potentially submitted request without authoritative result. Copy must not imply success, rejection, or safe automatic retry when outcome is unknown. Copy is not yet translated, comprehension-tested, or field-approved.

A standalone structural check was previously run against the then-committed JSON artifacts: 22 cases / 22 unique IDs and 9 role profiles / 9 unique IDs passed; the copy deck has since grown from 18 to 19 unique states with the added OutcomeUnknown copy contract. Re-run the verifier on the updated exact head. This is not a full standards-compliant JSON Schema engine, independent semantic oracle, CI qualification, usability result, or field validation.

## Research implications and design limits

The 2026 *Field-Ready HCI* paper presents a conceptual model for mobile agricultural tools and explicitly treats benefits as conditional on context and user characteristics; its illustrative case is hypothetical, not field-trial evidence. The June 2026 *Journal of Rural Studies* paper reports limited or short-lived use despite high registrations/subscriptions in its examined settings and describes socio-technical misfit, low-tech channels, intermediaries, and farmer autonomy as relevant factors. Treat these as research inputs requiring local validation, not universal laws about every farmer or service.

Use WCAG 2.2 AA as a baseline for applicable web UI conformance. The 44 x 44 CSS px primary-touch target is a product usability target informed by WCAG 2.2's AAA enhanced target criterion; WCAG 2.2 AA's minimum target-size criterion is 24 x 24 CSS px with exceptions. A 44 px target preference does not itself establish accessibility conformance.

## Multi-level evaluation matrix

See [`farmer-ux-001-evaluation-matrix-v1.json`](farmer-ux-001-evaluation-matrix-v1.json) and its schema for a machine-readable evaluation contract. It separates three layers rather than compressing them into one score:

- **Model:** factual/source fidelity, uncertainty/abstention, case-oracle agreement, and calibration only when outcome labels and data volumes justify it.
- **System:** full task journey, offline/replay integrity, state comprehension, recovery, accessibility and constrained-device behavior.
- **Process:** farmer agency, consent, human-mediated channels, equity, correction/repair, adverse outcomes and independently measured field benefit.

This follows IFPRI's May 18, 2026 review of agricultural AI advisory evaluation, which recommends model-, system-, and process-level assessment and highlights that technical scores miss user-journey, language, trust, and equity failures: https://www.ifpri.org/blog/beyond-the-model-evaluating-ai-agricultural-advisory-systems-so-they-work-in-the-field/

The matrix provides 12 metrics, 16 hard gates, denominators and claim-boundary rules. Proposed targets (90% task success/comprehension, under-30-second median capture, zero data-loss/duplicate-effect faults) are targets to test, not measured values. A schema or oracle pass cannot be reported as user validation, field readiness, or impact.

