# Financial frontend semantic mapping

This document defines how the Finance workspace composes with the canonical Mycelix Leptos Core.

## Principle

Frontend state dimensions are orthogonal unless the domain contract explicitly composes them.

```text
availability != freshness != evidence availability != qualification != authority != execution != finality
```

A UI component must not use one dimension as a proxy for another.

## Canonical core dimensions

`mycelix-leptos-core` already provides reusable presentation semantics for:

- availability (`Live`, `Mock`, `Empty`, `Locked`, `Degraded`, `Unavailable`);
- freshness (`Fresh`, `Aging`, `Stale`, `Unknown`);
- connection/provider state;
- shared trust/provenance-oriented presentation;
- common application shell/components.

These should be reused where applicable rather than redefined by Finance.

## Finance dimensions

Finance adds domain-specific state that the generic UI core cannot safely infer:

- `Observed` — a source observation exists;
- `Verified` — the relevant verification rule passed;
- `Qualified` — the applicable qualification policy passed;
- `Authorized` — authority has been granted for the exact subject;
- `Pending` — processing/authority exists but the requested terminal outcome is not established;
- `Included` — an external rail reports inclusion/acceptance, not necessarily finality;
- `Finalized` — the selected finality profile's condition is satisfied;
- `Reconciled` — the relevant records agree under the selected reconciliation policy;
- `Disputed` — an explicit dispute exists;
- `Superseded` — a successor state/projection replaces the prior subject;
- `Indeterminate` — the system cannot safely classify the outcome.

## Prohibited mappings

Do not implement shortcuts such as:

```text
availability Live       -> financial Verified
freshness Fresh         -> financial Qualified
connection healthy     -> settlement Finalized
transaction included   -> commercial satisfaction
visible balance        -> accounting authority
AI recommendation      -> Authorization
```

Each of these would strengthen the claim beyond what the source dimension establishes.

## Composite presentation

A production financial object may legitimately display several dimensions at once:

```text
Settlement A
  Availability: Live
  Freshness: Fresh
  Evidence: Present
  Qualification: Qualified
  Authority: Authorized
  Rail observation: Included
  Finality: Unknown
```

This is preferable to a single green `Success` state because it preserves the uncertainty boundary.

## Evidence and provenance

Evidence/provenance components should remain explanatory surfaces, not verifiers.

A provenance trail can show:

```text
source observation
  -> verification result
  -> policy evaluation
  -> economic event
  -> external receipt
```

but the visual chain itself does not establish that the underlying links are valid.

Missing, unavailable, stale and unknown inputs should remain visible at their original semantic location. They must not be filtered out to make an explanation appear complete.

## Authority

Authority is never inferred from:

- route visibility;
- button visibility;
- an institution profile label;
- client state;
- connection status;
- presence of a Holochain record;
- an AI recommendation;
- successful transport.

Production material actions require the separately authenticated and authorized control boundary.

## Accessibility

Every consequential semantic distinction must be available without relying solely on color, animation, icons or hover interaction.

Current W3C guidance identifies WCAG 2.2 as the latest WCAG 2.x recommendation. See the W3C WCAG 2.2 recommendation.

## Integration rule

Finance-specific UI components should migrate generic behavior into `mycelix-leptos-core` once the shared `mycelix-leptos-ui` source distribution is restored and qualified under #89.

Finance should retain only domain-specific semantics and adapters.

## Qualification boundary

A UI test can establish that the frontend renders the supplied state faithfully.

It cannot establish the correctness of the underlying accounting, authorization, evidence, settlement, legal or regulatory theorem unless the tested system boundary explicitly supplies those proofs.