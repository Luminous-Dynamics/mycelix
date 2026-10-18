# Farmer UX-001: low-fidelity wireframe and interaction specification

**Status:** Implementation-ready design hypothesis; not a rendered prototype and not field validated.  
**Tracking:** [FARMER-UX-001](https://github.com/Luminous-Dynamics/mycelix/issues/5194)  
**Primary target:** Mobile browser on an entry-level Android device, one-handed use outdoors, intermittent connectivity.  
**Secondary target:** Wider tablet/desktop workspace for extension workers, service operators and cooperatives.

## Design thesis

The first screen should not be an analytics dashboard. It should be an **attention queue tied to real decisions and work**. Maps, AI, forecasts and charts earn their space only when they help the user complete a task.

Avoid presenting the four projects as four different apps. The farmer sees one workflow with comprehensible evidence and explicit states; subsystem names remain internal implementation details.

## Screen 1 — Today

### Mobile layout, top to bottom

1. **Context bar**
   - Farm/operation label chosen by the user.
   - Current selected site/field when relevant.
   - Persistent sync chip using the canonical copy deck: “Saved on this device”, “Waiting to send”, “Request received”, or “Two changes need review”.
   - No exact coordinates, account IDs or protocol details on the home screen.

2. **Needs attention**
   - At most three highest-priority items initially; an explicit “See all”.
   - Each item has a plain-language heading, why now, due/useful window if known, evidence status, and a next action.
   - “Unknown” or “needs confirmation” must remain visible; do not manufacture a red/amber/green risk score when evidence is insufficient.
   - Separate real deadlines from modelled urgency.

3. **Today's work**
   - Task, responsible person/provider, timing, current lifecycle state, and what is blocking completion.
   - State text and icon; never color alone.
   - “Scheduled” must not be presented as “completed”.

4. **Useful conditions**
   - Only show available fields that matter to the current user: water availability, weather forecast, equipment state, crop observation, or stock.
   - Label each with source and as-of time. Prefer two useful observations over a wall of tiles.
   - If stale, conflicting or unknown, show that state and the next reasonable action.

5. **Quick actions**
   - “Record a problem”, “Add an observation”, “Request help”, “View work”.
   - Provide visible text labels, accessible names, and generous touch targets.
   - Do not require a map pin drop for every record; let the user choose a known site/asset or defer location resolution.

6. **Navigation hypothesis**
   - Today · Land · Work · Services · Sales
   - Role-specific navigation may hide irrelevant destinations but must not fork canonical data or make important records inaccessible.

### Empty/offline/loading states

- No attention items: “Nothing needs your attention right now” and a simple route to record a new issue.
- Offline: keep the screen functional; display the last sync time and the local-only/pending count.
- No imported farm/field data: “Add a field or continue without a map”; never imply spatial coverage.
- Slow operation: show a useful shell and last-synced data where safe, not an indefinite blocking spinner.

## Screen 2 — Record an irrigation problem

### Steps

1. **Where?** Pick an existing field/asset, select a saved site reference, or mark location as unknown. Precise GPS is optional and purpose-bound.
2. **What happened?** Choose simple labels such as “Pump not running”, “Leak visible”, “Low/no water”, “Pipe damaged”, or “Something else”. These labels describe reports, not diagnoses.
3. **What did you observe?** Optional short text, voice note, image, or structured measurement. Explain any data cost and allow capture without attachments.
4. **How soon does this need attention?** Let the farmer state the useful window or urgency. Do not silently substitute a model-generated deadline for the farmer's own assessment.
5. **Review and save.** Show exactly what is being saved/shared, with whom, and through which channel. Record local receipt before network submission.

### Success state

Show the canonical “Saved on this device” message when offline. A unique request reference should be visible and copyable. Do not claim the service provider has received it until an acknowledgement exists.

### Error and recovery

- Missing location: continue with an unresolved site reference; do not invent coordinates.
- Missing network: retain local record and show how to request human help.
- Permission denied: explain the data needed and provide a minimal-disclosure alternative.
- Upload failure: preserve user input and offer retry; no duplicate request on retry.

## Screen 3 — Recommendation and evidence

### Layout

1. **Plain-language conclusion:** action, defer, ask a person, or “not enough information”.
2. **Why now:** relevant window or consequence, including whether the window was supplied by the farmer or calculated by a model.
3. **What we know:** source, time, evidence kind (observed, reported, measured, forecast, modelled), units and freshness.
4. **What is uncertain:** missing input, contradictory observation, coverage gap or assumption.
5. **Alternatives:** at least one non-automated option when a consequential recommendation is shown; include “do nothing/defer” if it is a reasonable choice.
6. **Resources and trade-offs:** show costs, water, parts, time or production estimates only when the data and units support them.
7. **Action controls:** “Request help”, “Record new observation”, “Ask an adviser”, “Defer”, “Dismiss”, and “Correct information” as appropriate.

### Semantic safeguards

- A forecast is not an observation.
- Model confidence is not evidence confidence.
- Multiple feeds derived from the same source are not necessarily independent corroboration.
- A symptom is not a diagnosis.
- A recommendation is not permission to purchase, dispatch, pay or physically actuate equipment.
- An unknown state must not display as zero or “safe”.
- Source digest shape alone does not authenticate source origin.

## Screen 4 — Service options and quote review

Present offers as comparable cards on narrow screens and a table on wide screens. Comparison must not hide non-price trade-offs or rank paid placements as recommendations.

Every option displays:
- Provider identity/role as authorized, with verification scope described precisely.
- Work scope and exclusions.
- Price, currency, unit, labor, parts, travel/mobilization fees, taxes where applicable, and what remains unknown.
- Validity/expiry and expected timing; distinguish promised, modelled and unknown travel time.
- Availability state and last-confirmed time.
- Cancellation, access, payment and safety terms relevant to that job.
- Source of each material claim and dispute/contact route.

### Accept/decline behavior

- Incomplete, expired, wrong-site, wrong-service or withdrawn-capacity quote cannot be accepted as a confirmed booking.
- Before acceptance, summarize the full amount and terms in plain language.
- Acceptance is an explicit user act followed by a confirmed receipt; tapping the card is not acceptance.
- A booking does not automatically authorize a payment.
- When no provider exists, preserve the request and offer another channel without coercing app registration.

## Screen 5 — Work timeline and closeout

Use a chronological list with one visible status per event and distinct actors/receipts:

```
Request recorded
 -> request received (only after acknowledgement)
 -> complete quote received
 -> quote accepted / declined
 -> booking confirmed / unavailable / cancelled
 -> arrival recorded (observed vs reported)
 -> provider reports completion
 -> farmer accepts OR disputes / requests rework
 -> follow-up or correction, if any
```

Every row should show timestamp with relevant source/clock semantics, actor role, event kind, sync state, and a way to inspect the receipt. Corrections append a new event or correction link; do not silently rewrite prior agreements.

When the provider says work is complete, the farmer sees:
“**The provider reports the work is complete.** Please review the result. You can accept it, report a problem, or request rework.”

After acceptance, show a receipt with accepted scope/quantity, date, evidence, and separately indicated payment state if applicable.

For a dispute, do not assign guilt or automatically penalize either party. Preserve the record, allow optional evidence, and show the escalation path.

## Responsive behavior

- **Narrow/mobile:** single-column task flow; important actions reachable without precision gestures; maps open as a contextual sheet or dedicated view.
- **Tablet:** split view for list + map/evidence when useful; selected item state remains readable.
- **Desktop/service operator:** request inbox, route/parts/capacity panels and detail timeline; no assumption of live route data if unvalidated.
- **All sizes:** accessible keyboard path, visible focus, labels, errors connected to inputs, zoom/reflow, text not clipped, status message announced, state not conveyed only through color.

## Shared UI component contract

- `SyncState`: local-only, pending, syncing, confirmed, conflict, rejected, stale.
- `EvidenceBadge`: observed, reported, measured, forecast, modelled, conflicting, unknown; always with human-readable explanation and source/time where material.
- `CommitmentSummary`: scope, actors, cost terms, valid-until, confirmation state.
- `WorkTimeline`: ordered immutable events and explicit correction links.
- `DecisionCard`: recommendation/abstention, rationale, assumptions, alternatives, reversible actions.
- `PrivacyScope`: data category, recipient, purpose, precision, revocation status.
- `OutcomeReceipt`: provider-reported completion separated from farmer acceptance/dispute.

These are conceptual contracts, not claims that named reusable components already exist in the codebase.

## Usability test script

Ask representative users to complete these tasks without telling them where to tap:

1. Record an irrigation issue with location unavailable.
2. Identify whether the request reached a service provider.
3. Explain whether a recommendation relies on a current measurement or a forecast.
4. Compare two quotes where one is cheaper but excludes travel or parts.
5. Explain whether work is complete, accepted, and paid.
6. Resolve a conflict after the app has been offline.
7. Choose phone/cooperative help without creating a new account.
8. Revoke location sharing and explain the known limits of prior disclosures.

Record task success, time, errors, help requests and comprehension. Do not equate a fast tap with understanding. Run locally translated variants with speakers of the selected language; this English spec is not localization evidence.

## Release gates

- Critical workflows remain usable without a network after the relevant local data is available.
- 100% of state transitions have an explicit outcome; no silent state or denominator dropping.
- 100% of material claims in the interface have a declared source/evidence state or explicit unknown.
- Zero duplicate requests/bookings/payment effects after retry/replay.
- Zero booking from an expired/incomplete/mismatched quote.
- Zero “farmer accepted” label before a farmer decision record.
- WCAG 2.2 AA tested across supported surfaces; target at least 44 x 44 CSS px for primary touch controls as an additional usability target where feasible.
- Usability and field outcome results are documented separately from structural/synthetic checks.

## Research references

- 2026 *Field-Ready HCI* conceptual model for low-resource and smallholder contexts: https://doi.org/10.3390/app16146985
- June 2026 *Journal of Rural Studies* work on non-use, socio-technical fit and offline dimensions: https://doi.org/10.1016/j.jrurstud.2026.104210
- W3C WCAG 2.2: https://www.w3.org/TR/WCAG22/

Research is motivation for the prototype, not evidence that these exact screens fit local users. Lock the interface only after farmers, extension staff and service providers can complete and explain the critical tasks.
