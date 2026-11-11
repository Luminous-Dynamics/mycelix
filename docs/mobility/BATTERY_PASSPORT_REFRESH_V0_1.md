# Battery Passport event-triggered dynamic-data refresh planner v0.1

## Purpose and ownership

This is the executable first slice of [MOBILITY-BATTERY-PASSPORT-REFRESH-001 #6076](https://github.com/Luminous-Dynamics/mycelix/issues/6076).

It evaluates lifecycle and operational events and emits an evidence-carrying work ledger for dynamic Battery Passport data. It does not update the passport, publish a value, sign or register a DPP, or act as the responsible economic operator.

Composition:
- [Battery Passport projection compiler #6062](https://github.com/Luminous-Dynamics/mycelix/issues/6062) / [draft PR #6063](https://github.com/Luminous-Dynamics/mycelix/pull/6063) owns category/date applicability, disclosure, source evidence and the regulatory projection.
- Mobility Commons #3709 owns battery, pack, component, configuration and lifecycle identity/lineage.
- Symthaea owns advisory observations, SOH/capability estimates and qualification. Its estimates do not self-authorize passport facts.
- Symtropy owns physical service, inspection, repair, testing and lifecycle evidence.
- The responsible economic operator remains accountable for accuracy, completeness and up-to-date passport information. Delegation requires written authorization under the applicable regulatory role.

The refresh planner accepts the **already resolved applicable dynamic data-point IDs** from the versioned registry/projection compiler. It does not independently decide category applicability or disclosure.

## Regulatory update policy: event-triggered, proportionate, not a fixed timer

Sources consulted 11 November 2026:
- European Commission, *EU Digital Product Passport FAQ for Batteries*, Section 8 (dynamic data): https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/eu-digital-product-passport-faq-batteries_en
- Regulation (EU) 2023/1542, Article 77 and Annex XIII, point 4: https://eur-lex.europa.eu/eli/reg/2023/1542/2026-08-13/eng
- Commission Battery Passport Guidance v3.2, 19 October 2026, data points by category: https://single-market-economy.ec.europa.eu/document/download/cd1e5e6c-4a4a-4b99-995a-49eb6916187e_en?filename=Digital+Batteries+Passport+-+data+point+by+category.pdf

The Commission FAQ clarifies:
- Dynamic information must be updated at minimum when a battery's status changes **or is being considered** (for example, considering remanufacture, repurposing or recycling).
- More frequent updates may be necessary for certain dynamic data used for SOH monitoring, where technically feasible and proportionate.
- A universal weekly schedule is not required merely because a field is dynamic; the FAQ says weekly updates need not occur when SOH does not materially change week to week.
- Real-time communication is not generally required. Updating an offline e-bike passport when it is brought in for servicing can be appropriate; no precise cadence is prescribed.
- Operational responsibility remains with the economic operator that placed the battery on the market, even where practical data collection/update tasks are delegated in writing.

This planner therefore has **no hard-coded weekly/daily legal TTL**. Periodic usage-data capture is admitted only with an explicit, versioned operator policy and a declared source window. The planner never turns a locally chosen policy interval into a legal requirement.

## Dynamic point families

The planner consumes points from the v3.2 dynamic family 51–71, but uses purpose-specific groups:
- **51–60**: performance/durability values; refresh/review at placing on market and when battery status changes or is being considered.
- **61–66**: SOCE / remaining capacity / remaining power / remaining efficiency / self-discharge / ohmic resistance; applicability is category/BMS dependent and belongs to the projection compiler. Additional SOH-related refreshes require a declared monitoring purpose, technical-feasibility decision and materiality evidence.
- **67**: individual battery status.
- **68–71**: use-related charging/discharging cycles, negative events, operating-environment histograms and SOC histograms; review after first use and update from declared periodic use-data capture or appropriate service opportunities.

The planner only triggers the intersection of an event's point group and the category/date-resolved `applicable_dynamic_data_points`. A missing applicability resolution is not silently interpreted as not-applicable.

## Event contract

Every event binds:
- event ID and strictly increasing source sequence;
- exact battery, passport and pack-configuration identity;
- event type, occurred-at time and received-at time;
- actor and authority state;
- source class, evidence reference and digest;
- event-specific status/purpose/policy/window data where required.

Accepted authority modes:
- `RESPONSIBLE_OPERATOR_DIRECT`: event actor must be the responsible economic operator;
- `WRITTEN_DELEGATE`: the event must bind an authorization reference, digest and the explicit `DYNAMIC_PASSPORT_REFRESH` scope.

Source classes in v0.1 include synthetic known-truth fixtures, operator event records, measured observations and lifecycle documents. Synthetic sources exercise the software contract only; they are not physical or legal evidence.

Times are normalized to UTC RFC3339 with a `Z` suffix. Event sequence must increase, and event time may not regress. Duplicate IDs, wrong asset/configuration identity or invalid ordering are conflicting events, not accepted updates.

## v0.1 trigger behavior

| Event | Trigger behavior |
|---|---|
| `STATUS_CHANGE_CONSIDERED` | Creates `UPDATE_REQUIRED` for applicable points 51–67 before the status decision is completed. |
| `STATUS_CHANGED` | Creates `UPDATE_REQUIRED` for applicable points 51–67; repurposed/remanufactured status also requests a new-passport predecessor link. |
| `FIRST_USE_CONFIRMED` | Creates `REVIEW_REQUIRED` for applicable points 68–71. It activates use-data applicability without inventing a prior usage history. |
| `PERIODIC_USAGE_CAPTURE` | Creates `UPDATE_REQUIRED` for applicable points 68–71 only when the exact operator-policy reference and source window are present. |
| `SERVICE_VISIT` | Creates `REVIEW_REQUIRED` for applicable points 68–71. For offline batteries it records a service-reconciliation opportunity; a service visit alone does not assert that every field changed. |
| `SOH_MONITORING_REVIEW` | Requires a declared purpose and technical-feasibility decision. A material change creates `UPDATE_REQUIRED` for applicable points 61–66; no material change records a review without fabricating a new value; infeasibility defers the task. |
| `NO_MATERIAL_CHANGE_ASSESSED` | Closes pending SOH points 61–66 only with matching purpose, technical feasibility and a supported no-change result. |
| `CONNECTIVITY_UNAVAILABLE` | Records `OFFLINE`; does not clear, satisfy or erase outstanding updates. |
| `WASTE_PREPARATION_FOR_REUSE_OR_REPURPOSE` | Creates performance/status update work and a new-passport predecessor-link requirement when the resulting status is re-used or repurposed. |

Simple repair/service does not by itself create a new passport. A qualifying repurposing/remanufacturing transition or preparation for reuse/repurposing after waste triggers predecessor-link review; the planner does not create the passport.

## Dispositions and ledger semantics

Per event:
- `UPDATE_REQUIRED`
- `REVIEW_REQUIRED`
- `NO_CHANGE_RECORDED`
- `CONNECTIVITY_BLOCKED`
- `BLOCKED_UNQUALIFIED_EVENT`
- `CONFLICTING_EVENT`
- `DEFERRED_WAITING_FOR_SOURCE`

Case state:
- `OPERATOR_ACTION_REQUIRED`: one or more pending dynamic points or a new-passport link request.
- `NEEDS_RECONCILIATION`: invalid/conflicting source, unsupported authority or unresolved source feasibility.
- `NO_TRIGGER_RECORDED`: no applicable refresh trigger was received. This is not a general “passport is current” claim.
- `REVIEW_RECORDED`: a valid no-material-change review was recorded and there is no remaining work.
- `REJECTED_INPUT`: malformed case identity, digest, applicability set or policy.

Connectivity/delivery state is separate from update obligation. A battery can be offline while update work remains due. A service visit can create a collection opportunity but does not automatically satisfy the work.

Every report sets `registry_submission_authorized = false`. It is a work ledger, not an instruction to publish.

## Frozen synthetic corpus

Twelve scenarios exercise:
1. status-change consideration triggers before the transition;
2. completed remanufacture requests new passport linkage;
3. offline service visit creates usage-data review without creating a new passport;
4. first use activates points 68–71 without fabricated history;
5. connectivity loss preserves already-due status/performance work;
6. technically feasible SOH review with no material change records no-change and does not create a fake observation;
7. no universal weekly refresh is invented in the absence of a due trigger;
8. periodic usage capture is allowed only under a declared policy and source window;
9. delegated actor without written scope is blocked;
10. wrong pack-configuration identity is conflicting;
11. duplicate/out-of-order event sequence is conflicting;
12. waste preparation for reuse/re-purposing requests predecessor-linked new passport review.

All batteries, identities, policy references, event sources, times and digests are synthetic known-truth fixtures. They exercise event routing and reconciliation; they do not establish regulatory compliance, physical battery state or actual operator authority.

## Benchmark strategy

Compare the planner to:
- fixed weekly polling;
- generic TTL-based stale flags;
- static template/manual checklist;
- a human-adjudicated event-trigger workflow.

Metrics:
- missed-trigger rate and false-trigger rate;
- time from considered/actual status event to work-ledger creation;
- event-to-source evidence coverage;
- duplicate/late/conflicting-event quarantine accuracy;
- pending-task age and time to operator reconciliation;
- service-visit capture yield for offline batteries;
- proportion of SOH reviews closed by a justified no-change receipt;
- operator work per battery-month, by connectivity mode and battery category.

Software gates:
- 100% event/battery/passport/configuration binding;
- zero lost status-consideration events in frozen controls;
- zero duplicate/out-of-order events promoted to accepted refreshes;
- zero connectivity failures that clear pending updates;
- zero fabricated weekly/daily legal cadence rules;
- deterministic replay and order-invariance for non-temporal point/case arrays.

Do not claim superiority until an independently adjudicated operator event corpus exists and false-trigger/missed-trigger confidence intervals are reported.

## Claim ceiling

v0.1 is a synthetic event-to-work-ledger prototype. It is not a legal certifier, DPP Registry client, battery sensor, BMS, predictive SOH model or passport publisher. It does not determine whether a material SOH change occurred unless a separate qualified source supplies that result. The responsible economic operator retains legal responsibility.
