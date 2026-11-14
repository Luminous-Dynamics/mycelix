# FSRE time-expanded portfolio composition reference v0.1

**Status:** Read-only deterministic reference composer. Synthetic fixtures only. It is not a multi-period optimiser, scheduler, inventory ledger, physical simulator, evidence authenticator or operational controller.

## Gap addressed

The existing portfolio reference qualifies a proposed allocation for one scenario/interval. The existing resource stock-flow reference checks conservation for one resource across a contiguous horizon. This crate composes those two owner-bounded references over a common time axis.

It checks that:
- every interval has a portfolio evaluation and an explicit service floor for every cohort in the study boundary;
- every portfolio record is linked to owner/reference-bound evidence admissible at the exact declared snapshot, decision cutoff, scenario and clock basis;
- each stock resource has a source-owned stock-flow series with the same scenario, clock basis, interval IDs and exact time boundaries;
- the portfolio's resource-use quantity equals the stock-flow's recorded consumption in every interval;
- the proposed per-interval resource capacity does not exceed opening inventory minus all losses recorded in that interval (a conservative rule: replenishment within an interval is not assumed available until the next interval, and every recorded loss is treated as occurring before use);
- individual cohort floors are preserved in every interval, so surplus in one cohort or period cannot hide a deficit in another;
- each service/cohort pair retains the same exact unit across the horizon; no cross-interval unit conversion is inferred;
- unknown or incomplete evidence remains NOT_ESTIMABLE, and reconciliation errors cannot be promoted to a feasible horizon claim.

## Input format

The plan uses the same 13-column PSV rows as the existing fsre-portfolio-reference crate:

    record_type|record_id|option_id|service_id|cohort_id|scenario_id|interval_id|subject_id|quantity|unit|status|source_owner|evidence_ref

Metadata is:
- portfolio_id, scenario_id, snapshot_id;
- decision_at (integer seconds) and clock_basis_id;
- stock_resource_ids (comma-separated resource IDs);
- one repeated interval row per horizon interval, in the form ID|START_SECONDS|END_SECONDS.

An as-of evidence file uses the existing fsre-asof-evidence format. It must declare the same scenario, evidence snapshot ID, decision_at and UTC_UNIX_SECONDS_V1 clock basis as the plan. Each stock file uses the existing fsre-resource-stock-flow format. Supply exactly one file for each declared stock resource. All time and quantity checks reuse the existing crate validators; this crate does not reimplement their arithmetic.

Run:

    cargo test --locked --offline --manifest-path tools/fsre-time-expanded-portfolio-composer/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-time-expanded-portfolio-composer/Cargo.toml -- tools/fsre-time-expanded-portfolio-composer/fixtures/synthetic-plan.psv tools/fsre-time-expanded-portfolio-composer/fixtures/synthetic-asof-evidence.psv tools/fsre-time-expanded-portfolio-composer/fixtures/synthetic-diesel.psv

## Dispositions

- PORTFOLIO_FEASIBLE_FOR_DECLARED_HORIZON: each interval is feasible and every stock-flow/resource join reconciles.
- NOT_FEASIBLE: at least one known option/resource/service-floor constraint fails.
- NOT_RECONCILED: the plan and stock-flow records disagree on scenario, time, capacity or consumption.
- NOT_ESTIMABLE: a required input is unknown or cohort floor coverage is incomplete. This takes precedence over other aggregate dispositions; known interval results remain visible.

The output retains per-interval portfolio and as-of dispositions, owner/reference evidence match counts, every cohort floor result, owner/evidence references, stock-flow rows and resource reconciliation rows. It states that service quality, independent as-of replay and source authenticity are not established by this composer.

## Claim boundary and limitations

- Quantities use exact non-negative integers and exact unit-string equality; no implicit conversions are performed. The service/cohort measurement unit must also remain identical across all intervals.
- Inventory capacity is limited to opening stock less all recorded losses in that interval. The composer conservatively treats losses as occurring before use and does not credit same-interval replenishment because the stock-flow input has no within-interval event ordering. This is an intentionally strict lower bound, not a reconstruction of actual delivery timing.
- Crew-hours and other renewable per-interval constraints remain in the portfolio reference; only the declared stock resources are joined to stock-flow series.
- The composer does not establish that receipts are authentic, current, owner-authorised or independent; it only preserves their references and composes the current validators' declared results.
- It does not independently replay the as-known-at-cutoff evidence beyond the existing as-of verifier's declared replay semantics, evaluate water quality, measure achieved service, infer causal effects, calculate probabilities, optimise allocations, schedule crews, issue orders or authorise dispatch.
- A future version should add independently authored temporal challenge sets and separately evaluated outcome trajectories before any operational comparison.

This is a deterministic composition prototype, not field validation or a claim of superiority over restoration optimisation methods.
