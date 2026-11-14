# FSRE as-of portfolio composer v0.1

**Status:** Deterministic read-only composition of point-in-time evidence admissibility and one frozen recovery portfolio. Synthetic fixtures only. Not a source authenticator, scheduler, actuator or field-outcome validator.

## Purpose

The original `fsre-assurance-composer` consumes the earlier decision-time replay contract. This dedicated composition path consumes the stronger `fsre-asof-evidence` replay and the current `fsre-portfolio-reference`.

It composes four bindings:
1. service scenario;
2. immutable evidence snapshot ID;
3. exact decision cutoff;
4. clock basis.

It then verifies every owner/evidence-reference pair used by the submitted portfolio against the as-of report. A binding counts only when the evidence row is admissible at the cutoff **and** its enclosing claim has met its required evidence count. Duplicate owner/reference bindings across claim IDs are rejected as ambiguous.

The portfolio reference accepts optional `decision_at` and `clock_basis_id` metadata as a pair. Existing one-interval use remains unchanged when both are omitted; this composer requires both and an exact match with the as-of replay.

## Dispositions

- `ASOF_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION`: exact scenario/snapshot/cutoff/clock match, all portfolio references admissible, all referenced claims supported, and portfolio feasible.
- `SCOPE_MISMATCH`: one or more of scenario, evidence snapshot, decision cutoff or clock basis differs or is absent.
- `EVIDENCE_NOT_ESTIMABLE`: a portfolio evidence reference is missing, non-admissible, or belongs to a claim whose required evidence set is incomplete.
- `PORTFOLIO_NOT_ESTIMABLE`: evidence binds, but the portfolio reference lacks required inputs.
- `PORTFOLIO_NOT_FEASIBLE`: evidence binds, but known capacity, common-cause or service-floor constraints fail.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-asof-portfolio-composer/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-asof-portfolio-composer/Cargo.toml -- tools/fsre-asof-portfolio-composer/fixtures/synthetic-asof.psv tools/fsre-asof-portfolio-composer/fixtures/synthetic-portfolio.psv`

The output proves software-level linking to the declared inputs only. It does not establish source authenticity, claim completeness beyond referenced inputs, truth of the owner assertions, full-horizon feasibility or actual service restoration. No dispatch, rationing, procurement or actuation occurs.
