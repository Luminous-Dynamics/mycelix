# FSRE assurance composer v0.1

**Status:** Deterministic, read-only composition of decision-time evidence replay and one-interval portfolio feasibility. Synthetic fixture only. Not a scheduler, operational authority, source authenticator or field outcome validator.

## Purpose

The composer links two independently testable components:
1. decision-time evidence replay selects the latest revisions observed and recorded by the decision cut-off and checks valid-time/revocation state;
2. portfolio reference evaluates the submitted allocation against option capacity, per-cohort service floors, shared resources and common-cause states.

The composer then verifies that the scenario and evidence snapshot IDs match and that every source-owner/evidence-reference pair used by the portfolio input appears in the decision-time replay as KNOWN_VALID. A missing or nonvalid binding blocks the combined disposition.

## Input and output

The CLI accepts two pipe-separated files: decision evidence and portfolio plan. The synthetic decision fixture is generated from each portfolio record's source owner and evidence reference. The output reports component dispositions, matching scope, matched/unmatched/nonvalid evidence-reference counts and a non-authorisation statement.

The combined positive state is deliberately named SUBMITTED_EVIDENCE_LINKED_AND_PORTFOLIO_FEASIBLE_FOR_DECLARED_DECISION. It does not establish that the submitted references are a complete claim set, that owners are authentic, that assertions are true, or that a service outcome occurred. It is a software-level composition result only.

## Run

`cargo test --locked --offline --manifest-path tools/fsre-assurance-composer/Cargo.toml`

`cargo run --quiet --locked --offline --manifest-path tools/fsre-assurance-composer/Cargo.toml -- tools/fsre-assurance-composer/fixtures/synthetic-decision.psv tools/fsre-assurance-composer/fixtures/synthetic-portfolio.psv`

No dispatch, rationing, switching, procurement or crew assignment is performed.
