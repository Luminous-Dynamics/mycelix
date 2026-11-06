# Accounting Policy Projection Integrity Auditor v1.0.0 — S0

This package implements [FIN-PROJECTION-AUDIT-001 / #5891](https://github.com/Luminous-Dynamics/mycelix/issues/5891). It independently verifies that a policy projection is structurally bound to the exact event graph and that every event decision, source assertion and journal line remains internally consistent.

It does **not** determine whether the chosen accounting policy is correct, certify GAAP/IFRS treatment, authorize ledger posting, or replace professional review.

## Paradigm: policy projection as a proof-carrying transformation

A valid projection must preserve a closed chain:

`event graph bytes → policy/profile digest → projector digest → event decision → source assertion closure → journal line → balanced entry`

A correct JSON schema is necessary but insufficient. A balanced journal is also insufficient if its evidence belongs to another event or if the projection is still blocked for review.

## Inputs and outputs

The auditor consumes:
- `../fixtures/economic_event_graph_candidates_v1.0.0.json`: the exact graph bytes and graph case;
- `../fixtures/accounting_policy_projection_review_controls_v1.0.0.json`: two synthetic review-only projections over the same graph digest;
- `../economic_event_graph_v1.0.0.schema.json`;
- `../accounting_policy_projection_bundle_v1.0.0.schema.json`;
- `../accounting_policy_projection_audit_report_v1.0.0.schema.json`.

It emits one deterministic report containing SHA-256 digests of the raw graph and projection-bundle bytes, one result per projection, status (`pass`/`reject`), independent check booleans and explicit failure codes.

## Invariants enforced

1. **Exact graph binding.** Projection graph case ID and graph SHA-256 must match the exact candidate graph bytes.
2. **Policy/projector binding.** Policy profile and projector IDs/versions must carry canonical lowercase SHA-256 values. The verifier validates digest syntax and separation; it does not fetch the underlying policy/projector artifacts or prove their bytes match those digests.
3. **Total event disposition.** Exactly one policy decision must exist for every event in the graph. Decisions cannot be duplicated or orphaned.
4. **Evidence closure.** Every decision's source assertions must be attached to that event and available at the graph's valid-time/knowledge-time frontier. Future, integrity-failed, invalidated, disputed or superseded assertions cannot qualify as available support.
5. **Explicit assumptions.** Projection assumptions are typed objects with evidence refs, review state and decision-critical flag. Every decision assumption ref must resolve; orphan and duplicate assumptions are rejected. A decision-critical assumption must be source-supported or reviewer-approved with a non-empty review-record reference.
6. **Journal-to-decision binding.** Each journal line must reference exactly one policy decision, the decision's event and only assertions in that decision's evidence closure.
7. **Amount and currency agreement.** Every recognized decision must have an amount, currency and period. The sum of its debit lines and credit lines must each equal the declared recognized amount in its declared currency. Extra currencies or wrong period refs are rejected.
8. **Complete source carry-through.** All source assertions declared by a recognized decision must appear on at least one of its journal lines.
9. **Independent balance.** Each journal entry must balance separately by currency, entity and period. Entries cannot offset one another; debits in USD cannot cancel credits in EUR.
10. **No blocked posting.** Only `complete_for_review` may contain journal entries. A complete projection cannot contain a `needs_review` or `indeterminate` event decision or a blocking unresolved item. Non-recognized decisions cannot be journaled.
11. **No orphaned IDs.** Duplicate journal-entry, line, assumption and unresolved-item IDs are rejected; unresolved refs must resolve to graph events/assertions.

## Synthetic controls

The bundled two projections:
- point to the same exact graph SHA-256;
- have different synthetic policy-profile and projector digests;
- both remain `needs_review`;
- both carry one blocking service-evidence review item;
- contain no journal entries and no assumptions.

They should pass the structural audit while remaining non-posting.

The Rust unit tests also construct a synthetic `complete_for_review` projection with a balanced journal and a clearly synthetic rule identifier. This exists only to exercise amount, currency, period and source-binding checks; it is not a GAAP/IFRS accounting example or a model-performance result.

## Local validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin audit_accounting_policy_projection_s0
cargo run --locked --bin audit_accounting_policy_projection_s0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Optional arguments:
- `--data-dir PATH`: economic-event package directory;
- `--output PATH`: write report JSON to a file;
- `--help`: print usage.

The validator has no network or ledger side effect.

## Static authoring evidence

Fetched all six relevant artifacts/source files from GitHub. A separate JS simulation of the graph and projection fixture found:
- eight graph cases in the S0 corpus; the projection example targets one graph case containing one event;
- both review-only projections pass all structural checks;
- exact graph digest matches the fetched raw graph bytes;
- a structurally balanced synthetic complete projection passes;
- a balanced but wrong 126,000 journal against a 125,000 decision is rejected;
- a needs-review projection with a balanced journal is rejected;
- a decision-critical unsupported assumption is rejected;
- a supported assumption with available source evidence passes;
- wrong graph digest is rejected;
- every failure code emitted by the Rust source is present in the report schema;
- no source/event/decision/journal or unresolved references are dangling in the bundled review controls.

These are static authoring simulations, not Rust execution or accounting-policy adjudication.

Twenty-nine Rust tests are authored but **not executed**. No Cargo compile/test, rustfmt, Clippy, independent Draft 2020-12 runtime validation or CI pass is claimed.

## Claim boundary and benchmark progression

S0 demonstrates evaluator contract behavior on synthetic fixtures. It does not prove the policy mapping is correct or that a real accounting agent can perform a close.

S1 should contain at least 100 dual-reviewed, expert-adjudicated policy-projection cases. S2 should contain at least 250 cases plus a hidden holdout across policies, currencies, charts of accounts, entity structures, source systems, late-arriving evidence and corrections.

Hard targets (not achieved): zero wrong graph/policy/projector digest acceptance; zero orphan event decisions; 100% source assertion closure; zero journal imbalance; zero recognized-amount mismatch; zero posting from a blocked projection; 100% deterministic replay of frozen graph/policy/projector inputs.

The canonical benchmark run manifest and receipt remain in #5799. This package creates no second run-manifest schema.