# GCPN Warning-to-Action Benchmark v0.1.1 Addendum

**Status:** versioned extension; original v0.1 fixture remains frozen.  
**Parent:** PR #5612; hardening branch PR to follow.  
**Primary change:** financing is explicitly conditional on the protocol's `finance_required` field.

## Why the change was necessary

The first evaluator treated every action start as requiring a separate finance-release event. That is too restrictive: some pre-agreed actions may use pre-positioned resources, standing capacity, in-kind supplies, or existing operational budgets. A benchmark that rejects these without a release record would mistake a particular financing workflow for a universal property of anticipatory action.

v0.1.1 therefore makes financing requirements explicit in the synthetic protocol record.

## New semantics

- `finance_required=true`: an action start requires a released funding record. That release must be tied to an activated authorized decision, pre-arranged financing and financing authority. Action start must follow the finance-release timestamp.
- `finance_required=false`: an action may start after the authorized decision and action authorization without a separate finance-release record. A funding release in this state is an unapproved release and fails a hard gate.
- Timing is reported separately:
  - decision-to-finance for funded actions;
  - finance-to-action for funded action starts;
  - decision-to-action for explicitly unfunded action starts.

This records process differences, not a recommendation that all actions should or should not be funded in a particular way. Local protocol owners define the actual financing model and competent authorities govern releases.

## Additional positive control

`authorized-unfunded-action` is a second positive control. It has:
- an authorized warning and activation decision;
- `finance_required=false`;
- no finance-release event;
- an authorized action start after the decision;
- a separately recorded completion;
- 180 unique fictional people served from an intended cohort of 200.

This must pass all hard conformance gates. Its timing is reported as decision-to-action, not finance-to-action.

## Frozen v0.1.1 fixture oracle

| Counter | Expected |
|---|---:|
| Cases | 11 |
| Missing lineage | 0 |
| Unauthorized decisions | 1 |
| Unauthorized activations | 1 |
| Unapproved fund releases | 1 |
| Unauthorized action starts | 1 |
| False completions | 1 |
| Population overclaims | 1 |
| Service accounting mismatches | 0 |
| Grievance suppression | 1 |
| Grievance resolution overcounts | 0 |
| Unsupported impact claims | 1 |
| Chronology violations | 1 |
| Activation decisions | 10 |
| Funding-release records | 8 |
| Action-start assertions | 7 |
| Completion assertions (including one false assertion) | 7 |

The fixture sums 10,200 intended-person slots and 3,970 served-person records over separate synthetic cases. These are regression arithmetic, not a unique population or a coverage KPI.

Expected synthetic latency totals:
- warning-to-decision: 11 observations, 1,030 ticks;
- decision-to-finance: 8 observations, 800 ticks;
- finance-to-action: 6 observations, 800 ticks;
- decision-to-unfunded-action: 1 observation, 20 ticks;
- action-to-completion: 5 validly ordered observations, 2,130 ticks.

All ticks are arbitrary fixture units. The chronology negative control produces one invalid completion ordering and is not included in valid completion latency.

## Reproduction

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-warning-to-action/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-warning-to-action/Cargo.toml
```

The CLI consumes `action-chain-v0.1.1.csv`. v0.1 remains present as a historical frozen test artifact.

## Non-claims

This extension does not define eligibility, funding rules, public-warning triggers or operational action policy. GCPN does not release funds or authorize action. It evaluates the provenance and internal consistency of synthetic records supplied by competent, external authorities.
