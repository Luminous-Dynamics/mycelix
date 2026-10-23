# LIFE-DISCLOSURE-BUDGET-001

## Question

Can multiple independently operating verifiers collectively exceed a subject's declared optional-disclosure budget even when each local decision is within its own stale view of that budget?

This complements:
- #5405: information learned from a sequence of disclosures, conditional on associating that sequence with a subject;
- #5401/#5402/#5439: presentation and recipient linkability;
- #5458/#5459: population mismatch and uncertainty-aware admission;
- #5460/#5462: observer-visible trace distinguishability;
- #5447: protocol/profile conformance and implementation comparisons.

## Paradigm

Test **global conservation of disclosure authority**, not only per-query correctness:

`sum(cost of accepted unique optional disclosures for subject and policy epoch) <= global budget`

The negative control gives each of four verifier replicas a stale full budget of 10 units. If all accept a cost-3 disclosure independently, each local check passes but aggregate spend becomes 12, exceeding the global budget by 2.

Two safer comparison profiles are modeled:
- **Serialized shared ledger:** one shared state transition checks epoch, budget and request identity; repeated identical requests are idempotent.
- **Bounded escrow:** the global budget is partitioned into local verifier allocations whose sum cannot exceed the global limit. Offline authorization remains available within a verifier's own allocation; a verifier cannot borrow another verifier's allocation during a partition.

These are abstract cost units for evaluating budget enforcement. They are not differential-privacy epsilon/delta, not calibrated privacy loss, and not a claim that every disclosure system has a meaningful additive privacy budget.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-disclosure-budget-conservation/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-disclosure-budget-conservation/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-disclosure-budget-conservation/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-disclosure-budget-conservation/Cargo.toml
```

The benchmark emits deterministic CSV and a workflow receipt bound to the exact PR-head commit. It records accepted/denied optional requests, duplicates, conflicting key reuse, stale-epoch rejection, aggregate spend, overspend, and acceptance availability. Safety and availability are independent columns; no composite score is calculated.

## Frozen initial fixtures

- Global abstract budget: 10 units, policy epoch 1.
- Four verifier replicas.
- Stale-replica control: four unique requests, each cost 3.
- Serialized baseline: same requests against one shared ledger.
- Escrow baseline: local allocations [3, 3, 2, 2], with each verifier's request cost equal to its allocation.
- Invalid escrow control: allocations [4, 4, 4, 4], whose sum exceeds the global limit.
- Stale-epoch control: requests from epoch 1 against active epoch 2.

Additional tests enumerate every permutation of a mixed-cost request batch, exercise duplicate acknowledgement replay and conflicting idempotency-key reuse, and reject zero-cost/stale-epoch/missing-allocation cases.

The required task decision is explicitly separate from optional disclosure. Denying optional disclosure does not revoke or rewrite the required task result.

## Distributed-systems boundary

The serialized ledger is a semantic baseline, not a production implementation of a consensus protocol. The escrow model tests a bounded offline-authorization pattern in a deterministic state machine; it does not establish how leases are issued, revoked, reconciled or made tamper-resistant in a live federation.

Before integrating with Holochain or Mycelix runtime code, an independent design must specify authority, policy epochs, lease expiry, durable idempotency, revocation/currentness and unresolved-conflict semantics. A DHT observation alone must not be described as a globally atomic reservation. Under missing or stale required evidence, optional disclosure should be denied or marked inconclusive rather than silently expanded.

## Industry comparison

For statistical queries implemented with a formally specified differential-privacy mechanism, compare against published privacy-filter/odometer accounting rather than mapping these abstract integer costs to DP parameters. Rogers et al. distinguish adaptive privacy filters (stop before a target budget is exceeded) from odometers (track realized privacy loss); the concepts motivate accounting but do not qualify this toy budget.

For credential profiles, measure this state-machine behavior alongside—but separately from—SD-JWT/BBS/OpenID4VP conformance, cryptographic linkability, task utility and measured cost in #5447.

## Research anchors

- Rogers, Roth, Ullman, Vadhan, *Privacy Odometers and Filters: Pay-as-you-Go Composition*: https://proceedings.neurips.cc/paper/6170-privacy-odometers-and-filters-pay-as-you-go-composition.pdf
- Holochain validation documentation: deterministic/pure validation, addressable dependencies and indeterminate/retry behavior when required dependencies are unavailable: https://developer.holochain.org/build/validation/
- Mycelix request-bound metadata/existence projection: #2268.
- Mycelix evaluator/profile authority binding: #2269.
- Cross-project OPSEC composition boundary: #2746.

## Claim ceiling

This is a synthetic, deterministic state-machine benchmark. It is not a production distributed ledger, formal consensus proof, differential-privacy mechanism, cryptographic credential implementation, or SOTA-superiority result. A passing exact-head workflow qualifies only the frozen model and its tested invariants.
