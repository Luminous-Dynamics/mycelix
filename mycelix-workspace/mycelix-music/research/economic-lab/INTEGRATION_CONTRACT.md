# Economic Lab → Mycelix Music integration contract

Status: design boundary for a **research-only evaluator**. This document is not a statement that any linked draft PR has merged or passed exact-head qualification.

## 1. Boundary

The Rust Economic Lab answers a narrow question:

> Given one frozen, already-qualified set of listener contributions, eligible plays, cooperative membership, and a selected allocation policy, what exact artist-level allocation follows?

It is not a source of playback truth, legal rights, debt, settlement eligibility, or payment finality. The allocation report is a reproducible calculation artifact, not a ledger posting or authority receipt.

The current Music Plays DNA records a play on the listener's authored source chain and validates consistency with the referenced catalog song terms. That source-chain signature proves who authored the record; by itself it does not prove genuine human playback, qualify an event for royalties, or prevent a listener from generating repeated records. The eligibility boundary must be separately specified and evidenced.

## 2. Required flow

1. **Usage source snapshot.** Select an exact time boundary and a complete source set. Preserve source reference, source instance, observed-through time/high-water mark, and a digest/commitment to the precise set. Unknown or incomplete coverage must remain explicit; do not interpret absence of evidence as zero usage.
2. **Eligibility projection.** Exclude or hold events that have not passed the upstream eligibility policy. The evaluator must not determine eligibility from raw stream counts or trust caller-supplied booleans as authenticated proof. Stable per-run pseudonyms should be used where possible.
3. **Frozen economic inputs.** Bind the accepted play aggregates, listener contributions, unit/denomination, policy ID/version, membership version (if applicable), rounding rule, source root, and code revision to one experiment manifest. Do not mix currencies or treat credits and cash as interchangeable.
4. **Policy evaluation.** Run this crate with the frozen inputs. Verify pool conservation, deterministic tie-breaking, error-free arithmetic, and exact output ordering. Compare alternative policies on the same accepted input set.
5. **Rights resolution.** Keep composition and master-recording rights as separate pools. Map artist-level allocations to the appropriate rights/claim graph only through an explicit, reviewed rights policy. Unmatched, pending, unverified, and disputed shares remain held; the lab's in-memory revision chain is not authenticated rights evidence and not durable history.
6. **Obligation and settlement authority.** Only a reviewed adapter may turn verified allocation outputs into immutable economic obligations. Eligibility evidence, deductions/withholdings, held residuals, settlement attempts, and rail receipts must preserve their own authority and causality. A statement/report or this lab's CSV cannot create or cancel debt.
7. **Independent replay.** Re-run the policy from the frozen source manifest and exact revision. Compare normalized outputs and conservation results. A changed source set, policy version, rights resolution, or code revision must produce a distinguishable result rather than silently replacing history.

## 3. Relationship to existing creator-accounting work

The repository already has a staged set of **open/draft proposals** for creator accounting and settlement. They should be treated as architectural references, not as available merged APIs until verified:

- [#523 — creator-grade ledger and statement audit core](https://github.com/Luminous-Dynamics/mycelix/pull/523): balanced postings, value classes, statement completeness, immutable snapshots, and reconciliation.
- [#525 — creator money lineage and deterministic settlement planning](https://github.com/Luminous-Dynamics/mycelix/pull/525): eligibility/cutoff semantics and deterministic obligation-set batching.
- [#527 — value-safe settlement and threshold execution authority](https://github.com/Luminous-Dynamics/mycelix/pull/527): separation of accrued obligations, held/custodial value, ordinary balances, and execution capability.
- [#528 — economic audit capsule and royalty reporting projections](https://github.com/Luminous-Dynamics/mycelix/pull/528): provenance-rooted audit capsule and reporting-only DDEX/CRD projections.
- [#537 — creator accounting integration and restart-safe settlement](https://github.com/Luminous-Dynamics/mycelix/pull/537): integration and recovery boundaries.
- [#550, #553, #554, #555](https://github.com/Luminous-Dynamics/mycelix/pulls/550): append-only persistence, eligibility evidence, canonical replay, and rail-receipt amount authority.
- [#557–#561](https://github.com/Luminous-Dynamics/mycelix/pulls/557): conservation-bound allocation authority, persistence/replay, allocation lineage, statement binding, and completeness checkpoints.

This lab must not reimplement those durable ledger, persistence, rail-receipt, or statement-authority responsibilities. Its most that it could contribute is a policy result that a separately qualified adapter validates and binds into a versioned allocation authority. Before linking to any implementation type, verify the target PR's merge status, exact source tree and current public API.

## 4. Acceptance gates before integration

All gates below are independent; passing one does not imply the others.

### Usage and eligibility
- An exact source set and explicit completeness/high-water boundary exist for the experiment's period.
- Duplicate event/action identities are rejected before aggregation.
- Eligibility decisions are reproducible and bind the exact source record and policy version.
- Replay, repeated records, fabricated listener identities, partial coverage, late arrivals, and source disagreement have defined outcomes.
- A receipt signature authenticates its signer and payload, but is not by itself proof that playback was human or economically eligible.

### Allocation and rights
- All policies run against the same frozen contribution pool and play aggregates.
- Exact integer accounting and deterministic tie-breaking are independently replayed.
- The input denomination and unit are explicit; currency conversion must be a separate, quote-bound step.
- Composition and master rights are allocated independently under their applicable agreements.
- Unresolved shares remain held, with a reason and claim/reference identity.
- A correction appends a new revision referencing its exact predecessor and evidence; it never overwrites earlier evidence or retroactively changes a finalized external payment.

### Obligations and settlement
- Allocation outputs are not treated as debt until an authorized, canonical obligation compiler admits them.
- Obligation creation is idempotent by a stable identity derived from the authoritative inputs; retry cannot duplicate value.
- Per-currency conservation covers paid, held/escrowed, deducted, and explicit residual value.
- Finality requires a durable rail receipt bound to the exact batch, currency and amount; partial finality, reversal, and dispute retain the debt/receipt history.
- Statement generation is a verified projection over accounting authority, not an independent source of truth.

### Evidence and promotion
- The exact source commit, policy revision, input/source manifest, output manifest, and run parameters are preserved.
- The independent verifier reconstructs the output from the frozen inputs; matching hashes alone do not authenticate the source or signer.
- CI jobs must actually execute on the exact candidate head. Queued, skipped, cancelled, or jobs-less runtime-executor outcomes are not PASS.
- No production route, token issuance, payout, or payability decision is enabled by this research crate.

## 5. Research grounding

The policy comparison is motivated by research that treats pro-rata, user-centric, and Shapley-index allocation as distinct normative choices:

- Bergantiños & Moreno-Ternero, *The Shapley index for music streaming platforms*, Information Economics and Policy 71 (2025), [doi:10.1016/j.infoecopol.2025.101142](https://doi.org/10.1016/j.infoecopol.2025.101142).
- Moreau et al., *Alternative payment models in the music streaming market: A comparative approach based on stream-level data*, Information Economics and Policy 68 (2024), [doi:10.1016/j.infoecopol.2024.101103](https://doi.org/10.1016/j.infoecopol.2024.101103).
- Bergantiños & Moreno-Ternero, *Revenue Sharing at Music Streaming Platforms*, Management Science 71(10) (2025), [doi:10.1287/mnsc.2023.03830](https://doi.org/10.1287/mnsc.2023.03830).

Those studies support comparing mechanisms under explicit assumptions. They do not establish that a particular policy is universally fair, fraud-resistant, or financially sustainable.
