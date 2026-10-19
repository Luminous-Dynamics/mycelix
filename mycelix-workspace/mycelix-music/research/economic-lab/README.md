# Mycelix Music Economic Allocation Lab

Status: research-only, deterministic policy evaluator. This crate is not a payment processor, wallet, settlement authority, or production royalty router.

## Why this exists

Music-streaming research distinguishes allocation rules that are often conflated:

- **Pro-rata:** pool all eligible creator revenue, then allocate in proportion to platform-wide play counts.
- **User-centric:** allocate each listener's contribution among that listener's eligible artists in proportion to their play counts.
- **Shapley distinct-artist index:** divide each listener's contribution equally among the distinct artists that listener played. This is the Shapley-index rule described by Bergantiños and Moreno-Ternero (2025), not the unrelated practice of using arbitrary Shapley weights.
- **Cooperative equal share:** divide the pool evenly across an explicit membership list, independent of listening. This is a comparator for member-owned collectives, not a streaming-royalty standard.

These policies are deliberately not ranked as universally fair. A static allocation replay can reveal distribution changes, but it cannot by itself establish participant behavior, livelihood improvement, platform viability, cultural diversity, or fraud resistance.

## API and money semantics

The public API is `allocate(&AllocationInput, AllocationPolicy) -> Result<AllocationReport, AllocationError>`.

All amounts are unsigned integer **minor units** for one consistently selected currency/unit. The crate never uses floating-point money, converts currencies, takes platform fees, or creates additional funds. Callers must not combine incompatible currencies. The total pool is the checked sum of the listener contribution inputs.

Fractional allocations use deterministic largest-remainder apportionment. When remainders tie, the lexicographically smaller stable artist identifier gets the earlier minor unit. Pro-rata rounds once over the whole pool; user-centric and the Shapley index round each listener's contribution separately; cooperative equal-share rounds the whole pool across members.

If a listener contribution has no eligible artist plays under user-centric or Shapley allocation, that amount is reported as `unallocated_minor_units`. In production this would require an explicit contractual and legal policy; this crate does not authorize retaining or redirecting it. Pro-rata also reports the full pool as unallocated when there are no eligible plays. Cooperative equal-share requires a non-empty, duplicate-free membership list and can allocate to members who had no plays by design.

Input identifiers are stable labels for deterministic testing, not authentication. Real user identifiers should not be placed into experiment fixtures or exported reports; use per-run pseudonyms and appropriate privacy controls.

## Validation and boundaries

The crate rejects:
- blank or padded identifiers and duplicate listener IDs;
- repeated artist rows within one listener (inputs must be pre-aggregated);
- zero-count play rows;
- checked money or play-weight overflow;
- malformed cooperative membership; and
- any final allocation that does not reconcile exactly to the input pool.

It **does not** validate playback receipts, authenticate identities, detect fraud, resolve rights ownership, split artist shares among songwriters/producers/publishers, handle contested rights, value SAP/TEND/MYCEL, or execute transfers. Eligible play aggregates must come from an upstream process with its own independently qualified integrity rules. This crate must remain disconnected from real settlement until separate authorization, rights, replay, idempotency, and accounting controls are reviewed.

## Research references

- Bergantiños, G. & Moreno-Ternero, J. D. (2025). *The Shapley index for music streaming platforms*. Information Economics and Policy 71, 101142. [https://doi.org/10.1016/j.infoecopol.2025.101142](https://doi.org/10.1016/j.infoecopol.2025.101142). Introduces a subscription allocation rule that equally divides a listener's contribution among the distinct artists they listened to and compares it with pro-rata and user-centric rules.
- Moreau, F. et al. (2024). *Alternative payment models in the music streaming market: A comparative approach based on stream-level data*. Information Economics and Policy 68, 101103. [https://doi.org/10.1016/j.infoecopol.2024.101103](https://doi.org/10.1016/j.infoecopol.2024.101103). Uses 154,505 users and 890 million streams to estimate static distribution effects for alternatives to pro-rata. The reported effects are data- and model-specific.
- Chen, Y. et al. (2025, online publication). *Revenue Sharing at Music Streaming Platforms*. Management Science. [https://doi.org/10.1287/mnsc.2023.03830](https://doi.org/10.1287/mnsc.2023.03830). Develops axiomatic and game-theoretic foundations for existing and intermediate allocation families.

## Run

```sh
cargo test --locked --offline --manifest-path mycelix-workspace/mycelix-music/research/economic-lab/Cargo.toml
cargo fmt --manifest-path mycelix-workspace/mycelix-music/research/economic-lab/Cargo.toml -- --check
```

The dedicated workflow runs these commands on pull requests touching this lab. Passing unit tests proves the tested deterministic code paths only; it does not make any economic or security efficacy claim.


## Rights-holder allocation and correction provenance

The `rights` module models one **separately denominated rights pool** at a time. Composition and master-recording pools are kept distinct. Claims must be pre-aggregated per claimant and sum to exactly 10,000 basis points. Largest-remainder rounding is deterministic by claim ID.

Only `Confirmed` shares are marked payable. `Pending`, `Disputed`, `Unverified`, and `Unmatched` shares are enumerated as escrow lines; the report must reconcile confirmed payments plus escrow to the gross pool. This is a simulation/accounting result, not permission to make a payout.

`RightsRevisionChain` accepts an initial revision and then sequential corrections linked to the exact previous revision ID. It rejects a skipped revision, wrong parent, pool/class changes, missing evidence/reason references, and malformed share totals. Corrections append new definitions; previously generated allocation reports are immutable values and are not recomputed or overwritten.

The revision chain is **in-memory only**. Evidence and reason references are caller-supplied strings, not verified signatures or durable receipts. Production use would still need authenticated rights evidence, authorization, durable append-only persistence, replay/idempotency controls, dispute-resolution policy, and audited settlement behavior. The U.S. Copyright Office's [Music Modernization FAQ](https://www.copyright.gov/music-modernization/faq.html) describes claim and dispute processes for the U.S. blanket mechanical-licensing context; it is an example of why claim status must be explicit, not a universal rule for all rights classes or jurisdictions.


## Integration contract

See [INTEGRATION_CONTRACT.md](./INTEGRATION_CONTRACT.md) for the boundary between this research evaluator and the repository's draft creator-accounting / settlement architecture. The short version: this crate may compare allocation policies over already-qualified inputs; it must not become a second ledger, certify playback, create royalty debt, or execute settlement.
