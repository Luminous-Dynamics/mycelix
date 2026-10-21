# finance-liquidity-auction-sim

Dependency-free pure Rust reference model for consented participant-incentive liquidity offers, receiver bids and modeled side payments.

This is a **simulation and allocation proposal**, not a loan executor, a payment processor, an RTGS/LSM implementation or a legal conclusion. It does not transfer funds, create liquidity, charge a participant or establish external settlement finality.

## Model

- One explicit network, asset, clearing window and policy version per auction.
- Each participant provides an available-liquidity amount and a required reserve for their own obligations.
- A provider may offer only liquidity above their own reserve.
- A receiver may bid only for the shortfall between their current liquidity and required reserve.
- Offers state an integer minimum compensation price per minor unit; bids state a maximum. Wrong-profile observations are preserved as excluded records.
- This first version accepts at most one eligible offer and one eligible bid per participant; duplicate participant tranches fail closed. Multi-tier offer curves are a future model extension.
- Exact duplicate observations with the same canonical offer/bid ID are deduplicated; changed terms under the same ID fail closed.
- Offers sort by ask ascending and stable participant/offer ID tie-break; bids sort by maximum willingness to pay descending and stable participant/bid ID tie-break.
- The reference match rule clears the best remaining bid with the best remaining offer while bid price is at least ask price. Matched quantities are integer minor units.
- For each match, compensation follows the explicitly supplied price rule: floor midpoint, pay-as-ask or pay-as-bid. Compensation is reported in abstract model units; it is not executed, deducted from liquidity, FX-converted or treated as fiat.
- Providers can never lend below their own required reserve. Receivers can have a residual reserve deficit if the auction does not satisfy all requests.
- Total opening and closing available liquidity are compared. Liquidity is transferred between participants; none is created.

The caller-supplied authority evidence reference is retained for provenance only; this crate does not authenticate it or verify contracts, consent, creditworthiness, legal enforceability or provider quote authenticity. A successful match is not a binding loan or a settlement receipt.

## Objective and baseline

The reference matching rule maximizes matched quantity for divisible integer units with fixed reservation prices by matching asks ascending against bids descending. Pricing is configurable: floor midpoint, pay-as-ask or pay-as-bid. These rules change who retains the surplus, not which bids match under this fixed-quote model. None is claimed to be incentive-compatible, truth-revealing, strategically robust, fair in every distribution, or optimal when deadlines, risk, concentration, default or nonlinear utility are added.

Report separately:
- offered, requested, matched and unmatched volume;
- outstanding participant reserve deficits after modeled lending;
- modeled compensation and total bid/ask surplus (with provider/receiver surplus split);
- concentration of supplied liquidity and quote acceptance rates;
- deterministic reproducibility and runtime under a pinned toolchain.

## Run the benchmark

    cargo run --release --manifest-path mycelix-finance/Cargo.toml -p finance-liquidity-auction-sim --example liquidity_auction_benchmark

The example creates deterministic synthetic participants and quotes at 10, 30, 70, 140 and 700 participants. It emits rows for a no-offer/no-bid baseline and for all three pricing rules under the same opening-liquidity vector, reporting quantities, compensation, residual deficits, surplus distribution and p50/p95 runtime. Synthetic profiles are not calibrated to South African production data; they are for regression, scaling and objective-comparison tests.

## Research anchor

BIS Working Paper 1318 (16 December 2025), *Auction-based liquidity saving mechanisms*, studies an auction-based LSM where participants signal their willingness to supply liquidity or pay for it, with side payments used to compensate liquidity providers. It evaluates simulated configurations calibrated to BCRP-RTGS (Peru), SAMOS and SADC-RTGS (South Africa), and SIC (Switzerland), and discusses a trade-off between liquidity efficiency and settlement delay. Those are study-specific results; this crate is not a reproduction of Project Titus or its datasets. https://www.bis.org/publications/working-paper-1318-auction-based-liquidity-saving-mechanisms

## Acceptance gates

- Zero provider reserve-floor violations.
- Zero offered/borrowed amounts beyond the participant's declared surplus/shortfall.
- Matched supply equals matched demand; aggregate liquidity remains unchanged.
- Every match satisfies ask ≤ clearing price ≤ bid; provider plus receiver retained surplus equals total bid/ask surplus.
- Stable full report under reordered participant, offer and bid observations.
- Conflicting duplicate identities, unknown eligible participants, missing authority, and arithmetic overflow fail closed.
- Compare with no-auction and queue-only baselines on identical frozen synthetic fixtures; any future real-data comparison requires data rights and documented preprocessing.
- No superiority, actual savings, legal lending, creditworthiness or settlement-finality claims without independent evidence.
