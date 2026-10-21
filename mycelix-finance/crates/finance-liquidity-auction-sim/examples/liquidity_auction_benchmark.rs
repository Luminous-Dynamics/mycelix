use finance_liquidity_auction_sim::{
    run_auction, AuctionContext, AuthorityQualification, LiquidityBid, LiquidityOffer,
    ParticipantLiquidity,
};
use std::time::Instant;

fn make_case(
    participant_count: usize,
) -> (
    AuctionContext,
    Vec<ParticipantLiquidity>,
    Vec<LiquidityOffer>,
    Vec<LiquidityBid>,
) {
    assert!(participant_count >= 4 && participant_count % 2 == 0);
    let half = participant_count / 2;
    let context = AuctionContext {
        network_id: "synthetic-south-africa-profile".into(),
        asset_id: "zar-minor".into(),
        clearing_window_id: format!("synthetic-window-{participant_count}"),
        policy_version: "liquidity-auction-bench-v1".into(),
        authority: AuthorityQualification::ExternallyQualified {
            evidence_ref: "synthetic-fixture://not-legal-authority".into(),
        },
    };

    let mut participants = Vec::with_capacity(participant_count);
    let mut offers = Vec::with_capacity(half);
    let mut bids = Vec::with_capacity(half);

    for i in 0..half {
        let participant_id = format!("L{i:05}");
        let reserve = 1_000_u64;
        let surplus = ((i * 13 + 5) % 100 + 20) as u64;
        participants.push(ParticipantLiquidity {
            participant_id: participant_id.clone(),
            available_liquidity_minor: reserve + surplus,
            required_reserve_minor: reserve,
        });
        let offered = (surplus * 3 / 4).max(1);
        offers.push(LiquidityOffer {
            offer_id: format!("OFFER-{i:05}"),
            participant_id,
            network_id: context.network_id.clone(),
            asset_id: context.asset_id.clone(),
            clearing_window_id: context.clearing_window_id.clone(),
            policy_version: context.policy_version.clone(),
            offered_minor: offered,
            minimum_price_units_per_minor: ((i * 37 + 3) % 21 + 1) as u64,
        });
    }

    for i in 0..half {
        let participant_id = format!("B{i:05}");
        let opening = ((i * 17 + 3) % 20) as u64;
        let shortfall = ((i * 11 + 8) % 80 + 20) as u64;
        participants.push(ParticipantLiquidity {
            participant_id: participant_id.clone(),
            available_liquidity_minor: opening,
            required_reserve_minor: opening + shortfall,
        });
        bids.push(LiquidityBid {
            bid_id: format!("BID-{i:05}"),
            participant_id,
            network_id: context.network_id.clone(),
            asset_id: context.asset_id.clone(),
            clearing_window_id: context.clearing_window_id.clone(),
            policy_version: context.policy_version.clone(),
            requested_minor: (shortfall * 3 / 4).max(1),
            maximum_price_units_per_minor: ((i * 29 + 7) % 25 + 1) as u64,
        });
    }

    (context, participants, offers, bids)
}

fn percentile(values: &[u128], p: usize) -> u128 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (p * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn main() {
    println!(
        "participants,offer_count,bid_count,eligible_offered_minor,eligible_requested_minor,matched_minor,match_rate_bps,residual_unmet_reserve_minor,residual_unmatched_offer_minor,residual_unmatched_bid_minor,compensation_units,surplus_units,provider_surplus_units,receiver_surplus_units,runtime_p50_us,runtime_p95_us"
    );

    for participant_count in [10_usize, 30, 70, 140, 700] {
        let (context, participants, offers, bids) = make_case(participant_count);
        let mut timings = Vec::with_capacity(25);
        let mut final_report = None;
        for _ in 0..25 {
            let start = Instant::now();
            final_report = Some(
                run_auction(&context, &participants, &offers, &bids)
                    .expect("liquidity auction simulation"),
            );
            timings.push(start.elapsed().as_micros());
        }

        let report = final_report.expect("positive repetition count");
        let limiting_side = report
            .eligible_offered_minor
            .min(report.eligible_requested_minor);
        let match_rate_bps = if limiting_side == 0 {
            0
        } else {
            report.matched_minor * 10_000 / limiting_side
        };

        println!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            participant_count,
            offers.len(),
            bids.len(),
            report.eligible_offered_minor,
            report.eligible_requested_minor,
            report.matched_minor,
            match_rate_bps,
            report.residual_unmet_reserve_minor,
            report.residual_unmatched_offer_minor,
            report.residual_unmatched_bid_minor,
            report.total_compensation_units,
            report.total_surplus_units,
            report.provider_retained_surplus_units,
            report.receiver_retained_surplus_units,
            percentile(&timings, 50),
            percentile(&timings, 95),
        );
    }
}
