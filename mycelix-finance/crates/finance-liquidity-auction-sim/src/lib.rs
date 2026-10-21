#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure Rust research model for consented, participant-incentive liquidity auctions.
//!
//! Participants can offer liquidity above their own required reserve and bid for a
//! shortfall, specifying integer ask/bid prices. Eligible providers and receivers are
//! matched deterministically. The model conserves liquidity and emits a modeled side
//! payment obligation; it never transfers money, executes a loan, or establishes settlement
//! finality. The caller's authority reference is retained for provenance, not authenticated.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthorityQualification {
    NotQualified,
    /// A caller-side qualification assertion. The reference is not authenticated here.
    ExternallyQualified {
        evidence_ref: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuctionContext {
    pub network_id: String,
    pub asset_id: String,
    pub clearing_window_id: String,
    pub policy_version: String,
    pub authority: AuthorityQualification,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParticipantLiquidity {
    pub participant_id: String,
    pub available_liquidity_minor: u64,
    pub required_reserve_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LiquidityOffer {
    pub offer_id: String,
    pub participant_id: String,
    pub network_id: String,
    pub asset_id: String,
    pub clearing_window_id: String,
    pub policy_version: String,
    pub offered_minor: u64,
    /// Minimum modeled compensation units per unit of liquidity supplied.
    pub minimum_price_units_per_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LiquidityBid {
    pub bid_id: String,
    pub participant_id: String,
    pub network_id: String,
    pub asset_id: String,
    pub clearing_window_id: String,
    pub policy_version: String,
    pub requested_minor: u64,
    /// Maximum modeled compensation units per unit of liquidity received.
    pub maximum_price_units_per_minor: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExclusionReason {
    ProfileMismatch,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExcludedObservation {
    pub observation_id: String,
    pub reason: ExclusionReason,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClearingPriceRule {
    /// Clearing price is ask + floor((bid - ask) / 2).
    MidpointFloor,
    /// Receiver pays the provider's minimum ask; receiver retains all price surplus.
    PayAsAsk,
    /// Receiver pays their maximum bid; provider retains all price surplus.
    PayAsBid,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LiquidityMatch {
    pub offer_id: String,
    pub bid_id: String,
    pub provider_id: String,
    pub receiver_id: String,
    pub amount_minor: u64,
    pub ask_price_units_per_minor: u64,
    pub bid_price_units_per_minor: u64,
    /// Deterministic midpoint price rounded down to an integer unit.
    pub clearing_price_units_per_minor: u64,
    pub compensation_units: u128,
    /// Total bid/ask surplus, not net revenue and not an executed payment.
    pub total_surplus_units: u128,
    pub provider_retained_surplus_units: u128,
    pub receiver_retained_surplus_units: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParticipantAuctionOutcome {
    pub participant_id: String,
    pub opening_liquidity_minor: u64,
    pub required_reserve_minor: u64,
    pub supplied_minor: u64,
    pub received_minor: u64,
    pub available_after_minor: u64,
    pub declared_offer_minor: u64,
    pub unmatched_offer_minor: u64,
    pub declared_bid_minor: u64,
    pub unmatched_bid_minor: u64,
    /// Residual required reserve not covered by the post-auction available balance.
    pub unmet_reserve_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuctionReport {
    pub network_id: String,
    pub asset_id: String,
    pub clearing_window_id: String,
    pub policy_version: String,
    pub authority_evidence_ref: String,
    pub price_rule: ClearingPriceRule,
    pub opening_liquidity_total_minor: u128,
    pub closing_liquidity_total_minor: u128,
    pub eligible_offered_minor: u128,
    pub eligible_requested_minor: u128,
    pub matched_minor: u128,
    pub residual_unmet_reserve_minor: u128,
    pub residual_unmatched_offer_minor: u128,
    pub residual_unmatched_bid_minor: u128,
    pub total_compensation_units: u128,
    pub total_surplus_units: u128,
    pub provider_retained_surplus_units: u128,
    pub receiver_retained_surplus_units: u128,
    pub matches: Vec<LiquidityMatch>,
    pub participants: Vec<ParticipantAuctionOutcome>,
    pub excluded_offers: Vec<ExcludedObservation>,
    pub excluded_bids: Vec<ExcludedObservation>,
    pub deduplicated_observations: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuctionError {
    InvalidContext(&'static str),
    AuthorityNotQualified,
    InvalidParticipant(String),
    DuplicateParticipant(String),
    UnknownParticipant(String),
    InvalidOffer(String),
    InvalidBid(String),
    DuplicateOfferParticipant(String),
    DuplicateBidParticipant(String),
    ConflictingDuplicateOffer(String),
    ConflictingDuplicateBid(String),
    OfferExceedsSurplus {
        offer_id: String,
        offered_minor: u64,
        surplus_minor: u64,
    },
    BidExceedsShortfall {
        bid_id: String,
        requested_minor: u64,
        shortfall_minor: u64,
    },
    ParticipantOnBothSides(String),
    ArithmeticOverflow,
    CostOverflow,
    LiquidityConservationViolation,
    ReserveFloorViolation(String),
    PriceInvariantViolation,
}

impl fmt::Display for AuctionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for AuctionError {}

fn add_u128(left: u128, right: u128) -> Result<u128, AuctionError> {
    left.checked_add(right)
        .ok_or(AuctionError::ArithmeticOverflow)
}

fn checked_product(left: u64, right: u64) -> Result<u128, AuctionError> {
    (left as u128)
        .checked_mul(right as u128)
        .ok_or(AuctionError::CostOverflow)
}

fn require_nonempty(field: &'static str, value: &str) -> Result<(), AuctionError> {
    if value.trim().is_empty() {
        return Err(AuctionError::InvalidContext(field));
    }
    Ok(())
}

fn context_matches(
    context: &AuctionContext,
    network_id: &str,
    asset_id: &str,
    clearing_window_id: &str,
    policy_version: &str,
) -> bool {
    context.network_id == network_id
        && context.asset_id == asset_id
        && context.clearing_window_id == clearing_window_id
        && context.policy_version == policy_version
}

fn validate_context(context: &AuctionContext) -> Result<String, AuctionError> {
    for (name, value) in [
        ("network_id", context.network_id.as_str()),
        ("asset_id", context.asset_id.as_str()),
        ("clearing_window_id", context.clearing_window_id.as_str()),
        ("policy_version", context.policy_version.as_str()),
    ] {
        require_nonempty(name, value)?;
    }

    match &context.authority {
        AuthorityQualification::NotQualified => Err(AuctionError::AuthorityNotQualified),
        AuthorityQualification::ExternallyQualified { evidence_ref } => {
            require_nonempty("authority evidence reference", evidence_ref)?;
            Ok(evidence_ref.clone())
        }
    }
}

/// Run a deterministic pay-as-midpoint auction for one network/asset/window/profile.
///
/// Providers may offer only liquidity above their own reserve; receivers may request only
/// their deficit below reserve. Offers are sorted by ask ascending, bids by bid descending,
/// with canonical ID tie-breaks. Matching stops when the best remaining bid is below the
/// best remaining ask. The matched quantity is maximal for the declared divisible, fixed
/// reservation-price offers/bids. The midpoint price is an explicit reference rule, not a
/// claim of truthfulness or incentive compatibility.
///
/// Side-payment values use abstract compensation units and are *not* deducted from or added
/// to available payment liquidity. The caller must independently establish participant
/// consent, lending terms and any applicable authority before using the result as a plan.
pub fn run_auction(
    context: &AuctionContext,
    participant_observations: &[ParticipantLiquidity],
    offer_observations: &[LiquidityOffer],
    bid_observations: &[LiquidityBid],
) -> Result<AuctionReport, AuctionError> {
    run_auction_with_price_rule(
        context,
        participant_observations,
        offer_observations,
        bid_observations,
        ClearingPriceRule::MidpointFloor,
    )
}

/// Run the same deterministic matching policy with an explicit side-payment rule.
/// This changes only compensation distribution; eligibility and matched quantity remain
/// governed by the same ask/bid constraints.
pub fn run_auction_with_price_rule(
    context: &AuctionContext,
    participant_observations: &[ParticipantLiquidity],
    offer_observations: &[LiquidityOffer],
    bid_observations: &[LiquidityBid],
    price_rule: ClearingPriceRule,
) -> Result<AuctionReport, AuctionError> {
    let authority_evidence_ref = validate_context(context)?;

    let mut participants = BTreeMap::<String, ParticipantLiquidity>::new();
    let mut opening_liquidity_total_minor = 0_u128;
    for participant in participant_observations {
        if participant.participant_id.trim().is_empty() {
            return Err(AuctionError::InvalidParticipant(
                "empty participant_id".into(),
            ));
        }
        if participants.contains_key(&participant.participant_id) {
            return Err(AuctionError::DuplicateParticipant(
                participant.participant_id.clone(),
            ));
        }
        opening_liquidity_total_minor = add_u128(
            opening_liquidity_total_minor,
            participant.available_liquidity_minor as u128,
        )?;
        participants.insert(participant.participant_id.clone(), participant.clone());
    }

    let mut unique_offers = BTreeMap::<String, LiquidityOffer>::new();
    let mut deduplicated_observations = 0_usize;
    for offer in offer_observations {
        for (field, value) in [
            ("offer_id", offer.offer_id.as_str()),
            ("participant_id", offer.participant_id.as_str()),
            ("network_id", offer.network_id.as_str()),
            ("asset_id", offer.asset_id.as_str()),
            ("clearing_window_id", offer.clearing_window_id.as_str()),
            ("policy_version", offer.policy_version.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(AuctionError::InvalidOffer(format!("{field} is empty")));
            }
        }
        if offer.offered_minor == 0 {
            return Err(AuctionError::InvalidOffer(format!(
                "offer {} has zero amount",
                offer.offer_id
            )));
        }
        if let Some(previous) = unique_offers.get(&offer.offer_id) {
            if previous == offer {
                deduplicated_observations = deduplicated_observations
                    .checked_add(1)
                    .ok_or(AuctionError::ArithmeticOverflow)?;
                continue;
            }
            return Err(AuctionError::ConflictingDuplicateOffer(
                offer.offer_id.clone(),
            ));
        }
        unique_offers.insert(offer.offer_id.clone(), offer.clone());
    }

    let mut unique_bids = BTreeMap::<String, LiquidityBid>::new();
    for bid in bid_observations {
        for (field, value) in [
            ("bid_id", bid.bid_id.as_str()),
            ("participant_id", bid.participant_id.as_str()),
            ("network_id", bid.network_id.as_str()),
            ("asset_id", bid.asset_id.as_str()),
            ("clearing_window_id", bid.clearing_window_id.as_str()),
            ("policy_version", bid.policy_version.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(AuctionError::InvalidBid(format!("{field} is empty")));
            }
        }
        if bid.requested_minor == 0 {
            return Err(AuctionError::InvalidBid(format!(
                "bid {} has zero amount",
                bid.bid_id
            )));
        }
        if let Some(previous) = unique_bids.get(&bid.bid_id) {
            if previous == bid {
                deduplicated_observations = deduplicated_observations
                    .checked_add(1)
                    .ok_or(AuctionError::ArithmeticOverflow)?;
                continue;
            }
            return Err(AuctionError::ConflictingDuplicateBid(bid.bid_id.clone()));
        }
        unique_bids.insert(bid.bid_id.clone(), bid.clone());
    }

    let mut excluded_offers = Vec::new();
    let mut eligible_offers = Vec::new();
    let mut offer_by_participant = BTreeMap::<String, LiquidityOffer>::new();
    for offer in unique_offers.into_values() {
        if !context_matches(
            context,
            &offer.network_id,
            &offer.asset_id,
            &offer.clearing_window_id,
            &offer.policy_version,
        ) {
            excluded_offers.push(ExcludedObservation {
                observation_id: offer.offer_id,
                reason: ExclusionReason::ProfileMismatch,
            });
            continue;
        }
        let participant = participants
            .get(&offer.participant_id)
            .ok_or_else(|| AuctionError::UnknownParticipant(offer.participant_id.clone()))?;
        let surplus_minor = participant
            .available_liquidity_minor
            .saturating_sub(participant.required_reserve_minor);
        if offer.offered_minor > surplus_minor {
            return Err(AuctionError::OfferExceedsSurplus {
                offer_id: offer.offer_id,
                offered_minor: offer.offered_minor,
                surplus_minor,
            });
        }
        if offer_by_participant
            .insert(offer.participant_id.clone(), offer.clone())
            .is_some()
        {
            return Err(AuctionError::DuplicateOfferParticipant(
                offer.participant_id,
            ));
        }
        eligible_offers.push(offer);
    }

    let mut excluded_bids = Vec::new();
    let mut eligible_bids = Vec::new();
    let mut bid_by_participant = BTreeMap::<String, LiquidityBid>::new();
    for bid in unique_bids.into_values() {
        if !context_matches(
            context,
            &bid.network_id,
            &bid.asset_id,
            &bid.clearing_window_id,
            &bid.policy_version,
        ) {
            excluded_bids.push(ExcludedObservation {
                observation_id: bid.bid_id,
                reason: ExclusionReason::ProfileMismatch,
            });
            continue;
        }
        let participant = participants
            .get(&bid.participant_id)
            .ok_or_else(|| AuctionError::UnknownParticipant(bid.participant_id.clone()))?;
        let shortfall_minor = participant
            .required_reserve_minor
            .saturating_sub(participant.available_liquidity_minor);
        if bid.requested_minor > shortfall_minor {
            return Err(AuctionError::BidExceedsShortfall {
                bid_id: bid.bid_id,
                requested_minor: bid.requested_minor,
                shortfall_minor,
            });
        }
        if bid_by_participant
            .insert(bid.participant_id.clone(), bid.clone())
            .is_some()
        {
            return Err(AuctionError::DuplicateBidParticipant(bid.participant_id));
        }
        if offer_by_participant.contains_key(&bid.participant_id) {
            return Err(AuctionError::ParticipantOnBothSides(bid.participant_id));
        }
        eligible_bids.push(bid);
    }

    eligible_offers.sort_by(|left, right| {
        left.minimum_price_units_per_minor
            .cmp(&right.minimum_price_units_per_minor)
            .then_with(|| left.participant_id.cmp(&right.participant_id))
            .then_with(|| left.offer_id.cmp(&right.offer_id))
    });
    eligible_bids.sort_by(|left, right| {
        right
            .maximum_price_units_per_minor
            .cmp(&left.maximum_price_units_per_minor)
            .then_with(|| left.participant_id.cmp(&right.participant_id))
            .then_with(|| left.bid_id.cmp(&right.bid_id))
    });

    let mut eligible_offered_minor = 0_u128;
    for offer in &eligible_offers {
        eligible_offered_minor = add_u128(eligible_offered_minor, offer.offered_minor as u128)?;
    }
    let mut eligible_requested_minor = 0_u128;
    for bid in &eligible_bids {
        eligible_requested_minor = add_u128(eligible_requested_minor, bid.requested_minor as u128)?;
    }

    let mut offer_remaining = eligible_offers
        .iter()
        .map(|offer| offer.offered_minor)
        .collect::<Vec<_>>();
    let mut bid_remaining = eligible_bids
        .iter()
        .map(|bid| bid.requested_minor)
        .collect::<Vec<_>>();
    let mut matches = Vec::new();
    let mut total_matched_minor = 0_u128;
    let mut total_compensation_units = 0_u128;
    let mut total_surplus_units = 0_u128;
    let mut provider_retained_surplus_units = 0_u128;
    let mut receiver_retained_surplus_units = 0_u128;

    let mut offer_index = 0_usize;
    let mut bid_index = 0_usize;
    while offer_index < eligible_offers.len() && bid_index < eligible_bids.len() {
        let offer = &eligible_offers[offer_index];
        let bid = &eligible_bids[bid_index];
        if bid.maximum_price_units_per_minor < offer.minimum_price_units_per_minor {
            break;
        }

        let amount_minor = offer_remaining[offer_index].min(bid_remaining[bid_index]);
        if amount_minor == 0 {
            return Err(AuctionError::ArithmeticOverflow);
        }
        let price_spread = bid.maximum_price_units_per_minor - offer.minimum_price_units_per_minor;
        let clearing_price_units_per_minor = match price_rule {
            ClearingPriceRule::MidpointFloor => {
                offer.minimum_price_units_per_minor + price_spread / 2
            }
            ClearingPriceRule::PayAsAsk => offer.minimum_price_units_per_minor,
            ClearingPriceRule::PayAsBid => bid.maximum_price_units_per_minor,
        };
        if clearing_price_units_per_minor < offer.minimum_price_units_per_minor
            || clearing_price_units_per_minor > bid.maximum_price_units_per_minor
        {
            return Err(AuctionError::PriceInvariantViolation);
        }

        let compensation_units = checked_product(amount_minor, clearing_price_units_per_minor)?;
        let total_surplus_for_match = checked_product(amount_minor, price_spread)?;
        let provider_surplus_for_match = checked_product(
            amount_minor,
            clearing_price_units_per_minor - offer.minimum_price_units_per_minor,
        )?;
        let receiver_surplus_for_match = checked_product(
            amount_minor,
            bid.maximum_price_units_per_minor - clearing_price_units_per_minor,
        )?;

        matches.push(LiquidityMatch {
            offer_id: offer.offer_id.clone(),
            bid_id: bid.bid_id.clone(),
            provider_id: offer.participant_id.clone(),
            receiver_id: bid.participant_id.clone(),
            amount_minor,
            ask_price_units_per_minor: offer.minimum_price_units_per_minor,
            bid_price_units_per_minor: bid.maximum_price_units_per_minor,
            clearing_price_units_per_minor,
            compensation_units,
            total_surplus_units: total_surplus_for_match,
            provider_retained_surplus_units: provider_surplus_for_match,
            receiver_retained_surplus_units: receiver_surplus_for_match,
        });

        total_matched_minor = add_u128(total_matched_minor, amount_minor as u128)?;
        total_compensation_units = total_compensation_units
            .checked_add(compensation_units)
            .ok_or(AuctionError::CostOverflow)?;
        total_surplus_units = total_surplus_units
            .checked_add(total_surplus_for_match)
            .ok_or(AuctionError::CostOverflow)?;
        provider_retained_surplus_units = provider_retained_surplus_units
            .checked_add(provider_surplus_for_match)
            .ok_or(AuctionError::CostOverflow)?;
        receiver_retained_surplus_units = receiver_retained_surplus_units
            .checked_add(receiver_surplus_for_match)
            .ok_or(AuctionError::CostOverflow)?;

        offer_remaining[offer_index] -= amount_minor;
        bid_remaining[bid_index] -= amount_minor;
        if offer_remaining[offer_index] == 0 {
            offer_index += 1;
        }
        if bid_remaining[bid_index] == 0 {
            bid_index += 1;
        }
    }

    if provider_retained_surplus_units
        .checked_add(receiver_retained_surplus_units)
        .ok_or(AuctionError::CostOverflow)?
        != total_surplus_units
    {
        return Err(AuctionError::PriceInvariantViolation);
    }

    let mut supplied_by_participant = BTreeMap::<String, u64>::new();
    let mut received_by_participant = BTreeMap::<String, u64>::new();
    for matched in &matches {
        let current_supplied = supplied_by_participant
            .get(&matched.provider_id)
            .copied()
            .unwrap_or(0);
        supplied_by_participant.insert(
            matched.provider_id.clone(),
            current_supplied
                .checked_add(matched.amount_minor)
                .ok_or(AuctionError::ArithmeticOverflow)?,
        );

        let current_received = received_by_participant
            .get(&matched.receiver_id)
            .copied()
            .unwrap_or(0);
        received_by_participant.insert(
            matched.receiver_id.clone(),
            current_received
                .checked_add(matched.amount_minor)
                .ok_or(AuctionError::ArithmeticOverflow)?,
        );
    }

    let mut participants_outcome = Vec::with_capacity(participants.len());
    let mut closing_liquidity_total_minor = 0_u128;
    let mut residual_unmet_reserve_minor = 0_u128;
    let mut residual_unmatched_offer_minor = 0_u128;
    let mut residual_unmatched_bid_minor = 0_u128;

    for participant in participants.values() {
        let supplied_minor = supplied_by_participant
            .get(&participant.participant_id)
            .copied()
            .unwrap_or(0);
        let received_minor = received_by_participant
            .get(&participant.participant_id)
            .copied()
            .unwrap_or(0);

        let after_lending = participant
            .available_liquidity_minor
            .checked_sub(supplied_minor)
            .ok_or_else(|| {
                AuctionError::ReserveFloorViolation(participant.participant_id.clone())
            })?;
        let available_after_minor = after_lending
            .checked_add(received_minor)
            .ok_or(AuctionError::ArithmeticOverflow)?;

        if supplied_minor > 0 && after_lending < participant.required_reserve_minor {
            return Err(AuctionError::ReserveFloorViolation(
                participant.participant_id.clone(),
            ));
        }

        let declared_offer_minor = offer_by_participant
            .get(&participant.participant_id)
            .map(|offer| offer.offered_minor)
            .unwrap_or(0);
        let declared_bid_minor = bid_by_participant
            .get(&participant.participant_id)
            .map(|bid| bid.requested_minor)
            .unwrap_or(0);
        let unmatched_offer_minor = declared_offer_minor - supplied_minor;
        let unmatched_bid_minor = declared_bid_minor - received_minor;
        let unmet_reserve_minor = participant
            .required_reserve_minor
            .saturating_sub(available_after_minor);

        closing_liquidity_total_minor =
            add_u128(closing_liquidity_total_minor, available_after_minor as u128)?;
        residual_unmet_reserve_minor =
            add_u128(residual_unmet_reserve_minor, unmet_reserve_minor as u128)?;
        residual_unmatched_offer_minor = add_u128(
            residual_unmatched_offer_minor,
            unmatched_offer_minor as u128,
        )?;
        residual_unmatched_bid_minor =
            add_u128(residual_unmatched_bid_minor, unmatched_bid_minor as u128)?;

        participants_outcome.push(ParticipantAuctionOutcome {
            participant_id: participant.participant_id.clone(),
            opening_liquidity_minor: participant.available_liquidity_minor,
            required_reserve_minor: participant.required_reserve_minor,
            supplied_minor,
            received_minor,
            available_after_minor,
            declared_offer_minor,
            unmatched_offer_minor,
            declared_bid_minor,
            unmatched_bid_minor,
            unmet_reserve_minor,
        });
    }

    if closing_liquidity_total_minor != opening_liquidity_total_minor {
        return Err(AuctionError::LiquidityConservationViolation);
    }

    excluded_offers.sort_by(|left, right| left.observation_id.cmp(&right.observation_id));
    excluded_bids.sort_by(|left, right| left.observation_id.cmp(&right.observation_id));
    matches.sort_by(|left, right| {
        left.offer_id
            .cmp(&right.offer_id)
            .then_with(|| left.bid_id.cmp(&right.bid_id))
    });

    Ok(AuctionReport {
        network_id: context.network_id.clone(),
        asset_id: context.asset_id.clone(),
        clearing_window_id: context.clearing_window_id.clone(),
        policy_version: context.policy_version.clone(),
        authority_evidence_ref,
        price_rule,
        opening_liquidity_total_minor,
        closing_liquidity_total_minor,
        eligible_offered_minor,
        eligible_requested_minor,
        matched_minor: total_matched_minor,
        residual_unmet_reserve_minor,
        residual_unmatched_offer_minor,
        residual_unmatched_bid_minor,
        total_compensation_units,
        total_surplus_units,
        provider_retained_surplus_units,
        receiver_retained_surplus_units,
        matches,
        participants: participants_outcome,
        excluded_offers,
        excluded_bids,
        deduplicated_observations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> AuctionContext {
        AuctionContext {
            network_id: "network-test".into(),
            asset_id: "asset-zar-minor".into(),
            clearing_window_id: "window-2026-10-21T00".into(),
            policy_version: "liquidity-auction-v1".into(),
            authority: AuthorityQualification::ExternallyQualified {
                evidence_ref: "evidence://caller-qualified-auction-fixture".into(),
            },
        }
    }

    fn participant(id: &str, available: u64, reserve: u64) -> ParticipantLiquidity {
        ParticipantLiquidity {
            participant_id: id.into(),
            available_liquidity_minor: available,
            required_reserve_minor: reserve,
        }
    }

    fn offer(id: &str, participant_id: &str, amount: u64, ask: u64) -> LiquidityOffer {
        let context = context();
        LiquidityOffer {
            offer_id: id.into(),
            participant_id: participant_id.into(),
            network_id: context.network_id,
            asset_id: context.asset_id,
            clearing_window_id: context.clearing_window_id,
            policy_version: context.policy_version,
            offered_minor: amount,
            minimum_price_units_per_minor: ask,
        }
    }

    fn bid(id: &str, participant_id: &str, amount: u64, max_price: u64) -> LiquidityBid {
        let context = context();
        LiquidityBid {
            bid_id: id.into(),
            participant_id: participant_id.into(),
            network_id: context.network_id,
            asset_id: context.asset_id,
            clearing_window_id: context.clearing_window_id,
            policy_version: context.policy_version,
            requested_minor: amount,
            maximum_price_units_per_minor: max_price,
        }
    }

    #[test]
    fn deterministic_matching_conserves_liquidity_and_splits_modeled_surplus() {
        let participants = vec![
            participant("L1", 100, 60),
            participant("L2", 80, 40),
            participant("B1", 20, 50),
            participant("B2", 0, 20),
        ];
        let offers = vec![
            offer("offer-l1", "L1", 30, 2),
            offer("offer-l2", "L2", 20, 4),
        ];
        let bids = vec![bid("bid-b1", "B1", 20, 5), bid("bid-b2", "B2", 20, 3)];

        let report = run_auction(&context(), &participants, &offers, &bids).unwrap();

        assert_eq!(report.opening_liquidity_total_minor, 200);
        assert_eq!(report.closing_liquidity_total_minor, 200);
        assert_eq!(report.eligible_offered_minor, 50);
        assert_eq!(report.eligible_requested_minor, 40);
        assert_eq!(report.matched_minor, 30);
        assert_eq!(report.total_compensation_units, 80);
        assert_eq!(report.total_surplus_units, 70);
        assert_eq!(report.provider_retained_surplus_units, 20);
        assert_eq!(report.receiver_retained_surplus_units, 50);
        assert_eq!(report.residual_unmatched_offer_minor, 20);
        assert_eq!(report.residual_unmatched_bid_minor, 10);
        assert_eq!(report.residual_unmet_reserve_minor, 20);
        assert_eq!(report.matches.len(), 2);
        assert!(report.matches.iter().all(|matched| {
            matched.clearing_price_units_per_minor >= matched.ask_price_units_per_minor
                && matched.clearing_price_units_per_minor <= matched.bid_price_units_per_minor
        }));

        let provider = report
            .participants
            .iter()
            .find(|participant| participant.participant_id == "L1")
            .unwrap();
        assert_eq!(provider.available_after_minor, 70);
        assert!(provider.available_after_minor >= provider.required_reserve_minor);
    }

    #[test]
    fn price_rules_change_surplus_distribution_not_matched_quantity() {
        let participants = vec![participant("L", 10, 5), participant("B", 0, 5)];
        let offers = [offer("ask", "L", 5, 2)];
        let bids = [bid("bid", "B", 5, 5)];

        let midpoint = run_auction_with_price_rule(
            &context(),
            &participants,
            &offers,
            &bids,
            ClearingPriceRule::MidpointFloor,
        )
        .unwrap();
        let pay_as_ask = run_auction_with_price_rule(
            &context(),
            &participants,
            &offers,
            &bids,
            ClearingPriceRule::PayAsAsk,
        )
        .unwrap();
        let pay_as_bid = run_auction_with_price_rule(
            &context(),
            &participants,
            &offers,
            &bids,
            ClearingPriceRule::PayAsBid,
        )
        .unwrap();

        assert_eq!(midpoint.matched_minor, 5);
        assert_eq!(pay_as_ask.matched_minor, midpoint.matched_minor);
        assert_eq!(pay_as_bid.matched_minor, midpoint.matched_minor);
        assert_eq!(midpoint.price_rule, ClearingPriceRule::MidpointFloor);
        assert_eq!(pay_as_ask.price_rule, ClearingPriceRule::PayAsAsk);
        assert_eq!(pay_as_bid.price_rule, ClearingPriceRule::PayAsBid);

        assert_eq!(midpoint.total_compensation_units, 15);
        assert_eq!(pay_as_ask.total_compensation_units, 10);
        assert_eq!(pay_as_bid.total_compensation_units, 25);
        assert_eq!(midpoint.provider_retained_surplus_units, 5);
        assert_eq!(midpoint.receiver_retained_surplus_units, 10);
        assert_eq!(pay_as_ask.provider_retained_surplus_units, 0);
        assert_eq!(pay_as_ask.receiver_retained_surplus_units, 15);
        assert_eq!(pay_as_bid.provider_retained_surplus_units, 15);
        assert_eq!(pay_as_bid.receiver_retained_surplus_units, 0);

        assert_eq!(midpoint.total_surplus_units, pay_as_ask.total_surplus_units);
        assert_eq!(midpoint.total_surplus_units, pay_as_bid.total_surplus_units);
    }

    #[test]
    fn unmatched_prices_do_not_clear_and_do_not_transfer_liquidity() {
        let participants = vec![participant("L", 10, 5), participant("B", 0, 5)];
        let report = run_auction(
            &context(),
            &participants,
            &[offer("ask", "L", 5, 8)],
            &[bid("bid", "B", 5, 7)],
        )
        .unwrap();

        assert_eq!(report.matched_minor, 0);
        assert_eq!(report.total_compensation_units, 0);
        assert_eq!(
            report.opening_liquidity_total_minor,
            report.closing_liquidity_total_minor
        );
        assert_eq!(
            report
                .participants
                .iter()
                .find(|participant| participant.participant_id == "L")
                .unwrap()
                .available_after_minor,
            10
        );
        assert_eq!(
            report
                .participants
                .iter()
                .find(|participant| participant.participant_id == "B")
                .unwrap()
                .available_after_minor,
            0
        );
    }

    #[test]
    fn provider_cannot_offer_below_its_required_reserve() {
        let participants = vec![participant("L", 10, 8), participant("B", 0, 2)];
        assert_eq!(
            run_auction(
                &context(),
                &participants,
                &[offer("too-large", "L", 3, 1)],
                &[bid("bid", "B", 2, 2)],
            ),
            Err(AuctionError::OfferExceedsSurplus {
                offer_id: "too-large".into(),
                offered_minor: 3,
                surplus_minor: 2,
            })
        );
    }

    #[test]
    fn receiver_cannot_bid_for_more_than_its_reserve_shortfall() {
        let participants = vec![participant("L", 10, 5), participant("B", 1, 4)];
        assert_eq!(
            run_auction(
                &context(),
                &participants,
                &[offer("offer", "L", 5, 1)],
                &[bid("too-large", "B", 4, 2)],
            ),
            Err(AuctionError::BidExceedsShortfall {
                bid_id: "too-large".into(),
                requested_minor: 4,
                shortfall_minor: 3,
            })
        );
    }

    #[test]
    fn profile_mismatch_is_excluded_and_never_matched() {
        let participants = vec![participant("L", 10, 5), participant("B", 0, 5)];
        let mut bad_offer = offer("foreign-offer", "L", 5, 1);
        bad_offer.asset_id = "asset-usd-minor".into();
        let mut bad_bid = bid("foreign-bid", "B", 5, 3);
        bad_bid.clearing_window_id = "another-window".into();

        let report = run_auction(&context(), &participants, &[bad_offer], &[bad_bid]).unwrap();
        assert_eq!(report.matched_minor, 0);
        assert_eq!(
            report.excluded_offers,
            vec![ExcludedObservation {
                observation_id: "foreign-offer".into(),
                reason: ExclusionReason::ProfileMismatch,
            }]
        );
        assert_eq!(
            report.excluded_bids,
            vec![ExcludedObservation {
                observation_id: "foreign-bid".into(),
                reason: ExclusionReason::ProfileMismatch,
            }]
        );
    }

    #[test]
    fn identical_observations_deduplicate_but_conflicting_ids_fail_closed() {
        let participants = vec![participant("L", 10, 5), participant("B", 0, 5)];
        let offer_observation = offer("offer", "L", 5, 1);
        let bid_observation = bid("bid", "B", 5, 3);
        let report = run_auction(
            &context(),
            &participants,
            &[offer_observation.clone(), offer_observation.clone()],
            &[bid_observation.clone(), bid_observation.clone()],
        )
        .unwrap();
        assert_eq!(report.deduplicated_observations, 2);
        assert_eq!(report.matched_minor, 5);

        let conflicting = offer("offer", "L", 4, 1);
        assert_eq!(
            run_auction(
                &context(),
                &participants,
                &[offer_observation, conflicting],
                &[bid_observation],
            ),
            Err(AuctionError::ConflictingDuplicateOffer("offer".into()))
        );
    }

    #[test]
    fn conflicting_duplicate_bid_id_fails_closed() {
        let participants = vec![participant("L", 10, 5), participant("B", 0, 5)];
        let first = bid("bid", "B", 5, 3);
        let conflicting = bid("bid", "B", 4, 3);
        assert_eq!(
            run_auction(
                &context(),
                &participants,
                &[offer("offer", "L", 5, 1)],
                &[first, conflicting],
            ),
            Err(AuctionError::ConflictingDuplicateBid("bid".into()))
        );
    }

    #[test]
    fn one_participant_cannot_submit_multiple_quote_ids_in_one_side() {
        let participants = vec![participant("L", 20, 10), participant("B", 0, 10)];
        assert_eq!(
            run_auction(
                &context(),
                &participants,
                &[offer("offer-1", "L", 5, 1), offer("offer-2", "L", 5, 2),],
                &[bid("bid", "B", 5, 3)],
            ),
            Err(AuctionError::DuplicateOfferParticipant("L".into()))
        );
        assert_eq!(
            run_auction(
                &context(),
                &participants,
                &[offer("offer", "L", 5, 1)],
                &[bid("bid-1", "B", 3, 3), bid("bid-2", "B", 2, 2)],
            ),
            Err(AuctionError::DuplicateBidParticipant("B".into()))
        );
    }

    #[test]
    fn empty_authority_evidence_reference_is_rejected() {
        let mut invalid = context();
        invalid.authority = AuthorityQualification::ExternallyQualified {
            evidence_ref: "  ".into(),
        };
        assert_eq!(
            run_auction(
                &invalid,
                &[participant("L", 10, 5), participant("B", 0, 5)],
                &[],
                &[],
            ),
            Err(AuctionError::InvalidContext("authority evidence reference"))
        );
    }

    #[test]
    fn output_is_invariant_to_observation_order() {
        let participants = vec![
            participant("L1", 12, 6),
            participant("L2", 9, 4),
            participant("B1", 0, 7),
            participant("B2", 1, 5),
        ];
        let offers = vec![offer("offer-1", "L1", 6, 2), offer("offer-2", "L2", 5, 4)];
        let bids = vec![bid("bid-1", "B1", 7, 6), bid("bid-2", "B2", 4, 3)];
        let first = run_auction(&context(), &participants, &offers, &bids).unwrap();
        let mut reversed_participants = participants.clone();
        reversed_participants.reverse();
        let mut reversed_offers = offers.clone();
        reversed_offers.reverse();
        let mut reversed_bids = bids.clone();
        reversed_bids.reverse();
        let second = run_auction(
            &context(),
            &reversed_participants,
            &reversed_offers,
            &reversed_bids,
        )
        .unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn not_qualified_context_fails_closed() {
        let mut invalid = context();
        invalid.authority = AuthorityQualification::NotQualified;
        assert_eq!(
            run_auction(
                &invalid,
                &[participant("L", 10, 5), participant("B", 0, 5)],
                &[offer("offer", "L", 5, 1)],
                &[bid("bid", "B", 5, 3)],
            ),
            Err(AuctionError::AuthorityNotQualified)
        );
    }

    #[test]
    fn unknown_eligible_participant_fails_closed() {
        assert_eq!(
            run_auction(
                &context(),
                &[participant("L", 10, 5)],
                &[offer("offer", "missing", 5, 1)],
                &[],
            ),
            Err(AuctionError::UnknownParticipant("missing".into()))
        );
    }

    #[test]
    fn duplicate_participants_fail_closed() {
        let p = participant("L", 10, 5);
        assert_eq!(
            run_auction(&context(), &[p.clone(), p], &[], &[]),
            Err(AuctionError::DuplicateParticipant("L".into()))
        );
    }

    #[test]
    fn total_compensation_overflow_is_reported() {
        let participants = vec![
            participant("L1", u64::MAX, 0),
            participant("L2", u64::MAX, 0),
            participant("B1", 0, u64::MAX),
            participant("B2", 0, u64::MAX),
        ];
        let offers = vec![
            offer("offer-1", "L1", u64::MAX, u64::MAX),
            offer("offer-2", "L2", u64::MAX, u64::MAX),
        ];
        let bids = vec![
            bid("bid-1", "B1", u64::MAX, u64::MAX),
            bid("bid-2", "B2", u64::MAX, u64::MAX),
        ];
        assert_eq!(
            run_auction(&context(), &participants, &offers, &bids),
            Err(AuctionError::CostOverflow)
        );
    }
}
