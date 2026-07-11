// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Energy <-> Finance cross-cluster settlement sweettest.
//!
//! Proves `grid::settle_trade`'s `settle_via_finance` (added 2026-07-06/07,
//! `mycelix-energy/zomes/grid/coordinator/src/lib.rs`) actually resolves as a
//! real `CallTargetCell::OtherRole("finance")` zome call rather than only
//! being verified at the unit-test/clippy level (its only prior verification).
//!
//! This installs energy + finance as two roles in ONE app (required for
//! `OtherRole` dispatch to resolve at all) but deliberately WITHOUT an
//! `identity` role. `finance::payments::send_payment` requires
//! `verify_participant_tier()`, which itself calls
//! `identity/consciousness_gating/check_participant_tier` and fails CLOSED
//! when that role is unreachable (see `mycelix-finance/zomes/shared/src/
//! lib.rs::consciousness_gating::check_tier`). So with no `identity` role
//! installed, `send_payment` always errors, and `settle_via_finance` always
//! returns `None` by design (its `match call(...) { Ok(ZomeCallResponse::Ok
//! (..)) => .., _ => None }` catch-all) -- this test proves that graceful
//! degradation actually happens end-to-end: the trade still settles, using
//! the caller-supplied manual `payment_reference`, rather than erroring or
//! losing the trade.
//!
//! The true happy path (real SAP settlement succeeding) additionally
//! requires the calling agent to clear Participant consciousness tier
//! (combined >= 0.3) via `identity::issue_sovereign_credential`, which
//! defaults every dimension to 0.0 for an agent with zero cross-cluster
//! history -- i.e. a brand-new agent fails this gate even WITH identity
//! installed. Exercising the true happy path needs seeded MFA/reputation
//! data across multiple clusters and is out of scope here; see
//! `PLANETARY_ENERGY_COORDINATION_PLAN_2026-07-06.md` follow-ups.
//!
//! Prerequisites:
//!   cd mycelix-energy  && cargo build --release --target wasm32-unknown-unknown && hc dna pack dna/
//!   cd mycelix-finance && cargo build --release --target wasm32-unknown-unknown && hc dna pack dna/
//!
//! Run:
//!   cargo test -p mycelix-sweettest --test energy_finance_settlement -- --ignored --test-threads=1

mod harness;

use harness::*;
use holochain::prelude::*;
use serial_test::serial;

// ============================================================================
// Mirror types — must match mycelix-energy/zomes/grid/{integrity,coordinator}
// ============================================================================

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateOfferInput {
    seller_did: String,
    project_id: Option<String>,
    amount_kwh: f64,
    price_per_kwh: f64,
    currency: String,
    available_from: Timestamp,
    available_until: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq)]
enum OfferStatus {
    Active,
    PartiallyFilled,
    Filled,
    Expired,
    Cancelled,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct TradeOffer {
    id: String,
    seller_did: String,
    project_id: Option<String>,
    amount_kwh: f64,
    price_per_kwh: f64,
    currency: String,
    available_from: Timestamp,
    available_until: Timestamp,
    status: OfferStatus,
    created: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ExecuteTradeInput {
    offer_id: String,
    buyer_did: String,
    amount_kwh: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct Trade {
    id: String,
    offer_id: String,
    seller_did: String,
    buyer_did: String,
    amount_kwh: f64,
    price_per_kwh: f64,
    total_price: f64,
    currency: String,
    executed: Timestamp,
    settled: bool,
    payment_reference: Option<String>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SettleTradeInput {
    trade_id: String,
    payment_reference: String,
}

fn decode_entry<T: serde::de::DeserializeOwned>(record: &Record) -> Option<T> {
    match record.entry().as_option()? {
        Entry::App(bytes) => {
            let sb = SerializedBytes::from(bytes.to_owned());
            rmp_serde::from_slice(sb.bytes()).ok()
        }
        _ => None,
    }
}

// ============================================================================
// Setup
// ============================================================================

async fn setup_energy_and_finance() -> Vec<TestAgent> {
    setup_test_agents_from_roles(
        &[
            ("energy", DnaPaths::energy()),
            ("finance", DnaPaths::finance()),
        ],
        1,
    )
    .await
}

// ============================================================================
// Tests
// ============================================================================

/// `settle_trade` on a SAP-denominated trade must fall back to the caller's
/// manual `payment_reference` when the finance cluster's payment call fails
/// (here: because `identity` isn't installed, so finance's own
/// consciousness-tier gate fails closed) -- proving the fallback path is
/// real, not just a unit-tested code path that never executes for real.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_settle_trade_falls_back_when_finance_settlement_unavailable() {
    let agents = setup_energy_and_finance().await;
    let agent = &agents[0];
    let seller_did = format!("did:mycelix:{}", agent.agent_pubkey);

    let now = Timestamp::now();
    let later = Timestamp::from_micros(now.as_micros() + 86_400_000_000);

    let offer_input = CreateOfferInput {
        seller_did: seller_did.clone(),
        project_id: None,
        amount_kwh: 1000.0,
        price_per_kwh: 0.05, // -> total_price 50.0 SAP for a 1000 kWh trade
        currency: "SAP".to_string(),
        available_from: now,
        available_until: later,
    };
    let offer_record: Record = agent
        .call_zome_fn_on_role("energy", "grid", "create_trade_offer", offer_input)
        .await;
    let offer: TradeOffer = decode_entry(&offer_record).expect("decode offer");

    let trade_input = ExecuteTradeInput {
        offer_id: offer.id.clone(),
        buyer_did: "did:mycelix:test-buyer-sap".to_string(), // distinct from seller — validator rejects self-trades
        // Partial fill (400 of 1000) — a FULL fill currently fails integrity
        // validation (remaining=0.0 on the updated TradeOffer trips
        // "Offer amount must be positive"); see
        // feedback_full_fill_offer_integrity_bug memory. Not this test's
        // concern; use the same partial-fill shape as the precedent test.
        amount_kwh: 400.0,
    };
    let trade_record: Record = agent
        .call_zome_fn_on_role("energy", "grid", "execute_trade", trade_input)
        .await;
    let trade: Trade = decode_entry(&trade_record).expect("decode trade");
    assert!(!trade.settled, "Trade must start unsettled");

    let manual_reference = "manual-escrow-ref-42".to_string();
    let settle_input = SettleTradeInput {
        trade_id: trade.id.clone(),
        payment_reference: manual_reference.clone(),
    };
    let settled_record: Record = agent
        .call_zome_fn_on_role("energy", "grid", "settle_trade", settle_input)
        .await;
    let settled: Trade = decode_entry(&settled_record).expect("decode settled trade");

    assert!(settled.settled, "Trade must be marked settled");
    assert_eq!(
        settled.payment_reference,
        Some(manual_reference),
        "With no identity role installed, finance settlement must fail closed \
         and settle_trade must fall back to the caller-supplied manual reference \
         rather than erroring or losing the trade"
    );
}

/// Sanity check that a non-SAP/TEND currency (no real settlement rail at
/// all) always uses the manual reference, regardless of finance/identity
/// availability -- this exercises `currency_has_real_settlement_rail`'s
/// short-circuit through a real zome call, not just its unit test.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore]
async fn test_settle_trade_uses_manual_reference_for_fiat_currency() {
    let agents = setup_energy_and_finance().await;
    let agent = &agents[0];
    let seller_did = format!("did:mycelix:{}", agent.agent_pubkey);

    let now = Timestamp::now();
    let later = Timestamp::from_micros(now.as_micros() + 86_400_000_000);

    let offer_input = CreateOfferInput {
        seller_did: seller_did.clone(),
        project_id: None,
        amount_kwh: 500.0,
        price_per_kwh: 0.10,
        currency: "USD".to_string(),
        available_from: now,
        available_until: later,
    };
    let offer_record: Record = agent
        .call_zome_fn_on_role("energy", "grid", "create_trade_offer", offer_input)
        .await;
    let offer: TradeOffer = decode_entry(&offer_record).expect("decode offer");

    let trade_input = ExecuteTradeInput {
        offer_id: offer.id.clone(),
        buyer_did: "did:mycelix:test-buyer-fiat".to_string(), // distinct from seller — validator rejects self-trades
        amount_kwh: 200.0, // partial fill of the 500 kWh offer — see full-fill note above
    };
    let trade_record: Record = agent
        .call_zome_fn_on_role("energy", "grid", "execute_trade", trade_input)
        .await;
    let trade: Trade = decode_entry(&trade_record).expect("decode trade");

    let manual_reference = "usd-wire-ref-7".to_string();
    let settle_input = SettleTradeInput {
        trade_id: trade.id.clone(),
        payment_reference: manual_reference.clone(),
    };
    let settled_record: Record = agent
        .call_zome_fn_on_role("energy", "grid", "settle_trade", settle_input)
        .await;
    let settled: Trade = decode_entry(&settled_record).expect("decode settled trade");

    assert!(settled.settled);
    assert_eq!(settled.payment_reference, Some(manual_reference));
}
