// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! P2P Grid Trading Coordinator Zome

use grid_integrity::*;
use hdk::prelude::*;
use mycelix_bridge_proc::{mycelix_zome_fn, sovereign_gated};
use mycelix_energy_shared::anchors::anchor_hash;
use mycelix_energy_shared::batch::{filter_records_by, links_to_records};
use mycelix_zome_helpers as _;

#[hdk_extern]
#[sovereign_gated(basic, "energy_bridge")]
pub fn record_production(input: RecordProductionInput) -> ExternResult<Record> {
    let now = sys_time()?;
    let production = EnergyProduction {
        id: format!("prod:{}:{}", input.producer_did, now.as_micros()),
        producer_did: input.producer_did.clone(),
        project_id: input.project_id,
        amount_kwh: input.amount_kwh,
        timestamp: now,
        period_hours: input.period_hours,
        meter_reading: input.meter_reading,
        verified: false,
    };

    let action_hash = create_entry(&EntryTypes::EnergyProduction(production))?;
    create_link(
        anchor_hash(&input.producer_did)?,
        action_hash.clone(),
        LinkTypes::ProducerToProduction,
        (),
    )?;
    get(action_hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct RecordProductionInput {
    pub producer_did: String,
    pub project_id: String,
    pub amount_kwh: f64,
    pub period_hours: f64,
    pub meter_reading: Option<f64>,
}

/// Seconds per day, for bucketing offer-creation timestamps into daily
/// shards. See `day_bucket`.
const SECONDS_PER_DAY: i64 = 86_400;

/// How many days of shards `get_active_offers` scans backward from today.
/// Bounds the read cost to a fixed number of anchors regardless of how many
/// offers have EVER been created -- the actual DHT-hotspot fix. Offers
/// created further back than this are no longer discoverable via
/// `get_active_offers` (they've long since expired via `available_until`
/// in any realistic marketplace; this crate has no explicit archival path
/// for offers open longer than this window).
const OFFER_ANCHOR_LOOKBACK_DAYS: i64 = 30;

/// Which day-bucket a timestamp falls into (days since the Unix epoch,
/// UTC). Pure and unit-testable -- no HDK host calls.
fn day_bucket(timestamp: Timestamp) -> i64 {
    timestamp
        .as_micros()
        .div_euclid(SECONDS_PER_DAY * 1_000_000)
}

/// The topic-sharded, time-boxed anchor key for a given day-bucket. Linking
/// every offer ever created to one static "active_energy_offers" anchor
/// (the prior scheme) is exactly the DHT hotspot anti-pattern: one node
/// ends up holding links for the marketplace's entire history. Sharding by
/// creation day bounds each anchor's link count to roughly one day's worth
/// of offers.
fn offer_anchor_key(bucket: i64) -> String {
    format!("active_energy_offers:{bucket}")
}

/// The list of day-bucket anchor keys `get_active_offers` scans, from
/// `lookback_days` ago through today (inclusive). Pure and unit-testable.
fn offer_anchor_keys_for_lookback(now: Timestamp, lookback_days: i64) -> Vec<String> {
    let today = day_bucket(now);
    ((today - lookback_days)..=today)
        .map(offer_anchor_key)
        .collect()
}

#[hdk_extern]
pub fn create_trade_offer(input: CreateOfferInput) -> ExternResult<Record> {
    let now = sys_time()?;
    let offer = TradeOffer {
        id: format!("offer:{}:{}", input.seller_did, now.as_micros()),
        seller_did: input.seller_did.clone(),
        project_id: input.project_id,
        amount_kwh: input.amount_kwh,
        price_per_kwh: input.price_per_kwh,
        currency: input.currency,
        available_from: input.available_from,
        available_until: input.available_until,
        status: OfferStatus::Active,
        created: now,
    };

    let action_hash = create_entry(&EntryTypes::TradeOffer(offer))?;
    create_link(
        anchor_hash(&input.seller_did)?,
        action_hash.clone(),
        LinkTypes::SellerToOffers,
        (),
    )?;
    let anchor = anchor_hash(&offer_anchor_key(day_bucket(now)))?;
    create_link(anchor, action_hash.clone(), LinkTypes::ActiveOffers, ())?;
    get(action_hash, GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateOfferInput {
    pub seller_did: String,
    pub project_id: Option<String>,
    pub amount_kwh: f64,
    pub price_per_kwh: f64,
    pub currency: String,
    pub available_from: Timestamp,
    pub available_until: Timestamp,
}

/// Whether a trade offer's availability window still covers `now`. Pure and
/// unit-testable — no Holochain host calls, unlike `sys_time()` at the call
/// sites.
fn offer_not_expired(offer: &TradeOffer, now: Timestamp) -> bool {
    offer.available_until >= now
}

#[hdk_extern]
pub fn execute_trade(input: ExecuteTradeInput) -> ExternResult<Record> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::TradeOffer,
        )?))
        .include_entries(true);
    for record in query(filter)? {
        if let Some(offer) = record.entry().to_app_option::<TradeOffer>().ok().flatten() {
            if offer.id == input.offer_id && offer.status == OfferStatus::Active {
                let now = sys_time()?;
                if !offer_not_expired(&offer, now) {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Offer has expired".into()
                    )));
                }
                let total_price = input.amount_kwh * offer.price_per_kwh;

                let trade = Trade {
                    id: format!("trade:{}:{}", input.offer_id, now.as_micros()),
                    offer_id: input.offer_id.clone(),
                    seller_did: offer.seller_did.clone(),
                    buyer_did: input.buyer_did.clone(),
                    amount_kwh: input.amount_kwh,
                    price_per_kwh: offer.price_per_kwh,
                    total_price,
                    currency: offer.currency.clone(),
                    executed: now,
                    settled: false,
                    payment_reference: None,
                };

                let trade_hash = create_entry(&EntryTypes::Trade(trade))?;
                create_link(
                    anchor_hash(&input.offer_id)?,
                    trade_hash.clone(),
                    LinkTypes::OfferToTrades,
                    (),
                )?;
                create_link(
                    anchor_hash(&input.buyer_did)?,
                    trade_hash.clone(),
                    LinkTypes::BuyerToTrades,
                    (),
                )?;

                // Update offer status
                let remaining = offer.amount_kwh - input.amount_kwh;
                let new_status = if remaining <= 0.0 {
                    OfferStatus::Filled
                } else {
                    OfferStatus::PartiallyFilled
                };
                let updated_offer = TradeOffer {
                    amount_kwh: remaining.max(0.0),
                    status: new_status,
                    ..offer
                };
                update_entry(
                    record.action_address().clone(),
                    &EntryTypes::TradeOffer(updated_offer),
                )?;

                return get(trade_hash, GetOptions::default())?
                    .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Offer not found or not active".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ExecuteTradeInput {
    pub offer_id: String,
    pub buyer_did: String,
    pub amount_kwh: f64,
}

/// Get all active trade offers
///
/// OPTIMIZED: Uses batch query to avoid N+1 pattern
///
/// Expiry is enforced here on read rather than by mutating stored offers:
/// only an offer's own seller agent can `update_entry` it in Holochain, so a
/// getter called by any peer cannot durably transition someone else's offer
/// to `Expired`. Offers past `available_until` are therefore excluded from
/// the active set even though their persisted status still reads
/// Active/PartiallyFilled.
///
/// Offers are linked from `OFFER_ANCHOR_LOOKBACK_DAYS`+1 daily-sharded
/// anchors (see `offer_anchor_key`) rather than one global anchor, so this
/// scans a fixed, bounded number of anchors regardless of how many offers
/// have ever been created -- the DHT-hotspot fix per this repo's own
/// scalability rules (topic/time-sharded anchors, never a "list of all X").
#[hdk_extern]
pub fn get_active_offers(_: ()) -> ExternResult<Vec<Record>> {
    let now = sys_time()?;
    let mut all_records = Vec::new();
    for key in offer_anchor_keys_for_lookback(now, OFFER_ANCHOR_LOOKBACK_DAYS) {
        let anchor = anchor_hash(&key)?;
        let links = get_links(
            LinkQuery::try_new(anchor, LinkTypes::ActiveOffers)?,
            GetStrategy::default(),
        )?;
        // FIXED N+1: Batch fetch all records per shard, then filter
        all_records.extend(links_to_records(links)?);
    }
    Ok(filter_records_by::<TradeOffer, _>(&all_records, |offer| {
        (offer.status == OfferStatus::Active || offer.status == OfferStatus::PartiallyFilled)
            && offer_not_expired(offer, now)
    }))
}

/// Micro-units per whole SAP/TEND, matching the display convention used
/// throughout the finance cluster's frontend (`amount as f64 / 1_000_000.0`
/// -- see e.g. `mycelix-finance/apps/leptos/src/pages/profile.rs`).
const SAP_MICRO_UNITS_PER_UNIT: f64 = 1_000_000.0;

/// Whether `currency` has a real settlement rail in this system --
/// `payments::send_payment` only accepts "SAP" or "TEND". Pure and
/// unit-testable (no HDK host calls), unlike `settle_via_finance` itself.
fn currency_has_real_settlement_rail(currency: &str) -> bool {
    currency == "SAP" || currency == "TEND"
}

/// Convert a whole-unit SAP/TEND amount (as stored in `Trade::total_price`)
/// to the micro-unit integer `payments::send_payment` expects. Pure and
/// unit-testable.
fn whole_units_to_micro(whole_units: f64) -> u64 {
    (whole_units * SAP_MICRO_UNITS_PER_UNIT).round() as u64
}

/// Attempt real settlement via the finance cluster's payment rail.
///
/// Only SAP and TEND can be settled this way -- they're the only
/// currencies `payments::send_payment` accepts (`mycelix-finance/zomes/
/// payments/coordinator/src/lib.rs`). P2P grid trades denominated in
/// fiat-like currencies (USD, EUR, etc. -- used throughout this crate's
/// existing tests and offer data) have no real settlement rail anywhere
/// in this system; those fall back to the caller-supplied manual
/// `payment_reference`, exactly as `settle_trade` behaved before this
/// change.
///
/// Note: `send_payment` internally requires the CALLING agent's key to
/// match `from_did` (`verify_caller_is_did`), so real settlement only
/// succeeds when the buyer's own agent calls `settle_trade` -- this is
/// the correct authorization boundary (a payer must authorize their own
/// payment), not a bug to route around.
///
/// Returns `Some(payment_reference)` on a successful real payment, `None`
/// if the currency isn't SAP/TEND or the finance cluster call fails --
/// the caller falls back to the manual reference in either case.
fn settle_via_finance(trade: &Trade) -> Option<String> {
    if !currency_has_real_settlement_rail(&trade.currency) {
        return None;
    }
    let micro_amount = whole_units_to_micro(trade.total_price);

    #[derive(Serialize, Debug)]
    struct SendPaymentPayload {
        from_did: String,
        to_did: String,
        amount: u64,
        currency: String,
        payment_type: String,
        memo: Option<String>,
    }

    match call(
        CallTargetCell::OtherRole("finance".into()),
        ZomeName::from("payments"),
        FunctionName::from("send_payment"),
        None,
        SendPaymentPayload {
            from_did: trade.buyer_did.clone(),
            to_did: trade.seller_did.clone(),
            amount: micro_amount,
            currency: trade.currency.clone(),
            payment_type: "Direct".to_string(),
            memo: Some(format!("grid trade settlement: {}", trade.id)),
        },
    ) {
        Ok(ZomeCallResponse::Ok(result)) => result
            .decode::<Record>()
            .ok()
            .map(|record| format!("payments:{}", record.action_address())),
        _ => None,
    }
}

#[hdk_extern]
pub fn settle_trade(input: SettleTradeInput) -> ExternResult<Record> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::Trade,
        )?))
        .include_entries(true);
    for record in query(filter)? {
        if let Some(trade) = record.entry().to_app_option::<Trade>().ok().flatten() {
            if trade.id == input.trade_id {
                let payment_reference =
                    settle_via_finance(&trade).unwrap_or(input.payment_reference);
                let settled_trade = Trade {
                    settled: true,
                    payment_reference: Some(payment_reference),
                    ..trade
                };
                let action_hash = update_entry(
                    record.action_address().clone(),
                    &EntryTypes::Trade(settled_trade),
                )?;
                return get(action_hash, GetOptions::default())?
                    .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest("Trade not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SettleTradeInput {
    pub trade_id: String,
    pub payment_reference: String,
}

/// Get producer's energy production records
///
/// OPTIMIZED: Uses batch query to avoid N+1 pattern
#[hdk_extern]
pub fn get_producer_production(producer_did: String) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&producer_did)?, LinkTypes::ProducerToProduction)?,
        GetStrategy::default(),
    )?;
    // FIXED N+1: Use batch fetch instead of individual get() calls
    links_to_records(links)
}

/// Get seller's trade offers
///
/// OPTIMIZED: Uses batch query to avoid N+1 pattern
#[hdk_extern]
pub fn get_seller_offers(seller_did: String) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&seller_did)?, LinkTypes::SellerToOffers)?,
        GetStrategy::default(),
    )?;
    // FIXED N+1: Use batch fetch instead of individual get() calls
    links_to_records(links)
}

/// Get buyer's trade history
///
/// OPTIMIZED: Uses batch query to avoid N+1 pattern
#[hdk_extern]
pub fn get_buyer_trades(buyer_did: String) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&buyer_did)?, LinkTypes::BuyerToTrades)?,
        GetStrategy::default(),
    )?;
    // FIXED N+1: Use batch fetch instead of individual get() calls
    links_to_records(links)
}

/// Get trades for an offer
///
/// OPTIMIZED: Uses batch query to avoid N+1 pattern
#[hdk_extern]
pub fn get_offer_trades(offer_id: String) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&offer_id)?, LinkTypes::OfferToTrades)?,
        GetStrategy::default(),
    )?;
    // FIXED N+1: Use batch fetch instead of individual get() calls
    links_to_records(links)
}

/// Wire-compatible mirror of `mycelix-identity`'s `trust_credential::TrustTier`
/// (a local copy, not a shared-crate import, matching the same
/// cross-hApp-call convention used by `settle_via_finance` -- separately
/// deployable hApps don't share Rust types across a `call()` boundary,
/// only the serde wire shape). Variant names and their absence of
/// associated data must stay in sync with the real enum in
/// `mycelix-identity/zomes/trust_credential/integrity/src/lib.rs`.
#[derive(Deserialize, Debug, Clone, Copy, PartialEq)]
enum TrustTier {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

/// Parse a `did:mycelix:<agent_pub_key>` DID into its `AgentPubKey`, using
/// the exact same convention `mycelix-identity`'s own `did_registry::resolve_did`
/// uses (`did_registry/coordinator/src/lib.rs`) -- so this can be done
/// locally, no cross-cluster call needed for this step. Returns `None` for
/// any DID that isn't a real, well-formed mycelix DID (including this
/// crate's own test-data placeholders like "did:mycelix:verifier1").
fn agent_pub_key_from_did(did: &str) -> Option<AgentPubKey> {
    let agent_str = did.strip_prefix("did:mycelix:")?;
    AgentPubKey::try_from(agent_str).ok()
}

/// Minimum trust tier required to verify energy-production records.
/// "Standard" (trust_credential's own doc comment: score >= 0.4, "can vote
/// on major proposals") is a defensible, if tunable, policy choice: a
/// meter-verifier attesting real-world kWh production for a P2P energy
/// marketplace is a comparable trust responsibility to that tier's
/// existing bar, not a rubber stamp (Observer/Basic) but also not
/// requiring full governance rights (Guardian).
fn tier_meets_verification_threshold(tier: TrustTier) -> bool {
    matches!(
        tier,
        TrustTier::Standard | TrustTier::Elevated | TrustTier::Guardian
    )
}

/// Look up a verifier's trust tier from the identity cluster's
/// `trust_credential` zome (the real, DNA-registered credentialing zome --
/// `web_of_trust`'s `get_trust_score` exists as source but is NOT wired
/// into `mycelix-identity`'s actual `dna.yaml`, so it can't be called
/// cross-cluster today). Returns `None` if the DID doesn't parse or the
/// cross-cluster call fails.
fn verifier_trust_tier(verifier_did: &str) -> Option<TrustTier> {
    let agent = agent_pub_key_from_did(verifier_did)?;
    match call(
        CallTargetCell::OtherRole("identity".into()),
        ZomeName::from("trust_credential"),
        FunctionName::from("get_agent_trust_level"),
        None,
        agent,
    ) {
        Ok(ZomeCallResponse::Ok(result)) => result.decode::<TrustTier>().ok(),
        _ => None,
    }
}

/// Verify energy production (by verifier)
///
/// Previously an unauthenticated bool flip: any caller passing ANY
/// `verifier_did` string could mark ANY production record verified,
/// regardless of whether that DID belonged to a real, trusted identity.
/// Now requires the claimed verifier to hold a real `trust_credential` at
/// or above `tier_meets_verification_threshold` -- trust-weighted
/// verification, not simple existence, per this repo's own DHT scalability
/// rules (see feedback_dht_scalability_traps.md's "Translation Verification"
/// section).
#[hdk_extern]
pub fn verify_production(input: VerifyProductionInput) -> ExternResult<Record> {
    let trust_tier = verifier_trust_tier(&input.verifier_did);
    if !trust_tier.is_some_and(tier_meets_verification_threshold) {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "Verifier {} does not hold a trust credential meeting the tier required \
             to verify production records (tier: {:?})",
            input.verifier_did, trust_tier
        ))));
    }

    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::EnergyProduction,
        )?))
        .include_entries(true);

    for record in query(filter)? {
        if let Some(production) = record
            .entry()
            .to_app_option::<EnergyProduction>()
            .ok()
            .flatten()
        {
            if production.id == input.production_id {
                let verified = EnergyProduction {
                    verified: true,
                    ..production
                };
                let action_hash = update_entry(
                    record.action_address().clone(),
                    &EntryTypes::EnergyProduction(verified),
                )?;
                return get(action_hash, GetOptions::default())?
                    .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Production record not found".into()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VerifyProductionInput {
    pub production_id: String,
    pub verifier_did: String,
}

/// Wire-compatible partial mirror of `projects::EnergyProject` (a different
/// Rust crate -- `grid` and `projects` are separate zome crates within the
/// same energy hApp, and even same-hApp cross-zome calls go through
/// serde/ExternIO, not shared Rust types). Only `project_type` is needed
/// here. Holochain's entry serialization uses MessagePack struct-as-map
/// (`with_struct_map()` in `holochain_serialized_bytes`), so a partial
/// struct safely ignores the real type's many other fields rather than
/// requiring an exact field-for-field mirror (map-based formats tolerate
/// unknown keys; this would NOT be safe for a positional/array-based format).
#[derive(Debug, Serialize, Deserialize, SerializedBytes)]
struct EnergyProjectTypeOnly {
    project_type: ProjectType,
}

/// Wire-compatible mirror of `projects`' own `ProjectType` enum. Variant
/// names and shapes (all fieldless except `Other`) must stay in sync with
/// `mycelix-energy/zomes/projects/integrity/src/lib.rs`.
#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
enum ProjectType {
    Solar,
    Wind,
    Hydro,
    Nuclear,
    Geothermal,
    BatteryStorage,
    PumpedHydro,
    Hydrogen,
    Biomass,
    Other(String),
}

/// Whether a project type represents genuine renewable generation eligible
/// for carbon-credit issuance. Nuclear is zero-carbon but deliberately
/// excluded -- it is not renewable. BatteryStorage is storage, not
/// generation. Hydrogen and Other are excluded because they don't carry an
/// unambiguous displaced-fossil-fuel story (grey/blue/green hydrogen have
/// very different real emissions profiles; "Other" is arbitrary).
fn project_type_is_renewable_eligible(project_type: &ProjectType) -> bool {
    matches!(
        project_type,
        ProjectType::Solar
            | ProjectType::Wind
            | ProjectType::Hydro
            | ProjectType::Geothermal
            | ProjectType::PumpedHydro
            | ProjectType::Biomass
    )
}

/// Illustrative default grid emission factor used to estimate avoided
/// emissions from verified renewable generation, kg CO2e per kWh. This is a
/// clearly-labeled DEFAULT approximating commonly-cited global grid
/// averages, NOT a precise regional figure -- real deployments should use
/// the actual emission factor for the project's interconnection region
/// (e.g. eGRID subregion data, already vendored as a dataset in
/// sol-atlas/terra-atlas-mvp/data/egrid2022.xlsx per this repo's own reuse
/// map for this plan).
const ILLUSTRATIVE_GRID_EMISSION_FACTOR_KG_CO2_PER_KWH: f64 = 0.4;

/// Estimated tonnes of CO2e avoided by `amount_kwh` of verified renewable
/// generation, using `ILLUSTRATIVE_GRID_EMISSION_FACTOR_KG_CO2_PER_KWH`.
fn kwh_to_tonnes_co2e_avoided(amount_kwh: f64) -> f64 {
    amount_kwh * ILLUSTRATIVE_GRID_EMISSION_FACTOR_KG_CO2_PER_KWH / 1000.0
}

/// Approximate calendar year for a production timestamp, for carbon-credit
/// vintage labeling. Uses a 365.25-day average year length rather than
/// exact calendar/leap-year arithmetic -- adequate for a REC-style vintage
/// label (which conventionally tolerates being off by one near a Dec
/// 31/Jan 1 boundary) but NOT a general-purpose date library replacement.
fn approximate_vintage_year(timestamp: Timestamp) -> u32 {
    const MICROS_PER_YEAR: f64 = 365.25 * 24.0 * 3600.0 * 1_000_000.0;
    let years_since_epoch = timestamp.as_micros() as f64 / MICROS_PER_YEAR;
    (1970.0 + years_since_epoch).floor() as u32
}

/// Issue a renewable-energy carbon credit for a verified `EnergyProduction`
/// record.
///
/// Deliberately a separate extern from `verify_production` rather than an
/// automatic side effect of verification: crediting is an economically
/// consequential, auditable action that callers should trigger explicitly,
/// not something that happens invisibly the moment a verifier signs off.
///
/// Requires: the production record is already verified (via the
/// credentialed path in `verify_production`), and its linked project's type
/// is renewable-eligible (`project_type_is_renewable_eligible`) -- checked
/// via a real local cross-zome call to `projects::get_project`, not
/// assumed. Issues the credit via a real cross-cluster call to
/// `climate::carbon::create_carbon_credit`.
#[hdk_extern]
pub fn issue_renewable_carbon_credit(
    input: IssueCarbonCreditInput,
) -> ExternResult<CarbonCreditIssuance> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::EnergyProduction,
        )?))
        .include_entries(true);

    let production = query(filter)?
        .into_iter()
        .find_map(|record| {
            record
                .entry()
                .to_app_option::<EnergyProduction>()
                .ok()
                .flatten()
                .filter(|p| p.id == input.production_id)
        })
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("Production record not found".into())))?;

    if !production.verified {
        return Ok(CarbonCreditIssuance {
            issued: false,
            reason: Some("Production record is not verified".into()),
            credit_id: None,
        });
    }

    let project_record = match call(
        CallTargetCell::Local,
        ZomeName::from("projects"),
        FunctionName::from("get_project"),
        None,
        production.project_id.clone(),
    ) {
        Ok(ZomeCallResponse::Ok(result)) => result.decode::<Option<Record>>().ok().flatten(),
        _ => None,
    };
    let project_type = project_record.and_then(|record| {
        record
            .entry()
            .to_app_option::<EnergyProjectTypeOnly>()
            .ok()
            .flatten()
            .map(|p| p.project_type)
    });
    let Some(project_type) = project_type else {
        return Ok(CarbonCreditIssuance {
            issued: false,
            reason: Some(format!(
                "Could not resolve project {} to determine renewable eligibility",
                production.project_id
            )),
            credit_id: None,
        });
    };
    if !project_type_is_renewable_eligible(&project_type) {
        return Ok(CarbonCreditIssuance {
            issued: false,
            reason: Some(format!(
                "Project type {project_type:?} is not renewable-eligible for carbon-credit issuance"
            )),
            credit_id: None,
        });
    }

    let tonnes_co2e = kwh_to_tonnes_co2e_avoided(production.amount_kwh);
    let vintage_year = approximate_vintage_year(production.timestamp);
    let credit_id = format!("credit:{}:{}", production.id, sys_time()?.as_micros());

    #[derive(Serialize, Debug)]
    struct CreateCreditPayload {
        id: String,
        project_id: String,
        vintage_year: u32,
        tonnes_co2e: f64,
        owner_did: String,
    }

    match call(
        CallTargetCell::OtherRole("climate".into()),
        ZomeName::from("carbon"),
        FunctionName::from("create_carbon_credit"),
        None,
        CreateCreditPayload {
            id: credit_id.clone(),
            project_id: production.project_id.clone(),
            vintage_year,
            tonnes_co2e,
            owner_did: production.producer_did.clone(),
        },
    ) {
        Ok(ZomeCallResponse::Ok(_)) => Ok(CarbonCreditIssuance {
            issued: true,
            reason: None,
            credit_id: Some(credit_id),
        }),
        Ok(other) => Ok(CarbonCreditIssuance {
            issued: false,
            reason: Some(format!("Climate cluster returned: {other:?}")),
            credit_id: None,
        }),
        Err(e) => Ok(CarbonCreditIssuance {
            issued: false,
            reason: Some(format!("Climate cluster unreachable: {e:?}")),
            credit_id: None,
        }),
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct IssueCarbonCreditInput {
    pub production_id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct CarbonCreditIssuance {
    pub issued: bool,
    pub reason: Option<String>,
    pub credit_id: Option<String>,
}

/// Cancel a trade offer (seller only, only if active)
#[hdk_extern]
pub fn cancel_offer(input: CancelOfferInput) -> ExternResult<Record> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::TradeOffer,
        )?))
        .include_entries(true);

    for record in query(filter)? {
        if let Some(offer) = record.entry().to_app_option::<TradeOffer>().ok().flatten() {
            if offer.id == input.offer_id {
                // Only seller can cancel
                if offer.seller_did != input.requester_did {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Only seller can cancel offer".into()
                    )));
                }

                // Can only cancel active offers
                if offer.status != OfferStatus::Active {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Can only cancel active offers".into()
                    )));
                }

                let cancelled = TradeOffer {
                    status: OfferStatus::Cancelled,
                    ..offer
                };
                let action_hash = update_entry(
                    record.action_address().clone(),
                    &EntryTypes::TradeOffer(cancelled),
                )?;
                return get(action_hash, GetOptions::default())?
                    .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest("Offer not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CancelOfferInput {
    pub offer_id: String,
    pub requester_did: String,
}

/// Get a specific trade by ID
#[hdk_extern]
pub fn get_trade(trade_id: String) -> ExternResult<Option<Record>> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::Trade,
        )?))
        .include_entries(true);

    for record in query(filter)? {
        if let Some(trade) = record.entry().to_app_option::<Trade>().ok().flatten() {
            if trade.id == trade_id {
                return Ok(Some(record));
            }
        }
    }
    Ok(None)
}

/// Get a specific offer by ID
#[hdk_extern]
pub fn get_offer(offer_id: String) -> ExternResult<Option<Record>> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::TradeOffer,
        )?))
        .include_entries(true);

    for record in query(filter)? {
        if let Some(offer) = record.entry().to_app_option::<TradeOffer>().ok().flatten() {
            if offer.id == offer_id {
                return Ok(Some(record));
            }
        }
    }
    Ok(None)
}

/// Get unsettled trades (for payment processing)
#[hdk_extern]
pub fn get_unsettled_trades(_: ()) -> ExternResult<Vec<Record>> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::Trade,
        )?))
        .include_entries(true);

    let mut trades = Vec::new();
    for record in query(filter)? {
        if let Some(trade) = record.entry().to_app_option::<Trade>().ok().flatten() {
            if !trade.settled {
                trades.push(record);
            }
        }
    }
    Ok(trades)
}

/// Get total production for a producer
#[hdk_extern]
pub fn get_producer_total_production(producer_did: String) -> ExternResult<ProducerStats> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::EnergyProduction,
        )?))
        .include_entries(true);

    let mut total_kwh = 0.0;
    let mut verified_kwh = 0.0;
    let mut record_count = 0;

    for record in query(filter)? {
        if let Some(production) = record
            .entry()
            .to_app_option::<EnergyProduction>()
            .ok()
            .flatten()
        {
            if production.producer_did == producer_did {
                total_kwh += production.amount_kwh;
                if production.verified {
                    verified_kwh += production.amount_kwh;
                }
                record_count += 1;
            }
        }
    }

    Ok(ProducerStats {
        producer_did,
        total_kwh,
        verified_kwh,
        record_count,
    })
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ProducerStats {
    pub producer_did: String,
    pub total_kwh: f64,
    pub verified_kwh: f64,
    pub record_count: u32,
}

/// Get per-agent thermodynamic yield score [0.0, 1.0].
///
/// Normalizes verified energy production: 30+ verified records = saturation.
/// Quality = verified_kwh / total_kwh (higher = more trustworthy production).
///
/// Used by the 8D Sovereign Profile (D1: Thermodynamic Yield).
#[hdk_extern]
pub fn get_agent_thermodynamic_score(producer_did: String) -> ExternResult<f64> {
    let stats = get_producer_total_production(producer_did)?;
    if stats.record_count == 0 {
        return Ok(0.0);
    }
    let saturation = (stats.record_count as f64 / 30.0).min(1.0);
    let quality = if stats.total_kwh > 0.0 {
        (stats.verified_kwh / stats.total_kwh).clamp(0.0, 1.0)
    } else {
        0.0
    };
    Ok((saturation * quality).clamp(0.0, 1.0))
}

/// Update offer price (seller only)
#[hdk_extern]
pub fn update_offer_price(input: UpdateOfferPriceInput) -> ExternResult<Record> {
    let filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::TradeOffer,
        )?))
        .include_entries(true);

    for record in query(filter)? {
        if let Some(offer) = record.entry().to_app_option::<TradeOffer>().ok().flatten() {
            if offer.id == input.offer_id {
                // Only seller can update
                if offer.seller_did != input.requester_did {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Only seller can update price".into()
                    )));
                }

                // Can only update active offers
                if offer.status != OfferStatus::Active
                    && offer.status != OfferStatus::PartiallyFilled
                {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Can only update active offers".into()
                    )));
                }

                let updated = TradeOffer {
                    price_per_kwh: input.new_price_per_kwh,
                    ..offer
                };
                let action_hash = update_entry(
                    record.action_address().clone(),
                    &EntryTypes::TradeOffer(updated),
                )?;
                return get(action_hash, GetOptions::default())?
                    .ok_or(wasm_error!(WasmErrorInner::Guest("Not found".into())));
            }
        }
    }
    Err(wasm_error!(WasmErrorInner::Guest("Offer not found".into())))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateOfferPriceInput {
    pub offer_id: String,
    pub requester_did: String,
    pub new_price_per_kwh: f64,
}

/// Get grid trading summary
#[hdk_extern]
pub fn get_grid_summary(_: ()) -> ExternResult<GridSummary> {
    let offer_filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::TradeOffer,
        )?))
        .include_entries(true);

    let mut active_offers = 0;
    let mut total_kwh_available = 0.0;

    for record in query(offer_filter)? {
        if let Some(offer) = record.entry().to_app_option::<TradeOffer>().ok().flatten() {
            if offer.status == OfferStatus::Active || offer.status == OfferStatus::PartiallyFilled {
                active_offers += 1;
                total_kwh_available += offer.amount_kwh;
            }
        }
    }

    let trade_filter = ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::Trade,
        )?))
        .include_entries(true);

    let mut total_trades = 0;
    let mut total_kwh_traded = 0.0;
    let mut total_value_traded = 0.0;

    for record in query(trade_filter)? {
        if let Some(trade) = record.entry().to_app_option::<Trade>().ok().flatten() {
            total_trades += 1;
            total_kwh_traded += trade.amount_kwh;
            total_value_traded += trade.total_price;
        }
    }

    Ok(GridSummary {
        active_offers,
        total_kwh_available,
        total_trades,
        total_kwh_traded,
        total_value_traded,
    })
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GridSummary {
    pub active_offers: u32,
    pub total_kwh_available: f64,
    pub total_trades: u32,
    pub total_kwh_traded: f64,
    pub total_value_traded: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // Input Struct Validation Tests
    // =========================================================================

    fn create_test_timestamp() -> Timestamp {
        Timestamp::from_micros(1704067200000000) // 2024-01-01 00:00:00 UTC
    }

    // =========================================================================
    // RecordProductionInput Tests
    // =========================================================================

    fn valid_record_production_input() -> RecordProductionInput {
        RecordProductionInput {
            producer_did: "did:mycelix:producer1".to_string(),
            project_id: "project:solar_farm_alpha".to_string(),
            amount_kwh: 250.5,
            period_hours: 24.0,
            meter_reading: Some(50000.0),
        }
    }

    #[test]
    fn test_record_production_input_valid() {
        let input = valid_record_production_input();
        assert!(input.producer_did.starts_with("did:"));
        assert!(input.amount_kwh > 0.0);
        assert!(input.period_hours > 0.0);
    }

    #[test]
    fn test_record_production_input_with_meter_reading() {
        let input = valid_record_production_input();
        assert!(input.meter_reading.is_some());
    }

    #[test]
    fn test_record_production_input_without_meter_reading() {
        let input = RecordProductionInput {
            meter_reading: None,
            ..valid_record_production_input()
        };
        assert!(input.meter_reading.is_none());
    }

    #[test]
    fn test_record_production_input_serialization() {
        let input = valid_record_production_input();
        let json = serde_json::to_string(&input);
        assert!(json.is_ok());
    }

    // =========================================================================
    // CreateOfferInput Tests
    // =========================================================================

    fn valid_create_offer_input() -> CreateOfferInput {
        CreateOfferInput {
            seller_did: "did:mycelix:seller1".to_string(),
            project_id: Some("project:wind_farm_beta".to_string()),
            amount_kwh: 1000.0,
            price_per_kwh: 0.12,
            currency: "USD".to_string(),
            available_from: create_test_timestamp(),
            available_until: Timestamp::from_micros(1704153600000000),
        }
    }

    #[test]
    fn test_create_offer_input_valid() {
        let input = valid_create_offer_input();
        assert!(input.seller_did.starts_with("did:"));
        assert!(input.amount_kwh > 0.0);
        assert!(input.price_per_kwh >= 0.0);
        assert!(!input.currency.is_empty());
    }

    #[test]
    fn test_create_offer_input_with_project() {
        let input = valid_create_offer_input();
        assert!(input.project_id.is_some());
    }

    #[test]
    fn test_create_offer_input_without_project() {
        let input = CreateOfferInput {
            project_id: None,
            ..valid_create_offer_input()
        };
        assert!(input.project_id.is_none());
    }

    #[test]
    fn test_create_offer_input_free_energy() {
        let input = CreateOfferInput {
            price_per_kwh: 0.0,
            ..valid_create_offer_input()
        };
        assert_eq!(input.price_per_kwh, 0.0);
    }

    #[test]
    fn test_create_offer_input_various_currencies() {
        let currencies = vec!["USD", "EUR", "GBP", "CHF", "JPY"];
        for currency in currencies {
            let input = CreateOfferInput {
                currency: currency.to_string(),
                ..valid_create_offer_input()
            };
            assert_eq!(input.currency, currency);
        }
    }

    #[test]
    fn test_create_offer_input_timestamp_range() {
        let input = valid_create_offer_input();
        // available_until should be after available_from
        assert!(input.available_until.as_micros() >= input.available_from.as_micros());
    }

    // =========================================================================
    // Offer Expiry Tests (offer_not_expired)
    // =========================================================================

    fn offer_with_until(available_until: Timestamp) -> TradeOffer {
        TradeOffer {
            id: "offer:did:mycelix:seller1:123456789".to_string(),
            seller_did: "did:mycelix:seller1".to_string(),
            project_id: None,
            amount_kwh: 100.0,
            price_per_kwh: 0.1,
            currency: "USD".to_string(),
            available_from: create_test_timestamp(),
            available_until,
            status: OfferStatus::Active,
            created: create_test_timestamp(),
        }
    }

    #[test]
    fn test_offer_not_expired_before_deadline() {
        let offer = offer_with_until(Timestamp::from_micros(2_000_000_000_000));
        let now = Timestamp::from_micros(1_000_000_000_000);
        assert!(offer_not_expired(&offer, now));
    }

    #[test]
    fn test_offer_not_expired_after_deadline() {
        let offer = offer_with_until(Timestamp::from_micros(1_000_000_000_000));
        let now = Timestamp::from_micros(2_000_000_000_000);
        assert!(!offer_not_expired(&offer, now));
    }

    #[test]
    fn test_offer_not_expired_exactly_at_deadline() {
        // available_until >= now is inclusive: the deadline instant itself
        // still counts as available.
        let t = Timestamp::from_micros(1_500_000_000_000);
        let offer = offer_with_until(t);
        assert!(offer_not_expired(&offer, t));
    }

    // =========================================================================
    // Renewable Carbon Credit Tests (project_type_is_renewable_eligible,
    // kwh_to_tonnes_co2e_avoided, approximate_vintage_year) -- pure helpers,
    // no HDK host calls.
    // =========================================================================

    #[test]
    fn test_renewable_project_types_are_eligible() {
        for pt in [
            ProjectType::Solar,
            ProjectType::Wind,
            ProjectType::Hydro,
            ProjectType::Geothermal,
            ProjectType::PumpedHydro,
            ProjectType::Biomass,
        ] {
            assert!(
                project_type_is_renewable_eligible(&pt),
                "{pt:?} should be renewable-eligible"
            );
        }
    }

    #[test]
    fn test_non_renewable_project_types_are_not_eligible() {
        // Nuclear is zero-carbon but not renewable -- deliberately excluded,
        // not an oversight.
        for pt in [
            ProjectType::Nuclear,
            ProjectType::BatteryStorage,
            ProjectType::Hydrogen,
            ProjectType::Other("coal".to_string()),
        ] {
            assert!(
                !project_type_is_renewable_eligible(&pt),
                "{pt:?} should NOT be renewable-eligible"
            );
        }
    }

    #[test]
    fn test_kwh_to_tonnes_co2e_avoided_matches_hand_computation() {
        // 1000 kWh * 0.4 kg/kWh = 400 kg = 0.4 tonnes
        assert!((kwh_to_tonnes_co2e_avoided(1000.0) - 0.4).abs() < 1e-9);
        assert_eq!(kwh_to_tonnes_co2e_avoided(0.0), 0.0);
    }

    #[test]
    fn test_approximate_vintage_year_at_epoch() {
        assert_eq!(approximate_vintage_year(Timestamp::from_micros(0)), 1970);
    }

    #[test]
    fn test_approximate_vintage_year_ten_years_later() {
        let ten_years_micros = (365.25 * 24.0 * 3600.0 * 1_000_000.0 * 10.0) as i64;
        assert_eq!(
            approximate_vintage_year(Timestamp::from_micros(ten_years_micros)),
            1980
        );
    }

    // =========================================================================
    // Sharded Offer Anchor Tests (day_bucket, offer_anchor_key,
    // offer_anchor_keys_for_lookback) -- pure helpers, no HDK host calls.
    // =========================================================================

    const MICROS_PER_DAY: i64 = 86_400_000_000;

    #[test]
    fn test_day_bucket_same_day_same_bucket() {
        let start_of_day_5 = Timestamp::from_micros(MICROS_PER_DAY * 5);
        let one_hour_into_day_5 = Timestamp::from_micros(MICROS_PER_DAY * 5 + 3_600_000_000);
        assert_eq!(day_bucket(start_of_day_5), day_bucket(one_hour_into_day_5));
        assert_eq!(day_bucket(start_of_day_5), 5);
    }

    #[test]
    fn test_day_bucket_different_days_different_buckets() {
        let day_5 = Timestamp::from_micros(MICROS_PER_DAY * 5);
        let day_6 = Timestamp::from_micros(MICROS_PER_DAY * 6);
        assert_ne!(day_bucket(day_5), day_bucket(day_6));
        assert_eq!(day_bucket(day_6), 6);
    }

    #[test]
    fn test_offer_anchor_key_format() {
        assert_eq!(offer_anchor_key(5), "active_energy_offers:5");
        assert_eq!(offer_anchor_key(0), "active_energy_offers:0");
    }

    #[test]
    fn test_lookback_window_is_bounded_and_spans_expected_range() {
        let now = Timestamp::from_micros(MICROS_PER_DAY * 100);
        let keys = offer_anchor_keys_for_lookback(now, 30);
        // Bounded: exactly lookback_days + 1 anchors, regardless of how
        // long the marketplace has existed -- the actual hotspot fix.
        assert_eq!(keys.len(), 31);
        assert_eq!(keys.first().unwrap(), &offer_anchor_key(70));
        assert_eq!(keys.last().unwrap(), &offer_anchor_key(100));
    }

    #[test]
    fn test_lookback_window_zero_days_returns_only_today() {
        let now = Timestamp::from_micros(MICROS_PER_DAY * 100);
        let keys = offer_anchor_keys_for_lookback(now, 0);
        assert_eq!(keys, vec![offer_anchor_key(100)]);
    }

    // =========================================================================
    // ExecuteTradeInput Tests
    // =========================================================================

    fn valid_execute_trade_input() -> ExecuteTradeInput {
        ExecuteTradeInput {
            offer_id: "offer:did:mycelix:seller1:123456789".to_string(),
            buyer_did: "did:mycelix:buyer1".to_string(),
            amount_kwh: 100.0,
        }
    }

    #[test]
    fn test_execute_trade_input_valid() {
        let input = valid_execute_trade_input();
        assert!(!input.offer_id.is_empty());
        assert!(input.buyer_did.starts_with("did:"));
        assert!(input.amount_kwh > 0.0);
    }

    #[test]
    fn test_execute_trade_input_partial_fill() {
        let input = ExecuteTradeInput {
            amount_kwh: 50.0, // Less than the full offer
            ..valid_execute_trade_input()
        };
        assert!(input.amount_kwh > 0.0);
    }

    #[test]
    fn test_execute_trade_input_full_fill() {
        let input = ExecuteTradeInput {
            amount_kwh: 1000.0, // Full offer amount
            ..valid_execute_trade_input()
        };
        assert!(input.amount_kwh > 0.0);
    }

    // =========================================================================
    // SettleTradeInput Tests
    // =========================================================================

    fn valid_settle_trade_input() -> SettleTradeInput {
        SettleTradeInput {
            trade_id: "trade:offer1:123456789".to_string(),
            payment_reference: "PAY-2024-00001".to_string(),
        }
    }

    #[test]
    fn test_settle_trade_input_valid() {
        let input = valid_settle_trade_input();
        assert!(!input.trade_id.is_empty());
        assert!(!input.payment_reference.is_empty());
    }

    // =========================================================================
    // Real-settlement gating tests (currency_has_real_settlement_rail,
    // whole_units_to_micro) -- pure helpers, no HDK host calls needed.
    // =========================================================================

    #[test]
    fn test_sap_and_tend_have_real_settlement_rail() {
        assert!(currency_has_real_settlement_rail("SAP"));
        assert!(currency_has_real_settlement_rail("TEND"));
    }

    #[test]
    fn test_fiat_like_currencies_have_no_real_settlement_rail() {
        for currency in ["USD", "EUR", "GBP", "SOLAR", ""] {
            assert!(
                !currency_has_real_settlement_rail(currency),
                "{currency} should not have a real settlement rail"
            );
        }
    }

    #[test]
    fn test_whole_units_to_micro_matches_finance_frontend_convention() {
        // Inverse of the display conversion used throughout
        // mycelix-finance's frontend (amount as f64 / 1_000_000.0).
        assert_eq!(whole_units_to_micro(1.0), 1_000_000);
        assert_eq!(whole_units_to_micro(0.5), 500_000);
        assert_eq!(whole_units_to_micro(123.45), 123_450_000);
    }

    #[test]
    fn test_whole_units_to_micro_rounds_rather_than_truncates() {
        // 0.1234565 * 1_000_000 = 123456.5 -- must round to the nearest
        // integer micro-unit, not silently truncate and lose a fraction.
        assert_eq!(whole_units_to_micro(0.1234565), 123_457);
    }

    #[test]
    fn test_settle_trade_input_various_payment_refs() {
        let payment_refs = vec![
            "WIRE-123456",
            "ACH-789012",
            "CRYPTO-0xabc123",
            "CHECK-00001",
            "INTERNAL-TRANSFER-001",
        ];
        for ref_id in payment_refs {
            let input = SettleTradeInput {
                payment_reference: ref_id.to_string(),
                ..valid_settle_trade_input()
            };
            assert!(!input.payment_reference.is_empty());
        }
    }

    // =========================================================================
    // VerifyProductionInput Tests
    // =========================================================================

    fn valid_verify_production_input() -> VerifyProductionInput {
        VerifyProductionInput {
            production_id: "prod:did:mycelix:producer1:123456789".to_string(),
            verifier_did: "did:mycelix:verifier1".to_string(),
        }
    }

    #[test]
    fn test_verify_production_input_valid() {
        let input = valid_verify_production_input();
        assert!(!input.production_id.is_empty());
        assert!(input.verifier_did.starts_with("did:"));
    }

    // =========================================================================
    // Credentialed Verifier Tests (agent_pub_key_from_did,
    // tier_meets_verification_threshold) -- pure helpers, no HDK host calls.
    // =========================================================================

    #[test]
    fn test_agent_pub_key_from_did_round_trips_a_real_agent_pub_key() {
        // Genuine round-trip, not a hardcoded magic string: construct a real
        // AgentPubKey, format it into a DID the same way trust_credential's
        // get_agent_trust_level does (format!("did:mycelix:{}", agent)), then
        // confirm parsing it back recovers the identical key.
        //
        // Must use from_raw_32 (which computes and appends the correct
        // 4-byte DHT-location checksum), not from_raw_36 (which stores 36
        // raw bytes as-is, including an arbitrary/incorrect location
        // segment) -- HoloHash's TryFrom<&str> re-validates that checksum
        // on decode, so a from_raw_36 key's Display output doesn't actually
        // round-trip back through parsing, only a from_raw_32 key's does.
        let agent = AgentPubKey::from_raw_32(vec![7u8; 32]);
        let did = format!("did:mycelix:{agent}");
        assert_eq!(agent_pub_key_from_did(&did), Some(agent));
    }

    #[test]
    fn test_agent_pub_key_from_did_rejects_missing_prefix() {
        assert_eq!(agent_pub_key_from_did("mycelix:verifier1"), None);
    }

    #[test]
    fn test_agent_pub_key_from_did_rejects_non_pubkey_suffix() {
        // This crate's own existing test data (valid_verify_production_input,
        // valid_create_offer_input, etc.) uses placeholder DIDs like
        // "did:mycelix:verifier1" -- not real encoded AgentPubKeys. Those
        // must fail to parse (and therefore fail credentialed verification),
        // which is the correct, intended behavior now.
        assert_eq!(agent_pub_key_from_did("did:mycelix:verifier1"), None);
    }

    #[test]
    fn test_tier_meets_verification_threshold() {
        assert!(!tier_meets_verification_threshold(TrustTier::Observer));
        assert!(!tier_meets_verification_threshold(TrustTier::Basic));
        assert!(tier_meets_verification_threshold(TrustTier::Standard));
        assert!(tier_meets_verification_threshold(TrustTier::Elevated));
        assert!(tier_meets_verification_threshold(TrustTier::Guardian));
    }

    // =========================================================================
    // CancelOfferInput Tests
    // =========================================================================

    fn valid_cancel_offer_input() -> CancelOfferInput {
        CancelOfferInput {
            offer_id: "offer:did:mycelix:seller1:123456789".to_string(),
            requester_did: "did:mycelix:seller1".to_string(),
        }
    }

    #[test]
    fn test_cancel_offer_input_valid() {
        let input = valid_cancel_offer_input();
        assert!(!input.offer_id.is_empty());
        assert!(input.requester_did.starts_with("did:"));
    }

    #[test]
    fn test_cancel_offer_input_seller_must_match() {
        let input = valid_cancel_offer_input();
        // The offer_id contains seller DID, requester should match
        assert!(input.offer_id.contains("seller1"));
        assert!(input.requester_did.contains("seller1"));
    }

    // =========================================================================
    // UpdateOfferPriceInput Tests
    // =========================================================================

    fn valid_update_offer_price_input() -> UpdateOfferPriceInput {
        UpdateOfferPriceInput {
            offer_id: "offer:did:mycelix:seller1:123456789".to_string(),
            requester_did: "did:mycelix:seller1".to_string(),
            new_price_per_kwh: 0.18,
        }
    }

    #[test]
    fn test_update_offer_price_input_valid() {
        let input = valid_update_offer_price_input();
        assert!(!input.offer_id.is_empty());
        assert!(input.requester_did.starts_with("did:"));
        assert!(input.new_price_per_kwh >= 0.0);
    }

    #[test]
    fn test_update_offer_price_input_price_increase() {
        let input = UpdateOfferPriceInput {
            new_price_per_kwh: 0.25,
            ..valid_update_offer_price_input()
        };
        assert!(input.new_price_per_kwh > 0.0);
    }

    #[test]
    fn test_update_offer_price_input_price_decrease() {
        let input = UpdateOfferPriceInput {
            new_price_per_kwh: 0.05,
            ..valid_update_offer_price_input()
        };
        assert!(input.new_price_per_kwh > 0.0);
    }

    #[test]
    fn test_update_offer_price_input_free_energy() {
        let input = UpdateOfferPriceInput {
            new_price_per_kwh: 0.0,
            ..valid_update_offer_price_input()
        };
        assert_eq!(input.new_price_per_kwh, 0.0);
    }

    // =========================================================================
    // ProducerStats Tests
    // =========================================================================

    #[test]
    fn test_producer_stats_creation() {
        let stats = ProducerStats {
            producer_did: "did:mycelix:producer1".to_string(),
            total_kwh: 50000.0,
            verified_kwh: 45000.0,
            record_count: 100,
        };
        assert!(stats.producer_did.starts_with("did:"));
        assert!(stats.total_kwh >= stats.verified_kwh);
    }

    #[test]
    fn test_producer_stats_no_verified() {
        let stats = ProducerStats {
            producer_did: "did:mycelix:producer1".to_string(),
            total_kwh: 10000.0,
            verified_kwh: 0.0,
            record_count: 50,
        };
        assert_eq!(stats.verified_kwh, 0.0);
        assert!(stats.record_count > 0);
    }

    #[test]
    fn test_producer_stats_all_verified() {
        let stats = ProducerStats {
            producer_did: "did:mycelix:producer1".to_string(),
            total_kwh: 25000.0,
            verified_kwh: 25000.0,
            record_count: 75,
        };
        assert_eq!(stats.total_kwh, stats.verified_kwh);
    }

    #[test]
    fn test_producer_stats_empty() {
        let stats = ProducerStats {
            producer_did: "did:mycelix:new_producer".to_string(),
            total_kwh: 0.0,
            verified_kwh: 0.0,
            record_count: 0,
        };
        assert_eq!(stats.record_count, 0);
    }

    // =========================================================================
    // GridSummary Tests
    // =========================================================================

    #[test]
    fn test_grid_summary_creation() {
        let summary = GridSummary {
            active_offers: 50,
            total_kwh_available: 100000.0,
            total_trades: 200,
            total_kwh_traded: 500000.0,
            total_value_traded: 75000.0,
        };
        assert!(summary.active_offers > 0);
        assert!(summary.total_kwh_available > 0.0);
    }

    #[test]
    fn test_grid_summary_empty_grid() {
        let summary = GridSummary {
            active_offers: 0,
            total_kwh_available: 0.0,
            total_trades: 0,
            total_kwh_traded: 0.0,
            total_value_traded: 0.0,
        };
        assert_eq!(summary.active_offers, 0);
        assert_eq!(summary.total_trades, 0);
    }

    #[test]
    fn test_grid_summary_value_consistency() {
        let summary = GridSummary {
            active_offers: 10,
            total_kwh_available: 5000.0,
            total_trades: 100,
            total_kwh_traded: 10000.0,
            total_value_traded: 1500.0,
        };
        // Average price should be reasonable
        if summary.total_kwh_traded > 0.0 {
            let avg_price = summary.total_value_traded / summary.total_kwh_traded;
            assert!(avg_price >= 0.0);
        }
    }

    #[test]
    fn test_grid_summary_high_volume() {
        let summary = GridSummary {
            active_offers: 10000,
            total_kwh_available: 50_000_000.0,
            total_trades: 1_000_000,
            total_kwh_traded: 100_000_000.0,
            total_value_traded: 15_000_000.0,
        };
        assert!(summary.active_offers > 0);
    }

    // =========================================================================
    // Business Logic Edge Cases
    // =========================================================================

    #[test]
    fn test_offer_time_window_same_time() {
        // Edge case: available_from == available_until (instant availability)
        let input = CreateOfferInput {
            available_from: create_test_timestamp(),
            available_until: create_test_timestamp(),
            ..valid_create_offer_input()
        };
        assert_eq!(input.available_from, input.available_until);
    }

    #[test]
    fn test_fractional_kwh_trade() {
        let input = ExecuteTradeInput {
            amount_kwh: 0.001, // 1 Wh
            ..valid_execute_trade_input()
        };
        assert!(input.amount_kwh > 0.0);
    }

    #[test]
    fn test_large_kwh_trade() {
        let input = ExecuteTradeInput {
            amount_kwh: 1_000_000.0, // 1 GWh
            ..valid_execute_trade_input()
        };
        assert!(input.amount_kwh > 0.0);
    }

    #[test]
    fn test_very_high_price() {
        let input = CreateOfferInput {
            price_per_kwh: 10000.0, // Very high price
            ..valid_create_offer_input()
        };
        assert!(input.price_per_kwh > 0.0);
    }

    #[test]
    fn test_different_did_formats() {
        let did_formats = vec![
            "did:mycelix:user1",
            "did:web:example.com:user1",
            "did:key:z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK",
            "did:ion:EiDyOQbbZAa3aiRzeCkV7LOx3SERjjH93EXoIM3UoN4oWg",
        ];
        for did in did_formats {
            let input = CreateOfferInput {
                seller_did: did.to_string(),
                ..valid_create_offer_input()
            };
            assert!(input.seller_did.starts_with("did:"));
        }
    }

    #[test]
    fn test_serialization_deserialization_roundtrip() {
        let input = valid_create_offer_input();
        let json = serde_json::to_string(&input).unwrap();
        let deserialized: CreateOfferInput = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.seller_did, input.seller_did);
        assert_eq!(deserialized.amount_kwh, input.amount_kwh);
    }

    #[test]
    fn test_execute_trade_input_zero_amount_edge() {
        // This should be invalid in actual validation, but struct allows it
        let input = ExecuteTradeInput {
            amount_kwh: 0.0,
            ..valid_execute_trade_input()
        };
        // Business logic should reject this
        assert!(input.amount_kwh == 0.0);
    }

    #[test]
    fn test_long_offer_id_format() {
        let input = ExecuteTradeInput {
            offer_id: format!("offer:did:mycelix:seller1:{}", "9".repeat(50)),
            ..valid_execute_trade_input()
        };
        assert!(!input.offer_id.is_empty());
    }
}
