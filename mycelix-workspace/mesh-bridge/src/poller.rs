//! Conductor Poller — polls for new entries and relays to mesh.
//!
//! Watches all 11 resilience domains by polling conductor via AppWebsocket:
//! TEND exchanges, emergency messages, food harvests, water alerts, hearth alerts,
//! knowledge claims, care circle updates, shelter updates, supply updates,
//! mutual aid offers, and price reports. Deduplicates by action hash.
//!
//! Safety-critical domains (TEND, emergency, food, water, hearth) poll every
//! cycle. Lower-priority domains poll every other cycle to reduce bandwidth.

use crate::serializer::{
    self, CareCircleRelay, EmergencyRelay, FoodRelay, HearthAlertRelay, KnowledgeClaimRelay,
    MutualAidRelay, PriceReportRelay, RelayPayload, RelayType, ShelterRelay, SupplyRelay,
    TendRelay, WaterAlertRelay,
};
use crate::transport::MeshTransport;
use crate::BridgeMetrics;
use anyhow::Result;
use holochain_client::{AgentSigner, AppWebsocket, ClientAgentSigner, ExternIO, ZomeCallTarget};
use std::collections::HashSet;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use tokio_util::sync::CancellationToken;

/// Maximum dedup cache size (action hashes we've already relayed).
const MAX_DEDUP_CACHE: usize = 10_000;

/// LoRa frame size limit (SX1276 max payload).
const LORA_MAX_FRAME: usize = 255;

/// Heartbeat interval: broadcast presence every N poll cycles.
const HEARTBEAT_EVERY_N_POLLS: u64 = 4;

/// Max entries to relay per domain per poll cycle (backpressure).
const MAX_ENTRIES_PER_DOMAIN: usize = 20;

/// Max consecutive conductor connection failures before backoff.
const MAX_CONSECUTIVE_FAILURES: u32 = 5;

/// Backoff multiplier per failure (capped at 5 minutes).
const BACKOFF_BASE_SECS: u64 = 5;
const BACKOFF_MAX_SECS: u64 = 300;

/// Run the poller loop: conductor → serializer → mesh transport.
///
/// Respects the `cancel` token for graceful shutdown — drains the current
/// poll cycle before exiting.
pub async fn run(
    conductor_url: &str,
    poll_interval_secs: u64,
    transport: Box<dyn MeshTransport>,
    cancel: CancellationToken,
    metrics: BridgeMetrics,
) -> Result<()> {
    tracing::info!("Poller starting, conductor={conductor_url}, interval={poll_interval_secs}s");

    let mut dedup: HashSet<String> = crate::dedup_cache::load_string_cache("poller-dedup.cache");
    let origin = get_origin_id();
    let mut poll_count: u64 = 0;
    let mut consecutive_failures: u32 = 0;
    let mut ws_cached: Option<AppWebsocket> = None;

    loop {
        if cancel.is_cancelled() {
            tracing::info!("Poller shutting down (cancel requested)");
            break;
        }

        poll_count += 1;
        metrics.poll_cycles.fetch_add(1, Ordering::Relaxed);

        // Broadcast heartbeat periodically
        if poll_count % HEARTBEAT_EVERY_N_POLLS == 0 {
            let heartbeat = RelayPayload::new(RelayType::Heartbeat, origin, Vec::new());
            let bytes = heartbeat.to_bytes();
            let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
            for frame in &frames {
                let _ = transport.send(frame).await;
            }
            tracing::debug!("Heartbeat broadcast (cycle {})", poll_count);
        }

        // Try to connect and poll
        match poll_once(conductor_url, &mut dedup, &origin, &*transport, &mut ws_cached, poll_count)
            .await
        {
            Ok(relayed) => {
                consecutive_failures = 0;
                if relayed > 0 {
                    metrics
                        .messages_sent
                        .fetch_add(relayed as u64, Ordering::Relaxed);
                    metrics.last_send_at.store(
                        std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_millis() as u64,
                        Ordering::Relaxed,
                    );
                    tracing::info!("Relayed {relayed} entries to mesh");
                }
            }
            Err(e) => {
                consecutive_failures += 1;
                metrics.connection_failures.fetch_add(1, Ordering::Relaxed);
                // Drop cached connection on failure so next cycle reconnects
                ws_cached = None;
                tracing::warn!(
                    "Poll failed (attempt {}/{}): {e}",
                    consecutive_failures,
                    MAX_CONSECUTIVE_FAILURES
                );
            }
        }

        // Trim dedup cache
        if dedup.len() > MAX_DEDUP_CACHE {
            dedup.clear();
            tracing::debug!("Dedup cache cleared (exceeded {MAX_DEDUP_CACHE})");
        }

        // Exponential backoff when conductor is consistently unreachable
        let sleep_secs = if consecutive_failures > 0 {
            let backoff = BACKOFF_BASE_SECS * (1u64 << consecutive_failures.min(6));
            let capped = backoff.min(BACKOFF_MAX_SECS);
            if consecutive_failures >= MAX_CONSECUTIVE_FAILURES {
                tracing::warn!(
                    "Conductor unreachable for {} cycles, backing off {}s",
                    consecutive_failures,
                    capped
                );
            }
            capped
        } else {
            poll_interval_secs
        };

        // Interruptible sleep — wake early on cancellation
        tokio::select! {
            _ = tokio::time::sleep(tokio::time::Duration::from_secs(sleep_secs)) => {}
            _ = cancel.cancelled() => {
                tracing::info!("Poller interrupted during sleep");
                break;
            }
        }
    }

    // Persist dedup cache for restart resilience
    if let Err(e) = crate::dedup_cache::save_string_cache(&dedup, "poller-dedup.cache") {
        tracing::warn!("Failed to save poller dedup cache: {e}");
    }

    // Drop cached connection cleanly
    drop(ws_cached);
    tracing::info!("Poller stopped.");
    Ok(())
}

/// Single poll cycle. Returns number of entries relayed.
///
/// Reuses a cached AppWebsocket connection when available. Reconnects
/// on first call or after a failure clears the cache.
///
/// Safety-critical domains (TEND, emergency, water, hearth, food) poll every
/// cycle. Lower-priority domains (knowledge, shelter, supplies, care circles,
/// mutual aid, price reports) poll only on even cycles to reduce bandwidth.
///
/// Requires MESH_APP_TOKEN env var for authentication.
async fn poll_once(
    conductor_url: &str,
    dedup: &mut HashSet<String>,
    origin: &[u8; 8],
    transport: &dyn MeshTransport,
    ws_cached: &mut Option<AppWebsocket>,
    poll_count: u64,
) -> Result<usize> {
    // Reuse cached connection or establish a new one
    let ws = match ws_cached.take() {
        Some(ws) => ws,
        None => {
            let token: Vec<u8> = std::env::var("MESH_APP_TOKEN")
                .unwrap_or_default()
                .into_bytes();
            let signer: Arc<dyn AgentSigner + Send + Sync> =
                Arc::new(ClientAgentSigner::default());
            tracing::debug!("Connecting to conductor at {conductor_url}");
            AppWebsocket::connect(conductor_url, token, signer).await?
        }
    };

    let mut relayed = 0;

    // Whether to poll lower-priority domains this cycle
    let poll_low_priority = poll_count % 2 == 0;

    // =========================================================================
    // 1. TEND exchanges — every cycle
    // =========================================================================
    let tend_input = ExternIO::encode(serde_json::json!({
        "dao_did": "roodepoort-resilience",
        "limit": 50
    }))?;

    let tend_response = ws
        .call_zome(
            ZomeCallTarget::RoleName("finance".to_string().into()),
            "tend".into(),
            "get_my_exchanges".into(),
            tend_input,
        )
        .await;

    if let Ok(result) = tend_response {
        if let Ok(exchanges) = result.decode::<Vec<serde_json::Value>>() {
            let mut domain_count = 0usize;
            for exchange in &exchanges {
                if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                    break;
                }
                let id = exchange["id"].as_str().unwrap_or("");
                if id.is_empty() || dedup.contains(id) {
                    continue;
                }

                let tend = TendRelay {
                    receiver_did: exchange["receiver_did"]
                        .as_str()
                        .unwrap_or("")
                        .to_string(),
                    hours: exchange["hours"].as_f64().unwrap_or(0.0) as f32,
                    service_description: exchange["service_description"]
                        .as_str()
                        .unwrap_or("")
                        .to_string(),
                    service_category: exchange["service_category"]
                        .as_str()
                        .unwrap_or("GeneralAssistance")
                        .to_string(),
                    dao_did: exchange["dao_did"]
                        .as_str()
                        .unwrap_or("roodepoort-resilience")
                        .to_string(),
                };

                let data = bincode::serialize(&tend)?;
                let payload = RelayPayload::new(RelayType::TendExchange, *origin, data);
                let bytes = payload.to_bytes();
                let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                for frame in &frames {
                    transport.send(frame).await?;
                }

                dedup.insert(id.to_string());
                relayed += 1;
                domain_count += 1;
            }
        }
    }

    // =========================================================================
    // 2. Emergency messages — every cycle (safety-critical)
    // =========================================================================
    let emergency_input = ExternIO::encode(())?;

    let emergency_response = ws
        .call_zome(
            ZomeCallTarget::RoleName("civic".to_string().into()),
            "emergency_comms".into(),
            "get_unsynced_messages".into(),
            emergency_input,
        )
        .await;

    if let Ok(result) = emergency_response {
        if let Ok(messages) = result.decode::<Vec<serde_json::Value>>() {
            let mut domain_count = 0usize;
            for msg in &messages {
                if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                    break;
                }
                let id = msg["id"].as_str().unwrap_or("");
                if id.is_empty() || dedup.contains(id) {
                    continue;
                }

                let emergency = EmergencyRelay {
                    channel_id: msg["channel_id"].as_str().unwrap_or("").to_string(),
                    content: msg["content"].as_str().unwrap_or("").to_string(),
                    priority: msg["priority"].as_str().unwrap_or("Routine").to_string(),
                };

                let data = bincode::serialize(&emergency)?;
                let payload = RelayPayload::new(RelayType::EmergencyMessage, *origin, data);
                let bytes = payload.to_bytes();
                let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                for frame in &frames {
                    transport.send(frame).await?;
                }

                dedup.insert(id.to_string());
                relayed += 1;
                domain_count += 1;
            }
        }
    }

    // =========================================================================
    // 3. Food harvests — every cycle
    // =========================================================================
    let food_input = ExternIO::encode(serde_json::json!({"limit": 50}))?;

    let food_response = ws
        .call_zome(
            ZomeCallTarget::RoleName("commons_care".to_string().into()),
            "food_production".into(),
            "get_recent_harvests".into(),
            food_input,
        )
        .await;

    if let Ok(result) = food_response {
        if let Ok(harvests) = result.decode::<Vec<serde_json::Value>>() {
            let mut domain_count = 0usize;
            for harvest in &harvests {
                if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                    break;
                }
                let id = harvest["crop_hash"]
                    .as_str()
                    .or_else(|| harvest["id"].as_str())
                    .unwrap_or("");
                if id.is_empty() || dedup.contains(id) {
                    continue;
                }

                let food = FoodRelay {
                    crop_hash: id.to_string(),
                    quantity_kg: harvest["quantity_kg"].as_f64().unwrap_or(0.0) as f32,
                    quality: harvest["quality"].as_str().unwrap_or("").to_string(),
                    notes: harvest["notes"]
                        .as_str()
                        .unwrap_or("relayed via mesh")
                        .to_string(),
                };

                let data = bincode::serialize(&food)?;
                let payload = RelayPayload::new(RelayType::FoodHarvest, *origin, data);
                let bytes = payload.to_bytes();
                let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                for frame in &frames {
                    transport.send(frame).await?;
                }

                dedup.insert(id.to_string());
                relayed += 1;
                domain_count += 1;
            }
        }
    }

    // =========================================================================
    // 4. Water alerts — every cycle (safety-critical)
    // =========================================================================
    let water_input = ExternIO::encode(())?;

    let water_response = ws
        .call_zome(
            ZomeCallTarget::RoleName("commons_care".to_string().into()),
            "water_purity".into(),
            "get_active_alerts".into(),
            water_input,
        )
        .await;

    if let Ok(result) = water_response {
        if let Ok(alerts) = result.decode::<Vec<serde_json::Value>>() {
            let mut domain_count = 0usize;
            for alert in &alerts {
                if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                    break;
                }
                let id = alert["id"].as_str().unwrap_or("");
                if id.is_empty() || dedup.contains(id) {
                    continue;
                }

                let contaminant = alert["contaminant"].as_str().unwrap_or("");
                let description = alert["description"]
                    .as_str()
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| {
                        let value = alert["value"].as_f64().unwrap_or(0.0);
                        format!("{contaminant} at {value}")
                    });

                let water = WaterAlertRelay {
                    system_id: alert["source_id"].as_str().unwrap_or("").to_string(),
                    alert_type: contaminant.to_string(),
                    severity: alert["severity"].as_str().unwrap_or("").to_string(),
                    description,
                };

                let data = bincode::serialize(&water)?;
                let payload = RelayPayload::new(RelayType::WaterAlert, *origin, data);
                let bytes = payload.to_bytes();
                let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                for frame in &frames {
                    transport.send(frame).await?;
                }

                dedup.insert(id.to_string());
                relayed += 1;
                domain_count += 1;
            }
        }
    }

    // =========================================================================
    // 5. Hearth alerts — every cycle (safety-critical)
    // =========================================================================
    let hearth_input = ExternIO::encode(())?;

    let hearth_response = ws
        .call_zome(
            ZomeCallTarget::RoleName("hearth".to_string().into()),
            "hearth_emergency".into(),
            "get_active_alerts".into(),
            hearth_input,
        )
        .await;

    if let Ok(result) = hearth_response {
        if let Ok(alerts) = result.decode::<Vec<serde_json::Value>>() {
            let mut domain_count = 0usize;
            for alert in &alerts {
                if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                    break;
                }
                let id = alert["hearth_id"]
                    .as_str()
                    .or_else(|| alert["id"].as_str())
                    .unwrap_or("");
                if id.is_empty() || dedup.contains(id) {
                    continue;
                }

                let hearth = HearthAlertRelay {
                    hearth_id: id.to_string(),
                    alert_type: alert["alert_type"].as_str().unwrap_or("").to_string(),
                    message: alert["message"].as_str().unwrap_or("").to_string(),
                };

                let data = bincode::serialize(&hearth)?;
                let payload = RelayPayload::new(RelayType::HearthAlert, *origin, data);
                let bytes = payload.to_bytes();
                let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                for frame in &frames {
                    transport.send(frame).await?;
                }

                dedup.insert(id.to_string());
                relayed += 1;
                domain_count += 1;
            }
        }
    }

    // =========================================================================
    // 6. Knowledge claims — low priority (every other cycle)
    // =========================================================================
    if poll_low_priority {
        let knowledge_input = ExternIO::encode(serde_json::json!({"limit": 50}))?;

        let knowledge_response = ws
            .call_zome(
                ZomeCallTarget::RoleName("knowledge".to_string().into()),
                "claims".into(),
                "get_recent_claims".into(),
                knowledge_input,
            )
            .await;

        if let Ok(result) = knowledge_response {
            if let Ok(claims) = result.decode::<Vec<serde_json::Value>>() {
                let mut domain_count = 0usize;
                for claim in &claims {
                    if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                        break;
                    }
                    let id = claim["id"].as_str().unwrap_or("");
                    if id.is_empty() || dedup.contains(id) {
                        continue;
                    }

                    let tags: Vec<String> = claim["tags"]
                        .as_array()
                        .map(|arr| {
                            arr.iter()
                                .filter_map(|v| v.as_str().map(|s| s.to_string()))
                                .collect()
                        })
                        .unwrap_or_default();

                    let knowledge = KnowledgeClaimRelay {
                        claim_text: claim["content"].as_str().unwrap_or("").to_string(),
                        tags,
                        empirical: 0,
                        normative: 0,
                        materiality: 0,
                    };

                    let data = bincode::serialize(&knowledge)?;
                    let payload = RelayPayload::new(RelayType::KnowledgeClaim, *origin, data);
                    let bytes = payload.to_bytes();
                    let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                    for frame in &frames {
                        transport.send(frame).await?;
                    }

                    dedup.insert(id.to_string());
                    relayed += 1;
                    domain_count += 1;
                }
            }
        }
    }

    // =========================================================================
    // 7. Care circle updates — low priority (every other cycle)
    // =========================================================================
    if poll_low_priority {
        let care_input = ExternIO::encode(serde_json::json!({"limit": 20}))?;

        let care_response = ws
            .call_zome(
                ZomeCallTarget::RoleName("commons_care".to_string().into()),
                "care_circles".into(),
                "get_recent_updates".into(),
                care_input,
            )
            .await;

        if let Ok(result) = care_response {
            if let Ok(updates) = result.decode::<Vec<serde_json::Value>>() {
                let mut domain_count = 0usize;
                for update in &updates {
                    if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                        break;
                    }
                    let id = update["circle_id"]
                        .as_str()
                        .or_else(|| update["id"].as_str())
                        .unwrap_or("");
                    if id.is_empty() || dedup.contains(id) {
                        continue;
                    }

                    let care = CareCircleRelay {
                        circle_id: id.to_string(),
                        update_type: update["update_type"]
                            .as_str()
                            .or_else(|| update["update"].as_str())
                            .unwrap_or("")
                            .to_string(),
                        details: update["details"].as_str().unwrap_or("").to_string(),
                    };

                    let data = bincode::serialize(&care)?;
                    let payload = RelayPayload::new(RelayType::CareCircleUpdate, *origin, data);
                    let bytes = payload.to_bytes();
                    let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                    for frame in &frames {
                        transport.send(frame).await?;
                    }

                    dedup.insert(id.to_string());
                    relayed += 1;
                    domain_count += 1;
                }
            }
        }
    }

    // =========================================================================
    // 8. Shelter updates — low priority (every other cycle)
    // =========================================================================
    if poll_low_priority {
        let shelter_input = ExternIO::encode(serde_json::json!({"limit": 20}))?;

        let shelter_response = ws
            .call_zome(
                ZomeCallTarget::RoleName("commons_care".to_string().into()),
                "housing_units".into(),
                "get_recently_updated".into(),
                shelter_input,
            )
            .await;

        if let Ok(result) = shelter_response {
            if let Ok(units) = result.decode::<Vec<serde_json::Value>>() {
                let mut domain_count = 0usize;
                for unit in &units {
                    if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                        break;
                    }
                    let id = unit["unit_id"]
                        .as_str()
                        .or_else(|| unit["id"].as_str())
                        .unwrap_or("");
                    if id.is_empty() || dedup.contains(id) {
                        continue;
                    }

                    let shelter = ShelterRelay {
                        unit_id: id.to_string(),
                        status: unit["status"].as_str().unwrap_or("").to_string(),
                        bedrooms: unit["bedrooms"].as_u64().unwrap_or(0) as u8,
                    };

                    let data = bincode::serialize(&shelter)?;
                    let payload = RelayPayload::new(RelayType::ShelterUpdate, *origin, data);
                    let bytes = payload.to_bytes();
                    let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                    for frame in &frames {
                        transport.send(frame).await?;
                    }

                    dedup.insert(id.to_string());
                    relayed += 1;
                    domain_count += 1;
                }
            }
        }
    }

    // =========================================================================
    // 9. Supply updates — low priority (every other cycle)
    // =========================================================================
    if poll_low_priority {
        let supply_input = ExternIO::encode(())?;

        let supply_response = ws
            .call_zome(
                ZomeCallTarget::RoleName("supplychain".to_string().into()),
                "inventory_coordinator".into(),
                "get_low_stock_items".into(),
                supply_input,
            )
            .await;

        if let Ok(result) = supply_response {
            if let Ok(items) = result.decode::<Vec<serde_json::Value>>() {
                let mut domain_count = 0usize;
                for item in &items {
                    if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                        break;
                    }
                    let id = item["item_id"]
                        .as_str()
                        .or_else(|| item["id"].as_str())
                        .unwrap_or("");
                    if id.is_empty() || dedup.contains(id) {
                        continue;
                    }

                    let supply = SupplyRelay {
                        item_id: id.to_string(),
                        item_name: item["item_name"]
                            .as_str()
                            .or_else(|| item["name"].as_str())
                            .unwrap_or("")
                            .to_string(),
                        quantity: item["total_stock"].as_f64().unwrap_or(0.0) as f32,
                        category: item["category"].as_str().unwrap_or("").to_string(),
                    };

                    let data = bincode::serialize(&supply)?;
                    let payload = RelayPayload::new(RelayType::SupplyUpdate, *origin, data);
                    let bytes = payload.to_bytes();
                    let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                    for frame in &frames {
                        transport.send(frame).await?;
                    }

                    dedup.insert(id.to_string());
                    relayed += 1;
                    domain_count += 1;
                }
            }
        }
    }

    // =========================================================================
    // 10. Mutual aid offers — low priority (every other cycle)
    // =========================================================================
    if poll_low_priority {
        let aid_input = ExternIO::encode(serde_json::json!({"limit": 30}))?;

        let aid_response = ws
            .call_zome(
                ZomeCallTarget::RoleName("commons_care".to_string().into()),
                "mutualaid_timebank".into(),
                "get_recent_offers".into(),
                aid_input,
            )
            .await;

        if let Ok(result) = aid_response {
            if let Ok(offers) = result.decode::<Vec<serde_json::Value>>() {
                let mut domain_count = 0usize;
                for offer in &offers {
                    if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                        break;
                    }
                    let id = offer["id"].as_str().unwrap_or("");
                    if id.is_empty() || dedup.contains(id) {
                        continue;
                    }

                    let aid = MutualAidRelay {
                        offer_type: offer["offer_type"]
                            .as_str()
                            .unwrap_or("Offer")
                            .to_string(),
                        title: offer["title"].as_str().unwrap_or("").to_string(),
                        description: offer["description"].as_str().unwrap_or("").to_string(),
                        category: offer["category"].as_str().unwrap_or("").to_string(),
                    };

                    let data = bincode::serialize(&aid)?;
                    let payload = RelayPayload::new(RelayType::MutualAidOffer, *origin, data);
                    let bytes = payload.to_bytes();
                    let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                    for frame in &frames {
                        transport.send(frame).await?;
                    }

                    dedup.insert(id.to_string());
                    relayed += 1;
                    domain_count += 1;
                }
            }
        }
    }

    // =========================================================================
    // 11. Price reports — low priority (every other cycle)
    // =========================================================================
    if poll_low_priority {
        let price_input = ExternIO::encode(serde_json::json!({"limit": 30}))?;

        let price_response = ws
            .call_zome(
                ZomeCallTarget::RoleName("finance".to_string().into()),
                "price_oracle".into(),
                "get_recent_reports".into(),
                price_input,
            )
            .await;

        if let Ok(result) = price_response {
            if let Ok(reports) = result.decode::<Vec<serde_json::Value>>() {
                let mut domain_count = 0usize;
                for report in &reports {
                    if domain_count >= MAX_ENTRIES_PER_DOMAIN {
                        break;
                    }
                    let id = report["id"].as_str().unwrap_or("");
                    if id.is_empty() || dedup.contains(id) {
                        continue;
                    }

                    let price = PriceReportRelay {
                        item_name: report["item_name"]
                            .as_str()
                            .or_else(|| report["item"].as_str())
                            .unwrap_or("")
                            .to_string(),
                        price_tend: report["price_tend"].as_f64().unwrap_or(0.0) as f32,
                        evidence: report["evidence"].as_str().unwrap_or("").to_string(),
                    };

                    let data = bincode::serialize(&price)?;
                    let payload = RelayPayload::new(RelayType::PriceReport, *origin, data);
                    let bytes = payload.to_bytes();
                    let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
                    for frame in &frames {
                        transport.send(frame).await?;
                    }

                    dedup.insert(id.to_string());
                    relayed += 1;
                    domain_count += 1;
                }
            }
        }
    }

    // Cache connection for reuse on next poll cycle
    *ws_cached = Some(ws);

    Ok(relayed)
}

/// Get an 8-byte origin ID from the hostname.
fn get_origin_id() -> [u8; 8] {
    let hostname = hostname::get()
        .map(|h| h.to_string_lossy().to_string())
        .unwrap_or_else(|_| "unknown".into());
    let hash = blake3::hash(hostname.as_bytes());
    let mut id = [0u8; 8];
    id.copy_from_slice(&hash.as_bytes()[..8]);
    id
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::LoopbackTransport;

    #[test]
    fn test_heartbeat_frame_creation() {
        let origin = [1, 2, 3, 4, 5, 6, 7, 8];
        let heartbeat = RelayPayload::new(RelayType::Heartbeat, origin, Vec::new());
        assert_eq!(heartbeat.relay_type, RelayType::Heartbeat);
        assert_eq!(heartbeat.origin, origin);
        assert!(heartbeat.data.is_empty());

        // Heartbeat should fit in a single LoRa frame
        let bytes = heartbeat.to_bytes();
        let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn test_heartbeat_interval() {
        // Heartbeat fires on cycles divisible by HEARTBEAT_EVERY_N_POLLS
        for cycle in 1..=20u64 {
            let should_fire = cycle % HEARTBEAT_EVERY_N_POLLS == 0;
            if cycle == 4 || cycle == 8 || cycle == 12 || cycle == 16 || cycle == 20 {
                assert!(should_fire, "cycle {cycle} should fire heartbeat");
            } else {
                assert!(!should_fire, "cycle {cycle} should NOT fire heartbeat");
            }
        }
    }

    #[test]
    fn test_backoff_calculation() {
        // 0 failures → normal interval
        let failures: u32 = 0;
        assert_eq!(failures, 0);

        // 1 failure → 5 * 2^1 = 10s
        let backoff = BACKOFF_BASE_SECS * (1u64 << 1u32.min(6));
        assert_eq!(backoff.min(BACKOFF_MAX_SECS), 10);

        // 3 failures → 5 * 2^3 = 40s
        let backoff = BACKOFF_BASE_SECS * (1u64 << 3u32.min(6));
        assert_eq!(backoff.min(BACKOFF_MAX_SECS), 40);

        // 6 failures → 5 * 2^6 = 320 → capped at 300s
        let backoff = BACKOFF_BASE_SECS * (1u64 << 6u32.min(6));
        assert_eq!(backoff.min(BACKOFF_MAX_SECS), BACKOFF_MAX_SECS);

        // 10 failures → still capped at 6 bits → 320 → 300s
        let backoff = BACKOFF_BASE_SECS * (1u64 << 10u32.min(6));
        assert_eq!(backoff.min(BACKOFF_MAX_SECS), BACKOFF_MAX_SECS);
    }

    #[test]
    fn test_dedup_cache_overflow() {
        let mut dedup: HashSet<String> = HashSet::new();
        // Fill past MAX_DEDUP_CACHE
        for i in 0..=MAX_DEDUP_CACHE {
            dedup.insert(format!("hash-{i}"));
        }
        assert!(dedup.len() > MAX_DEDUP_CACHE);

        // Simulates the cache clear logic from run()
        if dedup.len() > MAX_DEDUP_CACHE {
            dedup.clear();
        }
        assert_eq!(dedup.len(), 0);
    }

    #[test]
    fn test_origin_id_deterministic() {
        let id1 = get_origin_id();
        let id2 = get_origin_id();
        assert_eq!(id1, id2, "origin ID should be deterministic for same hostname");
        assert_ne!(id1, [0u8; 8], "origin ID should not be all zeros");
    }

    #[tokio::test]
    async fn test_relay_pipeline() {
        let transport = LoopbackTransport::new();
        let origin = [1, 2, 3, 4, 5, 6, 7, 8];

        // Create a TEND relay
        let tend = TendRelay {
            receiver_did: "bob.did".into(),
            hours: 1.0,
            service_description: "Fixed tap".into(),
            service_category: "Maintenance".into(),
            dao_did: "roodepoort".into(),
        };
        let data = bincode::serialize(&tend).unwrap();
        let payload = RelayPayload::new(RelayType::TendExchange, origin, data);
        let bytes = payload.to_bytes();

        // Fragment and send
        let frames = serializer::fragment(&bytes, LORA_MAX_FRAME);
        for frame in &frames {
            transport.send(frame).await.unwrap();
        }

        // Receive and reassemble
        let mut received_frames = Vec::new();
        while let Ok(Some(frame)) = transport.recv(100).await {
            received_frames.push(frame);
        }

        let reassembled = serializer::reassemble(&received_frames).unwrap();
        let decoded = RelayPayload::from_bytes(&reassembled).unwrap();
        assert_eq!(decoded.relay_type, RelayType::TendExchange);

        let tend_decoded: TendRelay = bincode::deserialize(&decoded.data).unwrap();
        assert_eq!(tend_decoded.receiver_did, "bob.did");
    }
}
