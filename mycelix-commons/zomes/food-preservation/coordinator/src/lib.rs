// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Food Preservation Coordinator Zome
//! Business logic for preservation batches, methods, and storage management.

use food_preservation_integrity::*;
use hdk::prelude::*;
use mycelix_bridge_common::{civic_requirement_basic, civic_requirement_proposal};
use mycelix_zome_helpers as _;
use mycelix_zome_helpers::records_from_links;
use std::collections::BTreeSet;

fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    let anchor = Anchor(anchor_str.to_string());
    hash_entry(&EntryTypes::Anchor(anchor))
}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ContainerLifecycleSummary {
    pub container_hash: ActionHash,
    /// Derived from validated Filled events. This is never stored as a counter.
    pub derived_fill_cycle_count: u64,
    pub latest_event_hash: Option<ActionHash>,
    pub latest_sequence_number: Option<u64>,
    pub latest_condition: Option<ContainerCondition>,
}

fn container_history_records(container_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(container_hash, LinkTypes::ContainerToCycleEvents)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

/// Load and verify the complete linked history. A duplicated sequence,
/// missing link, fork or broken parent pointer returns an error instead of
/// guessing which branch is authoritative.
fn validate_container_event_chain(
    container_hash: &ActionHash,
    mut events: Vec<(ActionHash, ContainerCycleEvent)>,
) -> Result<Vec<(ActionHash, ContainerCycleEvent)>, String> {
    events.sort_by_key(|(_, event)| event.sequence_number);
    let mut expected_parent: Option<ActionHash> = None;
    let mut seen_sequences = BTreeSet::new();
    for (index, (action_hash, event)) in events.iter().enumerate() {
        if &event.container_hash != container_hash {
            return Err("Container history contains an event for another container".into());
        }
        if !seen_sequences.insert(event.sequence_number) {
            return Err("Container lifecycle conflict: duplicate sequence number; reconciliation required".into());
        }
        if event.sequence_number != index as u64 {
            return Err("Container lifecycle conflict: sequence gap; reconciliation required".into());
        }
        if event.previous_event_hash != expected_parent {
            return Err("Container lifecycle conflict: event parent does not match the unique preceding event".into());
        }
        expected_parent = Some(action_hash.clone());
    }
    Ok(events)
}

fn verified_container_history(
    container_hash: ActionHash,
) -> ExternResult<Vec<(ActionHash, ContainerCycleEvent)>> {
    let records = container_history_records(container_hash.clone())?;
    let mut events = Vec::with_capacity(records.len());
    for record in records {
        let action_hash = record.action_address().clone();
        let event: ContainerCycleEvent = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Container history link points to a non-event record".into()
            )))?;
        events.push((action_hash, event));
    }
    validate_container_event_chain(&container_hash, events)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason)))
}

fn derived_fill_cycle_count(events: &[(ActionHash, ContainerCycleEvent)]) -> u64 {
    events
        .iter()
        .filter(|(_, event)| event.event_kind == ContainerEventKind::Filled)
        .count() as u64
}

/// Register a reusable container in the existing food-preservation registry.
#[hdk_extern]
pub fn register_reusable_container(container: ReusableContainer) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "register_reusable_container",
    )?;
    let agent = agent_info()?.agent_initial_pubkey;
    if container.steward != agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the declared container steward may register the container".into()
        )));
    }
    for record in get_all_reusable_containers(())? {
        let existing: ReusableContainer = record
            .entry()
            .to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Container registry link points to an invalid entry".into()
            )))?;
        if existing.id == container.id {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Reusable container ID is already registered".into()
            )));
        }
    }
    let action_hash = create_entry(&EntryTypes::ReusableContainer(container))?;
    create_entry(&EntryTypes::Anchor(Anchor("all_reusable_containers".into())))?;
    create_link(
        anchor_hash("all_reusable_containers")?,
        action_hash.clone(),
        LinkTypes::AllReusableContainers,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created reusable container".into()
    )))
}

#[hdk_extern]
pub fn get_reusable_container(container_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(container_hash, GetOptions::default())
}

#[hdk_extern]
pub fn get_all_reusable_containers(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("all_reusable_containers")?, LinkTypes::AllReusableContainers)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

/// Append one lifecycle event. The event must extend the unique current head;
/// a fork or incomplete history blocks mutation until it is reconciled.
#[hdk_extern]
pub fn record_container_cycle_event(event: ContainerCycleEvent) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "record_container_cycle_event",
    )?;
    let container_record = get(event.container_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Container not found".into())))?;
    let container: ReusableContainer = container_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Container reference is not a ReusableContainer entry".into()
        )))?;
    let agent = agent_info()?.agent_initial_pubkey;
    if container.steward != agent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the container steward may append lifecycle events".into()
        )));
    }

    let history = verified_container_history(event.container_hash.clone())?;
    let expected_sequence = history.len() as u64;
    let expected_parent = history.last().map(|(hash, _)| hash.clone());
    if event.sequence_number != expected_sequence || event.previous_event_hash != expected_parent {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Container event does not extend the unique current head; reload and reconcile before retrying".into()
        )));
    }

    if event.event_kind == ContainerEventKind::Filled {
        if !container.compatible_product_refs.is_empty()
            && event.product_ref.as_deref().is_none_or(|product_ref| {
                !container.compatible_product_refs.iter().any(|allowed| allowed == product_ref)
            })
        {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Product is not listed as compatible with this container".into()
            )));
        }
        let fills = derived_fill_cycle_count(&history);
        if container.manufacturer_reuse_cycle_limit.is_some_and(|limit| fills >= u64::from(limit)) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Manufacturer reuse-cycle limit reached; container cannot be filled again".into()
            )));
        }
    }

    let action_hash = create_entry(&EntryTypes::ContainerCycleEvent(event.clone()))?;
    create_link(
        event.container_hash.clone(),
        action_hash.clone(),
        LinkTypes::ContainerToCycleEvents,
        event.sequence_number.to_be_bytes().to_vec().into(),
    )?;
    if let Some(previous_hash) = event.previous_event_hash {
        create_link(
            previous_hash,
            action_hash.clone(),
            LinkTypes::ContainerEventToNext,
            (),
        )?;
    }
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created container lifecycle event".into()
    )))
}

#[hdk_extern]
pub fn get_container_cycle_history(container_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let history = verified_container_history(container_hash)?;
    history
        .into_iter()
        .map(|(action_hash, _)| {
            get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
                "Container lifecycle event disappeared during history retrieval".into()
            )))
        })
        .collect()
}

#[hdk_extern]
pub fn get_container_lifecycle_summary(
    container_hash: ActionHash,
) -> ExternResult<ContainerLifecycleSummary> {
    let history = verified_container_history(container_hash.clone())?;
    let latest = history.last();
    Ok(ContainerLifecycleSummary {
        container_hash,
        derived_fill_cycle_count: derived_fill_cycle_count(&history),
        latest_event_hash: latest.map(|(hash, _)| hash.clone()),
        latest_sequence_number: latest.map(|(_, event)| event.sequence_number),
        latest_condition: latest.map(|(_, event)| event.condition.clone()),
    })
}


// ============================================================================
// BATCH MANAGEMENT
// ============================================================================

#[hdk_extern]
pub fn start_batch(batch: PreservationBatch) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "start_batch",
    )?;
    let agent = agent_info()?.agent_initial_pubkey;
    let action_hash = create_entry(&EntryTypes::PreservationBatch(batch.clone()))?;

    create_entry(&EntryTypes::Anchor(Anchor("all_batches".to_string())))?;
    create_link(
        anchor_hash("all_batches")?,
        action_hash.clone(),
        LinkTypes::AllBatches,
        (),
    )?;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToBatch, ())?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created batch".into()
    )))
}

#[hdk_extern]
pub fn complete_batch(batch_hash: ActionHash) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_proposal(),
        "complete_batch",
    )?;
    let record = get(batch_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Batch not found".into())))?;
    let mut batch: PreservationBatch = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid batch entry".into()
        )))?;

    batch.status = BatchStatus::Completed;
    let new_hash = update_entry(
        record.action_address().clone(),
        &EntryTypes::PreservationBatch(batch),
    )?;
    get(new_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find updated batch".into()
    )))
}

#[hdk_extern]
pub fn get_batch(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

#[hdk_extern]
pub fn get_agent_batches(_: ()) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToBatch)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

// ============================================================================
// METHOD REGISTRY
// ============================================================================

#[hdk_extern]
pub fn register_method(method: PreservationMethod) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "register_method",
    )?;
    let action_hash = create_entry(&EntryTypes::PreservationMethod(method))?;

    create_entry(&EntryTypes::Anchor(Anchor("all_methods".to_string())))?;
    create_link(
        anchor_hash("all_methods")?,
        action_hash.clone(),
        LinkTypes::AllMethods,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created method".into()
    )))
}

#[hdk_extern]
pub fn get_all_methods(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("all_methods")?, LinkTypes::AllMethods)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

// ============================================================================
// STORAGE MANAGEMENT
// ============================================================================

#[hdk_extern]
pub fn register_storage(storage: StorageUnit) -> ExternResult<Record> {
    mycelix_zome_helpers::require_civic(
        "commons_bridge",
        &civic_requirement_basic(),
        "register_storage",
    )?;
    let action_hash = create_entry(&EntryTypes::StorageUnit(storage))?;

    create_entry(&EntryTypes::Anchor(Anchor("all_storage".to_string())))?;
    create_link(
        anchor_hash("all_storage")?,
        action_hash.clone(),
        LinkTypes::AllStorage,
        (),
    )?;

    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not find created storage".into()
    )))
}

#[hdk_extern]
pub fn get_storage_inventory(_: ()) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(anchor_hash("all_storage")?, LinkTypes::AllStorage)?,
        GetStrategy::default(),
    )?;
    records_from_links(links)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    #[derive(Deserialize)]
    struct ContainerLifecycleFixtureBundle {
        container: ContainerFixtureIdentity,
        events: Vec<ContainerEventFixture>,
    }

    #[derive(Deserialize)]
    struct ContainerFixtureIdentity {
        id: String,
        material: ContainerMaterial,
        capacity_ml: u64,
        manufacturer_reuse_cycle_limit: Option<u32>,
    }

    #[derive(Deserialize)]
    struct ContainerEventFixture {
        container_hash: String,
        previous_event_hash: Option<String>,
        sequence_number: u64,
        event_kind: ContainerEventKind,
        occurred_at: u64,
        product_ref: Option<String>,
        product_lot_ref: Option<String>,
        location_ref: Option<String>,
        condition: ContainerCondition,
        sanitation: Option<ContainerSanitationObservation>,
        evidence: ContainerEvidenceRef,
    }

    #[test]
    fn synthetic_container_lifecycle_fixture_has_one_verified_linear_history() {
        let bundle: ContainerLifecycleFixtureBundle = serde_json::from_str(include_str!(
            "../../integrity/schemas/fixtures/container-lifecycle-v1.json"
        ))
        .expect("container lifecycle fixture bundle must deserialize");
        assert_eq!(bundle.container.id, "fixture:container-za-001");
        assert_eq!(bundle.container.capacity_ml, 1_000);
        assert_eq!(bundle.container.manufacturer_reuse_cycle_limit, Some(500));
        assert_eq!(bundle.container.material, ContainerMaterial::BorosilicateGlass);

        let container_hash = sample_container_hash();
        let mut records = Vec::new();
        let mut prior_hash: Option<ActionHash> = None;
        for (index, fixture) in bundle.events.into_iter().enumerate() {
            assert_eq!(fixture.container_hash, "uhCEkSyntheticContainerHash");
            assert_eq!(fixture.sequence_number, index as u64);
            assert_eq!(
                fixture.previous_event_hash,
                index.checked_sub(1).map(|previous| format!("uhCEkSyntheticEventHash{previous}"))
            );
            let action_hash = ActionHash::from_raw_36(vec![(index as u8).wrapping_add(20); 36]);
            let event = ContainerCycleEvent {
                container_hash: container_hash.clone(),
                previous_event_hash: prior_hash.clone(),
                sequence_number: fixture.sequence_number,
                event_kind: fixture.event_kind,
                occurred_at: fixture.occurred_at,
                product_ref: fixture.product_ref,
                product_lot_ref: fixture.product_lot_ref,
                preservation_batch_hash: None,
                location_ref: fixture.location_ref,
                condition: fixture.condition,
                sanitation: fixture.sanitation,
                evidence: fixture.evidence,
            };
            records.push((action_hash.clone(), event));
            prior_hash = Some(action_hash);
        }

        let checked = validate_container_event_chain(&container_hash, records).unwrap();
        assert_eq!(checked.len(), 12);
        assert_eq!(derived_fill_cycle_count(&checked), 1);
    }


    fn sample_container_hash() -> ActionHash {
        ActionHash::from_raw_36(vec![7u8; 36])
    }

    fn sample_cycle_event(
        sequence_number: u64,
        previous_event_hash: Option<ActionHash>,
        event_kind: ContainerEventKind,
    ) -> (ActionHash, ContainerCycleEvent) {
        let container_hash = sample_container_hash();
        let hash = ActionHash::from_raw_36(vec![(sequence_number as u8).wrapping_add(20); 36]);
        let event = ContainerCycleEvent {
            container_hash,
            previous_event_hash,
            sequence_number,
            event_kind,
            occurred_at: 1_700_000_000 + sequence_number,
            product_ref: None,
            product_lot_ref: None,
            preservation_batch_hash: None,
            location_ref: None,
            condition: ContainerCondition::Unknown,
            sanitation: None,
            evidence: ContainerEvidenceRef {
                source_ref: "fixture:event".into(),
                method_ref: None,
                calibration_ref: None,
                evidence_class: ContainerEvidenceClass::Declared,
            },
        };
        (hash, event)
    }


    #[test]
    fn container_history_accepts_one_linear_chain() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h1, e1) = sample_cycle_event(1, Some(h0.clone()), ContainerEventKind::Sanitized);
        let (h2, e2) = sample_cycle_event(2, Some(h1.clone()), ContainerEventKind::Inspected);
        let checked = validate_container_event_chain(&sample_container_hash(), vec![
            (h2, e2),
            (h0, e0),
            (h1, e1),
        ]).unwrap();
        assert_eq!(checked.len(), 3);
        assert_eq!(checked[2].1.sequence_number, 2);
    }

    #[test]
    fn container_history_rejects_duplicate_sequence_fork() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h1a, e1a) = sample_cycle_event(1, Some(h0.clone()), ContainerEventKind::Sanitized);
        let (h1b, e1b) = sample_cycle_event(1, Some(h0), ContainerEventKind::Sanitized);
        let error = validate_container_event_chain(&sample_container_hash(), vec![
            (h1a, e1a),
            (h1b, e1b),
            (ActionHash::from_raw_36(vec![1; 36]), e0),
        ]).unwrap_err();
        assert!(error.contains("duplicate sequence"));
    }

    #[test]
    fn container_history_rejects_gaps_and_wrong_parent() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h2, e2) = sample_cycle_event(2, Some(h0.clone()), ContainerEventKind::Inspected);
        assert!(validate_container_event_chain(&sample_container_hash(), vec![
            (h0.clone(), e0.clone()),
            (h2, e2),
        ]).unwrap_err().contains("sequence gap"));

        let (wrong_parent_hash, wrong_parent_event) = sample_cycle_event(
            1,
            Some(ActionHash::from_raw_36(vec![99; 36])),
            ContainerEventKind::Sanitized,
        );
        assert!(validate_container_event_chain(&sample_container_hash(), vec![
            (h0, e0),
            (wrong_parent_hash, wrong_parent_event),
        ]).unwrap_err().contains("parent"));
    }

    #[test]
    fn reuse_count_is_derived_only_from_fill_events() {
        let (h0, e0) = sample_cycle_event(0, None, ContainerEventKind::Washed);
        let (h1, e1) = sample_cycle_event(1, Some(h0.clone()), ContainerEventKind::Filled);
        let (h2, e2) = sample_cycle_event(2, Some(h1.clone()), ContainerEventKind::Emptied);
        let (h3, e3) = sample_cycle_event(3, Some(h2.clone()), ContainerEventKind::Filled);
        let history = validate_container_event_chain(&sample_container_hash(), vec![
            (h0,e0),(h1,e1),(h2,e2),(h3,e3)
        ]).unwrap();
        assert_eq!(derived_fill_cycle_count(&history), 2);
    }


    // ========================================================================
    // BatchStatus enum serde tests
    // ========================================================================

    #[test]
    fn batch_status_all_variants_serde() {
        let variants = vec![
            BatchStatus::InProgress,
            BatchStatus::Completed,
            BatchStatus::Failed,
            BatchStatus::Consumed,
        ];
        for variant in variants {
            let json = serde_json::to_string(&variant).unwrap();
            let decoded: BatchStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded, variant);
        }
    }

    // ========================================================================
    // SkillLevel enum serde tests
    // ========================================================================

    #[test]
    fn skill_level_all_variants_serde() {
        let variants = vec![
            SkillLevel::Beginner,
            SkillLevel::Intermediate,
            SkillLevel::Advanced,
        ];
        for variant in variants {
            let json = serde_json::to_string(&variant).unwrap();
            let decoded: SkillLevel = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded, variant);
        }
    }

    // ========================================================================
    // StorageType enum serde tests
    // ========================================================================

    #[test]
    fn storage_type_all_variants_serde() {
        let variants = vec![
            StorageType::RootCellar,
            StorageType::Cellar,
            StorageType::Freezer,
            StorageType::Dehydrator,
            StorageType::Fermenter,
            StorageType::Pantry,
            StorageType::Composter,
        ];
        for variant in variants {
            let json = serde_json::to_string(&variant).unwrap();
            let decoded: StorageType = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded, variant);
        }
    }

    // ========================================================================
    // PreservationBatch serde roundtrip tests
    // ========================================================================

    #[test]
    fn preservation_batch_serde_roundtrip() {
        let batch = PreservationBatch {
            id: "batch-1".to_string(),
            source_crop_hash: None,
            method: "Lacto-fermentation".to_string(),
            quantity_kg: 5.0,
            started_at: 1700000000,
            expected_ready: 1701000000,
            status: BatchStatus::InProgress,
            notes: Some("First fermentation attempt".to_string()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, "batch-1");
        assert_eq!(decoded.method, "Lacto-fermentation");
        assert_eq!(decoded.quantity_kg, 5.0);
        assert_eq!(decoded.status, BatchStatus::InProgress);
        assert_eq!(
            decoded.notes,
            Some("First fermentation attempt".to_string())
        );
    }

    #[test]
    fn preservation_batch_none_fields_roundtrip() {
        let batch = PreservationBatch {
            id: "batch-2".to_string(),
            source_crop_hash: None,
            method: "Drying".to_string(),
            quantity_kg: 2.0,
            started_at: 1700000000,
            expected_ready: 1700500000,
            status: BatchStatus::Completed,
            notes: None,
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.source_crop_hash, None);
        assert_eq!(decoded.notes, None);
        assert_eq!(decoded.status, BatchStatus::Completed);
    }

    // ========================================================================
    // PreservationMethod serde roundtrip tests
    // ========================================================================

    #[test]
    fn preservation_method_serde_roundtrip() {
        let method = PreservationMethod {
            name: "Water Bath Canning".to_string(),
            description: "High-acid food preservation using boiling water".to_string(),
            shelf_life_days: 365,
            equipment_needed: vec!["Canning pot".to_string(), "Jars".to_string()],
            skill_level: SkillLevel::Intermediate,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.name, "Water Bath Canning");
        assert_eq!(decoded.shelf_life_days, 365);
        assert_eq!(decoded.skill_level, SkillLevel::Intermediate);
        assert_eq!(decoded.equipment_needed.len(), 2);
    }

    // ========================================================================
    // StorageUnit serde roundtrip tests
    // ========================================================================

    #[test]
    fn storage_unit_serde_roundtrip() {
        let unit = StorageUnit {
            id: "store-1".to_string(),
            name: "Community Root Cellar".to_string(),
            capacity_kg: 500.0,
            storage_type: StorageType::RootCellar,
            steward: fake_agent(),
        };
        let json = serde_json::to_string(&unit).unwrap();
        let decoded: StorageUnit = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, "store-1");
        assert_eq!(decoded.name, "Community Root Cellar");
        assert_eq!(decoded.capacity_kg, 500.0);
        assert_eq!(decoded.storage_type, StorageType::RootCellar);
    }

    // ========================================================================
    // Clone/equality tests
    // ========================================================================

    #[test]
    fn preservation_batch_clone_equals_original() {
        let batch = PreservationBatch {
            id: "batch-clone".to_string(),
            source_crop_hash: Some(ActionHash::from_raw_36(vec![1u8; 36])),
            method: "Smoking".to_string(),
            quantity_kg: 3.5,
            started_at: 1700000000,
            expected_ready: 1700500000,
            status: BatchStatus::InProgress,
            notes: Some("Hickory chips".to_string()),
            allergen_flags: vec![],
        };
        assert_eq!(batch, batch.clone());
    }

    #[test]
    fn preservation_batch_ne_different_status() {
        let a = PreservationBatch {
            id: "batch-ne".to_string(),
            source_crop_hash: None,
            method: "Drying".to_string(),
            quantity_kg: 1.0,
            started_at: 0,
            expected_ready: 1,
            status: BatchStatus::InProgress,
            notes: None,
            allergen_flags: vec![],
        };
        let mut b = a.clone();
        b.status = BatchStatus::Failed;
        assert_ne!(a, b);
    }

    #[test]
    fn preservation_method_clone_equals_original() {
        let method = PreservationMethod {
            name: "Canning".to_string(),
            description: "Seal in jars".to_string(),
            shelf_life_days: 730,
            equipment_needed: vec!["Jars".to_string(), "Lids".to_string()],
            skill_level: SkillLevel::Advanced,
        };
        assert_eq!(method, method.clone());
    }

    #[test]
    fn storage_unit_clone_equals_original() {
        let unit = StorageUnit {
            id: "s-clone".to_string(),
            name: "Pantry".to_string(),
            capacity_kg: 50.0,
            storage_type: StorageType::Pantry,
            steward: fake_agent(),
        };
        assert_eq!(unit, unit.clone());
    }

    // ========================================================================
    // Edge case tests
    // ========================================================================

    #[test]
    fn preservation_batch_with_source_crop_hash() {
        let batch = PreservationBatch {
            id: "batch-crop".to_string(),
            source_crop_hash: Some(ActionHash::from_raw_36(vec![99u8; 36])),
            method: "Pickling".to_string(),
            quantity_kg: 10.0,
            started_at: 1700000000,
            expected_ready: 1702000000,
            status: BatchStatus::Completed,
            notes: Some("Pickled cucumbers".to_string()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert!(decoded.source_crop_hash.is_some());
        assert_eq!(decoded.status, BatchStatus::Completed);
    }

    #[test]
    fn preservation_batch_zero_quantity_serde() {
        let batch = PreservationBatch {
            id: "batch-zero".to_string(),
            source_crop_hash: None,
            method: "Freeze-drying".to_string(),
            quantity_kg: 0.0,
            started_at: 0,
            expected_ready: 0,
            status: BatchStatus::Failed,
            notes: None,
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.quantity_kg, 0.0);
        assert_eq!(decoded.status, BatchStatus::Failed);
    }

    #[test]
    fn preservation_batch_consumed_status_serde() {
        let batch = PreservationBatch {
            id: "batch-consumed".to_string(),
            source_crop_hash: None,
            method: "Fermenting".to_string(),
            quantity_kg: 2.5,
            started_at: 1700000000,
            expected_ready: 1700500000,
            status: BatchStatus::Consumed,
            notes: Some("Kimchi consumed by community".to_string()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.status, BatchStatus::Consumed);
    }

    #[test]
    fn preservation_method_empty_equipment_serde() {
        let method = PreservationMethod {
            name: "Sun drying".to_string(),
            description: "Dry in direct sunlight".to_string(),
            shelf_life_days: 90,
            equipment_needed: vec![],
            skill_level: SkillLevel::Beginner,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert!(decoded.equipment_needed.is_empty());
    }

    #[test]
    fn preservation_method_u32_max_shelf_life() {
        let method = PreservationMethod {
            name: "Ancient method".to_string(),
            description: "Lasts forever".to_string(),
            shelf_life_days: u32::MAX,
            equipment_needed: vec!["Magic jar".to_string()],
            skill_level: SkillLevel::Advanced,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.shelf_life_days, u32::MAX);
    }

    #[test]
    fn storage_unit_all_types_serde() {
        for st in [
            StorageType::RootCellar,
            StorageType::Cellar,
            StorageType::Freezer,
            StorageType::Dehydrator,
            StorageType::Fermenter,
            StorageType::Pantry,
            StorageType::Composter,
        ] {
            let unit = StorageUnit {
                id: "s".to_string(),
                name: "T".to_string(),
                capacity_kg: 1.0,
                storage_type: st.clone(),
                steward: fake_agent(),
            };
            let json = serde_json::to_string(&unit).unwrap();
            let decoded: StorageUnit = serde_json::from_str(&json).unwrap();
            assert_eq!(decoded.storage_type, st);
        }
    }

    #[test]
    fn preservation_method_unicode_name_serde() {
        let method = PreservationMethod {
            name: "\u{767D}\u{83DC}\u{306E}\u{6F2C}\u{7269}".to_string(),
            description: "Japanese pickling".to_string(),
            shelf_life_days: 30,
            equipment_needed: vec!["\u{6F2C}\u{7269}\u{77F3}".to_string()],
            skill_level: SkillLevel::Intermediate,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.name, "\u{767D}\u{83DC}\u{306E}\u{6F2C}\u{7269}");
    }

    #[test]
    fn storage_unit_large_capacity_serde() {
        let unit = StorageUnit {
            id: "s-big".to_string(),
            name: "Warehouse".to_string(),
            capacity_kg: f64::MAX,
            storage_type: StorageType::Freezer,
            steward: fake_agent(),
        };
        let json = serde_json::to_string(&unit).unwrap();
        let decoded: StorageUnit = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.capacity_kg, f64::MAX);
    }

    // ========================================================================
    // Additional boundary and edge case tests
    // ========================================================================

    #[test]
    fn preservation_method_many_equipment_items_serde() {
        let equipment: Vec<String> = (0..100).map(|i| format!("Equipment item #{}", i)).collect();
        let method = PreservationMethod {
            name: "Complex method".to_string(),
            description: "Requires many tools and supplies".to_string(),
            shelf_life_days: 180,
            equipment_needed: equipment.clone(),
            skill_level: SkillLevel::Advanced,
        };
        let json = serde_json::to_string(&method).unwrap();
        let decoded: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.equipment_needed.len(), 100);
        assert_eq!(decoded.equipment_needed[0], "Equipment item #0");
        assert_eq!(decoded.equipment_needed[99], "Equipment item #99");
    }

    #[test]
    fn preservation_batch_ne_different_id() {
        let a = PreservationBatch {
            id: "batch-a".to_string(),
            source_crop_hash: None,
            method: "Drying".to_string(),
            quantity_kg: 1.0,
            started_at: 0,
            expected_ready: 1,
            status: BatchStatus::InProgress,
            notes: None,
            allergen_flags: vec![],
        };
        let mut b = a.clone();
        b.id = "batch-b".to_string();
        assert_ne!(a, b);
    }

    #[test]
    fn storage_unit_ne_different_type() {
        let a = StorageUnit {
            id: "s-1".to_string(),
            name: "Unit A".to_string(),
            capacity_kg: 100.0,
            storage_type: StorageType::Freezer,
            steward: fake_agent(),
        };
        let mut b = a.clone();
        b.storage_type = StorageType::Pantry;
        assert_ne!(a, b);
    }

    #[test]
    fn preservation_batch_long_notes_serde() {
        let long_notes = "A".repeat(10_000);
        let batch = PreservationBatch {
            id: "batch-long".to_string(),
            source_crop_hash: None,
            method: "Salt curing".to_string(),
            quantity_kg: 25.0,
            started_at: 1700000000,
            expected_ready: 1702000000,
            status: BatchStatus::InProgress,
            notes: Some(long_notes.clone()),
            allergen_flags: vec![],
        };
        let json = serde_json::to_string(&batch).unwrap();
        let decoded: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.notes.unwrap().len(), 10_000);
    }
}
