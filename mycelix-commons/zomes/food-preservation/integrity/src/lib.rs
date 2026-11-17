// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Food Preservation Integrity Zome
//! Entry types and validation for food preservation batches, methods, and storage.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match};

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ============================================================================
// PRESERVATION BATCH
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum BatchStatus {
    InProgress,
    Completed,
    Failed,
    Consumed,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PreservationBatch {
    pub id: String,
    pub source_crop_hash: Option<ActionHash>,
    pub method: String,
    pub quantity_kg: f64,
    pub started_at: u64,
    pub expected_ready: u64,
    pub status: BatchStatus,
    pub notes: Option<String>,
    pub allergen_flags: Vec<String>,
}

// ============================================================================
// PRESERVATION METHOD
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SkillLevel {
    Beginner,
    Intermediate,
    Advanced,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PreservationMethod {
    pub name: String,
    pub description: String,
    pub shelf_life_days: u32,
    pub equipment_needed: Vec<String>,
    pub skill_level: SkillLevel,
}

// ============================================================================
// STORAGE UNIT
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum StorageType {
    RootCellar,
    Cellar,
    Freezer,
    Dehydrator,
    Fermenter,
    Pantry,
    Composter,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct StorageUnit {
    pub id: String,
    pub name: String,
    pub capacity_kg: f64,
    pub storage_type: StorageType,
    pub steward: AgentPubKey,
}


 // REUSABLE FOOD CONTAINERS AND APPEND-ONLY LIFECYCLE

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContainerMaterial {
    BorosilicateGlass,
    StainlessSteel,
    FoodGradePolymer,
    Silicone,
    Ceramic,
    Other,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContainerEventKind {
    Washed,
    Sanitized,
    Inspected,
    Filled,
    Sealed,
    Dispatched,
    Received,
    Emptied,
    Returned,
    Retired,
    Lost,
    Damaged,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContainerCondition {
    Serviceable,
    NeedsCleaning,
    Damaged,
    FailedInspection,
    Retired,
    Lost,
    Unknown,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ContainerEvidenceClass {
    Declared,
    Modelled,
    Simulated,
    Measured,
    IndependentlyAssayed,
    IndependentlyQualified,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContainerEvidenceRef {
    pub source_ref: String,
    pub method_ref: Option<String>,
    pub calibration_ref: Option<String>,
    pub evidence_class: ContainerEvidenceClass,
}

/// Capacity is millilitres; temperature limits are milli-degrees Celsius.
/// Food-contact qualification and cleaning references are evidence pointers.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ReusableContainer {
    pub id: String,
    pub material: ContainerMaterial,
    pub capacity_ml: u64,
    pub temperature_min_millic: Option<i32>,
    pub temperature_max_millic: Option<i32>,
    pub closure_type: String,
    pub food_contact_qualification_ref: String,
    pub cleaning_protocol_ref: String,
    pub manufacturer_reuse_cycle_limit: Option<u32>,
    pub compatible_product_refs: Vec<String>,
    pub registered_at: u64,
    pub steward: AgentPubKey,
    pub evidence: ContainerEvidenceRef,
}

/// Observations are not a claim that a microbiological reduction target was met.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContainerSanitationObservation {
    pub protocol_ref: String,
    pub wash_water_temp_millic: Option<i32>,
    pub sanitizer_ref: Option<String>,
    pub sanitizer_concentration_mg_l: Option<u64>,
    pub exposure_time_ms: Option<u64>,
    pub observation_ref: String,
    pub evidence: ContainerEvidenceRef,
}

/// Append-only lifecycle event. Reuse count is derived from Filled events.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ContainerCycleEvent {
    pub container_hash: ActionHash,
    pub previous_event_hash: Option<ActionHash>,
    pub sequence_number: u64,
    pub event_kind: ContainerEventKind,
    pub occurred_at: u64,
    pub product_lot_ref: Option<String>,
    pub preservation_batch_hash: Option<ActionHash>,
    pub location_ref: Option<String>,
    pub condition: ContainerCondition,
    pub sanitation: Option<ContainerSanitationObservation>,
    pub evidence: ContainerEvidenceRef,
}


// ============================================================================
// ENTRY & LINK TYPE REGISTRATION
// ============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    PreservationBatch(PreservationBatch),
    PreservationMethod(PreservationMethod),
    StorageUnit(StorageUnit),
    ReusableContainer(ReusableContainer),
    ContainerCycleEvent(ContainerCycleEvent),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllBatches,
    AllMethods,
    AllStorage,
    MethodToBatch,
    StorageToBatch,
    AgentToBatch,
    AllReusableContainers,
    ContainerToCycleEvents,
    ContainerEventToNext,
}

// ============================================================================
// VALIDATION
// ============================================================================

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::PreservationBatch(b) => validate_batch(b),
                EntryTypes::PreservationMethod(m) => validate_method(m),
                EntryTypes::StorageUnit(s) => validate_storage(s),
                EntryTypes::ReusableContainer(container) => validate_reusable_container(container),
                EntryTypes::ContainerCycleEvent(event) => validate_container_cycle_event(event),
            },
            OpEntry::UpdateEntry {
                app_entry: EntryTypes::PreservationBatch(b),
                ..
            } => validate_batch(b),
            OpEntry::UpdateEntry {
                app_entry: EntryTypes::ReusableContainer(container),
                ..
            } => validate_reusable_container(container),
            OpEntry::UpdateEntry {
                app_entry: EntryTypes::ContainerCycleEvent(_),
                ..
            } => Ok(ValidateCallbackResult::Invalid(
                "Container cycle events are append-only and cannot be updated".into(),
            )),
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Valid),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, tag, .. } => match link_type {
            LinkTypes::AllBatches => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AllBatches link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AllMethods => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AllMethods link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AllStorage => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AllStorage link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::MethodToBatch => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "MethodToBatch link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::StorageToBatch => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "StorageToBatch link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AgentToBatch => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToBatch link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AllReusableContainers
            | LinkTypes::ContainerToCycleEvents
            | LinkTypes::ContainerEventToNext => {
                if tag.0.len() > 512 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Container lifecycle link tag too long (max 512 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
        },
        FlatOp::RegisterDeleteLink { action, .. } => {
            let original_action = must_get_action(action.link_add_address.clone())?;
            Ok(check_link_author_match(
                original_action.action().author(),
                &action.author,
            ))
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "update",
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action, .. }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}


fn validate_container_evidence(
    evidence: &ContainerEvidenceRef,
) -> ExternResult<ValidateCallbackResult> {
    if evidence.source_ref.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid("Container evidence source_ref cannot be empty".into()));
    }
    if evidence.method_ref.as_deref().is_some_and(|value| value.trim().is_empty())
        || evidence.calibration_ref.as_deref().is_some_and(|value| value.trim().is_empty())
    {
        return Ok(ValidateCallbackResult::Invalid("Optional evidence references cannot be empty".into()));
    }
    if evidence.evidence_class == ContainerEvidenceClass::Measured
        && evidence.calibration_ref.as_deref().is_none_or(|value| value.trim().is_empty())
    {
        return Ok(ValidateCallbackResult::Invalid("Measured evidence requires a calibration reference".into()));
    }
    if matches!(evidence.evidence_class,
        ContainerEvidenceClass::IndependentlyAssayed | ContainerEvidenceClass::IndependentlyQualified)
        && evidence.method_ref.as_deref().is_none_or(|value| value.trim().is_empty())
    {
        return Ok(ValidateCallbackResult::Invalid("Independent assay/qualification requires a method reference".into()));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_reusable_container(
    container: ReusableContainer,
) -> ExternResult<ValidateCallbackResult> {
    if container.id.trim().is_empty() || container.id.len() > 128 {
        return Ok(ValidateCallbackResult::Invalid("Container ID must be non-empty and at most 128 characters".into()));
    }
    if container.capacity_ml == 0 {
        return Ok(ValidateCallbackResult::Invalid("Container capacity_ml must be positive".into()));
    }
    if container.closure_type.trim().is_empty()
        || container.closure_type.len() > 128
        || container.food_contact_qualification_ref.trim().is_empty()
        || container.cleaning_protocol_ref.trim().is_empty()
    {
        return Ok(ValidateCallbackResult::Invalid("Closure, food-contact and cleaning references are required".into()));
    }
    if container.temperature_min_millic.is_some_and(|minimum|
        container.temperature_max_millic.is_some_and(|maximum| minimum > maximum))
    {
        return Ok(ValidateCallbackResult::Invalid("Temperature minimum cannot exceed maximum".into()));
    }
    if container.manufacturer_reuse_cycle_limit == Some(0) {
        return Ok(ValidateCallbackResult::Invalid("Manufacturer reuse cycle limit must be positive when supplied".into()));
    }
    if container.compatible_product_refs.len() > 128
        || container.compatible_product_refs.iter().any(|value| value.trim().is_empty())
    {
        return Ok(ValidateCallbackResult::Invalid("Compatible product refs must be non-empty and at most 128 entries".into()));
    }
    validate_container_evidence(&container.evidence)
}

fn validate_container_cycle_event(
    event: ContainerCycleEvent,
) -> ExternResult<ValidateCallbackResult> {
    if event.occurred_at == 0 {
        return Ok(ValidateCallbackResult::Invalid("Container event time must be positive".into()));
    }
    if event.product_lot_ref.as_deref().is_some_and(|value| value.trim().is_empty())
        || event.location_ref.as_deref().is_some_and(|value| value.trim().is_empty())
    {
        return Ok(ValidateCallbackResult::Invalid("Optional lot/location refs must be non-empty when supplied".into()));
    }
    if event.event_kind == ContainerEventKind::Filled
        && event.product_lot_ref.is_none()
        && event.preservation_batch_hash.is_none()
    {
        return Ok(ValidateCallbackResult::Invalid("Filled event must reference a product lot or preservation batch".into()));
    }
    if matches!(event.event_kind, ContainerEventKind::Washed | ContainerEventKind::Sanitized) {
        let Some(observation) = event.sanitation.as_ref() else {
            return Ok(ValidateCallbackResult::Invalid("Washed/Sanitized events require a sanitation observation".into()));
        };
        if observation.protocol_ref.trim().is_empty() || observation.observation_ref.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid("Sanitation protocol and observation refs must be non-empty".into()));
        }
        if observation.sanitizer_ref.as_deref().is_some_and(|value| value.trim().is_empty())
            || observation.exposure_time_ms == Some(0)
        {
            return Ok(ValidateCallbackResult::Invalid("Sanitation optional refs must be non-empty and exposure time positive".into()));
        }
        validate_container_evidence(&observation.evidence)?;
    } else if event.sanitation.is_some() {
        return Ok(ValidateCallbackResult::Invalid("Sanitation observation is only valid on Washed/Sanitized events".into()));
    }

    if event.sequence_number == 0 {
        if event.previous_event_hash.is_some() {
            return Ok(ValidateCallbackResult::Invalid("Root container event must not have a previous event".into()));
        }
    } else if event.previous_event_hash.is_none() {
        return Ok(ValidateCallbackResult::Invalid("Non-root container event requires a previous event".into()));
    }

    let container_record = must_get_valid_record(event.container_hash.clone())?;
    let _: ReusableContainer = container_record.entry().to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Container reference is not a ReusableContainer entry".into())))?;

    if let Some(previous_hash) = event.previous_event_hash.as_ref() {
        let previous_record = must_get_valid_record(previous_hash.clone())?;
        let previous: ContainerCycleEvent = previous_record.entry().to_app_option()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest("Previous event is not a ContainerCycleEvent entry".into())))?;
        if previous.container_hash != event.container_hash {
            return Ok(ValidateCallbackResult::Invalid("Previous event belongs to another container".into()));
        }
        if previous.sequence_number.checked_add(1) != Some(event.sequence_number) {
            return Ok(ValidateCallbackResult::Invalid("Event sequence must be exactly previous sequence + 1".into()));
        }
        if event.occurred_at < previous.occurred_at {
            return Ok(ValidateCallbackResult::Invalid("Event time cannot precede parent event time".into()));
        }
    }
    validate_container_evidence(&event.evidence)
}

fn validate_batch(b: PreservationBatch) -> ExternResult<ValidateCallbackResult> {
    if b.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Batch ID cannot be empty".into(),
        ));
    }
    if b.method.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Method cannot be empty".into(),
        ));
    }
    if b.method.len() > 128 {
        return Ok(ValidateCallbackResult::Invalid(
            "Method too long (max 128 chars)".into(),
        ));
    }
    if !b.quantity_kg.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity must be a finite number".into(),
        ));
    }
    if b.quantity_kg <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity must be positive".into(),
        ));
    }
    // Batch ID max length
    if b.id.len() > 128 {
        return Ok(ValidateCallbackResult::Invalid(
            "Batch ID too long (max 128 chars)".into(),
        ));
    }
    // expected_ready must be >= started_at
    if b.expected_ready < b.started_at {
        return Ok(ValidateCallbackResult::Invalid(
            "expected_ready cannot be before started_at".into(),
        ));
    }
    // Allergen flags validation
    if b.allergen_flags.len() > 50 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 50 allergen flags".into(),
        ));
    }
    for flag in &b.allergen_flags {
        if flag.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Allergen flag cannot be empty".into(),
            ));
        }
        if flag.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Allergen flag too long (max 128 chars)".into(),
            ));
        }
    }
    // Notes max length (if present)
    if let Some(ref notes) = b.notes {
        if notes.len() > 4096 {
            return Ok(ValidateCallbackResult::Invalid(
                "Batch notes too long (max 4096 chars)".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_method(m: PreservationMethod) -> ExternResult<ValidateCallbackResult> {
    if m.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Method name cannot be empty".into(),
        ));
    }
    if m.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Method name too long (max 256 chars)".into(),
        ));
    }
    if m.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Description cannot be empty".into(),
        ));
    }
    if m.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Description too long (max 4096 chars)".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_storage(s: StorageUnit) -> ExternResult<ValidateCallbackResult> {
    if s.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Storage ID cannot be empty".into(),
        ));
    }
    if s.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Storage ID too long (max 256 chars)".into(),
        ));
    }
    if s.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Storage name cannot be empty".into(),
        ));
    }
    if s.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Storage name too long (max 256 chars)".into(),
        ));
    }
    if !s.capacity_kg.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Capacity must be a finite number".into(),
        ));
    }
    if s.capacity_kg <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Capacity must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────

    fn fake_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn valid_batch() -> PreservationBatch {
        PreservationBatch {
            id: "batch-1".into(),
            source_crop_hash: None,
            method: "Lacto-fermentation".into(),
            quantity_kg: 5.0,
            started_at: 1700000000,
            expected_ready: 1701000000,
            status: BatchStatus::InProgress,
            notes: None,
            allergen_flags: vec![],
        }
    }

    fn valid_method() -> PreservationMethod {
        PreservationMethod {
            name: "Water Bath Canning".into(),
            description: "High-acid food preservation using boiling water".into(),
            shelf_life_days: 365,
            equipment_needed: vec!["Canning pot".into(), "Jars".into()],
            skill_level: SkillLevel::Intermediate,
        }
    }

    fn valid_storage() -> StorageUnit {
        StorageUnit {
            id: "store-1".into(),
            name: "Community Root Cellar".into(),
            capacity_kg: 500.0,
            storage_type: StorageType::RootCellar,
            steward: fake_agent(),
        }
    }

    fn assert_valid(result: ExternResult<ValidateCallbackResult>) {
        match result {
            Ok(ValidateCallbackResult::Valid) => {}
            Ok(ValidateCallbackResult::Invalid(msg)) => {
                panic!("Expected Valid, got Invalid: {msg}")
            }
            other => panic!("Expected Valid, got {other:?}"),
        }
    }

    fn assert_invalid(result: ExternResult<ValidateCallbackResult>, expected_substr: &str) {
        match result {
            Ok(ValidateCallbackResult::Invalid(msg)) => {
                assert!(
                    msg.contains(expected_substr),
                    "Expected Invalid containing '{expected_substr}', got: '{msg}'"
                );
            }
            Ok(ValidateCallbackResult::Valid) => {
                panic!("Expected Invalid containing '{expected_substr}', got Valid")
            }
            other => panic!("Expected Invalid, got {other:?}"),
        }
    }

    // ── Serde roundtrip tests ───────────────────────────────────────────

    #[test]
    fn serde_roundtrip_batch_status() {
        let statuses = vec![
            BatchStatus::InProgress,
            BatchStatus::Completed,
            BatchStatus::Failed,
            BatchStatus::Consumed,
        ];
        for s in &statuses {
            let json = serde_json::to_string(s).unwrap();
            let back: BatchStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, s);
        }
    }

    #[test]
    fn serde_roundtrip_skill_level() {
        let levels = vec![
            SkillLevel::Beginner,
            SkillLevel::Intermediate,
            SkillLevel::Advanced,
        ];
        for l in &levels {
            let json = serde_json::to_string(l).unwrap();
            let back: SkillLevel = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, l);
        }
    }

    #[test]
    fn serde_roundtrip_storage_type() {
        let types = vec![
            StorageType::RootCellar,
            StorageType::Cellar,
            StorageType::Freezer,
            StorageType::Dehydrator,
            StorageType::Fermenter,
            StorageType::Pantry,
            StorageType::Composter,
        ];
        for t in &types {
            let json = serde_json::to_string(t).unwrap();
            let back: StorageType = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, t);
        }
    }

    #[test]
    fn serde_roundtrip_preservation_batch() {
        let b = valid_batch();
        let json = serde_json::to_string(&b).unwrap();
        let back: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(back, b);
    }

    #[test]
    fn serde_roundtrip_batch_with_optionals() {
        let mut b = valid_batch();
        b.source_crop_hash = Some(ActionHash::from_raw_36(vec![5u8; 36]));
        b.notes = Some("Good fermentation".into());
        let json = serde_json::to_string(&b).unwrap();
        let back: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(back, b);
    }

    #[test]
    fn serde_roundtrip_preservation_method() {
        let m = valid_method();
        let json = serde_json::to_string(&m).unwrap();
        let back: PreservationMethod = serde_json::from_str(&json).unwrap();
        assert_eq!(back, m);
    }

    #[test]
    fn serde_roundtrip_storage_unit() {
        let s = valid_storage();
        let json = serde_json::to_string(&s).unwrap();
        let back: StorageUnit = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);
    }

    // ── validate_batch: id ──────────────────────────────────────────────

    #[test]
    fn valid_batch_passes() {
        assert_valid(validate_batch(valid_batch()));
    }

    #[test]
    fn batch_empty_id_rejected() {
        let mut b = valid_batch();
        b.id = String::new();
        assert_invalid(validate_batch(b), "Batch ID cannot be empty");
    }

    #[test]
    fn batch_whitespace_id_rejected() {
        let mut b = valid_batch();
        b.id = "   ".into();
        assert_invalid(validate_batch(b), "Batch ID cannot be empty");
    }

    // ── validate_batch: method ──────────────────────────────────────────

    #[test]
    fn batch_empty_method_rejected() {
        let mut b = valid_batch();
        b.method = String::new();
        assert_invalid(validate_batch(b), "Method cannot be empty");
    }

    #[test]
    fn batch_whitespace_method_rejected() {
        let mut b = valid_batch();
        b.method = " ".into();
        assert_invalid(validate_batch(b), "Method cannot be empty");
    }

    // ── validate_batch: method length ─────────────────────────────────

    #[test]
    fn batch_method_too_long_rejected() {
        let mut b = valid_batch();
        b.method = "x".repeat(129);
        assert_invalid(validate_batch(b), "Method too long (max 128 chars)");
    }

    #[test]
    fn batch_method_at_max_valid() {
        let mut b = valid_batch();
        b.method = "x".repeat(128);
        assert_valid(validate_batch(b));
    }

    // ── validate_batch: quantity_kg ─────────────────────────────────────

    #[test]
    fn batch_zero_quantity_rejected() {
        let mut b = valid_batch();
        b.quantity_kg = 0.0;
        assert_invalid(validate_batch(b), "Quantity must be positive");
    }

    #[test]
    fn batch_negative_quantity_rejected() {
        let mut b = valid_batch();
        b.quantity_kg = -5.0;
        assert_invalid(validate_batch(b), "Quantity must be positive");
    }

    #[test]
    fn batch_barely_positive_quantity_valid() {
        let mut b = valid_batch();
        b.quantity_kg = 0.001;
        assert_valid(validate_batch(b));
    }

    #[test]
    fn batch_large_quantity_valid() {
        let mut b = valid_batch();
        b.quantity_kg = 99999.0;
        assert_valid(validate_batch(b));
    }

    #[test]
    fn batch_barely_negative_quantity_rejected() {
        let mut b = valid_batch();
        b.quantity_kg = -0.001;
        assert_invalid(validate_batch(b), "Quantity must be positive");
    }

    // ── validate_batch: status variants ─────────────────────────────────

    #[test]
    fn batch_all_statuses_valid() {
        for status in [
            BatchStatus::InProgress,
            BatchStatus::Completed,
            BatchStatus::Failed,
            BatchStatus::Consumed,
        ] {
            let mut b = valid_batch();
            b.status = status;
            assert_valid(validate_batch(b));
        }
    }

    // ── validate_batch: optional fields ─────────────────────────────────

    #[test]
    fn batch_with_source_crop_hash_valid() {
        let mut b = valid_batch();
        b.source_crop_hash = Some(ActionHash::from_raw_36(vec![10u8; 36]));
        assert_valid(validate_batch(b));
    }

    #[test]
    fn batch_with_notes_valid() {
        let mut b = valid_batch();
        b.notes = Some("Batch notes here".into());
        assert_valid(validate_batch(b));
    }

    #[test]
    fn batch_empty_id_with_valid_rest_rejected() {
        let mut b = valid_batch();
        b.id = String::new();
        b.notes = Some("notes".into());
        b.source_crop_hash = Some(ActionHash::from_raw_36(vec![1u8; 36]));
        assert_invalid(validate_batch(b), "Batch ID cannot be empty");
    }

    // ── validate_method: name ───────────────────────────────────────────

    #[test]
    fn valid_method_passes() {
        assert_valid(validate_method(valid_method()));
    }

    #[test]
    fn method_empty_name_rejected() {
        let mut m = valid_method();
        m.name = String::new();
        assert_invalid(validate_method(m), "Method name cannot be empty");
    }

    #[test]
    fn method_whitespace_name_rejected() {
        let mut m = valid_method();
        m.name = "  ".into();
        assert_invalid(validate_method(m), "Method name cannot be empty");
    }

    // ── validate_method: name length ──────────────────────────────────

    #[test]
    fn method_name_too_long_rejected() {
        let mut m = valid_method();
        m.name = "x".repeat(257);
        assert_invalid(validate_method(m), "Method name too long (max 256 chars)");
    }

    #[test]
    fn method_name_at_max_valid() {
        let mut m = valid_method();
        m.name = "x".repeat(256);
        assert_valid(validate_method(m));
    }

    // ── validate_method: description ────────────────────────────────────

    #[test]
    fn method_empty_description_rejected() {
        let mut m = valid_method();
        m.description = String::new();
        assert_invalid(validate_method(m), "Description cannot be empty");
    }

    #[test]
    fn method_whitespace_description_rejected() {
        let mut m = valid_method();
        m.description = " ".into();
        assert_invalid(validate_method(m), "Description cannot be empty");
    }

    // ── validate_method: description length ────────────────────────────

    #[test]
    fn method_description_too_long_rejected() {
        let mut m = valid_method();
        m.description = "x".repeat(4097);
        assert_invalid(validate_method(m), "Description too long (max 4096 chars)");
    }

    #[test]
    fn method_description_at_max_valid() {
        let mut m = valid_method();
        m.description = "x".repeat(4096);
        assert_valid(validate_method(m));
    }

    // ── validate_method: skill levels ───────────────────────────────────

    #[test]
    fn method_all_skill_levels_valid() {
        for level in [
            SkillLevel::Beginner,
            SkillLevel::Intermediate,
            SkillLevel::Advanced,
        ] {
            let mut m = valid_method();
            m.skill_level = level;
            assert_valid(validate_method(m));
        }
    }

    // ── validate_method: optional fields ────────────────────────────────

    #[test]
    fn method_zero_shelf_life_valid() {
        let mut m = valid_method();
        m.shelf_life_days = 0;
        assert_valid(validate_method(m));
    }

    #[test]
    fn method_empty_equipment_valid() {
        let mut m = valid_method();
        m.equipment_needed = vec![];
        assert_valid(validate_method(m));
    }

    #[test]
    fn method_large_shelf_life_valid() {
        let mut m = valid_method();
        m.shelf_life_days = 36500;
        assert_valid(validate_method(m));
    }

    // ── validate_storage: id ────────────────────────────────────────────

    #[test]
    fn valid_storage_passes() {
        assert_valid(validate_storage(valid_storage()));
    }

    #[test]
    fn storage_empty_id_rejected() {
        let mut s = valid_storage();
        s.id = String::new();
        assert_invalid(validate_storage(s), "Storage ID cannot be empty");
    }

    #[test]
    fn storage_whitespace_id_rejected() {
        let mut s = valid_storage();
        s.id = " ".into();
        assert_invalid(validate_storage(s), "Storage ID cannot be empty");
    }

    // ── validate_storage: id length ──────────────────────────────────

    #[test]
    fn storage_id_too_long_rejected() {
        let mut s = valid_storage();
        s.id = "x".repeat(257);
        assert_invalid(validate_storage(s), "Storage ID too long (max 256 chars)");
    }

    #[test]
    fn storage_id_at_max_valid() {
        let mut s = valid_storage();
        s.id = "x".repeat(64);
        assert_valid(validate_storage(s));
    }

    // ── validate_storage: name ──────────────────────────────────────────

    #[test]
    fn storage_empty_name_rejected() {
        let mut s = valid_storage();
        s.name = String::new();
        assert_invalid(validate_storage(s), "Storage name cannot be empty");
    }

    #[test]
    fn storage_whitespace_name_rejected() {
        let mut s = valid_storage();
        s.name = "  ".into();
        assert_invalid(validate_storage(s), "Storage name cannot be empty");
    }

    // ── validate_storage: name length ────────────────────────────────

    #[test]
    fn storage_name_too_long_rejected() {
        let mut s = valid_storage();
        s.name = "x".repeat(257);
        assert_invalid(validate_storage(s), "Storage name too long (max 256 chars)");
    }

    #[test]
    fn storage_name_at_max_valid() {
        let mut s = valid_storage();
        s.name = "x".repeat(256);
        assert_valid(validate_storage(s));
    }

    // ── validate_storage: capacity_kg ───────────────────────────────────

    #[test]
    fn storage_zero_capacity_rejected() {
        let mut s = valid_storage();
        s.capacity_kg = 0.0;
        assert_invalid(validate_storage(s), "Capacity must be positive");
    }

    #[test]
    fn storage_negative_capacity_rejected() {
        let mut s = valid_storage();
        s.capacity_kg = -10.0;
        assert_invalid(validate_storage(s), "Capacity must be positive");
    }

    #[test]
    fn storage_barely_positive_capacity_valid() {
        let mut s = valid_storage();
        s.capacity_kg = 0.01;
        assert_valid(validate_storage(s));
    }

    #[test]
    fn storage_large_capacity_valid() {
        let mut s = valid_storage();
        s.capacity_kg = 100000.0;
        assert_valid(validate_storage(s));
    }

    #[test]
    fn storage_barely_negative_capacity_rejected() {
        let mut s = valid_storage();
        s.capacity_kg = -0.001;
        assert_invalid(validate_storage(s), "Capacity must be positive");
    }

    // ── validate_storage: type variants ─────────────────────────────────

    #[test]
    fn all_storage_types_valid() {
        for st in [
            StorageType::RootCellar,
            StorageType::Cellar,
            StorageType::Freezer,
            StorageType::Dehydrator,
            StorageType::Fermenter,
            StorageType::Pantry,
            StorageType::Composter,
        ] {
            let mut s = valid_storage();
            s.storage_type = st;
            assert_valid(validate_storage(s));
        }
    }

    // ── validate_storage: empty_id takes priority ───────────────────────

    #[test]
    fn storage_empty_id_rejects_before_empty_name() {
        let mut s = valid_storage();
        s.id = String::new();
        s.name = String::new();
        // id check comes first
        assert_invalid(validate_storage(s), "Storage ID cannot be empty");
    }

    #[test]
    fn storage_empty_name_with_zero_capacity_rejects_name_first() {
        let mut s = valid_storage();
        s.name = String::new();
        s.capacity_kg = 0.0;
        // name check comes before capacity check
        assert_invalid(validate_storage(s), "Storage name cannot be empty");
    }

    // ── Anchor test ─────────────────────────────────────────────────────

    #[test]
    fn serde_roundtrip_anchor() {
        let a = Anchor("all_batches".to_string());
        let json = serde_json::to_string(&a).unwrap();
        let back: Anchor = serde_json::from_str(&json).unwrap();
        assert_eq!(back, a);
    }

    // ── Link tag length validation tests ────────────────────────────────

    fn validate_link_tag(link_type: &LinkTypes, tag_len: usize) -> ValidateCallbackResult {
        let tag = LinkTag(vec![0u8; tag_len]);
        let max = match link_type {
            LinkTypes::AllBatches
            | LinkTypes::AllMethods
            | LinkTypes::AllStorage
            | LinkTypes::MethodToBatch
            | LinkTypes::StorageToBatch
            | LinkTypes::AgentToBatch => 256,
        };
        let name = match link_type {
            LinkTypes::AllBatches => "AllBatches",
            LinkTypes::AllMethods => "AllMethods",
            LinkTypes::AllStorage => "AllStorage",
            LinkTypes::MethodToBatch => "MethodToBatch",
            LinkTypes::StorageToBatch => "StorageToBatch",
            LinkTypes::AgentToBatch => "AgentToBatch",
        };
        if tag.0.len() > max {
            ValidateCallbackResult::Invalid(format!(
                "{} link tag too long (max {} bytes)",
                name, max
            ))
        } else {
            ValidateCallbackResult::Valid
        }
    }

    #[test]
    fn test_link_all_batches_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::AllBatches, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_all_batches_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::AllBatches, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_all_methods_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::AllMethods, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_all_methods_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::AllMethods, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_all_storage_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::AllStorage, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_all_storage_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::AllStorage, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_method_to_batch_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::MethodToBatch, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_method_to_batch_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::MethodToBatch, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_storage_to_batch_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::StorageToBatch, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_storage_to_batch_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::StorageToBatch, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_agent_to_batch_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::AgentToBatch, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_agent_to_batch_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::AgentToBatch, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ── NaN/Infinity bypass hardening tests ────────────────────────────

    #[test]
    fn batch_nan_quantity_rejected() {
        let mut b = valid_batch();
        b.quantity_kg = f64::NAN;
        assert_invalid(validate_batch(b), "Quantity must be a finite number");
    }

    #[test]
    fn batch_infinity_quantity_rejected() {
        let mut b = valid_batch();
        b.quantity_kg = f64::INFINITY;
        assert_invalid(validate_batch(b), "Quantity must be a finite number");
    }

    #[test]
    fn storage_nan_capacity_rejected() {
        let mut s = valid_storage();
        s.capacity_kg = f64::NAN;
        assert_invalid(validate_storage(s), "Capacity must be a finite number");
    }

    #[test]
    fn storage_infinity_capacity_rejected() {
        let mut s = valid_storage();
        s.capacity_kg = f64::INFINITY;
        assert_invalid(validate_storage(s), "Capacity must be a finite number");
    }

    // ── Serde roundtrip: batch with allergens ──────────────────────────

    #[test]
    fn serde_roundtrip_batch_with_allergens() {
        let mut b = valid_batch();
        b.allergen_flags = vec!["Gluten".into(), "Dairy".into(), "Nuts".into()];
        let json = serde_json::to_string(&b).unwrap();
        let back: PreservationBatch = serde_json::from_str(&json).unwrap();
        assert_eq!(back, b);
    }

    // ── validate_batch: id max length ──────────────────────────────────

    #[test]
    fn batch_id_too_long_rejected() {
        let mut b = valid_batch();
        b.id = "x".repeat(129);
        assert_invalid(validate_batch(b), "Batch ID too long (max 128 chars)");
    }

    #[test]
    fn batch_id_exactly_128_accepted() {
        let mut b = valid_batch();
        b.id = "x".repeat(128);
        assert_valid(validate_batch(b));
    }

    // ── validate_batch: expected_ready vs started_at ───────────────────

    #[test]
    fn batch_expected_ready_before_started_rejected() {
        let mut b = valid_batch();
        b.started_at = 1700000000;
        b.expected_ready = 1699999999;
        assert_invalid(
            validate_batch(b),
            "expected_ready cannot be before started_at",
        );
    }

    #[test]
    fn batch_expected_ready_equals_started_accepted() {
        let mut b = valid_batch();
        b.started_at = 1700000000;
        b.expected_ready = 1700000000;
        assert_valid(validate_batch(b));
    }

    // ── validate_batch: allergen flags ─────────────────────────────────

    #[test]
    fn batch_allergen_flags_valid() {
        let mut b = valid_batch();
        b.allergen_flags = vec!["Gluten".into(), "Soy".into()];
        assert_valid(validate_batch(b));
    }

    #[test]
    fn batch_too_many_allergen_flags_rejected() {
        let mut b = valid_batch();
        b.allergen_flags = (0..51).map(|i| format!("allergen-{i}")).collect();
        assert_invalid(validate_batch(b), "Cannot have more than 50 allergen flags");
    }

    #[test]
    fn batch_empty_allergen_flag_rejected() {
        let mut b = valid_batch();
        b.allergen_flags = vec!["Gluten".into(), "".into()];
        assert_invalid(validate_batch(b), "Allergen flag cannot be empty");
    }

    #[test]
    fn batch_whitespace_allergen_flag_rejected() {
        let mut b = valid_batch();
        b.allergen_flags = vec!["   ".into()];
        assert_invalid(validate_batch(b), "Allergen flag cannot be empty");
    }

    #[test]
    fn batch_allergen_flag_too_long_rejected() {
        let mut b = valid_batch();
        b.allergen_flags = vec!["a".repeat(129)];
        assert_invalid(validate_batch(b), "Allergen flag too long (max 128 chars)");
    }

    #[test]
    fn batch_exactly_50_allergen_flags_accepted() {
        let mut b = valid_batch();
        b.allergen_flags = (0..50).map(|i| format!("allergen-{i}")).collect();
        assert_valid(validate_batch(b));
    }

    #[test]
    fn batch_allergen_flag_exactly_128_accepted() {
        let mut b = valid_batch();
        b.allergen_flags = vec!["a".repeat(128)];
        assert_valid(validate_batch(b));
    }

    // ── validate_batch: notes max length ───────────────────────────────

    #[test]
    fn batch_notes_too_long_rejected() {
        let mut b = valid_batch();
        b.notes = Some("n".repeat(4097));
        assert_invalid(validate_batch(b), "Batch notes too long (max 4096 chars)");
    }

    #[test]
    fn batch_notes_exactly_4096_accepted() {
        let mut b = valid_batch();
        b.notes = Some("n".repeat(4096));
        assert_valid(validate_batch(b));
    }
}
