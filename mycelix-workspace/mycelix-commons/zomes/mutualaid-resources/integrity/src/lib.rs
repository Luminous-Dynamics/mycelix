// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Resources Integrity Zome
//!
//! This zome defines entry types and validation rules for resource sharing
//! in the Mycelix Mutual Aid hApp. Supports tools, vehicles, spaces, and equipment.

use hdi::prelude::*;
use mutualaid_common::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match};

/// Entry types for the resources zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// A shared resource
    #[entry_type(visibility = "public")]
    SharedResource(SharedResource),
    /// A booking for a resource
    #[entry_type(visibility = "public")]
    Booking(Booking),
    /// Usage record
    #[entry_type(visibility = "public")]
    Usage(Usage),
    /// Maintenance record
    #[entry_type(visibility = "public")]
    Maintenance(Maintenance),
}

/// Link types for the resources zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Link from owner to their resources
    OwnerToResources,
    /// Link from resource type anchor to resources
    TypeToResources,
    /// Link from resource to its bookings
    ResourceToBookings,
    /// Link from booker to their bookings
    BookerToBookings,
    /// Link from resource to usage records
    ResourceToUsage,
    /// Link from resource to maintenance records
    ResourceToMaintenance,
    /// Link for all resources discovery
    AllResources,
    /// Link for available resources
    AvailableResources,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => validate_create_entry(action, app_entry),
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => validate_update_entry_type(action, original_action_hash, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            ..
        } => validate_create_link(link_type, base_address, target_address, tag),
        FlatOp::RegisterDeleteLink { action, .. } => {
            let original_action = must_get_action(action.link_add_address.clone())?;
            Ok(check_link_author_match(
                original_action.action().author(),
                &action.author,
            ))
        }
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
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Validate entry creation. SharedResource.owner/Booking.booker/
/// Maintenance.maintainer author-binding is belt-and-suspenders --
/// create_resource/create_booking/record_maintenance already derive these
/// from agent_info(). Usage has no self-declared identity field at all --
/// authorization for who may start/complete usage is instead enforced on
/// the corresponding Booking status transition (see
/// validate_update_booking), since start_usage/complete_usage both update
/// the linked Booking as part of the same atomic zome call.
fn validate_create_entry(
    action: Create,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::SharedResource(resource) => {
            if resource.owner != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Resource owner must correspond to the committing agent".into(),
                ));
            }
            validate_shared_resource(resource)
        }
        EntryTypes::Booking(booking) => {
            if booking.booker != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Booking booker must correspond to the committing agent".into(),
                ));
            }
            validate_booking(booking)
        }
        EntryTypes::Usage(usage) => validate_usage(usage),
        EntryTypes::Maintenance(maintenance) => {
            if maintenance.maintainer != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Maintenance maintainer must correspond to the committing agent".into(),
                ));
            }
            validate_maintenance(maintenance)
        }
    }
}

/// SharedResource/Booking/Usage all have live coordinator update paths;
/// Maintenance has none (record_maintenance is create-only) and is made
/// immutable.
fn validate_update_entry_type(
    action: Update,
    original_action_hash: ActionHash,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::SharedResource(resource) => {
            validate_update_shared_resource(action, original_action_hash, resource)
        }
        EntryTypes::Booking(booking) => {
            validate_update_booking(action, original_action_hash, booking)
        }
        EntryTypes::Usage(usage) => validate_update_usage(action, original_action_hash, usage),
        EntryTypes::Maintenance(_) => Ok(ValidateCallbackResult::Invalid(
            "Maintenance records are immutable".into(),
        )),
    }
}

/// **Real authorization fix**: set_resource_availability's own client-side
/// check ("only the owner can update availability") is now enforced at the
/// integrity level. Content restricted to currently_available/updated_at.
fn validate_update_shared_resource(
    action: Update,
    original_action_hash: ActionHash,
    resource: SharedResource,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: SharedResource = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original resource not found".to_string()
        )))?;

    if resource.id != original.id
        || resource.owner != original.owner
        || resource.name != original.name
        || resource.description != original.description
        || resource.resource_type != original.resource_type
        || resource.condition != original.condition
        || resource.photos != original.photos
        || resource.location != original.location
        || resource.availability != original.availability
        || resource.sharing_model != original.sharing_model
        || resource.usage_instructions != original.usage_instructions
        || resource.liability_notes != original.liability_notes
        || resource.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only currently_available/updated_at can change on a resource update".into(),
        ));
    }

    if action.author != original.owner {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the owner can update resource availability".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// **Real authorization fix, closing two live gaps**: confirm_booking and
/// cancel_booking already have client-side checks (owner-only;
/// booker-or-owner) now enforced for real here. start_usage and
/// complete_usage had NO caller-identity check anywhere, so ANY agent
/// could drive any booking to Active/Completed. Both are now required to
/// be driven by the booking's own booker (Completed also allows the
/// resource owner). Content otherwise restricted to `status` only.
fn validate_update_booking(
    action: Update,
    original_action_hash: ActionHash,
    booking: Booking,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Booking = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original booking not found".to_string()
        )))?;

    if booking.id != original.id
        || booking.resource_hash != original.resource_hash
        || booking.booker != original.booker
        || booking.start_time != original.start_time
        || booking.end_time != original.end_time
        || booking.purpose != original.purpose
        || booking.payment_method != original.payment_method
        || booking.payment_hash != original.payment_hash
        || booking.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on a booking update".into(),
        ));
    }

    let resource_record = must_get_valid_record(original.resource_hash.clone())?;
    let resource: SharedResource = resource_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Resource not found".to_string()
        )))?;

    let is_booker = action.author == original.booker;
    let is_owner = action.author == resource.owner;

    let authorized = match booking.status {
        BookingStatus::Confirmed => is_owner,
        BookingStatus::Cancelled => is_booker || is_owner,
        BookingStatus::Active => is_booker,
        BookingStatus::Completed => is_booker || is_owner,
        BookingStatus::Pending | BookingStatus::NoShow => is_booker || is_owner,
    };

    if !authorized {
        return Ok(ValidateCallbackResult::Invalid(
            "Not authorized to make this booking status transition".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Defense in depth alongside validate_update_booking: Usage has no
/// identity field of its own, so authorization is derived by fetching the
/// referenced Booking (via the immutable booking_hash) and requiring the
/// committer be that booking's booker or the resource's owner.
fn validate_update_usage(
    action: Update,
    original_action_hash: ActionHash,
    usage: Usage,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Usage = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original usage record not found".to_string()
        )))?;

    if usage.booking_hash != original.booking_hash
        || usage.actual_start != original.actual_start
        || usage.condition_before != original.condition_before
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only actual_end/condition_after/issues/notes can change on a usage update".into(),
        ));
    }

    let booking_record = must_get_valid_record(original.booking_hash.clone())?;
    let booking: Booking = booking_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Referenced booking not found".to_string()
        )))?;

    let resource_record = must_get_valid_record(booking.resource_hash.clone())?;
    let resource: SharedResource = resource_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Resource not found".to_string()
        )))?;

    if action.author != booking.booker && action.author != resource.owner {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the booker or resource owner can update a usage record".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a shared resource
fn validate_shared_resource(resource: SharedResource) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if resource.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource ID cannot be empty".to_string(),
        ));
    }

    // ID length limit
    if resource.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource ID must be 256 characters or fewer".to_string(),
        ));
    }

    // Name must not be empty
    if resource.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource name cannot be empty".to_string(),
        ));
    }

    // Name length limit
    if resource.name.len() > 150 {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource name cannot exceed 150 characters".to_string(),
        ));
    }

    // Description length limit
    if resource.description.len() > 3000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource description cannot exceed 3000 characters".to_string(),
        ));
    }

    // Photos limit
    if resource.photos.len() > 10 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 10 photos".to_string(),
        ));
    }

    // Photo entry length limit
    for photo in &resource.photos {
        if photo.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each photo reference must be 256 characters or fewer".to_string(),
            ));
        }
    }

    // Usage instructions length limit
    if resource.usage_instructions.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Usage instructions must be 4096 characters or fewer".to_string(),
        ));
    }

    // Liability notes length limit
    if let Some(ref notes) = resource.liability_notes {
        if notes.len() > 4096 {
            return Ok(ValidateCallbackResult::Invalid(
                "Liability notes must be 4096 characters or fewer".to_string(),
            ));
        }
    }

    // ResourceType::Custom variant length limit
    if let ResourceType::Custom(ref s) = resource.resource_type {
        if s.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Custom resource type cannot be empty".to_string(),
            ));
        }
        if s.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Custom resource type exceeds 128 character limit".to_string(),
            ));
        }
    }

    // Sharing model validation
    if resource.sharing_model.hourly_rate < 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hourly rate cannot be negative".to_string(),
        ));
    }

    if let Some(daily) = resource.sharing_model.daily_rate {
        if daily < 0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Daily rate cannot be negative".to_string(),
            ));
        }
    }

    if let Some(deposit) = resource.sharing_model.deposit {
        if deposit < 0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Deposit cannot be negative".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a booking
fn validate_booking(booking: Booking) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if booking.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Booking ID cannot be empty".to_string(),
        ));
    }

    // ID length limit
    if booking.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Booking ID must be 256 characters or fewer".to_string(),
        ));
    }

    // End time must be after start time
    if booking.end_time <= booking.start_time {
        return Ok(ValidateCallbackResult::Invalid(
            "End time must be after start time".to_string(),
        ));
    }

    // Purpose length limit
    if booking.purpose.len() > 500 {
        return Ok(ValidateCallbackResult::Invalid(
            "Booking purpose cannot exceed 500 characters".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a usage record
fn validate_usage(usage: Usage) -> ExternResult<ValidateCallbackResult> {
    // If actual_end is set, it must be after actual_start
    if let Some(end) = usage.actual_end {
        if end <= usage.actual_start {
            return Ok(ValidateCallbackResult::Invalid(
                "End time must be after start time".to_string(),
            ));
        }
    }

    // Notes length limit
    if usage.notes.len() > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Usage notes cannot exceed 1000 characters".to_string(),
        ));
    }

    // Issues limit
    if usage.issues.len() > 10 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot report more than 10 issues".to_string(),
        ));
    }

    // Issue entry length limit
    for issue in &usage.issues {
        if issue.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each issue must be 256 characters or fewer".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a maintenance record
fn validate_maintenance(maintenance: Maintenance) -> ExternResult<ValidateCallbackResult> {
    // Description must not be empty
    if maintenance.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Maintenance description cannot be empty".to_string(),
        ));
    }

    // Description length limit
    if maintenance.description.len() > 2000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Maintenance description cannot exceed 2000 characters".to_string(),
        ));
    }

    // Hours spent must be non-negative
    if maintenance.hours_spent < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours spent cannot be negative".to_string(),
        ));
    }

    // Hours spent should be reasonable
    if maintenance.hours_spent > 100.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours spent cannot exceed 100".to_string(),
        ));
    }

    // Cost must be non-negative if present
    if let Some(cost) = maintenance.cost {
        if cost < 0 {
            return Ok(ValidateCallbackResult::Invalid(
                "Maintenance cost cannot be negative".to_string(),
            ));
        }
    }

    // MaintenanceType::Other variant length limit
    if let MaintenanceType::Other(ref s) = maintenance.maintenance_type {
        if s.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Custom maintenance type cannot be empty".to_string(),
            ));
        }
        if s.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Custom maintenance type exceeds 128 character limit".to_string(),
            ));
        }
    }

    // Parts limit
    if maintenance.parts_used.len() > 20 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot list more than 20 parts".to_string(),
        ));
    }

    // Part entry length limit
    for part in &maintenance.parts_used {
        if part.len() > 256 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each part name must be 256 characters or fewer".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate link creation
fn validate_create_link(
    link_type: LinkTypes,
    _base_address: AnyLinkableHash,
    _target_address: AnyLinkableHash,
    tag: LinkTag,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::OwnerToResources => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "OwnerToResources link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::TypeToResources => {
            if tag.0.len() > 512 {
                return Ok(ValidateCallbackResult::Invalid(
                    "TypeToResources link tag too long (max 512 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::ResourceToBookings => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "ResourceToBookings link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::BookerToBookings => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "BookerToBookings link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::ResourceToUsage => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "ResourceToUsage link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::ResourceToMaintenance => {
            if tag.0.len() > 512 {
                return Ok(ValidateCallbackResult::Invalid(
                    "ResourceToMaintenance link tag too long (max 512 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::AllResources => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "AllResources link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        LinkTypes::AvailableResources => {
            if tag.0.len() > 256 {
                return Ok(ValidateCallbackResult::Invalid(
                    "AvailableResources link tag too long (max 256 bytes)".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // =============================================================================
    // FACTORY FUNCTIONS
    // =============================================================================

    fn test_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0xdb; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0xdc; 36])
    }

    fn create_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn test_action_hash() -> ActionHash {
        ActionHash::from_raw_36(vec![0xab; 36])
    }

    fn test_timestamp_early() -> Timestamp {
        Timestamp::from_micros(1000000)
    }

    fn test_timestamp_late() -> Timestamp {
        Timestamp::from_micros(2000000)
    }

    fn valid_shared_resource() -> SharedResource {
        SharedResource {
            id: "resource-1".to_string(),
            owner: test_agent(),
            name: "Test Power Drill".to_string(),
            description: "A reliable power drill for community use".to_string(),
            resource_type: ResourceType::PowerTool,
            condition: ResourceCondition::Good,
            photos: vec![],
            location: LocationConstraint::FixedLocation("123 Main St".to_string()),
            availability: Availability::default(),
            sharing_model: SharingModel {
                free: false,
                deposit: Some(50),
                hourly_rate: 10,
                daily_rate: Some(50),
                accepts_time_credits: true,
                accepts_circle_credits: true,
                circle_hash: None,
            },
            usage_instructions: "Handle with care".to_string(),
            liability_notes: Some("User responsible for damages".to_string()),
            currently_available: true,
            created_at: test_timestamp_early(),
            updated_at: test_timestamp_early(),
        }
    }

    fn valid_booking() -> Booking {
        Booking {
            id: "booking-1".to_string(),
            resource_hash: test_action_hash(),
            booker: test_agent(),
            start_time: test_timestamp_early(),
            end_time: test_timestamp_late(),
            purpose: "Weekend project".to_string(),
            status: BookingStatus::Pending,
            payment_method: Some(PaymentMethod::TimeCredits(2.0)),
            payment_hash: None,
            created_at: test_timestamp_early(),
        }
    }

    fn valid_usage() -> Usage {
        Usage {
            booking_hash: test_action_hash(),
            actual_start: test_timestamp_early(),
            actual_end: Some(test_timestamp_late()),
            condition_before: ResourceCondition::Good,
            condition_after: Some(ResourceCondition::Good),
            notes: "Used as expected".to_string(),
            issues: vec![],
        }
    }

    fn valid_maintenance() -> Maintenance {
        Maintenance {
            resource_hash: test_action_hash(),
            maintainer: test_agent(),
            maintenance_type: MaintenanceType::Routine,
            description: "Oil change and inspection".to_string(),
            cost: Some(25),
            hours_spent: 2.0,
            parts_used: vec!["Oil filter".to_string()],
            performed_at: test_timestamp_early(),
            next_due: Some(test_timestamp_late()),
        }
    }

    // =============================================================================
    // SHARED RESOURCE VALIDATION TESTS
    // =============================================================================

    #[test]
    fn test_valid_shared_resource() {
        let resource = valid_shared_resource();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_empty_id() {
        let mut resource = valid_shared_resource();
        resource.id = "".to_string();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_empty_name() {
        let mut resource = valid_shared_resource();
        resource.name = "".to_string();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_name_exact_limit() {
        let mut resource = valid_shared_resource();
        resource.name = "a".repeat(150);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_name_exceeds_limit() {
        let mut resource = valid_shared_resource();
        resource.name = "a".repeat(151);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_description_exact_limit() {
        let mut resource = valid_shared_resource();
        resource.description = "a".repeat(3000);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_description_exceeds_limit() {
        let mut resource = valid_shared_resource();
        resource.description = "a".repeat(3001);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_photos_exact_limit() {
        let mut resource = valid_shared_resource();
        resource.photos = (0..10).map(|i| format!("photo{}", i)).collect();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_photos_exceeds_limit() {
        let mut resource = valid_shared_resource();
        resource.photos = (0..11).map(|i| format!("photo{}", i)).collect();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_hourly_rate_zero() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.hourly_rate = 0;
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_hourly_rate_negative() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.hourly_rate = -1;
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_daily_rate_zero() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.daily_rate = Some(0);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_daily_rate_negative() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.daily_rate = Some(-1);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_daily_rate_none() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.daily_rate = None;
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_deposit_zero() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.deposit = Some(0);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_deposit_negative() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.deposit = Some(-1);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_deposit_none() {
        let mut resource = valid_shared_resource();
        resource.sharing_model.deposit = None;
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    // =============================================================================
    // BOOKING VALIDATION TESTS
    // =============================================================================

    #[test]
    fn test_valid_booking() {
        let booking = valid_booking();
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_booking_empty_id() {
        let mut booking = valid_booking();
        booking.id = "".to_string();
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_booking_end_before_start() {
        let mut booking = valid_booking();
        booking.start_time = test_timestamp_late();
        booking.end_time = test_timestamp_early();
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_booking_end_equals_start() {
        let mut booking = valid_booking();
        booking.start_time = test_timestamp_early();
        booking.end_time = test_timestamp_early();
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_booking_purpose_exact_limit() {
        let mut booking = valid_booking();
        booking.purpose = "a".repeat(500);
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_booking_purpose_exceeds_limit() {
        let mut booking = valid_booking();
        booking.purpose = "a".repeat(501);
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_booking_purpose_empty() {
        let mut booking = valid_booking();
        booking.purpose = "".to_string();
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    // =============================================================================
    // USAGE VALIDATION TESTS
    // =============================================================================

    #[test]
    fn test_valid_usage() {
        let usage = valid_usage();
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_usage_actual_end_none() {
        let mut usage = valid_usage();
        usage.actual_end = None;
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_usage_actual_end_before_start() {
        let mut usage = valid_usage();
        usage.actual_start = test_timestamp_late();
        usage.actual_end = Some(test_timestamp_early());
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_usage_actual_end_equals_start() {
        let mut usage = valid_usage();
        usage.actual_start = test_timestamp_early();
        usage.actual_end = Some(test_timestamp_early());
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_usage_notes_exact_limit() {
        let mut usage = valid_usage();
        usage.notes = "a".repeat(1000);
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_usage_notes_exceeds_limit() {
        let mut usage = valid_usage();
        usage.notes = "a".repeat(1001);
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_usage_notes_empty() {
        let mut usage = valid_usage();
        usage.notes = "".to_string();
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_usage_issues_exact_limit() {
        let mut usage = valid_usage();
        usage.issues = (0..10).map(|i| format!("issue{}", i)).collect();
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_usage_issues_exceeds_limit() {
        let mut usage = valid_usage();
        usage.issues = (0..11).map(|i| format!("issue{}", i)).collect();
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_usage_no_issues() {
        let mut usage = valid_usage();
        usage.issues = vec![];
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    // =============================================================================
    // MAINTENANCE VALIDATION TESTS
    // =============================================================================

    #[test]
    fn test_valid_maintenance() {
        let maintenance = valid_maintenance();
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_empty_description() {
        let mut maintenance = valid_maintenance();
        maintenance.description = "".to_string();
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_description_exact_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.description = "a".repeat(2000);
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_description_exceeds_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.description = "a".repeat(2001);
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_hours_zero() {
        let mut maintenance = valid_maintenance();
        maintenance.hours_spent = 0.0;
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_hours_negative() {
        let mut maintenance = valid_maintenance();
        maintenance.hours_spent = -0.1;
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_hours_exact_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.hours_spent = 100.0;
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_hours_exceeds_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.hours_spent = 100.1;
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_cost_zero() {
        let mut maintenance = valid_maintenance();
        maintenance.cost = Some(0);
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_cost_negative() {
        let mut maintenance = valid_maintenance();
        maintenance.cost = Some(-1);
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_cost_none() {
        let mut maintenance = valid_maintenance();
        maintenance.cost = None;
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_parts_exact_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.parts_used = (0..20).map(|i| format!("part{}", i)).collect();
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_parts_exceeds_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.parts_used = (0..21).map(|i| format!("part{}", i)).collect();
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_no_parts() {
        let mut maintenance = valid_maintenance();
        maintenance.parts_used = vec![];
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    // =============================================================================
    // ENUM SERIALIZATION TESTS
    // =============================================================================

    #[test]
    fn test_resource_type_serde_power_tool() {
        let rt = ResourceType::PowerTool;
        let json = serde_json::to_string(&rt).unwrap();
        let parsed: ResourceType = serde_json::from_str(&json).unwrap();
        assert_eq!(rt, parsed);
    }

    #[test]
    fn test_resource_type_serde_car() {
        let rt = ResourceType::Car;
        let json = serde_json::to_string(&rt).unwrap();
        let parsed: ResourceType = serde_json::from_str(&json).unwrap();
        assert_eq!(rt, parsed);
    }

    #[test]
    fn test_resource_type_serde_custom() {
        let rt = ResourceType::Custom("Specialized Tool".to_string());
        let json = serde_json::to_string(&rt).unwrap();
        let parsed: ResourceType = serde_json::from_str(&json).unwrap();
        assert_eq!(rt, parsed);
    }

    #[test]
    fn test_resource_condition_serde_excellent() {
        let rc = ResourceCondition::Excellent;
        let json = serde_json::to_string(&rc).unwrap();
        let parsed: ResourceCondition = serde_json::from_str(&json).unwrap();
        assert_eq!(rc, parsed);
    }

    #[test]
    fn test_resource_condition_serde_needs_repair() {
        let rc = ResourceCondition::NeedsRepair;
        let json = serde_json::to_string(&rc).unwrap();
        let parsed: ResourceCondition = serde_json::from_str(&json).unwrap();
        assert_eq!(rc, parsed);
    }

    #[test]
    fn test_booking_status_serde_pending() {
        let bs = BookingStatus::Pending;
        let json = serde_json::to_string(&bs).unwrap();
        let parsed: BookingStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(bs, parsed);
    }

    #[test]
    fn test_booking_status_serde_confirmed() {
        let bs = BookingStatus::Confirmed;
        let json = serde_json::to_string(&bs).unwrap();
        let parsed: BookingStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(bs, parsed);
    }

    #[test]
    fn test_booking_status_serde_no_show() {
        let bs = BookingStatus::NoShow;
        let json = serde_json::to_string(&bs).unwrap();
        let parsed: BookingStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(bs, parsed);
    }

    #[test]
    fn test_maintenance_type_serde_routine() {
        let mt = MaintenanceType::Routine;
        let json = serde_json::to_string(&mt).unwrap();
        let parsed: MaintenanceType = serde_json::from_str(&json).unwrap();
        assert_eq!(mt, parsed);
    }

    #[test]
    fn test_maintenance_type_serde_repair() {
        let mt = MaintenanceType::Repair;
        let json = serde_json::to_string(&mt).unwrap();
        let parsed: MaintenanceType = serde_json::from_str(&json).unwrap();
        assert_eq!(mt, parsed);
    }

    #[test]
    fn test_maintenance_type_serde_custom() {
        let mt = MaintenanceType::Other("Deep Clean".to_string());
        let json = serde_json::to_string(&mt).unwrap();
        let parsed: MaintenanceType = serde_json::from_str(&json).unwrap();
        assert_eq!(mt, parsed);
    }

    // =============================================================================
    // ADDITIONAL EDGE CASES
    // =============================================================================

    #[test]
    fn test_shared_resource_all_resource_types() {
        let types = vec![
            ResourceType::PowerTool,
            ResourceType::HandTool,
            ResourceType::GardenTool,
            ResourceType::Car,
            ResourceType::Bicycle,
            ResourceType::MeetingRoom,
            ResourceType::Workshop,
            ResourceType::CampingGear,
            ResourceType::Custom("Test".to_string()),
        ];

        for rt in types {
            let mut resource = valid_shared_resource();
            resource.resource_type = rt;
            let result = validate_shared_resource(resource);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    #[test]
    fn test_shared_resource_all_conditions() {
        let conditions = vec![
            ResourceCondition::Excellent,
            ResourceCondition::Good,
            ResourceCondition::Fair,
            ResourceCondition::NeedsRepair,
            ResourceCondition::BeingRepaired,
        ];

        for cond in conditions {
            let mut resource = valid_shared_resource();
            resource.condition = cond;
            let result = validate_shared_resource(resource);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    #[test]
    fn test_booking_all_statuses() {
        let statuses = vec![
            BookingStatus::Pending,
            BookingStatus::Confirmed,
            BookingStatus::Active,
            BookingStatus::Completed,
            BookingStatus::Cancelled,
            BookingStatus::NoShow,
        ];

        for status in statuses {
            let mut booking = valid_booking();
            booking.status = status;
            let result = validate_booking(booking);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    #[test]
    fn test_maintenance_all_types() {
        let types = vec![
            MaintenanceType::Routine,
            MaintenanceType::Repair,
            MaintenanceType::Cleaning,
            MaintenanceType::Upgrade,
            MaintenanceType::SafetyCheck,
            MaintenanceType::Other("Custom".to_string()),
        ];

        for mt in types {
            let mut maintenance = valid_maintenance();
            maintenance.maintenance_type = mt;
            let result = validate_maintenance(maintenance);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    #[test]
    fn test_shared_resource_free_model() {
        let mut resource = valid_shared_resource();
        resource.sharing_model = SharingModel {
            free: true,
            deposit: None,
            hourly_rate: 0,
            daily_rate: None,
            accepts_time_credits: false,
            accepts_circle_credits: false,
            circle_hash: None,
        };
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_booking_with_payment_methods() {
        let payment_methods = vec![
            Some(PaymentMethod::Free),
            Some(PaymentMethod::TimeCredits(5.0)),
            Some(PaymentMethod::CircleCredits {
                circle_hash: test_action_hash(),
                amount: 100,
            }),
            Some(PaymentMethod::External("Cash".to_string())),
            None,
        ];

        for pm in payment_methods {
            let mut booking = valid_booking();
            booking.payment_method = pm;
            let result = validate_booking(booking);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    #[test]
    fn test_usage_condition_changes() {
        let conditions = vec![
            (ResourceCondition::Excellent, Some(ResourceCondition::Good)),
            (ResourceCondition::Good, Some(ResourceCondition::Fair)),
            (
                ResourceCondition::Fair,
                Some(ResourceCondition::NeedsRepair),
            ),
            (ResourceCondition::Good, None),
        ];

        for (before, after) in conditions {
            let mut usage = valid_usage();
            usage.condition_before = before;
            usage.condition_after = after;
            let result = validate_usage(usage);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    #[test]
    fn test_maintenance_fractional_hours() {
        let hours = vec![0.0, 0.5, 1.5, 10.5, 50.25, 99.9, 100.0];

        for h in hours {
            let mut maintenance = valid_maintenance();
            maintenance.hours_spent = h;
            let result = validate_maintenance(maintenance);
            assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
        }
    }

    // =============================================================================
    // STRING/VEC LENGTH LIMIT TESTS
    // =============================================================================

    // SharedResource: id max 64
    #[test]
    fn test_shared_resource_id_exactly_64() {
        let mut resource = valid_shared_resource();
        resource.id = "x".repeat(64);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_id_65_rejected() {
        let mut resource = valid_shared_resource();
        resource.id = "x".repeat(257);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // SharedResource: photo entry max 256
    #[test]
    fn test_shared_resource_photo_exactly_256() {
        let mut resource = valid_shared_resource();
        resource.photos = vec!["x".repeat(256)];
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_photo_257_rejected() {
        let mut resource = valid_shared_resource();
        resource.photos = vec!["x".repeat(257)];
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // SharedResource: usage_instructions max 4096
    #[test]
    fn test_shared_resource_usage_instructions_exactly_4096() {
        let mut resource = valid_shared_resource();
        resource.usage_instructions = "x".repeat(4096);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_usage_instructions_4097_rejected() {
        let mut resource = valid_shared_resource();
        resource.usage_instructions = "x".repeat(4097);
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // SharedResource: liability_notes max 4096
    #[test]
    fn test_shared_resource_liability_notes_exactly_4096() {
        let mut resource = valid_shared_resource();
        resource.liability_notes = Some("x".repeat(4096));
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_liability_notes_4097_rejected() {
        let mut resource = valid_shared_resource();
        resource.liability_notes = Some("x".repeat(4097));
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_liability_notes_none_accepted() {
        let mut resource = valid_shared_resource();
        resource.liability_notes = None;
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    // Booking: id max 64
    #[test]
    fn test_booking_id_exactly_64() {
        let mut booking = valid_booking();
        booking.id = "x".repeat(64);
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_booking_id_65_rejected() {
        let mut booking = valid_booking();
        booking.id = "x".repeat(257);
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // Usage: issue entry max 256
    #[test]
    fn test_usage_issue_exactly_256() {
        let mut usage = valid_usage();
        usage.issues = vec!["x".repeat(256)];
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_usage_issue_257_rejected() {
        let mut usage = valid_usage();
        usage.issues = vec!["x".repeat(257)];
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_usage_multiple_issues_one_too_long() {
        let mut usage = valid_usage();
        usage.issues = vec!["valid issue".to_string(), "x".repeat(257)];
        let result = validate_usage(usage);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // Maintenance: part entry max 256
    #[test]
    fn test_maintenance_part_exactly_256() {
        let mut maintenance = valid_maintenance();
        maintenance.parts_used = vec!["x".repeat(256)];
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_part_257_rejected() {
        let mut maintenance = valid_maintenance();
        maintenance.parts_used = vec!["x".repeat(257)];
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_multiple_parts_one_too_long() {
        let mut maintenance = valid_maintenance();
        maintenance.parts_used = vec!["valid part".to_string(), "x".repeat(257)];
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // =============================================================================
    // LINK TAG VALIDATION TESTS
    // =============================================================================

    #[test]
    fn test_link_owner_to_resources_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 256]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::OwnerToResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_owner_to_resources_tag_too_long() {
        let tag = LinkTag(vec![0u8; 257]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::OwnerToResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_type_to_resources_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 512]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::TypeToResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_type_to_resources_tag_too_long() {
        let tag = LinkTag(vec![0u8; 513]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::TypeToResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_resource_to_bookings_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 256]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::ResourceToBookings, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_resource_to_bookings_tag_too_long() {
        let tag = LinkTag(vec![0u8; 257]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::ResourceToBookings, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_booker_to_bookings_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 256]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::BookerToBookings, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_booker_to_bookings_tag_too_long() {
        let tag = LinkTag(vec![0u8; 257]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::BookerToBookings, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_resource_to_usage_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 256]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::ResourceToUsage, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_resource_to_usage_tag_too_long() {
        let tag = LinkTag(vec![0u8; 257]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::ResourceToUsage, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_resource_to_maintenance_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 512]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::ResourceToMaintenance, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_resource_to_maintenance_tag_too_long() {
        let tag = LinkTag(vec![0u8; 513]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::ResourceToMaintenance, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_all_resources_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 256]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::AllResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_all_resources_tag_too_long() {
        let tag = LinkTag(vec![0u8; 257]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::AllResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_link_available_resources_tag_at_limit() {
        let tag = LinkTag(vec![0u8; 256]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::AvailableResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_link_available_resources_tag_too_long() {
        let tag = LinkTag(vec![0u8; 257]);
        let base = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let target = AnyLinkableHash::from(ActionHash::from_raw_36(vec![0u8; 36]));
        let result = validate_create_link(LinkTypes::AvailableResources, base, target, tag);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // =============================================================================
    // HARDENING: Custom enum variant string length boundary tests
    // =============================================================================

    // ── ResourceType::Custom (max 128) ─────────────────────────────────

    #[test]
    fn test_shared_resource_custom_type_at_limit() {
        let mut resource = valid_shared_resource();
        resource.resource_type = ResourceType::Custom("c".repeat(128));
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_shared_resource_custom_type_too_long() {
        let mut resource = valid_shared_resource();
        resource.resource_type = ResourceType::Custom("c".repeat(129));
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_shared_resource_custom_type_empty() {
        let mut resource = valid_shared_resource();
        resource.resource_type = ResourceType::Custom("".to_string());
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── MaintenanceType::Other (max 128) ───────────────────────────────

    #[test]
    fn test_maintenance_custom_type_at_limit() {
        let mut maintenance = valid_maintenance();
        maintenance.maintenance_type = MaintenanceType::Other("m".repeat(128));
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Valid)));
    }

    #[test]
    fn test_maintenance_custom_type_too_long() {
        let mut maintenance = valid_maintenance();
        maintenance.maintenance_type = MaintenanceType::Other("m".repeat(129));
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    #[test]
    fn test_maintenance_custom_type_empty() {
        let mut maintenance = valid_maintenance();
        maintenance.maintenance_type = MaintenanceType::Other("".to_string());
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── SharedResource whitespace-only name ────────────────────────────

    #[test]
    fn test_shared_resource_whitespace_name() {
        let mut resource = valid_shared_resource();
        resource.name = "   \t  ".to_string();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── SharedResource whitespace-only ID ──────────────────────────────

    #[test]
    fn test_shared_resource_whitespace_id() {
        let mut resource = valid_shared_resource();
        resource.id = "  \t ".to_string();
        let result = validate_shared_resource(resource);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── Booking whitespace-only ID ─────────────────────────────────────

    #[test]
    fn test_booking_whitespace_id() {
        let mut booking = valid_booking();
        booking.id = "  \t ".to_string();
        let result = validate_booking(booking);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // ── Maintenance whitespace-only description ────────────────────────

    #[test]
    fn test_maintenance_whitespace_description() {
        let mut maintenance = valid_maintenance();
        maintenance.description = "  \n\t  ".to_string();
        let result = validate_maintenance(maintenance);
        assert!(matches!(result, Ok(ValidateCallbackResult::Invalid(_))));
    }

    // =============================================================================
    // AUTHOR-BINDING TESTS (create) and UPDATE-IMMUTABILITY TESTS
    // =============================================================================

    #[test]
    fn create_resource_valid_when_committer_is_owner() {
        let resource = valid_shared_resource();
        let result = validate_create_entry(
            create_action(test_agent()),
            EntryTypes::SharedResource(resource),
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_resource_forgery_rejected() {
        let resource = valid_shared_resource();
        let result = validate_create_entry(
            create_action(other_agent()),
            EntryTypes::SharedResource(resource),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_booking_valid_when_committer_is_booker() {
        let booking = valid_booking();
        let result =
            validate_create_entry(create_action(test_agent()), EntryTypes::Booking(booking))
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_booking_forgery_rejected() {
        let booking = valid_booking();
        let result =
            validate_create_entry(create_action(other_agent()), EntryTypes::Booking(booking))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_maintenance_valid_when_committer_is_maintainer() {
        let maintenance = valid_maintenance();
        let result = validate_create_entry(
            create_action(test_agent()),
            EntryTypes::Maintenance(maintenance),
        )
        .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_maintenance_forgery_rejected() {
        let maintenance = valid_maintenance();
        let result = validate_create_entry(
            create_action(other_agent()),
            EntryTypes::Maintenance(maintenance),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_maintenance_update() {
        let maintenance = valid_maintenance();
        let result = validate_update_entry_type(
            Update {
                author: test_agent(),
                timestamp: Timestamp::from_micros(1),
                action_seq: 1,
                prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
                original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
                original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
                entry_type: EntryType::App(AppEntryDef::new(
                    EntryDefIndex::from(0),
                    0.into(),
                    EntryVisibility::Public,
                )),
                entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
                weight: Default::default(),
            },
            ActionHash::from_raw_36(vec![9u8; 36]),
            EntryTypes::Maintenance(maintenance),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_{shared_resource,booking,usage} all call
    // must_get_valid_record, which requires a live HDI host and can't run
    // in a plain unit test -- matching the established pattern from
    // circles'/needs' equivalent update validators.
}
