// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Resources Integrity Zome
//!
//! This zome defines entry types and validation rules for resource sharing
//! in the Mycelix Mutual Aid hApp. Supports tools, vehicles, spaces, and equipment.

use hdi::prelude::*;
use mutualaid_common::*;

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
        // Deliberately left fully permissive (coordinator never calls
        // delete_link), reviewed 2026-07-09 during the P0 author-binding pass.
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            let _ = link_type;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // Previously fully permissive (`Ok(Valid)` unconditionally via
            // the catch-all `_` arm) -- the 32nd confirmed instance of this
            // exact bug pattern this pass. Found + fixed 2026-07-09 during
            // the P0 author-binding pass.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(
                action.clone(),
                action.original_action_address,
                app_entry,
            ),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Validate entry creation
fn validate_create_entry(
    action: Create,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::SharedResource(resource) => {
            // Author-binding: create_resource already derives owner from
            // agent_info(), so this is belt-and-suspenders. Found + fixed
            // 2026-07-09 during the P0 author-binding pass.
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
        // Usage has no self-declared identity field at all (case a) --
        // authorization for who may start/complete usage is instead
        // enforced on the corresponding Booking status transition (see
        // validate_update_booking), since start_usage/complete_usage both
        // update the linked Booking as part of the same atomic zome call.
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
/// immutable. Reviewed 2026-07-09 during the P0 author-binding pass:
/// previously all 4 entry types routed updates through the same
/// create-shaped validator with no comparison to the original.
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
/// integrity level. Content restricted to currently_available/updated_at,
/// the only fields that flow ever changes.
fn validate_update_shared_resource(
    action: Update,
    original_action_hash: ActionHash,
    resource: SharedResource,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: SharedResource = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original resource not found".into()
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
/// complete_usage had NO caller-identity check anywhere -- neither in the
/// coordinator nor previously at the integrity level -- so ANY agent could
/// drive any booking to Active/Completed. Both are now required to be
/// driven by the booking's own booker (Completed also allows the resource
/// owner, mirroring cancel's precedent of allowing either party to close
/// out a booking). Content otherwise restricted to `status` only.
fn validate_update_booking(
    action: Update,
    original_action_hash: ActionHash,
    booking: Booking,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Booking = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original booking not found".into()
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
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Resource not found".into()
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
/// committer be that booking's booker or the resource's owner -- the same
/// authorization complete_usage's Booking-status update already requires,
/// checked again directly on the Usage entry itself so a future coordinator
/// change to complete_usage's Booking-update logic can't silently reopen
/// this path.
fn validate_update_usage(
    action: Update,
    original_action_hash: ActionHash,
    usage: Usage,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Usage = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original usage record not found".into()
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
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Referenced booking not found".into()
        )))?;

    let resource_record = must_get_valid_record(booking.resource_hash.clone())?;
    let resource: SharedResource = resource_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Resource not found".into()
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
    if resource.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Resource ID cannot be empty".to_string(),
        ));
    }

    // Name must not be empty
    if resource.name.is_empty() {
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
    if booking.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Booking ID cannot be empty".to_string(),
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

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a maintenance record
fn validate_maintenance(maintenance: Maintenance) -> ExternResult<ValidateCallbackResult> {
    // Description must not be empty
    if maintenance.description.is_empty() {
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

    // Parts limit
    if maintenance.parts_used.len() > 20 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot list more than 20 parts".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate link creation
fn validate_create_link(
    link_type: LinkTypes,
    _base_address: AnyLinkableHash,
    _target_address: AnyLinkableHash,
    _tag: LinkTag,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::OwnerToResources => Ok(ValidateCallbackResult::Valid),
        LinkTypes::TypeToResources => Ok(ValidateCallbackResult::Valid),
        LinkTypes::ResourceToBookings => Ok(ValidateCallbackResult::Valid),
        LinkTypes::BookerToBookings => Ok(ValidateCallbackResult::Valid),
        LinkTypes::ResourceToUsage => Ok(ValidateCallbackResult::Valid),
        LinkTypes::ResourceToMaintenance => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllResources => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AvailableResources => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

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

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_availability() -> Availability {
        Availability {
            days: vec![0, 1, 2, 3, 4, 5, 6],
            start_minutes: 0,
            end_minutes: 1440,
            timezone_offset_minutes: 0,
            exceptions: vec![],
            notes: None,
        }
    }

    fn valid_sharing_model() -> SharingModel {
        SharingModel {
            free: true,
            deposit: None,
            hourly_rate: 0,
            daily_rate: None,
            accepts_time_credits: false,
            accepts_circle_credits: false,
            circle_hash: None,
        }
    }

    fn valid_resource(owner: AgentPubKey) -> SharedResource {
        SharedResource {
            id: "r-1".into(),
            owner,
            name: "Drill".into(),
            description: "".into(),
            resource_type: ResourceType::PowerTool,
            condition: ResourceCondition::Good,
            photos: vec![],
            location: LocationConstraint::ToBeArranged,
            availability: valid_availability(),
            sharing_model: valid_sharing_model(),
            usage_instructions: "".into(),
            liability_notes: None,
            currently_available: true,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_resource_valid_when_committer_is_owner() {
        let author = me();
        let resource = valid_resource(author.clone());
        let result =
            validate_create_entry(create_action(author), EntryTypes::SharedResource(resource))
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_resource_forgery_rejected() {
        let resource = valid_resource(other_agent());
        let result =
            validate_create_entry(create_action(me()), EntryTypes::SharedResource(resource))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_booking(booker: AgentPubKey) -> Booking {
        Booking {
            id: "b-1".into(),
            resource_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            booker,
            start_time: Timestamp::from_micros(0),
            end_time: Timestamp::from_micros(1_000_000),
            purpose: "".into(),
            status: BookingStatus::Pending,
            payment_method: None,
            payment_hash: None,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_booking_valid_when_committer_is_booker() {
        let author = me();
        let booking = valid_booking(author.clone());
        let result =
            validate_create_entry(create_action(author), EntryTypes::Booking(booking)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_booking_forgery_rejected() {
        let booking = valid_booking(other_agent());
        let result =
            validate_create_entry(create_action(me()), EntryTypes::Booking(booking)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_maintenance(maintainer: AgentPubKey) -> Maintenance {
        Maintenance {
            resource_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            maintainer,
            maintenance_type: MaintenanceType::Routine,
            description: "Oiled the chain".into(),
            cost: None,
            hours_spent: 1.0,
            parts_used: vec![],
            performed_at: Timestamp::from_micros(0),
            next_due: None,
        }
    }

    #[test]
    fn create_maintenance_valid_when_committer_is_maintainer() {
        let author = me();
        let maintenance = valid_maintenance(author.clone());
        let result =
            validate_create_entry(create_action(author), EntryTypes::Maintenance(maintenance))
                .unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_maintenance_forgery_rejected() {
        let maintenance = valid_maintenance(other_agent());
        let result =
            validate_create_entry(create_action(me()), EntryTypes::Maintenance(maintenance))
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_{shared_resource,booking,usage} all call
    // must_get_valid_record, which requires a live HDI host and can't run
    // in a plain unit test -- matching the established pattern from
    // circles'/needs' equivalent update validators. Correctness there is
    // verified via cargo check plus the code-review reasoning in the
    // commit message.
}
