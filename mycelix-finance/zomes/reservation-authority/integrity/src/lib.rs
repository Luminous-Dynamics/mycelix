#![deny(unsafe_code)]

//! FIN-ECO-004B reservation authority integrity zome.
//!
//! This zome binds the pure FIN-ECO-004A reservation commitments/lifecycle
//! semantics to an actual Holochain action author. It deliberately does not
//! declare DHT presence to be generic finality and does not make a malicious
//! configured issuer honest.

use hdi::prelude::*;

mod validation;
mod wire;

pub use wire::*;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    ReservationAuthority(ReservationAuthorityEntry),
}

/// No links are part of the reservation authority theorem. A dummy link type is
/// declared only because FlatOp flattening requires a link enum; every link op
/// is rejected below.
#[hdk_link_types]
pub enum LinkTypes {
    Unsupported,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::ReservationAuthority(entry) => {
                    validation::validate_create_reservation(action, entry)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::ReservationAuthority(entry) => {
                    validation::validate_update_reservation(action, entry)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Finance reservation authority records are immutable-by-history and cannot be deleted"
                .into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FIN-ECO reservation authority does not use DHT links as scarce-state authority"
                    .into(),
            ))
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}
