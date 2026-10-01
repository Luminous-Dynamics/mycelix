// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Trusted Holochain-side qualification boundary for durable Pulse V2 inbox data.
//!
//! This module is deliberately the only place where Holochain retrieval is
//! translated into the transport-neutral pulse-social-types qualification
//! contract. It does not create a Chat store and it does not activate /chat.
//!
//! Boundary:
//! live inbox links -> explicit candidate accounting -> record validation
//! state -> action timestamp + authenticated V2 metadata -> pure projection.
//!
//! Holochain get() silently omits records that are not valid from the
//! delivering agent's perspective, so this path uses get_details() for each
//! discovered ActionHash. A successful link enumeration is treated as complete
//! for the live link set returned by the Holochain host; any candidate that
//! cannot be resolved or qualified is retained in the accounting vector and
//! prevents projection through the downstream fail-closed contract.

use hdk::prelude::*;
use mail_messages_integrity::EncryptedEmailV2;
use pulse_social_types::chat_projection::{
    AgentKeyV1, DurableEvidenceIdV1, QualifiedV2MessageV1,
};
use pulse_social_types::v2_qualification::{
    V2CandidateAccountingV1, V2CandidateDispositionV1, V2CandidateEnumerationV1,
    V2InboxQualificationV1, V2QualifiedEvidenceV1, V2RecordValidationStateV1,
};
use pulse_social_types::V2_QUALIFICATION_SCHEMA_VERSION_V1;

const INBOX_V2_TAG: &[u8] = b"inbox-v2";

/// Qualify the current agent's durable V2 inbox without projecting it into Chat.
///
/// This is intentionally not a /chat route and does not return UI state.
/// Every live V2 inbox-link candidate is represented, including missing,
/// rejected, non-V2, wrong-action, and unreadable records. Only an entirely
/// valid qualification can cross the downstream semantic projection boundary.
#[hdk_extern]
pub fn get_inbox_v2_qualification(_: ()) -> ExternResult<V2InboxQualificationV1> {
    let me = agent_info()?.agent_initial_pubkey;

    // get_links returns live links for the exact V2 inbox link type/tag.
    // A successful call is the adapter's explicit enumeration boundary. It
    // does not claim that an arbitrary remote peer has an omniscient view of
    // the DHT; it establishes the complete set returned by this host query.
    let links = get_links(
        LinkQuery::try_new(me.clone(), LinkTypes::AgentToInboxV2)?
            .tag_prefix(LinkTag::new(INBOX_V2_TAG.to_vec())),
        GetStrategy::default(),
    )?;

    let mut candidates = Vec::with_capacity(links.len());
    let mut qualified = Vec::new();
    let mut evidence = Vec::new();

    for link in links {
        let Some(hash) = link.target.clone().into_action_hash() else {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: None,
                disposition: V2CandidateDispositionV1::Unreadable,
            });
            continue;
        };

        let evidence_id = DurableEvidenceIdV1(hash.to_string());

        let Some(details) = get_details(hash.clone(), GetOptions::default())? else {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: V2CandidateDispositionV1::Missing,
            });
            continue;
        };

        let Details::Record(details) = details else {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: V2CandidateDispositionV1::WrongActionType,
            });
            continue;
        };

        let validation_state = match details.validation_status {
            ValidationStatus::Valid => V2RecordValidationStateV1::Valid,
            ValidationStatus::Rejected => V2RecordValidationStateV1::Rejected,
            _ => V2RecordValidationStateV1::Unavailable,
        };

        if validation_state != V2RecordValidationStateV1::Valid {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: if validation_state == V2RecordValidationStateV1::Rejected {
                    V2CandidateDispositionV1::Invalid
                } else {
                    V2CandidateDispositionV1::Unreadable
                },
            });
            continue;
        }

        let Action::Create(_) = details.record.action() else {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: V2CandidateDispositionV1::WrongActionType,
            });
            continue;
        };

        let Some(email) = details
            .record
            .entry()
            .to_app_option::<EncryptedEmailV2>()
            .map_err(|error| wasm_error!(error))?
        else {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: V2CandidateDispositionV1::NonV2,
            });
            continue;
        };

        // Integrity validation already binds sender to the Create action and
        // authenticates the V2 envelope. Re-check the routing identities at
        // the qualification boundary so a future adapter regression cannot
        // project a valid record into the wrong inbox.
        if details.record.action().author != email.sender || email.recipient != me {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: V2CandidateDispositionV1::Invalid,
            });
            continue;
        }

        let action_timestamp_micros = details.record.action().timestamp.as_micros();
        if action_timestamp_micros < 0 || email.created_at_micros < 0 {
            candidates.push(V2CandidateAccountingV1 {
                evidence_id: Some(evidence_id),
                disposition: V2CandidateDispositionV1::Invalid,
            });
            continue;
        }

        candidates.push(V2CandidateAccountingV1 {
            evidence_id: Some(evidence_id.clone()),
            disposition: V2CandidateDispositionV1::Valid,
        });

        qualified.push(QualifiedV2MessageV1 {
            message_id: email.message_id,
            evidence_id: evidence_id.clone(),
            sender: AgentKeyV1(email.sender.get_raw_39().to_vec()),
            recipient: AgentKeyV1(email.recipient.get_raw_39().to_vec()),
            thread_id: email.thread_id,
            in_reply_to: email.in_reply_to,
            created_at_micros: email.created_at_micros,
        });

        evidence.push(V2QualifiedEvidenceV1 {
            evidence_id,
            action_timestamp_micros,
            validation_state,
        });
    }

    let qualification = V2InboxQualificationV1 {
        schema_version: V2_QUALIFICATION_SCHEMA_VERSION_V1,
        enumeration: V2CandidateEnumerationV1::Complete,
        candidates,
        qualified,
        evidence,
    };

    Ok(qualification)
}

/// Strict qualification gate for callers that require projection-ready V2 data.
///
/// The inspection endpoint above intentionally returns the full candidate
/// accounting object even when some candidates are disqualified. This wrapper
/// is the fail-closed promotion gate: only a fully valid accounting object is
/// returned as success.
#[hdk_extern]
pub fn qualify_inbox_v2(_: ()) -> ExternResult<V2InboxQualificationV1> {
    let qualification = get_inbox_v2_qualification(())?;
    qualification
        .validate()
        .map_err(|error| wasm_error!(WasmErrorInner::Guest(format!(
            "V2 qualification failed closed: {error:?}"
        )))?;
    Ok(qualification)
}
