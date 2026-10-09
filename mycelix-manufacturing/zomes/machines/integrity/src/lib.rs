// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Machines Integrity Zome
//!
//! Entry types and validation for the machine registry.

use hdi::prelude::*;
use manufacturing_common::{MachineStatus, MachineType};

/// Maximum controller lease duration for the initial deterministic authority model.
pub const MAX_MACHINE_CONTROLLER_LEASE_MICROS: i64 = 86_400_000_000;
/// Domain-separated payload signed by the machine registrant for a controller lease.
pub const MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V1: &str =
    "mycelix-manufacturing-machine-controller-lease-v1";
pub const MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V2: &str =
    "mycelix-manufacturing-machine-controller-lease-v2";
pub const MACHINE_CONTROLLER_TRANSITION_APPROVAL_SCHEMA_ID: &str =
    "mycelix-manufacturing-machine-transition-approval-v1";
pub const MAX_MACHINE_TRANSITION_APPROVAL_MICROS: i64 = 300_000_000;

pub const MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID: &str =
    "mycelix-manufacturing-machine-time-authority-profile-v5";
pub const MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID: &str =
    "mycelix-manufacturing-machine-temporal-attestation-v4";
/// Hard upper bound used to keep temporal uncertainty arithmetic bounded.
pub const MAX_MACHINE_TEMPORAL_ACCURACY_MICROS: i64 = 86_400_000_000;
/// Maximum encoded size for the opaque external evidence commitment.
pub const MAX_MACHINE_TEMPORAL_SOURCE_COMMITMENT_BYTES: usize = 128;
/// Bounded textual identifiers carried by temporal profiles.
pub const MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES: usize = 128;
/// Bounded external evidence locator size.
pub const MAX_MACHINE_TEMPORAL_SOURCE_REFERENCE_BYTES: usize = 512;
/// Maximum distinct temporal-attestation targets hydrated and retained in one resolution.
/// The host's get_links result is materialized before this application-level bound is applied.
pub const MAX_MACHINE_TEMPORAL_EVIDENCE_OBSERVATIONS: usize = 256;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTemporalEvidenceObservation {
    pub attestation_hash: ActionHash,
    pub authority_agent: AgentPubKey,
    pub profile_hash: ActionHash,
    pub source_authority_commitment: Vec<u8>,
    pub source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget,
    pub subject_hash: ActionHash,
    pub evidence_kind: MachineTemporalEvidenceKind,
    pub attested_at: Timestamp,
    pub accuracy_micros: i64,
    pub source_reference: String,
    pub source_commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub source_commitment: Vec<u8>,
    /// Exact signed profile payload and its application-level signer statement.
    ///
    /// This permits offline verification of the registrant signature. It does not,
    /// by itself, prove that profile_hash resolves to this action: a durable archive
    /// must also retain the original Holochain record/header for action-hash binding.
    pub profile_statement: MachineTemporalSignedProfileStatement,
    /// Exact signed attestation payload and its application-level signer statement.
    ///
    /// This permits offline verification of the authority signature. It does not,
    /// by itself, prove that attestation_hash resolves to this action: a durable
    /// archive must also retain the original Holochain record/header for action-hash binding.
    pub attestation_statement: MachineTemporalSignedAttestationStatement,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineTemporalEvidenceResolution {
    /// No evidence was returned by this query.
    ///
    /// This is not proof that no evidence exists elsewhere in the DHT.
    NoEvidenceObserved,

    /// All evidence returned by this query agrees on attested time and accuracy.
    ///
    /// This is observational agreement only, not a complete network census,
    /// trust score, quorum, or assertion of witness independence.
    UniqueObserved {
        time: Timestamp,
        evidence: Vec<MachineTemporalEvidenceObservation>,
    },

    /// Evidence returned by this query disagrees on attested time or accuracy.
    /// No winner is selected.
    ConflictingObserved(Vec<MachineTemporalEvidenceObservation>),

    /// At least one validation-critical dependency or semantic invariant failed.
    InvalidEvidence,

    /// The observed evidence set exceeded the resolver's processing bound.
    /// No partial set is reported as unique or conflict-free.
    EvidenceSetLimitExceeded {
        limit: u32,
    },
}

impl MachineTemporalEvidenceObservation {
    fn signed_statements_match_observation(&self) -> bool {
        let profile = &self.profile_statement;
        let attestation = &self.attestation_statement;
        profile.action_hash == self.profile_hash
            && attestation.action_hash == self.attestation_hash
            && profile.payload.schema_id == MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID
            && attestation.payload.schema_id == MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID
            && profile.payload.machine_hash == attestation.payload.machine_hash
            && profile.payload.authority_agent == self.authority_agent
            && attestation.signer == self.authority_agent
            && profile.payload.source_authority_commitment == self.source_authority_commitment
            && profile.payload.source_authority_commitment_algorithm
                == self.source_authority_commitment_algorithm
            && profile.payload.source_authority_commitment_target
                == self.source_authority_commitment_target
            && profile.payload.commitment_algorithm == attestation.payload.source_commitment_algorithm
            && attestation.payload.profile_hash == self.profile_hash
            && attestation.payload.subject_hash == self.subject_hash
            && attestation.payload.evidence_kind == self.evidence_kind
            && attestation.payload.attested_at == self.attested_at
            && attestation.payload.accuracy_micros == self.accuracy_micros
            && attestation.payload.source_reference == self.source_reference
            && attestation.payload.source_commitment_algorithm == self.source_commitment_algorithm
            && attestation.payload.source_commitment == self.source_commitment
    }
}

pub fn resolve_temporal_evidence(
    evidence: Vec<MachineTemporalEvidenceObservation>,
) -> MachineTemporalEvidenceResolution {
    let mut by_attestation = std::collections::HashMap::new();
    for observation in evidence {
        if !observation.signed_statements_match_observation() {
            return MachineTemporalEvidenceResolution::InvalidEvidence;
        }
        match by_attestation.get(&observation.attestation_hash) {
            Some(existing) if existing != &observation => {
                return MachineTemporalEvidenceResolution::InvalidEvidence;
            }
            Some(_) => {}
            None => {
                by_attestation.insert(observation.attestation_hash.clone(), observation);
            }
        }
    }

    let mut evidence: Vec<MachineTemporalEvidenceObservation> =
        by_attestation.into_values().collect();
    if evidence.len() > MAX_MACHINE_TEMPORAL_EVIDENCE_OBSERVATIONS {
        return MachineTemporalEvidenceResolution::EvidenceSetLimitExceeded {
            limit: MAX_MACHINE_TEMPORAL_EVIDENCE_OBSERVATIONS as u32,
        };
    }
    evidence.sort_by(|left, right| {
        left.attested_at
            .as_micros()
            .cmp(&right.attested_at.as_micros())
            .then_with(|| left.attestation_hash.get_raw_36().cmp(right.attestation_hash.get_raw_36()))
    });
    match evidence.as_slice() {
        [] => MachineTemporalEvidenceResolution::NoEvidenceObserved,
        [first, rest @ ..]
            if rest.iter().all(|observation| {
                observation.attested_at == first.attested_at
                    && observation.accuracy_micros == first.accuracy_micros
            }) =>
        {
            MachineTemporalEvidenceResolution::UniqueObserved {
                time: first.attested_at,
                evidence,
            }
        }
        _ => MachineTemporalEvidenceResolution::ConflictingObserved(evidence),
    }
}
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineTemporalEvidenceKind {
    /// Time evidence attached to an exact, immutable transition approval.
    ///
    /// The temporal claim is about the approval datum and its signed validity
    /// interval. It is not an external proof that the authorized physical work
    /// occurred at attested_at.
    TransitionApproval,

    /// Time evidence attached to an exact machine action/record.
    ///
    /// The temporal claim is that the referenced datum was presented to the
    /// selected temporal authority at the attested time, subject to declared
    /// accuracy. It is not proof that a physical/business event occurred then.
    MachineActionExistence,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineTemporalCommitmentAlgorithm {
    Sha256,
    Sha384,
    Sha512,
    Sha3_256,
    Sha3_384,
    Sha3_512,
    Blake3_256,
    ProfileDefined(String),
}

impl MachineTemporalCommitmentAlgorithm {
    fn is_empty(&self) -> bool {
        matches!(self, Self::ProfileDefined(value) if value.trim().is_empty())
    }

    fn text_len(&self) -> usize {
        match self {
            Self::ProfileDefined(value) => value.len(),
            _ => 0,
        }
    }

    fn expected_digest_len(&self) -> Option<usize> {
        match self {
            Self::Sha256 | Self::Sha3_256 | Self::Blake3_256 => Some(32),
            Self::Sha384 | Self::Sha3_384 => Some(48),
            Self::Sha512 | Self::Sha3_512 => Some(64),
            Self::ProfileDefined(_) => None,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum MachineTemporalAuthorityCommitmentTarget {
    CertificateDer,
    SubjectPublicKeyInfo,
    PublicKey,
    ProfileDefined(String),
}

impl MachineTemporalAuthorityCommitmentTarget {
    fn is_empty(&self) -> bool {
        matches!(self, Self::ProfileDefined(value) if value.trim().is_empty())
    }

    fn text_len(&self) -> usize {
        match self {
            Self::ProfileDefined(value) => value.len(),
            _ => 0,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTimeAuthorityProfilePayload {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub authority_agent: AgentPubKey,
    pub profile_id: String,
    pub source_profile: String,
    pub source_authority_commitment: Vec<u8>,
    pub source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget,
    pub commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub max_accuracy_micros: i64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTemporalAttestationPayload {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub profile_hash: ActionHash,
    pub subject_hash: ActionHash,
    pub evidence_kind: MachineTemporalEvidenceKind,
    pub attested_at: Timestamp,
    pub accuracy_micros: i64,
    pub source_reference: String,
    pub source_commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub source_commitment: Vec<u8>,
}

/// Portable application-level signed statement for a time-authority profile.
///
/// The original Holochain record/action must still be archived to bind action_hash
/// to this statement and preserve the action's own signature/header.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTemporalSignedProfileStatement {
    pub action_hash: ActionHash,
    pub signer: AgentPubKey,
    pub payload: MachineTimeAuthorityProfilePayload,
    pub signature: Signature,
}

/// Portable application-level signed statement for a temporal attestation.
///
/// The original Holochain record/action must still be archived to bind action_hash
/// to this statement and preserve the action's own signature/header.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineTemporalSignedAttestationStatement {
    pub action_hash: ActionHash,
    pub signer: AgentPubKey,
    pub payload: MachineTemporalAttestationPayload,
    pub signature: Signature,
}

fn default_lease_schema_version() -> u8 {
    1
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineControllerLeasePayloadV1 {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineControllerLeasePayloadV2 {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub requires_transition_approval: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct MachineControllerTransitionApprovalPayload {
    pub schema_id: String,
    pub machine_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub predecessor_action: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
}
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineEntry {
    pub name: String,
    pub machine_type: MachineType,
    pub capabilities: Vec<String>,
    pub location: String,
    pub max_throughput_per_hour: u32,
    pub status: MachineStatus,
    pub current_work_order: Option<ActionHash>,
    #[serde(default)]
    pub last_status_authority_hash: Option<ActionHash>,
    #[serde(default)]
    pub last_status_transition_approval_hash: Option<ActionHash>,
    pub registered_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineTimeAuthorityProfileEntry {
    pub machine_hash: ActionHash,
    pub authority_agent: AgentPubKey,
    pub profile_id: String,
    pub source_profile: String,
    pub source_authority_commitment: Vec<u8>,
    pub source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget,
    pub commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub max_accuracy_micros: i64,
    pub registrant_signature: Signature,
}

impl MachineTimeAuthorityProfileEntry {
    pub fn signed_payload(&self) -> MachineTimeAuthorityProfilePayload {
        MachineTimeAuthorityProfilePayload {
            schema_id: MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID.to_string(),
            machine_hash: self.machine_hash.clone(),
            authority_agent: self.authority_agent.clone(),
            profile_id: self.profile_id.clone(),
            source_profile: self.source_profile.clone(),
            source_authority_commitment: self.source_authority_commitment.clone(),
            source_authority_commitment_algorithm: self.source_authority_commitment_algorithm.clone(),
            source_authority_commitment_target: self.source_authority_commitment_target.clone(),
            commitment_algorithm: self.commitment_algorithm.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
            max_accuracy_micros: self.max_accuracy_micros,
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineTemporalAttestationEntry {
    pub machine_hash: ActionHash,
    pub profile_hash: ActionHash,
    pub subject_hash: ActionHash,
    pub evidence_kind: MachineTemporalEvidenceKind,
    pub attested_at: Timestamp,
    pub accuracy_micros: i64,
    pub source_reference: String,
    pub source_commitment_algorithm: MachineTemporalCommitmentAlgorithm,
    pub source_commitment: Vec<u8>,
    pub authority_signature: Signature,
}

impl MachineTemporalAttestationEntry {
    pub fn signed_payload(&self) -> MachineTemporalAttestationPayload {
        MachineTemporalAttestationPayload {
            schema_id: MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID.to_string(),
            machine_hash: self.machine_hash.clone(),
            profile_hash: self.profile_hash.clone(),
            subject_hash: self.subject_hash.clone(),
            evidence_kind: self.evidence_kind.clone(),
            attested_at: self.attested_at,
            accuracy_micros: self.accuracy_micros,
            source_reference: self.source_reference.clone(),
            source_commitment_algorithm: self.source_commitment_algorithm.clone(),
            source_commitment: self.source_commitment.clone(),
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineControllerAuthorityEntry {
    pub machine_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    #[serde(default = "default_lease_schema_version")]
    pub lease_schema_version: u8,
    #[serde(default)]
    pub requires_transition_approval: bool,
    /// Cryptographic proof that the machine registrant authored the exact lease fields.
    /// None is migration compatibility only; unsigned legacy authorities cannot be
    /// used for new control-field updates.
    #[serde(default)]
    pub issuer_signature: Option<Signature>,
}

impl MachineControllerAuthorityEntry {
    pub fn signed_payload_v1(&self) -> MachineControllerLeasePayloadV1 {
        MachineControllerLeasePayloadV1 {
            schema_id: MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V1.to_string(),
            machine_hash: self.machine_hash.clone(),
            controller_agent: self.controller_agent.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
        }
    }

    pub fn signed_payload_v2(&self) -> MachineControllerLeasePayloadV2 {
        MachineControllerLeasePayloadV2 {
            schema_id: MACHINE_CONTROLLER_LEASE_SCHEMA_ID_V2.to_string(),
            machine_hash: self.machine_hash.clone(),
            controller_agent: self.controller_agent.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
            requires_transition_approval: true,
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineControllerTransitionApprovalEntry {
    pub machine_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub controller_agent: AgentPubKey,
    pub predecessor_action: ActionHash,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub issuer_signature: Signature,
}

impl MachineControllerTransitionApprovalEntry {
    pub fn signed_payload(&self) -> MachineControllerTransitionApprovalPayload {
        MachineControllerTransitionApprovalPayload {
            schema_id: MACHINE_CONTROLLER_TRANSITION_APPROVAL_SCHEMA_ID.to_string(),
            machine_hash: self.machine_hash.clone(),
            authority_hash: self.authority_hash.clone(),
            controller_agent: self.controller_agent.clone(),
            predecessor_action: self.predecessor_action.clone(),
            new_status: self.new_status.clone(),
            work_order_hash: self.work_order_hash.clone(),
            valid_from: self.valid_from,
            valid_until: self.valid_until,
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MachineStatusLog {
    pub machine_hash: ActionHash,
    pub machine_update_hash: ActionHash,
    pub authority_hash: ActionHash,
    pub previous_status: MachineStatus,
    pub new_status: MachineStatus,
    pub work_order_hash: Option<ActionHash>,
    #[serde(default)]
    pub transition_approval_hash: Option<ActionHash>,
    pub changed_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Machine(MachineEntry),
    StatusLog(MachineStatusLog),
    MachineControllerAuthority(MachineControllerAuthorityEntry),
    MachineControllerTransitionApproval(MachineControllerTransitionApprovalEntry),
    MachineTimeAuthorityProfile(MachineTimeAuthorityProfileEntry),
    MachineTemporalAttestation(MachineTemporalAttestationEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMachines,
    TypeToMachines,
    MachineToStatusLog,
    MachineUpdateToStatusLog,
    MachineToAuthorities,
    AllMachineControllerAuthorities,
    MachineToTransitionApprovals,
    AllMachineTransitionApprovals,
    MachineToTimeAuthorityProfiles,
    AllMachineTimeAuthorityProfiles,
    MachineToTemporalAttestations,
    AllMachineTemporalAttestations,
    SubjectToTemporalAttestations,
    LocationToMachines,
}

/// **P0 author-binding pass, 2026-07-09**: no identity field exists on
/// either entry (case a -- shared machine registry, not per-agent-owned).
/// What IS fixed: updates were previously routed through the wide-open
/// catch-all `_ => Valid` -- `update_machine_status`'s `update_entry`
/// call was accepted with zero validation, so a modified coordinator
/// could silently rewrite ANY field (name, machine_type, capabilities,
/// location, max_throughput_per_hour), not just the intended
/// status/current_work_order, and could skip the state-machine
/// transition check entirely. Fixed via must_get content-restriction
/// plus re-running `can_transition_to`.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, action }) => {
            match app_entry {
                EntryTypes::MachineControllerAuthority(authority) => {
                    validate_create_authority(action, authority)
                }
                EntryTypes::StatusLog(log) => validate_create_status_log(action, log),
                EntryTypes::MachineControllerTransitionApproval(approval) => {
                    validate_create_transition_approval(action, approval)
                }
                EntryTypes::MachineTimeAuthorityProfile(profile) => {
                    validate_create_time_authority_profile(action, profile)
                }
                EntryTypes::MachineTemporalAttestation(attestation) => {
                    validate_create_temporal_attestation(action, attestation)
                }
                other => validate_create_entry(other),
            }
        }
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            action,
            ..
        }) => validate_update_entry(original_action_hash, action, app_entry),
        FlatOp::RegisterUpdate(OpUpdate::Entry {
            app_entry, action, ..
        }) => validate_update_entry(action.original_action_address, action, app_entry),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine registry records are immutable".into(),
        )),
        FlatOp::Link(OpLink::CreateLink { link_type, action }) => match link_type {
            LinkTypes::SubjectToTemporalAttestations => {
                validate_create_temporal_attestation_link(action)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::Link(OpLink::DeleteLink { link_type, .. }) => match link_type {
            LinkTypes::SubjectToTemporalAttestations => Ok(ValidateCallbackResult::Invalid(
                "Temporal attestation subject links are append-only".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_temporal_attestation_link(
    action: TypedAction<CreateLinkData>,
) -> ExternResult<ValidateCallbackResult> {
    let Some(subject_hash) = action.data.base_address.clone().into_action_hash() else {
        return Ok(ValidateCallbackResult::Invalid(
            "Temporal attestation subject link base must be an action hash".into(),
        ));
    };
    let Some(attestation_hash) = action.data.target_address.clone().into_action_hash() else {
        return Ok(ValidateCallbackResult::Invalid(
            "Temporal attestation subject link target must be an action hash".into(),
        ));
    };

    let attestation_record = must_get_valid_record(attestation_hash)?;
    let Some(attestation) = attestation_record
        .entry()
        .to_app_option::<MachineTemporalAttestationEntry>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Temporal attestation subject link target is not a temporal attestation".into(),
        ));
    };

    if attestation.subject_hash != subject_hash
        || attestation_record.action().author() != action.author()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Temporal attestation subject link must match its exact attestation and author".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_time_authority_profile(
    action: TypedAction<CreateData>,
    profile: MachineTimeAuthorityProfileEntry,
) -> ExternResult<ValidateCallbackResult> {
    if profile.profile_id.trim().is_empty()
        || profile.profile_id.len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES
        || profile.source_profile.trim().is_empty()
        || profile.source_profile.len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES
        || profile.source_authority_commitment.is_empty()
        || profile.source_authority_commitment.len() > MAX_MACHINE_TEMPORAL_SOURCE_COMMITMENT_BYTES
        || profile.max_accuracy_micros < 0
        || profile.max_accuracy_micros > MAX_MACHINE_TEMPORAL_ACCURACY_MICROS
        || profile.source_authority_commitment_algorithm.is_empty()
        || profile.source_authority_commitment_algorithm.text_len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES
        || !digest_length_matches(
            &profile.source_authority_commitment_algorithm,
            profile.source_authority_commitment.len(),
        )
        || profile.commitment_algorithm.is_empty()
        || profile.commitment_algorithm.text_len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES
        || profile.source_authority_commitment_target.is_empty()
        || profile.source_authority_commitment_target.text_len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES
    {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile has invalid identity or accuracy bounds".into(),
        ));
    }
    if profile.valid_until < profile.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile validity window is inverted".into(),
        ));
    }
    let machine_record = must_get_valid_record(profile.machine_hash.clone())?;
    let machine: Option<MachineEntry> = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if !matches!(machine_record.action(), Action::Create(_)) || machine.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile must bind to a machine root entry".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may establish a time authority profile".into(),
        ));
    }
    if !verify_signature(
        action.author().clone(),
        profile.registrant_signature.clone(),
        profile.signed_payload(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "time authority profile registrant signature is invalid".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_temporal_attestation(
    action: TypedAction<CreateData>,
    attestation: MachineTemporalAttestationEntry,
) -> ExternResult<ValidateCallbackResult> {
    if attestation.source_reference.trim().is_empty()
        || attestation.source_reference.len() > MAX_MACHINE_TEMPORAL_SOURCE_REFERENCE_BYTES
        || attestation.source_commitment_algorithm.is_empty()
        || attestation.source_commitment_algorithm.text_len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES
        || !digest_length_matches(
            &attestation.source_commitment_algorithm,
            attestation.source_commitment.len(),
        )
        || attestation.source_commitment.is_empty()
        || attestation.source_commitment.len() > MAX_MACHINE_TEMPORAL_SOURCE_COMMITMENT_BYTES
        || attestation.accuracy_micros < 0
        || attestation.accuracy_micros > MAX_MACHINE_TEMPORAL_ACCURACY_MICROS
    {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation requires bounded source reference and commitment".into(),
        ));
    }
    let profile_record = must_get_valid_record(attestation.profile_hash.clone())?;
    let profile: Option<MachineTimeAuthorityProfileEntry> = profile_record
        .entry().to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(profile) = profile else {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation profile reference is not a time authority profile".into(),
        ));
    };
    if profile.machine_hash != attestation.machine_hash
        || profile.authority_agent != action.author()
        || attestation.accuracy_micros > profile.max_accuracy_micros
        || attestation.source_commitment_algorithm != profile.commitment_algorithm
        || !temporal_profile_contains_interval(
            &profile,
            attestation.attested_at,
            attestation.accuracy_micros,
        )
    {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation is outside its exact machine/authority profile".into(),
        ));
    }
    let subject_record = must_get_valid_record(attestation.subject_hash.clone())?;
    match attestation.evidence_kind {
        MachineTemporalEvidenceKind::TransitionApproval => {
            let approval: Option<MachineControllerTransitionApprovalEntry> = subject_record
                .entry().to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            let Some(approval) = approval else {
                return Ok(ValidateCallbackResult::Invalid(
                    "temporal approval evidence subject is not a transition approval".into(),
                ));
            };
            if approval.machine_hash != attestation.machine_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "temporal approval evidence subject belongs to a different machine".into(),
                ));
            }
            if !temporal_interval_contains_interval(
                approval.valid_from,
                approval.valid_until,
                attestation.attested_at,
                attestation.accuracy_micros,
            ) {
                return Ok(ValidateCallbackResult::Invalid(
                    "temporal approval evidence falls outside approval validity".into(),
                ));
            }
        }
        MachineTemporalEvidenceKind::MachineActionExistence => {
            let machine: Option<MachineEntry> = subject_record
                .entry().to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
            let Some(_) = machine else {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine action temporal evidence subject is not a machine record".into(),
                ));
            };
            let root = resolve_machine_root_action_hash(attestation.subject_hash.clone())?;
            if root != attestation.machine_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine action temporal evidence subject belongs to a different machine".into(),
                ));
            }
        }
    }
    if !verify_signature(
        profile.authority_agent.clone(),
        attestation.authority_signature.clone(),
        attestation.signed_payload(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "temporal attestation authority signature is invalid".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_transition_approval(
    action: TypedAction<CreateData>,
    approval: MachineControllerTransitionApprovalEntry,
) -> ExternResult<ValidateCallbackResult> {
    if approval.valid_until < approval.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval validity window is inverted".into(),
        ));
    }
    let Some(duration) = approval
        .valid_until
        .as_micros()
        .checked_sub(approval.valid_from.as_micros())
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval validity arithmetic overflow".into(),
        ));
    };
    if duration > MAX_MACHINE_TRANSITION_APPROVAL_MICROS {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval exceeds the maximum 5-minute validity".into(),
        ));
    }

    let authority_record = must_get_valid_record(approval.authority_hash.clone())?;
    let authority: Option<MachineControllerAuthorityEntry> = authority_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(authority) = authority else {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval references a non-authority record".into(),
        ));
    };
    if authority.lease_schema_version != 2 || !authority.requires_transition_approval {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval requires a schema-v2 controller lease".into(),
        ));
    }
    if authority.machine_hash != approval.machine_hash
        || authority.controller_agent != approval.controller_agent
    {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval does not match the controller authority".into(),
        ));
    }
    if approval.valid_from < authority.valid_from || approval.valid_until > authority.valid_until {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval interval exceeds controller lease interval".into(),
        ));
    }

    let machine_record = must_get_valid_record(approval.machine_hash.clone())?;
    let machine: Option<MachineEntry> = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if machine.is_none() || !matches!(machine_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval must target a machine root".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may create transition approvals".into(),
        ));
    }
    if !verify_signature(
        machine_record.action().author().clone(),
        approval.issuer_signature.clone(),
        approval.signed_payload(),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval issuer signature does not match the exact approval payload".into(),
        ));
    }

    let predecessor_record = must_get_valid_record(approval.predecessor_action.clone())?;
    if !matches!(predecessor_record.action(), Action::Create(_) | Action::Update(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval predecessor is not a machine entry action".into(),
        ));
    }
    let predecessor_machine: MachineEntry = predecessor_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "transition approval predecessor is not a machine record".into(),
        )))?;
    if !predecessor_machine.status.can_transition_to(&approval.new_status) {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval does not describe a valid machine transition".into(),
        ));
    }
    let predecessor_root = resolve_machine_root_action_hash(approval.predecessor_action.clone())?;
    if predecessor_root != approval.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval predecessor belongs to a different machine".into(),
        ));
    }
    if !authority_valid_at(&authority, approval.valid_from)
        || !authority_valid_at(&authority, approval.valid_until)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "transition approval interval is outside controller authority validity".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_authority(
    action: TypedAction<CreateData>,
    authority: MachineControllerAuthorityEntry,
) -> ExternResult<ValidateCallbackResult> {
    if authority.lease_schema_version != 1 && authority.lease_schema_version != 2 {
        return Ok(ValidateCallbackResult::Invalid("unsupported machine controller lease schema version".into()));
    }
    if authority.lease_schema_version == 2 && !authority.requires_transition_approval {
        return Ok(ValidateCallbackResult::Invalid("lease schema v2 requires per-transition approval".into()));
    }
    if authority.valid_until < authority.valid_from {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority validity window is inverted".into(),
        ));
    }
    if !authority_duration_is_bounded(&authority) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority exceeds the maximum 24-hour lease duration".into(),
        ));
    }
    let Some(signature) = authority.issuer_signature.clone() else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority requires a registrant signature".into(),
        ));
    };

    let machine_record = must_get_valid_record(authority.machine_hash.clone())?;
    let machine: Option<MachineEntry> = machine_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    if machine.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority references a non-machine record".into(),
        ));
    }
    if !matches!(machine_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority must bind to the machine's root creation action".into(),
        ));
    }
    if machine_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "only the machine registrant may issue controller authority".into(),
        ));
    }
        let signature_valid = match authority.lease_schema_version {
        1 => verify_signature(
            action.author().clone(),
            signature,
            authority.signed_payload_v1(),
        )?,
        2 => verify_signature(
            action.author().clone(),
            signature,
            authority.signed_payload_v2(),
        )?,
        _ => false,
    };
    if !signature_valid {
        return Ok(ValidateCallbackResult::Invalid(
            "machine controller authority issuer signature does not match the lease payload".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_status_log(
    action: TypedAction<CreateData>,
    log: MachineStatusLog,
) -> ExternResult<ValidateCallbackResult> {
    if !log.previous_status.can_transition_to(&log.new_status) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid machine transition: {:?} -> {:?}",
            log.previous_status, log.new_status
        )));
    }

    let authority_record = must_get_valid_record(log.authority_hash.clone())?;
    let authority: Option<MachineControllerAuthorityEntry> = authority_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(authority) = authority else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status authority reference is not an authority record".into(),
        ));
    };

    if authority.machine_hash != log.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status authority is bound to a different machine".into(),
        ));
    }
    if authority.issuer_signature.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "unsigned legacy controller authority cannot authorize new machine status".into(),
        ));
    }
    if authority.requires_transition_approval && log.transition_approval_hash.is_none() {
        return Ok(ValidateCallbackResult::Invalid(
            "schema-v2 controller status requires a transition approval".into(),
        ));
    }
    if !authority.requires_transition_approval && log.transition_approval_hash.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "legacy controller status cannot carry a transition approval".into(),
        ));
    }
    if action.author() != authority.controller_agent {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status action author is not an authorized controller".into(),
        ));
    }
    if !authority_valid_at(&authority, action.timestamp()) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status action falls outside controller authority validity".into(),
        ));
    }
    if log.changed_at != action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status changed_at must equal the action timestamp".into(),
        ));
    }

    let update_record = must_get_valid_record(log.machine_update_hash.clone())?;
    if !matches!(update_record.action(), Action::Update(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log must reference a machine update action".into(),
        ));
    }
    let updated_machine: Option<MachineEntry> = update_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
    let Some(updated_machine) = updated_machine else {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log update reference is not a machine record".into(),
        ));
    };

    let update_root = resolve_machine_root_action_hash(log.machine_update_hash.clone())?;
    if update_root != log.machine_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log update belongs to a different machine".into(),
        ));
    }
    if update_record.action().author() != action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log and machine update have different authors".into(),
        ));
    }
    if update_record.action().timestamp() > action.timestamp() {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log cannot predate its referenced machine update".into(),
        ));
    }
    if updated_machine.status != log.new_status
        || updated_machine.current_work_order != log.work_order_hash
        || updated_machine.last_status_authority_hash != Some(log.authority_hash.clone())
        || (authority.requires_transition_approval
            && updated_machine.last_status_transition_approval_hash != log.transition_approval_hash)
    {
        return Ok(ValidateCallbackResult::Invalid(
            "machine status log does not match its referenced machine update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_entry(entry: EntryTypes) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Machine(m) => {
            if m.name.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine name is required".into(),
                ));
            }
            if m.location.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "machine location is required".into(),
                ));
            }
            if m.max_throughput_per_hour == 0 {
                return Ok(ValidateCallbackResult::Invalid(
                    "max_throughput_per_hour must be > 0".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusLog(log) => {
            if !log.previous_status.can_transition_to(&log.new_status) {
                return Ok(ValidateCallbackResult::Invalid(format!(
                    "Invalid machine transition: {:?} -> {:?}",
                    log.previous_status, log.new_status
                )));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::MachineControllerAuthority(_) => unreachable!(),
        EntryTypes::MachineControllerTransitionApproval(_) => unreachable!(),
        EntryTypes::MachineTimeAuthorityProfile(_) => unreachable!(),
        EntryTypes::MachineTemporalAttestation(_) => unreachable!(),
    }
}

fn resolve_machine_root_action_hash(
    start: ActionHash,
) -> ExternResult<ActionHash> {
    let mut current = start;
    for _ in 0..4096 {
        let record = must_get_valid_record(current.clone())?;
        match record.action() {
            Action::Create(_) => {
                let root_record = must_get_valid_record(current.clone())?;
                let root_machine: Option<MachineEntry> = root_record
                    .entry()
                    .to_app_option()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                if root_machine.is_none() {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "machine update lineage terminates at a non-machine root".into(),
                    )));
                }
                return Ok(current);
            }
            Action::Update(update) => {
                current = update.original_action_address.clone();
            }
            _ => {
                return Err(wasm_error!(WasmErrorInner::Guest(
                    "machine update lineage terminated at a non-entry action".into(),
                )));
            }
        }
    }

    Err(wasm_error!(WasmErrorInner::Guest(
        "machine update lineage exceeds safety bound".into(),
    )))
}

fn machine_control_fields_changed(
    original: &MachineEntry,
    updated: &MachineEntry,
) -> bool {
    original.status != updated.status
        || original.current_work_order != updated.current_work_order
        || original.last_status_authority_hash != updated.last_status_authority_hash
        || original.last_status_transition_approval_hash != updated.last_status_transition_approval_hash
}

fn transition_approval_matches_update(
    approval: &MachineControllerTransitionApprovalEntry,
    machine_root: &ActionHash,
    authority_hash: &ActionHash,
    controller_agent: &AgentPubKey,
    predecessor_action: &ActionHash,
    new_status: &MachineStatus,
    work_order_hash: Option<&ActionHash>,
    action_timestamp: Timestamp,
) -> bool {
    approval.machine_hash == *machine_root
        && approval.authority_hash == *authority_hash
        && approval.controller_agent == *controller_agent
        && approval.predecessor_action == *predecessor_action
        && approval.new_status == *new_status
        && approval.work_order_hash.as_ref() == work_order_hash
        && approval_valid_at(approval, action_timestamp)
}
fn temporal_interval_contains(
    valid_from: Timestamp,
    valid_until: Timestamp,
    timestamp: Timestamp,
) -> bool {
    valid_from <= timestamp && timestamp <= valid_until
}

fn digest_length_matches(
    algorithm: &MachineTemporalCommitmentAlgorithm,
    digest_len: usize,
) -> bool {
    algorithm
        .expected_digest_len()
        .map(|expected| expected == digest_len)
        .unwrap_or(true)
}

fn temporal_interval_bounds(
    timestamp: Timestamp,
    accuracy_micros: i64,
) -> Option<(Timestamp, Timestamp)> {
    if accuracy_micros < 0 || accuracy_micros > MAX_MACHINE_TEMPORAL_ACCURACY_MICROS {
        return None;
    }
    let micros = timestamp.as_micros();
    let lower = micros.checked_sub(accuracy_micros)?;
    let upper = micros.checked_add(accuracy_micros)?;
    Some((Timestamp::from_micros(lower), Timestamp::from_micros(upper)))
}

fn temporal_interval_contains_interval(
    valid_from: Timestamp,
    valid_until: Timestamp,
    timestamp: Timestamp,
    accuracy_micros: i64,
) -> bool {
    temporal_interval_bounds(timestamp, accuracy_micros)
        .map(|(lower, upper)| valid_from <= lower && upper <= valid_until)
        .unwrap_or(false)
}

fn temporal_profile_contains_interval(
    profile: &MachineTimeAuthorityProfileEntry,
    timestamp: Timestamp,
    accuracy_micros: i64,
) -> bool {
    temporal_interval_contains_interval(
        profile.valid_from,
        profile.valid_until,
        timestamp,
        accuracy_micros,
    )
}

fn temporal_profile_contains(
    profile: &MachineTimeAuthorityProfileEntry,
    timestamp: Timestamp,
) -> bool {
    temporal_interval_contains(profile.valid_from, profile.valid_until, timestamp)
}
fn authority_valid_at(
    authority: &MachineControllerAuthorityEntry,
    timestamp: Timestamp,
) -> bool {
    authority.valid_from <= timestamp && timestamp <= authority.valid_until
}

fn authority_duration_is_bounded(
    authority: &MachineControllerAuthorityEntry,
) -> bool {
    let Some(duration) = authority
        .valid_until
        .as_micros()
        .checked_sub(authority.valid_from.as_micros())
    else {
        return false;
    };
    duration <= MAX_MACHINE_CONTROLLER_LEASE_MICROS
}

fn validate_update_entry(
    original_action_hash: ActionHash,
    action: TypedAction<UpdateData>,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Machine(m) => {
            let original_record = must_get_valid_record(original_action_hash)?;
            let original: MachineEntry = original_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Original machine not found".into()
                )))?;

            if m.name != original.name
                || m.machine_type != original.machine_type
                || m.capabilities != original.capabilities
                || m.location != original.location
                || m.max_throughput_per_hour != original.max_throughput_per_hour
                || m.registered_at != original.registered_at
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only status/current_work_order/last_status_authority_hash/last_status_transition_approval_hash can change on a machine update".into(),
                ));
            }

            if machine_control_fields_changed(&original, &m) {
                let Some(authority_hash) = m.last_status_authority_hash.clone() else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine control-field changes require controller authority".into(),
                    ));
                };
                let authority_record = must_get_valid_record(authority_hash)?;
                let authority: Option<MachineControllerAuthorityEntry> = authority_record
                    .entry()
                    .to_app_option()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                let Some(authority) = authority else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status authority reference is not an authority record".into(),
                    ));
                };

                let machine_root = resolve_machine_root_action_hash(
                    original_action_hash.clone(),
                )?;
                if authority.machine_hash != machine_root {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine status authority is bound to a different machine".into(),
                    ));
                }
                if authority.issuer_signature.is_none() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "unsigned legacy controller authority cannot authorize new machine updates".into(),
                    ));
                }
                if !authority.requires_transition_approval
                    && m.last_status_transition_approval_hash.is_some()
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "legacy controller update cannot carry a transition approval".into(),
                    ));
                }
                if authority.requires_transition_approval {
                    let Some(approval_hash) = m.last_status_transition_approval_hash.clone() else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "schema-v2 controller updates require a transition approval".into(),
                        ));
                    };
                    let approval_record = must_get_valid_record(approval_hash)?;
                    let approval: Option<MachineControllerTransitionApprovalEntry> = approval_record
                        .entry().to_app_option()
                        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?;
                    let Some(approval) = approval else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "transition approval reference is not an approval record".into(),
                        ));
                    };
                    if approval.valid_from < authority.valid_from
                        || approval.valid_until > authority.valid_until
                        || !transition_approval_matches_update(
                            &approval,
                            &machine_root,
                            &authority_hash,
                            action.author(),
                            &original_action_hash,
                            &m.status,
                            m.current_work_order.as_ref(),
                            action.timestamp(),
                        )
                    {
                        return Ok(ValidateCallbackResult::Invalid(
                            "transition approval does not exactly authorize this machine update".into(),
                        ));
                    }
                    let machine_root_record = must_get_valid_record(machine_root.clone())?;
                    if !verify_signature(
                        machine_root_record.action().author().clone(),
                        approval.issuer_signature.clone(),
                        approval.signed_payload(),
                    )? {
                        return Ok(ValidateCallbackResult::Invalid(
                            "transition approval signature is invalid for this machine registrant".into(),
                        ));
                    }
                }
                if action.author() != authority.controller_agent {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine update author is not an authorized controller".into(),
                    ));
                }
                if !authority_valid_at(&authority, action.timestamp()) {
                    return Ok(ValidateCallbackResult::Invalid(
                        "machine update falls outside controller authority validity".into(),
                    ));
                }
                if m.status != original.status
                    && !original.status.can_transition_to(&m.status)
                {
                    return Ok(ValidateCallbackResult::Invalid(format!(
                        "Invalid machine transition: {:?} -> {:?}",
                        original.status, m.status
                    )));
                }
            }

            Ok(ValidateCallbackResult::Valid)
        }
        EntryTypes::StatusLog(_) => Ok(ValidateCallbackResult::Invalid(
            "Status log records are immutable".into(),
        )),
        EntryTypes::MachineControllerAuthority(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine controller authorities are immutable".into(),
        )),
        EntryTypes::MachineControllerTransitionApproval(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine transition approvals are immutable".into(),
        )),
        EntryTypes::MachineTimeAuthorityProfile(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine time authority profiles are immutable".into(),
        )),
        EntryTypes::MachineTemporalAttestation(_) => Ok(ValidateCallbackResult::Invalid(
            "Machine temporal attestations are immutable".into(),
        )),
    }
}

#[cfg(test)]
mod content_restriction_tests {
    use super::*;

    fn valid_machine() -> MachineEntry {
        MachineEntry {
            name: "Mill-1".into(),
            machine_type: MachineType::CNC3Axis,
            capabilities: vec![],
            location: "Bay 1".into(),
            max_throughput_per_hour: 10,
            status: MachineStatus::Available,
            current_work_order: None,
            last_status_authority_hash: None,
            last_status_transition_approval_hash: None,
            registered_at: Timestamp::from_micros(0),
        }
    }

    fn sample_approval() -> MachineControllerTransitionApprovalEntry {
        let valid_from = Timestamp::from_micros(100);
        MachineControllerTransitionApprovalEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_hash: ActionHash::from_raw_36(vec![2; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![3; 32]),
            predecessor_action: ActionHash::from_raw_36(vec![4; 36]),
            new_status: MachineStatus::Running,
            work_order_hash: Some(ActionHash::from_raw_36(vec![5; 36])),
            valid_from,
            valid_until: Timestamp::from_micros(200),
            issuer_signature: Signature(vec![0; 64]),
        }
    }

    #[test]
    fn transition_approval_binds_exact_update_tuple() {
        let approval = sample_approval();
        let machine = ActionHash::from_raw_36(vec![1; 36]);
        let authority = ActionHash::from_raw_36(vec![2; 36]);
        let controller = AgentPubKey::from_raw_32(vec![3; 32]);
        let predecessor = ActionHash::from_raw_36(vec![4; 36]);
        let work_order = ActionHash::from_raw_36(vec![5; 36]);
        assert!(transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(150)
        ));

        let wrong_machine = ActionHash::from_raw_36(vec![9; 36]);
        assert!(!transition_approval_matches_update(
            &approval, &wrong_machine, &authority, &controller, &predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(150)
        ));

        let wrong_predecessor = ActionHash::from_raw_36(vec![8; 36]);
        assert!(!transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &wrong_predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(150)
        ));

        assert!(!transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &predecessor,
            &MachineStatus::Maintenance, Some(&work_order), Timestamp::from_micros(150)
        ));

        assert!(!transition_approval_matches_update(
            &approval, &machine, &authority, &controller, &predecessor,
            &MachineStatus::Running, Some(&work_order), Timestamp::from_micros(201)
        ));
    }
    #[test]
    fn temporal_evidence_kind_serde_round_trips() {
        for kind in [
            MachineTemporalEvidenceKind::TransitionApproval,
            MachineTemporalEvidenceKind::MachineActionExistence,
        ] {
            let encoded = serde_json::to_string(&kind).unwrap();
            let decoded: MachineTemporalEvidenceKind = serde_json::from_str(&encoded).unwrap();
            assert_eq!(decoded, kind);
        }
    }

    fn temporal_observation(
        attestation_byte: u8,
        authority_byte: u8,
        time: i64,
    ) -> MachineTemporalEvidenceObservation {
        let attestation_hash = ActionHash::from_raw_36(vec![attestation_byte; 36]);
        let profile_hash = ActionHash::from_raw_36(vec![3; 36]);
        let machine_hash = ActionHash::from_raw_36(vec![1; 36]);
        let authority_agent = AgentPubKey::from_raw_32(vec![authority_byte; 32]);
        let source_reference = format!("tsa://example/{attestation_byte}");
        let source_commitment = vec![attestation_byte; 32];
        let profile_payload = MachineTimeAuthorityProfilePayload {
            schema_id: MACHINE_TIME_AUTHORITY_PROFILE_SCHEMA_ID.to_string(),
            machine_hash: machine_hash.clone(),
            authority_agent: authority_agent.clone(),
            profile_id: "profile-1".into(),
            source_profile: "rfc3161".into(),
            source_authority_commitment: vec![9; 32],
            source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget::CertificateDer,
            commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(1_000),
            max_accuracy_micros: 5,
        };
        let attestation_payload = MachineTemporalAttestationPayload {
            schema_id: MACHINE_TEMPORAL_ATTESTATION_SCHEMA_ID.to_string(),
            machine_hash,
            profile_hash: profile_hash.clone(),
            subject_hash: ActionHash::from_raw_36(vec![4; 36]),
            evidence_kind: MachineTemporalEvidenceKind::TransitionApproval,
            attested_at: Timestamp::from_micros(time),
            accuracy_micros: 5,
            source_reference: source_reference.clone(),
            source_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_commitment: source_commitment.clone(),
        };
        MachineTemporalEvidenceObservation {
            attestation_hash: attestation_hash.clone(),
            authority_agent: authority_agent.clone(),
            profile_hash: profile_hash.clone(),
            source_authority_commitment: vec![9; 32],
            source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget::CertificateDer,
            subject_hash: ActionHash::from_raw_36(vec![4; 36]),
            evidence_kind: MachineTemporalEvidenceKind::TransitionApproval,
            attested_at: Timestamp::from_micros(time),
            accuracy_micros: 5,
            source_reference,
            source_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_commitment,
            profile_statement: MachineTemporalSignedProfileStatement {
                action_hash: profile_hash,
                signer: AgentPubKey::from_raw_32(vec![6; 32]),
                payload: profile_payload,
                signature: Signature(vec![0; 64]),
            },
            attestation_statement: MachineTemporalSignedAttestationStatement {
                action_hash: attestation_hash,
                signer: authority_agent,
                payload: attestation_payload,
                signature: Signature(vec![0; 64]),
            },
        }
    }

    #[test]
    fn temporal_commitment_identifiers_reject_empty_profile_defined_values() {
        assert!(MachineTemporalCommitmentAlgorithm::ProfileDefined("".into()).is_empty());
        assert!(!MachineTemporalCommitmentAlgorithm::Sha256.is_empty());
        assert!(MachineTemporalAuthorityCommitmentTarget::ProfileDefined("".into()).is_empty());
        assert!(!MachineTemporalAuthorityCommitmentTarget::CertificateDer.is_empty());
    }

    #[test]
    fn temporal_digest_lengths_match_known_algorithms() {
        assert!(digest_length_matches(&MachineTemporalCommitmentAlgorithm::Sha256, 32));
        assert!(!digest_length_matches(&MachineTemporalCommitmentAlgorithm::Sha256, 31));
        assert!(digest_length_matches(&MachineTemporalCommitmentAlgorithm::Sha384, 48));
        assert!(digest_length_matches(&MachineTemporalCommitmentAlgorithm::Sha512, 64));
        assert!(digest_length_matches(
            &MachineTemporalCommitmentAlgorithm::ProfileDefined("future".into()),
            17,
        ));
    }

    #[test]
    fn temporal_profile_signed_payload_binds_accuracy_bound() {
        let mut profile = MachineTimeAuthorityProfileEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            profile_id: "profile-1".into(),
            source_profile: "rfc3161".into(),
            source_authority_commitment: vec![9; 32],
            source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget::CertificateDer,
            commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(1_000),
            max_accuracy_micros: 5,
            registrant_signature: Signature(vec![0; 64]),
        };
        let before = profile.signed_payload();
        profile.max_accuracy_micros += 1;
        assert_ne!(profile.signed_payload(), before);

        let before_source = profile.signed_payload();
        profile.source_authority_commitment[0] ^= 1;
        assert_ne!(profile.signed_payload(), before_source);

        let encoded_profile_id = "p".repeat(MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES + 1);
        assert!(encoded_profile_id.len() > MAX_MACHINE_TEMPORAL_PROFILE_TEXT_BYTES);
    }

    #[test]
    fn temporal_profile_signs_authority_commitment_target() {
        let mut profile = MachineTimeAuthorityProfileEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            profile_id: "profile-1".into(),
            source_profile: "rfc3161".into(),
            source_authority_commitment: vec![9; 32],
            source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget::CertificateDer,
            commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(1_000),
            max_accuracy_micros: 5,
            registrant_signature: Signature(vec![0; 64]),
        };
        let before = profile.signed_payload();
        profile.source_authority_commitment_target =
            MachineTemporalAuthorityCommitmentTarget::SubjectPublicKeyInfo;
        assert_ne!(profile.signed_payload(), before);
    }

    #[test]
    fn temporal_profile_signs_authority_commitment_algorithm() {
        let mut profile = MachineTimeAuthorityProfileEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            profile_id: "profile-1".into(),
            source_profile: "rfc3161".into(),
            source_authority_commitment: vec![9; 32],
            source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget::CertificateDer,
            commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(1_000),
            max_accuracy_micros: 5,
            registrant_signature: Signature(vec![0; 64]),
        };
        let before = profile.signed_payload();
        profile.source_authority_commitment_algorithm = MachineTemporalCommitmentAlgorithm::Sha512;
        assert_ne!(profile.signed_payload(), before);
    }

    #[test]
    fn temporal_profile_commitment_algorithm_is_signed() {
        let mut profile = MachineTimeAuthorityProfileEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            profile_id: "profile-1".into(),
            source_profile: "rfc3161".into(),
            source_authority_commitment: vec![9; 32],
            source_authority_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_authority_commitment_target: MachineTemporalAuthorityCommitmentTarget::CertificateDer,
            commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(1_000),
            max_accuracy_micros: 5,
            registrant_signature: Signature(vec![0; 64]),
        };
        let before = profile.signed_payload();
        profile.commitment_algorithm = MachineTemporalCommitmentAlgorithm::Sha512;
        assert_ne!(profile.signed_payload(), before);
    }

    #[test]
    fn temporal_attestation_signed_payload_binds_source_commitment() {
        let mut attestation = MachineTemporalAttestationEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            profile_hash: ActionHash::from_raw_36(vec![2; 36]),
            subject_hash: ActionHash::from_raw_36(vec![3; 36]),
            evidence_kind: MachineTemporalEvidenceKind::TransitionApproval,
            attested_at: Timestamp::from_micros(100),
            accuracy_micros: 5,
            source_reference: "tsa://example/1".into(),
            source_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_commitment: vec![7; 32],
            authority_signature: Signature(vec![0; 64]),
        };
        let before = attestation.signed_payload();
        attestation.source_commitment[0] ^= 1;
        assert_ne!(attestation.signed_payload(), before);
    }

    #[test]
    fn temporal_attestation_signed_payload_binds_commitment_algorithm() {
        let mut attestation = MachineTemporalAttestationEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            profile_hash: ActionHash::from_raw_36(vec![2; 36]),
            subject_hash: ActionHash::from_raw_36(vec![3; 36]),
            evidence_kind: MachineTemporalEvidenceKind::TransitionApproval,
            attested_at: Timestamp::from_micros(100),
            accuracy_micros: 5,
            source_reference: "tsa://example/1".into(),
            source_commitment_algorithm: MachineTemporalCommitmentAlgorithm::Sha256,
            source_commitment: vec![7; 32],
            authority_signature: Signature(vec![0; 64]),
        };
        let before = attestation.signed_payload();
        attestation.source_commitment_algorithm = MachineTemporalCommitmentAlgorithm::Sha512;
        assert_ne!(attestation.signed_payload(), before);
    }

    #[test]
    fn temporal_evidence_resolution_preserves_agreeing_provenance() {
        let a = temporal_observation(1, 7, 100);
        let b = temporal_observation(2, 8, 100);
        assert_eq!(
            resolve_temporal_evidence(vec![b.clone(), a.clone()]),
            MachineTemporalEvidenceResolution::UniqueObserved {
                time: Timestamp::from_micros(100),
                evidence: vec![a, b],
            }
        );
    }

    #[test]
    fn temporal_evidence_accuracy_disagreement_fails_closed() {
        let a = temporal_observation(1, 7, 100);
        let mut b = temporal_observation(2, 8, 100);
        b.accuracy_micros = 6;
        b.attestation_statement.payload.accuracy_micros = 6;
        assert!(matches!(
            resolve_temporal_evidence(vec![a, b]),
            MachineTemporalEvidenceResolution::ConflictingObserved(_)
        ));
    }

    #[test]
    fn temporal_evidence_resolution_deduplicates_identical_attestation_hashes() {
        let a = temporal_observation(1, 7, 100);
        assert_eq!(
            resolve_temporal_evidence(vec![a.clone(), a.clone()]),
            MachineTemporalEvidenceResolution::UniqueObserved {
                time: Timestamp::from_micros(100),
                evidence: vec![a],
            }
        );
    }

    #[test]
    fn temporal_evidence_resolution_rejects_observation_statement_mismatch() {
        let mut observation = temporal_observation(1, 7, 100);
        observation.source_commitment[0] ^= 1;
        assert_eq!(
            resolve_temporal_evidence(vec![observation]),
            MachineTemporalEvidenceResolution::InvalidEvidence
        );
    }

    #[test]
    fn temporal_evidence_resolution_rejects_divergent_duplicate_hashes() {
        let a = temporal_observation(1, 7, 100);
        let mut divergent = a.clone();
        divergent.source_reference = "tsa://different-reference".into();
        assert_eq!(
            resolve_temporal_evidence(vec![a, divergent]),
            MachineTemporalEvidenceResolution::InvalidEvidence
        );
    }

    #[test]
    fn temporal_evidence_resolution_bounds_distinct_observation_work() {
        let evidence = (0..=MAX_MACHINE_TEMPORAL_EVIDENCE_OBSERVATIONS)
            .map(|index| {
                let mut observation = temporal_observation((index % 250) as u8, 7, 100);
                let mut raw = vec![0u8; 36];
                raw[..4].copy_from_slice(&(index as u32).to_le_bytes());
                observation.attestation_hash = ActionHash::from_raw_36(raw);
                observation
            })
            .collect::<Vec<_>>();
        assert_eq!(
            resolve_temporal_evidence(evidence),
            MachineTemporalEvidenceResolution::EvidenceSetLimitExceeded {
                limit: MAX_MACHINE_TEMPORAL_EVIDENCE_OBSERVATIONS as u32,
            }
        );
    }

    #[test]
    fn temporal_evidence_conflicts_fail_closed_with_provenance() {
        let a = temporal_observation(1, 7, 100);
        let b = temporal_observation(2, 8, 200);
        assert_eq!(
            resolve_temporal_evidence(vec![b.clone(), a.clone()]),
            MachineTemporalEvidenceResolution::ConflictingObserved(vec![a, b])
        );
        assert_eq!(
            resolve_temporal_evidence(vec![]),
            MachineTemporalEvidenceResolution::NoEvidenceObserved
        );
    }
    #[test]
    fn temporal_uncertainty_must_fit_whole_approval_window() {
        assert!(temporal_interval_contains_interval(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(150),
            50,
        ));
        assert!(!temporal_interval_contains_interval(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(150),
            51,
        ));
        assert!(!temporal_interval_contains_interval(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(99),
            0,
        ));
    }

    #[test]
    fn temporal_approval_evidence_must_overlap_approval_window() {
        assert!(temporal_interval_contains(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(100),
        ));
        assert!(temporal_interval_contains(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(200),
        ));
        assert!(!temporal_interval_contains(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(99),
        ));
        assert!(!temporal_interval_contains(
            Timestamp::from_micros(100),
            Timestamp::from_micros(200),
            Timestamp::from_micros(201),
        ));
    }
    #[test]
    fn temporal_profile_boundaries_are_inclusive() {
        let from = Timestamp::from_micros(100);
        let until = Timestamp::from_micros(200);
        assert!(temporal_interval_contains(from, until, Timestamp::from_micros(100)));
        assert!(temporal_interval_contains(from, until, Timestamp::from_micros(200)));
        assert!(!temporal_interval_contains(from, until, Timestamp::from_micros(99)));
        assert!(!temporal_interval_contains(from, until, Timestamp::from_micros(201)));
    }
    #[test]
    fn controller_lease_duration_is_bounded() {
        let start = Timestamp::from_micros(1_000_000);
        let within = MachineControllerAuthorityEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            valid_from: start,
            valid_until: Timestamp::from_micros(
                start.as_micros() + MAX_MACHINE_CONTROLLER_LEASE_MICROS,
            ),
            issuer_signature: None,
            lease_schema_version: 1,
            requires_transition_approval: false,
        };
        assert!(authority_duration_is_bounded(&within));

        let beyond = MachineControllerAuthorityEntry {
            valid_until: Timestamp::from_micros(
                start.as_micros() + MAX_MACHINE_CONTROLLER_LEASE_MICROS + 1,
            ),
            ..within.clone()
        };
        assert!(!authority_duration_is_bounded(&beyond));
    }

    #[test]
    fn authority_validity_is_inclusive() {
        let authority = MachineControllerAuthorityEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![2; 32]),
            valid_from: Timestamp::from_micros(100),
            valid_until: Timestamp::from_micros(200),
            lease_schema_version: 1,
            requires_transition_approval: false,
            issuer_signature: None,
        };

        assert!(authority_valid_at(&authority, Timestamp::from_micros(100)));
        assert!(authority_valid_at(&authority, Timestamp::from_micros(200)));
        assert!(!authority_valid_at(&authority, Timestamp::from_micros(99)));
        assert!(!authority_valid_at(&authority, Timestamp::from_micros(201)));
    }

    #[test]
    fn transition_approval_interval_is_bounded() {
        let start = Timestamp::from_micros(10_000);
        let within = MachineControllerTransitionApprovalEntry {
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            authority_hash: ActionHash::from_raw_36(vec![2; 36]),
            controller_agent: AgentPubKey::from_raw_32(vec![3; 32]),
            predecessor_action: ActionHash::from_raw_36(vec![4; 36]),
            new_status: MachineStatus::Running,
            work_order_hash: None,
            valid_from: start,
            valid_until: Timestamp::from_micros(start.as_micros() + MAX_MACHINE_TRANSITION_APPROVAL_MICROS),
            issuer_signature: Signature(vec![0; 64]),
        };
        assert!(approval_valid_at(&within, Timestamp::from_micros(10_000)));
        assert!(approval_valid_at(&within, Timestamp::from_micros(10_000 + MAX_MACHINE_TRANSITION_APPROVAL_MICROS)));
        assert!(!approval_valid_at(&within, Timestamp::from_micros(9_999)));
        assert!(!approval_valid_at(&within, Timestamp::from_micros(10_000 + MAX_MACHINE_TRANSITION_APPROVAL_MICROS + 1)));
    }
    #[test]
    fn control_field_change_requires_authority() {
        let original = valid_machine();
        let mut updated = original.clone();

        updated.current_work_order = Some(ActionHash::from_raw_36(vec![7; 36]));
        assert!(machine_control_fields_changed(&original, &updated));

        updated = original.clone();
        updated.last_status_authority_hash = Some(ActionHash::from_raw_36(vec![8; 36]));
        assert!(machine_control_fields_changed(&original, &updated));

        updated = original.clone();
        updated.status = MachineStatus::Running;
        assert!(machine_control_fields_changed(&original, &updated));

        assert!(!machine_control_fields_changed(&original, &original));
    }

    #[test]
    fn create_machine_requires_name_and_location() {
        let mut m = valid_machine();
        m.name = "".into();
        let result = validate_create_entry(EntryTypes::Machine(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_machine_requires_positive_throughput() {
        let mut m = valid_machine();
        m.max_throughput_per_hour = 0;
        let result = validate_create_entry(EntryTypes::Machine(m)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_machine_valid() {
        let result = validate_create_entry(EntryTypes::Machine(valid_machine())).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn status_log_rejects_invalid_transition() {
        let log = MachineStatusLog {
            machine_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            machine_update_hash: ActionHash::from_raw_36(vec![8u8; 36]),
            authority_hash: ActionHash::from_raw_36(vec![9u8; 36]),
            previous_status: MachineStatus::Offline,
            new_status: MachineStatus::Running,
            work_order_hash: None,
            transition_approval_hash: None,
            changed_at: Timestamp::from_micros(0),
        };
        let result = validate_create_entry(EntryTypes::StatusLog(log)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_entry calls must_get_valid_record, which requires a
    // live HDI host and can't run in a plain unit test -- matching the
    // established pattern from every other zome's update validator this
    // pass.
}
