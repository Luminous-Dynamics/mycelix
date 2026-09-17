#![deny(unsafe_code)]

//! Pure, storage-independent capacity projection semantics for Mycelix Finance.
//!
//! This crate intentionally contains no HDK/HDI dependency and grants no runtime
//! authority. V1 reconstructs a closed allocation projection from bounded semantic
//! observations, derives reserved/consumed/available totals with exact arithmetic,
//! and commits the resulting projection with language-neutral bytes.
//!
//! Expiry is represented by an explicit semantic `Expired` transition in V1. The
//! evaluation time is committed as context but does not silently synthesize lifecycle
//! transitions from wall-clock comparison.

use mycelix_finance_exact::{AssetAmount, AssetId, ExactArithmeticError};
use serde::{Deserialize, Deserializer, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fmt;

/// V1 semantic profile revision for the closed allocation projection.
pub const CAPACITY_PROJECTION_PROFILE_V1: u16 = 1;
/// Exact allocation semantic profile revision accepted by this V1 projection.
pub const CAPACITY_ALLOCATION_PROFILE_V1: u16 = 1;
/// Maximum number of raw allocation observations admitted by the bounded V1 profile.
pub const MAX_PROJECTION_OBSERVATIONS_V1: usize = 4_096;
/// Maximum UTF-8 byte length of a semantic identifier in the V1 profile.
pub const MAX_SEMANTIC_ID_BYTES: usize = 128;

const SOURCE_DOMAIN_V1: &[u8] = b"mycelix/finance/capacity-source/v1\0";
const ALLOCATION_DOMAIN_V1: &[u8] = b"mycelix/finance/capacity-allocation/v1\0";
const TRANSITION_DOMAIN_V1: &[u8] = b"mycelix/finance/capacity-transition/v1\0";
const PROJECTION_DOMAIN_V1: &[u8] = b"mycelix/finance/capacity-projection/v1\0";

/// Fixed 32-byte commitment used by the V1 semantic profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Commitment32([u8; 32]);

impl Commitment32 {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

/// Bounded semantic identifier. Storage/action hashes are not required to use this type.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct SemanticId(String);

impl SemanticId {
    pub fn new(value: impl Into<String>) -> Result<Self, ProjectionError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_SEMANTIC_ID_BYTES
            || value.chars().any(char::is_control)
        {
            return Err(ProjectionError::InvalidSemanticId);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl<'de> Deserialize<'de> for SemanticId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(serde::de::Error::custom)
    }
}

/// Pure projection failures. Conflict is kept distinct from malformed lifecycle evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProjectionError {
    InvalidSemanticId,
    TooManyObservations,
    SourceMismatch,
    AssetMismatch,
    UnsupportedAllocationProfile,
    AllocationConflict,
    LifecycleConflict,
    InvalidLifecycle,
    CapacityExceeded,
    ArithmeticOverflow,
    ArithmeticUnderflow,
}

impl fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidSemanticId => {
                "semantic identifier is empty, too long, or contains control characters"
            }
            Self::TooManyObservations => "capacity projection observation bound exceeded",
            Self::SourceMismatch => "allocation observation belongs to a different capacity source",
            Self::AssetMismatch => "allocation or authorized capacity uses a different asset",
            Self::UnsupportedAllocationProfile => {
                "allocation observation uses an unsupported semantic profile revision"
            }
            Self::AllocationConflict => {
                "one allocation reference presents incompatible economic facts"
            }
            Self::LifecycleConflict => {
                "one allocation has incompatible semantic lifecycle transitions"
            }
            Self::InvalidLifecycle => {
                "allocation lifecycle is not valid under the V1 projection profile"
            }
            Self::CapacityExceeded => {
                "derived reserved plus consumed capacity exceeds authorized capacity"
            }
            Self::ArithmeticOverflow => "exact capacity arithmetic overflow",
            Self::ArithmeticUnderflow => "exact capacity arithmetic underflow",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ProjectionError {}

impl From<ExactArithmeticError> for ProjectionError {
    fn from(value: ExactArithmeticError) -> Self {
        match value {
            ExactArithmeticError::AssetMismatch => Self::AssetMismatch,
            ExactArithmeticError::Overflow => Self::ArithmeticOverflow,
            ExactArithmeticError::Underflow => Self::ArithmeticUnderflow,
            ExactArithmeticError::InvalidAssetId | ExactArithmeticError::DivisionByZero => {
                Self::InvalidLifecycle
            }
        }
    }
}

/// Exact semantic identity of one capacity source under the bounded V1 profile.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapacitySourceRefV1 {
    domain: SemanticId,
    source_id: SemanticId,
    generation: u64,
    asset: AssetId,
    provider_profile_commitment: Commitment32,
}

impl CapacitySourceRefV1 {
    pub fn new(
        domain: SemanticId,
        source_id: SemanticId,
        generation: u64,
        asset: AssetId,
        provider_profile_commitment: Commitment32,
    ) -> Self {
        Self {
            domain,
            source_id,
            generation,
            asset,
            provider_profile_commitment,
        }
    }

    pub fn asset(&self) -> &AssetId {
        &self.asset
    }

    pub fn commitment(&self) -> Commitment32 {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(SOURCE_DOMAIN_V1);
        self.encode_into(&mut bytes);
        sha256(&bytes)
    }

    fn encode_into(&self, bytes: &mut Vec<u8>) {
        push_str(bytes, self.domain.as_str());
        push_str(bytes, self.source_id.as_str());
        push_u64(bytes, self.generation);
        push_asset_id(bytes, &self.asset);
        push_commitment(bytes, self.provider_profile_commitment);
    }
}

/// V1 allocation states reconstructed by the closed projection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AllocationStateV1 {
    Reserved,
    Consumed,
    Released,
    Expired,
    Revoked,
}

impl AllocationStateV1 {
    const fn tag(self) -> u8 {
        match self {
            Self::Reserved => 0,
            Self::Consumed => 1,
            Self::Released => 2,
            Self::Expired => 3,
            Self::Revoked => 4,
        }
    }

    const fn is_terminal(self) -> bool {
        !matches!(self, Self::Reserved)
    }
}

/// Stable reference identity used to detect incompatible reuse of one allocation slot.
///
/// This reference is audit/conflict provenance, not the canonical economic allocation ID.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AllocationIdentityV1 {
    allocation_ref: SemanticId,
    reservation_id: SemanticId,
    reservation_commitment: Commitment32,
    idempotency_domain: SemanticId,
    allocation_profile_revision: u16,
}

impl AllocationIdentityV1 {
    pub fn new(
        allocation_ref: SemanticId,
        reservation_id: SemanticId,
        reservation_commitment: Commitment32,
        idempotency_domain: SemanticId,
        allocation_profile_revision: u16,
    ) -> Self {
        Self {
            allocation_ref,
            reservation_id,
            reservation_commitment,
            idempotency_domain,
            allocation_profile_revision,
        }
    }

    pub fn allocation_ref(&self) -> &SemanticId {
        &self.allocation_ref
    }
}

/// Economic descriptor of one allocation.
///
/// `allocation_ref` is deliberately excluded from the canonical economic allocation ID.
/// Two physical/reference observations of identical economic facts therefore collapse to
/// one allocation rather than consuming capacity twice. Reusing the same reference with
/// changed economic facts is detected separately as `AllocationConflict`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AllocationDescriptorV1 {
    identity: AllocationIdentityV1,
    source: CapacitySourceRefV1,
    amount: AssetAmount,
    expires_at_unix_ms: Option<u64>,
}

impl AllocationDescriptorV1 {
    pub fn new(
        identity: AllocationIdentityV1,
        source: CapacitySourceRefV1,
        amount: AssetAmount,
        expires_at_unix_ms: Option<u64>,
    ) -> Self {
        Self {
            identity,
            source,
            amount,
            expires_at_unix_ms,
        }
    }

    pub fn allocation_ref(&self) -> &SemanticId {
        self.identity.allocation_ref()
    }

    pub fn canonical_allocation_id(&self) -> Commitment32 {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(ALLOCATION_DOMAIN_V1);
        self.source.encode_into(&mut bytes);
        push_str(&mut bytes, self.identity.reservation_id.as_str());
        push_commitment(&mut bytes, self.identity.reservation_commitment);
        push_asset_amount(&mut bytes, &self.amount);
        push_str(&mut bytes, self.identity.idempotency_domain.as_str());
        push_u16(&mut bytes, self.identity.allocation_profile_revision);
        push_optional_u64(&mut bytes, self.expires_at_unix_ms);
        sha256(&bytes)
    }

    fn economically_eq(&self, other: &Self) -> bool {
        self.source == other.source
            && self.identity.reservation_id == other.identity.reservation_id
            && self.identity.reservation_commitment == other.identity.reservation_commitment
            && self.amount == other.amount
            && self.identity.idempotency_domain == other.identity.idempotency_domain
            && self.identity.allocation_profile_revision
                == other.identity.allocation_profile_revision
            && self.expires_at_unix_ms == other.expires_at_unix_ms
    }
}

/// One authenticated-at-an-outer-layer semantic observation.
///
/// `physical_observation_id` is audit provenance. It is intentionally excluded from
/// economic identity and from the projection commitment.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AllocationObservationV1 {
    descriptor: AllocationDescriptorV1,
    state: AllocationStateV1,
    transition_sequence: u8,
    predecessor_transition: Option<Commitment32>,
    physical_observation_id: Commitment32,
}

impl AllocationObservationV1 {
    pub fn new(
        descriptor: AllocationDescriptorV1,
        state: AllocationStateV1,
        transition_sequence: u8,
        predecessor_transition: Option<Commitment32>,
        physical_observation_id: Commitment32,
    ) -> Self {
        Self {
            descriptor,
            state,
            transition_sequence,
            predecessor_transition,
            physical_observation_id,
        }
    }

    pub fn descriptor(&self) -> &AllocationDescriptorV1 {
        &self.descriptor
    }

    pub fn state(&self) -> AllocationStateV1 {
        self.state
    }

    pub fn physical_observation_id(&self) -> Commitment32 {
        self.physical_observation_id
    }

    pub fn semantic_transition_id(&self) -> Commitment32 {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(TRANSITION_DOMAIN_V1);
        push_commitment(&mut bytes, self.descriptor.canonical_allocation_id());
        bytes.push(self.state.tag());
        bytes.push(self.transition_sequence);
        push_optional_commitment(&mut bytes, self.predecessor_transition);
        sha256(&bytes)
    }

    fn semantically_eq(&self, other: &Self) -> bool {
        self.descriptor.economically_eq(&other.descriptor)
            && self.state == other.state
            && self.transition_sequence == other.transition_sequence
            && self.predecessor_transition == other.predecessor_transition
    }
}

/// Bounded raw input to the pure projection theorem.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapacityProjectionInputV1 {
    source: CapacitySourceRefV1,
    authorized_capacity: AssetAmount,
    observations: Vec<AllocationObservationV1>,
    evaluation_context: Commitment32,
    evaluation_time_unix_ms: u64,
}

impl CapacityProjectionInputV1 {
    pub fn new(
        source: CapacitySourceRefV1,
        authorized_capacity: AssetAmount,
        observations: Vec<AllocationObservationV1>,
        evaluation_context: Commitment32,
        evaluation_time_unix_ms: u64,
    ) -> Self {
        Self {
            source,
            authorized_capacity,
            observations,
            evaluation_context,
            evaluation_time_unix_ms,
        }
    }
}

/// Canonical final record for one semantic economic allocation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CanonicalAllocationRecordV1 {
    allocation_id: Commitment32,
    reservation_commitment: Commitment32,
    amount: AssetAmount,
    final_state: AllocationStateV1,
    reserved_transition: Commitment32,
    terminal_transition: Option<Commitment32>,
    expires_at_unix_ms: Option<u64>,
}

impl CanonicalAllocationRecordV1 {
    pub fn allocation_id(&self) -> Commitment32 {
        self.allocation_id
    }

    pub fn final_state(&self) -> AllocationStateV1 {
        self.final_state
    }

    pub fn amount(&self) -> &AssetAmount {
        &self.amount
    }
}

/// Positive projection result. Deliberately serializable but not deserializable:
/// callers must reconstruct it through `project_capacity_v1`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CapacityProjectionV1 {
    source: CapacitySourceRefV1,
    authorized_capacity: AssetAmount,
    active_reserved: AssetAmount,
    consumed_or_drawn: AssetAmount,
    available: AssetAmount,
    allocations: Vec<CanonicalAllocationRecordV1>,
    evaluation_context: Commitment32,
    evaluation_time_unix_ms: u64,
    commitment: Commitment32,
}

impl CapacityProjectionV1 {
    pub fn source(&self) -> &CapacitySourceRefV1 {
        &self.source
    }

    pub fn authorized_capacity(&self) -> &AssetAmount {
        &self.authorized_capacity
    }

    pub fn active_reserved(&self) -> &AssetAmount {
        &self.active_reserved
    }

    pub fn consumed_or_drawn(&self) -> &AssetAmount {
        &self.consumed_or_drawn
    }

    pub fn available(&self) -> &AssetAmount {
        &self.available
    }

    pub fn allocations(&self) -> &[CanonicalAllocationRecordV1] {
        &self.allocations
    }

    pub fn evaluation_context(&self) -> Commitment32 {
        self.evaluation_context
    }

    pub fn evaluation_time_unix_ms(&self) -> u64 {
        self.evaluation_time_unix_ms
    }

    pub fn commitment(&self) -> Commitment32 {
        self.commitment
    }
}

/// Reconstruct the complete bounded V1 projection from semantic observations.
///
/// Outer runtime/evidence layers remain responsible for authenticating observation
/// provenance and proving completeness of the supplied evidence set.
pub fn project_capacity_v1(
    input: CapacityProjectionInputV1,
) -> Result<CapacityProjectionV1, ProjectionError> {
    if input.observations.len() > MAX_PROJECTION_OBSERVATIONS_V1 {
        return Err(ProjectionError::TooManyObservations);
    }
    if input.authorized_capacity.asset() != input.source.asset() {
        return Err(ProjectionError::AssetMismatch);
    }

    let mut ref_to_allocation = BTreeMap::<SemanticId, Commitment32>::new();
    let mut reservation_id_to_allocation = BTreeMap::<SemanticId, Commitment32>::new();
    let mut reservation_commitment_to_allocation = BTreeMap::<Commitment32, Commitment32>::new();
    let mut grouped = BTreeMap::<Commitment32, Vec<AllocationObservationV1>>::new();

    for observation in input.observations {
        if observation.descriptor.source != input.source {
            return Err(ProjectionError::SourceMismatch);
        }
        if observation.descriptor.amount.asset() != input.source.asset() {
            return Err(ProjectionError::AssetMismatch);
        }
        if observation.descriptor.identity.allocation_profile_revision
            != CAPACITY_ALLOCATION_PROFILE_V1
        {
            return Err(ProjectionError::UnsupportedAllocationProfile);
        }

        let allocation_id = observation.descriptor.canonical_allocation_id();
        if let Some(existing) = ref_to_allocation.get(observation.descriptor.allocation_ref())
            && existing != &allocation_id
        {
            return Err(ProjectionError::AllocationConflict);
        }
        ref_to_allocation
            .entry(observation.descriptor.allocation_ref().clone())
            .or_insert(allocation_id);

        if let Some(existing) =
            reservation_id_to_allocation.get(&observation.descriptor.identity.reservation_id)
            && existing != &allocation_id
        {
            return Err(ProjectionError::AllocationConflict);
        }
        reservation_id_to_allocation
            .entry(observation.descriptor.identity.reservation_id.clone())
            .or_insert(allocation_id);

        if let Some(existing) = reservation_commitment_to_allocation
            .get(&observation.descriptor.identity.reservation_commitment)
            && existing != &allocation_id
        {
            return Err(ProjectionError::AllocationConflict);
        }
        reservation_commitment_to_allocation
            .entry(observation.descriptor.identity.reservation_commitment)
            .or_insert(allocation_id);

        grouped.entry(allocation_id).or_default().push(observation);
    }

    let zero = AssetAmount::new(0, input.source.asset().clone());
    let mut active_reserved = zero.clone();
    let mut consumed_or_drawn = zero;
    let mut records = Vec::with_capacity(grouped.len());

    for (allocation_id, observations) in grouped {
        let record = reconstruct_allocation(allocation_id, observations)?;

        match record.final_state {
            AllocationStateV1::Reserved => {
                active_reserved = active_reserved.checked_add(&record.amount)?;
            }
            AllocationStateV1::Consumed => {
                consumed_or_drawn = consumed_or_drawn.checked_add(&record.amount)?;
            }
            AllocationStateV1::Released
            | AllocationStateV1::Expired
            | AllocationStateV1::Revoked => {}
        }

        records.push(record);
    }

    let encumbered = consumed_or_drawn.checked_add(&active_reserved)?;
    if encumbered.atomic_units() > input.authorized_capacity.atomic_units() {
        return Err(ProjectionError::CapacityExceeded);
    }
    let available = input.authorized_capacity.checked_sub(&encumbered)?;

    let commitment = projection_commitment(ProjectionCommitmentInput {
        source: &input.source,
        authorized_capacity: &input.authorized_capacity,
        active_reserved: &active_reserved,
        consumed_or_drawn: &consumed_or_drawn,
        available: &available,
        records: &records,
        evaluation_context: input.evaluation_context,
        evaluation_time_unix_ms: input.evaluation_time_unix_ms,
    });

    Ok(CapacityProjectionV1 {
        source: input.source,
        authorized_capacity: input.authorized_capacity,
        active_reserved,
        consumed_or_drawn,
        available,
        allocations: records,
        evaluation_context: input.evaluation_context,
        evaluation_time_unix_ms: input.evaluation_time_unix_ms,
        commitment,
    })
}

fn reconstruct_allocation(
    allocation_id: Commitment32,
    observations: Vec<AllocationObservationV1>,
) -> Result<CanonicalAllocationRecordV1, ProjectionError> {
    let descriptor = observations
        .first()
        .ok_or(ProjectionError::InvalidLifecycle)?
        .descriptor
        .clone();

    if observations
        .iter()
        .any(|observation| observation.descriptor.canonical_allocation_id() != allocation_id)
    {
        return Err(ProjectionError::AllocationConflict);
    }

    let mut transitions = BTreeMap::<Commitment32, AllocationObservationV1>::new();
    for observation in observations {
        let transition_id = observation.semantic_transition_id();
        if let Some(existing) = transitions.get(&transition_id) {
            if !existing.semantically_eq(&observation) {
                return Err(ProjectionError::LifecycleConflict);
            }
            continue;
        }
        transitions.insert(transition_id, observation);
    }

    let roots: Vec<_> = transitions
        .iter()
        .filter(|(_, observation)| observation.transition_sequence == 0)
        .collect();

    if roots.len() != 1 {
        return Err(ProjectionError::InvalidLifecycle);
    }

    let (reserved_transition, root) = roots[0];
    if root.state != AllocationStateV1::Reserved || root.predecessor_transition.is_some() {
        return Err(ProjectionError::InvalidLifecycle);
    }

    let mut terminal: Option<(Commitment32, AllocationStateV1)> = None;
    for (transition_id, observation) in &transitions {
        if observation.transition_sequence == 0 {
            continue;
        }
        if observation.transition_sequence != 1
            || !observation.state.is_terminal()
            || observation.predecessor_transition != Some(*reserved_transition)
        {
            return Err(ProjectionError::InvalidLifecycle);
        }
        if terminal.is_some() {
            return Err(ProjectionError::LifecycleConflict);
        }
        terminal = Some((*transition_id, observation.state));
    }

    let (final_state, terminal_transition) = match terminal {
        Some((transition_id, state)) => (state, Some(transition_id)),
        None => (AllocationStateV1::Reserved, None),
    };

    Ok(CanonicalAllocationRecordV1 {
        allocation_id,
        reservation_commitment: descriptor.identity.reservation_commitment,
        amount: descriptor.amount,
        final_state,
        reserved_transition: *reserved_transition,
        terminal_transition,
        expires_at_unix_ms: descriptor.expires_at_unix_ms,
    })
}

struct ProjectionCommitmentInput<'a> {
    source: &'a CapacitySourceRefV1,
    authorized_capacity: &'a AssetAmount,
    active_reserved: &'a AssetAmount,
    consumed_or_drawn: &'a AssetAmount,
    available: &'a AssetAmount,
    records: &'a [CanonicalAllocationRecordV1],
    evaluation_context: Commitment32,
    evaluation_time_unix_ms: u64,
}

fn projection_commitment(input: ProjectionCommitmentInput<'_>) -> Commitment32 {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(PROJECTION_DOMAIN_V1);
    push_u16(&mut bytes, CAPACITY_PROJECTION_PROFILE_V1);
    input.source.encode_into(&mut bytes);
    push_asset_amount(&mut bytes, input.authorized_capacity);
    push_commitment(&mut bytes, input.evaluation_context);
    push_u64(&mut bytes, input.evaluation_time_unix_ms);
    push_u32(&mut bytes, input.records.len() as u32);

    for record in input.records {
        push_commitment(&mut bytes, record.allocation_id);
        push_commitment(&mut bytes, record.reservation_commitment);
        push_asset_amount(&mut bytes, &record.amount);
        bytes.push(record.final_state.tag());
        push_commitment(&mut bytes, record.reserved_transition);
        push_optional_commitment(&mut bytes, record.terminal_transition);
        push_optional_u64(&mut bytes, record.expires_at_unix_ms);
    }

    push_asset_amount(&mut bytes, input.active_reserved);
    push_asset_amount(&mut bytes, input.consumed_or_drawn);
    push_asset_amount(&mut bytes, input.available);
    sha256(&bytes)
}

fn sha256(bytes: &[u8]) -> Commitment32 {
    let digest = Sha256::digest(bytes);
    let mut output = [0_u8; 32];
    output.copy_from_slice(&digest);
    Commitment32(output)
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_str(bytes: &mut Vec<u8>, value: &str) {
    let raw = value.as_bytes();
    push_u16(bytes, raw.len() as u16);
    bytes.extend_from_slice(raw);
}

fn push_asset_id(bytes: &mut Vec<u8>, asset: &AssetId) {
    push_str(bytes, asset.as_str());
}

fn push_asset_amount(bytes: &mut Vec<u8>, amount: &AssetAmount) {
    push_asset_id(bytes, amount.asset());
    push_u64(bytes, amount.atomic_units());
}

fn push_commitment(bytes: &mut Vec<u8>, commitment: Commitment32) {
    bytes.extend_from_slice(commitment.as_bytes());
}

fn push_optional_commitment(bytes: &mut Vec<u8>, commitment: Option<Commitment32>) {
    match commitment {
        Some(value) => {
            bytes.push(1);
            push_commitment(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn push_optional_u64(bytes: &mut Vec<u8>, value: Option<u64>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_u64(bytes, value);
        }
        None => bytes.push(0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn commitment(byte: u8) -> Commitment32 {
        Commitment32::from_bytes([byte; 32])
    }

    fn semantic(value: &str) -> SemanticId {
        SemanticId::new(value).expect("static semantic id")
    }

    fn hex_commitment(value: &str) -> Commitment32 {
        assert_eq!(value.len(), 64);
        let mut bytes = [0_u8; 32];
        for (index, byte) in bytes.iter_mut().enumerate() {
            let offset = index * 2;
            *byte = u8::from_str_radix(&value[offset..offset + 2], 16).expect("valid hex");
        }
        Commitment32::from_bytes(bytes)
    }

    fn sap() -> AssetId {
        AssetId::new("SAP").expect("static asset id")
    }

    fn reservation_commitment(value: &str) -> Commitment32 {
        sha256(value.as_bytes())
    }

    fn source() -> CapacitySourceRefV1 {
        CapacitySourceRefV1::new(
            semantic("treasury"),
            semantic("treasury-alpha"),
            1,
            sap(),
            commitment(1),
        )
    }

    fn descriptor(reference: &str, reservation: &str, amount: u64) -> AllocationDescriptorV1 {
        let identity = AllocationIdentityV1::new(
            semantic(reference),
            semantic(reservation),
            reservation_commitment(reservation),
            semantic("reservation-v1"),
            1,
        );
        AllocationDescriptorV1::new(
            identity,
            source(),
            AssetAmount::new(amount, sap()),
            Some(9_999_999),
        )
    }

    fn reserved(
        reference: &str,
        reservation: &str,
        amount: u64,
        physical: u8,
    ) -> AllocationObservationV1 {
        AllocationObservationV1::new(
            descriptor(reference, reservation, amount),
            AllocationStateV1::Reserved,
            0,
            None,
            commitment(physical),
        )
    }

    fn terminal(
        root: &AllocationObservationV1,
        state: AllocationStateV1,
        physical: u8,
    ) -> AllocationObservationV1 {
        AllocationObservationV1::new(
            root.descriptor.clone(),
            state,
            1,
            Some(root.semantic_transition_id()),
            commitment(physical),
        )
    }

    fn project(
        authorized: u64,
        observations: Vec<AllocationObservationV1>,
    ) -> Result<CapacityProjectionV1, ProjectionError> {
        project_capacity_v1(CapacityProjectionInputV1::new(
            source(),
            AssetAmount::new(authorized, sap()),
            observations,
            commitment(9),
            123_456,
        ))
    }

    #[test]
    fn v1_canonical_golden_vector_is_frozen() {
        let root = reserved("alloc-a", "reservation-a", 80, 2);
        let projection = project(100, vec![root.clone()]).expect("valid projection");

        assert_eq!(
            source().commitment(),
            hex_commitment("a5defdb1013c7ced8adaefb8ec2d71e906133ca2b29688e3f4303fec3eae8dbd")
        );
        assert_eq!(
            root.descriptor().canonical_allocation_id(),
            hex_commitment("9c7b87a20bcbc74911ab24f5767c28c75c91c8934ff5edc819fcd01ecb867022")
        );
        assert_eq!(
            root.semantic_transition_id(),
            hex_commitment("b98ac47bbc6f133dc34d1264cc554e0c7820380a5e9f4fa67fdad0f5a1c1e956")
        );
        assert_eq!(
            projection.commitment(),
            hex_commitment("4d67b75e7e199116cedc836c0620ceb82d7c1cf9e9bdd4b41a8d290360154ae1")
        );
    }

    #[test]
    fn empty_projection_derives_full_availability() {
        let projection = project(100, Vec::new()).expect("valid projection");
        assert_eq!(projection.active_reserved().atomic_units(), 0);
        assert_eq!(projection.consumed_or_drawn().atomic_units(), 0);
        assert_eq!(projection.available().atomic_units(), 100);
        assert!(projection.allocations().is_empty());
    }

    #[test]
    fn active_reservation_is_derived_exactly_once() {
        let first = reserved("alloc-a", "reservation-a", 80, 2);
        let duplicate = reserved("alloc-a", "reservation-a", 80, 3);
        let projection = project(100, vec![first, duplicate]).expect("duplicate observation");
        assert_eq!(projection.active_reserved().atomic_units(), 80);
        assert_eq!(projection.available().atomic_units(), 20);
        assert_eq!(projection.allocations().len(), 1);
    }

    #[test]
    fn different_physical_reference_same_economic_allocation_counts_once() {
        let first = reserved("alloc-a", "reservation-a", 80, 2);
        let second = reserved("alloc-b", "reservation-a", 80, 3);
        let projection = project(100, vec![first, second]).expect("same economic allocation");
        assert_eq!(projection.active_reserved().atomic_units(), 80);
        assert_eq!(projection.allocations().len(), 1);
    }

    #[test]
    fn same_reference_with_changed_amount_is_conflict() {
        let first = reserved("alloc-a", "reservation-a", 80, 2);
        let changed = reserved("alloc-a", "reservation-a", 81, 3);
        assert_eq!(
            project(100, vec![first, changed]),
            Err(ProjectionError::AllocationConflict)
        );
    }

    #[test]
    fn unsupported_allocation_profile_fails_closed() {
        let mut wrong = descriptor("alloc-a", "reservation-a", 80);
        wrong.identity.allocation_profile_revision = 2;
        let observation = AllocationObservationV1::new(
            wrong,
            AllocationStateV1::Reserved,
            0,
            None,
            commitment(2),
        );
        assert_eq!(
            project(100, vec![observation]),
            Err(ProjectionError::UnsupportedAllocationProfile)
        );
    }

    #[test]
    fn reservation_alias_with_changed_amount_is_conflict() {
        let first = reserved("alloc-a", "reservation-a", 80, 2);
        let alias = reserved("alloc-b", "reservation-a", 81, 3);
        assert_eq!(
            project(200, vec![first, alias]),
            Err(ProjectionError::AllocationConflict)
        );
    }

    #[test]
    fn same_reference_with_changed_reservation_commitment_is_conflict() {
        let first = reserved("alloc-a", "reservation-a", 80, 2);
        let mut changed = descriptor("alloc-a", "reservation-a", 80);
        changed.identity.reservation_commitment = commitment(55);
        let changed = AllocationObservationV1::new(
            changed,
            AllocationStateV1::Reserved,
            0,
            None,
            commitment(3),
        );
        assert_eq!(
            project(100, vec![first, changed]),
            Err(ProjectionError::AllocationConflict)
        );
    }

    #[test]
    fn observation_from_wrong_source_fails_closed() {
        let mut wrong = descriptor("alloc-a", "reservation-a", 10);
        wrong.source = CapacitySourceRefV1::new(
            semantic("treasury"),
            semantic("treasury-beta"),
            1,
            sap(),
            commitment(1),
        );
        let observation = AllocationObservationV1::new(
            wrong,
            AllocationStateV1::Reserved,
            0,
            None,
            commitment(2),
        );
        assert_eq!(
            project(100, vec![observation]),
            Err(ProjectionError::SourceMismatch)
        );
    }

    #[test]
    fn duplicate_consumed_transition_does_not_double_consume() {
        let root = reserved("alloc-a", "reservation-a", 80, 2);
        let consumed = terminal(&root, AllocationStateV1::Consumed, 3);
        let duplicate = terminal(&root, AllocationStateV1::Consumed, 4);
        let projection = project(100, vec![root, consumed, duplicate]).expect("semantic duplicate");
        assert_eq!(projection.active_reserved().atomic_units(), 0);
        assert_eq!(projection.consumed_or_drawn().atomic_units(), 80);
        assert_eq!(projection.available().atomic_units(), 20);
    }

    #[test]
    fn consumed_allocation_moves_out_of_reserved_total() {
        let root = reserved("alloc-a", "reservation-a", 80, 2);
        let consumed = terminal(&root, AllocationStateV1::Consumed, 3);
        let projection = project(100, vec![consumed, root]).expect("valid consumed lifecycle");
        assert_eq!(projection.active_reserved().atomic_units(), 0);
        assert_eq!(projection.consumed_or_drawn().atomic_units(), 80);
        assert_eq!(projection.available().atomic_units(), 20);
    }

    #[test]
    fn conflicting_terminal_transitions_fail_closed() {
        let root = reserved("alloc-a", "reservation-a", 80, 2);
        let consumed = terminal(&root, AllocationStateV1::Consumed, 3);
        let released = terminal(&root, AllocationStateV1::Released, 4);
        assert_eq!(
            project(100, vec![root, consumed, released]),
            Err(ProjectionError::LifecycleConflict)
        );
    }

    #[test]
    fn terminal_with_wrong_predecessor_is_invalid() {
        let root = reserved("alloc-a", "reservation-a", 80, 2);
        let invalid = AllocationObservationV1::new(
            root.descriptor.clone(),
            AllocationStateV1::Consumed,
            1,
            Some(commitment(77)),
            commitment(3),
        );
        assert_eq!(
            project(100, vec![root, invalid]),
            Err(ProjectionError::InvalidLifecycle)
        );
    }

    #[test]
    fn projection_rejects_capacity_oversubscription() {
        let root = reserved("alloc-a", "reservation-a", 101, 2);
        assert_eq!(
            project(100, vec![root]),
            Err(ProjectionError::CapacityExceeded)
        );
    }

    #[test]
    fn observation_order_and_physical_ids_do_not_change_commitment() {
        let a = reserved("alloc-a", "reservation-a", 30, 2);
        let b = reserved("alloc-b", "reservation-b", 40, 3);
        let left = project(100, vec![a.clone(), b.clone()]).expect("valid projection");

        let a_other_physical = AllocationObservationV1::new(
            a.descriptor.clone(),
            a.state,
            a.transition_sequence,
            a.predecessor_transition,
            commitment(88),
        );
        let b_other_physical = AllocationObservationV1::new(
            b.descriptor.clone(),
            b.state,
            b.transition_sequence,
            b.predecessor_transition,
            commitment(89),
        );
        let right = project(100, vec![b_other_physical, a_other_physical]).expect("same economics");

        assert_eq!(left.commitment(), right.commitment());
        assert_eq!(left.active_reserved().atomic_units(), 70);
        assert_eq!(right.active_reserved().atomic_units(), 70);
    }

    #[test]
    fn released_and_expired_allocations_are_not_active() {
        let released_root = reserved("alloc-a", "reservation-a", 30, 2);
        let released = terminal(&released_root, AllocationStateV1::Released, 3);
        let expired_root = reserved("alloc-b", "reservation-b", 40, 4);
        let expired = terminal(&expired_root, AllocationStateV1::Expired, 5);

        let projection = project(100, vec![released_root, released, expired_root, expired])
            .expect("valid terminal lifecycles");

        assert_eq!(projection.active_reserved().atomic_units(), 0);
        assert_eq!(projection.consumed_or_drawn().atomic_units(), 0);
        assert_eq!(projection.available().atomic_units(), 100);
        assert_eq!(projection.allocations().len(), 2);
    }

    #[test]
    fn checked_sum_overflow_fails_closed() {
        let first = reserved("alloc-a", "reservation-a", u64::MAX, 2);
        let second = reserved("alloc-b", "reservation-b", 1, 3);
        assert_eq!(
            project(u64::MAX, vec![first, second]),
            Err(ProjectionError::ArithmeticOverflow)
        );
    }

    #[test]
    fn projection_input_rejects_unknown_cached_aggregate_fields() {
        let input = CapacityProjectionInputV1::new(
            source(),
            AssetAmount::new(100, sap()),
            Vec::new(),
            commitment(9),
            123_456,
        );
        let mut wire = serde_json::to_value(input).expect("serialize raw input");
        wire.as_object_mut()
            .expect("projection input is an object")
            .insert("available".to_owned(), serde_json::Value::from(999_u64));

        assert!(serde_json::from_value::<CapacityProjectionInputV1>(wire).is_err());
    }

    #[test]
    fn bounded_observation_limit_fails_closed() {
        let observation = reserved("alloc-a", "reservation-a", 1, 2);
        let observations = vec![observation; MAX_PROJECTION_OBSERVATIONS_V1 + 1];
        assert_eq!(
            project(u64::MAX, observations),
            Err(ProjectionError::TooManyObservations)
        );
    }

    #[test]
    fn canonical_semantic_change_changes_projection_commitment() {
        let first = project(100, vec![reserved("alloc-a", "reservation-a", 10, 2)])
            .expect("first projection");
        let changed = project(100, vec![reserved("alloc-a", "reservation-a", 11, 2)])
            .expect("changed projection");

        assert_ne!(first.commitment(), changed.commitment());
    }

    #[test]
    fn wrong_asset_fails_closed() {
        let mut wrong = descriptor("alloc-a", "reservation-a", 10);
        wrong.amount = AssetAmount::new(10, AssetId::new("TEND").expect("asset"));
        let observation = AllocationObservationV1::new(
            wrong,
            AllocationStateV1::Reserved,
            0,
            None,
            commitment(2),
        );
        assert_eq!(
            project(100, vec![observation]),
            Err(ProjectionError::AssetMismatch)
        );
    }

    proptest! {
        #[test]
        fn one_reservation_derives_exact_available(
            authorized in 0_u64..=1_000_000,
            reserved_amount in 0_u64..=1_000_000
        ) {
            prop_assume!(reserved_amount <= authorized);
            let root = reserved("alloc-a", "reservation-a", reserved_amount, 2);
            let projection = project(authorized, vec![root]).expect("bounded by assumption");
            prop_assert_eq!(projection.active_reserved().atomic_units(), reserved_amount);
            prop_assert_eq!(projection.available().atomic_units(), authorized - reserved_amount);
        }

        #[test]
        fn optimized_projection_matches_simple_reference_model(
            items in prop::collection::vec((0_u16..=1_000, any::<bool>()), 0..32)
        ) {
            let mut observations = Vec::new();
            let mut reference_reserved = 0_u64;
            let mut reference_consumed = 0_u64;

            for (index, (amount, consumed)) in items.iter().copied().enumerate() {
                let amount = u64::from(amount);
                let allocation_ref = format!("alloc-{index}");
                let reservation_id = format!("reservation-{index}");
                let root = reserved(
                    &allocation_ref,
                    &reservation_id,
                    amount,
                    (index * 2) as u8,
                );

                if consumed {
                    let terminal = terminal(
                        &root,
                        AllocationStateV1::Consumed,
                        (index * 2 + 1) as u8,
                    );
                    reference_consumed += amount;
                    observations.push(root);
                    observations.push(terminal);
                } else {
                    reference_reserved += amount;
                    observations.push(root);
                }
            }

            observations.reverse();
            let authorized = reference_reserved + reference_consumed;
            let projection =
                project(authorized, observations).expect("reference-bounded projection");

            prop_assert_eq!(
                projection.active_reserved().atomic_units(),
                reference_reserved
            );
            prop_assert_eq!(
                projection.consumed_or_drawn().atomic_units(),
                reference_consumed
            );
            prop_assert_eq!(projection.available().atomic_units(), 0);
        }
    }
}
