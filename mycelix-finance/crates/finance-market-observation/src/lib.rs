#![forbid(unsafe_code)]

//! FIN-MKT-002A: append-only provider-neutral market order observations.
//!
//! This crate defines immutable observation identity for order lifecycle events,
//! fills, and fill corrections/busts. It deliberately does not project current
//! order state, establish provider truth, position ownership, or settlement.

use mycelix_finance_market_core::{
    CanonicalMarketOrderIntentV1, Digest32, MarketExternalIdV1, MarketInstrumentRefV1,
    MarketPriceV1, MarketProfileRefV1, MarketQuantityV1, MarketSubjectRefV1,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest as _, Sha256};
use std::fmt;

pub const OBSERVATION_PROFILE_REVISION_V1: u32 = 1;

const EVENT_ID_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_EVENT_ID_V1\0";
const EVENT_SEM_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_EVENT_SEM_V1\0";
const EVENT_EVIDENCE_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_EVENT_EVIDENCE_V1\0";
const FILL_ID_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_FILL_ID_V1\0";
const FILL_SEM_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_FILL_SEM_V1\0";
const FILL_EVIDENCE_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_FILL_EVIDENCE_V1\0";
const ADJUST_ID_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_ADJUST_ID_V1\0";
const ADJUST_SEM_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_ADJUST_SEM_V1\0";
const ADJUST_EVIDENCE_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_ADJUST_EVIDENCE_V1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObservationError {
    InvalidChronology,
    ZeroFillQuantity,
    ZeroFillPrice,
    SubjectMismatch,
    SelfCorrection,
}

impl fmt::Display for ObservationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidChronology => {
                "chronology profile and chronology evidence must be present together"
            }
            Self::ZeroFillQuantity => "fill quantity must be non-zero",
            Self::ZeroFillPrice => "fill execution price must be non-zero",
            Self::SubjectMismatch => "observation subject does not match canonical order intent",
            Self::SelfCorrection => "fill correction must reference a distinct replacement fill",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ObservationError {}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketOrderObservationSubjectV1 {
    pub intent_commitment: Digest32,
    pub account_subject: MarketSubjectRefV1,
    pub instrument: MarketInstrumentRefV1,
    pub provider_profile: MarketProfileRefV1,
    pub provider_order_ref: Option<MarketExternalIdV1>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProviderObservationRefV1 {
    pub observation_profile: MarketProfileRefV1,
    pub observation_id: MarketExternalIdV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProviderChronologyV1 {
    pub chronology_profile: Option<MarketProfileRefV1>,
    pub sequence: Option<u64>,
    pub provider_time: Option<MarketExternalIdV1>,
}

impl ProviderChronologyV1 {
    fn validate(&self) -> Result<(), ObservationError> {
        let has_profile = self.chronology_profile.is_some();
        let has_evidence = self.sequence.is_some() || self.provider_time.is_some();
        if has_profile != has_evidence {
            return Err(ObservationError::InvalidChronology);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum MarketEventKindV1 {
    SubmitAttemptObserved,
    ProviderAcceptedObserved,
    PendingNewObserved,
    WorkingObserved,
    CancelRequestedObserved,
    CancelAcceptedObserved,
    CancelRejectedObserved,
    ReplaceRequestedObserved,
    ReplaceAcceptedObserved,
    ReplaceRejectedObserved,
    DoneForDayObserved,
    ExpiredObserved,
    SuspendedOrHaltedObserved,
    ProviderRejectedObserved,
    OpaqueProviderStatus(MarketExternalIdV1),
    SubmissionOutcomeUnknownObserved,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketEventObservationInputV1 {
    pub subject: MarketOrderObservationSubjectV1,
    pub observation_ref: ProviderObservationRefV1,
    pub event_kind: MarketEventKindV1,
    pub chronology: ProviderChronologyV1,
    pub source_evidence_commitment: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CanonicalMarketEventObservationV1 {
    input: MarketEventObservationInputV1,
    identity_commitment: Digest32,
    semantic_commitment: Digest32,
    evidence_binding_commitment: Digest32,
}

impl CanonicalMarketEventObservationV1 {
    pub fn input(&self) -> &MarketEventObservationInputV1 {
        &self.input
    }

    pub const fn identity_commitment(&self) -> Digest32 {
        self.identity_commitment
    }

    pub const fn semantic_commitment(&self) -> Digest32 {
        self.semantic_commitment
    }

    pub const fn evidence_binding_commitment(&self) -> Digest32 {
        self.evidence_binding_commitment
    }

    pub fn canonical_identity_bytes(&self) -> Vec<u8> {
        event_identity_bytes_unchecked(&self.input)
    }

    pub fn canonical_semantic_bytes(&self) -> Vec<u8> {
        event_semantic_bytes_unchecked(&self.input, self.identity_commitment)
    }

    pub fn canonical_evidence_binding_bytes(&self) -> Vec<u8> {
        event_evidence_bytes_unchecked(
            &self.input,
            self.identity_commitment,
            self.semantic_commitment,
        )
    }
}

pub fn canonicalize_event_observation_v1(
    input: MarketEventObservationInputV1,
) -> Result<CanonicalMarketEventObservationV1, ObservationError> {
    input.chronology.validate()?;
    let identity_commitment = sha256_digest(&event_identity_bytes_unchecked(&input));
    let semantic_commitment =
        sha256_digest(&event_semantic_bytes_unchecked(&input, identity_commitment));
    let evidence_binding_commitment = sha256_digest(&event_evidence_bytes_unchecked(
        &input,
        identity_commitment,
        semantic_commitment,
    ));
    Ok(CanonicalMarketEventObservationV1 {
        input,
        identity_commitment,
        semantic_commitment,
        evidence_binding_commitment,
    })
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FillObservationInputV1 {
    pub subject: MarketOrderObservationSubjectV1,
    pub observation_ref: ProviderObservationRefV1,
    pub provider_execution_ref: MarketExternalIdV1,
    pub executed_quantity: MarketQuantityV1,
    pub execution_price: MarketPriceV1,
    pub venue_ref: Option<MarketExternalIdV1>,
    pub chronology: ProviderChronologyV1,
    pub source_evidence_commitment: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CanonicalFillObservationV1 {
    input: FillObservationInputV1,
    identity_commitment: Digest32,
    semantic_commitment: Digest32,
    evidence_binding_commitment: Digest32,
}

impl CanonicalFillObservationV1 {
    pub fn input(&self) -> &FillObservationInputV1 {
        &self.input
    }

    pub const fn identity_commitment(&self) -> Digest32 {
        self.identity_commitment
    }

    pub const fn semantic_commitment(&self) -> Digest32 {
        self.semantic_commitment
    }

    pub const fn evidence_binding_commitment(&self) -> Digest32 {
        self.evidence_binding_commitment
    }

    pub fn canonical_identity_bytes(&self) -> Vec<u8> {
        fill_identity_bytes_unchecked(&self.input)
    }

    pub fn canonical_semantic_bytes(&self) -> Vec<u8> {
        fill_semantic_bytes_unchecked(&self.input, self.identity_commitment)
    }

    pub fn canonical_evidence_binding_bytes(&self) -> Vec<u8> {
        fill_evidence_bytes_unchecked(
            &self.input,
            self.identity_commitment,
            self.semantic_commitment,
        )
    }
}

pub fn canonicalize_fill_observation_v1(
    input: FillObservationInputV1,
) -> Result<CanonicalFillObservationV1, ObservationError> {
    input.chronology.validate()?;
    if input.executed_quantity.amount.atomic_units() == 0 {
        return Err(ObservationError::ZeroFillQuantity);
    }
    if input.execution_price.quote_amount.atomic_units() == 0 {
        return Err(ObservationError::ZeroFillPrice);
    }

    let identity_commitment = sha256_digest(&fill_identity_bytes_unchecked(&input));
    let semantic_commitment =
        sha256_digest(&fill_semantic_bytes_unchecked(&input, identity_commitment));
    let evidence_binding_commitment = sha256_digest(&fill_evidence_bytes_unchecked(
        &input,
        identity_commitment,
        semantic_commitment,
    ));
    Ok(CanonicalFillObservationV1 {
        input,
        identity_commitment,
        semantic_commitment,
        evidence_binding_commitment,
    })
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum FillAdjustmentKindV1 {
    Correction {
        prior_fill_commitment: Digest32,
        replacement_fill_commitment: Digest32,
    },
    Bust {
        prior_fill_commitment: Digest32,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FillAdjustmentObservationInputV1 {
    pub subject: MarketOrderObservationSubjectV1,
    pub observation_ref: ProviderObservationRefV1,
    pub adjustment_kind: FillAdjustmentKindV1,
    pub chronology: ProviderChronologyV1,
    pub source_evidence_commitment: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CanonicalFillAdjustmentObservationV1 {
    input: FillAdjustmentObservationInputV1,
    identity_commitment: Digest32,
    semantic_commitment: Digest32,
    evidence_binding_commitment: Digest32,
}

impl CanonicalFillAdjustmentObservationV1 {
    pub fn input(&self) -> &FillAdjustmentObservationInputV1 {
        &self.input
    }

    pub const fn identity_commitment(&self) -> Digest32 {
        self.identity_commitment
    }

    pub const fn semantic_commitment(&self) -> Digest32 {
        self.semantic_commitment
    }

    pub const fn evidence_binding_commitment(&self) -> Digest32 {
        self.evidence_binding_commitment
    }

    pub fn canonical_identity_bytes(&self) -> Vec<u8> {
        adjustment_identity_bytes_unchecked(&self.input)
    }

    pub fn canonical_semantic_bytes(&self) -> Vec<u8> {
        adjustment_semantic_bytes_unchecked(&self.input, self.identity_commitment)
    }

    pub fn canonical_evidence_binding_bytes(&self) -> Vec<u8> {
        adjustment_evidence_bytes_unchecked(
            &self.input,
            self.identity_commitment,
            self.semantic_commitment,
        )
    }
}

pub fn canonicalize_fill_adjustment_v1(
    input: FillAdjustmentObservationInputV1,
) -> Result<CanonicalFillAdjustmentObservationV1, ObservationError> {
    input.chronology.validate()?;
    if let FillAdjustmentKindV1::Correction {
        prior_fill_commitment,
        replacement_fill_commitment,
    } = &input.adjustment_kind
    {
        if prior_fill_commitment == replacement_fill_commitment {
            return Err(ObservationError::SelfCorrection);
        }
    }

    let identity_commitment = sha256_digest(&adjustment_identity_bytes_unchecked(&input));
    let semantic_commitment =
        sha256_digest(&adjustment_semantic_bytes_unchecked(&input, identity_commitment));
    let evidence_binding_commitment = sha256_digest(&adjustment_evidence_bytes_unchecked(
        &input,
        identity_commitment,
        semantic_commitment,
    ));
    Ok(CanonicalFillAdjustmentObservationV1 {
        input,
        identity_commitment,
        semantic_commitment,
        evidence_binding_commitment,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObservationComparisonV1 {
    DifferentIdentity,
    ExactReplay,
    SameSemanticsDifferentEvidence,
    ConflictingReuse,
}

pub fn classify_observation_commitments_v1(
    left_identity: Digest32,
    left_semantic: Digest32,
    left_evidence_binding: Digest32,
    right_identity: Digest32,
    right_semantic: Digest32,
    right_evidence_binding: Digest32,
) -> ObservationComparisonV1 {
    if left_identity != right_identity {
        ObservationComparisonV1::DifferentIdentity
    } else if left_semantic != right_semantic {
        ObservationComparisonV1::ConflictingReuse
    } else if left_evidence_binding == right_evidence_binding {
        ObservationComparisonV1::ExactReplay
    } else {
        ObservationComparisonV1::SameSemanticsDifferentEvidence
    }
}

pub fn validate_subject_against_intent_v1(
    subject: &MarketOrderObservationSubjectV1,
    intent: &CanonicalMarketOrderIntentV1,
) -> Result<(), ObservationError> {
    if subject.intent_commitment != intent.commitment()
        || subject.account_subject != intent.input().account_subject
        || subject.instrument != intent.input().instrument
    {
        return Err(ObservationError::SubjectMismatch);
    }
    Ok(())
}

fn event_identity_bytes_unchecked(input: &MarketEventObservationInputV1) -> Vec<u8> {
    common_identity_bytes(EVENT_ID_DOMAIN_V1, &input.subject, &input.observation_ref)
}

fn fill_identity_bytes_unchecked(input: &FillObservationInputV1) -> Vec<u8> {
    common_identity_bytes(FILL_ID_DOMAIN_V1, &input.subject, &input.observation_ref)
}

fn adjustment_identity_bytes_unchecked(input: &FillAdjustmentObservationInputV1) -> Vec<u8> {
    common_identity_bytes(ADJUST_ID_DOMAIN_V1, &input.subject, &input.observation_ref)
}

fn common_identity_bytes(
    domain: &[u8],
    subject: &MarketOrderObservationSubjectV1,
    observation_ref: &ProviderObservationRefV1,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(512);
    output.extend_from_slice(domain);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    push_observation_subject(&mut output, subject);
    push_observation_ref(&mut output, observation_ref);
    output
}

fn event_semantic_bytes_unchecked(
    input: &MarketEventObservationInputV1,
    identity_commitment: Digest32,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(288);
    output.extend_from_slice(EVENT_SEM_DOMAIN_V1);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    output.extend_from_slice(identity_commitment.as_bytes());
    push_event_kind(&mut output, &input.event_kind);
    push_chronology(&mut output, &input.chronology);
    output
}

fn event_evidence_bytes_unchecked(
    input: &MarketEventObservationInputV1,
    identity_commitment: Digest32,
    semantic_commitment: Digest32,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(160);
    output.extend_from_slice(EVENT_EVIDENCE_DOMAIN_V1);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    output.extend_from_slice(identity_commitment.as_bytes());
    output.extend_from_slice(semantic_commitment.as_bytes());
    output.extend_from_slice(input.source_evidence_commitment.as_bytes());
    output
}

fn fill_semantic_bytes_unchecked(
    input: &FillObservationInputV1,
    identity_commitment: Digest32,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(608);
    output.extend_from_slice(FILL_SEM_DOMAIN_V1);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    output.extend_from_slice(identity_commitment.as_bytes());
    push_text(&mut output, input.provider_execution_ref.as_str());
    push_quantity(&mut output, &input.executed_quantity);
    push_price(&mut output, &input.execution_price);
    push_optional_external_id(&mut output, input.venue_ref.as_ref());
    push_chronology(&mut output, &input.chronology);
    output
}

fn fill_evidence_bytes_unchecked(
    input: &FillObservationInputV1,
    identity_commitment: Digest32,
    semantic_commitment: Digest32,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(160);
    output.extend_from_slice(FILL_EVIDENCE_DOMAIN_V1);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    output.extend_from_slice(identity_commitment.as_bytes());
    output.extend_from_slice(semantic_commitment.as_bytes());
    output.extend_from_slice(input.source_evidence_commitment.as_bytes());
    output
}

fn adjustment_semantic_bytes_unchecked(
    input: &FillAdjustmentObservationInputV1,
    identity_commitment: Digest32,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(352);
    output.extend_from_slice(ADJUST_SEM_DOMAIN_V1);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    output.extend_from_slice(identity_commitment.as_bytes());
    push_adjustment_kind(&mut output, &input.adjustment_kind);
    push_chronology(&mut output, &input.chronology);
    output
}

fn adjustment_evidence_bytes_unchecked(
    input: &FillAdjustmentObservationInputV1,
    identity_commitment: Digest32,
    semantic_commitment: Digest32,
) -> Vec<u8> {
    let mut output = Vec::with_capacity(160);
    output.extend_from_slice(ADJUST_EVIDENCE_DOMAIN_V1);
    output.extend_from_slice(&OBSERVATION_PROFILE_REVISION_V1.to_be_bytes());
    output.extend_from_slice(identity_commitment.as_bytes());
    output.extend_from_slice(semantic_commitment.as_bytes());
    output.extend_from_slice(input.source_evidence_commitment.as_bytes());
    output
}

fn push_observation_subject(output: &mut Vec<u8>, value: &MarketOrderObservationSubjectV1) {
    output.extend_from_slice(value.intent_commitment.as_bytes());
    push_subject_ref(output, &value.account_subject);
    push_instrument_ref(output, &value.instrument);
    push_profile_ref(output, &value.provider_profile);
    push_optional_external_id(output, value.provider_order_ref.as_ref());
}

fn push_observation_ref(output: &mut Vec<u8>, value: &ProviderObservationRefV1) {
    push_profile_ref(output, &value.observation_profile);
    push_text(output, value.observation_id.as_str());
}

fn push_event_kind(output: &mut Vec<u8>, value: &MarketEventKindV1) {
    match value {
        MarketEventKindV1::SubmitAttemptObserved => output.push(0),
        MarketEventKindV1::ProviderAcceptedObserved => output.push(1),
        MarketEventKindV1::PendingNewObserved => output.push(2),
        MarketEventKindV1::WorkingObserved => output.push(3),
        MarketEventKindV1::CancelRequestedObserved => output.push(4),
        MarketEventKindV1::CancelAcceptedObserved => output.push(5),
        MarketEventKindV1::CancelRejectedObserved => output.push(6),
        MarketEventKindV1::ReplaceRequestedObserved => output.push(7),
        MarketEventKindV1::ReplaceAcceptedObserved => output.push(8),
        MarketEventKindV1::ReplaceRejectedObserved => output.push(9),
        MarketEventKindV1::DoneForDayObserved => output.push(10),
        MarketEventKindV1::ExpiredObserved => output.push(11),
        MarketEventKindV1::SuspendedOrHaltedObserved => output.push(12),
        MarketEventKindV1::ProviderRejectedObserved => output.push(13),
        MarketEventKindV1::OpaqueProviderStatus(status) => {
            output.push(14);
            push_text(output, status.as_str());
        }
        MarketEventKindV1::SubmissionOutcomeUnknownObserved => output.push(15),
    }
}

fn push_adjustment_kind(output: &mut Vec<u8>, value: &FillAdjustmentKindV1) {
    match value {
        FillAdjustmentKindV1::Correction {
            prior_fill_commitment,
            replacement_fill_commitment,
        } => {
            output.push(0);
            output.extend_from_slice(prior_fill_commitment.as_bytes());
            output.extend_from_slice(replacement_fill_commitment.as_bytes());
        }
        FillAdjustmentKindV1::Bust { prior_fill_commitment } => {
            output.push(1);
            output.extend_from_slice(prior_fill_commitment.as_bytes());
        }
    }
}

fn push_chronology(output: &mut Vec<u8>, value: &ProviderChronologyV1) {
    match &value.chronology_profile {
        None => output.push(0),
        Some(profile) => {
            output.push(1);
            push_profile_ref(output, profile);
            match value.sequence {
                None => output.push(0),
                Some(sequence) => {
                    output.push(1);
                    output.extend_from_slice(&sequence.to_be_bytes());
                }
            }
            push_optional_external_id(output, value.provider_time.as_ref());
        }
    }
}

fn push_quantity(output: &mut Vec<u8>, value: &MarketQuantityV1) {
    push_profile_ref(output, &value.unit_profile);
    output.extend_from_slice(&value.amount.atomic_units().to_be_bytes());
    push_text(output, value.amount.asset().as_str());
}

fn push_price(output: &mut Vec<u8>, value: &MarketPriceV1) {
    push_profile_ref(output, &value.pricing_profile);
    output.extend_from_slice(&value.quote_amount.atomic_units().to_be_bytes());
    push_text(output, value.quote_amount.asset().as_str());
}

fn push_profile_ref(output: &mut Vec<u8>, value: &MarketProfileRefV1) {
    push_text(output, value.profile_id.as_str());
    output.extend_from_slice(&value.revision.to_be_bytes());
    output.extend_from_slice(value.digest.as_bytes());
}

fn push_subject_ref(output: &mut Vec<u8>, value: &MarketSubjectRefV1) {
    push_profile_ref(output, &value.subject_profile);
    push_text(output, value.subject_id.as_str());
}

fn push_instrument_ref(output: &mut Vec<u8>, value: &MarketInstrumentRefV1) {
    push_profile_ref(output, &value.instrument_profile);
    push_text(output, value.instrument_id.as_str());
}

fn push_optional_external_id(output: &mut Vec<u8>, value: Option<&MarketExternalIdV1>) {
    match value {
        None => output.push(0),
        Some(value) => {
            output.push(1);
            push_text(output, value.as_str());
        }
    }
}

fn push_text(output: &mut Vec<u8>, value: &str) {
    let len = u32::try_from(value.len()).expect("bounded FIN-MKT identifiers fit u32");
    output.extend_from_slice(&len.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
}

fn sha256_digest(bytes: &[u8]) -> Digest32 {
    let hash = Sha256::digest(bytes);
    let mut output = [0_u8; 32];
    output.copy_from_slice(&hash);
    Digest32::from_bytes(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    const EVENT_ID: &str =
        "bd0a7c994a58859f7fd99455433def8e9bfda2781149e90a23192042d60a8b19";
    const EVENT_SEM: &str =
        "e1cb0ee74f32f092ce23eae260196d7edff1f4db6f65619a65ad3df5bd740d57";
    const EVENT_EVIDENCE: &str =
        "dc2a1ad70de2b9219da30e5cd9af8cd78473008ab7eca83204ec9b6d3363345c";
    const FILL_ID: &str =
        "b6fb55af9d6f0ed1e4b2d5f696f9d8e8bf659efcd20c2ee7d807f492cd8dd134";
    const FILL_SEM: &str =
        "23121d3de29ccc6f5dce122a7b7475c6ebd86670a063e5ea54ff9901e82ee196";
    const FILL_EVIDENCE: &str =
        "19816667fa931cee5b9ecd9a8579d8d39facc3927c05719dbb736c4f3cad71d0";
    const ADJUST_ID: &str =
        "f33bcce43385ce6d1f310439360baed8e3dbcc656890ce0e9ba286abbe2ba4d1";
    const ADJUST_SEM: &str =
        "62e2abcd01078dba27daed66d69e0a1fa9d9a43b6333749f051e5f433d792e73";
    const ADJUST_EVIDENCE: &str =
        "fcec2b73c14fbd78eaf4e18273fdeb2ada4655fe7bed5f337cb163e3b976c476";

    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Fixture {
        event: MarketEventObservationInputV1,
        fill: FillObservationInputV1,
        correction: FillAdjustmentObservationInputV1,
    }

    fn fixture() -> Fixture {
        serde_json::from_str(include_str!("../test-vectors/observations-v1.json")).unwrap()
    }

    #[test]
    fn frozen_independent_vectors_match() {
        let fixture = fixture();
        let event = canonicalize_event_observation_v1(fixture.event).unwrap();
        let fill = canonicalize_fill_observation_v1(fixture.fill).unwrap();
        let correction = canonicalize_fill_adjustment_v1(fixture.correction).unwrap();

        assert_eq!(event.canonical_identity_bytes().len(), 419);
        assert_eq!(event.canonical_semantic_bytes().len(), 171);
        assert_eq!(event.canonical_evidence_binding_bytes().len(), 134);
        assert_eq!(event.identity_commitment().to_hex(), EVENT_ID);
        assert_eq!(event.semantic_commitment().to_hex(), EVENT_SEM);
        assert_eq!(event.evidence_binding_commitment().to_hex(), EVENT_EVIDENCE);

        assert_eq!(fill.canonical_identity_bytes().len(), 422);
        assert_eq!(fill.canonical_semantic_bytes().len(), 409);
        assert_eq!(fill.canonical_evidence_binding_bytes().len(), 133);
        assert_eq!(fill.identity_commitment().to_hex(), FILL_ID);
        assert_eq!(fill.semantic_commitment().to_hex(), FILL_SEM);
        assert_eq!(fill.evidence_binding_commitment().to_hex(), FILL_EVIDENCE);

        assert_eq!(correction.canonical_identity_bytes().len(), 429);
        assert_eq!(correction.canonical_semantic_bytes().len(), 236);
        assert_eq!(correction.canonical_evidence_binding_bytes().len(), 135);
        assert_eq!(correction.identity_commitment().to_hex(), ADJUST_ID);
        assert_eq!(correction.semantic_commitment().to_hex(), ADJUST_SEM);
        assert_eq!(correction.evidence_binding_commitment().to_hex(), ADJUST_EVIDENCE);
    }

    #[test]
    fn exact_replay_semantic_alias_and_conflict_are_distinct() {
        let fixture = fixture();
        let original = canonicalize_event_observation_v1(fixture.event.clone()).unwrap();
        let duplicate = canonicalize_event_observation_v1(fixture.event.clone()).unwrap();
        assert_eq!(
            classify_observation_commitments_v1(
                original.identity_commitment(),
                original.semantic_commitment(),
                original.evidence_binding_commitment(),
                duplicate.identity_commitment(),
                duplicate.semantic_commitment(),
                duplicate.evidence_binding_commitment(),
            ),
            ObservationComparisonV1::ExactReplay
        );

        let mut alternate_evidence = fixture.event.clone();
        alternate_evidence.source_evidence_commitment = Digest32::from_bytes([0xBC; 32]);
        let alternate = canonicalize_event_observation_v1(alternate_evidence).unwrap();
        assert_eq!(original.identity_commitment(), alternate.identity_commitment());
        assert_eq!(original.semantic_commitment(), alternate.semantic_commitment());
        assert_ne!(
            original.evidence_binding_commitment(),
            alternate.evidence_binding_commitment()
        );
        assert_eq!(
            classify_observation_commitments_v1(
                original.identity_commitment(),
                original.semantic_commitment(),
                original.evidence_binding_commitment(),
                alternate.identity_commitment(),
                alternate.semantic_commitment(),
                alternate.evidence_binding_commitment(),
            ),
            ObservationComparisonV1::SameSemanticsDifferentEvidence
        );

        let mut changed = fixture.event;
        changed.event_kind = MarketEventKindV1::CancelRequestedObserved;
        let conflict = canonicalize_event_observation_v1(changed).unwrap();
        assert_eq!(original.identity_commitment(), conflict.identity_commitment());
        assert_ne!(original.semantic_commitment(), conflict.semantic_commitment());
        assert_eq!(
            classify_observation_commitments_v1(
                original.identity_commitment(),
                original.semantic_commitment(),
                original.evidence_binding_commitment(),
                conflict.identity_commitment(),
                conflict.semantic_commitment(),
                conflict.evidence_binding_commitment(),
            ),
            ObservationComparisonV1::ConflictingReuse
        );
    }

    #[test]
    fn provider_observation_id_is_scoped_by_subject() {
        let fixture = fixture();
        let original = canonicalize_event_observation_v1(fixture.event.clone()).unwrap();
        let mut changed = fixture.event;
        changed.subject.account_subject.subject_id =
            MarketExternalIdV1::new("account:demo:002").unwrap();
        let other = canonicalize_event_observation_v1(changed).unwrap();
        assert_ne!(original.identity_commitment(), other.identity_commitment());
    }

    #[test]
    fn observation_subject_must_match_exact_intent() {
        let fixture = fixture();
        let intent_input: mycelix_finance_market_core::MarketOrderIntentInputV1 =
            serde_json::from_str(include_str!(
                "../../finance-market-core/test-vectors/order-intent-v1.json"
            ))
            .unwrap();
        let intent =
            mycelix_finance_market_core::canonicalize_order_intent_v1(intent_input).unwrap();

        validate_subject_against_intent_v1(&fixture.event.subject, &intent).unwrap();

        let mut wrong = fixture.event.subject;
        wrong.instrument.instrument_id = MarketExternalIdV1::new("instrument:demo:MSFT").unwrap();
        assert_eq!(
            validate_subject_against_intent_v1(&wrong, &intent),
            Err(ObservationError::SubjectMismatch)
        );
    }

    #[test]
    fn zero_fill_quantity_and_price_fail_closed() {
        let mut value: serde_json::Value =
            serde_json::from_str(include_str!("../test-vectors/observations-v1.json")).unwrap();
        value["fill"]["executed_quantity"]["amount"]["atomic_units"] = 0_u64.into();
        let zero_quantity: Fixture = serde_json::from_value(value).unwrap();
        assert_eq!(
            canonicalize_fill_observation_v1(zero_quantity.fill),
            Err(ObservationError::ZeroFillQuantity)
        );

        let mut value: serde_json::Value =
            serde_json::from_str(include_str!("../test-vectors/observations-v1.json")).unwrap();
        value["fill"]["execution_price"]["quote_amount"]["atomic_units"] = 0_u64.into();
        let zero_price: Fixture = serde_json::from_value(value).unwrap();
        assert_eq!(
            canonicalize_fill_observation_v1(zero_price.fill),
            Err(ObservationError::ZeroFillPrice)
        );
    }

    #[test]
    fn chronology_requires_an_explicit_profile() {
        let fixture = fixture();
        let mut changed = fixture.event;
        changed.chronology.chronology_profile = None;
        assert_eq!(
            canonicalize_event_observation_v1(changed),
            Err(ObservationError::InvalidChronology)
        );
    }

    #[test]
    fn submission_unknown_is_not_rejection() {
        assert_ne!(
            MarketEventKindV1::SubmissionOutcomeUnknownObserved,
            MarketEventKindV1::ProviderRejectedObserved
        );
    }

    #[test]
    fn correction_fixture_binds_exact_prior_fill_semantics() {
        let fixture = fixture();
        let fill = canonicalize_fill_observation_v1(fixture.fill).unwrap();
        match fixture.correction.adjustment_kind {
            FillAdjustmentKindV1::Correction {
                prior_fill_commitment,
                replacement_fill_commitment: _,
            } => assert_eq!(prior_fill_commitment, fill.semantic_commitment()),
            FillAdjustmentKindV1::Bust { .. } => panic!("fixture must be a correction"),
        }
    }

    #[test]
    fn correction_cannot_replace_a_fill_with_itself() {
        let fixture = fixture();
        let mut changed = fixture.correction;
        let same = Digest32::from_lower_hex(FILL_SEM).unwrap();
        changed.adjustment_kind = FillAdjustmentKindV1::Correction {
            prior_fill_commitment: same,
            replacement_fill_commitment: same,
        };
        assert_eq!(
            canonicalize_fill_adjustment_v1(changed),
            Err(ObservationError::SelfCorrection)
        );
    }

    #[test]
    fn unknown_fields_fail_closed() {
        let mut value: serde_json::Value =
            serde_json::from_str(include_str!("../test-vectors/observations-v1.json")).unwrap();
        value["event"]["authorized"] = serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<Fixture>(value).is_err());

        let mut value: serde_json::Value =
            serde_json::from_str(include_str!("../test-vectors/observations-v1.json")).unwrap();
        value["fill"]["settled"] = serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<Fixture>(value).is_err());
    }
}
