// Copyright (C) 2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Scoped botanical-material transfer preflight for Mycelix.
//!
//! This crate evaluates identity and purpose alignment in a proposed transfer.
//! It does not authenticate a species, verify evidence bytes, establish that a
//! permit is legally required or valid, determine material quality, grant
//! consent, or authorize physical movement. An empty blocker list means only
//! that the supplied record references are structurally in-scope for custodian
//! review. Traditional or sacred knowledge is not represented by these types;
//! material-transfer scope never implies knowledge-use permission.

use serde::{Deserialize, Serialize};

pub const BOTANICAL_TRANSFER_SCHEMA_VERSION: u16 = 1;

/// A raw botanical lot's reviewed workflow state, not a clinical or safety claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialLotState {
    Reported,
    IdentityPending,
    QualityPending,
    AcceptedForDeclaredUse,
    Rejected,
    Quarantined,
    Recalled,
    Exhausted,
    Unknown,
    Superseded,
}

/// An intended transfer purpose. Each purpose needs its own external rights review.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialTransferPurpose {
    TaxonomicIdentification,
    LaboratoryQualityAssessment,
    NonCommercialResearch,
    AuthorizedBuyerIntake,
    CommercialBioprospecting,
    Export,
}

/// State of an externally reviewed rights/consent decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RightsDecisionState {
    Draft,
    ReviewRequired,
    ConsentPending,
    AuthorizedForDeclaredPurpose,
    Restricted,
    Suspended,
    Withdrawn,
    Expired,
    Superseded,
}

/// Minimal lot identity. Sensitive site coordinates, personal data and knowledge payloads
/// deliberately do not appear in this shared record.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BotanicalMaterialLotV1 {
    pub schema_version: u16,
    pub lot_id: String,
    /// Reference to the separately reviewed taxonomic assessment, not a taxon assertion.
    pub taxonomic_assessment_ref: Option<String>,
    /// Exact rights-decision identity required for this lot's transfer preflight.
    pub rights_assessment_ref: Option<String>,
    pub state: MaterialLotState,
}

/// Requested hand-off. The request records intended scope; it does not authorize movement.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaterialTransferRequestV1 {
    pub schema_version: u16,
    pub lot_id: String,
    pub recipient_ref: String,
    pub purpose: MaterialTransferPurpose,
}

/// External, purpose- and recipient-scoped decision metadata.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaterialRightsAssessmentV1 {
    pub schema_version: u16,
    pub assessment_id: String,
    pub subject_lot_id: String,
    pub recipient_ref: String,
    pub purpose: MaterialTransferPurpose,
    pub state: RightsDecisionState,
    /// Inclusive start of the reviewed decision's validity window (Unix seconds).
    pub valid_from_unix_s: u64,
    /// Exclusive end of the reviewed decision's validity window (Unix seconds).
    pub valid_until_unix_s: u64,
    /// Reference to the reviewed source decision/permit/consent record.
    pub evidence_ref: Option<String>,
    /// Reference to the accountable reviewer or decision-making body.
    pub reviewer_ref: Option<String>,
}

/// Stable reasons for failing a proposed transfer preflight.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferBlockerCode {
    UnsupportedLotSchema,
    UnsupportedRequestSchema,
    UnsupportedRightsSchema,
    EmptyLotId,
    EmptyRightsAssessmentId,
    EmptyRecipient,
    MissingTaxonomicAssessment,
    LotRequestMismatch,
    LotRightsReferenceMismatch,
    RightsSubjectMismatch,
    RightsRecipientMismatch,
    RightsPurposeMismatch,
    RightsNotAuthorized,
    InvalidLotStateForPurpose,
    InvalidRightsValidityWindow,
    RightsNotYetValid,
    RightsExpiredAtEvaluation,
    MissingRightsEvidence,
    MissingRightsReviewer,
}

/// Structured blocker, ordered by the evaluator's fixed validation sequence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransferBlockerV1 {
    pub code: TransferBlockerCode,
    pub detail: String,
}

/// A conservative preflight result. This is deliberately not called "authorized".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferPreflightDisposition {
    PreflightCompleteForCustodianReview,
    Blocked,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransferPreflightDecisionV1 {
    pub lot_id: String,
    pub disposition: TransferPreflightDisposition,
    pub blockers: Vec<TransferBlockerV1>,
}

/// Evaluate record shape and exact scope before a qualified custodian reviews the hand-off.
///
/// Even a preflight-complete result cannot override applicable law, a site's operating
/// procedures, quarantine decisions, or external approval requirements.
pub fn evaluate_material_transfer(
    lot: &BotanicalMaterialLotV1,
    request: &MaterialTransferRequestV1,
    rights: &MaterialRightsAssessmentV1,
    evaluation_time_unix_s: u64,
) -> TransferPreflightDecisionV1 {
    let mut blockers = Vec::new();

    if lot.schema_version != BOTANICAL_TRANSFER_SCHEMA_VERSION {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::UnsupportedLotSchema,
            "unsupported material-lot schema version",
        );
    }
    if request.schema_version != BOTANICAL_TRANSFER_SCHEMA_VERSION {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::UnsupportedRequestSchema,
            "unsupported transfer-request schema version",
        );
    }
    if rights.schema_version != BOTANICAL_TRANSFER_SCHEMA_VERSION {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::UnsupportedRightsSchema,
            "unsupported rights-assessment schema version",
        );
    }
    if lot.lot_id.trim().is_empty() || request.lot_id.trim().is_empty() {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::EmptyLotId,
            "lot identifiers must be non-empty",
        );
    }
    if rights.assessment_id.trim().is_empty() {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::EmptyRightsAssessmentId,
            "rights assessment identity is missing",
        );
    }
    if request.recipient_ref.trim().is_empty() || rights.recipient_ref.trim().is_empty() {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::EmptyRecipient,
            "recipient references must be non-empty",
        );
    }
    if request.lot_id != lot.lot_id {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::LotRequestMismatch,
            "transfer request names a different lot",
        );
    }
    if lot.rights_assessment_ref.as_deref() != Some(rights.assessment_id.as_str()) {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::LotRightsReferenceMismatch,
            "lot does not reference this exact rights assessment",
        );
    }
    if rights.subject_lot_id != lot.lot_id {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::RightsSubjectMismatch,
            "rights decision is scoped to a different lot",
        );
    }
    if rights.recipient_ref != request.recipient_ref {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::RightsRecipientMismatch,
            "rights decision does not cover the requested recipient",
        );
    }
    if rights.purpose != request.purpose {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::RightsPurposeMismatch,
            "rights decision does not cover the requested purpose",
        );
    }
    if rights.valid_from_unix_s >= rights.valid_until_unix_s {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::InvalidRightsValidityWindow,
            "rights validity window must have valid_from earlier than valid_until",
        );
    } else if evaluation_time_unix_s < rights.valid_from_unix_s {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::RightsNotYetValid,
            "rights decision is not yet valid at evaluation time",
        );
    } else if evaluation_time_unix_s >= rights.valid_until_unix_s {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::RightsExpiredAtEvaluation,
            "rights decision is expired at evaluation time",
        );
    }
    if rights.state != RightsDecisionState::AuthorizedForDeclaredPurpose {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::RightsNotAuthorized,
            "rights state is not current authorization for the declared purpose",
        );
    }
    if requires_taxonomic_assessment(request.purpose)
        && !non_empty(lot.taxonomic_assessment_ref.as_deref())
    {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::MissingTaxonomicAssessment,
            "this downstream transfer purpose requires a taxonomic assessment reference",
        );
    }
    if !lot_state_allows_purpose(lot.state, request.purpose) {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::InvalidLotStateForPurpose,
            "lot state cannot be used for the requested transfer purpose",
        );
    }
    if !non_empty(rights.evidence_ref.as_deref()) {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::MissingRightsEvidence,
            "rights decision requires an evidence reference",
        );
    }
    if !non_empty(rights.reviewer_ref.as_deref()) {
        push_blocker(
            &mut blockers,
            TransferBlockerCode::MissingRightsReviewer,
            "rights decision requires an accountable reviewer reference",
        );
    }

    let disposition = if blockers.is_empty() {
        TransferPreflightDisposition::PreflightCompleteForCustodianReview
    } else {
        TransferPreflightDisposition::Blocked
    };

    TransferPreflightDecisionV1 {
        lot_id: lot.lot_id.clone(),
        disposition,
        blockers,
    }
}

/// Downstream buyer/commercial/export use must reference a separate taxonomic
/// assessment. The reference is not authenticated by this preflight; taxonomy remains
/// an externally reviewed authority and the referenced bytes/decision must be checked
/// by the responsible integration before actual movement.
fn requires_taxonomic_assessment(purpose: MaterialTransferPurpose) -> bool {
    matches!(
        purpose,
        MaterialTransferPurpose::AuthorizedBuyerIntake
            | MaterialTransferPurpose::CommercialBioprospecting
            | MaterialTransferPurpose::Export
    )
}

fn lot_state_allows_purpose(
    state: MaterialLotState,
    purpose: MaterialTransferPurpose,
) -> bool {
    match purpose {
        MaterialTransferPurpose::TaxonomicIdentification
        | MaterialTransferPurpose::LaboratoryQualityAssessment => matches!(
            state,
            MaterialLotState::Reported
                | MaterialLotState::IdentityPending
                | MaterialLotState::QualityPending
                | MaterialLotState::AcceptedForDeclaredUse
        ),
        MaterialTransferPurpose::NonCommercialResearch
        | MaterialTransferPurpose::AuthorizedBuyerIntake
        | MaterialTransferPurpose::CommercialBioprospecting
        | MaterialTransferPurpose::Export => {
            state == MaterialLotState::AcceptedForDeclaredUse
        }
    }
}

fn non_empty(value: Option<&str>) -> bool {
    value.is_some_and(|text| !text.trim().is_empty())
}

fn push_blocker(
    blockers: &mut Vec<TransferBlockerV1>,
    code: TransferBlockerCode,
    detail: &str,
) {
    blockers.push(TransferBlockerV1 {
        code,
        detail: detail.to_owned(),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_fixture() -> (
        BotanicalMaterialLotV1,
        MaterialTransferRequestV1,
        MaterialRightsAssessmentV1,
    ) {
        let lot = BotanicalMaterialLotV1 {
            schema_version: BOTANICAL_TRANSFER_SCHEMA_VERSION,
            lot_id: "BOT-LOT-001".to_owned(),
            taxonomic_assessment_ref: Some("TAXON-REVIEW-001".to_owned()),
            rights_assessment_ref: Some("RIGHTS-001".to_owned()),
            state: MaterialLotState::AcceptedForDeclaredUse,
        };
        let request = MaterialTransferRequestV1 {
            schema_version: BOTANICAL_TRANSFER_SCHEMA_VERSION,
            lot_id: "BOT-LOT-001".to_owned(),
            recipient_ref: "RECIPIENT-LAB-01".to_owned(),
            purpose: MaterialTransferPurpose::LaboratoryQualityAssessment,
        };
        let rights = MaterialRightsAssessmentV1 {
            schema_version: BOTANICAL_TRANSFER_SCHEMA_VERSION,
            assessment_id: "RIGHTS-001".to_owned(),
            subject_lot_id: "BOT-LOT-001".to_owned(),
            recipient_ref: "RECIPIENT-LAB-01".to_owned(),
            purpose: MaterialTransferPurpose::LaboratoryQualityAssessment,
            state: RightsDecisionState::AuthorizedForDeclaredPurpose,
            valid_from_unix_s: 1_800_000_000,
            valid_until_unix_s: 1_900_000_000,
            evidence_ref: Some("EXTERNAL-REVIEW-001".to_owned()),
            reviewer_ref: Some("REVIEWER-001".to_owned()),
        };
        (lot, request, rights)
    }

    const EVALUATION_TIME_UNIX_S: u64 = 1_850_000_000;

    fn evaluate_fixture(
        lot: &BotanicalMaterialLotV1,
        request: &MaterialTransferRequestV1,
        rights: &MaterialRightsAssessmentV1,
    ) -> TransferPreflightDecisionV1 {
        evaluate_material_transfer(lot, request, rights, EVALUATION_TIME_UNIX_S)
    }

    #[test]
    fn exact_current_scope_reaches_custodian_review_only() {
        let (lot, request, rights) = valid_fixture();
        let result = evaluate_fixture(&lot, &request, &rights);
        assert_eq!(
            result.disposition,
            TransferPreflightDisposition::PreflightCompleteForCustodianReview
        );
        assert!(result.blockers.is_empty());
    }

    #[test]
    fn rights_window_is_start_inclusive_and_end_exclusive() {
        let (lot, request, mut rights) = valid_fixture();
        assert_eq!(
            evaluate_material_transfer(&lot, &request, &rights, rights.valid_from_unix_s).disposition,
            TransferPreflightDisposition::PreflightCompleteForCustodianReview
        );
        rights.valid_from_unix_s = 1_800_000_000;
        let at_expiry =
            evaluate_material_transfer(&lot, &request, &rights, rights.valid_until_unix_s);
        assert!(at_expiry.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::RightsExpiredAtEvaluation
        }));
    }

    #[test]
    fn malformed_rights_window_is_blocked() {
        let (lot, request, mut rights) = valid_fixture();
        rights.valid_from_unix_s = rights.valid_until_unix_s;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::InvalidRightsValidityWindow
        }));
    }

    #[test]
    fn future_rights_window_is_blocked() {
        let (lot, request, mut rights) = valid_fixture();
        rights.valid_from_unix_s = EVALUATION_TIME_UNIX_S + 1;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::RightsNotYetValid
        }));
    }

    #[test]
    fn recipient_mismatch_is_blocked() {
        let (lot, request, mut rights) = valid_fixture();
        rights.recipient_ref = "RECIPIENT-OTHER".to_owned();
        let result = evaluate_fixture(&lot, &request, &rights);
        assert_eq!(result.disposition, TransferPreflightDisposition::Blocked);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::RightsRecipientMismatch
        }));
    }

    #[test]
    fn purpose_mismatch_is_blocked_even_when_recipient_matches() {
        let (lot, request, mut rights) = valid_fixture();
        rights.purpose = MaterialTransferPurpose::CommercialBioprospecting;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::RightsPurposeMismatch
        }));
    }

    #[test]
    fn withdrawn_rights_never_pass() {
        let (lot, request, mut rights) = valid_fixture();
        rights.state = RightsDecisionState::Withdrawn;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::RightsNotAuthorized
        }));
    }

    #[test]
    fn quarantined_lot_never_passes() {
        let (mut lot, request, rights) = valid_fixture();
        lot.state = MaterialLotState::Quarantined;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::InvalidLotStateForPurpose
        }));
    }

    #[test]
    fn buyer_intake_requires_accepted_for_declared_use() {
        let (mut lot, mut request, mut rights) = valid_fixture();
        lot.state = MaterialLotState::QualityPending;
        request.purpose = MaterialTransferPurpose::AuthorizedBuyerIntake;
        rights.purpose = MaterialTransferPurpose::AuthorizedBuyerIntake;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::InvalidLotStateForPurpose
        }));
    }

    #[test]
    fn commercial_or_buyer_transfer_requires_taxonomic_assessment_reference() {
        let (mut lot, mut request, mut rights) = valid_fixture();
        lot.taxonomic_assessment_ref = None;
        request.purpose = MaterialTransferPurpose::CommercialBioprospecting;
        rights.purpose = MaterialTransferPurpose::CommercialBioprospecting;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::MissingTaxonomicAssessment
        }));
    }

    #[test]
    fn rights_reference_must_match_the_lots_exact_assessment() {
        let (mut lot, request, rights) = valid_fixture();
        lot.rights_assessment_ref = Some("RIGHTS-OLD".to_owned());
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::LotRightsReferenceMismatch
        }));
    }

    #[test]
    fn missing_review_evidence_blocks_even_an_authorized_status() {
        let (lot, request, mut rights) = valid_fixture();
        rights.evidence_ref = None;
        rights.reviewer_ref = None;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::MissingRightsEvidence
        }));
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::MissingRightsReviewer
        }));
    }

    #[test]
    fn noncommercial_research_permission_does_not_imply_commercial_use() {
        let (lot, mut request, mut rights) = valid_fixture();
        request.purpose = MaterialTransferPurpose::NonCommercialResearch;
        rights.purpose = MaterialTransferPurpose::NonCommercialResearch;
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.is_empty());

        request.purpose = MaterialTransferPurpose::CommercialBioprospecting;
        let changed_scope = evaluate_fixture(&lot, &request, &rights);
        assert!(changed_scope.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::RightsPurposeMismatch
        }));
    }

    #[test]
    fn wrong_lot_identity_is_blocked() {
        let (lot, mut request, rights) = valid_fixture();
        request.lot_id = "BOT-LOT-OTHER".to_owned();
        let result = evaluate_fixture(&lot, &request, &rights);
        assert!(result.blockers.iter().any(|item| {
            item.code == TransferBlockerCode::LotRequestMismatch
        }));
    }
}
