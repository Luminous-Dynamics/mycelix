// Copyright (C) 2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Versioned source-snapshot and reuse-rights contract for Mycelix-DeSci.
//!
//! Rights, evidence quality, clinical validity, regulatory authorization and
//! permission to manufacture are different questions. This module evaluates
//! only the requested content-reuse operation against supplied metadata.
//! It does not authenticate licenses/reviewers, inspect remote websites,
//! hash artifact bytes, or grant legal permission.
//!
//! When reuse rights are unknown, only a reference pointer is allowed by this
//! evaluator. That is not a legal opinion that every reference/citation is
//! permitted under every contract or jurisdiction; source-specific review may
//! impose additional restrictions.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const SOURCE_SNAPSHOT_SCHEMA_V1: u16 = 1;
pub const RIGHTS_ASSESSMENT_SCHEMA_V1: u16 = 1;

/// Identifies source metadata and the exact captured artifact, if any.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceSnapshotRefV1 {
    pub schema_version: u16,
    /// Provider namespace; must not be a free-form description.
    pub provider_id: String,
    /// Provider-stable record ID, accession, DOI, release ID or equivalent.
    pub source_record_id: String,
    /// Canonical source URL. This contract does not fetch it.
    pub canonical_url: String,
    /// Dataset, edition, release or revision if the provider defines one.
    pub source_version: Option<String>,
    /// Source publication time, separate from retrieval time.
    pub published_at_utc: Option<String>,
    /// When this source record was captured by the local adapter.
    pub retrieved_at_utc: String,
    /// Lowercase SHA-256 of the captured artifact when one is available.
    pub artifact_sha256: Option<String>,
    /// Evidence status reported by the independent capture/verification path.
    pub digest_status: DigestVerificationStatus,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum DigestVerificationStatus {
    NotCaptured,
    CapturedNotVerified,
    VerificationReceiptReferenced { receipt_ref: String },
}

/// A review state, not a confidence score. Only Verified assessments can
/// authorize operations beyond a source reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LicenseReviewStatus {
    Verified,
    Unknown,
    ReviewRequired,
    Expired,
}

/// Reuse operations are intentionally separate. Granting one does not grant
/// any other (for example, redistribution does not imply model-training use).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReuseOperation {
    ReferenceSource,
    StoreMinimalMetadata,
    StoreFullArtifact,
    RedistributeArtifact,
    CreateTranslation,
    CommercialUse,
    TrainModel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PermissionDisposition {
    Granted,
    Denied,
    NotAssessed,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OperationPermission {
    pub operation: ReuseOperation,
    pub disposition: PermissionDisposition,
}

/// Terms/rights review tied to a source or dataset. The permission set is
/// explicit and non-transitive; absent operations are not granted.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RightsAndLicenseAssessmentV1 {
    pub schema_version: u16,
    pub assessment_id: String,
    /// Exact provider/record/version to which this review applies.
    pub subject_provider_id: String,
    pub subject_record_id: String,
    pub subject_version: Option<String>,
    /// Optional artifact-specific scope. None means the reviewed terms apply
    /// to this source record/version, rather than a single byte blob.
    pub subject_artifact_sha256: Option<String>,
    pub review_status: LicenseReviewStatus,
    /// SPDX expression, license URL or another stable terms identifier.
    pub license_identifier: Option<String>,
    /// Stable ID of the accountable reviewer or reviewing organization.
    pub reviewer_id: Option<String>,
    /// Evidence reference for the actual terms/permission assessment.
    pub review_evidence_ref: Option<String>,
    pub reviewed_at_utc: Option<String>,
    /// Required for Verified assessments to force periodic re-review.
    pub valid_until_utc: Option<String>,
    pub attribution_required: bool,
    pub attribution_text: Option<String>,
    pub permissions: Vec<OperationPermission>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReuseBlockerCode {
    UnsupportedSnapshotSchema,
    EmptyProviderId,
    EmptySourceRecordId,
    InvalidSourceUrl,
    EmptySourceVersion,
    InvalidPublishedTimestamp,
    InvalidRetrievedTimestamp,
    MissingArtifactDigest,
    InvalidArtifactDigest,
    DigestStateMismatch,
    DigestVerificationReceiptRequired,
    MissingDigestVerificationReceipt,
    InvalidEvaluationTimestamp,
    EmptyRightsProviderScope,
    EmptyRightsRecordScope,
    RightsProviderMismatch,
    RightsRecordMismatch,
    RightsVersionMismatch,
    RightsArtifactMismatch,
    SourceRetrievedInFuture,
    UnsupportedRightsSchema,
    EmptyAssessmentId,
    LicenseNotVerified,
    MissingLicenseIdentifier,
    MissingReviewerId,
    MissingReviewEvidence,
    InvalidReviewTimestamp,
    ReviewInFuture,
    MissingValidityEnd,
    InvalidValidityWindow,
    AssessmentExpired,
    DuplicateOperationPermission,
    PermissionNotAssessed,
    PermissionDenied,
    PermissionNotGranted,
    AttributionTextMissing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReuseCaveatCode {
    ReferenceOnlyDoesNotGrantContentReuse,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReuseDecisionV1 {
    pub operation: ReuseOperation,
    pub allowed_by_supplied_assessment: bool,
    pub blockers: Vec<ReuseBlockerCode>,
    pub caveats: Vec<ReuseCaveatCode>,
}

impl EvidenceSnapshotRefV1 {
    /// Shape checks only. A returned clean result does not authenticate source
    /// identity or prove that a digest matches actual remote/captured bytes.
    pub fn validation_blockers(&self) -> Vec<ReuseBlockerCode> {
        let mut blockers = Vec::new();
        if self.schema_version != SOURCE_SNAPSHOT_SCHEMA_V1 {
            blockers.push(ReuseBlockerCode::UnsupportedSnapshotSchema);
        }
        if self.provider_id.trim().is_empty() {
            blockers.push(ReuseBlockerCode::EmptyProviderId);
        }
        if self.source_record_id.trim().is_empty() {
            blockers.push(ReuseBlockerCode::EmptySourceRecordId);
        }
        if !is_http_url(&self.canonical_url) {
            blockers.push(ReuseBlockerCode::InvalidSourceUrl);
        }
        if self.source_version.as_deref().is_some_and(|v| v.trim().is_empty()) {
            blockers.push(ReuseBlockerCode::EmptySourceVersion);
        }
        if self
            .published_at_utc
            .as_deref()
            .is_some_and(|v| parse_utc(v).is_none())
        {
            blockers.push(ReuseBlockerCode::InvalidPublishedTimestamp);
        }
        if parse_utc(&self.retrieved_at_utc).is_none() {
            blockers.push(ReuseBlockerCode::InvalidRetrievedTimestamp);
        }

        match (&self.artifact_sha256, &self.digest_status) {
            (None, DigestVerificationStatus::NotCaptured) => {}
            (None, _) => blockers.push(ReuseBlockerCode::MissingArtifactDigest),
            (Some(hash), _) if !is_sha256(hash) => {
                blockers.push(ReuseBlockerCode::InvalidArtifactDigest);
            }
            (Some(_), DigestVerificationStatus::NotCaptured) => {
                blockers.push(ReuseBlockerCode::DigestStateMismatch);
            }
            (Some(_), DigestVerificationStatus::CapturedNotVerified) => {}
            (
                Some(_),
                DigestVerificationStatus::VerificationReceiptReferenced { receipt_ref },
            ) if receipt_ref.trim().is_empty() => {
                blockers.push(ReuseBlockerCode::MissingDigestVerificationReceipt);
            }
            (Some(_), DigestVerificationStatus::VerificationReceiptReferenced { .. }) => {}
        }
        blockers
    }

    pub fn has_digest_verification_receipt(&self) -> bool {
        matches!(
            &self.digest_status,
            DigestVerificationStatus::VerificationReceiptReferenced { receipt_ref }
                if !receipt_ref.trim().is_empty()
                    && self.artifact_sha256.as_deref().is_some_and(is_sha256)
        )
    }
}

impl RightsAndLicenseAssessmentV1 {
    /// Validate the assessment's structure and validity window. Permission
    /// evaluation remains operation-specific and fail-closed.
    pub fn validation_blockers(&self, now_utc: &str) -> Vec<ReuseBlockerCode> {
        let mut blockers = Vec::new();
        if self.schema_version != RIGHTS_ASSESSMENT_SCHEMA_V1 {
            blockers.push(ReuseBlockerCode::UnsupportedRightsSchema);
        }
        if self.assessment_id.trim().is_empty() {
            blockers.push(ReuseBlockerCode::EmptyAssessmentId);
        }
        if self.subject_provider_id.trim().is_empty() {
            blockers.push(ReuseBlockerCode::EmptyRightsProviderScope);
        }
        if self.subject_record_id.trim().is_empty() {
            blockers.push(ReuseBlockerCode::EmptyRightsRecordScope);
        }
        if self.subject_version.as_deref().is_some_and(|v| v.trim().is_empty()) {
            blockers.push(ReuseBlockerCode::EmptySourceVersion);
        }
        if self
            .subject_artifact_sha256
            .as_deref()
            .is_some_and(|hash| !is_sha256(hash))
        {
            blockers.push(ReuseBlockerCode::InvalidArtifactDigest);
        }
        if parse_utc(now_utc).is_none() {
            blockers.push(ReuseBlockerCode::InvalidEvaluationTimestamp);
        }

        if self.review_status != LicenseReviewStatus::Verified {
            blockers.push(if self.review_status == LicenseReviewStatus::Expired {
                ReuseBlockerCode::AssessmentExpired
            } else {
                ReuseBlockerCode::LicenseNotVerified
            });
        }

        if self.review_status == LicenseReviewStatus::Verified {
            if self
                .license_identifier
                .as_deref()
                .map(str::trim)
                .unwrap_or_default()
                .is_empty()
            {
                blockers.push(ReuseBlockerCode::MissingLicenseIdentifier);
            }
            if self
                .reviewer_id
                .as_deref()
                .map(str::trim)
                .unwrap_or_default()
                .is_empty()
            {
                blockers.push(ReuseBlockerCode::MissingReviewerId);
            }
            if self
                .review_evidence_ref
                .as_deref()
                .map(str::trim)
                .unwrap_or_default()
                .is_empty()
            {
                blockers.push(ReuseBlockerCode::MissingReviewEvidence);
            }

            let reviewed = self
                .reviewed_at_utc
                .as_deref()
                .and_then(parse_utc);
            let valid_until = self
                .valid_until_utc
                .as_deref()
                .and_then(parse_utc);
            let now = parse_utc(now_utc);

            if reviewed.is_none() {
                blockers.push(ReuseBlockerCode::InvalidReviewTimestamp);
            } else if now
                .as_ref()
                .zip(reviewed.as_ref())
                .is_some_and(|(now, reviewed)| reviewed > now)
            {
                blockers.push(ReuseBlockerCode::ReviewInFuture);
            }

            match (reviewed, valid_until, now) {
                (_, None, _) => blockers.push(ReuseBlockerCode::MissingValidityEnd),
                (Some(reviewed), Some(valid_until), _) if valid_until <= reviewed => {
                    blockers.push(ReuseBlockerCode::InvalidValidityWindow);
                }
                (_, Some(valid_until), Some(now)) if valid_until <= now => {
                    blockers.push(ReuseBlockerCode::AssessmentExpired);
                }
                _ => {}
            }
        }

        let mut seen = BTreeSet::new();
        if self
            .permissions
            .iter()
            .any(|entry| !seen.insert(entry.operation))
        {
            blockers.push(ReuseBlockerCode::DuplicateOperationPermission);
        }
        blockers
    }
}

/// Evaluate exactly one requested reuse operation. Unknown terms only permit a
/// pointer-level reference here; they never permit copying, redistribution,
/// translation, commercial reuse or model-training ingestion.
pub fn evaluate_reuse(
    snapshot: &EvidenceSnapshotRefV1,
    assessment: &RightsAndLicenseAssessmentV1,
    operation: ReuseOperation,
    now_utc: &str,
) -> ReuseDecisionV1 {
    let mut blockers = snapshot.validation_blockers();
    let mut caveats = Vec::new();

    if operation == ReuseOperation::ReferenceSource {
        if blockers.is_empty() {
            caveats.push(ReuseCaveatCode::ReferenceOnlyDoesNotGrantContentReuse);
        }
        return ReuseDecisionV1 {
            operation,
            allowed_by_supplied_assessment: blockers.is_empty(),
            blockers,
            caveats,
        };
    }

    blockers.extend(assessment.validation_blockers(now_utc));
    if assessment.subject_provider_id != snapshot.provider_id {
        blockers.push(ReuseBlockerCode::RightsProviderMismatch);
    }
    if assessment.subject_record_id != snapshot.source_record_id {
        blockers.push(ReuseBlockerCode::RightsRecordMismatch);
    }
    if assessment.subject_version != snapshot.source_version {
        blockers.push(ReuseBlockerCode::RightsVersionMismatch);
    }
    if assessment
        .subject_artifact_sha256
        .as_deref()
        .is_some_and(|hash| Some(hash) != snapshot.artifact_sha256.as_deref())
    {
        blockers.push(ReuseBlockerCode::RightsArtifactMismatch);
    }
    if let (Some(now), Some(retrieved)) =
        (parse_utc(now_utc), parse_utc(&snapshot.retrieved_at_utc))
    {
        if retrieved > now {
            blockers.push(ReuseBlockerCode::SourceRetrievedInFuture);
        }
    }

    if assessment.review_status == LicenseReviewStatus::Verified {
        let permission = assessment
            .permissions
            .iter()
            .find(|entry| entry.operation == operation);

        match permission {
            None => blockers.push(ReuseBlockerCode::PermissionNotGranted),
            Some(entry) => match entry.disposition {
                PermissionDisposition::Granted => {}
                PermissionDisposition::Denied => {
                    blockers.push(ReuseBlockerCode::PermissionDenied)
                }
                PermissionDisposition::NotAssessed => {
                    blockers.push(ReuseBlockerCode::PermissionNotAssessed)
                }
            },
        }

        if assessment.attribution_required
            && assessment
                .attribution_text
                .as_deref()
                .map(str::trim)
                .unwrap_or_default()
                .is_empty()
        {
            blockers.push(ReuseBlockerCode::AttributionTextMissing);
        }
    }

    if operation_requires_artifact_digest(operation)
        && !snapshot.has_digest_verification_receipt()
    {
        blockers.push(ReuseBlockerCode::DigestVerificationReceiptRequired);
    }

    blockers.sort_by_key(|code| *code as u8);
    blockers.dedup();

    ReuseDecisionV1 {
        operation,
        allowed_by_supplied_assessment: blockers.is_empty(),
        blockers,
        caveats,
    }
}

fn operation_requires_artifact_digest(operation: ReuseOperation) -> bool {
    matches!(
        operation,
        ReuseOperation::StoreFullArtifact
            | ReuseOperation::RedistributeArtifact
            | ReuseOperation::CreateTranslation
            | ReuseOperation::CommercialUse
            | ReuseOperation::TrainModel
    )
}

fn is_http_url(value: &str) -> bool {
    let remainder = value
        .strip_prefix("https://")
        .or_else(|| value.strip_prefix("http://"));
    let Some(remainder) = remainder else {
        return false;
    };
    let host = remainder.split(['/', '?', '#']).next().unwrap_or_default();
    !host.trim().is_empty()
        && !host.contains(char::is_whitespace)
        && !host.contains('@')
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn parse_utc(value: &str) -> Option<DateTime<Utc>> {
    if !value.ends_with('Z') {
        return None;
    }
    DateTime::parse_from_rfc3339(value)
        .ok()
        .map(|parsed| parsed.with_timezone(&Utc))
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: &str = "2026-10-14T18:45:00Z";

    fn snapshot(digest_status: DigestVerificationStatus) -> EvidenceSnapshotRefV1 {
        let has_artifact = !matches!(digest_status, DigestVerificationStatus::NotCaptured);
        EvidenceSnapshotRefV1 {
            schema_version: SOURCE_SNAPSHOT_SCHEMA_V1,
            provider_id: "synthetic-provider".to_owned(),
            source_record_id: "SYNTHETIC-RECORD-001".to_owned(),
            canonical_url: "https://example.invalid/source/1".to_owned(),
            source_version: Some("synthetic-v1".to_owned()),
            published_at_utc: Some("2026-10-01T00:00:00Z".to_owned()),
            retrieved_at_utc: "2026-10-02T00:00:00Z".to_owned(),
            artifact_sha256: has_artifact.then(|| "a".repeat(64)),
            digest_status,
        }
    }

    fn assessment(
        status: LicenseReviewStatus,
        permissions: Vec<OperationPermission>,
    ) -> RightsAndLicenseAssessmentV1 {
        RightsAndLicenseAssessmentV1 {
            schema_version: RIGHTS_ASSESSMENT_SCHEMA_V1,
            assessment_id: "SYNTHETIC-RIGHTS-001".to_owned(),
            subject_provider_id: "synthetic-provider".to_owned(),
            subject_record_id: "SYNTHETIC-RECORD-001".to_owned(),
            subject_version: Some("synthetic-v1".to_owned()),
            subject_artifact_sha256: None,
            review_status: status,
            license_identifier: Some("SYNTHETIC-LICENSE-ID".to_owned()),
            reviewer_id: Some("synthetic-reviewer".to_owned()),
            review_evidence_ref: Some("synthetic-review-record".to_owned()),
            reviewed_at_utc: Some("2026-10-10T00:00:00Z".to_owned()),
            valid_until_utc: Some("2027-10-10T00:00:00Z".to_owned()),
            attribution_required: false,
            attribution_text: None,
            permissions,
        }
    }

    fn grant(operation: ReuseOperation) -> OperationPermission {
        OperationPermission {
            operation,
            disposition: PermissionDisposition::Granted,
        }
    }

    fn includes(decision: &ReuseDecisionV1, blocker: ReuseBlockerCode) -> bool {
        decision.blockers.contains(&blocker)
    }

    #[test]
    fn unknown_rights_allow_only_pointer_reference() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let rights = assessment(LicenseReviewStatus::Unknown, Vec::new());
        let reference = evaluate_reuse(&source, &rights, ReuseOperation::ReferenceSource, NOW);
        assert!(reference.allowed_by_supplied_assessment);
        assert_eq!(
            reference.caveats,
            vec![ReuseCaveatCode::ReferenceOnlyDoesNotGrantContentReuse]
        );

        let metadata = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!metadata.allowed_by_supplied_assessment);
        assert!(includes(&metadata, ReuseBlockerCode::LicenseNotVerified));
    }

    #[test]
    fn absent_permission_is_not_inferred_from_a_license_label() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let rights = assessment(LicenseReviewStatus::Verified, Vec::new());
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::PermissionNotGranted));
    }

    #[test]
    fn explicit_minimal_metadata_grant_does_not_require_artifact_bytes() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(decision.allowed_by_supplied_assessment, "{decision:?}");
    }

    #[test]
    fn redistribution_requires_independent_digest_receipt_and_explicit_permission() {
        let source = snapshot(DigestVerificationStatus::CapturedNotVerified);
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::RedistributeArtifact)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::RedistributeArtifact,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(
            &decision,
            ReuseBlockerCode::DigestVerificationReceiptRequired
        ));

        let source = snapshot(DigestVerificationStatus::VerificationReceiptReferenced {
            receipt_ref: "SYNTHETIC-DIGEST-RECEIPT".to_owned(),
        });
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::RedistributeArtifact,
            NOW,
        );
        assert!(decision.allowed_by_supplied_assessment, "{decision:?}");
    }

    #[test]
    fn empty_digest_receipt_never_counts_as_verified() {
        let source = snapshot(DigestVerificationStatus::VerificationReceiptReferenced {
            receipt_ref: " ".to_owned(),
        });
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::RedistributeArtifact)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::RedistributeArtifact,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(
            &decision,
            ReuseBlockerCode::MissingDigestVerificationReceipt
        ));
        assert!(includes(
            &decision,
            ReuseBlockerCode::DigestVerificationReceiptRequired
        ));
    }

    #[test]
    fn redistribution_does_not_imply_model_training_permission() {
        let source = snapshot(DigestVerificationStatus::VerificationReceiptReferenced {
            receipt_ref: "SYNTHETIC-DIGEST-RECEIPT".to_owned(),
        });
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::RedistributeArtifact)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::TrainModel,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::PermissionNotGranted));
    }

    #[test]
    fn attribution_required_needs_actual_attribution_text() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let mut rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        rights.attribution_required = true;
        rights.attribution_text = Some(" ".to_owned());
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::AttributionTextMissing));

        rights.attribution_text = Some("Synthetic required attribution".to_owned());
        assert!(evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW
        ).allowed_by_supplied_assessment);
    }

    #[test]
    fn expired_assessment_blocks_even_with_granted_operation() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let mut rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        rights.valid_until_utc = Some("2026-10-14T18:44:59Z".to_owned());
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::AssessmentExpired));
    }

    #[test]
    fn future_review_blocks_use() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let mut rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        rights.reviewed_at_utc = Some("2026-10-15T00:00:00Z".to_owned());
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::ReviewInFuture));
    }

    #[test]
    fn duplicate_operation_permissions_block_reuse() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![
                grant(ReuseOperation::StoreMinimalMetadata),
                OperationPermission {
                    operation: ReuseOperation::StoreMinimalMetadata,
                    disposition: PermissionDisposition::Denied,
                },
            ],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::DuplicateOperationPermission));
    }

    #[test]
    fn invalid_hash_blocks_artifact_use() {
        let mut source = snapshot(DigestVerificationStatus::CapturedNotVerified);
        source.artifact_sha256 = Some("bad-hash".to_owned());
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreFullArtifact)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreFullArtifact,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::InvalidArtifactDigest));
    }

    #[test]
    fn invalid_source_url_blocks_even_reference() {
        let mut source = snapshot(DigestVerificationStatus::NotCaptured);
        source.canonical_url = "file:///private/source.pdf".to_owned();
        let rights = assessment(LicenseReviewStatus::Unknown, Vec::new());
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::ReferenceSource,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::InvalidSourceUrl));
    }

    #[test]
    fn malformed_snapshot_timestamps_fail_closed() {
        let mut source = snapshot(DigestVerificationStatus::NotCaptured);
        source.retrieved_at_utc = "yesterday".to_owned();
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(&decision, ReuseBlockerCode::InvalidRetrievedTimestamp));
    }

    #[test]
    fn unknown_license_does_not_block_unrelated_synthetic_research_record() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let rights = assessment(LicenseReviewStatus::Unknown, Vec::new());
        assert!(evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::ReferenceSource,
            NOW
        ).allowed_by_supplied_assessment);
        assert!(!evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreFullArtifact,
            NOW
        ).allowed_by_supplied_assessment);
    }

    #[test]
    fn rights_assessment_must_match_exact_source_record_and_version() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let mut rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        rights.subject_record_id = "SYNTHETIC-OTHER-RECORD".to_owned();
        let mismatched_record = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(includes(
            &mismatched_record,
            ReuseBlockerCode::RightsRecordMismatch
        ));

        rights.subject_record_id = "SYNTHETIC-RECORD-001".to_owned();
        rights.subject_version = Some("old-version".to_owned());
        let mismatched_version = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(includes(
            &mismatched_version,
            ReuseBlockerCode::RightsVersionMismatch
        ));
    }

    #[test]
    fn invalid_evaluation_timestamp_blocks_reuse() {
        let source = snapshot(DigestVerificationStatus::NotCaptured);
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            "not-a-time",
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(
            &decision,
            ReuseBlockerCode::InvalidEvaluationTimestamp
        ));
    }

    #[test]
    fn future_source_capture_blocks_reuse() {
        let mut source = snapshot(DigestVerificationStatus::NotCaptured);
        source.retrieved_at_utc = "2026-10-15T00:00:00Z".to_owned();
        let rights = assessment(
            LicenseReviewStatus::Verified,
            vec![grant(ReuseOperation::StoreMinimalMetadata)],
        );
        let decision = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert!(!decision.allowed_by_supplied_assessment);
        assert!(includes(
            &decision,
            ReuseBlockerCode::SourceRetrievedInFuture
        ));
    }

    #[test]
    fn blockers_are_deterministic() {
        let mut source = snapshot(DigestVerificationStatus::NotCaptured);
        source.schema_version = 77;
        source.provider_id.clear();
        let rights = assessment(LicenseReviewStatus::Unknown, Vec::new());
        let one = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        let two = evaluate_reuse(
            &source,
            &rights,
            ReuseOperation::StoreMinimalMetadata,
            NOW,
        );
        assert_eq!(one, two);
    }
}
